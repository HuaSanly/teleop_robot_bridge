#include "video/video_converter.hpp"
#include "video/nvbuf_mutex.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>

#include <ros/ros.h>

#include <linux/videodev2.h>

#include "nvbufsurface.h"
#include "NvBuffer.h"
#include "nvbufsurftransform.h"

namespace trb::video
{

namespace
{

const NvBufSurfTransformParams& defaultTransformParams()
{
    static const NvBufSurfTransformParams params = [] {
        NvBufSurfTransformParams value;
        std::memset(&value, 0, sizeof(value));
        value.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
        value.src_rect = nullptr;
        value.dst_rect = nullptr;
        value.transform_filter = NvBufSurfTransformInter_Nearest;
        return value;
    }();
    return params;
}

void logTransformFailureOnce(int ret, int src_fd, uint32_t width, uint32_t height)
{
    static std::atomic<uint64_t> failure_count{0};
    const uint64_t count = failure_count.fetch_add(1) + 1;
    if (count <= 5 || (count % 300 == 0))
    {
        std::cerr << "VideoConverter: NvBufSurfTransform(decoded-yuv->NV12) failed ret=" << ret
                  << " src_fd=" << src_fd << " w=" << width << " h=" << height
                  << " failures=" << count << std::endl;
    }
}

} // namespace

VideoConverter::VideoConverter() = default;

VideoConverter::~VideoConverter()
{
    destroyBuffers();
}

bool VideoConverter::initialize(const Config &config)
{
    config_ = config;

    const uint32_t input_w = (config_.input_width > 0) ? config_.input_width : config_.width;
    const uint32_t input_h = (config_.input_height > 0) ? config_.input_height : config_.height;

    {
        NvBufSurfTransformConfigParams transform_config;
        memset(&transform_config, 0, sizeof(transform_config));
        if (config_.transform_compute_mode == 1)
        {
            transform_config.compute_mode = NvBufSurfTransformCompute_GPU;
        }
        else if (config_.transform_compute_mode == 2)
        {
            transform_config.compute_mode = NvBufSurfTransformCompute_VIC;
        }
        else
        {
            transform_config.compute_mode = NvBufSurfTransformCompute_Default;
        }
        transform_config.gpu_id = 0;
        transform_config.cuda_stream = 0;
        const int sret = NvBufSurfTransformSetSessionParams(&transform_config);
        if (sret != 0)
        {
            std::cerr << "VideoConverter: NvBufSurfTransformSetSessionParams failed ret=" << sret
                      << " (compute_mode="
                      << ((config_.transform_compute_mode == 1) ? "GPU" : (config_.transform_compute_mode == 2) ? "VIC" : "DEFAULT")
                      << ")" << std::endl;
        }

    }

    NvBufSurfaceCreateParams out_params;
    memset(&out_params, 0, sizeof(out_params));
    out_params.gpuId = 0;
    out_params.width = config_.width;
    out_params.height = config_.height;
    out_params.layout = (config_.output_surface_layout == 1) ? NVBUF_LAYOUT_BLOCK_LINEAR : NVBUF_LAYOUT_PITCH;
    out_params.colorFormat = NVBUF_COLOR_FORMAT_NV12;
    out_params.memType = NVBUF_MEM_SURFACE_ARRAY;
    out_params.isContiguous = true;

    surfaces_.resize(config_.buffer_pool_size, nullptr);
    dmabuf_fds_.assign(config_.buffer_pool_size, -1);

    {
        std::lock_guard<std::mutex> lk(pool_mutex_);
        while (!free_indices_.empty())
            free_indices_.pop();
        fd_to_index_.clear();
    }

    for (size_t i = 0; i < config_.buffer_pool_size; ++i)
    {
        NvBufSurface *surf = nullptr;
        if (NvBufSurfaceCreate(&surf, 1, &out_params) != 0)
        {
            std::cerr << "Failed to create NvBufSurface " << i << std::endl;
            return false;
        }
        surfaces_[i] = surf;
        dmabuf_fds_[i] = surf->surfaceList[0].bufferDesc;

        std::lock_guard<std::mutex> lk(pool_mutex_);
        free_indices_.push(i);
        fd_to_index_.emplace(dmabuf_fds_[i], i);
    }

    logged_mode_.store(true, std::memory_order_relaxed);

    return true;
}

void VideoConverter::destroyBuffers()
{
    {
        std::lock_guard<std::mutex> lk(pool_mutex_);
        while (!free_indices_.empty())
            free_indices_.pop();
        fd_to_index_.clear();
    }

    for (void *surf : surfaces_)
    {
        if (surf)
        {
            NvBufSurfaceDestroy(reinterpret_cast<NvBufSurface *>(surf));
        }
    }

    surfaces_.clear();
    dmabuf_fds_.clear();
}

void VideoConverter::releaseFd(int dmabuf_fd)
{
    std::lock_guard<std::mutex> lk(pool_mutex_);
    auto it = fd_to_index_.find(dmabuf_fd);
    if (it != fd_to_index_.end())
    {
        free_indices_.push(it->second);
    }
}

VideoConverter::StatsSnapshot VideoConverter::consumeStats()
{
    StatsSnapshot snapshot;
    snapshot.processed_frames = stats_processed_frames_.exchange(0);
    snapshot.malformed_frames = stats_malformed_frames_.exchange(0);
    snapshot.pool_drops = stats_pool_drops_.exchange(0);
    snapshot.failed_frames = stats_failed_frames_.exchange(0);
    snapshot.decode_us_total = stats_decode_us_total_.exchange(0);
    snapshot.transform_us_total = stats_transform_us_total_.exchange(0);
    return snapshot;
}

bool VideoConverter::transformSync(int yuv_dmabuf_fd,
                                   uint64_t timestamp_us,
                                   uint64_t decode_us,
                                   int &nv12_fd_out)
{
    (void)timestamp_us;
    nv12_fd_out = -1;

    if (yuv_dmabuf_fd < 0 || dmabuf_fds_.empty())
    {
        return false;
    }

    if (decode_us > 0)
    {
        stats_decode_us_total_.fetch_add(static_cast<int64_t>(decode_us));
    }

    size_t out_idx = 0;
    {
        std::lock_guard<std::mutex> lk(pool_mutex_);
        if (free_indices_.empty())
        {
            const uint64_t d = stats_pool_drops_.fetch_add(1) + 1;
            if (d <= 5 || (d % 300 == 0))
            {
                std::cerr << "VideoConverter: buffer pool empty, dropping frame (drops=" << d
                          << ", pool_size=" << dmabuf_fds_.size() << ")" << std::endl;
            }
            return false;
        }
        out_idx = free_indices_.front();
        free_indices_.pop();
    }

    const int out_fd = dmabuf_fds_[out_idx];

    NvBufSurface *src_surf = nullptr;
    {
        std::lock_guard<std::mutex> lk(getNvBufMutex());
        if (NvBufSurfaceFromFd(yuv_dmabuf_fd, reinterpret_cast<void **>(&src_surf)) != 0 || !src_surf)
        {
            std::cerr << "VideoConverter: NvBufSurfaceFromFd failed for decoded fd=" << yuv_dmabuf_fd << std::endl;
            stats_failed_frames_.fetch_add(1);
            std::lock_guard<std::mutex> pool_lk(pool_mutex_);
            free_indices_.push(out_idx);
            return false;
        }
    }

    const auto &src_sp = src_surf->surfaceList[0];
    const uint32_t width  = src_sp.width;
    const uint32_t height = src_sp.height;

    static std::atomic<bool> warned_dimension_mismatch{false};
    const uint32_t requested_width  = (config_.input_width  > 0) ? config_.input_width  : config_.width;
    const uint32_t requested_height = (config_.input_height > 0) ? config_.input_height : config_.height;
    if ((width != requested_width || height != requested_height) &&
        !warned_dimension_mismatch.exchange(true))
    {
        ROS_WARN(
            "VideoConverter: decoded surface dimensions differ from requested input: requested=%ux%u actual=%ux%u",
            requested_width, requested_height, width, height);
    }

    NvBufSurface *dst = reinterpret_cast<NvBufSurface *>(surfaces_[out_idx]);
    if (!dst)
    {
        stats_failed_frames_.fetch_add(1);
        std::lock_guard<std::mutex> lk(pool_mutex_);
        free_indices_.push(out_idx);
        return false;
    }

    src_surf->numFilled = 1;
    dst->numFilled = 1;

    const auto t0 = std::chrono::steady_clock::now();
    int tret;
    {
        std::lock_guard<std::mutex> lk(getNvBufMutex());
        NvBufSurfTransformParams tparams = defaultTransformParams();
        tret = NvBufSurfTransform(src_surf, dst, &tparams);
    }
    const auto t1 = std::chrono::steady_clock::now();
    stats_transform_us_total_.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

    if (tret != 0)
    {
        logTransformFailureOnce(tret, yuv_dmabuf_fd, width, height);
        stats_failed_frames_.fetch_add(1);
        std::lock_guard<std::mutex> lk(pool_mutex_);
        free_indices_.push(out_idx);
        return false;
    }

    stats_processed_frames_.fetch_add(1);
    nv12_fd_out = out_fd;
    return true;
}

} // namespace trb::video
