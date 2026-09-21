#include "video/video_scaler.hpp"

#include <iostream>

#include "NvBufSurface.h"
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"

#include "video/nvbuf_mutex.hpp"

namespace trb::video
{

VideoScaler::VideoScaler() = default;

VideoScaler::~VideoScaler()
{
    destroyBuffers();
}

bool VideoScaler::initialize(const Config &config)
{
    destroyBuffers();
    if (config.input_width == 0 || config.input_height == 0 ||
        config.output_width == 0 || config.output_height == 0 ||
        (config.input_width & 1u) != 0 || (config.input_height & 1u) != 0 ||
        (config.output_width & 1u) != 0 || (config.output_height & 1u) != 0 ||
        config.buffer_pool_size == 0)
    {
        std::cerr << "VideoScaler: invalid dimensions or pool size" << std::endl;
        return false;
    }

    config_ = config;
    NvBufSurfaceCreateParams params{};
    params.gpuId = 0;
    params.width = config_.output_width;
    params.height = config_.output_height;
    params.layout = NVBUF_LAYOUT_PITCH;
    params.colorFormat = NVBUF_COLOR_FORMAT_NV12;
    params.memType = NVBUF_MEM_SURFACE_ARRAY;
    params.isContiguous = true;

    surfaces_.assign(config_.buffer_pool_size, nullptr);
    dmabuf_fds_.assign(config_.buffer_pool_size, -1);
    for (size_t i = 0; i < surfaces_.size(); ++i)
    {
        NvBufSurface *surface = nullptr;
        if (NvBufSurfaceCreate(&surface, 1, &params) != 0 || !surface)
        {
            std::cerr << "VideoScaler: failed to create output surface " << i << std::endl;
            destroyBuffers();
            return false;
        }
        surface->numFilled = 1;
        surfaces_[i] = surface;
        dmabuf_fds_[i] = surface->surfaceList[0].bufferDesc;
        free_indices_.push(i);
        fd_to_index_.emplace(dmabuf_fds_[i], i);
    }

    return true;
}

void VideoScaler::destroyBuffers()
{
    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        while (!free_indices_.empty())
        {
            free_indices_.pop();
        }
        fd_to_index_.clear();
    }

    for (void *surface : surfaces_)
    {
        if (surface)
        {
            NvBufSurfaceDestroy(reinterpret_cast<NvBufSurface *>(surface));
        }
    }
    surfaces_.clear();
    dmabuf_fds_.clear();
}

bool VideoScaler::transformSync(int input_fd, int &output_fd_out)
{
    output_fd_out = -1;
    if (input_fd < 0 || dmabuf_fds_.empty())
    {
        return false;
    }

    size_t output_index = 0;
    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        if (free_indices_.empty())
        {
            return false;
        }
        output_index = free_indices_.front();
        free_indices_.pop();
    }

    NvBufSurface *input_surface = nullptr;
    NvBufSurface *output_surface = reinterpret_cast<NvBufSurface *>(surfaces_[output_index]);
    bool valid = false;
    {
        std::lock_guard<std::mutex> lock(getNvBufMutex());
        valid = output_surface &&
                NvBufSurfaceFromFd(input_fd, reinterpret_cast<void **>(&input_surface)) == 0 &&
                input_surface && input_surface->numFilled > 0;
        if (valid)
        {
            const auto &src = input_surface->surfaceList[0];
            valid = src.layout == NVBUF_LAYOUT_PITCH &&
                    src.colorFormat == NVBUF_COLOR_FORMAT_NV12 &&
                    src.planeParams.num_planes >= 2 &&
                    src.width == config_.input_width &&
                    src.height == config_.input_height;
        }
    }

    if (!valid)
    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        free_indices_.push(output_index);
        return false;
    }

    NvBufSurfTransformRect src_rect{};
    src_rect.width = config_.input_width;
    src_rect.height = config_.input_height;
    NvBufSurfTransformRect dst_rect{};
    dst_rect.width = config_.output_width;
    dst_rect.height = config_.output_height;
    NvBufSurfTransformParams transform{};
    transform.transform_flag = static_cast<NvBufSurfTransform_Transform_Flag>(
        NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_FLIP);
    transform.transform_flip = NvBufSurfTransform_None;
    transform.transform_filter = NvBufSurfTransformInter_Bilinear;
    transform.src_rect = &src_rect;
    transform.dst_rect = &dst_rect;

    int ret = -1;
    {
        std::lock_guard<std::mutex> lock(getNvBufMutex());
        ret = NvBufSurfTransform(input_surface, output_surface, &transform);
    }
    if (ret != 0)
    {
        std::cerr << "VideoScaler: NvTransform failed ret=" << ret
                  << " input_fd=" << input_fd
                  << " input=" << config_.input_width << "x" << config_.input_height
                  << " output=" << config_.output_width << "x" << config_.output_height
                  << std::endl;
        std::lock_guard<std::mutex> lock(pool_mutex_);
        free_indices_.push(output_index);
        return false;
    }

    output_fd_out = dmabuf_fds_[output_index];
    return true;
}

void VideoScaler::releaseFd(int dmabuf_fd)
{
    std::lock_guard<std::mutex> lock(pool_mutex_);
    const auto it = fd_to_index_.find(dmabuf_fd);
    if (it != fd_to_index_.end())
    {
        free_indices_.push(it->second);
    }
}

} // namespace trb::video
