#include "video/video_undistorter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <ros/ros.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include "nvbufsurface.h"

#ifdef TRB_CUDA_AVAILABLE
#include "video/cuda_video_pipeline.hpp"
#endif

#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/Types.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/Remap.h>

namespace trb::video
{

    namespace
    {

        bool checkVpi(VPIStatus s, const char *what)
        {
            if (s == VPI_SUCCESS)
            {
                return true;
            }
            char buf[VPI_MAX_STATUS_MESSAGE_LENGTH];
            vpiGetLastStatusMessage(buf, sizeof(buf));
            ROS_ERROR("[UNDISTORT] %s failed: %d (%s)", what, static_cast<int>(s), buf);
            return false;
        }

        // Read a flat yaml sequence into a fixed-size cv::Mat. Returns false if
        // the node is missing AND |required| is true.
        bool loadMatrixFromYaml(const YAML::Node &eye, const char *key,
                                int rows, int cols, cv::Mat &out, bool required)
        {
            if (!eye[key])
            {
                if (required)
                {
                    ROS_ERROR("[UNDISTORT] calibration missing required field '%s'", key);
                    return false;
                }
                return false;
            }
            const auto values = eye[key].as<std::vector<double>>();
            const int expected = rows * cols;
            if (static_cast<int>(values.size()) != expected)
            {
                ROS_ERROR("[UNDISTORT] '%s' expects %d floats, got %zu", key, expected, values.size());
                return false;
            }
            out.create(rows, cols, CV_64F);
            std::memcpy(out.ptr<double>(), values.data(), expected * sizeof(double));
            return true;
        }

        // Read distortion vector (5..14 coefficients depending on model).
        bool loadDistortion(const YAML::Node &eye, cv::Mat &out)
        {
            if (!eye["D"])
            {
                ROS_ERROR("[UNDISTORT] calibration missing 'D'");
                return false;
            }
            const auto values = eye["D"].as<std::vector<double>>();
            if (values.size() < 4)
            {
                ROS_ERROR("[UNDISTORT] 'D' too short: %zu (need >= 4)", values.size());
                return false;
            }
            out = cv::Mat(1, static_cast<int>(values.size()), CV_64F);
            std::memcpy(out.ptr<double>(), values.data(), values.size() * sizeof(double));
            return true;
        }

        // Build a single-eye mapping (R = optional, P = optional). Returns
        // CV_32FC2 dense map of size runtime_size. Falls back to identity P
        // via getOptimalNewCameraMatrix(K_run, D, runtime_size, alpha) when
        // P is not provided in the YAML.
        bool buildEyeMap(const YAML::Node &eye_node,
                         const cv::Size &calib_size,
                         const cv::Size &runtime_size,
                         double alpha,
                         cv::Mat &map_xy_out)
        {
            cv::Mat K_calib, D, R, P;
            if (!loadMatrixFromYaml(eye_node, "K", 3, 3, K_calib, true))
            {
                return false;
            }
            if (!loadDistortion(eye_node, D))
            {
                return false;
            }
            if (!loadMatrixFromYaml(eye_node, "R", 3, 3, R, false))
            {
                R = cv::Mat::eye(3, 3, CV_64F);
            }

            // Scale K from calibration resolution to runtime resolution.
            // K is in pixel units, so K' = diag(sx, sy, 1) * K.
            const double sx = static_cast<double>(runtime_size.width) /
                              static_cast<double>(calib_size.width);
            const double sy = static_cast<double>(runtime_size.height) /
                              static_cast<double>(calib_size.height);
            cv::Mat K_run = K_calib.clone();
            K_run.at<double>(0, 0) *= sx; // fx
            K_run.at<double>(0, 2) *= sx; // cx
            K_run.at<double>(1, 1) *= sy; // fy
            K_run.at<double>(1, 2) *= sy; // cy

            const bool have_P = loadMatrixFromYaml(eye_node, "P", 3, 4, P, false);
            cv::Mat new_K;
            if (have_P)
            {
                new_K = P; // P is already a 3x4 projection in rectified frame.
            }
            else
            {
                new_K = cv::getOptimalNewCameraMatrix(
                    K_run, D, runtime_size, alpha, runtime_size, nullptr, false);
            }

            cv::Mat map1, map2;
            cv::initUndistortRectifyMap(K_run, D, R, new_K, runtime_size,
                                        CV_32FC2, map1, map2);
            map_xy_out = map1; // CV_32FC2: each pixel = (src_x, src_y)
            return true;
        }

    } // namespace

    struct VideoUndistorter::Impl
    {
        Config cfg;

        // Output NV12 NvBufSurface pool (same layout as VideoConverter output).
        std::vector<void *> surfaces;
        std::vector<int> dmabuf_fds;
        std::mutex pool_mutex;
        std::queue<size_t> free_indices;
        std::unordered_map<int, size_t> fd_to_index;

        // VPI resources.
        VPIStream stream{nullptr};
        VPIPayload remap_payload{nullptr};
        VPIWarpMap warp_map{};
        bool warp_allocated{false};

#ifdef TRB_CUDA_AVAILABLE
        std::unique_ptr<CudaStereoRemapper> cuda_remapper;
        std::vector<float> cuda_map_xy;
#endif

        // VPI image wrappers per dmabuf fd. Wrapping is cheap but not free,
        // so we cache one VPIImage per pool entry plus one for the input.
        // The input fd may change frame-to-frame (it comes from the converter
        // pool), so we maintain a small LRU-ish cache keyed by fd.
        std::unordered_map<int, VPIImage> input_image_cache;
        std::unordered_map<int, VPIImage> output_image_cache;

        // Stats.
        std::atomic<uint64_t> processed_frames{0};
        std::atomic<uint64_t> pool_drops{0};
        std::atomic<uint64_t> failed_frames{0};
        std::atomic<int64_t> remap_us_total{0};

        ~Impl() { teardown(); }

        void teardown()
        {
#ifdef TRB_CUDA_AVAILABLE
            // The CUDA remapper owns EGL registrations for converter and
            // output-pool fds. Destroy it before the NvBufSurface pool.
            cuda_remapper.reset();
#endif
            for (auto &kv : input_image_cache)
            {
                if (kv.second)
                {
                    vpiImageDestroy(kv.second);
                }
            }
            input_image_cache.clear();

            for (auto &kv : output_image_cache)
            {
                if (kv.second)
                {
                    vpiImageDestroy(kv.second);
                }
            }
            output_image_cache.clear();

            if (remap_payload)
            {
                vpiPayloadDestroy(remap_payload);
                remap_payload = nullptr;
            }
            if (stream)
            {
                vpiStreamDestroy(stream);
                stream = nullptr;
            }
            if (warp_allocated)
            {
                vpiWarpMapFreeData(&warp_map);
                warp_allocated = false;
            }

            {
                std::lock_guard<std::mutex> lk(pool_mutex);
                while (!free_indices.empty())
                {
                    free_indices.pop();
                }
                fd_to_index.clear();
            }
            for (void *s : surfaces)
            {
                if (s)
                {
                    NvBufSurfaceDestroy(reinterpret_cast<NvBufSurface *>(s));
                }
            }
            surfaces.clear();
            dmabuf_fds.clear();
        }

        bool createOutputPool()
        {
            NvBufSurfaceCreateParams p;
            std::memset(&p, 0, sizeof(p));
            p.gpuId = 0;
            p.width = cfg.width;
            p.height = cfg.height;
#ifdef TRB_CUDA_AVAILABLE
            // CUDA EGL interop consumes pitch-linear planes. Keep the
            // explicit legacy/VPI backend's configured layout untouched.
            p.layout = cfg.use_cuda ? NVBUF_LAYOUT_PITCH
                                    : ((cfg.output_surface_layout == 1)
                                           ? NVBUF_LAYOUT_BLOCK_LINEAR
                                           : NVBUF_LAYOUT_PITCH);
#else
            p.layout = (cfg.output_surface_layout == 1) ? NVBUF_LAYOUT_BLOCK_LINEAR
                                                       : NVBUF_LAYOUT_PITCH;
#endif
            p.colorFormat = NVBUF_COLOR_FORMAT_NV12;
            p.memType = NVBUF_MEM_SURFACE_ARRAY;
            p.isContiguous = true;

            surfaces.assign(cfg.buffer_pool_size, nullptr);
            dmabuf_fds.assign(cfg.buffer_pool_size, -1);
            for (uint32_t i = 0; i < cfg.buffer_pool_size; ++i)
            {
                NvBufSurface *surf = nullptr;
                if (NvBufSurfaceCreate(&surf, 1, &p) != 0)
                {
                    ROS_ERROR("[UNDISTORT] NvBufSurfaceCreate failed at i=%u", i);
                    return false;
                }
                surfaces[i] = surf;
                surf->numFilled = 1;
                dmabuf_fds[i] = surf->surfaceList[0].bufferDesc;
                free_indices.push(i);
                fd_to_index.emplace(dmabuf_fds[i], i);
            }
            return true;
        }

        // Load the calibration YAML and assemble the SBS warp map. The map
        // sits dense (interval=1) in a single VPI region matching the full
        // SBS frame size. Right-eye dst x is offset by +single_w; per-eye
        // src coordinates are clamped to their own half so bilinear sampling
        // never crosses the seam.
        bool buildWarpMap(const std::string &yaml_path)
        {
            YAML::Node root;
            try
            {
                root = YAML::LoadFile(yaml_path);
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("[UNDISTORT] failed to load '%s': %s", yaml_path.c_str(), e.what());
                return false;
            }

            // Figure out per-eye runtime size. The SBS frame is cfg.width x
            // cfg.height; each eye occupies the left/right half.
            const cv::Size runtime_eye(static_cast<int>(cfg.width / 2u),
                                       static_cast<int>(cfg.height));

            cv::Size calib_size = runtime_eye;
            if (root["calibration_image_size"])
            {
                const auto v = root["calibration_image_size"].as<std::vector<int>>();
                if (v.size() == 2)
                {
                    calib_size = cv::Size(v[0], v[1]);
                }
            }
            if (root["runtime_image_size"])
            {
                const auto v = root["runtime_image_size"].as<std::vector<int>>();
                if (v.size() == 2 &&
                    (v[0] != runtime_eye.width || v[1] != runtime_eye.height))
                {
                    ROS_WARN("[UNDISTORT] runtime_image_size=[%d,%d] in yaml differs from "
                             "pipeline single-eye size [%d,%d]; using pipeline size",
                             v[0], v[1], runtime_eye.width, runtime_eye.height);
                }
            }
            const double alpha = root["alpha"] ? root["alpha"].as<double>() : 0.0;

            if (!root["left"] || !root["right"])
            {
                ROS_ERROR("[UNDISTORT] yaml missing 'left' or 'right' section");
                return false;
            }

            cv::Mat map_left, map_right;
            if (!buildEyeMap(root["left"], calib_size, runtime_eye, alpha, map_left))
            {
                return false;
            }
            if (!buildEyeMap(root["right"], calib_size, runtime_eye, alpha, map_right))
            {
                return false;
            }

#ifdef TRB_CUDA_AVAILABLE
            // Keep the exact same clamped per-eye maps used by the legacy VPI
            // path, but flatten them as (x,y) float pairs for the CUDA kernel.
            cuda_map_xy.resize(static_cast<size_t>(cfg.width) * cfg.height * 2u);
            const int cuda_single_w = runtime_eye.width;
            const int cuda_single_h = runtime_eye.height;
            const float cuda_clamp_x_max = static_cast<float>(cuda_single_w - 1);
            const float cuda_clamp_y_max = static_cast<float>(cuda_single_h - 1);
            for (uint32_t y = 0; y < cfg.height; ++y)
            {
                const int map_y = std::min<int>(static_cast<int>(y), cuda_single_h - 1);
                for (uint32_t x = 0; x < cfg.width; ++x)
                {
                    const bool right = x >= static_cast<uint32_t>(cuda_single_w);
                    const int local_x = right
                        ? std::min<int>(static_cast<int>(x) - cuda_single_w, cuda_single_w - 1)
                        : static_cast<int>(x);
                    cv::Vec2f src = right
                        ? map_right.at<cv::Vec2f>(map_y, local_x)
                        : map_left.at<cv::Vec2f>(map_y, local_x);
                    src[0] = std::clamp(src[0], 0.0f, cuda_clamp_x_max);
                    src[1] = std::clamp(src[1], 0.0f, cuda_clamp_y_max);
                    if (right)
                        src[0] += static_cast<float>(cuda_single_w);
                    const size_t index = (static_cast<size_t>(y) * cfg.width + x) * 2u;
                    cuda_map_xy[index + 0] = src[0];
                    cuda_map_xy[index + 1] = src[1];
                }
            }
#endif

            // Configure a single dense region covering the full SBS frame.
            // Interval=1 power-of-two, region sizes meet 64-wide / 16-high
            // alignment requirements (5120, 1440 both qualify).
            std::memset(&warp_map, 0, sizeof(warp_map));
            warp_map.grid.numHorizRegions = 1;
            warp_map.grid.numVertRegions = 1;
            warp_map.grid.horizInterval[0] = 1;
            warp_map.grid.vertInterval[0] = 1;
            warp_map.grid.regionWidth[0] = static_cast<int16_t>(cfg.width);
            warp_map.grid.regionHeight[0] = static_cast<int16_t>(cfg.height);

            if (!checkVpi(vpiWarpMapAllocData(&warp_map), "vpiWarpMapAllocData"))
            {
                return false;
            }
            warp_allocated = true;

            // numHorizPoints = (width / 1) + 1, numVertPoints = (height / 1) + 1.
            const int nx = warp_map.numHorizPoints;
            const int ny = warp_map.numVertPoints;
            const int single_w = runtime_eye.width;
            const int single_h = runtime_eye.height;
            const float clamp_x_max = static_cast<float>(single_w - 1);
            const float clamp_y_max = static_cast<float>(single_h - 1);

            for (int v = 0; v < ny; ++v)
            {
                auto *row = reinterpret_cast<VPIKeypointF32 *>(
                    reinterpret_cast<uint8_t *>(warp_map.keypoints) +
                    static_cast<ptrdiff_t>(v) * warp_map.pitchBytes);
                // Use clamped row index for src lookup (last grid row may be
                // height, one past the last image row).
                const int v_src = std::min(v, single_h - 1);

                for (int h = 0; h < nx; ++h)
                {
                    cv::Vec2f src;
                    if (h < single_w)
                    {
                        // Left eye dst column.
                        src = map_left.at<cv::Vec2f>(v_src, h);
                        src[0] = std::clamp(src[0], 0.0f, clamp_x_max);
                        src[1] = std::clamp(src[1], 0.0f, clamp_y_max);
                        // No offset; left-eye src lives in [0, single_w).
                    }
                    else
                    {
                        // Right eye dst column. Look up in right map at h - single_w.
                        const int h_local = std::min(h - single_w, single_w - 1);
                        src = map_right.at<cv::Vec2f>(v_src, h_local);
                        src[0] = std::clamp(src[0], 0.0f, clamp_x_max);
                        src[1] = std::clamp(src[1], 0.0f, clamp_y_max);
                        // Offset src x into the right half of the SBS frame.
                        src[0] += static_cast<float>(single_w);
                    }
                    row[h].x = src[0];
                    row[h].y = src[1];
                }
            }
            return true;
        }

        bool createVpiResources()
        {
            if (!checkVpi(vpiStreamCreate(VPI_BACKEND_CUDA, &stream),
                          "vpiStreamCreate(CUDA)"))
            {
                return false;
            }
            if (!checkVpi(vpiCreateRemap(VPI_BACKEND_CUDA, &warp_map, &remap_payload),
                          "vpiCreateRemap"))
            {
                return false;
            }
            return true;
        }

        // Wrap or fetch a cached VPIImage for an NV12 NvBuffer fd. Returns
        // nullptr on failure.
        VPIImage getOrWrapImage(int fd, std::unordered_map<int, VPIImage> &cache)
        {
            auto it = cache.find(fd);
            if (it != cache.end())
            {
                return it->second;
            }
            VPIImageData data;
            std::memset(&data, 0, sizeof(data));
            data.bufferType = VPI_IMAGE_BUFFER_NVBUFFER;
            data.buffer.fd = fd;

            VPIImage img = nullptr;
            const VPIStatus s = vpiImageCreateWrapper(
                &data, nullptr, VPI_BACKEND_CUDA, &img);
            if (s != VPI_SUCCESS || !img)
            {
                checkVpi(s, "vpiImageCreateWrapper(NVBUFFER)");
                return nullptr;
            }
            cache.emplace(fd, img);
            return img;
        }
    };

    VideoUndistorter::VideoUndistorter() : impl_(new Impl()) {}
    VideoUndistorter::~VideoUndistorter() = default;

    bool VideoUndistorter::initialize(const Config &config)
    {
        impl_->cfg = config;

        if (config.calibration_file.empty())
        {
            ROS_ERROR("[UNDISTORT] empty calibration_file");
            return false;
        }
        if (!impl_->buildWarpMap(config.calibration_file))
        {
            if (config.require_calibration)
            {
                return false;
            }
            ROS_WARN("[UNDISTORT] calibration unavailable; falling back to identity map");
            // Fallback: identity-ish keypoints. Build a trivial map.
            std::memset(&impl_->warp_map, 0, sizeof(impl_->warp_map));
            impl_->warp_map.grid.numHorizRegions = 1;
            impl_->warp_map.grid.numVertRegions = 1;
            impl_->warp_map.grid.horizInterval[0] = 1;
            impl_->warp_map.grid.vertInterval[0] = 1;
            impl_->warp_map.grid.regionWidth[0] = static_cast<int16_t>(config.width);
            impl_->warp_map.grid.regionHeight[0] = static_cast<int16_t>(config.height);
            if (!checkVpi(vpiWarpMapAllocData(&impl_->warp_map),
                          "vpiWarpMapAllocData(identity)"))
            {
                return false;
            }
            impl_->warp_allocated = true;
            if (!checkVpi(vpiWarpMapGenerateIdentity(&impl_->warp_map),
                          "vpiWarpMapGenerateIdentity"))
            {
                return false;
            }
#ifdef TRB_CUDA_AVAILABLE
            impl_->cuda_map_xy.resize(static_cast<size_t>(config.width) * config.height * 2u);
            for (uint32_t y = 0; y < config.height; ++y)
            {
                for (uint32_t x = 0; x < config.width; ++x)
                {
                    const size_t index = (static_cast<size_t>(y) * config.width + x) * 2u;
                    impl_->cuda_map_xy[index + 0] = static_cast<float>(x);
                    impl_->cuda_map_xy[index + 1] = static_cast<float>(y);
                }
            }
#endif
        }

#ifdef TRB_CUDA_AVAILABLE
        if (config.use_cuda && !impl_->cuda_map_xy.empty())
        {
            auto cuda_remapper = std::make_unique<CudaStereoRemapper>();
            if (cuda_remapper->initialize(config.width, config.height,
                                          impl_->cuda_map_xy))
            {
                impl_->cuda_remapper = std::move(cuda_remapper);
                ROS_INFO("[UNDISTORT] CUDA remapper enabled");
            }
            else
            {
                ROS_WARN("[UNDISTORT] CUDA remapper initialization failed; using VPI fallback");
            }
        }
#endif
#ifdef TRB_CUDA_AVAILABLE
        if (!impl_->cuda_remapper)
#endif
        {
            if (!impl_->createVpiResources())
            {
                return false;
            }
        }
        if (!impl_->createOutputPool())
        {
            return false;
        }

        ROS_INFO("[UNDISTORT] initialized: %ux%u, pool=%u, calib=%s",
                 config.width, config.height, config.buffer_pool_size,
                 config.calibration_file.c_str());
        return true;
    }

    bool VideoUndistorter::process(int nv12_fd_in, int &nv12_fd_out)
    {
        nv12_fd_out = -1;
        if (nv12_fd_in < 0 ||
#ifdef TRB_CUDA_AVAILABLE
            (!impl_->cuda_remapper && (!impl_->stream || !impl_->remap_payload))
#else
            (!impl_->stream || !impl_->remap_payload)
#endif
        )
        {
            return false;
        }

        // Acquire an output fd from the pool.
        size_t out_idx = 0;
        int out_fd = -1;
        {
            std::lock_guard<std::mutex> lk(impl_->pool_mutex);
            if (impl_->free_indices.empty())
            {
                impl_->pool_drops.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            out_idx = impl_->free_indices.front();
            impl_->free_indices.pop();
            out_fd = impl_->dmabuf_fds[out_idx];
        }

        const auto t0 = std::chrono::steady_clock::now();

#ifdef TRB_CUDA_AVAILABLE
        if (impl_->cuda_remapper)
        {
            if (!impl_->cuda_remapper->process(nv12_fd_in, out_fd))
            {
                {
                    std::lock_guard<std::mutex> lk(impl_->pool_mutex);
                    impl_->free_indices.push(out_idx);
                }
                impl_->failed_frames.fetch_add(1, std::memory_order_relaxed);
                // A runtime CUDA/EGL failure should not permanently stop the
                // video path. Tear down the CUDA remapper, initialize the
                // existing VPI backend, and retry this frame once.
                impl_->cuda_remapper.reset();
                if (impl_->createVpiResources())
                {
                    ROS_WARN_THROTTLE(5.0,
                                      "[UNDISTORT] CUDA frame failed; switched to VPI legacy backend");
                    return process(nv12_fd_in, nv12_fd_out);
                }
                return false;
            }
            const auto t1 = std::chrono::steady_clock::now();
            impl_->remap_us_total.fetch_add(
                std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count(),
                std::memory_order_relaxed);
            impl_->processed_frames.fetch_add(1, std::memory_order_relaxed);
            nv12_fd_out = out_fd;
            return true;
        }
#endif

        VPIImage in_img = impl_->getOrWrapImage(nv12_fd_in, impl_->input_image_cache);
        VPIImage out_img = impl_->getOrWrapImage(out_fd, impl_->output_image_cache);
        if (!in_img || !out_img)
        {
            std::lock_guard<std::mutex> lk(impl_->pool_mutex);
            impl_->free_indices.push(out_idx);
            impl_->failed_frames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        // Submit one CUDA Remap and wait for completion. Synchronous submit
        // matches the encode thread's existing transformSync() pattern and
        // guarantees the encoder sees a finished output buffer.
        const VPIStatus s_submit = vpiSubmitRemap(
            impl_->stream, VPI_BACKEND_CUDA, impl_->remap_payload,
            in_img, out_img, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
        if (s_submit != VPI_SUCCESS)
        {
            checkVpi(s_submit, "vpiSubmitRemap");
            std::lock_guard<std::mutex> lk(impl_->pool_mutex);
            impl_->free_indices.push(out_idx);
            impl_->failed_frames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        const VPIStatus s_sync = vpiStreamSync(impl_->stream);
        if (s_sync != VPI_SUCCESS)
        {
            checkVpi(s_sync, "vpiStreamSync");
            std::lock_guard<std::mutex> lk(impl_->pool_mutex);
            impl_->free_indices.push(out_idx);
            impl_->failed_frames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        const auto t1 = std::chrono::steady_clock::now();
        impl_->remap_us_total.fetch_add(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count(),
            std::memory_order_relaxed);
        impl_->processed_frames.fetch_add(1, std::memory_order_relaxed);

        nv12_fd_out = out_fd;
        return true;
    }

    void VideoUndistorter::releaseFd(int dmabuf_fd)
    {
        std::lock_guard<std::mutex> lk(impl_->pool_mutex);
        auto it = impl_->fd_to_index.find(dmabuf_fd);
        if (it != impl_->fd_to_index.end())
        {
            impl_->free_indices.push(it->second);
        }
    }

    VideoUndistorter::StatsSnapshot VideoUndistorter::consumeStats()
    {
        StatsSnapshot s;
        s.processed_frames = impl_->processed_frames.exchange(0, std::memory_order_relaxed);
        s.pool_drops = impl_->pool_drops.exchange(0, std::memory_order_relaxed);
        s.failed_frames = impl_->failed_frames.exchange(0, std::memory_order_relaxed);
        s.remap_us_total = impl_->remap_us_total.exchange(0, std::memory_order_relaxed);
#ifdef TRB_CUDA_AVAILABLE
        if (impl_->cuda_remapper)
        {
            const auto cuda_stats = impl_->cuda_remapper->consumeStats();
            s.cuda_submitted = cuda_stats.submitted;
            s.cuda_completed = cuda_stats.completed;
            s.cuda_failures = cuda_stats.failures;
            s.cuda_kernel_us_total = cuda_stats.kernel_us_total;
        }
#endif
        return s;
    }

} // namespace trb::video
