#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace trb::video
{

    // Stereo undistort + rectify on Jetson, single-pass over a side-by-side
    // (left-half + right-half) NV12 frame. The default implementation uses a
    // custom CUDA remap; the legacy VPI path remains an explicit fallback.
    //
    // Lifecycle:
    //   1. construct, then initialize(cfg). initialize() reads the calibration
    //      YAML, runs cv::initUndistortRectifyMap() once per eye on the host,
    //      hconcat's the two per-eye dense maps into a single full-frame map
    //      (right-eye dst x is offset by +single_eye_width), uploads it to
    //      a CUDA device map (or VPI warp map for the fallback) and creates a
    //      pool of NV12 output NvBufSurface dmabufs for the encoder.
    //   2. process(in_fd, out_fd) submits one Remap operation and synchronously
    //      waits for completion. The caller owns the input fd lifecycle (this
    //      class never releases it) and must return the output fd via
    //      releaseFd() once the downstream consumers are done.
    //   3. destruction tears down CUDA/VPI resources and destroys the pool.
    //
    // Calibration YAML schema (see config/calibration_stereo.yaml.example):
    //   calibration_image_size: [W_calib, H_calib]
    //   runtime_image_size:     [W_run,   H_run]      # single-eye, e.g. 2560x1440
    //   distortion_model: "rational"                  # plumb_bob | rational
    //   alpha: 0.0                                    # 0=no black border, 1=keep FOV
    //   left:
    //     K: [9 floats]
    //     D: [4..14 floats]
    //     R: [9 floats]   # optional, defaults to identity
    //     P: [12 floats]  # optional, defaults to getOptimalNewCameraMatrix(...)
    //   right: { ... same ... }
    class VideoUndistorter
    {
    public:
        struct Config
        {
            // Full SBS frame size that VideoConverter emits (e.g. 5120x1440).
            uint32_t width = 5120;
            uint32_t height = 1440;

            // Output buffer pool depth. Mirror VideoConverter so we never
            // become the bottleneck of the encode thread.
            uint32_t buffer_pool_size = 4;

            // 0 = PITCH (linear) — required for NVENC input on Jetson.
            // 1 = BLOCK_LINEAR.
            int32_t output_surface_layout = 0;

            // Path (absolute or relative to package share) to the stereo
            // calibration YAML.
            std::string calibration_file;

            // If true and calibration cannot be loaded, initialize() returns
            // false and the pipeline should bypass undistortion. If false,
            // initialize() will fall back to an identity map (debug only).
            bool require_calibration = true;

            // Set false to force the legacy VPI backend.
            bool use_cuda = true;
        };

        struct StatsSnapshot
        {
            uint64_t processed_frames = 0;
            uint64_t pool_drops = 0;
            uint64_t failed_frames = 0;
            int64_t  remap_us_total = 0;
            uint64_t cuda_submitted = 0;
            uint64_t cuda_completed = 0;
            uint64_t cuda_failures = 0;
            uint64_t cuda_kernel_us_total = 0;
        };

        VideoUndistorter();
        ~VideoUndistorter();

        VideoUndistorter(const VideoUndistorter &) = delete;
        VideoUndistorter &operator=(const VideoUndistorter &) = delete;

        bool initialize(const Config &config);

        // Synchronously remap one full-frame SBS NV12 dmabuf into a freshly
        // pool-acquired NV12 dmabuf. Returns false if the pool is exhausted
        // or the CUDA/VPI submit/sync failed. The input fd is NOT released by this
        // call.
        bool process(int nv12_fd_in, int &nv12_fd_out);

        // Return a previously emitted dmabuf fd back to the internal pool.
        void releaseFd(int dmabuf_fd);

        StatsSnapshot consumeStats();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace trb::video
