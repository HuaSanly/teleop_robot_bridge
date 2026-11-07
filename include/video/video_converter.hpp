#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <unordered_map>
#include <atomic>

namespace trb::video
{

    class VideoConverter
    {
    public:
        struct StatsSnapshot
        {
            uint64_t processed_frames = 0;
            uint64_t malformed_frames = 0;
            uint64_t pool_drops = 0;
            uint64_t failed_frames = 0;
            int64_t decode_us_total = 0;
            int64_t transform_us_total = 0;
        };

        struct Config
        {
            uint32_t width = 3840;
            uint32_t height = 1520;
            // Optional: input size (e.g. original SBS width/height). If 0, uses width/height.
            uint32_t input_width = 0;
            uint32_t input_height = 0;
            uint32_t buffer_pool_size = 4;

            // NvBufSurface layout controls:
            // 0 = PITCH (linear), 1 = BLOCK_LINEAR.
            // On Jetson, BLOCK_LINEAR can be faster for some HW paths, but NVENC input
            // often behaves best with PITCH. Therefore we allow separate layouts.

            // Layout for the converter output NV12 surfaces (fed to encoder).
            int32_t output_surface_layout = 0;

            // Layout for the decoder output YUV422 surfaces (fed into transform).
            int32_t decode_surface_layout = 0;

            // Legacy (kept for API/backward compatibility). If you still set this,
            // it is treated as a default for decode surfaces only.
            int32_t surface_layout = 0;

            // NvBufSurfTransform compute device:
            // 0=Default, 1=GPU, 2=VIC (Jetson).
            // -1 means "use default".
            int32_t transform_compute_mode = -1;
        };

        VideoConverter();
        ~VideoConverter();

        VideoConverter(const VideoConverter &) = delete;
        VideoConverter &operator=(const VideoConverter &) = delete;

        bool initialize(const Config &config);

        // Synchronously convert one decoded YUV DMA-BUF to an NV12 DMA-BUF.
        // On success, fills |nv12_fd_out| with a pool-owned fd; caller must
        // return it via releaseFd() after the downstream consumer (encoder)
        // has finished with it.
        // Returns false if the pool is exhausted or the transform failed.
        bool transformSync(int yuv_dmabuf_fd,
                           uint64_t timestamp_us,
                           uint64_t decode_us,
                           int &nv12_fd_out);

        // Return a previously emitted dmabuf fd back to the internal pool.
        // Safe to call from any thread.
        void releaseFd(int dmabuf_fd);

        // Consume and reset converter statistics accumulated since the last snapshot.
        StatsSnapshot consumeStats();

    private:
        void destroyBuffers();

        Config config_;

        // Store NvBufSurface pointers to manage memory
        // Using void* to avoid including nvbufsurface.h in header
        std::vector<void *> surfaces_;
        std::vector<int> dmabuf_fds_;

        std::mutex pool_mutex_;
        std::queue<size_t> free_indices_;
        std::unordered_map<int, size_t> fd_to_index_;

        std::atomic<bool> logged_mode_{false};
        std::atomic<uint64_t> stats_processed_frames_{0};
        std::atomic<uint64_t> stats_malformed_frames_{0};
        std::atomic<uint64_t> stats_pool_drops_{0};
        std::atomic<uint64_t> stats_failed_frames_{0};
        std::atomic<int64_t> stats_decode_us_total_{0};
        std::atomic<int64_t> stats_transform_us_total_{0};
    };

} // namespace trb::video
