#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>

// Forward declarations (Jetson MM API / V4L2 types live in global namespace)
struct v4l2_buffer;
class NvBuffer;
class NvVideoDecoder;

namespace trb::video
{

/**
 * Synchronous MJPEG decoder wrapper around NvVideoDecoder.
 *
 * Threading model:
 *   - submit()        → called from capture_thread to push MJPEG bitstream.
 *   - dequeueSync()   → called from encode_thread to pull decoded YUV DMA-BUF.
 *   - requeueCapture() → called from encode_thread after the YUV fd is no
 *                        longer needed, to return the capture buffer to the hw.
 *
 * The hardware capture-plane FIFO naturally decouples the two threads;
 * no additional CPU-side queue is required.
 */
class VideoDecoder
{
public:
    struct Config
    {
        uint32_t width = 3840;
        uint32_t height = 1520;

        // Number of decoder OUTPUT (bitstream input) buffers.
        uint32_t output_plane_buffers = 6;

        // Number of decoder CAPTURE (raw output) buffers.
        uint32_t capture_plane_buffers = 12;

        // Upper bound for one MJPEG frame size in bytes.
        uint32_t max_mjpeg_size_bytes = 4 * 1024 * 1024;
    };

    // A decoded YUV frame dequeued from the hardware capture plane.
    // Must be returned via requeueCapture() when no longer needed.
    struct DecodedFrame
    {
        int      yuv_dmabuf_fd{-1};
        uint64_t frame_id{0};
        uint64_t capture_timestamp_us{0};
        uint64_t capture_steady_us{0};
        uint64_t decode_us{0};
        // Internal bookkeeping – do not modify.
        uint32_t _cap_buf_index{0};
        NvBuffer* _nvbuf{nullptr};
    };

    VideoDecoder();
    ~VideoDecoder();

    VideoDecoder(const VideoDecoder&) = delete;
    VideoDecoder& operator=(const VideoDecoder&) = delete;

    bool initialize(const Config& config);

    // Feed one MJPEG bitstream frame into the decoder.
    // Returns false if the frame was dropped (e.g. no free input buffer).
    // Thread-safe; intended for capture_thread.
    bool submit(const uint8_t* data,
                size_t size,
                uint64_t frame_id,
                uint64_t capture_timestamp_us,
                uint64_t capture_steady_us);

    // Block until a decoded YUV frame is available or timeout_ms elapses.
    // Returns true and fills |out| on success.
    // Handles the first-frame RESOLUTION_CHANGE event internally.
    // Thread-safe; intended for encode_thread.
    bool dequeueSync(DecodedFrame& out, int timeout_ms = 100);

    // Return a previously dequeued frame's capture buffer to the hardware.
    // Must be called from the same thread as dequeueSync.
    void requeueCapture(DecodedFrame& frame);

private:
    struct FrameMeta
    {
        uint64_t frame_id{0};
        uint64_t capture_timestamp_us{0};
        uint64_t capture_steady_us{0};
        uint64_t decode_start_steady_us{0};
    };

    static bool decoderOutputPlaneDqCallback(::v4l2_buffer* v4l2_buf,
                                             ::NvBuffer* buffer,
                                             ::NvBuffer* shared_buffer,
                                             void* arg);

    bool setupCapturePlane_();
    void drainOutputDone_();

    Config config_;

    std::unique_ptr<NvVideoDecoder> decoder_;

    std::mutex input_mutex_;
    std::queue<int> free_output_indices_;

    std::mutex meta_mutex_;
    std::queue<FrameMeta> meta_queue_;

    std::mutex cap_setup_mutex_;
    std::atomic<bool> capture_plane_ready_{false};

    std::thread event_thread_;

    std::atomic<bool> stop_{false};
};

} // namespace trb::video
