#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

#include <linux/videodev2.h>

namespace trb::video
{

    class VideoV4L2Capturer
    {
    public:
        struct Frame
        {
            // Points into the currently dequeued V4L2 mmap buffer.
            // The pointer remains valid until requeue(frame.v4l2_buf) is called.
            const uint8_t *data = nullptr;
            size_t size = 0;
            uint64_t timestamp_us = 0;
            struct v4l2_buffer v4l2_buf;
        };

        struct Config
        {
            std::string device_path = "/dev/video0";
            uint32_t width = 3840;
            uint32_t height = 1520;
            uint32_t framerate = 30;
            uint32_t pixel_format = V4L2_PIX_FMT_MJPEG; // common for UVC
            uint32_t buffer_count = 8;
        };

        VideoV4L2Capturer();
        ~VideoV4L2Capturer();

        VideoV4L2Capturer(const VideoV4L2Capturer &) = delete;
        VideoV4L2Capturer &operator=(const VideoV4L2Capturer &) = delete;

        bool initialize(const Config &config);

        // Start/stop V4L2 streaming. No internal thread is created.
        bool start();
        void stop();

        bool isRunning() const;

        uint32_t width() const;
        uint32_t height() const;
        uint32_t pixelFormat() const;

        // Dequeue one frame. On success, caller owns the dequeued buffer until requeue().
        // timeout_ms: poll timeout; use -1 for infinite.
        bool dequeue(Frame &frame, int timeout_ms);

        // Requeue a previously dequeued buffer back to the driver.
        bool requeue(const struct v4l2_buffer &v4l2_buf);

    private:
        struct CamBuffer
        {
            void *start = nullptr;
            size_t length = 0;
        };

        bool openDevice();
        bool configureDevice();
        bool requestBuffers();
        bool streamOn();
        void streamOff();
        void closeDevice();

        bool pollForFrame(int timeout_ms);

        Config config_;
        uint32_t actual_width_ = 0;
        uint32_t actual_height_ = 0;
        uint32_t actual_pixel_format_ = 0;

        int cam_fd_ = -1;
        std::vector<CamBuffer> buffers_;

        // Serialize ioctl operations (DQBUF/QBUF/STREAMON/OFF) across threads.
        mutable std::mutex ioctl_mutex_;

        std::atomic<bool> running_{false};
    };

} // namespace trb::video
