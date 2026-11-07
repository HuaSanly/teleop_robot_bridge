#include "video/video_v4l2_capturer.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace trb::video
{

    VideoV4L2Capturer::VideoV4L2Capturer() = default;

    VideoV4L2Capturer::~VideoV4L2Capturer()
    {
        stop();
    }

    bool VideoV4L2Capturer::initialize(const Config &config)
    {
        config_ = config;
        actual_width_ = 0;
        actual_height_ = 0;
        actual_pixel_format_ = 0;

        if (!openDevice())
            return false;

        if (!configureDevice())
        {
            closeDevice();
            return false;
        }

        return true;
    }

    bool VideoV4L2Capturer::start()
    {
        if (running_)
            return true;

        if (cam_fd_ < 0)
        {
            std::cerr << "VideoV4L2Capturer not initialized (device not opened)" << std::endl;
            return false;
        }

        if (!requestBuffers())
            return false;

        if (!streamOn())
            return false;

        running_ = true;
        return true;
    }

    void VideoV4L2Capturer::stop()
    {
        if (!running_)
        {
            closeDevice();
            return;
        }

        running_ = false;

        // STREAMOFF can break poll/DQBUF quickly
        streamOff();

        closeDevice();
    }

    bool VideoV4L2Capturer::isRunning() const
    {
        return running_.load();
    }

    uint32_t VideoV4L2Capturer::width() const
    {
        return actual_width_;
    }

    uint32_t VideoV4L2Capturer::height() const
    {
        return actual_height_;
    }

    uint32_t VideoV4L2Capturer::pixelFormat() const
    {
        return actual_pixel_format_;
    }

    bool VideoV4L2Capturer::openDevice()
    {
        if (cam_fd_ >= 0)
            return true;

        cam_fd_ = open(config_.device_path.c_str(), O_RDWR | O_NONBLOCK);
        if (cam_fd_ < 0)
        {
            std::cerr << "Failed to open V4L2 device '" << config_.device_path
                      << "': " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        return true;
    }

    bool VideoV4L2Capturer::pollForFrame(int timeout_ms)
    {
        struct pollfd fds[1];
        fds[0].fd = cam_fd_;
        fds[0].events = POLLIN;
        fds[0].revents = 0;

        const int pr = poll(fds, 1, timeout_ms);
        if (pr < 0)
        {
            if (errno == EINTR)
                return false;
            std::cerr << "poll failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        if (pr == 0)
            return false;

        return (fds[0].revents & POLLIN) != 0;
    }

    bool VideoV4L2Capturer::dequeue(Frame &frame, int timeout_ms)
    {
        if (!running_ || cam_fd_ < 0)
            return false;

        if (!pollForFrame(timeout_ms))
            return false;

        struct v4l2_buffer v4l2_buf;
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;

        {
            std::lock_guard<std::mutex> lk(ioctl_mutex_);
            if (ioctl(cam_fd_, VIDIOC_DQBUF, &v4l2_buf) < 0)
            {
                if (errno == EAGAIN || errno == EINTR)
                    return false;
                if (!running_)
                    return false;
                std::cerr << "VIDIOC_DQBUF failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
                return false;
            }
        }

        if (v4l2_buf.index >= buffers_.size() || buffers_[v4l2_buf.index].start == nullptr)
        {
            std::cerr << "VIDIOC_DQBUF returned invalid index=" << v4l2_buf.index
                      << " (buffers=" << buffers_.size() << ")" << std::endl;
            return false;
        }

        if (v4l2_buf.bytesused == 0)
        {
            std::cerr << "VIDIOC_DQBUF returned empty frame for index=" << v4l2_buf.index << std::endl;
            (void)requeue(v4l2_buf);
            return false;
        }

        if (v4l2_buf.bytesused > buffers_[v4l2_buf.index].length)
        {
            std::cerr << "VIDIOC_DQBUF returned oversized frame: bytesused=" << v4l2_buf.bytesused
                      << " mapped_length=" << buffers_[v4l2_buf.index].length
                      << " index=" << v4l2_buf.index << std::endl;
            (void)requeue(v4l2_buf);
            return false;
        }

        frame.v4l2_buf = v4l2_buf;
        frame.data = static_cast<const uint8_t *>(buffers_[v4l2_buf.index].start);
        frame.size = static_cast<size_t>(v4l2_buf.bytesused);
        frame.timestamp_us = static_cast<uint64_t>(v4l2_buf.timestamp.tv_sec) * 1000000ULL +
                             static_cast<uint64_t>(v4l2_buf.timestamp.tv_usec);

        return frame.data != nullptr && frame.size != 0;
    }

    bool VideoV4L2Capturer::requeue(const struct v4l2_buffer &v4l2_buf)
    {
        if (cam_fd_ < 0)
            return false;

        struct v4l2_buffer buf = v4l2_buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        std::lock_guard<std::mutex> lk(ioctl_mutex_);
        if (ioctl(cam_fd_, VIDIOC_QBUF, &buf) < 0)
        {
            std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }
        return true;
    }

    bool VideoV4L2Capturer::configureDevice()
    {
        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        if (ioctl(cam_fd_, VIDIOC_QUERYCAP, &cap) < 0)
        {
            std::cerr << "VIDIOC_QUERYCAP failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING))
        {
            std::cerr << "Device does not support V4L2 capture/streaming" << std::endl;
            return false;
        }

        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = config_.width;
        fmt.fmt.pix.height = config_.height;
        fmt.fmt.pix.pixelformat = config_.pixel_format;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        if (ioctl(cam_fd_, VIDIOC_S_FMT, &fmt) < 0)
        {
            std::cerr << "VIDIOC_S_FMT failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam_fd_, VIDIOC_G_FMT, &fmt) < 0)
        {
            std::cerr << "VIDIOC_G_FMT failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        actual_width_ = fmt.fmt.pix.width;
        actual_height_ = fmt.fmt.pix.height;
        actual_pixel_format_ = fmt.fmt.pix.pixelformat;

        // Log actual resolution
        char fourcc_str[5] = {0};
        fourcc_str[0] = static_cast<char>(actual_pixel_format_ & 0xFF);
        fourcc_str[1] = static_cast<char>((actual_pixel_format_ >> 8) & 0xFF);
        fourcc_str[2] = static_cast<char>((actual_pixel_format_ >> 16) & 0xFF);
        fourcc_str[3] = static_cast<char>((actual_pixel_format_ >> 24) & 0xFF);
        std::cout << "========== V4L2 Capturer Config ==========" << std::endl;
        std::cout << "  Requested: " << config_.width << "x" << config_.height << " @ " << config_.framerate << " fps" << std::endl;
        std::cout << "  Actual resolution: " << actual_width_ << "x" << actual_height_ << std::endl;
        std::cout << "  Actual pixel format: " << fourcc_str << " (0x" << std::hex << actual_pixel_format_ << std::dec << ")" << std::endl;

        // best-effort set framerate
        if (config_.framerate != 0)
        {
            struct v4l2_streamparm sp;
            memset(&sp, 0, sizeof(sp));
            sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            sp.parm.capture.timeperframe.numerator = 1;
            sp.parm.capture.timeperframe.denominator = config_.framerate;
            if (ioctl(cam_fd_, VIDIOC_S_PARM, &sp) < 0)
            {
                std::cerr << "WARN: VIDIOC_S_PARM failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            }
            
            // Read back actual framerate
            memset(&sp, 0, sizeof(sp));
            sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(cam_fd_, VIDIOC_G_PARM, &sp) == 0)
            {
                uint32_t num = sp.parm.capture.timeperframe.numerator;
                uint32_t den = sp.parm.capture.timeperframe.denominator;
                double actual_fps = (num > 0) ? static_cast<double>(den) / static_cast<double>(num) : 0.0;
                std::cout << "  Actual framerate: " << actual_fps << " fps (" << den << "/" << num << ")" << std::endl;
            }
        }
        std::cout << "===========================================" << std::endl;

        return true;
    }

    bool VideoV4L2Capturer::requestBuffers()
    {
        if (!buffers_.empty())
            return true;

        const uint32_t count = (config_.buffer_count == 0) ? 4 : config_.buffer_count;

        struct v4l2_requestbuffers rb;
        memset(&rb, 0, sizeof(rb));
        rb.count = count;
        rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rb.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam_fd_, VIDIOC_REQBUFS, &rb) < 0)
        {
            std::cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        if (rb.count == 0)
        {
            std::cerr << "Driver returned 0 buffers" << std::endl;
            return false;
        }

        buffers_.resize(rb.count);

        for (uint32_t index = 0; index < rb.count; ++index)
        {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.index = index;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (ioctl(cam_fd_, VIDIOC_QUERYBUF, &buf) < 0)
            {
                std::cerr << "VIDIOC_QUERYBUF failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
                return false;
            }

            buffers_[index].length = buf.length;
            buffers_[index].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd_, buf.m.offset);
            if (buffers_[index].start == MAP_FAILED)
            {
                std::cerr << "mmap failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
                buffers_[index].start = nullptr;
                return false;
            }

            if (ioctl(cam_fd_, VIDIOC_QBUF, &buf) < 0)
            {
                std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
                return false;
            }
        }

        return true;
    }

    bool VideoV4L2Capturer::streamOn()
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam_fd_, VIDIOC_STREAMON, &type) < 0)
        {
            std::cerr << "VIDIOC_STREAMON failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
            return false;
        }

        return true;
    }

    void VideoV4L2Capturer::streamOff()
    {
        if (cam_fd_ < 0)
            return;

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(cam_fd_, VIDIOC_STREAMOFF, &type);
    }

    void VideoV4L2Capturer::closeDevice()
    {
        for (auto &b : buffers_)
        {
            if (b.start && b.start != MAP_FAILED)
                munmap(b.start, b.length);
            b.start = nullptr;
            b.length = 0;
        }
        buffers_.clear();

        if (cam_fd_ >= 0)
        {
            close(cam_fd_);
            cam_fd_ = -1;
        }
    }

} // namespace trb::video
