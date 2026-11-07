#include "video/video_decoder.hpp"

#include <cstring>
#include <chrono>
#include <cerrno>
#include <fcntl.h>

#include <linux/videodev2.h>

#include <ros/ros.h>

#include "NvVideoDecoder.h"
#include "NvBuffer.h"

namespace trb::video
{

namespace
{
static uint64_t nowSteadyUs()
{
    const auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());
}
} // namespace

VideoDecoder::VideoDecoder() = default;

VideoDecoder::~VideoDecoder()
{
    stop_.store(true, std::memory_order_relaxed);

    if (event_thread_.joinable())
    {
        event_thread_.join();
    }

    if (decoder_)
    {
        decoder_.reset();
    }
}

bool VideoDecoder::initialize(const Config& config)
{
    config_ = config;
    stop_.store(false, std::memory_order_relaxed);
    capture_plane_ready_.store(false, std::memory_order_relaxed);

    // O_NONBLOCK makes dqBuffer's `num_retries` arg act as a millisecond timeout
    // (per NvV4l2ElementPlane.h docs) instead of a retry counter that sleeps on
    // each iteration. Required for non-blocking polls with timeout_ms=0.
    decoder_.reset(NvVideoDecoder::createVideoDecoder("dec0", O_NONBLOCK));
    if (!decoder_)
    {
        ROS_ERROR("VideoDecoder: failed to create NvVideoDecoder");
        return false;
    }

    if (decoder_->setOutputPlaneFormat(V4L2_PIX_FMT_MJPEG, config_.max_mjpeg_size_bytes) < 0)
    {
        ROS_ERROR("VideoDecoder: setOutputPlaneFormat(MJPEG) failed");
        return false;
    }

    const uint32_t out_bufs = (config_.output_plane_buffers > 0) ? config_.output_plane_buffers : 6;
    if (decoder_->output_plane.setupPlane(V4L2_MEMORY_MMAP, out_bufs, true, false) < 0)
    {
        ROS_ERROR("VideoDecoder: setupPlane(output) failed");
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(input_mutex_);
        while (!free_output_indices_.empty())
            free_output_indices_.pop();
        for (uint32_t i = 0; i < decoder_->output_plane.getNumBuffers(); ++i)
            free_output_indices_.push(static_cast<int>(i));
    }

    if (decoder_->output_plane.setStreamStatus(true) < 0)
    {
        ROS_ERROR("VideoDecoder: setStreamStatus(output) failed");
        return false;
    }

    if (decoder_->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0) < 0)
    {
        ROS_ERROR("VideoDecoder: subscribeEvent(RESOLUTION_CHANGE) failed");
        return false;
    }

    // NOTE: No output-plane DQ thread. With O_NONBLOCK on the fd the DQ-thread
    // wrapper would immediately get EAGAIN and stop recycling indices. Instead
    // we lazily drain completed output buffers on each submit() via
    // drainOutputDone_().

    // One-shot helper thread: wait for V4L2_EVENT_RESOLUTION_CHANGE, then
    // initialize the capture plane. dqEvent with a small blocking timeout is
    // used so we reliably catch the event instead of polling at 0ms from the
    // hot path.
    event_thread_ = std::thread([this]() {
        while (!stop_.load(std::memory_order_relaxed))
        {
            if (capture_plane_ready_.load(std::memory_order_acquire))
                return;

            struct v4l2_event ev;
            std::memset(&ev, 0, sizeof(ev));
            const int ret = decoder_->dqEvent(ev, 100);
            if (ret < 0)
            {
                if (errno == EAGAIN || errno == ETIMEDOUT)
                    continue;
                ROS_WARN("VideoDecoder: dqEvent failed errno=%d", errno);
                return;
            }
            if (ev.type != V4L2_EVENT_RESOLUTION_CHANGE)
                continue;
            if (!setupCapturePlane_())
            {
                ROS_ERROR("VideoDecoder: setupCapturePlane_ failed after RESOLUTION_CHANGE");
            }
            return;
        }
    });

    return true;
}

bool VideoDecoder::submit(const uint8_t* data,
                          size_t size,
                          uint64_t frame_id,
                          uint64_t capture_timestamp_us,
                          uint64_t capture_steady_us)
{
    if (!decoder_ || !data || size == 0)
        return false;

    // Lazily reap finished output-plane buffers so their indices are recycled.
    drainOutputDone_();

    if (size > config_.max_mjpeg_size_bytes)
    {
        ROS_WARN_THROTTLE(1.0, "VideoDecoder: MJPEG frame too large (%zu > %u), dropping",
                          size, config_.max_mjpeg_size_bytes);
        return false;
    }

    int index = -1;
    {
        std::lock_guard<std::mutex> lk(input_mutex_);
        if (free_output_indices_.empty())
            return false;
        index = free_output_indices_.front();
        free_output_indices_.pop();
    }

    // Throttle submissions before the capture plane is ready.
    if (!capture_plane_ready_.load(std::memory_order_acquire))
    {
        std::lock_guard<std::mutex> lk(meta_mutex_);
        if (meta_queue_.size() >= 2)
        {
            std::lock_guard<std::mutex> lk2(input_mutex_);
            free_output_indices_.push(index);
            return false;
        }
    }

    NvBuffer* buffer = decoder_->output_plane.getNthBuffer(index);
    if (!buffer || buffer->planes[0].data == nullptr)
    {
        std::lock_guard<std::mutex> lk(input_mutex_);
        free_output_indices_.push(index);
        return false;
    }

    if (buffer->planes[0].length < size)
    {
        ROS_WARN_THROTTLE(1.0, "VideoDecoder: output plane buffer too small (%u < %zu)",
                          buffer->planes[0].length, size);
        std::lock_guard<std::mutex> lk(input_mutex_);
        free_output_indices_.push(index);
        return false;
    }

    std::memcpy(buffer->planes[0].data, data, size);
    buffer->planes[0].bytesused = static_cast<uint32_t>(size);

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane  planes[MAX_PLANES];
    std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    std::memset(planes, 0, sizeof(planes));

    v4l2_buf.index            = static_cast<uint32_t>(index);
    v4l2_buf.m.planes         = planes;
    v4l2_buf.type             = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    v4l2_buf.memory           = V4L2_MEMORY_MMAP;
    v4l2_buf.length           = decoder_->output_plane.getNumPlanes();
    v4l2_buf.timestamp.tv_sec  = capture_timestamp_us / 1000000ULL;
    v4l2_buf.timestamp.tv_usec = capture_timestamp_us % 1000000ULL;
    planes[0].bytesused       = static_cast<uint32_t>(size);

    {
        std::lock_guard<std::mutex> lk(meta_mutex_);
        FrameMeta m;
        m.frame_id               = frame_id;
        m.capture_timestamp_us   = capture_timestamp_us;
        m.capture_steady_us      = capture_steady_us;
        m.decode_start_steady_us = nowSteadyUs();
        meta_queue_.push(m);
    }

    if (decoder_->output_plane.qBuffer(v4l2_buf, buffer) < 0)
    {
        {
            std::lock_guard<std::mutex> lk(meta_mutex_);
            if (!meta_queue_.empty())
                meta_queue_.pop();
        }
        std::lock_guard<std::mutex> lk(input_mutex_);
        free_output_indices_.push(index);
        return false;
    }

    return true;
}

bool VideoDecoder::dequeueSync(DecodedFrame& out, int timeout_ms)
{
    if (!decoder_)
        return false;

    // Capture plane is set up asynchronously by event_thread_. Until that is
    // done there is nothing to dequeue.
    if (!capture_plane_ready_.load(std::memory_order_acquire))
        return false;

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane  planes[MAX_PLANES];
    std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    std::memset(planes,    0, sizeof(planes));
    v4l2_buf.m.planes = planes;

    // In O_NONBLOCK mode, num_retries is the timeout in milliseconds.
    const uint32_t retries = static_cast<uint32_t>(std::max(0, timeout_ms));
    NvBuffer* nvbuf = nullptr;
    if (decoder_->capture_plane.dqBuffer(v4l2_buf, &nvbuf, nullptr, retries) < 0)
    {
        if (errno != EAGAIN && errno != ETIMEDOUT)
        {
            ROS_WARN_THROTTLE(1.0, "VideoDecoder: capture_plane.dqBuffer failed errno=%d", errno);
        }
        return false;
    }

    if (!nvbuf)
        return false;

    FrameMeta meta{};
    bool have_meta = false;
    {
        std::lock_guard<std::mutex> lk(meta_mutex_);
        if (!meta_queue_.empty())
        {
            meta      = meta_queue_.front();
            meta_queue_.pop();
            have_meta = true;
        }
    }

    const uint64_t now_us = nowSteadyUs();
    out.yuv_dmabuf_fd        = nvbuf->planes[0].fd;
    out.frame_id             = have_meta ? meta.frame_id             : 0;
    out.capture_timestamp_us = have_meta ? meta.capture_timestamp_us : 0;
    out.capture_steady_us    = have_meta ? meta.capture_steady_us    : 0;
    out.decode_us            = (have_meta && now_us >= meta.decode_start_steady_us)
                                   ? (now_us - meta.decode_start_steady_us) : 0;
    out._cap_buf_index = v4l2_buf.index;
    out._nvbuf         = nvbuf;

    return true;
}

void VideoDecoder::requeueCapture(DecodedFrame& frame)
{
    if (!decoder_ || !frame._nvbuf)
        return;

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane  planes[MAX_PLANES];
    std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    std::memset(planes,    0, sizeof(planes));
    v4l2_buf.index    = frame._cap_buf_index;
    v4l2_buf.m.planes = planes;
    v4l2_buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l2_buf.memory   = V4L2_MEMORY_MMAP;

    decoder_->capture_plane.qBuffer(v4l2_buf, frame._nvbuf);
    frame._nvbuf = nullptr;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool VideoDecoder::setupCapturePlane_()
{
    std::lock_guard<std::mutex> lk(cap_setup_mutex_);
    if (capture_plane_ready_.load(std::memory_order_relaxed))
        return true;

    struct v4l2_format format;
    std::memset(&format, 0, sizeof(format));
    if (decoder_->capture_plane.getFormat(format) < 0)
    {
        ROS_ERROR("VideoDecoder: capture_plane.getFormat failed");
        return false;
    }

    decoder_->capture_plane.deinitPlane();

    if (decoder_->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,
                                        format.fmt.pix_mp.width,
                                        format.fmt.pix_mp.height) < 0)
    {
        ROS_ERROR("VideoDecoder: setCapturePlaneFormat failed pixfmt=0x%x w=%u h=%u",
                  format.fmt.pix_mp.pixelformat,
                  format.fmt.pix_mp.width,
                  format.fmt.pix_mp.height);
        return false;
    }

    int32_t min_cap_bufs = 0;
    if (decoder_->getMinimumCapturePlaneBuffers(min_cap_bufs) < 0)
    {
        ROS_ERROR("VideoDecoder: getMinimumCapturePlaneBuffers failed");
        return false;
    }

    const uint32_t cap_bufs = std::max<uint32_t>(
        static_cast<uint32_t>(std::max<int32_t>(min_cap_bufs, 1)),
        (config_.capture_plane_buffers > 0) ? config_.capture_plane_buffers : 12u);

    if (decoder_->capture_plane.setupPlane(V4L2_MEMORY_MMAP, cap_bufs, false, false) < 0)
    {
        ROS_ERROR("VideoDecoder: setupPlane(capture, MMAP) failed");
        return false;
    }

    if (decoder_->capture_plane.setStreamStatus(true) < 0)
    {
        ROS_ERROR("VideoDecoder: setStreamStatus(capture) failed");
        return false;
    }

    for (uint32_t i = 0; i < decoder_->capture_plane.getNumBuffers(); ++i)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane  planes[MAX_PLANES];
        std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        std::memset(planes,    0, sizeof(planes));

        v4l2_buf.index    = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory   = V4L2_MEMORY_MMAP;

        if (decoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0)
        {
            ROS_ERROR("VideoDecoder: qBuffer(capture, %u) failed", i);
            return false;
        }
    }

    capture_plane_ready_.store(true, std::memory_order_release);
    ROS_INFO("VideoDecoder: capture plane ready (%u bufs, %ux%u)",
             cap_bufs, format.fmt.pix_mp.width, format.fmt.pix_mp.height);
    return true;
}

bool VideoDecoder::decoderOutputPlaneDqCallback(struct v4l2_buffer* /*v4l2_buf*/,
                                                NvBuffer* /*buffer*/,
                                                NvBuffer* /*shared_buffer*/,
                                                void* /*arg*/)
{
    // Unused: output plane is drained lazily in drainOutputDone_() on each
    // submit() call. Kept only to satisfy the header signature.
    return false;
}

void VideoDecoder::drainOutputDone_()
{
    if (!decoder_)
        return;

    while (true)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane  planes[MAX_PLANES];
        std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        std::memset(planes,    0, sizeof(planes));
        v4l2_buf.m.planes = planes;

        NvBuffer* buf = nullptr;
        // num_retries=0 in O_NONBLOCK mode = non-blocking single attempt.
        if (decoder_->output_plane.dqBuffer(v4l2_buf, &buf, nullptr, 0) < 0)
        {
            break;
        }
        std::lock_guard<std::mutex> lk(input_mutex_);
        free_output_indices_.push(static_cast<int>(v4l2_buf.index));
    }
}

} // namespace trb::video
