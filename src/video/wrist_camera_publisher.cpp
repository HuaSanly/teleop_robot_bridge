#include "video/wrist_camera_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "sensor_msgs/image_encodings.hpp"

namespace trb::video
{

WristCameraPublisher::WristCameraPublisher(rclcpp::Node &node, std::string name, Config config)
    : node_(node), logger_(node.get_logger()), name_(std::move(name)), config_(std::move(config))
{
}

WristCameraPublisher::~WristCameraPublisher()
{
    stop();
}

bool WristCameraPublisher::start()
{
    if (running_.exchange(true, std::memory_order_acq_rel))
    {
        return true;
    }

    if (config_.device_path.empty() || config_.topic.empty() || config_.width == 0 || config_.height == 0 ||
        config_.framerate == 0 || config_.pixel_format != V4L2_PIX_FMT_YUYV)
    {
        RCLCPP_ERROR(logger_, "Invalid %s wrist camera configuration", name_.c_str());
        running_.store(false, std::memory_order_release);
        return false;
    }

    try
    {
        publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(config_.topic, rclcpp::QoS(1));
        if (!publisher_)
        {
            running_.store(false, std::memory_order_release);
            RCLCPP_ERROR(logger_, "Failed to create %s wrist camera publisher for topic '%s'",
                         name_.c_str(), config_.topic.c_str());
            return false;
        }
        worker_thread_ = std::thread(&WristCameraPublisher::threadMain, this);
    }
    catch (const std::exception &error)
    {
        running_.store(false, std::memory_order_release);
        publisher_.reset();
        RCLCPP_ERROR(logger_, "Failed to start %s wrist camera publisher: %s", name_.c_str(), error.what());
        return false;
    }

    RCLCPP_INFO(logger_, "%s wrist camera worker started: device=%s topic=%s", name_.c_str(),
                config_.device_path.c_str(), config_.topic.c_str());
    return true;
}

void WristCameraPublisher::stop()
{
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    retry_cv_.notify_all();

    std::shared_ptr<VideoV4L2Capturer> capturer;
    {
        std::lock_guard<std::mutex> lock(capturer_mutex_);
        capturer = active_capturer_;
    }
    if (capturer)
    {
        capturer->interrupt();
    }

    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(capturer_mutex_);
        active_capturer_.reset();
    }
    publisher_.reset();

    if (was_running)
    {
        RCLCPP_INFO(logger_, "%s wrist camera worker stopped", name_.c_str());
    }
}

void WristCameraPublisher::clearActiveCapturer(const std::shared_ptr<VideoV4L2Capturer> &capturer)
{
    std::lock_guard<std::mutex> lock(capturer_mutex_);
    if (active_capturer_ == capturer)
    {
        active_capturer_.reset();
    }
}

void WristCameraPublisher::waitForRetry()
{
    std::unique_lock<std::mutex> lock(retry_mutex_);
    retry_cv_.wait_for(lock, std::chrono::milliseconds(std::max(1u, config_.retry_interval_ms)),
                       [this]() { return !running_.load(std::memory_order_acquire); });
}

void WristCameraPublisher::threadMain()
{
    const size_t expected_frame_size = static_cast<size_t>(config_.width) * config_.height * 2u;
    uint64_t failed_attempts = 0;

    while (running_.load(std::memory_order_acquire))
    {
        auto capturer = std::make_shared<VideoV4L2Capturer>();
        VideoV4L2Capturer::Config capture_config;
        capture_config.device_path = config_.device_path;
        capture_config.width = config_.width;
        capture_config.height = config_.height;
        capture_config.framerate = config_.framerate;
        capture_config.pixel_format = config_.pixel_format;
        capture_config.buffer_count = config_.buffer_count;
        capture_config.memory_mode = VideoV4L2Capturer::MemoryMode::kMmap;

        if (!capturer->initialize(capture_config) ||
            capturer->width() != config_.width || capturer->height() != config_.height ||
            capturer->pixelFormat() != config_.pixel_format || !capturer->start())
        {
            ++failed_attempts;
            capturer->stop();
            if ((failed_attempts == 1u) || (failed_attempts % 10u == 0u))
            {
                RCLCPP_WARN(logger_, "%s wrist camera unavailable at %s; retrying in %u ms (attempt=%lu)",
                            name_.c_str(), config_.device_path.c_str(), config_.retry_interval_ms,
                            static_cast<unsigned long>(failed_attempts));
            }
            waitForRetry();
            continue;
        }

        failed_attempts = 0;
        {
            std::lock_guard<std::mutex> lock(capturer_mutex_);
            active_capturer_ = capturer;
        }
        RCLCPP_INFO(logger_, "%s wrist camera streaming: %ux%u @ %u fps, YUYV",
                    name_.c_str(), config_.width, config_.height, config_.framerate);

        uint32_t consecutive_dequeue_failures = 0;
        while (running_.load(std::memory_order_acquire) && capturer->isRunning())
        {
            VideoV4L2Capturer::Frame frame;
            if (!capturer->dequeue(frame, 100))
            {
                ++consecutive_dequeue_failures;
                if (consecutive_dequeue_failures >= 10u)
                {
                    RCLCPP_WARN(logger_, "%s wrist camera stopped returning frames; restarting", name_.c_str());
                    break;
                }
                continue;
            }
            consecutive_dequeue_failures = 0;

            if (frame.data == nullptr || frame.size < expected_frame_size)
            {
                RCLCPP_WARN(logger_, "%s wrist camera returned a short YUYV frame (%zu bytes, expected %zu)",
                            name_.c_str(), frame.size, expected_frame_size);
                (void)capturer->requeue(frame.v4l2_buf);
                continue;
            }

            if (publisher_->get_subscription_count() > 0)
            {
                try
                {
                    const cv::Mat yuyv(static_cast<int>(config_.height), static_cast<int>(config_.width),
                                       CV_8UC2, const_cast<uint8_t *>(frame.data));
                    cv::Mat rgb;
                    cv::cvtColor(yuyv, rgb, cv::COLOR_YUV2RGB_YUY2);

                    sensor_msgs::msg::Image message;
                    message.header.stamp = node_.get_clock()->now();
                    message.header.frame_id = config_.frame_id;
                    message.height = config_.height;
                    message.width = config_.width;
                    message.encoding = sensor_msgs::image_encodings::RGB8;
                    message.is_bigendian = false;
                    message.step = config_.width * 3u;
                    message.data.assign(rgb.data, rgb.data + static_cast<size_t>(message.step) * message.height);
                    publisher_->publish(message);
                }
                catch (const cv::Exception &error)
                {
                    RCLCPP_WARN(logger_, "%s wrist camera YUYV to RGB conversion failed: %s",
                                name_.c_str(), error.what());
                }
            }

            if (!capturer->requeue(frame.v4l2_buf))
            {
                RCLCPP_WARN(logger_, "%s wrist camera failed to requeue a V4L2 buffer; restarting", name_.c_str());
                break;
            }
        }

        capturer->stop();
        clearActiveCapturer(capturer);
        if (running_.load(std::memory_order_acquire))
        {
            waitForRetry();
        }
    }
}

} // namespace trb::video
