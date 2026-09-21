#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <linux/videodev2.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "video/video_v4l2_capturer.hpp"

namespace trb::video
{

/**
 * @brief Captures one V4L2 wrist camera and publishes RGB images on a worker thread.
 *
 * A publisher owns its V4L2 session and retries initialization after device
 * failures so the two wrist cameras can fail and recover independently.
 */
class WristCameraPublisher
{
public:
    struct Config
    {
        std::string device_path;
        std::string topic;
        std::string frame_id;
        uint32_t width = 640;
        uint32_t height = 480;
        uint32_t framerate = 30;
        uint32_t pixel_format = V4L2_PIX_FMT_YUYV;
        uint32_t buffer_count = 4;
        uint32_t retry_interval_ms = 1000;
    };

    WristCameraPublisher(rclcpp::Node &node, std::string name, Config config);
    ~WristCameraPublisher();

    WristCameraPublisher(const WristCameraPublisher &) = delete;
    WristCameraPublisher &operator=(const WristCameraPublisher &) = delete;

    bool start();
    void stop();

private:
    void threadMain();
    void waitForRetry();
    void clearActiveCapturer(const std::shared_ptr<VideoV4L2Capturer> &capturer);

    rclcpp::Node &node_;
    rclcpp::Logger logger_;
    std::string name_;
    Config config_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::atomic<bool> running_{false};
    std::thread worker_thread_;
    std::mutex capturer_mutex_;
    std::shared_ptr<VideoV4L2Capturer> active_capturer_;
    std::mutex retry_mutex_;
    std::condition_variable retry_cv_;
};

} // namespace trb::video

