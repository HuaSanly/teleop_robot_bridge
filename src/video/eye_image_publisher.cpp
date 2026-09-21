#include "video/eye_image_publisher.hpp"

#include "video/cuda_eye_image_processor.hpp"

#include <chrono>

#include <cuda_runtime_api.h>
#include "sensor_msgs/image_encodings.hpp"

namespace trb::video
{

EyeImagePublisher::EyeImagePublisher() = default;

EyeImagePublisher::~EyeImagePublisher()
{
    stop();
    destroyBuffers();
}

void EyeImagePublisher::stop()
{
    if (!initialized_.load())
    {
        return;
    }
    initialized_.store(false);
}

void EyeImagePublisher::destroyBuffers()
{
    for (uint8_t *buf : rgb_buffers_)
    {
        if (buf)
        {
            cudaFreeHost(buf);
        }
    }
    rgb_buffers_.clear();
    rgb_buffer_size_ = 0;

    if (cuda_processor_)
    {
        cuda_processor_->reset();
        cuda_processor_.reset();
    }
    initialized_.store(false);
}

bool EyeImagePublisher::initialize(rclcpp::Node & nh, uint32_t src_width, uint32_t src_height, const Config& config)
{
    if (initialized_.load())
    {
        RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "EyeImagePublisher already initialized");
        return true;
    }

    config_ = config;
    src_width_ = src_width;
    src_height_ = src_height;

    if (!config_.enabled)
    {
        RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "EyeImagePublisher disabled by config");
        return true;
    }

    left_pub_ = nh.create_publisher<sensor_msgs::msg::Image>(config_.left_topic, rclcpp::QoS(1));
    right_pub_ = nh.create_publisher<sensor_msgs::msg::Image>(config_.right_topic, rclcpp::QoS(1));

    rgb_buffer_size_ = static_cast<size_t>(config_.output_width) * config_.output_height * 3u;
    rgb_buffers_.resize(2, nullptr);
    for (size_t i = 0; i < rgb_buffers_.size(); ++i)
    {
        if (cudaHostAlloc(reinterpret_cast<void **>(&rgb_buffers_[i]), rgb_buffer_size_, cudaHostAllocPortable) != cudaSuccess)
        {
            RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"), "EyeImagePublisher: cudaHostAlloc failed");
            destroyBuffers();
            return false;
        }
    }

    cuda_processor_ = std::make_unique<CudaEyeImageProcessor>();
    if (!cuda_processor_->initialize(src_width_, src_height_, config_.output_width, config_.output_height))
    {
        RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"), "EyeImagePublisher: CUDA processor initialize failed");
        destroyBuffers();
        return false;
    }

    initialized_.store(true);
    RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                "EyeImagePublisher initialized (CUDA): src=%ux%u, out=%ux%u, topics=[%s, %s]",
                src_width_, src_height_,
                config_.output_width, config_.output_height,
                config_.left_topic.c_str(), config_.right_topic.c_str());
    return true;
}

void EyeImagePublisher::processFrame(NvBufSurface* src_surface, uint64_t timestamp_us)
{
    if (!initialized_.load() || !config_.enabled || !src_surface || !cuda_processor_)
    {
        return;
    }

    if (left_pub_->get_subscription_count() == 0 && right_pub_->get_subscription_count() == 0)
    {
        return;
    }

    const uint64_t n = frame_count_.fetch_add(1);
    const auto t0 = std::chrono::steady_clock::now();

    if (left_pub_->get_subscription_count() > 0)
    {
        CudaEyeImageResult result;
        if (cuda_processor_->processEye(src_surface, false, rgb_buffers_[0], rgb_buffer_size_, &result))
        {
            publishImage(left_pub_, rgb_buffers_[0], timestamp_us);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 *rclcpp::Clock::make_shared(),
                                 1000,
                                 "EyeImagePublisher: left CUDA process failed stage=%s err=%d",
                                 result.error_stage,
                                 result.error_code);
        }
    }

    if (right_pub_->get_subscription_count() > 0)
    {
        CudaEyeImageResult result;
        if (cuda_processor_->processEye(src_surface, true, rgb_buffers_[1], rgb_buffer_size_, &result))
        {
            publishImage(right_pub_, rgb_buffers_[1], timestamp_us);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 *rclcpp::Clock::make_shared(),
                                 1000,
                                 "EyeImagePublisher: right CUDA process failed stage=%s err=%d",
                                 result.error_stage,
                                 result.error_code);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    if (n == 0 || (n % 300 == 0))
    {
        const int64_t total_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                    "EyeImagePublisher (CUDA): frame %lu, process=%.2fms, subs=[L:%zu, R:%zu]",
                    static_cast<unsigned long>(n),
                    static_cast<double>(total_us) / 1000.0,
                    left_pub_->get_subscription_count(),
                    right_pub_->get_subscription_count());
    }
}

void EyeImagePublisher::publishImage(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub, const uint8_t* rgb_data, uint64_t timestamp_us)
{
    sensor_msgs::msg::Image msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_us) * 1000LL);
    msg.header.frame_id = config_.frame_id;
    msg.height = config_.output_height;
    msg.width = config_.output_width;
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    msg.is_bigendian = false;
    msg.step = config_.output_width * 3;
    msg.data.assign(rgb_data, rgb_data + static_cast<size_t>(msg.step) * msg.height);
    pub->publish(msg);
}

} // namespace trb::video
