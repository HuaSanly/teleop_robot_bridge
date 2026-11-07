#pragma once

#include <cstdint>
#include <string>
#include <atomic>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace trb::udp
{

/**
 * @brief Configuration for PosePublisher
 */
struct PosePublisherConfig
{
    bool enabled = true;
    
    // Frame IDs for published messages
    std::string frame_id_hmd = "vr_hmd";
    std::string frame_id_left_controller = "vr_left_controller";
    std::string frame_id_right_controller = "vr_right_controller";
    std::string frame_id_left_aim = "vr_left_aim";
    std::string frame_id_right_aim = "vr_right_aim";
    
    // What to publish
    bool publish_euler = true;
    bool publish_buttons = true;
    bool publish_aim = true;
};

/**
 * @brief Parses VR pose UDP packets and publishes to ROS topics.
 * 
 * Protocol (v1, type=0x02):
 *   [0]:     type (0x02 = pose)
 *   [1-8]:   timestamp_ms (uint64 LE)
 *   [9]:     version (1)
 *   [10]:    flags (bit0=euler, bit1=aim, bit2=buttons)
 *   [11-12]: reserved
 *   [13+]:   payload (3x pose @ 28 bytes each = 84 bytes base)
 *            + optional euler (36 bytes)
 *            + optional buttons (46 bytes)
 *            + optional aim (76 bytes)
 */
class PosePublisher
{
public:
    PosePublisher();
    ~PosePublisher() = default;
    
    PosePublisher(const PosePublisher&) = delete;
    PosePublisher& operator=(const PosePublisher&) = delete;
    
    /**
     * @brief Initialize publishers with ROS NodeHandle and config.
     */
    bool initialize(ros::NodeHandle& nh, const PosePublisherConfig& config);
    
    /**
     * @brief Handle a received UDP datagram. Parses and publishes if valid pose packet.
     * @param data Pointer to packet data
     * @param size Packet size in bytes
     */
    void handleDatagram(const uint8_t* data, size_t size);
    
    /**
     * @brief Check if publisher is enabled
     */
    bool isEnabled() const { return config_.enabled && initialized_.load(); }

private:
    PosePublisherConfig config_;
    std::atomic<bool> initialized_{false};
    
    // Pose publishers
    ros::Publisher pub_hmd_;
    ros::Publisher pub_left_controller_;
    ros::Publisher pub_right_controller_;
    
    // Euler publishers
    ros::Publisher pub_hmd_euler_;
    ros::Publisher pub_left_controller_euler_;
    ros::Publisher pub_right_controller_euler_;
    
    // Joy (button) publishers
    ros::Publisher pub_left_joy_;
    ros::Publisher pub_right_joy_;
    
    // Aim publishers
    ros::Publisher pub_left_aim_;
    ros::Publisher pub_right_aim_;
    ros::Publisher pub_left_aim_valid_;
    ros::Publisher pub_right_aim_valid_;
    ros::Publisher pub_left_pinch_;
    ros::Publisher pub_right_pinch_;
    ros::Publisher pub_left_pinch_strength_;
    ros::Publisher pub_right_pinch_strength_;
    
    // Statistics
    std::atomic<uint64_t> packet_count_{0};
    ros::Time last_warn_time_;
    
    void parseAndPublish(const uint8_t* data, size_t len);
};

} // namespace trb::udp
