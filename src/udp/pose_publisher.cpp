#include "udp/pose_publisher.hpp"

#include <cstring>

namespace trb::udp
{

namespace
{
    constexpr uint8_t kPoseType = 0x02;
    constexpr uint8_t kProtocolVersion = 1;

    // Flags
    constexpr uint8_t kIncludeEuler = 0x01;
    constexpr uint8_t kIncludeAim = 0x02;
    constexpr uint8_t kIncludeButtons = 0x04;

    uint64_t readLeU64(const uint8_t* p)
    {
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i)
        {
            v |= (static_cast<uint64_t>(p[i]) << (8 * i));
        }
        return v;
    }

    uint32_t readLeU32(const uint8_t* p)
    {
        return (static_cast<uint32_t>(p[0]) << 0) |
               (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
    }

    float readLeF32(const uint8_t* p)
    {
        const uint32_t u = readLeU32(p);
        float f;
        static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
        std::memcpy(&f, &u, sizeof(f));
        return f;
    }

    size_t expectedPacketSize(uint8_t flags)
    {
        size_t total = 13 + 84;  // header + 3 poses
        if (flags & kIncludeEuler)
        {
            total += 36;  // 3 euler angles
        }
        if (flags & kIncludeButtons)
        {
            total += 46;  // 2 controllers' buttons
        }
        if (flags & kIncludeAim)
        {
            total += 76;  // 2 aim poses
        }
        return total;
    }

    geometry_msgs::PoseStamped makePoseMsg(
        const std::string& frame_id,
        const ros::Time& stamp,
        float px, float py, float pz,
        float qx, float qy, float qz, float qw)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.pose.position.x = px;
        msg.pose.position.y = py;
        msg.pose.position.z = pz;
        msg.pose.orientation.x = qx;
        msg.pose.orientation.y = qy;
        msg.pose.orientation.z = qz;
        msg.pose.orientation.w = qw;
        return msg;
    }

    geometry_msgs::Vector3Stamped makeEulerMsg(
        const std::string& frame_id,
        const ros::Time& stamp,
        float ex, float ey, float ez)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.vector.x = ex;
        msg.vector.y = ey;
        msg.vector.z = ez;
        return msg;
    }

    sensor_msgs::Joy makeJoyMsg(
        const ros::Time& stamp,
        float trigger_value,
        uint8_t trigger_button,
        float grip_value,
        uint8_t grip_button,
        uint8_t primary_button,
        uint8_t secondary_button,
        uint8_t menu_button,
        float thumbstick_x,
        float thumbstick_y,
        uint8_t thumbstick_click,
        uint8_t thumbstick_touch)
    {
        sensor_msgs::Joy msg;
        msg.header.stamp = stamp;

        // axes: trigger, grip, thumbstick_x, thumbstick_y
        msg.axes.resize(4);
        msg.axes[0] = trigger_value;
        msg.axes[1] = grip_value;
        msg.axes[2] = thumbstick_x;
        msg.axes[3] = thumbstick_y;

        // buttons: trigger, grip, primary, secondary, menu, thumbstick_click, thumbstick_touch
        msg.buttons.resize(7);
        msg.buttons[0] = static_cast<int32_t>(trigger_button);
        msg.buttons[1] = static_cast<int32_t>(grip_button);
        msg.buttons[2] = static_cast<int32_t>(primary_button);
        msg.buttons[3] = static_cast<int32_t>(secondary_button);
        msg.buttons[4] = static_cast<int32_t>(menu_button);
        msg.buttons[5] = static_cast<int32_t>(thumbstick_click);
        msg.buttons[6] = static_cast<int32_t>(thumbstick_touch);

        return msg;
    }

} // namespace

PosePublisher::PosePublisher() = default;

bool PosePublisher::initialize(ros::NodeHandle& nh, const PosePublisherConfig& config)
{
    if (initialized_.load())
    {
        ROS_WARN("PosePublisher already initialized");
        return true;
    }

    config_ = config;

    if (!config_.enabled)
    {
        ROS_INFO("PosePublisher disabled by config");
        return true;
    }

    // Create publishers with queue size 1 (latest only)
    pub_hmd_ = nh.advertise<geometry_msgs::PoseStamped>("teleop/pose/hmd", 1);
    pub_left_controller_ = nh.advertise<geometry_msgs::PoseStamped>("teleop/pose/left_controller", 1);
    pub_right_controller_ = nh.advertise<geometry_msgs::PoseStamped>("teleop/pose/right_controller", 1);

    if (config_.publish_euler)
    {
        pub_hmd_euler_ = nh.advertise<geometry_msgs::Vector3Stamped>("teleop/euler/hmd", 1);
        pub_left_controller_euler_ = nh.advertise<geometry_msgs::Vector3Stamped>("teleop/euler/left_controller", 1);
        pub_right_controller_euler_ = nh.advertise<geometry_msgs::Vector3Stamped>("teleop/euler/right_controller", 1);
    }

    if (config_.publish_buttons)
    {
        pub_left_joy_ = nh.advertise<sensor_msgs::Joy>("teleop/controller/left_joy", 1);
        pub_right_joy_ = nh.advertise<sensor_msgs::Joy>("teleop/controller/right_joy", 1);
    }

    if (config_.publish_aim)
    {
        pub_left_aim_ = nh.advertise<geometry_msgs::PoseStamped>("teleop/aim/left", 1);
        pub_right_aim_ = nh.advertise<geometry_msgs::PoseStamped>("teleop/aim/right", 1);
        pub_left_aim_valid_ = nh.advertise<std_msgs::Bool>("teleop/aim/left_valid", 1);
        pub_right_aim_valid_ = nh.advertise<std_msgs::Bool>("teleop/aim/right_valid", 1);
        pub_left_pinch_ = nh.advertise<std_msgs::Bool>("teleop/aim/left_pinch", 1);
        pub_right_pinch_ = nh.advertise<std_msgs::Bool>("teleop/aim/right_pinch", 1);
        pub_left_pinch_strength_ = nh.advertise<std_msgs::Float32>("teleop/aim/left_pinch_strength", 1);
        pub_right_pinch_strength_ = nh.advertise<std_msgs::Float32>("teleop/aim/right_pinch_strength", 1);
    }

    last_warn_time_ = ros::Time::now();
    initialized_.store(true);

    ROS_INFO("PosePublisher initialized: euler=%s buttons=%s aim=%s",
             config_.publish_euler ? "yes" : "no",
             config_.publish_buttons ? "yes" : "no",
             config_.publish_aim ? "yes" : "no");

    return true;
}

void PosePublisher::handleDatagram(const uint8_t* data, size_t size)
{
    if (!initialized_.load() || !config_.enabled)
        return;

    parseAndPublish(data, size);
}

void PosePublisher::parseAndPublish(const uint8_t* data, size_t len)
{
    if (data == nullptr || len < 13)
        return;

    const uint8_t type = data[0];
    if (type != kPoseType)
        return;

    const uint64_t ts_ms = readLeU64(data + 1);
    const uint8_t version = data[9];
    const uint8_t flags = data[10];

    if (version != kProtocolVersion)
    {
        const ros::Time now = ros::Time::now();
        if ((now - last_warn_time_).toSec() > 1.0)
        {
            last_warn_time_ = now;
            ROS_WARN("PosePublisher: unsupported version=%u", static_cast<unsigned>(version));
        }
        return;
    }

    const size_t expected = expectedPacketSize(flags);
    if (len < expected)
    {
        const ros::Time now = ros::Time::now();
        if ((now - last_warn_time_).toSec() > 1.0)
        {
            last_warn_time_ = now;
            ROS_WARN("PosePublisher: short packet len=%zu expected>=%zu flags=0x%02x",
                     len, expected, static_cast<unsigned>(flags));
        }
        return;
    }

    // Convert Unix ms to ROS time
    const ros::Time stamp(static_cast<double>(ts_ms) / 1000.0);

    size_t off = 13;

    auto readPose = [&](geometry_msgs::PoseStamped& out, const std::string& frame_id)
    {
        const float px = readLeF32(data + off + 0);
        const float py = readLeF32(data + off + 4);
        const float pz = readLeF32(data + off + 8);
        const float qx = readLeF32(data + off + 12);
        const float qy = readLeF32(data + off + 16);
        const float qz = readLeF32(data + off + 20);
        const float qw = readLeF32(data + off + 24);
        out = makePoseMsg(frame_id, stamp, px, py, pz, qx, qy, qz, qw);
        off += 28;
    };

    // Read and publish 3 poses
    geometry_msgs::PoseStamped hmd_pose;
    geometry_msgs::PoseStamped left_pose;
    geometry_msgs::PoseStamped right_pose;
    readPose(hmd_pose, config_.frame_id_hmd);
    readPose(left_pose, config_.frame_id_left_controller);
    readPose(right_pose, config_.frame_id_right_controller);

    pub_hmd_.publish(hmd_pose);
    pub_left_controller_.publish(left_pose);
    pub_right_controller_.publish(right_pose);

    // Euler angles (optional)
    if ((flags & kIncludeEuler) && config_.publish_euler)
    {
        auto readEuler = [&](const std::string& frame_id, ros::Publisher& pub)
        {
            const float ex = readLeF32(data + off + 0);
            const float ey = readLeF32(data + off + 4);
            const float ez = readLeF32(data + off + 8);
            off += 12;
            pub.publish(makeEulerMsg(frame_id, stamp, ex, ey, ez));
        };

        readEuler(config_.frame_id_hmd, pub_hmd_euler_);
        readEuler(config_.frame_id_left_controller, pub_left_controller_euler_);
        readEuler(config_.frame_id_right_controller, pub_right_controller_euler_);
    }
    else if (flags & kIncludeEuler)
    {
        off += 36;  // Skip euler data
    }

    // Controller buttons (optional)
    if (flags & kIncludeButtons)
    {
        auto readControllerButtons = [&](ros::Publisher& pub)
        {
            const float trigger_value = readLeF32(data + off + 0);
            const uint8_t trigger_button = data[off + 4];
            const float grip_value = readLeF32(data + off + 5);
            const uint8_t grip_button = data[off + 9];
            const uint8_t primary_button = data[off + 10];
            const uint8_t secondary_button = data[off + 11];
            const uint8_t menu_button = data[off + 12];
            const float thumbstick_x = readLeF32(data + off + 13);
            const float thumbstick_y = readLeF32(data + off + 17);
            const uint8_t thumbstick_click = data[off + 21];
            const uint8_t thumbstick_touch = data[off + 22];
            off += 23;

            if (config_.publish_buttons)
            {
                pub.publish(makeJoyMsg(stamp,
                                       trigger_value,
                                       trigger_button,
                                       grip_value,
                                       grip_button,
                                       primary_button,
                                       secondary_button,
                                       menu_button,
                                       thumbstick_x,
                                       thumbstick_y,
                                       thumbstick_click,
                                       thumbstick_touch));
            }
        };

        readControllerButtons(pub_left_joy_);
        readControllerButtons(pub_right_joy_);
    }

    // Aim poses (optional)
    if (flags & kIncludeAim)
    {
        auto readAim = [&](
            const std::string& frame_id,
            ros::Publisher& pub_pose,
            ros::Publisher& pub_valid,
            ros::Publisher& pub_pinch,
            ros::Publisher& pub_strength)
        {
            const uint8_t valid = data[off + 0];
            const uint8_t pinch = data[off + 1];
            const float pinch_strength = readLeF32(data + off + 2);
            const float px = readLeF32(data + off + 6);
            const float py = readLeF32(data + off + 10);
            const float pz = readLeF32(data + off + 14);
            const float qx = readLeF32(data + off + 18);
            const float qy = readLeF32(data + off + 22);
            const float qz = readLeF32(data + off + 26);
            const float qw = readLeF32(data + off + 30);
            off += 38;

            if (!config_.publish_aim)
                return;

            pub_pose.publish(makePoseMsg(frame_id, stamp, px, py, pz, qx, qy, qz, qw));

            std_msgs::Bool valid_msg;
            valid_msg.data = (valid != 0);
            pub_valid.publish(valid_msg);

            std_msgs::Bool pinch_msg;
            pinch_msg.data = (pinch != 0);
            pub_pinch.publish(pinch_msg);

            std_msgs::Float32 strength_msg;
            strength_msg.data = pinch_strength;
            pub_strength.publish(strength_msg);
        };

        readAim(config_.frame_id_left_aim, pub_left_aim_, pub_left_aim_valid_, pub_left_pinch_, pub_left_pinch_strength_);
        readAim(config_.frame_id_right_aim, pub_right_aim_, pub_right_aim_valid_, pub_right_pinch_, pub_right_pinch_strength_);
    }

    packet_count_.fetch_add(1);
}

} // namespace trb::udp
