#include "skydroid/gimbal_node.hpp"
#include <cmath>

/*
ros2 run skydroid gimbal_node --ros-args --params-file \
  ~/nhatbot_remote/install/skydroid/share/skydroid/config/gimbal.yaml

*/

GimbalNode::GimbalNode() : Node("gimbal_node")
{
    this->declare_parameter("sky_ip",      "192.168.144.108");
    this->declare_parameter("local_ip",    "192.168.144.10");
    this->declare_parameter("sky_port",    5000);
    this->declare_parameter("local_port",  5000);
    this->declare_parameter("timeout",     1.0);
    this->declare_parameter("attitude_hz", 10);

    const std::string sky_ip   = this->get_parameter("sky_ip").as_string();
    const std::string local_ip = this->get_parameter("local_ip").as_string();
    const int sky_port         = this->get_parameter("sky_port").as_int();
    const int local_port       = this->get_parameter("local_port").as_int();
    const double timeout       = this->get_parameter("timeout").as_double();
    attitude_hz_               = this->get_parameter("attitude_hz").as_int();

    try {
        driver_ = std::make_unique<SkydroidDriver>(sky_ip, sky_port, local_ip, local_port, timeout);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Failed to init driver: %s", e.what());
        throw;
    }

    auto version = driver_->check_connection();
    if (version) 
    {
        is_connected_     = true;
        firmware_version_ = *version;
        RCLCPP_INFO(get_logger(), "Gimbal connected — Firmware: %s", version->c_str());
    } else 
    {
        is_connected_     = false;
        firmware_version_ = "";
        RCLCPP_WARN(get_logger(), "Gimbal connection check failed, continuing...");
    }

    cmd_sub_ = this->create_subscription<GimbalCommand>(
        "/gimbal_command", 10, std::bind(&GimbalNode::cmd_callback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<GimbalState>("/gimbal_state", 10);

    running_ = true;
    attitude_thread_ = std::thread(&GimbalNode::attitude_loop, this);

    RCLCPP_INFO(get_logger(), "Gimbal node ready — sub: /gimbal_command  pub: /gimbal_state");
}

GimbalNode::~GimbalNode()
{
    running_ = false;
    if (attitude_thread_.joinable())
        attitude_thread_.join();
    driver_->close();
}

void GimbalNode::cmd_callback(const GimbalCommand::SharedPtr msg)
{
    if (msg->control_mode != current_mode_) 
    {
        current_mode_ = msg->control_mode;
        RCLCPP_INFO(get_logger(), "Switched to %s mode", current_mode_ == GimbalCommand::AUTO ? "AUTO" : "MANUAL");
    }
    try 
    {
        execute_command(*msg);
    } catch (const std::exception& e) 
    {
        RCLCPP_ERROR(get_logger(), "cmd_callback error: %s", e.what());
    }
}

void GimbalNode::execute_command(const GimbalCommand& msg)
{
    switch (msg.mode) {
        case GimbalCommand::MODE_PTZ:      handle_ptz(msg.ptz_cmd); break;
        case GimbalCommand::MODE_VELOCITY: handle_velocity(msg);    break;
        case GimbalCommand::MODE_POSITION: handle_position(msg);    break;
        default:
            RCLCPP_WARN(get_logger(), "Unknown gimbal mode: %d", msg.mode);
    }
}

void GimbalNode::handle_ptz(uint8_t ptz_cmd)
{
    switch (ptz_cmd) {
        case GimbalCommand::PTZ_STOP:   driver_->g_ptz("stop");   break;
        case GimbalCommand::PTZ_UP:     driver_->g_ptz("up");     break;
        case GimbalCommand::PTZ_DOWN:   driver_->g_ptz("down");   break;
        case GimbalCommand::PTZ_CENTER: driver_->g_ptz("center"); break;
        case GimbalCommand::PTZ_FOLLOW: driver_->g_ptz("follow"); break;
        case GimbalCommand::PTZ_LOCK:   driver_->g_ptz("lock");   break;
        default:
            RCLCPP_WARN(get_logger(), "Unknown PTZ cmd: %d", ptz_cmd);
    }
}

void GimbalNode::handle_velocity(const GimbalCommand& msg)
{
    int yaw   = static_cast<int>(msg.yaw_vel_dps);
    int pitch = static_cast<int>(msg.pitch_vel_dps);
    driver_->g_speed(yaw, pitch);
    RCLCPP_DEBUG(get_logger(), "Velocity: yaw=%d pitch=%d", yaw, pitch);
}

void GimbalNode::handle_position(const GimbalCommand& msg)
{
    double yaw   = msg.enable_yaw   ? static_cast<double>(msg.yaw_deg)   : NAN;
    double pitch = msg.enable_pitch ? static_cast<double>(msg.pitch_deg) : NAN;

    if (std::isnan(yaw) && std::isnan(pitch)) {
        RCLCPP_WARN(get_logger(), "Position mode: both enable_pitch and enable_yaw are false");
        return;
    }

    double y_spd = msg.yaw_speed_dps   > 0.0f ? static_cast<double>(msg.yaw_speed_dps)   : NAN;
    double p_spd = msg.pitch_speed_dps > 0.0f ? static_cast<double>(msg.pitch_speed_dps) : NAN;

    driver_->g_angle(yaw, pitch, 3.0, y_spd, p_spd);
    RCLCPP_DEBUG(get_logger(), "Position: yaw=%.1f pitch=%.1f", msg.yaw_deg, msg.pitch_deg);
}

void GimbalNode::attitude_loop()
{
    driver_->g_enable_attitude_push(attitude_hz_);
    RCLCPP_INFO(get_logger(), "Attitude push enabled at %d Hz", attitude_hz_);

    while (running_) {
        auto att = driver_->recv_attitude(0.5);

        GimbalState out;
        out.header.stamp     = this->now();
        out.header.frame_id  = "gimbal";
        out.is_connected     = is_connected_;
        out.firmware_version = firmware_version_;
        out.control_mode     = current_mode_;

        if (att) {
            out.yaw_deg   = static_cast<float>(att->yaw_deg);
            out.pitch_deg = static_cast<float>(att->pitch_deg);
            out.roll_deg  = static_cast<float>(att->roll_deg);
        }

        state_pub_->publish(out);
    }

    driver_->g_enable_attitude_push(0);
    RCLCPP_INFO(get_logger(), "Attitude push disabled");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalNode>());
    rclcpp::shutdown();
    return 0;
}
