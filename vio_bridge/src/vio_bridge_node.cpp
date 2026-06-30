#include "vio_bridge/vio_bridge_node.hpp"  
#include <cmath>

VioBridge::VioBridge() : Node("vio_bridge")
{

    // parameters
    input_topic_ = declare_parameter("input_topic", std::string("/ov_msckf/odomimu"));
    output_topic_ = declare_parameter("output_topic", std::string("/mavros/odometry/out"));
    out_frame_ = declare_parameter("frame_id", std::string("odom"));
    child_frame_ = declare_parameter("child_frame_id", std::string("base_link"));
    max_velocity_ = declare_parameter("max_velocity", 5.0 ); // m/s  -> nghi diverge
    max_jump_ = declare_parameter("max_jump", 0.5 );  // m giua 2 frame -> nghi diverge
    warmup_ = declare_parameter("warmup_msgs",10 ); // bo qua jump-check cho N msg dau

    rclcpp::QoS qos(rclcpp::KeepLast(20));

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(input_topic_, qos, 
                                                        std::bind(&VioBridge::callback, this, 
                                                                        std::placeholders::_1));
    diag_timer_ = create_wall_timer(std::chrono::seconds(5),
                                                        std::bind(&VioBridge::print_diag, this));


    RCLCPP_INFO(get_logger(), "VIO Bridge: %s --> %s", input_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Health limits: max_velocity=%.1f m/s, max_jump=%.2f m", max_velocity_, max_jump_);     

}

bool VioBridge::is_finite(const geometry_msgs::msg::Point &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
bool VioBridge::is_finite(const geometry_msgs::msg::Vector3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);

}
 bool VioBridge::is_finite(const geometry_msgs::msg::Quaternion &q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}
double VioBridge::norm(const geometry_msgs::msg::Vector3 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

void VioBridge::callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto &pos = msg->pose.pose.position;
    const auto &ori = msg->pose.pose.orientation;
    const auto &lin = msg->twist.twist.linear;

    // 1) NAN/ Inf -> reject
    if(!is_finite(pos) || !is_finite(ori) || !is_finite(lin))
    {
        reject("non-finite pose/twist");
        return;
    }

    // 2) Velocity spike -> reject (dau hieu diverge)
    double v = norm(lin);
    if(v > max_velocity_)
    {
        reject("velocity spike");
        return;
    }

    // 3) Position jump vetween two frames --> reject

    if(have_last_ && msg_count_ > static_cast<uint64_t>(warmup_))
    {
        double dx = pos.x - last_pos_.x;
        double dy = pos.y - last_pos_.y;
        double dz = pos.z - last_pos_.z;
        double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
        if(jump > max_jump_)
        {
            reject("position jump");
            return;
        }
    }

    // --- Healthy -> republish ---
    auto out = *msg; // copy giu nguyen pose + twist + covariance
    out.header.frame_id = out_frame_; // ENU world (MAVROS)
    out.child_frame_id = child_frame_; // body frame

    // TODO

    pub_->publish(out);
    last_pos_ = pos;
    have_last_ = true;
    pass_count_++;
    msg_count_++;


}


void VioBridge::reject(const char* reason)
{
    reject_count_ ++;
    have_last_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                                    "VIO rejected (%s) -> Not feeding PX4 (failsafe takes over)", reason);
}
void VioBridge::print_diag()
{
    uint64_t total = pass_count_ + reject_count_;
    if(total == 0)
    {
        RCLCPP_INFO(get_logger(), "[DIAG] waiting for VIO odometry on %s ...", input_topic_.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "[DIAG] fed=%lu rejected=%lu (%.1f%% healthy)",
                pass_count_, reject_count_, 100.0 * pass_count_ / total);
    
    pass_count_ = 0;
    reject_count_ = 0;

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioBridge>());
    rclcpp::shutdown();
}
