#pragma once

#include <chrono>
#include <memory>
#include <fstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();
    ~OffboardControl();

private:
    void state_cb(const mavros_msgs::msg::State & msg);
    void position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void main_loop();
    void log_position();
    void arm_drone();
    void landing();

    void arm_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void landing_cb(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);

    mavros_msgs::msg::State         current_state_;
    geometry_msgs::msg::PoseStamped target_pose_;

    double current_x_{0.0}, current_y_{0.0}, current_z_{0.0};
    double takeoff_height_{5.0};
    double hover_duration_{10.0};
    double altitude_timeout_{60.0};

    bool takeoff_flag_{false};
    bool offboard_flag_{true};
    bool check_armed_{false};
    bool check_landed_{false};
    bool reach_attitude_{false};
    bool landing_flag_{false};
    bool delay_started_{false};

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr         arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr             set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_csv_;

    rclcpp::Time last_pose_time_;
    rclcpp::Time last_state_time_;
    rclcpp::Time attitude_reach_time_;
    rclcpp::Time takeoff_start_time_;
    rclcpp::Time takeoff_delay_start_time_;
    rclcpp::Time last_arm_request_;
    rclcpp::Time csv_start_;
    rclcpp::Duration takeoff_delay_;

    std::ofstream csv_file_;
};
