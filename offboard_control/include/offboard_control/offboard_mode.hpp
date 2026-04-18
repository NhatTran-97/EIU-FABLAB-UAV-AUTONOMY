#ifndef OFFBOARD_MODE_HPP
#define OFFBOARD_MODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/qos.hpp> 
#include "rmw/qos_profiles.h"
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>

using namespace std::chrono_literals;

class OffboardMode : public rclcpp::Node
{
public:

    OffboardMode();

private:


    // Callback functions
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
    void handle_mode_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void handle_arm_response(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void handle_land_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void handle_disarm_respone(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void control_loop();
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void ext_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
    


    // Helper logic
    void wait_for_fcu_connection();
    void send_initial_setpoints();
    void set_offboard_mode();
    void arm_vehicle();
    void land_vehicle();
    void disarm_drone();
    


    // ROS2 communication interfaces
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;

    // Current state and pose
    geometry_msgs::msg::PoseStamped current_pose_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped target_pose_; 


    rclcpp::Time arm_time;
    rclcpp::Time altitude_reach_time_;

    rclcpp::Time last_arm_request_;
    rclcpp::Time last_mode_request_;
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_state_time_;
    rclcpp::Time takeoff_start_time_;
    rclcpp::Time takeoff_delay_start_time_;
    rclcpp::Duration takeoff_delay_;
    



    bool has_armed;
    bool landing_started_;
    bool landed_;
    bool reached_altitude_;
    double takeoff_height_;
    float battery_valtage_;
    uint8_t landed_state_;
    float limit_min_battery_;
    bool delay_started_;
    uint8_t last_landed_state_;
};

#endif // OFFBOARD_MODE_HPP
