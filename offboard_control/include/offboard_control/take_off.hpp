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


class TakeOff : public rclcpp::Node
{
public:

    TakeOff();
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);

private:
    void control_loop();

    void wait_for_fcu_connection();
    void send_initial_setpoints();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped target_pose_; 

    rclcpp::Time last_state_time_;



    double takeoff_height_;
};


#endif 