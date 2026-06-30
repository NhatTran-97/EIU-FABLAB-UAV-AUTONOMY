#ifndef VIO_BRIDGE__VIO_BRIDGE_NODE_HPP_
#define VIO_BRIDGE__VIO_BRIDGE_NODE_HPP_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>


class VioBridge : public rclcpp::Node
{
    public:
        VioBridge();
    
    private:
        // Health-check helpers
        static bool is_finite(const geometry_msgs::msg::Point &p); // drone position
        static bool is_finite(const geometry_msgs::msg::Vector3 &v); // drone velocity
        static bool is_finite(const geometry_msgs::msg::Quaternion &q); // drone orientation
        static double norm(const geometry_msgs::msg::Vector3 &v); // compare with max_velocity_


        void callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void reject(const char* reason);
        void print_diag();

       
        /*
        params
            input_topic_:  /ov_msckf/odomimu from OpenVINS
            output_topic_: /mavros/odometry/out to MAVROS
            out_frame_ : header.frame_id - world frame
            child_frame_ : child_frame_id - body frame 
        */
        std::string input_topic_, output_topic_, out_frame_, child_frame_;
        /*
        health check
            max_velocity_: Maximum velocity if over --> VIO diverge reject (not feed PX4)   ex: 5.0 m/s
            max_jump_: jump range between frames. If over --> diverge --> reject            ex: 0.5 m
        */

        double max_velocity_, max_jump_;
        int warmup_;   // Ignore 10 first frame when VIO init and wait for stable after check

        // state
        geometry_msgs::msg::Point last_pos_;
        bool have_last_ = false;
        uint64_t pass_count_ = 0, reject_count_ = 0, msg_count_ = 0;


        // ROS handles
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr diag_timer_;
};


#endif  // VIO_BRIDGE__VIO_BRIDGE_NODE_HPP_