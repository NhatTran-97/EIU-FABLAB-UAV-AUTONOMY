#include "offboard_control/take_off.hpp"


TakeOff::TakeOff() : Node("offboard_node")
{


    this->declare_parameter("takeoff_height_", 5.0);
    this->get_parameter("takeoff_height_", takeoff_height_);



    rclcpp::QoS qos_pub(rclcpp::KeepLast(10));
    qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::QoS qos_state(rclcpp::KeepLast(10));
    qos_state.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_state.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);


    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", qos_pub);
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos_state, std::bind(&TakeOff::state_cb, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(50ms, std::bind(&TakeOff::control_loop, this));



    // First Initialization for Drone  

    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = takeoff_height_;


    // wait for FCU connection
    this->wait_for_fcu_connection();

    // Send a few initial setpoints before switching to OFFBOARD mode
    this->send_initial_setpoints();


}


void TakeOff::wait_for_fcu_connection()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(this->get_node_base_interface());  
        rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "FCU connected");
}

void TakeOff::send_initial_setpoints()
{
    RCLCPP_INFO(this->get_logger(), "Sending initial setpoints");
    for (int i = 100; rclcpp::ok() && i > 0; --i)
    {
        local_pos_pub_->publish(target_pose_);
        rclcpp::sleep_for(50ms);
    }
}


void TakeOff::state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
    last_state_time_ = this->now();  

    RCLCPP_INFO(this->get_logger(), "last_state_time: %.3f", last_state_time_.seconds());

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "🔄 Drone Status: mode = %s | armed = %s", current_state_.mode.c_str(), current_state_.armed ? "true" : "false");
}


void TakeOff::control_loop()
{

}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TakeOff>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}