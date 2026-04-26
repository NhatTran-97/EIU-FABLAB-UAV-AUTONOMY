#pragma once

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <skydroid_msgs/msg/gimbal_state.hpp>

class GimbalController;

class RosGimbalStateBridge
{
public:
    explicit RosGimbalStateBridge(GimbalController *controller);
    ~RosGimbalStateBridge();

private:
    void onState(const skydroid_msgs::msg::GimbalState::SharedPtr msg);

    GimbalController *m_controller;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<skydroid_msgs::msg::GimbalState>::SharedPtr m_stateSub;
    rclcpp::executors::SingleThreadedExecutor m_executor;
    std::thread m_spinThread;
};
