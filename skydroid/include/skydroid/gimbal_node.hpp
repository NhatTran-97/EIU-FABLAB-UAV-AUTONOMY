#pragma once

#include <rclcpp/rclcpp.hpp>
#include <skydroid_msgs/msg/gimbal_command.hpp>
#include <skydroid_msgs/msg/gimbal_state.hpp>
#include "skydroid/skydroid_driver.hpp"

#include <thread>
#include <atomic>
#include <memory>
#include <string>

class GimbalNode : public rclcpp::Node {
    using GimbalCommand = skydroid_msgs::msg::GimbalCommand;
    using GimbalState   = skydroid_msgs::msg::GimbalState;

public:
    GimbalNode();
    ~GimbalNode();

private:
    std::unique_ptr<SkydroidDriver> driver_;

    rclcpp::Subscription<GimbalCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<GimbalState>::SharedPtr      state_pub_;

    std::thread       attitude_thread_;
    std::atomic<bool> running_{false};

    int         attitude_hz_{10};
    bool        is_connected_{false};
    std::string firmware_version_;
    uint8_t     current_mode_{GimbalCommand::MANUAL};   // MANUAL mặc định

    void cmd_callback(const GimbalCommand::SharedPtr msg);
    void execute_command(const GimbalCommand& msg);
    void handle_ptz(uint8_t ptz_cmd);
    void handle_velocity(const GimbalCommand& msg);
    void handle_position(const GimbalCommand& msg);
    void attitude_loop();
};
