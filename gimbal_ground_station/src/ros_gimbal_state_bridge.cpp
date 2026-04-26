#include "gimbal_ground_station/ros_gimbal_state_bridge.hpp"

#include <QMetaObject>

#include "gimbal_ground_station/gimbal_controller.hpp"

RosGimbalStateBridge::RosGimbalStateBridge(GimbalController *controller)
    : m_controller(controller)
{
    m_node = std::make_shared<rclcpp::Node>("drone_ground_station_bridge");

    m_stateSub = m_node->create_subscription<skydroid_msgs::msg::GimbalState>(
        "/gimbal_state", 10,
        [this](const skydroid_msgs::msg::GimbalState::SharedPtr msg) {
            onState(msg);
        });

    m_executor.add_node(m_node);
    m_spinThread = std::thread([this]() {
        m_executor.spin();
    });
}

RosGimbalStateBridge::~RosGimbalStateBridge()
{
    m_executor.cancel();
    if (m_spinThread.joinable())
        m_spinThread.join();
}

void RosGimbalStateBridge::onState(const skydroid_msgs::msg::GimbalState::SharedPtr msg)
{
    QMetaObject::invokeMethod(
        m_controller,
        "setFeedbackState",
        Qt::QueuedConnection,
        Q_ARG(float, msg->yaw_deg),
        Q_ARG(float, msg->pitch_deg),
        Q_ARG(float, msg->roll_deg),
        Q_ARG(bool, msg->is_connected));
}
