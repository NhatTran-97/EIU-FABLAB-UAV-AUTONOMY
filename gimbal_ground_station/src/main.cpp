#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "gimbal_ground_station/serial_bridge.hpp"
#include "gimbal_ground_station/gimbal_controller.hpp"
#include "gimbal_ground_station/ros_gimbal_state_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QGuiApplication app(argc, argv);
    app.setApplicationName("Gimbal Ground Station");
    app.setApplicationVersion("1.0.0");

    SerialBridge     serialBridge;
    GimbalController gimbalController(&serialBridge);
    RosGimbalStateBridge rosBridge(&gimbalController);

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("serialBridge",     &serialBridge);
    engine.rootContext()->setContextProperty("gimbalController", &gimbalController);

    engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));
    if (engine.rootObjects().isEmpty()) {
        rclcpp::shutdown();
        return -1;
    }

    const int rc = app.exec();
    rclcpp::shutdown();
    return rc;
}
