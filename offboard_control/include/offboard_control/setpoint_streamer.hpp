#ifndef OFFBOARD_CONTROL_SETPOINT_STREAMER_HPP
#define OFFBOARD_CONTROL_SETPOINT_STREAMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// Streams the current position setpoint to PX4 on a DEDICATED thread, isolated from the
// control loop / mission processing. PX4 drops OFFBOARD if setpoints stop arriving (>~0.5s),
// so this thread guarantees the >2Hz stream regardless of how busy the rest of the node is.
// It only ever reads a COPY of the target (small mutex), so the state machine stays
// single-threaded. The thread tries to run at SCHED_FIFO priority (best-effort).
//
// Unlike CarrotFollower this is infrastructure, not pure logic — it owns a ROS publisher and
// uses the node's clock/logger. The owning node feeds it the target and toggles streaming.
class SetpointStreamer
{
public:
    SetpointStreamer(rclcpp::Node *node,
                     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                     double rate_hz);
    ~SetpointStreamer();

    void start();                          // launch the streaming thread
    void stop();                           // signal + join the thread (idempotent)

    // Hand the latest target to the thread and start streaming it.
    void set_target(const geometry_msgs::msg::PoseStamped &p);
    void set_streaming(bool on);           // enable/disable publishing without changing target
    bool streaming() const { return streaming_.load(); }

    // Heartbeat: ns (node clock) of the last publish, for the control loop's stall watchdog.
    int64_t last_publish_ns() const { return last_pub_ns_.load(std::memory_order_relaxed); }

private:
    void loop();
    void try_set_realtime_priority();

    rclcpp::Node *node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    double rate_hz_;

    std::thread thread_;
    std::mutex mtx_;
    geometry_msgs::msg::PoseStamped target_;     // guarded by mtx_ — what the thread sends
    std::atomic<bool> streaming_{false};         // publish right now?
    std::atomic<bool> running_{true};            // thread keep-alive
    std::atomic<int64_t> last_pub_ns_{0};        // heartbeat
};

#endif // OFFBOARD_CONTROL_SETPOINT_STREAMER_HPP
