#ifndef TEST_HPP
#define TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/qos.hpp> 
#include "rmw/qos_profiles.h"
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <custom_msgs/srv/mode_signal.hpp>
#include <nav_msgs/msg/path.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <fstream>
#include <string>

using namespace std::chrono_literals;

class OffboardMode : public rclcpp::Node
{
public:

    OffboardMode();
    ~OffboardMode();

private:


    // Callback functions
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
    void handle_mode_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void handle_arm_response(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void handle_land_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void control_loop();
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void ext_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
    void on_mode_signal(const std::shared_ptr<custom_msgs::srv::ModeSignal::Request> req, std::shared_ptr<custom_msgs::srv::ModeSignal::Response> res); 
    


    // Helper logic
    void set_offboard_mode();
    void set_manual_mode();      // after abort: kick PX4 back to MANUAL for a clean idle
    void arm_vehicle();
    void land_vehicle();
    void reset_flight_state();   // clear all phase flags so a new OFFBOARD can re-fly
    void setpoint_loop();        // dedicated thread: stream the setpoint to PX4 @20Hz
    void try_set_realtime_priority();  // raise the streamer thread to SCHED_FIFO (best-effort)
    void stream_setpoint(const geometry_msgs::msg::PoseStamped& p);  // hand a target to the streamer
    void mission_cb(const nav_msgs::msg::Path::SharedPtr msg);       // receive /mission_path waypoints
    


    // ROS2 communication interfaces
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;


    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;

    rclcpp::Service<custom_msgs::srv::ModeSignal>::SharedPtr mode_srv_;

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
    int low_batt_count_{0};
    rclcpp::Time low_batt_since_{0, 0, RCL_ROS_TIME};  // when V first dropped below threshold (0 = not low)
    double low_batt_hold_sec_{5.0};  // V must stay below threshold this long before emergency land
    // bool start_offboard_;
    std::atomic<bool> start_offboard_{false};

    // --- improved state machine ---
    bool takeoff_target_set_{false};   // takeoff target captured once per flight
    bool offboard_engaged_{false};     // we have actually entered OFFBOARD at least once
    bool aborted_{false};              // external override / arm-fail -> stop commanding
    bool manual_restored_{false};      // PX4 kicked back to MANUAL after an abort
    bool pose_received_{false};
    bool state_received_{false};
    rclcpp::Time arm_attempt_start_;    // when we began trying to arm (for arm-fail timeout)
    rclcpp::Time offboard_engage_time_; // when PX4 first confirmed OFFBOARD (for override debounce)
    double hover_seconds_{10.0};       // hover duration; <=0 => hold indefinitely until LAND
    double arm_timeout_sec_{15.0};     // abort if not armed within this window
    double offboard_stable_sec_{3.0};  // min seconds in OFFBOARD before a revert is "external override"

    // --- dedicated setpoint streamer: guarantees PX4's >2Hz setpoint stream on its OWN
    //     thread, independent of the control loop / mission processing. Only sp_target_ is
    //     shared (small mutex); the rest of the state machine stays single-threaded. ---
    std::thread setpoint_thread_;
    std::mutex sp_mtx_;
    geometry_msgs::msg::PoseStamped sp_target_;   // guarded by sp_mtx_ — what the streamer sends
    std::atomic<bool> streaming_{false};          // stream the setpoint right now?
    std::atomic<bool> running_{true};             // streamer thread keep-alive
    double setpoint_rate_hz_{20.0};               // streamer publish rate (PX4 needs >2Hz)
    std::atomic<int64_t> last_setpoint_pub_ns_{0}; // heartbeat: ns of last setpoint publish

    // --- Phase 2: mission waypoints from /mission_path ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mission_sub_;
    std::vector<geometry_msgs::msg::PoseStamped> mission_wps_;   // inbox: latest mission, guarded by mission_mtx_
    std::mutex mission_mtx_;
    rclcpp::Time last_mission_recv_;
    // The mission currently being flown. Taken (and mission_wps_ cleared) once at takeoff so a
    // finished mission is NOT replayed on the next plain OFFBOARD — only control_loop touches it.
    std::vector<geometry_msgs::msg::PoseStamped> active_wps_;

    bool arm_lost_{false};          // true once we detect unexpected disarm mid-flight
    rclcpp::Time arm_lost_time_;    // when the unexpected disarm was first detected

    // --- Phase 2: waypoint-following runtime state ---
    bool flying_mission_{false};    // currently executing mission waypoints
    size_t wp_index_{0};            // index of the WP we are flying toward
    double wp_reach_radius_{1.5};   // metres: sphere within which a WP is "reached"
    double ground_z_{0.0};          // EKF local-z at takeoff; WP z values are relative to this
    rclcpp::Time wp_enter_time_;    // when we started targeting wp_index_
    double wp_timeout_sec_{30.0};   // skip WP and advance if not reached within this time

    // --- Carrot (moving-setpoint) following: smooth, speed-limited waypoint flight ---
    // Instead of commanding the far WP (PX4 sprints then brakes hard at corners), advance a
    // "carrot" setpoint toward the WP at a fixed speed, never leading the drone by more than
    // carrot_lead_. The drone chases an always-near, smoothly-moving target.
    double cruise_speed_{2.5};      // m/s: how fast the carrot advances along the path
    double carrot_lead_{2.5};       // m: max distance the carrot may lead the drone (anti-sprint)
    double carrot_x_{0.0};          // current carrot position (EKF frame; z already absolute)
    double carrot_y_{0.0};
    double carrot_z_{0.0};

    // --- Debug file logging: append timestamped mission/flight events to a file for offline
    //     analysis (separate from the ROS console). Opened in the constructor, flushed each
    //     write so a power-off still leaves a complete trace. Path = 'mission_log_file' param,
    //     default $HOME/mission_debug.log. ---
    void log_event(const std::string &msg);     // thread-safe: timestamp + msg -> mission_log_
    std::ofstream mission_log_;
    std::mutex mission_log_mtx_;
    bool flight_decision_logged_{false};         // one-shot: log the mission-vs-hover decision once

};

#endif // OFFBOARD_MODE_HPP