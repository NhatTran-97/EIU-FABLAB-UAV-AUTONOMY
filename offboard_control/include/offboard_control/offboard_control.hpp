#ifndef OFFBOARD_CONTROL_HPP
#define OFFBOARD_CONTROL_HPP

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
#include "offboard_control/carrot_follower.hpp"
#include "offboard_control/setpoint_streamer.hpp"
#include "offboard_control/battery_monitor.hpp"
#include <memory>
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
    


    // --- Explicit flight state machine (incremental migration) -------------------------
    // state_ is the single source of truth for "which flight phase we are in". STEP 1: it is
    // written alongside the legacy boolean flags at each transition and currently only drives
    // logging (no control branches on it yet), so behaviour is unchanged. Later steps will
    // dispatch control_loop() on state_ and retire the redundant flags one by one.
    enum class FlightState {
        IDLE,            // waiting for an OFFBOARD command
        WAIT_LINK,       // OFFBOARD requested; waiting for fresh pose/state + FCU link
        TAKEOFF_DELAY,   // priming PX4's setpoint stream before engaging OFFBOARD
        ARMING,          // target altitude captured; requesting OFFBOARD + ARM
        CLIMBING,        // armed; climbing toward takeoff altitude
        HOVER,           // at altitude with no mission loaded — holding position
        MISSION,         // flying waypoints
        LANDING,         // AUTO.LAND in progress
        ABORT            // external override / arm-fail — returning PX4 to MANUAL
    };
    FlightState state_{FlightState::IDLE};
    void transition_to(FlightState s);           // single choke point: log entry/exit + set state_
    static const char *state_name(FlightState s);

    // --- control_loop() phase helpers (STEP 2 refactor) --------------------------------
    // control_loop() is now a thin dispatcher that calls these in order. Each helper is a
    // verbatim extraction of one phase of the old monolithic loop — same code, same order,
    // no behaviour change. Helpers returning bool report "tick is done, control_loop should
    // return" (true) so the early-returns of the old single function are preserved.
    bool handle_inactive();                              // idle/finished/aborted: stop stream, maybe restore MANUAL
    bool begin_takeoff_delay(const rclcpp::Time &now);   // phase 0: start the takeoff-delay window
    bool link_ready(const rclcpp::Time &now);            // gate: fresh pose/state + connected FCU
    bool prime_during_delay(const rclcpp::Time &now);    // hold current pose to prime PX4's setpoint stream
    void capture_takeoff_target(const rclcpp::Time &now);// latch takeoff target once (ground_z_ + height)
    bool check_external_override(const rclcpp::Time &now);// PX4 left OFFBOARD after stable -> abort
    void drive_and_stream(const rclcpp::Time &now);      // request OFFBOARD + ARM, stream target, stream watchdog
    bool check_arm_watchdogs(const rclcpp::Time &now);   // arm-fail timeout + lost-arm detection -> abort
    bool check_altitude_progress(const rclcpp::Time &now);// reached-altitude detection + altitude timeout
    bool run_mission_or_hover(const rclcpp::Time &now);  // post-altitude: decide + fly mission, or hover
    void decide_mission_vs_hover(const rclcpp::Time &now);// one-shot: consume mission inbox or commit to hover
    bool fly_active_mission(const rclcpp::Time &now);    // carrot-follow the active waypoints
    void finalize_if_landed();                           // landed + disarmed -> go idle

    // Helper logic
    void set_offboard_mode();
    void set_manual_mode();      // after abort: kick PX4 back to MANUAL for a clean idle
    void arm_vehicle();
    void land_vehicle();
    void reset_flight_state();   // clear all phase flags so a new OFFBOARD can re-fly
    void stream_setpoint(const geometry_msgs::msg::PoseStamped& p);  // hand a target to the streamer (-> streamer_)
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
    uint8_t landed_state_;
    bool delay_started_;
    uint8_t last_landed_state_;

    // Battery safety debounce (pure, testable — see battery_monitor.hpp). Holds the danger
    // threshold + sustained-low timer; battery_cb feeds it samples and acts on the verdict.
    BatteryMonitor battery_;
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

    // --- dedicated setpoint streamer (own thread @ >2Hz, see setpoint_streamer.hpp). Owns the
    //     thread, RT priority, target mutex and heartbeat; the node just feeds it a target. ---
    std::unique_ptr<SetpointStreamer> streamer_;
    double setpoint_rate_hz_{20.0};               // streamer publish rate (PX4 needs >2Hz)

    // --- Phase 2: mission waypoints from /mission_path ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mission_sub_;
    std::vector<geometry_msgs::msg::PoseStamped> mission_wps_;   // inbox: latest mission, guarded by mission_mtx_
    std::mutex mission_mtx_;
    rclcpp::Time last_mission_recv_;
    // Waypoint-following algorithm (pure geometry, no ROS — see carrot_follower.hpp). Loaded
    // with the mission at takeoff (decide_mission_vs_hover) and advanced each tick in
    // fly_active_mission(). Owns the active waypoints + carrot state.
    CarrotFollower follower_;

    // Mission = an explicit GO trigger. mission_intent_ is true ONLY for a flight started by a
    // freshly-received mission (mission_cb auto-starts it); an OFFBOARD command leaves it false
    // (plain takeoff/hover/land). decide_mission_vs_hover() flies waypoints only when true, so a
    // stale/latched mission can never turn an OFFBOARD-hover into a mission flight.
    bool mission_intent_{false};

    bool arm_lost_{false};          // true once we detect unexpected disarm mid-flight
    rclcpp::Time arm_lost_time_;    // when the unexpected disarm was first detected

    // --- Waypoint-following runtime state (carrot/WP geometry now lives in follower_) ---
    // (the old flying_mission_ flag was retired in STEP 3 — state_ == MISSION is the source
    //  of truth for "currently executing waypoints".)
    double ground_z_{0.0};          // EKF local-z at takeoff; WP z values are relative to this
    rclcpp::Time wp_enter_time_;    // when the current WP became active (for the wp_timeout)

    // --- Debug file logging: append timestamped mission/flight events to a file for offline
    //     analysis (separate from the ROS console). Opened in the constructor, flushed each
    //     write so a power-off still leaves a complete trace. Path = 'mission_log_file' param,
    //     default $HOME/mission_debug.log. ---
    void log_event(const std::string &msg);     // thread-safe: timestamp + msg -> mission_log_
    std::ofstream mission_log_;
    std::mutex mission_log_mtx_;
    bool flight_decision_logged_{false};         // one-shot: log the mission-vs-hover decision once

};

#endif // OFFBOARD_CONTROL_HPP