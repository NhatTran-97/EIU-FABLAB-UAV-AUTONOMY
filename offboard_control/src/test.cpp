#include "offboard_control/test.hpp"
#include <cmath>
#include <algorithm>
#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>   // std::getenv — resolve $HOME for the default log path
#include <sstream>   // std::ostringstream — format log lines
#include <iomanip>   // std::setprecision — timestamp / distance formatting

/*
    This program commands the drone to take off to a specified altitude, hover in the air for approximately 10 seconds, and then initiate an automatic landing sequence.

    This program includes the following phases:
    1. FCU connection 
    2. Takeoff delay
    3. Takeoff 
    4. Hover 
    5. Landing 

    Safety checks are included, such as low battery monitoring and altitude timeout detection 

    -----------------------------------------------------------------------------------------
    I. Warm up and Preparation:
    ----------------------------------------------------------------------------------------
        * FCU connection (/mavros/state): Wait for drone to establish a connection with Flight Control Unit (FCU)
        * Initial setpoints: Sends 100 initial setpoints before requesting the OFFBOARD mode, as required by PX4
        * take_off Delay: Introduces a short delay before takeoff to ensure that the system is stable and ready
    
    -------------------------------------------------------------------------------------------------------------
    II. Take_Off and keep position 
    ------------------------------------------------------------------------------------------------------------
        * Arm drone: Sends an arming request to enable motors
        * Switch to OFFBOARD mode: Commands are now received from ROS setpoints instead of manual RC input
        * Reach Target Altitude: Monitors position feedbacks to determine if the target altitude has been reached.
        * Keep position (hover): Once the desired altitude is reached, the drone hovers in place for 10 seconds 
    
    -----------------------------------------------------------------------------------------------------------
    III. Automatic Landing
    -----------------------------------------------------------------------------------------------------------
        * Landing Trigger: If the drone has reached the desired altitude and hovered for 10s, it initiates landing using `AUTO.LAND`.
        * Post-Landing Actions: After successful landing (auto-disarm), the timer is canceled to reduce CPU usage

    -------------------------------------------------------------------------------------------------------------------------------------------------
    IV. Safety Check
    -------------------------------------------------------------------------------------------------------------------------------------------------
        * Altitude Timeout: If the drone does not reach the target altitude within 30 seconds after arming, it triggers an emergency landing.
        * Battery voltage monitoring : If the battery is lower than threshold, emergency landing immediately
        * Checking the lost FCU: If lost with FCU, stop sending setpoint to avoid the dangerous behavior

*/



OffboardMode::OffboardMode() : Node("offboard_node"), last_arm_request_(this->now()),
                                                      last_mode_request_(this->now()),
                                                      takeoff_delay_(rclcpp::Duration::from_seconds(5.0)) ,
                                                      has_armed(false), landing_started_(false), 
                                                      landed_(false),reached_altitude_(false)                                                       
{
   

    this->declare_parameter("takeoff_height_", 5.0);
    this->get_parameter("takeoff_height_", takeoff_height_);
    RCLCPP_INFO(this->get_logger(), "Takeoff height set to: %.2f m", takeoff_height_);

    this->declare_parameter("min_battery_", 14.0);  // 3.5V/cell for 4S — safe low-battery threshold
    this->get_parameter("min_battery_", limit_min_battery_);

    this->declare_parameter("takeoff_delay_sec", 5.0);
    double delay_sec;
    this->get_parameter("takeoff_delay_sec", delay_sec);
    takeoff_delay_ = rclcpp::Duration::from_seconds(delay_sec);   // actually apply the param (was ignored before)

    this->declare_parameter("hover_seconds", 10.0);
    this->get_parameter("hover_seconds", hover_seconds_);

    this->declare_parameter("arm_timeout_sec", 15.0);
    this->get_parameter("arm_timeout_sec", arm_timeout_sec_);

    this->declare_parameter("wp_reach_radius", 1.5);
    this->get_parameter("wp_reach_radius", wp_reach_radius_);

    this->declare_parameter("wp_timeout_sec", 30.0);
    this->get_parameter("wp_timeout_sec", wp_timeout_sec_);

    this->declare_parameter("cruise_speed", 2.0);       // m/s carrot advance along the path
    this->get_parameter("cruise_speed", cruise_speed_);
    this->declare_parameter("carrot_lead", 2.5);        // m max carrot lead over the drone
    this->get_parameter("carrot_lead", carrot_lead_);

    this->declare_parameter("setpoint_rate_hz", 20.0);  // streamer rate to PX4 (must stay >2Hz)
    this->get_parameter("setpoint_rate_hz", setpoint_rate_hz_);

    // Open the mission/flight debug log file (append mode). Default $HOME/mission_debug.log;
    // override with the 'mission_log_file' param. Every event is flushed immediately so a
    // mid-flight power-off still leaves a complete trace to analyse afterwards.
    this->declare_parameter("mission_log_file", "");
    std::string mission_log_path;
    this->get_parameter("mission_log_file", mission_log_path);
    if (mission_log_path.empty())
    {
        const char *home = std::getenv("HOME");
        mission_log_path = std::string(home ? home : "/tmp") + "/mission_debug.log";
    }
    mission_log_.open(mission_log_path, std::ios::out | std::ios::app);
    if (mission_log_.is_open())
        RCLCPP_INFO(this->get_logger(), "📝 Mission debug log -> %s", mission_log_path.c_str());
    else
        RCLCPP_WARN(this->get_logger(), "⚠️ Could not open mission log file: %s", mission_log_path.c_str());
    log_event("================ node start ================");

    delay_started_ = false;
    
    last_landed_state_ = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;



    // QoS Settings for different topics
    rclcpp::QoS qos_state(rclcpp::KeepLast(10));
    qos_state.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_state.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);


    rclcpp::QoS qos_pose(rclcpp::KeepLast(10));
    qos_pose.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pose.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


    rclcpp::QoS qos_pub(rclcpp::KeepLast(10));
    qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::QoS qos_battery(rclcpp::KeepLast(10));
    qos_battery.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_battery.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    rclcpp::QoS qos_lora(rclcpp::KeepLast(10));
    qos_lora.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_lora.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // ROS2 interfaces
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos_state, std::bind(&OffboardMode::state_cb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", qos_pose, std::bind(&OffboardMode::pose_cb, this, std::placeholders::_1));
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", qos_pub);
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", qos_battery, std::bind(&OffboardMode::battery_cb, this, std::placeholders::_1));
    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10, std::bind(&OffboardMode::ext_state_cb, this, std::placeholders::_1));
    
    mode_srv_ = this->create_service<custom_msgs::srv::ModeSignal>("/mode_signal",std::bind(&OffboardMode::on_mode_signal, this, std::placeholders::_1, std::placeholders::_2));

    // Phase 2: receive map waypoints. lora_drone publishes here with RELIABLE +
    // TRANSIENT_LOCAL (latched), so match that QoS to also catch the latest mission.
    mission_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mission_path", qos_lora, std::bind(&OffboardMode::mission_cb, this, std::placeholders::_1));

    
    // Default target (overwritten at takeoff with the drone's current x,y and a RELATIVE
    // altitude, so it no longer depends on where the EKF local-z origin happens to sit).
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = takeoff_height_;

    // Non-blocking startup: do NOT wait for the FCU or pre-stream setpoints in the
    // constructor (that blocks the node, and hangs forever if the FCU never connects).
    // The control loop gates on connection + fresh pose/state, and the takeoff-delay
    // phase primes PX4's setpoint stream before requesting OFFBOARD.

    // Periodic timer (50ms): the control STATE MACHINE (decides target, arm, mode, phases).
    timer_ = this->create_wall_timer(50ms, std::bind(&OffboardMode::control_loop, this));

    // Dedicated thread that PUBLISHES the setpoint @20Hz, separate from the state machine so
    // PX4's >2Hz requirement is met even if the control loop / mission processing is busy.
    setpoint_thread_ = std::thread(&OffboardMode::setpoint_loop, this);
}

OffboardMode::~OffboardMode()
{
    // Stop the streamer thread cleanly so it never publishes into a destroyed node.
    running_.store(false);
    if (setpoint_thread_.joinable())
        setpoint_thread_.join();

    log_event("================ node stop ================");
    if (mission_log_.is_open())
        mission_log_.close();
}

void OffboardMode::log_event(const std::string &msg)
{
    // Thread-safe append of one timestamped line to the debug log, flushed immediately.
    // Called from the control loop, mission_cb, and the service callback — hence the mutex.
    std::lock_guard<std::mutex> lk(mission_log_mtx_);
    if (!mission_log_.is_open())
        return;
    std::ostringstream os;
    os << std::fixed << std::setprecision(3) << this->now().seconds() << "  " << msg;
    mission_log_ << os.str() << "\n";
    mission_log_.flush();
}

void OffboardMode::try_set_realtime_priority()
{
    // Ask the OS to run this thread at real-time (SCHED_FIFO) priority so it is never
    // starved by other ROS work on the CPU-constrained Jetson. We ALWAYS yield via
    // sleep_until each cycle, so a high FIFO priority can't monopolise the core.
    // If the process lacks CAP_SYS_NICE (e.g. unprivileged container) this just fails
    // and we keep normal scheduling — still fine, just less hard a guarantee.
    sched_param sp{};
    sp.sched_priority = 80;   // high, but below typical kernel/IRQ threads
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc == 0)
        RCLCPP_INFO(this->get_logger(),
                    "🧵 Setpoint streamer running at SCHED_FIFO priority %d.", sp.sched_priority);
    else
        RCLCPP_WARN(this->get_logger(),
                    "🧵 Could not set RT priority for setpoint streamer (%s) — "
                    "using normal scheduling. Stream still runs, just without hard preemption.",
                    std::strerror(rc));
}

void OffboardMode::setpoint_loop()
{
    // Stream the current setpoint to PX4 on a dedicated thread. PX4 drops OFFBOARD if
    // setpoints stop arriving (>~0.5s), so isolating this guarantees the stream never
    // stalls because of control-loop / mission work. It only ever reads a COPY of sp_target_.
    try_set_realtime_priority();

    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / (setpoint_rate_hz_ > 0.0 ? setpoint_rate_hz_ : 20.0)));
    auto next = clk::now();

    while (rclcpp::ok() && running_.load())
    {
        next += period;

        if (streaming_.load() && local_pos_pub_)
        {
            geometry_msgs::msg::PoseStamped p;
            { std::lock_guard<std::mutex> lk(sp_mtx_); p = sp_target_; }
            p.header.stamp = this->now();
            local_pos_pub_->publish(p);
            // Heartbeat: lets the control loop watchdog confirm the stream is alive.
            last_setpoint_pub_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
        }

        // Deadline-based pacing: sleep to the NEXT tick (not "now + period"), so the rate
        // stays constant regardless of how long publish took. If we overran a full period
        // the stream was at risk — warn (rate-limited) and resync so we don't spiral.
        const auto t = clk::now();
        if (t < next)
        {
            std::this_thread::sleep_until(next);
        }
        else
        {
            const auto late_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(t - next).count();
            if (late_ms > 100)
                RCLCPP_WARN(this->get_logger(),
                            "⚠️ Setpoint streamer overran by %ldms — stream rate dipped. "
                            "Check CPU load on the companion.", static_cast<long>(late_ms));
            next = t;   // resync schedule to current time
        }
    }
}

void OffboardMode::stream_setpoint(const geometry_msgs::msg::PoseStamped &p)
{
    // Hand the latest target to the streamer thread and make sure it is streaming.
    { std::lock_guard<std::mutex> lk(sp_mtx_); sp_target_ = p; }
    streaming_.store(true);
}

void OffboardMode::mission_cb(const nav_msgs::msg::Path::SharedPtr msg)
{
    // Store incoming waypoints. The control loop picks them up after takeoff completes.
    {
        std::lock_guard<std::mutex> lk(mission_mtx_);
        mission_wps_.assign(msg->poses.begin(), msg->poses.end());
        last_mission_recv_ = this->now();
    }
    RCLCPP_INFO(this->get_logger(), "🗺️  Mission received: %zu waypoints on /mission_path",
                msg->poses.size());
    log_event("MISSION_RX: received " + std::to_string(msg->poses.size()) +
              " waypoints on /mission_path");
    const size_t n = std::min<size_t>(msg->poses.size(), 5);
    for (size_t i = 0; i < n; ++i)
    {
        const auto &pos = msg->poses[i].pose.position;
        RCLCPP_INFO(this->get_logger(), "   WP%zu: x=%.2f y=%.2f z=%.2f", i + 1, pos.x, pos.y, pos.z);
    }
}

void OffboardMode::reset_flight_state()
{
    // Clear every per-flight flag so a fresh OFFBOARD command can fly again without
    // restarting the node.
    delay_started_      = false;
    has_armed           = false;
    reached_altitude_   = false;
    landing_started_    = false;
    landed_             = false;
    takeoff_target_set_ = false;
    offboard_engaged_   = false;
    aborted_            = false;
    low_batt_count_     = 0;
    flying_mission_     = false;
    wp_index_           = 0;
    arm_lost_           = false;
    manual_restored_    = false;
    flight_decision_logged_ = false;
}

void OffboardMode::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_ = *msg;
    last_pose_time_ = this->now();
    pose_received_ = true;
    // Reached-altitude detection moved into control_loop, where it compares against the
    // RELATIVE target (ground z + takeoff_height_) instead of an absolute z.
}

void OffboardMode::on_mode_signal(const std::shared_ptr<custom_msgs::srv::ModeSignal::Request> req, std::shared_ptr<custom_msgs::srv::ModeSignal::Response> res)
{
    using custom_msgs::srv::ModeSignal;

    if (req->mode == ModeSignal::Request::OFFBOARD) {
        // Don't reset state mid-flight — only (re)start when idle / done / aborted.
        if (start_offboard_.load() && !landed_ && !aborted_) {
            res->accepted = true;
            res->msg = "OFFBOARD already in progress";
            RCLCPP_INFO(this->get_logger(), "ModeSignal: OFFBOARD ignored (flight already in progress)");
            return;
        }
        reset_flight_state();          // clear flags so we can fly again without a restart
        start_offboard_.store(true);
        res->accepted = true;
        res->msg = "OFFBOARD sequence started";
        log_event("CMD: OFFBOARD received -> reset + start takeoff sequence");
        RCLCPP_INFO(this->get_logger(), "ModeSignal: OFFBOARD -> reset + start takeoff sequence");
        return;
    }

    if (req->mode == ModeSignal::Request::LAND) {
        this->land_vehicle();
        res->accepted = true;
        res->msg = "LAND requested";
        RCLCPP_INFO(this->get_logger(), "ModeSignal: LAND -> land_vehicle()");
        return;
    }

    res->accepted = false;
    res->msg = "Unknown/none mode";
    RCLCPP_WARN(this->get_logger(), "ModeSignal: Unknown mode");
}



void OffboardMode::state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
    last_state_time_ = this->now();
    state_received_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "🔄 Drone Status: mode = %s | armed = %s", current_state_.mode.c_str(), current_state_.armed ? "true" : "false");
  
}

void OffboardMode::battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    const float v = msg->voltage;

    // Ignore invalid/placeholder readings (0V or NaN before the first valid sample),
    // otherwise "0 < threshold" would latch an emergency landing while on the ground.
    if (!std::isfinite(v) || v <= 1.0f)
    {
        return;
    }
    battery_valtage_ = v;

    // Only emergency-land while actually armed/flying, and require a few consecutive
    // low samples so a single voltage sag doesn't latch an irreversible landing.
    // NOTE: do NOT pre-set landing_started_ here — land_vehicle() sets it itself, and
    // its own `if(landing_started_) return;` guard would otherwise skip the AUTO.LAND.
    if (current_state_.armed && !landing_started_ && v < limit_min_battery_)
    {
        if (++low_batt_count_ >= 3)
        {
            RCLCPP_WARN(this->get_logger(), "⚡ Điện áp pin thấp: %.2fV! Tiến hành hạ cánh khẩn cấp.", v);
            land_vehicle();
        }
    }
    else
    {
        low_batt_count_ = 0;
    }
}


void OffboardMode::ext_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
    landed_state_ = msg->landed_state;

    if (landed_state_ != last_landed_state_)
    {
        last_landed_state_ = landed_state_;  // cập nhật sau khi thay đổi

        if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)
        {
            RCLCPP_INFO(this->get_logger(), "🛬 Drone is on the ground.");
        }
        else if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR)
        {
            RCLCPP_INFO(this->get_logger(), "✈️ Drone is in the air.");
        }
        else if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_TAKEOFF)
        {
            RCLCPP_INFO(this->get_logger(), "🚀 Drone is taking off...");
        }
        else if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING)
        {
            RCLCPP_INFO(this->get_logger(), "🟢 Drone is landing...");
        }
    }
}

void OffboardMode::set_offboard_mode()
{
    /*
    Enter OFFBOARD control mode
    */
    if (current_state_.mode != "OFFBOARD" && (this->now() - last_mode_request_ > 5s))
    {
        auto request_setmode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request_setmode->custom_mode = "OFFBOARD";

        if (set_mode_client_->service_is_ready())
        {
            last_mode_request_ = this->now();   // throttle from SEND time, not response time
            set_mode_client_->async_send_request(request_setmode,
                std::bind(static_cast<void (OffboardMode::*)(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture)>
                (&OffboardMode::handle_mode_response),this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "🟢 Gửi yêu cầu chuyển sang OFFBOARD");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ set_mode service chưa sẵn sàng.");
        }
    }
}

void OffboardMode::set_manual_mode()
{
    // Actively command PX4 back to MANUAL after an abort, so the FC is not left sitting in
    // OFFBOARD with no setpoint stream. SAFETY: only ever called while disarmed/on-ground,
    // and we also guard on !armed here — never yank a flying drone out of OFFBOARD.
    if (current_state_.armed) return;                 // never force-mode a flying drone
    if (current_state_.mode == "MANUAL") return;      // already there
    if ((this->now() - last_mode_request_) < 1s) return;   // throttle (FCU link)

    if (!set_mode_client_->service_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ set_mode service not ready (MANUAL).");
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "MANUAL";
    last_mode_request_ = this->now();
    set_mode_client_->async_send_request(req,
        std::bind(static_cast<void (OffboardMode::*)(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture)>
        (&OffboardMode::handle_mode_response), this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "↩️ Requesting PX4 MANUAL (clean idle after abort).");
}

void OffboardMode::arm_vehicle()
{
    /*
    Start the ARM
    */
    if (!current_state_.armed && (this->now() - last_arm_request_ > 5s))
    {
        auto request_arm_vehicle = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request_arm_vehicle->value = true;

        if (arming_client_->service_is_ready())
        {
            last_arm_request_ = this->now();   // throttle from SEND time so a failed response
                                               // doesn't let the control loop retry every 50ms
            arming_client_->async_send_request(request_arm_vehicle, std::bind(static_cast<void (OffboardMode::*)(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture)>
                                                    (&OffboardMode::handle_arm_response), this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "🟢 Sent ARM request");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ arming service not ready");
        }
    }
}

void OffboardMode::handle_mode_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
{
    try {
        auto response = future.get();
        if (response->mode_sent)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Mode-change request accepted by PX4");
            last_mode_request_ = this->now();
        } else
        {
            RCLCPP_WARN(this->get_logger(), "❌ PX4 rejected mode-change request");
        }
    } catch (const std::exception &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Error in OFFBOARD mode response: %s", e.what());
    }
}


void OffboardMode::handle_arm_response(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
{
    try {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Vehicle armed");
            log_event("ARMED");
            last_arm_request_ = this->now();
            arm_time = this->now();
            has_armed = true;
        } else
        {
            RCLCPP_WARN(this->get_logger(), "❌ Failed to arm vehicle");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error in ARM response: %s", e.what());
    }
}


void OffboardMode::land_vehicle()

{
    
    
    if(landing_started_)  // ✅ Block resend
    {
        return;
    }

    log_event("LAND: switching to AUTO.LAND");

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req -> custom_mode = "AUTO.LAND";

    if(set_mode_client_->service_is_ready())
    {
         set_mode_client_->async_send_request(req, std::bind(&OffboardMode::handle_land_response, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "🔻 Sent request to land drone");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ set_mode service chưa sẵn sàng.");
    }

    landing_started_ = true;
    
}
void OffboardMode::handle_land_response(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
{
    auto response = future.get();
    if (response->mode_sent)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Successfully switched to AUTO.LAND mode");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "❌ Failed to send AUTO.LAND request");
    }
}


void OffboardMode::control_loop()
{
    // Not actively flying (idle / finished / aborted) -> stop the setpoint stream and bail.
    if (!start_offboard_.load() || landed_ || aborted_)
    {
        streaming_.store(false);

        // After an abort, actively return PX4 to MANUAL so it isn't stuck in OFFBOARD with
        // no setpoint stream. Leaves a clean idle state (MANUAL, disarmed) for the next
        // flight. External-override aborts already left OFFBOARD, so this is a no-op there.
        if (aborted_ && !manual_restored_ && state_received_)
        {
            if (current_state_.mode != "OFFBOARD")
            {
                manual_restored_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "✅ PX4 out of OFFBOARD (now '%s') — idle, ready for next flight.",
                            current_state_.mode.c_str());
            }
            else
            {
                set_manual_mode();   // throttled + guarded on !armed inside
            }
        }
        return;
    }

    const auto now = this->now();

    // Phase 0: takeoff delay (also primes PX4's setpoint stream before OFFBOARD)
    if (!delay_started_)
    {
        takeoff_delay_start_time_ = now;
        delay_started_ = true;
        RCLCPP_INFO(this->get_logger(), "⏳ Starting takeoff delay (%.1fs)...", takeoff_delay_.seconds());
        return;
    }

    // Need real data + a live link before commanding anything that moves the drone.
    if (!pose_received_ || !state_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⚠️ Waiting for first pose/state from MAVROS...");
        return;
    }
    if ((now - last_pose_time_).seconds() > 1.0 || (now - last_state_time_).seconds() > 1.5)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⚠️ Pose/state stale (>1s) — holding setpoints off.");
        return;
    }
    if (!current_state_.connected)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "❌ FCU not connected — not commanding.");
        return;
    }

    // During the delay window: hold the CURRENT pose to prime the stream (publishing the
    // live pose avoids a lurch when OFFBOARD engages).
    if ((now - takeoff_delay_start_time_) < takeoff_delay_)
    {
        geometry_msgs::msg::PoseStamped hold = current_pose_;
        stream_setpoint(hold);   // streamer keeps publishing this to prime OFFBOARD
        return;
    }

    // Capture the takeoff target ONCE: hold current x,y; climb to (ground z + height).
    // RELATIVE altitude => robust no matter where the EKF local-z origin sits.
    if (!takeoff_target_set_)
    {
        ground_z_ = current_pose_.pose.position.z;   // WP z values are relative to this
        target_pose_.pose.position.x = current_pose_.pose.position.x;
        target_pose_.pose.position.y = current_pose_.pose.position.y;
        target_pose_.pose.position.z = ground_z_ + takeoff_height_;
        target_pose_.pose.orientation = current_pose_.pose.orientation;
        takeoff_target_set_ = true;
        arm_attempt_start_ = now;
        RCLCPP_INFO(this->get_logger(),
                    "🎯 Takeoff target: x=%.2f y=%.2f z=%.2f (ground z=%.2f + %.2f m)",
                    target_pose_.pose.position.x, target_pose_.pose.position.y,
                    target_pose_.pose.position.z, current_pose_.pose.position.z, takeoff_height_);
    }

    // External override: if PX4 leaves OFFBOARD after we were stably in it, a pilot or
    // failsafe took over — stop fighting for control. "Stably in it" means OFFBOARD was
    // confirmed for at least offboard_stable_sec_ seconds; a brief PX4 revert on the
    // first entry (e.g. while the EKF is still warming up) is NOT treated as an override.
    if (current_state_.mode == "OFFBOARD")
    {
        if (!offboard_engaged_)
        {
            offboard_engaged_ = true;
            offboard_engage_time_ = now;
        }
    }
    else if (offboard_engaged_ && !landing_started_)
    {
        double stable_sec = (now - offboard_engage_time_).seconds();
        if (stable_sec >= offboard_stable_sec_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "🛑 Left OFFBOARD ('%s') after %.1fs stable — external override. Stop.",
                        current_state_.mode.c_str(), stable_sec);
            log_event("ABORT: external override (PX4 left OFFBOARD -> '" +
                      current_state_.mode + "')");
            aborted_ = true;
            return;
        }
        // Brief revert (< offboard_stable_sec_): reset so the NEXT OFFBOARD entry
        // restarts the stability clock fresh instead of accumulating across sessions.
        offboard_engaged_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "⚠️ PX4 left OFFBOARD ('%s') %.1fs after engage — retrying...",
                             current_state_.mode.c_str(), stable_sec);
    }

    // Drive: request OFFBOARD + ARM and keep streaming the target until landing begins.
    if (!landing_started_)
    {
        set_offboard_mode();
        arm_vehicle();
        stream_setpoint(target_pose_);

        // Stream watchdog: while we expect the streamer to be publishing, confirm its
        // heartbeat is fresh. If the dedicated thread ever died or stalled (>250ms with
        // no publish), PX4 is about to drop OFFBOARD — make the failure LOUD instead of
        // silently losing the drone. 250ms is half of PX4's ~0.5s OFFBOARD timeout.
        if (streaming_.load() && current_state_.armed)
        {
            const int64_t last_ns = last_setpoint_pub_ns_.load(std::memory_order_relaxed);
            const double  age_s   = (now.nanoseconds() - last_ns) * 1e-9;
            if (last_ns != 0 && age_s > 0.25)
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "🚨 Setpoint stream STALLED (%.0fms since last publish)! "
                    "PX4 may drop OFFBOARD — companion CPU likely overloaded.", age_s * 1e3);
        }
    }
    else
    {
        streaming_.store(false);   // AUTO.LAND in progress -> stop our setpoint stream
    }

    // Arm-fail timeout: clear feedback instead of retrying forever (usually PX4 pre-arm:
    // no position estimate / sensors / safety switch).
    if (!has_armed && (now - arm_attempt_start_) > rclcpp::Duration::from_seconds(arm_timeout_sec_))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "❌ Failed to ARM within %.0fs — aborting. Check PX4 pre-arm (position estimate / sensors).",
                     arm_timeout_sec_);
        log_event("ABORT: failed to ARM within timeout (PX4 pre-arm: position/sensors?)");
        aborted_ = true;
        return;
    }

    // Lost-arm detection: drone was armed but PX4 disarmed it (COM_DISARM_PRFLT, failsafe…)
    // while we are not intentionally landing. Give arm_timeout_sec_ seconds to re-arm,
    // then abort so the user can re-send OFFBOARD without needing the RC.
    if (has_armed && !current_state_.armed && !landing_started_)
    {
        if (!arm_lost_)
        {
            arm_lost_      = true;
            arm_lost_time_ = now;
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ Drone disarmed unexpectedly (auto-disarm?). %.0fs to re-arm before abort.",
                        arm_timeout_sec_);
        }
        else if ((now - arm_lost_time_) > rclcpp::Duration::from_seconds(arm_timeout_sec_))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ Could not re-ARM within %.0fs — aborting. Send OFFBOARD to retry.",
                         arm_timeout_sec_);
            log_event("ABORT: lost arm mid-flight, could not re-ARM within timeout");
            aborted_ = true;
            return;
        }
    }
    else
    {
        arm_lost_ = false;   // armed again (or landing) — clear the flag
    }

    // Reached-altitude detection against the RELATIVE target.
    if (has_armed && !reached_altitude_ &&
        current_pose_.pose.position.z >= target_pose_.pose.position.z - 0.1)
    {
        reached_altitude_ = true;
        altitude_reach_time_ = now;
        RCLCPP_INFO(this->get_logger(), "✅ Target altitude reached.");
        log_event("ALTITUDE reached -> deciding mission vs hover");
    }

    // Altitude timeout (after ARM): emergency-land if we never climb.
    if (has_armed && !reached_altitude_ && !landing_started_ &&
        (now - arm_time) > rclcpp::Duration(30s))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "🕒 Timeout: altitude not reached within 30s after ARM. Landing...");
        log_event("ABORT: altitude not reached within 30s after ARM -> emergency land");
        land_vehicle();
        return;
    }

    // After reaching altitude: fly mission waypoints if loaded, otherwise pure hover.
    if (has_armed && reached_altitude_ && !landing_started_)
    {
        // On first entry after altitude reached: decide mission vs hover mode.
        if (!flying_mission_)
        {
            std::lock_guard<std::mutex> lk(mission_mtx_);
            if (!mission_wps_.empty())
            {
                // CONSUME the mission: take a private copy to fly, then clear the inbox so a
                // finished mission is NOT replayed on the next plain OFFBOARD (= hover). To
                // fly it again, the ground must re-send the mission.
                active_wps_ = mission_wps_;
                mission_wps_.clear();
                flying_mission_ = true;
                wp_index_       = 0;
                wp_enter_time_  = now;
                // Start the carrot at the drone's current position so it eases out from the
                // hover point instead of jumping straight to the first waypoint.
                carrot_x_ = current_pose_.pose.position.x;
                carrot_y_ = current_pose_.pose.position.y;
                carrot_z_ = current_pose_.pose.position.z;
                flight_decision_logged_ = true;
                log_event("DECISION: FLY MISSION (" + std::to_string(active_wps_.size()) +
                          " waypoints)");
                RCLCPP_INFO(this->get_logger(),
                            "🗺️ Mission start: %zu WPs — cruising %.1f m/s, lead %.1fm",
                            active_wps_.size(), cruise_speed_, carrot_lead_);
            }
            else if (!flight_decision_logged_)
            {
                // At altitude with an EMPTY mission inbox -> we will just hover. This is the
                // exact "mission behaved like a plain OFFBOARD" symptom: the upload never
                // reached us on /mission_path. Log it ONCE, with how long ago (if ever) a
                // mission last arrived, so the cause is unambiguous in the file.
                flight_decision_logged_ = true;
                const double age = (last_mission_recv_.nanoseconds() > 0)
                                       ? (now - last_mission_recv_).seconds() : -1.0;
                std::ostringstream os;
                os << "DECISION: HOVER (no mission on /mission_path) - last mission ";
                if (age < 0.0)
                    os << "NEVER received since node start";
                else
                    os << "received " << std::fixed << std::setprecision(1) << age << "s ago";
                log_event(os.str());
                RCLCPP_WARN(this->get_logger(),
                            "🚁 At altitude but NO mission on /mission_path — hovering only "
                            "(mission upload likely failed).");
            }
        }

        if (flying_mission_)
        {
            // Fly the private copy taken at mission start — stable for the whole flight and
            // independent of any new mission arriving in the inbox (mission_wps_).
            const std::vector<geometry_msgs::msg::PoseStamped> &wps = active_wps_;

            if (wp_index_ >= wps.size())
            {
                // Safety guard (should not happen with a private copy) — land cleanly.
                RCLCPP_WARN(this->get_logger(),
                            "⚠️ WP index %zu out of range — landing.",
                            wp_index_);
                flying_mission_ = false;
                land_vehicle();
                return;
            }

            const auto  &cur  = wps[wp_index_];
            const double wp_x = cur.pose.position.x;
            const double wp_y = cur.pose.position.y;
            const double wp_z = ground_z_ + cur.pose.position.z;   // relative to takeoff EKF z

            // --- Carrot following: advance the setpoint toward the active WP at a fixed
            //     speed, but never let it lead the drone by more than carrot_lead_. PX4 then
            //     chases an always-near, smoothly-moving target => steady speed, smooth
            //     fly-through at corners, gentle stop at the final WP (no hard braking). ---
            const double dt   = 0.05;                 // control-loop period (50 ms timer)
            const double step = cruise_speed_ * dt;   // carrot advance per tick

            double vx = wp_x - carrot_x_;
            double vy = wp_y - carrot_y_;
            double vz = wp_z - carrot_z_;
            double d  = std::sqrt(vx*vx + vy*vy + vz*vz);

            bool carrot_at_wp = false;
            if (d <= step || d < 1e-6)
            {
                carrot_x_ = wp_x; carrot_y_ = wp_y; carrot_z_ = wp_z;
                carrot_at_wp = true;
            }
            else
            {
                carrot_x_ += vx / d * step;
                carrot_y_ += vy / d * step;
                carrot_z_ += vz / d * step;
            }

            // Clamp the carrot so it never leads the drone by more than carrot_lead_.
            double lx = carrot_x_ - current_pose_.pose.position.x;
            double ly = carrot_y_ - current_pose_.pose.position.y;
            double lz = carrot_z_ - current_pose_.pose.position.z;
            double lead = std::sqrt(lx*lx + ly*ly + lz*lz);
            if (lead > carrot_lead_ && lead > 1e-6)
            {
                carrot_x_ = current_pose_.pose.position.x + lx / lead * carrot_lead_;
                carrot_y_ = current_pose_.pose.position.y + ly / lead * carrot_lead_;
                carrot_z_ = current_pose_.pose.position.z + lz / lead * carrot_lead_;
            }

            // Publish the carrot as the position setpoint.
            target_pose_.pose.position.x  = carrot_x_;
            target_pose_.pose.position.y  = carrot_y_;
            target_pose_.pose.position.z  = carrot_z_;
            target_pose_.pose.orientation = cur.pose.orientation;

            // Distance from the DRONE (not the carrot) to the active WP, for arrival logic.
            const double ddx = current_pose_.pose.position.x - wp_x;
            const double ddy = current_pose_.pose.position.y - wp_y;
            const double ddz = current_pose_.pose.position.z - wp_z;
            const double drone_dist = std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);

            const bool timedout = (now - wp_enter_time_) >
                                  rclcpp::Duration::from_seconds(wp_timeout_sec_);
            const bool is_final = (wp_index_ + 1 >= wps.size());

            if (is_final)
            {
                // Final WP: wait for the drone to actually arrive, then land (gentle stop).
                if (drone_dist < wp_reach_radius_ || timedout)
                {
                    if (timedout)
                        RCLCPP_WARN(this->get_logger(),
                                    "⏰ Final WP%zu timeout (%.0fs) — landing anyway.",
                                    wp_index_ + 1, wp_timeout_sec_);
                    else
                        RCLCPP_INFO(this->get_logger(),
                                    "🏁 Mission complete (WP%zu, dist=%.2fm) — landing.",
                                    wp_index_ + 1, drone_dist);
                    log_event(std::string("MISSION COMPLETE at WP") +
                              std::to_string(wp_index_ + 1) +
                              (timedout ? " (timeout)" : "") + " -> landing");
                    flying_mission_ = false;
                    land_vehicle();
                }
            }
            else if (carrot_at_wp || timedout)
            {
                // Intermediate WP: once the carrot has flowed through it (fly-through) or on
                // timeout, advance — the carrot keeps moving toward the next WP (no full stop).
                if (timedout)
                    RCLCPP_WARN(this->get_logger(),
                                "⏰ WP%zu/%zu timeout (%.0fs) — skipping",
                                wp_index_ + 1, wps.size(), wp_timeout_sec_);
                else
                    RCLCPP_INFO(this->get_logger(),
                                "✅ WP%zu/%zu passed (drone %.2fm) → next",
                                wp_index_ + 1, wps.size(), drone_dist);
                log_event(std::string("WP ") + std::to_string(wp_index_ + 1) + "/" +
                          std::to_string(wps.size()) +
                          (timedout ? " TIMEOUT -> skip" : " reached -> next"));
                wp_index_++;
                wp_enter_time_ = now;
            }
        }
        else
        {
            // Pure hover (no mission): auto-land after hover_seconds_. <=0 = hold forever.
            if (hover_seconds_ > 0.0 &&
                (now - altitude_reach_time_) > rclcpp::Duration::from_seconds(hover_seconds_))
            {
                RCLCPP_INFO(this->get_logger(), "🔻 Hover %.0fs done — landing.", hover_seconds_);
                land_vehicle();
            }
        }
    }

    // Landing complete: AUTO.LAND auto-disarms → go idle, ready for the next OFFBOARD.
    // (Timer keeps running — control_loop returns cheaply while idle — so we can re-fly.)
    if (landing_started_ && !current_state_.armed)
    {
        landed_ = true;
        start_offboard_.store(false);
        log_event("LANDED & disarmed -> idle (flight complete)");
        RCLCPP_INFO(this->get_logger(),
                    "✅ Landed & disarmed. Idle — send OFFBOARD again to fly once more.");
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}