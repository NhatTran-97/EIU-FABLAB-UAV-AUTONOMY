#include "offboard_control/offboard_control.hpp"
#include <cmath>
#include <algorithm>
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

    this->declare_parameter("min_battery_", 13.2);  // 3.3V/cell for 4S — real danger zone (14V sags under motor load)
    this->get_parameter("min_battery_", battery_.params.min_voltage);

    // Voltage must stay below min_battery_ continuously for this long before we emergency-land.
    // Rejects the transient sag during takeoff/climb (high current, recovers in 1-3s) while
    // still catching a genuinely depleted pack.
    this->declare_parameter("low_batt_hold_sec", 5.0);
    this->get_parameter("low_batt_hold_sec", battery_.params.hold_sec);

    this->declare_parameter("takeoff_delay_sec", 5.0);
    double delay_sec;
    this->get_parameter("takeoff_delay_sec", delay_sec);
    takeoff_delay_ = rclcpp::Duration::from_seconds(delay_sec);   // actually apply the param (was ignored before)

    this->declare_parameter("hover_seconds", 10.0);
    this->get_parameter("hover_seconds", hover_seconds_);

    this->declare_parameter("arm_timeout_sec", 15.0);
    this->get_parameter("arm_timeout_sec", arm_timeout_sec_);

    this->declare_parameter("wp_reach_radius", 1.5);
    this->get_parameter("wp_reach_radius", follower_.params.reach_radius);

    this->declare_parameter("wp_timeout_sec", 30.0);
    this->get_parameter("wp_timeout_sec", follower_.params.wp_timeout);

    this->declare_parameter("cruise_speed", 2.0);       // m/s carrot advance along the path
    this->get_parameter("cruise_speed", follower_.params.cruise_speed);
    this->declare_parameter("carrot_lead", 2.5);        // m max carrot lead over the drone
    this->get_parameter("carrot_lead", follower_.params.carrot_lead);

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
    
    // Mission inbox QoS: RELIABLE for delivery, but VOLATILE (NOT transient-local). A mission
    // is a GO trigger (it auto-starts a flight), so we must NOT receive lora_drone's latched
    // mission history when we subscribe at startup — replaying an old mission would launch the
    // drone unexpectedly. VOLATILE = act only on missions published while we are alive.
    rclcpp::QoS qos_mission(rclcpp::KeepLast(10));
    qos_mission.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_mission.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // ROS2 interfaces
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos_state, std::bind(&OffboardMode::state_cb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", qos_pose, std::bind(&OffboardMode::pose_cb, this, std::placeholders::_1));
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", qos_pub);
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", qos_battery, std::bind(&OffboardMode::battery_cb, this, std::placeholders::_1));
    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10, std::bind(&OffboardMode::ext_state_cb, this, std::placeholders::_1));
    
    mode_srv_ = this->create_service<custom_msgs::srv::ModeSignal>("/mode_signal",std::bind(&OffboardMode::on_mode_signal, this, std::placeholders::_1, std::placeholders::_2));

    // Phase 2: receive map waypoints. A fresh mission auto-starts a mission flight (see
    // mission_cb). VOLATILE QoS (qos_mission) ensures we ignore any latched mission from a
    // previous session and only act on missions sent while we are running.
    mission_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mission_path", qos_mission, std::bind(&OffboardMode::mission_cb, this, std::placeholders::_1));

    
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

    // Dedicated thread that PUBLISHES the setpoint @>2Hz, separate from the state machine so
    // PX4's requirement is met even if the control loop / mission processing is busy.
    streamer_ = std::make_unique<SetpointStreamer>(this, local_pos_pub_, setpoint_rate_hz_);
    streamer_->start();
}

OffboardMode::~OffboardMode()
{
    // Stop the streamer thread cleanly so it never publishes into a destroyed node.
    if (streamer_)
        streamer_->stop();

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

void OffboardMode::stream_setpoint(const geometry_msgs::msg::PoseStamped &p)
{
    // Hand the latest target to the streamer thread and make sure it is streaming.
    streamer_->set_target(p);
}

void OffboardMode::mission_cb(const nav_msgs::msg::Path::SharedPtr msg)
{
    // Ignore an empty path (some publishers send one to "clear" a mission).
    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "🗺️  Empty mission on /mission_path — ignored.");
        return;
    }

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

    // A mission is its own GO trigger: if we are idle (on the ground, not already flying),
    // auto-start the takeoff sequence in MISSION intent. A separate OFFBOARD command instead
    // does a plain takeoff/hover/land. We only trigger from idle (same condition as
    // handle_inactive) so a mission arriving mid-flight can never hijack an active flight.
    if (!start_offboard_.load() || landed_ || aborted_)
    {
        reset_flight_state();
        mission_intent_ = true;          // this flight will fly the mission, not hover
        start_offboard_.store(true);
        transition_to(FlightState::WAIT_LINK);
        log_event("CMD: MISSION received -> auto-start mission flight");
        RCLCPP_INFO(this->get_logger(),
                    "🚀 Mission received while idle — auto-starting mission flight.");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                    "🗺️  Mission received during an active flight — stored only, NOT "
                    "auto-started (land first).");
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
    battery_.reset();
    arm_lost_           = false;
    manual_restored_    = false;
    flight_decision_logged_ = false;
    mission_intent_     = false;   // default to plain hover; mission_cb sets it true for a mission
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
        transition_to(FlightState::WAIT_LINK);
        res->accepted = true;
        res->msg = "OFFBOARD sequence started";
        log_event("CMD: OFFBOARD received -> takeoff / hover / land (no mission)");
        RCLCPP_INFO(this->get_logger(), "ModeSignal: OFFBOARD -> takeoff / hover / land (no mission)");
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
    // Feed the sample to the pure debounce monitor (battery_monitor.hpp); it owns the
    // sustained-low timer and filters invalid readings. We just act on the verdict.
    // NOTE: do NOT pre-set landing_started_ — land_vehicle() sets it itself; on the next
    // sample the monitor's gate closes and its timer resets.
    const double v = msg->voltage;
    const auto verdict = battery_.update(v, current_state_.armed, landing_started_,
                                         this->now().seconds());

    if (verdict == BatteryMonitor::Verdict::WATCH_STARTED)
    {
        RCLCPP_WARN(this->get_logger(),
                    "⚠️ Điện áp %.2fV < %.2fV — theo dõi %.1fs trước khi hạ cánh (lọc sụt áp tạm thời).",
                    v, battery_.params.min_voltage, battery_.params.hold_sec);
        std::ostringstream bos;
        bos << "BATTERY: V=" << std::fixed << std::setprecision(2) << v
            << " dropped below " << battery_.params.min_voltage << "V — starting "
            << battery_.params.hold_sec << "s sustained-low watch";
        log_event(bos.str());
    }
    else if (verdict == BatteryMonitor::Verdict::EMERGENCY_LAND)
    {
        RCLCPP_WARN(this->get_logger(), "⚡ Điện áp pin thấp BỀN VỮNG: %.2fV! Tiến hành hạ cánh khẩn cấp.", v);
        std::ostringstream bos;
        bos << "BATTERY: emergency land — V=" << std::fixed << std::setprecision(2) << v
            << "V stayed below " << battery_.params.min_voltage << "V for "
            << battery_.params.hold_sec << "s (sustained, not a transient sag)";
        log_event(bos.str());
        land_vehicle();
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
            transition_to(FlightState::CLIMBING);
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

    transition_to(FlightState::LANDING);
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


const char *OffboardMode::state_name(FlightState s)
{
    switch (s) {
        case FlightState::IDLE:          return "IDLE";
        case FlightState::WAIT_LINK:     return "WAIT_LINK";
        case FlightState::TAKEOFF_DELAY: return "TAKEOFF_DELAY";
        case FlightState::ARMING:        return "ARMING";
        case FlightState::CLIMBING:      return "CLIMBING";
        case FlightState::HOVER:         return "HOVER";
        case FlightState::MISSION:       return "MISSION";
        case FlightState::LANDING:       return "LANDING";
        case FlightState::ABORT:         return "ABORT";
    }
    return "?";
}

// Single choke point for every state change: log the transition (console + file) and update
// state_. Guarded so re-entering the same state is a no-op (no log spam from the 20Hz loop).
void OffboardMode::transition_to(FlightState s)
{
    if (s == state_) return;
    RCLCPP_INFO(this->get_logger(), "🔀 STATE %s -> %s", state_name(state_), state_name(s));
    log_event(std::string("STATE: ") + state_name(state_) + " -> " + state_name(s));
    state_ = s;
}

void OffboardMode::control_loop()
{
    // STEP 2: control_loop() is a thin dispatcher. Each helper below is a verbatim
    // extraction of one phase of the old monolithic loop, called in the SAME order with the
    // SAME guards — behaviour is unchanged. Helpers returning bool report "this tick is done,
    // return now", preserving the early-returns of the original single function.

    if (handle_inactive())               // idle / finished / aborted
        return;

    const auto now = this->now();

    if (begin_takeoff_delay(now))        // phase 0: start the takeoff-delay window
        return;

    if (!link_ready(now))                // gate: fresh pose/state + connected FCU
        return;

    if (prime_during_delay(now))         // hold current pose to prime PX4's setpoint stream
        return;

    capture_takeoff_target(now);         // latch takeoff target once (ground_z_ + height)

    if (check_external_override(now))    // PX4 left OFFBOARD after stable -> abort
        return;

    drive_and_stream(now);               // request OFFBOARD + ARM, stream target, watchdog

    if (check_arm_watchdogs(now))        // arm-fail timeout + unexpected disarm -> abort
        return;

    if (check_altitude_progress(now))    // reached-altitude detection + altitude timeout
        return;

    if (run_mission_or_hover(now))       // fly mission waypoints, or hover
        return;

    finalize_if_landed();                // landed + disarmed -> go idle
}

// ---------------------------------------------------------------------------------------
// control_loop() phase helpers (STEP 2). Each is the exact code lifted from the old loop.
// ---------------------------------------------------------------------------------------

bool OffboardMode::handle_inactive()
{
    // Not actively flying (idle / finished / aborted) -> stop the setpoint stream and bail.
    if (!start_offboard_.load() || landed_ || aborted_)
    {
        streamer_->set_streaming(false);

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
        return true;
    }
    return false;
}

bool OffboardMode::begin_takeoff_delay(const rclcpp::Time &now)
{
    // Phase 0: takeoff delay (also primes PX4's setpoint stream before OFFBOARD)
    if (!delay_started_)
    {
        takeoff_delay_start_time_ = now;
        delay_started_ = true;
        RCLCPP_INFO(this->get_logger(), "⏳ Starting takeoff delay (%.1fs)...", takeoff_delay_.seconds());
        return true;
    }
    return false;
}

bool OffboardMode::link_ready(const rclcpp::Time &now)
{
    // Need real data + a live link before commanding anything that moves the drone.
    if (!pose_received_ || !state_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⚠️ Waiting for first pose/state from MAVROS...");
        return false;
    }
    if ((now - last_pose_time_).seconds() > 1.0 || (now - last_state_time_).seconds() > 1.5)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⚠️ Pose/state stale (>1s) — holding setpoints off.");
        return false;
    }
    if (!current_state_.connected)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "❌ FCU not connected — not commanding.");
        return false;
    }
    return true;
}

bool OffboardMode::prime_during_delay(const rclcpp::Time &now)
{
    // During the delay window: hold the CURRENT pose to prime the stream (publishing the
    // live pose avoids a lurch when OFFBOARD engages).
    if ((now - takeoff_delay_start_time_) < takeoff_delay_)
    {
        transition_to(FlightState::TAKEOFF_DELAY);
        geometry_msgs::msg::PoseStamped hold = current_pose_;
        stream_setpoint(hold);   // streamer keeps publishing this to prime OFFBOARD
        return true;
    }
    return false;
}

void OffboardMode::capture_takeoff_target(const rclcpp::Time &now)
{
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
        transition_to(FlightState::ARMING);
        RCLCPP_INFO(this->get_logger(),
                    "🎯 Takeoff target: x=%.2f y=%.2f z=%.2f (ground z=%.2f + %.2f m)",
                    target_pose_.pose.position.x, target_pose_.pose.position.y,
                    target_pose_.pose.position.z, current_pose_.pose.position.z, takeoff_height_);
    }
}

bool OffboardMode::check_external_override(const rclcpp::Time &now)
{
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
            transition_to(FlightState::ABORT);
            return true;
        }
        // Brief revert (< offboard_stable_sec_): reset so the NEXT OFFBOARD entry
        // restarts the stability clock fresh instead of accumulating across sessions.
        offboard_engaged_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "⚠️ PX4 left OFFBOARD ('%s') %.1fs after engage — retrying...",
                             current_state_.mode.c_str(), stable_sec);
    }
    return false;
}

void OffboardMode::drive_and_stream(const rclcpp::Time &now)
{
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
        if (streamer_->streaming() && current_state_.armed)
        {
            const int64_t last_ns = streamer_->last_publish_ns();
            const double  age_s   = (now.nanoseconds() - last_ns) * 1e-9;
            if (last_ns != 0 && age_s > 0.25)
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "🚨 Setpoint stream STALLED (%.0fms since last publish)! "
                    "PX4 may drop OFFBOARD — companion CPU likely overloaded.", age_s * 1e3);
        }
    }
    else
    {
        streamer_->set_streaming(false);   // AUTO.LAND in progress -> stop our setpoint stream
    }
}

bool OffboardMode::check_arm_watchdogs(const rclcpp::Time &now)
{
    // Arm-fail timeout: clear feedback instead of retrying forever (usually PX4 pre-arm:
    // no position estimate / sensors / safety switch).
    if (!has_armed && (now - arm_attempt_start_) > rclcpp::Duration::from_seconds(arm_timeout_sec_))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "❌ Failed to ARM within %.0fs — aborting. Check PX4 pre-arm (position estimate / sensors).",
                     arm_timeout_sec_);
        log_event("ABORT: failed to ARM within timeout (PX4 pre-arm: position/sensors?)");
        aborted_ = true;
        transition_to(FlightState::ABORT);
        return true;
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
            transition_to(FlightState::ABORT);
            return true;
        }
    }
    else
    {
        arm_lost_ = false;   // armed again (or landing) — clear the flag
    }
    return false;
}

bool OffboardMode::check_altitude_progress(const rclcpp::Time &now)
{
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
        return true;
    }
    return false;
}

bool OffboardMode::run_mission_or_hover(const rclcpp::Time &now)
{
    // After reaching altitude: fly mission waypoints if loaded, otherwise pure hover.
    if (!(has_armed && reached_altitude_ && !landing_started_))
        return false;

    // On first entry after altitude reached: decide mission vs hover mode.
    // (state_ == MISSION is now the source of truth, set by decide_mission_vs_hover.)
    if (state_ != FlightState::MISSION)
        decide_mission_vs_hover(now);

    if (state_ == FlightState::MISSION)
        return fly_active_mission(now);

    // Pure hover (no mission): auto-land after hover_seconds_. <=0 = hold forever.
    if (hover_seconds_ > 0.0 &&
        (now - altitude_reach_time_) > rclcpp::Duration::from_seconds(hover_seconds_))
    {
        RCLCPP_INFO(this->get_logger(), "🔻 Hover %.0fs done — landing.", hover_seconds_);
        land_vehicle();
    }
    return false;
}

void OffboardMode::decide_mission_vs_hover(const rclcpp::Time &now)
{
    std::lock_guard<std::mutex> lk(mission_mtx_);
    // Fly waypoints ONLY when this flight was started by a mission (mission_intent_). An
    // OFFBOARD-triggered flight always hovers, even if mission_wps_ happens to hold an old
    // mission — the two intents are kept strictly separate.
    if (mission_intent_ && !mission_wps_.empty())
    {
        // CONSUME the mission: hand a private copy to the follower, then clear the inbox so a
        // finished mission is NOT replayed. To fly it again, the ground must re-send it.
        std::vector<CarrotFollower::Waypoint> wps;
        wps.reserve(mission_wps_.size());
        for (const auto &p : mission_wps_)
            wps.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z,
                           p.pose.orientation.x, p.pose.orientation.y,
                           p.pose.orientation.z, p.pose.orientation.w});
        mission_wps_.clear();
        // Seat the carrot at the drone's current position so it eases out from the hover
        // point instead of jumping straight to the first waypoint.
        follower_.begin(wps, ground_z_,
                        current_pose_.pose.position.x,
                        current_pose_.pose.position.y,
                        current_pose_.pose.position.z);
        wp_enter_time_  = now;
        flight_decision_logged_ = true;
        transition_to(FlightState::MISSION);
        log_event("DECISION: FLY MISSION (" + std::to_string(follower_.count()) +
                  " waypoints)");
        RCLCPP_INFO(this->get_logger(),
                    "🗺️ Mission start: %zu WPs — cruising %.1f m/s, lead %.1fm",
                    follower_.count(), follower_.params.cruise_speed, follower_.params.carrot_lead);
    }
    else if (!flight_decision_logged_)
    {
        // Plain hover flight (OFFBOARD intent, or a mission flight with no waypoints): hold
        // position, then auto-land after hover_seconds_. This is the intended takeoff →
        // hover → land behaviour, NOT a failure.
        flight_decision_logged_ = true;
        transition_to(FlightState::HOVER);
        log_event("DECISION: HOVER (OFFBOARD intent — takeoff/hover/land, no mission)");
        RCLCPP_INFO(this->get_logger(),
                    "🚁 OFFBOARD hover — takeoff/hover/land (no mission).");
    }
}

bool OffboardMode::fly_active_mission(const rclcpp::Time &now)
{
    if (!follower_.active())
    {
        // Safety guard (should not happen — the node lands on MISSION_COMPLETE).
        RCLCPP_WARN(this->get_logger(), "⚠️ No active waypoint — landing.");
        land_vehicle();   // transitions to LANDING (state_ leaves MISSION)
        return true;
    }

    // Advance the carrot one tick. The follower owns the carrot/WP geometry; the node owns
    // the WP timer (wp_enter_time_) and acts on the returned event.
    const double elapsed = (now - wp_enter_time_).seconds();
    const CarrotFollower::Result r = follower_.update(
        current_pose_.pose.position.x,
        current_pose_.pose.position.y,
        current_pose_.pose.position.z,
        elapsed);

    // Publish the carrot as the position setpoint.
    target_pose_.pose.position.x    = r.x;
    target_pose_.pose.position.y    = r.y;
    target_pose_.pose.position.z    = r.z;
    target_pose_.pose.orientation.x = r.qx;
    target_pose_.pose.orientation.y = r.qy;
    target_pose_.pose.orientation.z = r.qz;
    target_pose_.pose.orientation.w = r.qw;

    using Event = CarrotFollower::Event;
    if (r.event == Event::MISSION_COMPLETE)
    {
        // Final WP reached (or timed out) — land (gentle stop).
        if (r.by_timeout)
            RCLCPP_WARN(this->get_logger(),
                        "⏰ Final WP%zu timeout (%.0fs) — landing anyway.",
                        r.wp_index + 1, follower_.params.wp_timeout);
        else
            RCLCPP_INFO(this->get_logger(),
                        "🏁 Mission complete (WP%zu, dist=%.2fm) — landing.",
                        r.wp_index + 1, r.drone_dist);
        log_event(std::string("MISSION COMPLETE at WP") +
                  std::to_string(r.wp_index + 1) +
                  (r.by_timeout ? " (timeout)" : "") + " -> landing");
        land_vehicle();   // transitions to LANDING (state_ leaves MISSION)
    }
    else if (r.event == Event::WAYPOINT_PASSED)
    {
        // Intermediate WP passed (fly-through) or timed out — the follower already advanced;
        // reset our WP timer for the new active waypoint.
        if (r.by_timeout)
            RCLCPP_WARN(this->get_logger(),
                        "⏰ WP%zu/%zu timeout (%.0fs) — skipping",
                        r.wp_index + 1, r.wp_count, follower_.params.wp_timeout);
        else
            RCLCPP_INFO(this->get_logger(),
                        "✅ WP%zu/%zu passed (drone %.2fm) → next",
                        r.wp_index + 1, r.wp_count, r.drone_dist);
        log_event(std::string("WP ") + std::to_string(r.wp_index + 1) + "/" +
                  std::to_string(r.wp_count) +
                  (r.by_timeout ? " TIMEOUT -> skip" : " reached -> next"));
        wp_enter_time_ = now;
    }
    return false;
}

void OffboardMode::finalize_if_landed()
{
    // Landing complete: AUTO.LAND auto-disarms → go idle, ready for the next OFFBOARD.
    // (Timer keeps running — control_loop returns cheaply while idle — so we can re-fly.)
    if (landing_started_ && !current_state_.armed)
    {
        landed_ = true;
        start_offboard_.store(false);
        transition_to(FlightState::IDLE);
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