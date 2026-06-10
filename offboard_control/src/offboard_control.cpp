#include "offboard_control.hpp"

using std::placeholders::_1;

OffboardControl::OffboardControl()
: Node("offboard_control"),
  takeoff_delay_(rclcpp::Duration::from_seconds(5.0))
{
    this->declare_parameter("takeoff_height", 5.0);
    this->declare_parameter("hover_duration", 10.0);
    this->declare_parameter("altitude_timeout", 60.0);
    this->get_parameter("takeoff_height",    takeoff_height_);
    this->get_parameter("hover_duration",    hover_duration_);
    this->get_parameter("altitude_timeout",  altitude_timeout_);

    // hardcode
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = takeoff_height_;
    target_pose_.header.frame_id = "map";

    // Initialize timestamps — prevents false "no data" warnings before first messages arrive
    auto now = this->now();
    last_pose_time_   = now;
    last_state_time_  = now;
    csv_start_        = now;
    takeoff_start_time_ = now;
    last_arm_request_ = now - rclcpp::Duration::from_seconds(10.0);

    rclcpp::QoS qos_state(rclcpp::KeepLast(10));
    qos_state.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_state.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rclcpp::QoS qos_pose(rclcpp::KeepLast(10));
    qos_pose.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pose.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::QoS qos_pub(rclcpp::KeepLast(10));
    qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", qos_state,
        std::bind(&OffboardControl::state_cb, this, _1));

    local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", qos_pose,
        std::bind(&OffboardControl::position_cb, this, _1));

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", qos_pub);

    arming_client_   = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&OffboardControl::main_loop, this));

    timer_csv_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OffboardControl::log_position, this));

    csv_file_.open("drone_log_position.csv", std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
        csv_file_ << "time,x,y,z\n";
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open drone_log_position.csv");
    }

    RCLCPP_INFO(this->get_logger(),
                "OffboardControl started — height: %.1fm, hover: %.1fs, timeout: %.1fs",
                takeoff_height_, hover_duration_, altitude_timeout_);
}

OffboardControl::~OffboardControl()
{
    if (csv_file_.is_open()) {
        csv_file_.flush();
        csv_file_.close();
    }
}

void OffboardControl::state_cb(const mavros_msgs::msg::State & msg)
{
    current_state_   = msg;
    last_state_time_ = this->now();

    if (msg.mode == "OFFBOARD" && offboard_flag_) {
        RCLCPP_INFO(this->get_logger(), "OFFBOARD MODE detected");
        takeoff_flag_ = true;
        landing_flag_ = false;
    } else if (msg.mode == "AUTO.LAND") {
        landing_flag_  = true;
        offboard_flag_ = false;
        RCLCPP_INFO(this->get_logger(), "AUTO.LAND MODE");
    } else if (msg.mode == "STABILIZED") {
        offboard_flag_ = true;
    }
}

void OffboardControl::position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_pose_time_ = this->now();
    current_x_      = msg->pose.position.x;
    current_y_      = msg->pose.position.y;
    current_z_      = msg->pose.position.z;

    if (!reach_attitude_ && current_z_ >= takeoff_height_ - 0.3) {
        reach_attitude_      = true;
        attitude_reach_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Target altitude reached (%.2f m), hovering for %.0fs...",
                    current_z_, hover_duration_);
    }
}

void OffboardControl::arm_drone()
{
    // Throttle requests — don't spam FCU
    if ((this->now() - last_arm_request_).seconds() < 5.0) return;

    if (!arming_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Arming service not ready");
        return;
    }

    last_arm_request_ = this->now();
    auto request      = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value    = true;
    arming_client_->async_send_request(request,
        std::bind(&OffboardControl::arm_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Sending arm request...");
}

void OffboardControl::arm_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
{
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Drone armed!");
            check_armed_        = true;
            takeoff_start_time_ = this->now(); // altitude timeout counts from actual arm time
        } else {
            RCLCPP_WARN(this->get_logger(), "Arm request rejected by FCU");
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Arm service call failed: %s", e.what());
    }
}

void OffboardControl::landing()
{
    if (landing_flag_) return;

    if (!set_mode_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
        return;
    }

    landing_flag_ = true;
    auto request         = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode   = 0;
    request->custom_mode = "AUTO.LAND";

    set_mode_client_->async_send_request(request,
        std::bind(&OffboardControl::landing_cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Requesting AUTO.LAND mode...");
}

void OffboardControl::landing_cb(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
{
    try {
        auto response = future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "AUTO.LAND mode set successfully.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to set AUTO.LAND mode, will retry.");
            landing_flag_ = false;
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Landing service call failed: %s", e.what());
        landing_flag_ = false;
    }
}

void OffboardControl::main_loop()
{
    if (!delay_started_) {
        takeoff_delay_start_time_ = this->now();
        delay_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Publishing setpoints for %.1fs before OFFBOARD switch...",
                    takeoff_delay_.seconds());
    }

    // FCU requires continuous setpoints at >2Hz to enter/maintain OFFBOARD mode
    if (!check_landed_) {
        target_pose_.header.stamp = this->now();
        setpoint_pub_->publish(target_pose_);
    }

    if ((this->now() - takeoff_delay_start_time_) < takeoff_delay_) {
        return;
    }

    if (!current_state_.connected) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Lost connection to FCU!");
        return;
    }
    if ((this->now() - last_pose_time_).seconds() > 1.0 ||
        (this->now() - last_state_time_).seconds() > 1.5) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "No pose or state data received recently!");
        return;
    }

    if (takeoff_flag_ && !check_armed_ && !landing_flag_) {
        arm_drone();
    }

    if (check_armed_ && !reach_attitude_ &&
        (this->now() - takeoff_start_time_) > rclcpp::Duration::from_seconds(altitude_timeout_)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Timeout: failed to reach %.1fm in %.0fs. Initiating landing...",
                     takeoff_height_, altitude_timeout_);
        landing();
        return;
    }

    if (check_armed_ && reach_attitude_ && !landing_flag_ &&
        (this->now() - attitude_reach_time_) > rclcpp::Duration::from_seconds(hover_duration_)) {
        RCLCPP_INFO(this->get_logger(), "%.0fs hover complete, initiating landing...", hover_duration_);
        landing();
        return;
    }

    if (landing_flag_ && !current_state_.armed && !check_landed_) {
        check_landed_ = true;
        RCLCPP_INFO(this->get_logger(), "Drone has landed successfully.");
        timer_->cancel();
    }
}

void OffboardControl::log_position()
{
    if (!csv_file_.is_open()) return;

    double elapsed = (this->now() - csv_start_).seconds();
    csv_file_ << std::fixed << std::setprecision(3)
              << elapsed << ","
              << current_x_ << ","
              << current_y_ << ","
              << current_z_ << "\n";
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
