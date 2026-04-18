#include "offboard_control/offboard_mode.hpp"

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
                                                      takeoff_start_time_(this->now()),
                                                      takeoff_delay_(rclcpp::Duration::from_seconds(5.0)) ,
                                                      has_armed(false), landing_started_(false), 
                                                      landed_(false),reached_altitude_(false)                                                        
{
   

    this->declare_parameter("takeoff_height_", 5.0);
    this->get_parameter("takeoff_height_", takeoff_height_);
    RCLCPP_INFO(this->get_logger(), "Takeoff height set to: %.2f m", takeoff_height_);

    this->declare_parameter("min_battery_", 15.5);
    this->get_parameter("min_battery_", limit_min_battery_);

    this->declare_parameter("takeoff_delay_sec", 5.0);
    double delay_sec;
    this->get_parameter("takeoff_delay_sec", delay_sec);

    
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



    // ROS2 interfaces
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos_state, std::bind(&OffboardMode::state_cb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", qos_pose, std::bind(&OffboardMode::pose_cb, this, std::placeholders::_1));
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", qos_pub);
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", qos_battery, std::bind(&OffboardMode::battery_cb, this, std::placeholders::_1));
    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10, std::bind(&OffboardMode::ext_state_cb, this, std::placeholders::_1));


    
    // Set initial target pose 
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = takeoff_height_;


    // wait for FCU connection
    this->wait_for_fcu_connection();

    // Send a few initial setpoints before switching to OFFBOARD mode
    send_initial_setpoints();

    // Create perdiodic timer (50ms)
    timer_ = this->create_wall_timer(50ms, std::bind(&OffboardMode::control_loop, this));
   
}

void OffboardMode::wait_for_fcu_connection()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(this->get_node_base_interface());  
        rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "FCU connected");
}

void OffboardMode::send_initial_setpoints()
{
    RCLCPP_INFO(this->get_logger(), "Sending initial setpoints");
    for (int i = 100; rclcpp::ok() && i > 0; --i)
    {
        local_pos_pub_->publish(target_pose_);
        rclcpp::sleep_for(50ms);
    }
}

void OffboardMode::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_ = *msg;
    last_pose_time_ = this->now();  

    if (!reached_altitude_ && current_pose_.pose.position.z >= takeoff_height_ - 0.1)
    {
        reached_altitude_ = true;
        altitude_reach_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "✅ Target altitude reached, starting countdown...");
    }
}


void OffboardMode::state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
    last_state_time_ = this->now();  

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "🔄 Drone Status: mode = %s | armed = %s", current_state_.mode.c_str(), current_state_.armed ? "true" : "false");
}

void OffboardMode::battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    //return;
    battery_valtage_ = msg->voltage;

    if (!landing_started_ && battery_valtage_ < limit_min_battery_)
    {
        landing_started_ = true;  

        RCLCPP_WARN(this->get_logger(), "⚡ Điện áp pin thấp: %.2fV! Tiến hành hạ cánh khẩn cấp.", battery_valtage_);

        land_vehicle();  // 🔻 Switch to landing mode
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
            // set_mode_client_->async_send_request(request, std::bind(&OffboardMode::handle_mode_response, this, std::placeholders::_1));

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
            // arming_client_->async_send_request(req, std::bind(&OffboardMode::handle_arm_response, this, std::placeholders::_1));

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
            RCLCPP_INFO(this->get_logger(), "✅ Offboard mode set");
            last_mode_request_ = this->now();
        } else 
        {
            RCLCPP_WARN(this->get_logger(), "❌ Failed to set Offboard mode");
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

void OffboardMode::disarm_drone()
{
    if (!current_state_.armed)
    {
        RCLCPP_INFO(this->get_logger(), "❎ Drone is already DISARMED, no need to send request.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "🟠 Sending DISARM request...");

    if (!arming_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Arming service unavailable.");
        return;
    }

    auto request_arm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request_arm->value = false; // false = DISARM

    arming_client_->async_send_request(
        request_arm,
        std::bind(&OffboardMode::handle_disarm_respone, this, std::placeholders::_1)
    );
}



void OffboardMode::handle_disarm_respone(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
{
    try
    {
        auto response_arm = future.get();

        if (response_arm->success)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Drone DISARMED successfully.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "❌ DISARM request sent but failed.");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Error receiving DISARM response: %s", e.what());
    }
}

void OffboardMode::control_loop()
{
    // If the drone has already landed → do nothing
    if(this->landed_)
    {
        return;
    }

    // If this is the first time the timer runs → start the delay countdown
    if(!delay_started_)
    {
        takeoff_delay_start_time_ = this->now();
        delay_started_ = true;
        RCLCPP_INFO(this->get_logger(), "⏳ Starting takeoff delay (%.1fs)...", takeoff_delay_.seconds());
        return;
    }

    // If delay time has not passed yet → only publish the target pose
    if ((this->now() - takeoff_delay_start_time_) < takeoff_delay_)
    {
        local_pos_pub_->publish(target_pose_);
        return;
    }

    // Delay completed → proceed with normal flight operations

    if ((this->now() - last_pose_time_).seconds() > 1.0 || (this->now() - last_state_time_).seconds() > 1.5)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ No pose or state received in the last 1 second!");
        return;
    }


    // Check lost connection to FC
    if(!current_state_.connected)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Lost connection to FCU! Stopping setpoint publication.");
        return;
    }


    
    //Continue sending setpoints for flight control
    // Gửi setpoint liên tục trong khi chưa bắt đầu hạ cánh

    if(!landing_started_)
    {
        set_offboard_mode();
        arm_vehicle();
        local_pos_pub_->publish(target_pose_);
    }


    if(!reached_altitude_ && (this->now() - takeoff_start_time_) > rclcpp::Duration(30s))
    {
        RCLCPP_ERROR(this->get_logger(), "🕒 Timeout: Failed to reach target altitude within 30 seconds after ARMED. Initiating landing...");
        this->land_vehicle();
        return;
    }


        // After reaching the target altitude and waiting for 10 seconds → begin landing
    // ✅ If all conditions are met → proceed with normal landing
    if (has_armed && reached_altitude_ && (this->now() - altitude_reach_time_) > 10s && !landing_started_)
    {
        land_vehicle();
      //  landing_started_ = true;
        RCLCPP_INFO(this->get_logger(), "🔻 Landing initiated after holding altitude for 10 seconds");
    }


    // If the drone is in AUTO.LAND mode and is no longer armed → it has landed
    // Check if the drone has auto-disarmed → confirm landing complete
    if (landing_started_ && !current_state_.armed)
    {
        landed_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ Drone has successfully landed.");

       this->disarm_drone();
       RCLCPP_INFO(this->get_logger(), "✅ Successfully DISARM.");

       if(timer_ )
       {
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "⏹️ Timer has been stopped to save CPU resources.");
       }

        
        
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