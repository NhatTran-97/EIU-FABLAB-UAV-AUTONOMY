#include "realsense_d435i/d435i_driver.hpp"

D435iDriver::D435iDriver() : Node("d435i_driver")
{
    load_parameters();

    pub_left_  = create_publisher<sensor_msgs::msg::Image>(declare_parameter("topic_cam0", "/cam0/image_raw"), 10);
    pub_right_ = create_publisher<sensor_msgs::msg::Image>(declare_parameter("topic_cam1", "/cam1/image_raw"), 10);
    pub_imu_   = create_publisher<sensor_msgs::msg::Imu>(declare_parameter("topic_imu", "/imu0"), 200);

    diag_.timer = create_wall_timer(std::chrono::seconds(5), std::bind(&D435iDriver::print_diagnostics, this));

    thread_ = std::thread(&D435iDriver::poll_loop, this);
}

D435iDriver::~D435iDriver()
{
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
    disconnect();
}

// ── Parameters ──────────────────────────────────────────────

void D435iDriver::load_parameters()
{
    config_.width          = declare_parameter("width", 848);
    config_.height         = declare_parameter("height", 480);
    config_.fps            = declare_parameter("fps", 30);
    config_.enable_emitter = declare_parameter("enable_emitter", false);
    config_.accel_rate     = declare_parameter("accel_rate", 200);
    config_.gyro_rate      = declare_parameter("gyro_rate", 200);
    config_.auto_exposure  = declare_parameter("auto_exposure", true);
    config_.warmup_frames  = declare_parameter("warmup_frames", 30);
}

// ── State Machine ───────────────────────────────────────────

void D435iDriver::transition_to(State new_state)
{
    State old = state_.exchange(new_state);

    if (old == new_state) return;

    const char* names[] = {"IDLE", "CONNECTING", "STREAMING", "ERROR"};

    RCLCPP_INFO(get_logger(), "State: %s -> %s", names[static_cast<int>(old)], names[static_cast<int>(new_state)]);
}

bool D435iDriver::try_connect()
{
    transition_to(State::CONNECTING);

    auto devices = ctx_.query_devices();
    if (devices.size() == 0)
    {
        RCLCPP_WARN(get_logger(), "No RealSense device found");
        return false;
    }

    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, config_.width, config_.height, RS2_FORMAT_Y8, config_.fps);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, config_.width, config_.height, RS2_FORMAT_Y8, config_.fps);

        pipe_ = rs2::pipeline(ctx_);
        profile_ = pipe_.start(cfg);

        configure_device();
        start_imu_callback();

        // Pre-allocate image buffers
        size_t img_size = config_.width * config_.height;
        img_buf_.left.data.resize(img_size);
        img_buf_.left.encoding     = "mono8";
        img_buf_.left.width        = config_.width;
        img_buf_.left.height       = config_.height;
        img_buf_.left.step         = config_.width;
        img_buf_.left.is_bigendian = false;
        img_buf_.right = img_buf_.left;

        // Reset IMU cache
        imu_cache_.has_accel = false;
        imu_cache_.has_gyro  = false;
        imu_cache_.accel = geometry_msgs::msg::Vector3();
        imu_cache_.gyro  = geometry_msgs::msg::Vector3();

        // Reset diagnostics
        diag_.last_frame_number = 0;
        diag_.last_img_ts       = 0.0;
        diag_.last_accel_ts     = 0.0;
        diag_.last_gyro_ts      = 0.0;
        diag_.drop_count        = 0;
        diag_.ts_error_count    = 0;
        diag_.warmup_remaining  = config_.warmup_frames;

        retry_count_ = 0;

        RCLCPP_INFO(get_logger(), "Warmup: skipping first %d frames...", config_.warmup_frames);

        transition_to(State::STREAMING);
        return true;

    } catch (const rs2::error& e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to start pipeline: %s", e.what());
        return false;
    }
}

void D435iDriver::disconnect()
{
    try
    {
        imu_sensor_.stop();
        imu_sensor_.close();
    } catch (...) {}
    try
    {
        pipe_.stop();
    } catch (...) {}
    transition_to(State::IDLE);
}

void D435iDriver::start_imu_callback()
{
    auto device = profile_.get_device();

    for (auto& sensor : device.query_sensors())
    {
        if (sensor.is<rs2::motion_sensor>())
        {
            imu_sensor_ = sensor;
            break;
        }
    }

    std::vector<rs2::stream_profile> imu_profiles;
    for (auto& p : imu_sensor_.get_stream_profiles())
    {
        if (p.stream_type() == RS2_STREAM_ACCEL && p.fps() == config_.accel_rate &&
            p.format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            imu_profiles.push_back(p);
        }
        if (p.stream_type() == RS2_STREAM_GYRO && p.fps() == config_.gyro_rate &&
            p.format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            imu_profiles.push_back(p);
        }
    }

    imu_sensor_.open(imu_profiles);
    imu_sensor_.start([this](rs2::frame frame) {
        if (!running_) return;
        if (auto mf = frame.as<rs2::motion_frame>())
        {
            publish_imu(mf);
        }
    });

    RCLCPP_INFO(get_logger(), "IMU callback started: Accel=%dHz Gyro=%dHz",
                config_.accel_rate, config_.gyro_rate);
}

void D435iDriver::configure_device()
{
    auto device = profile_.get_device();

    RCLCPP_INFO(get_logger(), "Device: %s  FW: %s  SN: %s",
                device.get_info(RS2_CAMERA_INFO_NAME),
                device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
                device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // Global time — dong bo timestamp hardware voi system clock
    for (auto& sensor : device.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
        {
            sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f);
        }
    }

    // Auto-exposure cho IR stereo
    auto stereo_sensor = device.first<rs2::depth_sensor>();
    stereo_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, config_.enable_emitter ? 1.f : 0.f);

    if (config_.auto_exposure)
    {
        stereo_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.f);
    }

    RCLCPP_INFO(get_logger(), "Config: [%dx%d@%dfps] Emitter=%s AutoExposure=%s Accel=%dHz Gyro=%dHz",
                                                            config_.width, config_.height, config_.fps,
                                                            config_.enable_emitter ? "ON" : "OFF",
                                                            config_.auto_exposure ? "ON" : "OFF",
                                                            config_.accel_rate, config_.gyro_rate);
}

// ── Publish ─────────────────────────────────────────────────

rclcpp::Time D435iDriver::rs2_to_ros_time(double ts_ms)
{
    int64_t ns = static_cast<int64_t>(ts_ms * 1e6);
    return rclcpp::Time(ns);
}

bool D435iDriver::validate_timestamp(double ts_ms, double& last_ts)
{
    if (last_ts == 0.0)
    {
        last_ts = ts_ms;
        return true;
    }

    double dt = ts_ms - last_ts;

    // Timestamp di nguoc hoac nhay qua lon (>2 giay)
    if (dt < 0.0 || dt > 2000.0)
    {
        diag_.ts_error_count++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Timestamp anomaly: dt=%.1fms (expected >0, <2000)", dt);
        last_ts = ts_ms;
        return false;
    }

    last_ts = ts_ms;
    return true;
}

void D435iDriver::publish_image(sensor_msgs::msg::Image& msg, const rs2::video_frame& frame,
                                 rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                                 const std::string& frame_id)
{
    msg.header.stamp    = rs2_to_ros_time(frame.get_timestamp());
    msg.header.frame_id = frame_id;

    auto* data = reinterpret_cast<const uint8_t*>(frame.get_data());
    std::memcpy(msg.data.data(), data, msg.data.size());

    pub->publish(msg);
}

void D435iDriver::publish_imu(const rs2::motion_frame& frame)
{
    bool is_accel = (frame.get_profile().stream_type() == RS2_STREAM_ACCEL);
    double& last_ts = is_accel ? diag_.last_accel_ts : diag_.last_gyro_ts;
    if (!validate_timestamp(frame.get_timestamp(), last_ts))
        return;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = rs2_to_ros_time(frame.get_timestamp());
    msg.header.frame_id = "imu0";

    auto data = frame.get_motion_data();

    if (frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
    {
        msg.linear_acceleration.x = data.x;
        msg.linear_acceleration.y = data.y;
        msg.linear_acceleration.z = data.z;
        imu_cache_.accel = msg.linear_acceleration;
        imu_cache_.has_accel = true;
        msg.angular_velocity = imu_cache_.gyro;
    } else {
        msg.angular_velocity.x = data.x;
        msg.angular_velocity.y = data.y;
        msg.angular_velocity.z = data.z;
        imu_cache_.gyro = msg.angular_velocity;
        imu_cache_.has_gyro = true;
        msg.linear_acceleration = imu_cache_.accel;
    }

    if (imu_cache_.has_accel && imu_cache_.has_gyro)
    {
        pub_imu_->publish(msg);
        diag_.imu_count++;
    }
}

// ── Main Loop ───────────────────────────────────────────────

void D435iDriver::poll_loop()
{
    while (running_)
    {
        State current = state_.load();

        switch (current)
        {
            case State::IDLE:
            case State::CONNECTING:
            {
                if (try_connect()) break;

                retry_count_++;
                transition_to(State::ERROR);

                if (retry_count_ >= MAX_RETRIES)
                {
                    RCLCPP_FATAL(get_logger(), "Failed to connect after %d attempts. Giving up.", MAX_RETRIES);
                    running_ = false;
                    return;
                }

                RCLCPP_WARN(get_logger(), "Retry %d/%d in %d seconds...", retry_count_, MAX_RETRIES, RETRY_DELAY_SEC);

                for (int i = 0; i < RETRY_DELAY_SEC && running_; i++)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                break;
            }

            case State::STREAMING:
            {
                try {
                    auto frames = pipe_.wait_for_frames(5000);

                    // Warmup — skip frame dau, cho IMU + auto-exposure on dinh
                    if (diag_.warmup_remaining > 0)
                    {
                        diag_.warmup_remaining--;
                        if (diag_.warmup_remaining == 0)
                        {
                            RCLCPP_INFO(get_logger(), "Warmup complete. Publishing data.");
                        }
                        break;
                    }

                    // Stereo IR
                    auto left  = frames.get_infrared_frame(1);
                    auto right = frames.get_infrared_frame(2);

                    if (left && right)
                    {
                        // Frame drop detection
                        uint64_t frame_num = left.get_frame_number();
                        if (diag_.last_frame_number > 0)
                        {
                            uint64_t expected = diag_.last_frame_number + 1;
                            if (frame_num > expected)
                            {
                                uint64_t dropped = frame_num - expected;
                                diag_.drop_count += dropped;
                                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                    "Frame drop detected: %lu frames lost", dropped);
                            }
                        }
                        diag_.last_frame_number = frame_num;

                        // Validate timestamp truoc khi publish
                        if (validate_timestamp(left.get_timestamp(), diag_.last_img_ts))
                        {
                            publish_image(img_buf_.left,  left,  pub_left_,  "cam0");
                            publish_image(img_buf_.right, right, pub_right_, "cam1");
                        }
                    }

                    diag_.frame_count++;

                } catch (const rs2::error& e) {
                    RCLCPP_ERROR(get_logger(), "Stream error: %s", e.what());
                    disconnect();
                    transition_to(State::ERROR);
                }
                break;
            }

            case State::ERROR:
            {
                disconnect();
                RCLCPP_WARN(get_logger(), "Attempting reconnect...");
                std::this_thread::sleep_for(std::chrono::seconds(RETRY_DELAY_SEC));
                transition_to(State::CONNECTING);
                break;
            }
        }
    }
}

// ── Diagnostics ─────────────────────────────────────────────

void D435iDriver::print_diagnostics()
{
    if (state_.load() != State::STREAMING) return;

    if (diag_.warmup_remaining > 0)
    {
        RCLCPP_INFO(get_logger(), "[DIAG] Warming up... %d frames remaining", diag_.warmup_remaining);
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[DIAG] cam=%.1fHz imu=%.1fHz drops=%lu ts_errors=%lu",
        diag_.frame_count / 5.0, diag_.imu_count / 5.0,
        diag_.drop_count, diag_.ts_error_count);

    // Canh bao neu Hz thap qua
    double cam_hz = diag_.frame_count / 5.0;
    if (cam_hz < config_.fps * 0.8 && diag_.frame_count > 0)
    {
        RCLCPP_WARN(get_logger(), "[DIAG] Camera Hz low: %.1f (expected %d). Check USB bandwidth.", cam_hz, config_.fps);
    }

    diag_.frame_count = 0;
    diag_.imu_count = 0;
}
