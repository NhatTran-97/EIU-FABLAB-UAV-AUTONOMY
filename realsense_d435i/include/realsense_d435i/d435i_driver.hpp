#ifndef REALSENSE_D435I__D435I_DRIVER_HPP_
#define REALSENSE_D435I__D435I_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include <atomic>
#include <cstring>

class D435iDriver : public rclcpp::Node
{
public:
    D435iDriver();
    ~D435iDriver();

    enum class State { IDLE, CONNECTING, STREAMING, ERROR };

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    // RealSense
    rs2::pipeline pipe_;
    rs2::pipeline_profile profile_;
    rs2::context ctx_;
    rs2::sensor imu_sensor_;

    // State machine
    std::atomic<State> state_{State::IDLE};
    int retry_count_ = 0;
    static constexpr int MAX_RETRIES = 10;
    static constexpr int RETRY_DELAY_SEC = 3;

    // Thread
    std::thread thread_;
    std::atomic<bool> running_{true};

    struct StreamConfig
    {
        int width           = 848;
        int height          = 480;
        int fps             = 30;
        bool enable_emitter = false;
        int accel_rate      = 200;
        int gyro_rate       = 200;
        bool auto_exposure  = true;
        int warmup_frames   = 30;
    } config_;

    struct ImageBuffer
    {
        sensor_msgs::msg::Image left;
        sensor_msgs::msg::Image right;
    } img_buf_;

    struct ImuCache
    {
        geometry_msgs::msg::Vector3 accel;
        geometry_msgs::msg::Vector3 gyro;
        bool has_accel = false;
        bool has_gyro  = false;
    } imu_cache_;

    struct Diagnostics
    {
        uint64_t frame_count   = 0;
        uint64_t imu_count     = 0;
        uint64_t drop_count    = 0;
        uint64_t ts_error_count = 0;
        uint64_t last_frame_number = 0;
        double last_img_ts     = 0.0;
        double last_accel_ts   = 0.0;
        double last_gyro_ts    = 0.0;
        int warmup_remaining   = 0;
        rclcpp::TimerBase::SharedPtr timer;
    } diag_;

    // State machine transitions
    void transition_to(State new_state);
    bool try_connect();
    void disconnect();

    // Core
    void load_parameters();
    void configure_device();
    void start_imu_callback();
    void poll_loop();
    void print_diagnostics();

    // Publish
    rclcpp::Time rs2_to_ros_time(double ts_ms);
    bool validate_timestamp(double ts_ms, double& last_ts);
    void publish_image(sensor_msgs::msg::Image& msg, const rs2::video_frame& frame,
                       rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                       const std::string& frame_id);
    void publish_imu(const rs2::motion_frame& frame);
};

#endif
