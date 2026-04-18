#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct StreamHandle
{
    cv::VideoCapture cap;
    std::string url;
    std::string name;       // "RGB" / "Thermal"
    std::string frame_id;
    int fail_count{0};
    bool was_opened{false};
    std::optional<std::chrono::steady_clock::time_point> last_reconnect_attempt;
    std::mutex mtx;         // bảo vệ cap khi reconnect từ capture thread
};

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), running_(false)
    {
        this->declare_parameter<std::string>(
            "rgb_url",
            "rtspsrc location=rtsp://192.168.144.108:554/stream=1 latency=50 ! "
            "decodebin ! videoconvert ! appsink sync=false drop=true max-buffers=1");

        this->declare_parameter<std::string>(
            "thermal_url",
            "rtspsrc location=rtsp://192.168.144.108:555/stream=2 latency=50 ! "
            "decodebin ! videoconvert ! appsink sync=false drop=true max-buffers=1");

        this->declare_parameter<int>("fail_threshold", 90);
        this->declare_parameter<int>("reconnect_cooldown_s", 5);

        rgb_.url      = this->get_parameter("rgb_url").as_string();
        rgb_.name     = "RGB";
        rgb_.frame_id = "rgb_camera";

        thermal_.url      = this->get_parameter("thermal_url").as_string();
        thermal_.name     = "Thermal";
        thermal_.frame_id = "thermal_camera";

        fail_threshold_     = this->get_parameter("fail_threshold").as_int();
        reconnect_cooldown_ = std::chrono::seconds(
            this->get_parameter("reconnect_cooldown_s").as_int());

        if (fail_threshold_ <= 0) fail_threshold_ = 90;

        auto qos = rclcpp::SensorDataQoS();

        pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/skydroid/rgb/image_raw", qos);

        pub_thermal_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/skydroid/thermal/image_raw", qos);

        // Mở stream trước khi spawn thread
        openStream(rgb_);
        openStream(thermal_);

        running_ = true;
        thread_rgb_     = std::thread(&CameraNode::captureLoop, this,
                                      std::ref(rgb_), std::ref(pub_rgb_));
        thread_thermal_ = std::thread(&CameraNode::captureLoop, this,
                                      std::ref(thermal_), std::ref(pub_thermal_));

        RCLCPP_INFO(this->get_logger(),
            "CameraNode started | fail_threshold=%d | cooldown=%lds",
            fail_threshold_, static_cast<long>(reconnect_cooldown_.count()));
    }

    ~CameraNode() override
    {
        running_ = false;

        if (thread_rgb_.joinable())     thread_rgb_.join();
        if (thread_thermal_.joinable()) thread_thermal_.join();

        rgb_.cap.release();
        thermal_.cap.release();
    }

private:
    static std::string getRosEncoding(const cv::Mat & frame)
    {
        switch (frame.channels())
        {
            case 1:  return sensor_msgs::image_encodings::MONO8;
            case 4:  return sensor_msgs::image_encodings::BGRA8;
            default: return sensor_msgs::image_encodings::BGR8;
        }
    }

    bool cooldownElapsed(const StreamHandle & s) const
    {
        if (!s.last_reconnect_attempt.has_value()) return true;
        return (std::chrono::steady_clock::now() - *s.last_reconnect_attempt)
               >= reconnect_cooldown_;
    }

    void openStream(StreamHandle & s)
    {
        s.last_reconnect_attempt = std::chrono::steady_clock::now();
        s.cap.release();
        s.fail_count = 0;

        const bool ok = s.cap.open(s.url, cv::CAP_GSTREAMER);

        if (ok && s.cap.isOpened())
        {
            if (!s.was_opened)
                RCLCPP_INFO(this->get_logger(), "[%s] Stream opened", s.name.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "[%s] Reconnect OK", s.name.c_str());
            s.was_opened = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "[%s] Khong mo duoc stream, thu lai sau %lds...",
                s.name.c_str(),
                static_cast<long>(reconnect_cooldown_.count()));
        }
    }

    // Mỗi stream chạy vòng lặp riêng — block trên cap.read() độc lập nhau
    void captureLoop(StreamHandle & s,
                     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub)
    {
        while (running_)
        {
            // --- Nếu stream chưa mở: chờ cooldown rồi thử lại ---
            {
                std::lock_guard<std::mutex> lock(s.mtx);
                if (!s.cap.isOpened())
                {
                    if (cooldownElapsed(s)) openStream(s);
                }
            }

            if (!s.cap.isOpened())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            // --- Đọc frame (blocking — chờ đến khi RTSP gửi frame mới) ---
            cv::Mat frame;
            bool ok;
            {
                std::lock_guard<std::mutex> lock(s.mtx);
                ok = s.cap.read(frame);
            }

            if (ok && !frame.empty())
            {
                s.fail_count = 0;

                std_msgs::msg::Header header;
                header.stamp    = this->now();   // timestamp ngay khi nhận frame
                header.frame_id = s.frame_id;

                auto msg = cv_bridge::CvImage(header, getRosEncoding(frame), frame)
                               .toImageMsg();
                pub->publish(*msg);
            }
            else
            {
                s.fail_count++;

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[%s] Mat frame (%d/%d)",
                    s.name.c_str(), s.fail_count, fail_threshold_);

                if (s.fail_count >= fail_threshold_)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "[%s] Stream mat ket noi, dang reconnect...", s.name.c_str());
                    std::lock_guard<std::mutex> lock(s.mtx);
                    openStream(s);
                }
            }
        }
    }

    std::atomic<bool> running_;
    int fail_threshold_{90};
    std::chrono::seconds reconnect_cooldown_{5};

    StreamHandle rgb_;
    StreamHandle thermal_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_thermal_;

    std::thread thread_rgb_;
    std::thread thread_thermal_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
