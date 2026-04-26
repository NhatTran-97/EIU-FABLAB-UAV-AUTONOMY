
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
    std::mutex mtx;
};

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), running_(false)
    {
        this->declare_parameter<std::string>(
                                                "rgb_url",
                                                "rtspsrc location=rtsp://192.168.144.108:554/stream=1 protocols=tcp latency=50 drop-on-latency=true ! "
                                                "rtph265depay ! h265parse ! "
                                                "nvv4l2decoder enable-max-performance=1 ! "
                                                "nvvidconv ! video/x-raw,format=BGRx ! "
                                                "videoconvert ! video/x-raw,format=BGR ! "
                                                "appsink sync=false drop=true max-buffers=1");
        this->declare_parameter<std::string>(
                                                "thermal_url",
                                                "rtspsrc location=rtsp://192.168.144.108:555/stream=2 protocols=udp latency=50 drop-on-latency=true ! "
                                                "rtph265depay ! h265parse ! "
                                                "nvv4l2decoder enable-max-performance=1 ! "
                                                "nvvidconv ! video/x-raw,format=BGRx ! "
                                                "videoconvert ! video/x-raw,format=BGR ! "
                                                "appsink sync=false drop=true max-buffers=1");

        // this->declare_parameter<std::string>(
        //                                         "thermal_url",
        //                                         "rtspsrc location=rtsp://192.168.144.108:555/stream=2 protocols=tcp latency=50 ! "
        //                                         "rtph265depay ! h265parse ! avdec_h265 ! "
        //                                         "videoconvert ! video/x-raw,format=BGR ! "
        //                                         "appsink sync=false drop=true max-buffers=1");

        this->declare_parameter<int>("fail_threshold", 90);
        this->declare_parameter<int>("reconnect_cooldown_s", 5);

        rgb_.url      = this->get_parameter("rgb_url").as_string();
        rgb_.name     = "RGB";
        rgb_.frame_id = "rgb_camera";

        thermal_.url      = this->get_parameter("thermal_url").as_string();
        thermal_.name     = "Thermal";
        thermal_.frame_id = "thermal_camera";

        fail_threshold_ = this->get_parameter("fail_threshold").as_int();
        if (fail_threshold_ <= 0)
        {
            fail_threshold_ = 90;
        }

        int cooldown_s = this->get_parameter("reconnect_cooldown_s").as_int();

        if (cooldown_s <= 0)
        {
            cooldown_s = 5;
        }
        reconnect_cooldown_ = std::chrono::seconds(cooldown_s);

        auto qos = rclcpp::SensorDataQoS();

        pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("/skydroid/rgb/image_raw", qos);

        pub_thermal_ = this->create_publisher<sensor_msgs::msg::Image>("/skydroid/thermal/image_raw", qos);

        RCLCPP_INFO(this->get_logger(), "RGB URL: %s", rgb_.url.c_str());
        RCLCPP_INFO(this->get_logger(), "Thermal URL: %s", thermal_.url.c_str());

        running_ = true;

        thread_rgb_ = std::thread( &CameraNode::captureLoop, this, std::ref(rgb_), pub_rgb_);

        thread_thermal_ = std::thread(&CameraNode::captureLoop, this, std::ref(thermal_), pub_thermal_);

        RCLCPP_INFO( this->get_logger(), "CameraNode started | fail_threshold=%d | cooldown=%lds", fail_threshold_, static_cast<long>(reconnect_cooldown_.count()));
    }

    ~CameraNode() override
    {
        RCLCPP_INFO(this->get_logger(), "CameraNode shutting down...");

        running_ = false;

        {
            std::lock_guard<std::mutex> lock(rgb_.mtx);
            rgb_.cap.release();
        }

        {
            std::lock_guard<std::mutex> lock(thermal_.mtx);
            thermal_.cap.release();
        }

        if (thread_rgb_.joinable())
        {
            thread_rgb_.join();
        }

        if (thread_thermal_.joinable())
        {
            thread_thermal_.join();
        }

        RCLCPP_INFO(this->get_logger(), "CameraNode stopped.");
    }

private:
    static std::string getRosEncoding(const cv::Mat & frame)
    {
        switch (frame.channels())
        {
            case 1:
                return sensor_msgs::image_encodings::MONO8;

            case 3:
                return sensor_msgs::image_encodings::BGR8;

            case 4:
                return sensor_msgs::image_encodings::BGRA8;

            default:
                return sensor_msgs::image_encodings::BGR8;
        }
    }

    bool cooldownElapsed(const StreamHandle & s) const
    {
        if (!s.last_reconnect_attempt.has_value())
        {
            return true;
        }

        return (std::chrono::steady_clock::now() - *s.last_reconnect_attempt) >= reconnect_cooldown_;
    }

    // Chỉ gọi hàm này khi đã giữ s.mtx
    void openStreamUnlocked(StreamHandle & s)
    {
        s.last_reconnect_attempt = std::chrono::steady_clock::now();

        RCLCPP_INFO( this->get_logger(), "[%s] Trying to open stream...", s.name.c_str());

        RCLCPP_INFO(this->get_logger(), "[%s] URL: %s", s.name.c_str(), s.url.c_str());

        s.cap.release();
        s.fail_count = 0;

        const bool ok = s.cap.open(s.url, cv::CAP_GSTREAMER);

        RCLCPP_INFO(
            this->get_logger(),"[%s] cap.open() result=%d | isOpened=%d", s.name.c_str(), static_cast<int>(ok), static_cast<int>(s.cap.isOpened()));

        if (ok && s.cap.isOpened())
        {
            if (!s.was_opened)
            {
                RCLCPP_INFO(this->get_logger(), "[%s] Stream opened", s.name.c_str());
            }
            else
            {
                RCLCPP_INFO( this->get_logger(), "[%s] Reconnect OK", s.name.c_str());
            }

            s.was_opened = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),"[%s] Cannot open stream, retry after %lds...",
                s.name.c_str(),static_cast<long>(reconnect_cooldown_.count()));
        }
    }

    void captureLoop(StreamHandle & s,rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub)
    {


        RCLCPP_INFO(this->get_logger(),"[%s] Capture thread started", s.name.c_str());

        while (running_)
        {
            bool stream_opened = false;

            {
                std::lock_guard<std::mutex> lock(s.mtx);

                if (!s.cap.isOpened())
                {
                    if (cooldownElapsed(s))
                    {
                        openStreamUnlocked(s);
                    }
                }

                stream_opened = s.cap.isOpened();
            }

            if (!stream_opened)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,"[%s] Stream is not opened yet", s.name.c_str());

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            cv::Mat frame;
            bool ok = false;

            {
                std::lock_guard<std::mutex> lock(s.mtx);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s] Before cap.read()", s.name.c_str());

                ok = s.cap.read(frame);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),*this->get_clock(),2000,"[%s] After cap.read() | ok=%d | empty=%d",
                    s.name.c_str(), static_cast<int>(ok),  static_cast<int>(frame.empty()));
            }

            if (ok && !frame.empty())
            {
                s.fail_count = 0;

                const std::string encoding = getRosEncoding(frame);

                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                //     "[%s] Got frame | width=%d | height=%d | channels=%d | encoding=%s",
                //     s.name.c_str(), frame.cols,frame.rows, frame.channels(), encoding.c_str());

                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = s.frame_id;

                auto msg = cv_bridge::CvImage(header, encoding, frame).toImageMsg();

                pub->publish(*msg);

                static thread_local int frame_count = 0;
                static thread_local auto last_time = std::chrono::steady_clock::now();

                frame_count++;

                auto now = std::chrono::steady_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_time).count();

                if (elapsed_ms >= 1000)
                {
                    double fps = frame_count * 1000.0 / elapsed_ms;

                    RCLCPP_INFO(this->get_logger(), "[%s] FPS = %.2f", s.name.c_str(), fps);
                    frame_count = 0;
                    last_time = now;
                }



                // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,
                //     "[%s] Published image | frame_id=%s | data_size=%zu bytes",
                //     s.name.c_str(), msg->header.frame_id.c_str(), msg->data.size());
            }
            else
            {
                s.fail_count++;

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),*this->get_clock(),1000, "[%s] Read failed | ok=%d | empty=%d | fail_count=%d/%d",
                    s.name.c_str(), static_cast<int>(ok), static_cast<int>(frame.empty()), s.fail_count,fail_threshold_);

                if (s.fail_count >= fail_threshold_)
                {
                    RCLCPP_WARN(this->get_logger(),"[%s] Stream disconnected, reconnecting...", s.name.c_str());
                    std::lock_guard<std::mutex> lock(s.mtx);
                    openStreamUnlocked(s);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(),"[%s] Capture thread stopped", s.name.c_str());
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

    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}