// STL-50B2 lidar driver (port of lidar_scripts/stl_50b2_node.py).
// Reads 47-byte packets from the serial port, assembles a 360-degree LaserScan and publishes it on /scan.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

constexpr size_t kPacketSize = 47;
constexpr int kPointsPerPacket = 12;

std::array<uint8_t, 256> generate_crc8_table(uint8_t poly = 0x4D)
{
  std::array<uint8_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    uint8_t crc = static_cast<uint8_t>(i);
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ poly) : static_cast<uint8_t>(crc << 1);
    }
    table[i] = crc;
  }
  return table;
}

const std::array<uint8_t, 256> kCrc8Table = generate_crc8_table();

uint8_t calc_crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = kCrc8Table[crc ^ data[i]];
  }
  return crc;
}

inline uint16_t read_u16_le(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B0;
  }
}

struct ScanPoint
{
  double angle_rad;
  double distance;
  uint8_t intensity;
};

}  // namespace

class STL50B2Driver : public rclcpp::Node
{
public:
  STL50B2Driver()
  : Node("stl_50b2_driver")
  {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud_rate", 230400);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_link");

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    connect_serial();
  }

  ~STL50B2Driver() override
  {
    is_running_ = false;
    if (reading_thread_.joinable()) {
      reading_thread_.join();
    }
    if (fd_ >= 0) {
      const uint8_t stop_cmd[] = {0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A};
      if (write(fd_, stop_cmd, sizeof(stop_cmd)) < 0) {
        RCLCPP_WARN(get_logger(), "Failed to send Stop Command.");
      }
      close(fd_);
      fd_ = -1;
      RCLCPP_INFO(get_logger(), "Sent Stop Command and closed serial port.");
    }
  }

private:
  bool open_port()
  {
    const speed_t speed = baud_to_speed(baud_);
    if (speed == B0) {
      RCLCPP_ERROR(get_logger(), "Unsupported baud rate: %d", baud_);
      return false;
    }

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void connect_serial()
  {
    if (!open_port()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to Lidar on %s at %d baud.", port_.c_str(), baud_);

    const uint8_t start_cmd[] = {0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};
    if (write(fd_, start_cmd, sizeof(start_cmd)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to send Start Command: %s", std::strerror(errno));
    } else {
      RCLCPP_INFO(get_logger(), "Sent Start Command to Lidar.");
    }

    reading_thread_ = std::thread(&STL50B2Driver::read_loop, this);
  }

  void read_loop()
  {
    std::vector<uint8_t> buffer;
    uint8_t chunk[512];

    while (is_running_ && rclcpp::ok()) {
      pollfd pfd{fd_, POLLIN, 0};
      const int ret = poll(&pfd, 1, 100);
      if (ret < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "Error reading serial: %s", std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      if (ret == 0) {
        continue;
      }

      const ssize_t n = read(fd_, chunk, sizeof(chunk));
      if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "Error reading serial: %s", std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      buffer.insert(buffer.end(), chunk, chunk + n);

      while (buffer.size() >= kPacketSize) {
        // Find the header 0x54 0x2C
        size_t header_idx = std::string::npos;
        for (size_t i = 0; i + 1 < buffer.size(); ++i) {
          if (buffer[i] == 0x54 && buffer[i + 1] == 0x2C) {
            header_idx = i;
            break;
          }
        }

        if (header_idx == std::string::npos) {
          // Keep the last byte in case it is 0x54
          buffer.erase(buffer.begin(), buffer.end() - 1);
          break;
        }
        if (header_idx > 0) {
          buffer.erase(buffer.begin(), buffer.begin() + header_idx);
        }
        if (buffer.size() < kPacketSize) {
          break;
        }

        uint8_t packet[kPacketSize];
        std::memcpy(packet, buffer.data(), kPacketSize);
        buffer.erase(buffer.begin(), buffer.begin() + kPacketSize);

        if (calc_crc8(packet, kPacketSize - 1) == packet[kPacketSize - 1]) {
          parse_packet(packet);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "CRC Mismatch! Dropping packet.");
        }
      }
    }
  }

  void parse_packet(const uint8_t * packet)
  {
    // packet[0]: 0x54, packet[1]: 0x2C
    // speed (packet[2:4], deg/s) and timestamp (packet[44:46], ms) are not used.
    const double start_angle = read_u16_le(packet + 4) / 100.0;
    const double end_angle = read_u16_le(packet + 42) / 100.0;

    double diff = end_angle - start_angle;
    if (diff < 0) {
      diff += 360.0;
    }
    const double step = diff > 0 ? diff / 11.0 : 0.0;

    for (int i = 0; i < kPointsPerPacket; ++i) {
      const int idx = 6 + i * 3;
      const double distance = read_u16_le(packet + idx) / 1000.0;  // mm -> m
      const uint8_t intensity = packet[idx + 2];

      double angle_deg = start_angle + step * i;
      if (angle_deg >= 360.0) {
        angle_deg -= 360.0;
      }
      const double angle_rad = angle_deg * M_PI / 180.0;

      // Full turn detected
      if (angle_deg < last_angle_ && std::abs(angle_deg - last_angle_) > 180.0) {
        publish_scan();
        scan_points_.clear();
      }

      if (distance > 0) {
        scan_points_.push_back({angle_rad, distance, intensity});
      }
      last_angle_ = angle_deg;
    }
  }

  void publish_scan()
  {
    if (scan_points_.empty()) {
      return;
    }

    std::sort(
      scan_points_.begin(), scan_points_.end(),
      [](const ScanPoint & a, const ScanPoint & b) {return a.angle_rad < b.angle_rad;});

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.angle_min = static_cast<float>(scan_points_.front().angle_rad);
    msg.angle_max = static_cast<float>(scan_points_.back().angle_rad);
    if (scan_points_.size() > 1) {
      msg.angle_increment =
        (msg.angle_max - msg.angle_min) / static_cast<float>(scan_points_.size() - 1);
    } else {
      msg.angle_increment = 0.0f;
    }

    msg.time_increment = 0.0f;
    msg.scan_time = 0.15f;  // ~6.7Hz
    msg.range_min = 0.05f;
    msg.range_max = 12.0f;

    msg.ranges.reserve(scan_points_.size());
    msg.intensities.reserve(scan_points_.size());
    for (const auto & p : scan_points_) {
      msg.ranges.push_back(static_cast<float>(p.distance));
      msg.intensities.push_back(static_cast<float>(p.intensity));
    }

    scan_pub_->publish(msg);
  }

  std::string port_;
  int baud_;
  std::string frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  int fd_{-1};
  std::thread reading_thread_;
  std::atomic<bool> is_running_{true};

  // Accessed only from reading_thread_
  std::vector<ScanPoint> scan_points_;
  double last_angle_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<STL50B2Driver>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
