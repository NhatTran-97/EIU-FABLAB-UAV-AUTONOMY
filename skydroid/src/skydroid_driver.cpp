#include "skydroid/skydroid_driver.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <iostream>

// ================================================================
// UdpTransport
// ================================================================

UdpTransport::UdpTransport(const std::string& local_ip, int local_port,
                           const std::string& remote_ip, int remote_port,
                           double timeout) : default_timeout_(timeout)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0)
        throw std::runtime_error("Failed to create UDP socket");

    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port   = htons(local_port);
    inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr);

    if (bind(sock_, reinterpret_cast<struct sockaddr*>(&local_addr), sizeof(local_addr)) < 0)
    {
        throw std::runtime_error("Failed to bind UDP socket to " + local_ip);
    }
        

    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port   = htons(remote_port);
    inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr);

    set_timeout(timeout);
}

UdpTransport::~UdpTransport() { close(); }

void UdpTransport::set_timeout(double secs) 
{
    struct timeval tv;
    tv.tv_sec  = static_cast<long>(secs);
    tv.tv_usec = static_cast<long>((secs - tv.tv_sec) * 1e6);
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void UdpTransport::send(const std::vector<uint8_t>& packet) 
{
    sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<const struct sockaddr*>(&remote_addr_), sizeof(remote_addr_));
}

std::optional<std::string> UdpTransport::recv(double timeout) 
{
    if (timeout >= 0.0)
        set_timeout(timeout);

    char buf[4096];
    struct sockaddr_in sender{};
    socklen_t sender_len = sizeof(sender);
    ssize_t n = recvfrom(sock_, buf, sizeof(buf) - 1, 0, reinterpret_cast<struct sockaddr*>(&sender), &sender_len);

    if (timeout >= 0.0)
        set_timeout(default_timeout_);

    if (n <= 0) return std::nullopt;
    return std::string(buf, static_cast<size_t>(n));
}

void UdpTransport::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

// ================================================================
// SkydroidDriver — protocol helpers
// ================================================================

const std::map<std::string, std::string> SkydroidDriver::PTZ_CODES = {
    {"stop",   "00"}, {"up",     "01"}, {"down",   "02"},
    {"left",   "03"}, {"right",  "04"}, {"center", "05"},
    {"follow", "06"}, {"lock",   "07"},
};

std::string SkydroidDriver::calc_crc(const std::string& body) 
{
    int sum = 0;
    for (unsigned char c : body) sum += c;
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (sum & 0xFF);
    return oss.str();
}

std::vector<uint8_t> SkydroidDriver::build_cmd_packet(const std::string& body) 
{
    std::string pkt = body + calc_crc(body);
    return std::vector<uint8_t>(pkt.begin(), pkt.end());
}

std::optional<Frame> SkydroidDriver::decode_frame(const std::string& resp) 
{
    if (resp.size() < 12) return std::nullopt;
    try {
        Frame f;
        f.header  = resp.substr(0, 3);
        f.src     = resp[3];
        f.dst     = resp[4];
        f.length  = std::stoi(resp.substr(5, 1), nullptr, 16);
        f.control = resp[6];
        f.cmd     = resp.substr(7, 3);
        f.data    = resp.substr(10, resp.size() - 12);

        std::string body     = resp.substr(0, resp.size() - 2);
        std::string recv_crc = resp.substr(resp.size() - 2);
        for (char& c : recv_crc) c = static_cast<char>(std::toupper(c));
        f.crc_ok = (recv_crc == calc_crc(body));
        return f;
    } catch (...) {
        return std::nullopt;
    }
}

// ================================================================
// SkydroidDriver — gimbal helpers
// ================================================================

int SkydroidDriver::clamp_int(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

double SkydroidDriver::clamp_dbl(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

std::string SkydroidDriver::s8_hex(int v) {
    int raw = clamp_int(v, -127, 127) & 0xFF;
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << raw;
    return oss.str();
}

std::string SkydroidDriver::angle_hex(double deg) {
    int raw = static_cast<int>(std::round(clamp_dbl(deg, -90.0, 90.0) * 100)) & 0xFFFF;
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << raw;
    return oss.str();
}

std::string SkydroidDriver::speed_hex(double dps) {
    int raw = clamp_int(static_cast<int>(std::round(dps * 10)), 0, 99);
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << raw;
    return oss.str();
}

// ================================================================
// SkydroidDriver — public API
// ================================================================

SkydroidDriver::SkydroidDriver(const std::string& sky_ip, int sky_port,
                               const std::string& local_ip, int local_port,
                               double timeout)
    : transport_(local_ip, local_port, sky_ip, sky_port, timeout)
    , timeout_(timeout)
{}

SkydroidDriver::~SkydroidDriver() { close(); }

void SkydroidDriver::close() { transport_.close(); }

void SkydroidDriver::g_ptz(const std::string& action) {
    auto it = PTZ_CODES.find(action);
    if (it == PTZ_CODES.end())
        throw std::invalid_argument("Unknown PTZ action: " + action);
    send_cmd("#TPUG2wPTZ" + it->second, 0.1, false);
}

void SkydroidDriver::g_speed(int yaw, int pitch) {
    if (yaw == 0 && pitch == 0)
        send_cmd("#TPUG4wGSM0000", 0.1, false);
    else if (pitch == 0)
        send_cmd("#TPUG2wGSY" + s8_hex(yaw), 0.1, false);
    else if (yaw == 0)
        send_cmd("#TPUG2wGSP" + s8_hex(pitch), 0.1, false);
    else
        send_cmd("#TPUG4wGSM" + s8_hex(yaw) + s8_hex(pitch), 0.1, false);
}

void SkydroidDriver::g_angle(double yaw, double pitch, double speed_dps, double yaw_speed_dps, double pitch_speed_dps)
{
    if (std::isnan(yaw) && std::isnan(pitch))
        throw std::invalid_argument("At least one of yaw or pitch must be provided");

    std::string y_spd = speed_hex(std::isnan(yaw_speed_dps)   ? speed_dps : yaw_speed_dps);
    std::string p_spd = speed_hex(std::isnan(pitch_speed_dps) ? speed_dps : pitch_speed_dps);

    if (!std::isnan(yaw) && !std::isnan(pitch))
        send_cmd("#TPUGCwGAM" + angle_hex(yaw) + y_spd + angle_hex(pitch) + p_spd, 0.1, false);
    else if (!std::isnan(yaw))
        send_cmd("#TPUG6wGAY" + angle_hex(yaw) + y_spd, 0.1, false);
    else
        send_cmd("#TPUG6wGAP" + angle_hex(pitch) + p_spd, 0.1, false);
}

void SkydroidDriver::g_enable_attitude_push(int freq_hz) {
    std::string data;
    if (freq_hz == 0) {
        data = "00";
    } else {
        freq_hz = clamp_int(freq_hz, 1, 100);
        std::ostringstream oss;
        oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << freq_hz;
        data = oss.str();
    }
    send_cmd("#TPUG2wGAA" + data, 0.1, false);
}

std::optional<Attitude> SkydroidDriver::parse_gac_packet(const std::string& reply) {
    size_t idx = reply.find("GAC");
    if (idx == std::string::npos) return std::nullopt;
    if (idx + 3 + 12 > reply.size()) return std::nullopt;

    std::string data = reply.substr(idx + 3, 12);

    auto hex4_to_deg = [](const std::string& h) -> double {
        int v = std::stoi(h, nullptr, 16);
        if (v & 0x8000) v -= 0x10000;
        return v / 100.0;
    };

    try {
        return Attitude{
            hex4_to_deg(data.substr(0, 4)),
            hex4_to_deg(data.substr(4, 4)),
            hex4_to_deg(data.substr(8, 4))
        };
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<Attitude> SkydroidDriver::recv_attitude(double timeout) {
    auto raw = transport_.recv(timeout);
    if (!raw) return std::nullopt;
    return parse_gac_packet(*raw);
}

std::optional<Frame> SkydroidDriver::send_cmd(const std::string& body, double delay, bool expect_reply) {
    transport_.send(build_cmd_packet(body));

    if (!expect_reply) {
        std::this_thread::sleep_for(std::chrono::duration<double>(delay));
        return std::nullopt;
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_);
    std::optional<Frame> frame;

    while (std::chrono::steady_clock::now() < deadline) 
    {
        double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0.0) break;

        auto raw = transport_.recv(remaining);
        if (!raw) break;

        if (parse_gac_packet(*raw)) continue;  // skip async attitude packet

        frame = decode_frame(*raw);
        if (frame) break;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(delay));
    return frame;
}

std::optional<std::string> SkydroidDriver::check_connection() 
{
    auto frame = send_cmd("#TPUD2rVER00");
    if (frame && frame->cmd == "VER" && frame->crc_ok)
        return frame->data;
    return std::nullopt;
}
