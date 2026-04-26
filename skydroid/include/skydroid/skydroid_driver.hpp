#pragma once
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <cmath>
#include <netinet/in.h>

struct Frame {
    std::string header;
    char src;
    char dst;
    int length;
    char control;
    std::string cmd;
    std::string data;
    bool crc_ok;
};

struct Attitude {
    double yaw_deg;
    double pitch_deg;
    double roll_deg;
};

// ---------- UDP Transport ----------

class UdpTransport {
public:
    UdpTransport(const std::string& local_ip, int local_port,
                 const std::string& remote_ip, int remote_port,
                 double timeout = 1.0);
    ~UdpTransport();

    void send(const std::vector<uint8_t>& packet);
    std::optional<std::string> recv(double timeout = -1.0);
    void close();

private:
    int sock_ = -1;
    struct sockaddr_in remote_addr_;
    double default_timeout_;

    void set_timeout(double secs);
};

// ---------- Skydroid Driver ----------

class SkydroidDriver {
public:
    SkydroidDriver(const std::string& sky_ip, int sky_port = 5000,
                   const std::string& local_ip = "0.0.0.0", int local_port = 5000,
                   double timeout = 1.0);
    ~SkydroidDriver();

    void g_ptz(const std::string& action);
    void g_speed(int yaw = 0, int pitch = 0);
    void g_angle(double yaw = NAN, double pitch = NAN,
                 double speed_dps = 3.0,
                 double yaw_speed_dps = NAN,
                 double pitch_speed_dps = NAN);
    void g_enable_attitude_push(int freq_hz);

    std::optional<Attitude> recv_attitude(double timeout = -1.0);
    std::optional<std::string> check_connection();
    void close();

private:
    UdpTransport transport_;
    double timeout_;

    std::optional<Frame> send_cmd(const std::string& body, double delay = 0.1, bool expect_reply = true);
    std::optional<Attitude> parse_gac_packet(const std::string& reply);

    static std::string calc_crc(const std::string& body);
    static std::vector<uint8_t> build_cmd_packet(const std::string& body);
    static std::optional<Frame> decode_frame(const std::string& resp);

    static int    clamp_int(int v, int lo, int hi);
    static double clamp_dbl(double v, double lo, double hi);
    static std::string s8_hex(int v);
    static std::string angle_hex(double deg);
    static std::string speed_hex(double dps);

    static const std::map<std::string, std::string> PTZ_CODES;
};
