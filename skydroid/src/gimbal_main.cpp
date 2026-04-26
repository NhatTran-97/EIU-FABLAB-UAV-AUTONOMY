#include "skydroid/skydroid_driver.hpp"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

static const std::string SKY_IP   = "192.168.144.108";
static const std::string LOCAL_IP = "192.168.144.10";

static void sleep_s(double s) {
    std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

int main() {
    std::cout << ">>> Skydroid Driver Test <<<\n";

    SkydroidDriver drv(SKY_IP, 5000, LOCAL_IP, 5000);

    // Test 1: Connection
    std::cout << "\n=== Test 1: Connection ===\n";
    auto version = drv.check_connection();
    if (!version) {
        std::cout << "Connection failed\n";
        return 1;
    }
    std::cout << "Connection OK — Firmware: " << *version << "\n";

    // Test 2: PTZ
    std::cout << "\n=== Test 2: PTZ ===\n";
    for (auto& [action, label] : std::vector<std::pair<std::string, std::string>>{
        {"up", "tilt up"}, {"stop", "stop"}, {"down", "tilt down"}, {"stop", "stop"}, {"center", "center"}
    }) {
        std::cout << "  -> " << label << "\n";
        drv.g_ptz(action);
        sleep_s(0.8);
    }
    std::cout << "PTZ OK\n";

    // Test 3: Speed
    std::cout << "\n=== Test 3: Speed ===\n";
    drv.g_speed(0, -90);  sleep_s(1.0);
    drv.g_speed(0,  90);  sleep_s(1.0);
    drv.g_speed(120, 0);  sleep_s(1.0);
    drv.g_speed(-120, 0); sleep_s(1.0);
    drv.g_speed(120,  120); sleep_s(1.0);
    drv.g_speed(-120, -120); sleep_s(1.0);
    drv.g_speed();
    std::cout << "Speed OK\n";

    // Test 4: Angle
    std::cout << "\n=== Test 4: Angle ===\n";
    drv.g_angle(NAN, 0.0);
    sleep_s(0.5);
    drv.g_angle(NAN, -30.0);
    sleep_s(1.0);
    drv.g_angle(NAN, 30.0, 3.0, NAN, 13.0);
    sleep_s(1.0);
    drv.g_angle(NAN, 0.0);

    drv.g_angle(0.0,   NAN, 3.0, 10.0); sleep_s(1.0);
    drv.g_angle(45.0,  NAN, 3.0, 10.0); sleep_s(1.0);
    drv.g_angle(-45.0, NAN, 3.0, 10.0); sleep_s(1.0);
    drv.g_angle(0.0,   NAN, 3.0, 10.0); sleep_s(1.0);

    drv.g_angle(30.0, -30.0, 3.0, 5.0, 2.0); sleep_s(1.0);
    drv.g_angle(0.0,   0.0,  3.0, 5.0, 2.0);
    std::cout << "Angle OK\n";

    // Test 5: Attitude push
    std::cout << "\n=== Test 5: Attitude push (5 seconds) ===\n";
    drv.g_enable_attitude_push(1);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    int count = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto att = drv.recv_attitude(0.5);
        if (!att) continue;
        ++count;
        std::cout << std::fixed << std::setprecision(2)
                  << "  Y=" << std::setw(7) << att->yaw_deg
                  << "  P=" << std::setw(7) << att->pitch_deg
                  << "  R=" << std::setw(7) << att->roll_deg << "\n";
    }
    drv.g_enable_attitude_push(0);
    std::cout << "Attitude OK — " << count << " packets received\n";

    drv.g_ptz("center");
    sleep_s(1.0);

    std::cout << "\n>>> Done <<<\n";
    return 0;
}
