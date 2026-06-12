#include "offboard_control/setpoint_streamer.hpp"

#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <chrono>

SetpointStreamer::SetpointStreamer(rclcpp::Node *node,
                                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                                   double rate_hz)
    : node_(node), pub_(std::move(pub)), rate_hz_(rate_hz)
{
}

SetpointStreamer::~SetpointStreamer()
{
    stop();
}

void SetpointStreamer::start()
{
    running_.store(true);
    thread_ = std::thread(&SetpointStreamer::loop, this);
}

void SetpointStreamer::stop()
{
    running_.store(false);
    if (thread_.joinable())
        thread_.join();
}

void SetpointStreamer::set_target(const geometry_msgs::msg::PoseStamped &p)
{
    { std::lock_guard<std::mutex> lk(mtx_); target_ = p; }
    streaming_.store(true);
}

void SetpointStreamer::set_streaming(bool on)
{
    streaming_.store(on);
}

void SetpointStreamer::try_set_realtime_priority()
{
    // Ask the OS to run this thread at real-time (SCHED_FIFO) priority so it is never
    // starved by other ROS work on the CPU-constrained Jetson. We ALWAYS yield via
    // sleep_until each cycle, so a high FIFO priority can't monopolise the core.
    // If the process lacks CAP_SYS_NICE (e.g. unprivileged container) this just fails
    // and we keep normal scheduling — still fine, just less hard a guarantee.
    sched_param sp{};
    sp.sched_priority = 80;   // high, but below typical kernel/IRQ threads
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc == 0)
        RCLCPP_INFO(node_->get_logger(),
                    "🧵 Setpoint streamer running at SCHED_FIFO priority %d.", sp.sched_priority);
    else
        RCLCPP_WARN(node_->get_logger(),
                    "🧵 Could not set RT priority for setpoint streamer (%s) — "
                    "using normal scheduling. Stream still runs, just without hard preemption.",
                    std::strerror(rc));
}

void SetpointStreamer::loop()
{
    // Stream the current setpoint to PX4 on a dedicated thread. It only ever reads a COPY of
    // target_, so the state machine stays single-threaded.
    try_set_realtime_priority();

    using clk = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / (rate_hz_ > 0.0 ? rate_hz_ : 20.0)));
    auto next = clk::now();

    while (rclcpp::ok() && running_.load())
    {
        next += period;

        if (streaming_.load() && pub_)
        {
            geometry_msgs::msg::PoseStamped p;
            { std::lock_guard<std::mutex> lk(mtx_); p = target_; }
            p.header.stamp = node_->now();
            pub_->publish(p);
            // Heartbeat: lets the control loop watchdog confirm the stream is alive.
            last_pub_ns_.store(node_->now().nanoseconds(), std::memory_order_relaxed);
        }

        // Deadline-based pacing: sleep to the NEXT tick (not "now + period"), so the rate
        // stays constant regardless of how long publish took. If we overran a full period
        // the stream was at risk — warn (rate-limited) and resync so we don't spiral.
        const auto t = clk::now();
        if (t < next)
        {
            std::this_thread::sleep_until(next);
        }
        else
        {
            const auto late_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(t - next).count();
            if (late_ms > 100)
                RCLCPP_WARN(node_->get_logger(),
                            "⚠️ Setpoint streamer overran by %ldms — stream rate dipped. "
                            "Check CPU load on the companion.", static_cast<long>(late_ms));
            next = t;   // resync schedule to current time
        }
    }
}
