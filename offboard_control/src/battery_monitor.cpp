#include "offboard_control/battery_monitor.hpp"
#include <cmath>

void BatteryMonitor::reset()
{
    low_since_sec_ = -1.0;
}

BatteryMonitor::Verdict BatteryMonitor::update(double voltage, bool armed,
                                               bool landing_started, double now_sec)
{
    // Ignore invalid/placeholder readings (0 V or NaN before the first valid sample), otherwise
    // "0 < threshold" would latch an emergency landing while on the ground.
    if (!std::isfinite(voltage) || voltage <= 1.0)
        return Verdict::OK;
    last_voltage_ = voltage;

    // Only emergency-land while actually armed/flying and not already landing. Require the
    // voltage to stay below threshold CONTINUOUSLY for hold_sec before landing.
    if (armed && !landing_started && voltage < params.min_voltage)
    {
        if (low_since_sec_ < 0.0)
        {
            // First sample below threshold: start the sustained-low timer, don't land yet.
            low_since_sec_ = now_sec;
            return Verdict::WATCH_STARTED;
        }
        if (now_sec - low_since_sec_ >= params.hold_sec)
            return Verdict::EMERGENCY_LAND;   // sustained, not a transient sag
        return Verdict::OK;
    }

    // Voltage recovered (sag ended) or gate closed — reset so the next dip starts fresh.
    low_since_sec_ = -1.0;
    return Verdict::OK;
}
