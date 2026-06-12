#ifndef OFFBOARD_CONTROL_BATTERY_MONITOR_HPP
#define OFFBOARD_CONTROL_BATTERY_MONITOR_HPP

// Pure battery-safety debounce — NO ROS dependency, so it can be unit-tested standalone.
// A genuinely depleted pack must stay below the danger threshold CONTINUOUSLY for hold_sec
// before we emergency-land; a transient sag (high current during takeoff/climb, recovers in
// 1-3 s) does NOT trigger. The node feeds one battery sample per callback and acts on the
// verdict (log it, or land). Invalid samples (0 V / NaN before the first valid reading) are
// ignored. This is the exact logic that used to live inline in OffboardMode::battery_cb.
class BatteryMonitor
{
public:
    struct Params
    {
        double min_voltage = 13.2;   // V: danger threshold (3.3 V/cell, 4S)
        double hold_sec    = 5.0;    // V must stay below threshold this long before landing
    };
    Params params;

    enum class Verdict
    {
        OK,              // nothing to do
        WATCH_STARTED,   // voltage just dropped below threshold — debounce timer started
        EMERGENCY_LAND   // below threshold continuously for hold_sec — land now
    };

    // Feed one battery sample. armed + landing_started gate when a low actually matters;
    // now_sec is a monotonic seconds timestamp from the node clock. Resets the debounce timer
    // whenever the voltage recovers (or the gate opens). Invalid samples (v <= 1 / non-finite)
    // are ignored and return OK without disturbing the timer.
    Verdict update(double voltage, bool armed, bool landing_started, double now_sec);

    void reset();                              // clear the debounce timer (call on a new flight)
    double voltage() const { return last_voltage_; }   // last VALID voltage seen

private:
    double low_since_sec_ = -1.0;              // -1 = not currently below threshold
    double last_voltage_  = 0.0;
};

#endif // OFFBOARD_CONTROL_BATTERY_MONITOR_HPP
