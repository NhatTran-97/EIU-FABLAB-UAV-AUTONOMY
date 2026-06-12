#ifndef OFFBOARD_CONTROL_CARROT_FOLLOWER_HPP
#define OFFBOARD_CONTROL_CARROT_FOLLOWER_HPP

#include <vector>
#include <cstddef>

// Pure waypoint-following geometry — NO ROS / MAVROS dependencies, so it can be unit-tested
// on its own. Advances a "carrot" (an intermediate setpoint) toward the active waypoint at a
// fixed speed, never letting it lead the drone by more than carrot_lead. PX4 then chases an
// always-near, smoothly-moving target => steady speed, smooth fly-through at corners, gentle
// stop at the final waypoint.
//
// The owning node feeds the live drone position + elapsed-in-waypoint time each tick and acts
// on the returned event (log it, reset its WP timer, or land). Keeping this class free of ROS
// types lets a different follower algorithm (pure-pursuit, spline, …) drop in by swapping it.
class CarrotFollower
{
public:
    struct Params
    {
        double cruise_speed = 2.0;   // m/s: carrot advance along the path
        double carrot_lead  = 2.5;   // m: max distance the carrot may lead the drone
        double reach_radius = 1.5;   // m: sphere within which the final WP is "reached"
        double wp_timeout   = 30.0;  // s: skip a WP if not reached within this time
        double dt           = 0.05;  // s: control-loop period (carrot advance per tick)
    };

    // A waypoint. x,y are absolute (EKF frame); z is RELATIVE to takeoff ground (ground_z is
    // added internally). q* is the desired orientation, copied straight into the setpoint.
    struct Waypoint { double x, y, z, qx, qy, qz, qw; };

    enum class Event { NONE, WAYPOINT_PASSED, MISSION_COMPLETE };

    struct Result
    {
        double x = 0, y = 0, z = 0;             // carrot setpoint position (EKF frame, absolute z)
        double qx = 0, qy = 0, qz = 0, qw = 1;  // setpoint orientation (from the active WP)
        Event  event = Event::NONE;
        bool   by_timeout = false;              // event fired because of wp_timeout (not arrival)
        std::size_t wp_index = 0;               // 0-based index of the WP the event refers to
        std::size_t wp_count = 0;
        double drone_dist = 0.0;                // drone's distance to the active WP
    };

    // Load a mission and seat the carrot at the drone's current position so it eases out from
    // the hover point instead of jumping straight to WP1. ground_z offsets every waypoint's z.
    void begin(const std::vector<Waypoint> &wps, double ground_z,
               double start_x, double start_y, double start_z);

    // One control tick. drone_* is the live EKF position; wp_elapsed_sec is seconds since the
    // active WP became active (the node resets its timer when a WAYPOINT_PASSED event fires).
    Result update(double drone_x, double drone_y, double drone_z, double wp_elapsed_sec);

    std::size_t index() const { return wp_index_; }
    std::size_t count() const { return wps_.size(); }
    bool        active() const { return wp_index_ < wps_.size(); }

    Params params;

private:
    std::vector<Waypoint> wps_;
    double ground_z_ = 0.0;
    double carrot_x_ = 0.0, carrot_y_ = 0.0, carrot_z_ = 0.0;
    std::size_t wp_index_ = 0;
};

#endif // OFFBOARD_CONTROL_CARROT_FOLLOWER_HPP
