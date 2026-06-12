#include "offboard_control/carrot_follower.hpp"
#include <cmath>

void CarrotFollower::begin(const std::vector<Waypoint> &wps, double ground_z,
                           double start_x, double start_y, double start_z)
{
    wps_      = wps;
    ground_z_ = ground_z;
    wp_index_ = 0;
    carrot_x_ = start_x;
    carrot_y_ = start_y;
    carrot_z_ = start_z;
}

CarrotFollower::Result CarrotFollower::update(double drone_x, double drone_y, double drone_z,
                                              double wp_elapsed_sec)
{
    Result r;
    r.wp_count = wps_.size();

    if (wp_index_ >= wps_.size())
    {
        // No active WP (should not happen — the node lands on MISSION_COMPLETE). Report done.
        r.event    = Event::MISSION_COMPLETE;
        r.wp_index = wps_.empty() ? 0 : wps_.size() - 1;
        return r;
    }

    const Waypoint &cur = wps_[wp_index_];
    const double wp_x = cur.x;
    const double wp_y = cur.y;
    const double wp_z = ground_z_ + cur.z;   // z is relative to takeoff EKF z

    // Advance the carrot toward the active WP at a fixed speed.
    const double step = params.cruise_speed * params.dt;   // carrot advance per tick

    double vx = wp_x - carrot_x_;
    double vy = wp_y - carrot_y_;
    double vz = wp_z - carrot_z_;
    double d  = std::sqrt(vx*vx + vy*vy + vz*vz);

    bool carrot_at_wp = false;
    if (d <= step || d < 1e-6)
    {
        carrot_x_ = wp_x; carrot_y_ = wp_y; carrot_z_ = wp_z;
        carrot_at_wp = true;
    }
    else
    {
        carrot_x_ += vx / d * step;
        carrot_y_ += vy / d * step;
        carrot_z_ += vz / d * step;
    }

    // Clamp so the carrot never leads the drone by more than carrot_lead.
    double lx = carrot_x_ - drone_x;
    double ly = carrot_y_ - drone_y;
    double lz = carrot_z_ - drone_z;
    double lead = std::sqrt(lx*lx + ly*ly + lz*lz);
    if (lead > params.carrot_lead && lead > 1e-6)
    {
        carrot_x_ = drone_x + lx / lead * params.carrot_lead;
        carrot_y_ = drone_y + ly / lead * params.carrot_lead;
        carrot_z_ = drone_z + lz / lead * params.carrot_lead;
    }

    r.x = carrot_x_; r.y = carrot_y_; r.z = carrot_z_;
    r.qx = cur.qx; r.qy = cur.qy; r.qz = cur.qz; r.qw = cur.qw;

    // Distance from the DRONE (not the carrot) to the active WP, for arrival logic.
    const double ddx = drone_x - wp_x;
    const double ddy = drone_y - wp_y;
    const double ddz = drone_z - wp_z;
    r.drone_dist = std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);

    const bool timedout = wp_elapsed_sec > params.wp_timeout;
    const bool is_final = (wp_index_ + 1 >= wps_.size());
    r.wp_index = wp_index_;   // index acted on (before any advance)

    if (is_final)
    {
        // Final WP: wait for the drone to actually arrive, then signal completion.
        if (r.drone_dist < params.reach_radius || timedout)
        {
            r.event      = Event::MISSION_COMPLETE;
            r.by_timeout = timedout;
        }
    }
    else if (carrot_at_wp || timedout)
    {
        // Intermediate WP: once the carrot flowed through it (fly-through) or on timeout,
        // advance to the next WP — the carrot keeps moving (no full stop).
        r.event      = Event::WAYPOINT_PASSED;
        r.by_timeout = timedout;
        ++wp_index_;
    }
    return r;
}
