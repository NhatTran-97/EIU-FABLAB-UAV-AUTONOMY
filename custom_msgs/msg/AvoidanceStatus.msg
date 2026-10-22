# Result of obstacle_avoidance (VFH+) for mission_manager.
# All angles (heading / bearing) are in the local NED frame: 0 = north, positive clockwise.

uint8 CLEAR=0      # path to the goal is free; safe_setpoint lies on the straight line to the goal
uint8 AVOIDING=1   # path to the goal is blocked; safe_setpoint follows the direction chosen by VFH+
uint8 BLOCKED=2    # no passable valley left -> the manager must HOLD
uint8 STALE=3      # /scan, pose, attitude or goal timed out -> the manager must HOLD

std_msgs/Header header
uint8 state

geometry_msgs/Point safe_setpoint   # carrot in the local NED frame (m); z = goal z
float32 selected_heading            # flight direction chosen by VFH+ (rad, NED)
float32 goal_heading                # direction from the drone to the goal (rad, NED)
float32 speed_limit                 # m/s that still allows stopping in time (from clearance and measured latency)
float32 clearance                   # distance to the (inflated) obstacle along the flight direction (m), inf if free
float32 min_distance                # nearest obstacle horizontally, any direction (m), inf if none
float32 min_distance_bearing        # direction of the nearest obstacle (rad, NED)

# ---- Measured at runtime, so receivers derive timeouts instead of hardcoding rates ----
float32 scan_period                 # measured /scan period, p90 (s); NaN if not known yet
float32 latency                     # p90 latency from lidar scan to status publish (s)
