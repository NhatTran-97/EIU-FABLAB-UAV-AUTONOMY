# How the drone reacts to obstacles

VFH+ does not push the drone away from obstacles. On every scan it picks a free direction that still leads towards the goal. What you see therefore depends on what the drone is doing at that moment.

## While flying a mission leg (avoidance ticked at Start mission)

**Someone stands in the way: the drone goes around.**

- Farther than `block_distance` (3 m): the drone only slows down, by an amount that depends on the distance and on the measured latency.
- Within 3 m: the sector towards the goal is blocked. VFH+ takes the free gap closest to the goal direction, usually to the left or right of the person, and the status becomes AVOIDING.
- Once past, the goal direction is free again and the drone returns to the straight line (CLEAR).
- The drone backs away only when every direction in front of it is blocked, because a direction opposite to the goal has the highest cost.

**Someone walks right up to the drone (closer than 1.5 m = `drone_radius` + `safety_margin`): the drone moves to the other side.**

At this range each obstacle point is enlarged to a ±90° cone, which blocks the whole half towards the person. Only directions away from them are free, so the drone takes one. It looks like backing off, but only because no other way is left. If the drone is approaching faster than it can still stop, it brakes first (setpoint = current position) and turns afterwards.

**Boxed in on all sides: BLOCKED.**

The drone holds its position. After `blocked_timeout` (5 s) `mission_manager` applies `blocked_action`: `hold` (default) waits for the pilot to take over, `land` lands.

**Stuck in a U-shaped trap:** VFH+ can swing back and forth without reporting BLOCKED. If the distance to the waypoint does not shrink by `stuck_min_progress` (1 m) within `stuck_timeout` (15 s), `mission_manager` holds, as if blocked.

**No fresh lidar, pose or goal:** the node reports STALE and the drone holds until the data is back.

## While hovering

By default nothing happens: the avoidance node is disabled outside mission legs, so standing in front of a hovering drone does not move it.

With `hover_keep_distance` set (for example 2 m), `mission_manager` keeps that distance:

```
HOVER with hover_keep_distance > 0
  mission_manager ── /avoidance/enable = true, /mission/goal = hover point ──► obstacle_avoidance
  mission_manager ◄── /avoidance/status (min_distance, min_distance_bearing) ──┘
  if min_distance < hover_keep_distance:
      the hover point moves straight away from the nearest obstacle at 0.5 m/s,
      at most 3 m from where HOVER began
```

- Someone 1 m away with a 2 m setting: the drone moves away until it is 2 m from them and stops. The ground station shows "Obstacle 1.0 m: moving away".
- They keep walking towards it: the drone keeps moving, up to `hover_max_drift` (3 m) from where it started, and then stays there.
- They leave: the drone stays where it is and does not go back.
- Lidar data lost or STALE: the drone holds.

Only the nearest obstacle is known, so the drone does not check what is behind it while moving away. The 3 m drift limit bounds that risk. Keep-distance works only in HOVER, not during Take-off, Land, or while the pilot has control.

## Pilot flying (Position mode)

Neither of the above applies when the pilot flies. With `publish_obstacle_distance: true` and PX4 `CP_DIST` set, PX4 Collision Prevention stops the pilot from flying into an obstacle. It blocks movement towards the obstacle but does not move the drone away, and it works only in Position mode.

## Trying it safely

Test in SITL first: fly a single waypoint with avoidance ticked and place a box in the middle of the leg. Then hover with `hover_keep_distance` set and move a box towards the drone.
