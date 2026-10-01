# sac_local_planner: design

Follows the global route (`/sac/planning/path`) around the obstacles of the perception grid
(`/sac/perception/grid`), or stops before what it cannot pass, and gives the controller a
short trajectory with a speed at each point (`/sac/planning/trajectory`). Pieces are
plugins set in YAML, like sac_localization and sac_perception.

```
route (map) ─► Frenet frame ─► generator (plugin) ─► candidates ─► collision check ─► costs (plugins) ─► the cheapest free one
grid (odom) ─► distance map ─────────────────────────────────────┘                                          + speed profile ─► trajectory
```

## Why not a search on the grid
A* on the grid gives jagged paths a car at 10 m/s cannot drive, and forgets the route
(Hybrid A* drives them, but is for low speeds, like parking). Road and racing cars sample
paths around the route instead: in the route's Frenet frame (s along it, d to its left) a
quintic d(s) shifts from the car's offset to a target offset (Werling et al., ICRA 2010, the
lateral part). Each candidate is checked against the grid's distance map, O(1) per point.

## Pieces
- `ReferencePath`: the route as a Frenet frame (loops and open routes; a search window
  around the last s, so a route passing close to itself is followed in order).
- Generator `frenet_lattice`: targets from -max_offset to +max_offset (3.5 m, the route
  keeps 3 m from the asphalt's edge) every 0.5 m, over shift lengths of speed x
  [1.5, 2.5, 3.5] s (at least 6 m), to a horizon of speed x 4 s (at least 30 m): 45
  candidates.
- Collision: the car as three circles (0.95 m) along its body; a point collides where a
  circle comes within 0.3 m of an occupied cell. The distance map is the perception
  package's exact distance transform of the grid.
- Costs (each times its weight): `obstacle_clearance` (closer than 1 m beyond the
  footprint), `offset_from_route`, `lateral_acceleration`, `consistency` (the target offset
  against the last cycle's).
- Choice: the cheapest free candidate; with none free, the one blocked furthest away, driven
  with a stop `stop_margin` (3 m) before the obstacle.
- Speed: the curvature limit (3 m/s^2), braking towards lower limits and a stop (4 m/s^2),
  accelerating from the car's speed (2 m/s^2), as sac_control's limits.
- Safety: no grid for `max_grid_age` (1 s), no TF: a stopping trajectory. The controller
  (pure_pursuit with `input: trajectory`) stops when the trajectory is older than 0.5 s.

## Results in the simulation
Ground truth TF, dry tyres, 10 m/s. `SIM_LAUNCH_ARGS="obstacles:=on_route"`, then
`autonomy.launch.py use_sim_time:=true local_planner:=true`.

**Obstacles on the route** (`on_route`: boxes, parked cars, a cone slalom, on the driving
line or up to 1.5 m off it), 5.4 km (1.7 laps):

| | |
|---|---|
| obstacles passed | 9 of 9, no collision |
| closest between the car's body and an obstacle | 0.67 - 1.47 m |
| offset from the route, \|d\| | median 0.20 m, p95 1.68 m, max 2.73 m |
| speed | 10 m/s kept; no stop while driving |
| planning | 0.85 ms mean, 6 ms max (45 candidates); distance map 2 ms |
| cycles with no free candidate | 10 of 5968 (brief, the car did not stop) |

The cycles with no free candidate came from a bridge over the track: its deck 8 m up passed
as ground far out in the ground filter and its railings as obstacles in the lane (see
sac_perception's DESIGN.md). With that fixed, 4.8 km: 9 of 7200 cycles, all 37-40 m before
the bridge, no stop, no collision, the obstacles passed at 0.72-1.30 m.

With the consistency weight at 2 the car kept a 0.5 m offset after an obstacle (changing
the target cost more than the offset); at 0.5 it goes back to the route and still keeps its
side around an obstacle.

**A closed road** (`blocked`: a 20 m wall across the track 150 m after the start): from
10 m/s the car stops with 3.8 m between its body and the wall (stop_margin 3 m + the 0.3 m
safety margin), no collision.

## Moving objects: yielding to people crossing
The grid shows where things are; for what moves (`/sac/perception/objects`, the ones called
moving, held 1.5 s after they were last seen so: a track may drop out for a moment) the
planner looks at where they will be:
- each candidate is driven in time with its free speed profile; a point collides where a
  moving object, going on at its velocity, comes within the footprint, its own radius, 0.5 m
  and its growing uncertainty (0.3 m/s, at most 1.5 m) within 1 s of when the car would be
  there, up to 6 s ahead: one walking along the road is passed on its free side
- yielding: one moving across the route (faster than 0.3 m/s across it) that is in the
  corridor (4 m each side) or enters it before the car could be there blocks every candidate
  where it crosses: the car stops `stop_margin` before, and drives on once they have left
  the corridor; it does not swerve in front of them
Unit tests: yields to a person crossing 22 m ahead, drives on once they have left the
corridor, passes one walking towards it along the road, does not stop for one who comes after
the car is past.

`pedestrians:=crossing` (gazebo_environment), four people crossing as the car comes 35-40 m
near, a lap (perception's DESIGN.md for how they are found): the car stopped before each of
the walkers 4.4-5.9 m away from them, and slowed for the runner (2.5 m/s), passing 7.5 m
behind; 2 stops (2.2 s) with nobody within 30 m.
