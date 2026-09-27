# sac_autonomy

Hardware-independent ROS 2 (Jazzy) packages of the **SAC** car: they only talk through the
topics and frames below, so the same code runs on the real car and in the simulation.

| Where | Workspace | Provides the interface |
|---|---|---|
| Real car (Jetson Orin) | [Vehicle-Workspace](https://github.com/ITU-EMAV/Vehicle-Workspace) | [sac_drivers](https://github.com/ITU-EMAV/sac_drivers) |
| Simulation (Gazebo) | [Docker-Workspaces](https://github.com/ITU-EMAV/Docker-Workspaces) | [gazebo_environment](https://github.com/ITU-EMAV/gazebo_environment) |

Both workspaces include this repository as a git submodule under `src/`.

## Packages
| Package | Content |
|---|---|
| `sac_description` | URDF of the car: dimensions, limits, roof rack and sensor frames. The one model of the car; the simulation adds its Gazebo sensors on top of it. |
| `sac_planning` | the path to follow. Method so far: `route_planner`, a route given on the world map (latitude/longitude waypoints, see [Routes](#routes)). |
| `sac_control` | follows the path: `pure_pursuit` (steering) with a speed profile from the path's curvature |
| `sac_bringup` | `autonomy.launch.py`: planning and control together |

Planned: `sac_localization`, `sac_perception`.

What belongs here: anything that runs from topics alone. Anything that opens a device, a
serial port or a network socket to hardware belongs in `sac_drivers`; anything Gazebo-specific
in `gazebo_environment`.

## Driving a route
With the simulation (or the car's drivers) running:
```bash
ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true            # simulation
ros2 launch sac_bringup autonomy.launch.py                               # real car
ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true route:=my_route.geojson
```
The car drives the site's route (`sac_planning/config/<site>.yaml`, default `site:=sonoma`);
a loop is driven lap after lap, an open route to its end. Speed limits and the lookahead are
in `sac_control/config/pure_pursuit.yaml` (10 m/s, 3 m/s^2 in corners by default). Stop it
with Ctrl+C: the car gets a stop command.

## Routes
A route is a list of waypoints on the world map, so the same file works in the simulation and
on the real car. Files live in `sac_planning/routes/`:
- `.geojson`: a LineString, or Point features in driving order. Draw one on
  [geojson.io](https://geojson.io) (satellite view, "Draw a LineString") and save it.
- `.csv`: one `latitude,longitude` per line.

A route whose last point is within 5 m of its first is a loop. The planner rounds the corners
between waypoints and resamples the route every 0.5 m. Waypoints every ~10 m follow the
track closely; in tight corners put them closer.

The site config holds the **datum**: the latitude/longitude of the map frame's origin and its
heading. In the simulation it is the Gazebo world's `<spherical_coordinates>`; on the real
car the localization must use the same datum, so a route lands on the same place in both.

`sonoma_lap.geojson` is the middle of the simulated Sonoma track, one lap from the start line
(3.1 km, 313 waypoints, at least 3 m from the asphalt edge everywhere). It was made from the
track model, so it follows the simulated asphalt, which lies within ~5 m of the real one.

## Interface
The contract between the car (or the simulation) and the packages here. Both sides must keep
these names, types and meanings; a change here is a change in `sac_drivers` and
`gazebo_environment` too.

### Sensors (provided by the drivers / the simulation)
| Topic | Type | frame_id | Notes |
|---|---|---|---|
| `/sac/sensors/front_camera/image` | `sensor_msgs/Image` | `front_camera_left_camera_optical_frame` | ZED 2 rectified left image |
| `/sac/sensors/front_camera/depth_image` | `sensor_msgs/Image` | `front_camera_left_camera_optical_frame` | 32FC1 metres; +inf beyond range |
| `/sac/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | `front_camera_left_camera_optical_frame` | |
| `/sac/sensors/front_camera/points` | `sensor_msgs/PointCloud2` | `front_camera_left_camera_optical_frame` | |
| `/sac/sensors/front_camera/imu` | `sensor_msgs/Imu` | `front_camera_imu_link` | ZED 2 IMU |
| `/sac/sensors/middle_imu/imu` | `sensor_msgs/Imu` | `middle_imu_frame` | middle of the car |
| `/sac/sensors/front_imu/imu` | `sensor_msgs/Imu` | `front_imu_frame` | over the front axle |
| `/sac/sensors/back_imu/imu` | `sensor_msgs/Imu` | `back_imu_frame` | over the rear axle |
| `/sac/sensors/navsat_front_right/navsat` | `sensor_msgs/NavSatFix` | `navsat_front_right_frame` | front-right antenna |
| `/sac/sensors/navsat_rear_left/navsat` | `sensor_msgs/NavSatFix` | `navsat_rear_left_frame` | rear-left antenna |
| `/sac/sensors/roof_lidar/points` | `sensor_msgs/PointCloud2` | `roof_lidar_frame` | VLP-16, returns only (no NaN/inf points) |

### Vehicle
| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/sac/actuators/cmd_vel` | `geometry_msgs/Twist` | to the car | `linear.x` speed [m/s] (negative: reverse), `angular.z` yaw rate [rad/s]. The car turns them into a steering angle with its wheel base. |
| `/joint_states` | `sensor_msgs/JointState` | from the car | wheel speeds (`rear_*_wheel_joint`, velocity [rad/s]) and steering angles (`front_*_wheel_steering_joint`, position [rad]); joint names from `sac_description` |
| `/sac/calculations/steering_odom` | `nav_msgs/Odometry` | from the car | wheel odometry, `odom` -> `base_footprint` |

### Planning and control (inside sac_autonomy)
| Topic | Type | Meaning |
|---|---|---|
| `/sac/planning/path` | `nav_msgs/Path` | the path to follow, in `map`, latched; a pose every ~0.5 m; a loop does not repeat its first pose. Every planning method publishes here. |
| `/sac/planning/route_geojson` | `foxglove_msgs/GeoJSON` | the route on the world map, for a Map panel |
| `/sac/control/lookahead` | `geometry_msgs/PointStamped` | the point the controller steers to |
| `/sac/control/cross_track_error` | `std_msgs/Float64` | distance of the rear axle from the path [m], positive left |

Safety: the real car's vehicle interface stops the car when no `cmd_vel` arrives for 0.5 s.
The simulation keeps driving with the last command, so controllers must publish continuously.

### Frames
REP 105: `map -> odom -> base_footprint -> ...`
- `base_footprint -> sensors`: `robot_state_publisher` with `sac_description`, on both sides.
- `map -> odom -> base_footprint`: the localization. The simulation can publish it from
  Gazebo's exact pose instead (`ground_truth_tf:=true`, its default).
- Drivers must not publish TF of their own for these frames (e.g. the ZED wrapper's URDF and
  positional tracking), or a frame gets two parents.

### Time
The simulation runs on `/clock` (`use_sim_time:=true`); the real car on system time. Nodes
here take `use_sim_time` as a parameter and never assume either.

## Build
```bash
colcon build --symlink-install --packages-up-to sac_description
ros2 launch sac_description display.launch.py   # the model in RViz, with joint sliders
```
