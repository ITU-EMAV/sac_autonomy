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

Planned: `sac_localization`, `sac_perception`, `sac_planning`, `sac_control`, `sac_bringup`.

What belongs here: anything that runs from topics alone. Anything that opens a device, a
serial port or a network socket to hardware belongs in `sac_drivers`; anything Gazebo-specific
in `gazebo_environment`.

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
