# sac_localization design

State estimation for the SAC car that is configured, not programmed: sensors are listed in a
YAML file (any number of each kind), and the estimator (EKF, UKF, ...) and the motion model
are plugins that can be exchanged without touching the rest. It publishes the REP 105
frames the rest of sac_autonomy uses (`map -> odom -> base_footprint`), in the simulation
and on the real car alike.

Ideas taken from:
[robot_localization](https://github.com/cra-ros-pkg/robot_localization) (sensors from YAML,
two filter instances, fusing lagged data),
[fuse](https://github.com/locusrobotics/fuse) (sensor models, motion models and optimizers as
plugins),
[Autoware ekf_localizer](https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_ekf_localizer)
(delay compensation for fast vehicles),
[mherb/kalman](https://github.com/mherb/kalman) (one system model for EKF and UKF).

## Packages

| Package | Content | Depends on ROS |
|---|---|---|
| `sac_localization` | `core/`: state, models, estimators' interface, fuser, geodesy. `ros/`: the node, the adapter interface, outputs. The built-in estimators and motion models (plugins). | core: no (Eigen only); the rest: yes |
| `sac_localization_adapters` | sensor adapters (plugins): standard messages, and vendor messages (u-blox, ...) built only when those packages are installed | yes |
| `sac_localization_msgs` | `FilterStatus`, `SensorStatus`: per sensor received/accepted/rejected, innovation, Mahalanobis distance, delay | messages |

```
sac_localization/
  include/sac_localization/
    core/                     no ROS: unit tests and offline replay without a ROS graph
      state.hpp               State (named blocks on a manifold), StateLayout(Builder), Belief
      params.hpp              Params: plugin parameters without ROS
      measurement.hpp         MeasurementModel, Measurement, Input, Inputs, UpdateResult
      measurement_models.hpp  Position, Orientation, Yaw, BodyVelocity, WorldVelocity,
                              AngularVelocity, SpecificForce, MagneticField, Subset, Stacked
      motion_model.hpp        MotionModel (plugin base)
      estimator.hpp           Estimator (plugin base)
      fuser.hpp               Fuser: queues, history, late measurements
      geodesy.hpp             MapFrame: latitude/longitude <-> map (same as sac_planning)
    ros/
      sensor_adapter.hpp      SensorAdapter (plugin base), TopicAdapter<Msg>, AdapterContext
      outputs.hpp             Output: TF, odometry, fix, status, error (simulation)
      localization_node.hpp   LocalizationNode
  src/  estimators/ motion_models/ ...
  config/  sim_local.yaml sim_global.yaml car_local.yaml car_global.yaml
  launch/  localization.launch.py
  test/
```

## Data flow

```
 ROS messages ──> [SensorAdapter xN] ──Measurement / Input──> [Fuser] ──> [Outputs]
                   mounting from TF,                            │  history, replay of late data
                   noise, `use`, gating options                 ▼
                                                    [Estimator] predict / update
                                                        │ uses
                                          [MotionModel]   [MeasurementModel]
```

- **SensorAdapter** knows ROS and one message type, not the estimator. It reads the sensor's
  mounting from TF (sac_description), picks the fields the config lists in `use`, sets the noise,
  and builds a `Measurement` from the standard measurement models, or an `Input` for the
  motion model (`as_input: true`, e.g. an IMU driving the `imu_driven` model).
- **MeasurementModel**: `h(x)`, a residual (angles wrap), optionally an analytic Jacobian.
- **MotionModel**: `f(x, dt, u)`, process noise `Q`, optionally a Jacobian.
- **Estimator** only sees the three interfaces above. EKF uses Jacobians (numeric on the
  manifold when a model has none); UKF calls `f` and `h` on sigma points. Changing the engine
  is one line in the config.
- **Fuser** does the timing for every engine: fuses in stamp order, rewinds and replays for a
  late measurement (within `history`), drops older ones, splits long predictions.

## State

A state is a set of named blocks, each a vector or a 3D rotation; the covariance is in the
tangent space (rotations have 3 degrees of freedom) and estimators move between the two with
`boxplus` / `boxminus`. This is what lets EKF, error-state EKF and UKF share the models.

Core blocks, always present (world = map or odom, body = base_footprint):

| Block | Size | Frame |
|---|---|---|
| `position` | 3 | world [m] |
| `orientation` | rotation | body in world |
| `linear_velocity` | 3 | body [m/s] |
| `angular_velocity` | 3 | body [rad/s] |
| `linear_acceleration` | 3 | body, without gravity [m/s^2] |

Plugins add their own blocks, e.g. `middle_imu/gyro_bias`, `middle_imu/accel_bias`,
`wheels/scale` (wheel radius error), `magnetometer/offset`. So every IMU has its own bias,
and a sensor's calibration can be estimated online.

## Plugins

| Kind | Lookup name | What |
|---|---|---|
| Estimator | `ekf` | extended Kalman filter on the manifold (error-state) |
| | `ukf` | unscented Kalman filter on the manifold |
| | later: `iekf`, `particle`, a sliding-window optimizer | |
| MotionModel | `constant_acceleration` | 3D, no inputs; everything else from measurements |
| | `kinematic_bicycle` | 2D bicycle, input: wheel speed and steering (wheel adapter `as_input`) |
| | `imu_driven` | strapdown: an IMU's rates and specific force as input, biases in the state |
| SensorAdapter | `imu` | `sensor_msgs/Imu`: angular_velocity, linear_acceleration, orientation |
| | `gnss_position` | `sensor_msgs/NavSatFix` (antenna lever arm from TF; fix status gating) |
| | `gnss_velocity` | `geometry_msgs/TwistWithCovarianceStamped`, velocity over ground |
| | `wheel` | `sensor_msgs/JointState`: speed from wheel joints, yaw rate from steering joints |
| | `odometry` | `nav_msgs/Odometry`: twist; pose too, absolute or differential |
| | `twist` | `geometry_msgs/TwistWithCovarianceStamped` |
| | `pose` | `PoseWithCovarianceStamped`, `PoseStamped`, `TransformStamped`, a TF frame pair |
| | `geo_pose` | `geographic_msgs/GeoPose(WithCovariance)Stamped` (optional package) |
| | `heading` | `sensor_msgs/Imu` (orientation only), `geometry_msgs/QuaternionStamped` |
| | `magnetometer` | `sensor_msgs/MagneticField` |
| | `nonholonomic` | no topic: lateral and vertical body velocity are ~0 |
| | `zero_velocity` | from a wheel topic: speed 0 means standing still (also calibrates gyro biases) |
| | vendor | `gps_msgs/GPSFix`, `ublox_msgs/NavPVT`, `NavRELPOSNED9` (RTK heading), ... |

## Configuration

ROS 2 parameters cannot hold a list of maps, so sensors are named in `sensors.names` and
configured under `sensors.<name>`. Every noise value is a parameter that can be changed while
the node runs (`ros2 param set`, Lichtblick's parameter panel).

```yaml
localization_global:
  ros__parameters:
    frequency: 50.0                 # [Hz] update and output rate
    world_frame: map                # map: global filter, odom: local filter
    base_frame: base_footprint
    odom_frame: odom                # the global filter publishes map -> odom
    history: 1.0                    # [s] how late a measurement may be
    datum: {latitude: 38.1628083, longitude: -122.4579944, altitude: 0.0, heading: 0.83}

    estimator:
      type: ekf                     # ekf | ukf
    motion_model:
      type: constant_acceleration
      acceleration_noise: [1.0, 0.5, 0.2]     # [m/s^2/sqrt(s)] x y z
      angular_acceleration_noise: [0.2, 0.2, 0.5]
      bias_random_walk: {gyro: 1.0e-4, accel: 1.0e-3}
    initial_state:
      pose_from: first_gnss         # config | first_gnss | initial_pose topic
      covariance: {position: 100.0, yaw: 10.0, velocity: 1.0}

    sensors:
      names: [middle_imu, front_imu, gnss_front_right, gnss_rear_left, wheels, nonholonomic]
      middle_imu:
        type: imu
        topic: /sac/sensors/middle_imu/imu
        use: [angular_velocity, linear_acceleration]
        estimate_biases: true
      front_imu:
        type: imu
        topic: /sac/sensors/front_imu/imu
        use: [angular_velocity]
      gnss_front_right:
        type: gnss_position
        topic: /sac/sensors/navsat_front_right/navsat
        rejection_threshold: 5.0    # [sigma]
        max_delay: 0.3              # [s]
      gnss_rear_left:
        type: gnss_position
        topic: /sac/sensors/navsat_rear_left/navsat
      wheels:
        type: wheel
        topic: /joint_states
        speed_joints: [rear_left_wheel_joint, rear_right_wheel_joint]
        steering_joints: [front_left_wheel_steering_joint, front_right_wheel_steering_joint]
        wheel_radius: 0.30
        wheel_base: 1.873
        covariance: [0.05, 0.02]    # speed [m/s]^2, yaw rate [rad/s]^2
      nonholonomic:
        type: nonholonomic
        covariance: [0.01, 0.01]

    outputs:
      tf: true
      odometry: /sac/localization/odometry
      fix: /sac/localization/fix
      status: /sac/localization/status
      ground_truth: /sac/ground_truth/pose   # simulation only: publishes the error
```

Common options of every topic-based sensor: `topic`, `frame` (overrides frame_id), `use`,
`covariance` (overrides the message's, diagonal) or `covariance_scale`,
`rejection_threshold`, `max_delay`, `as_input`, `enabled`.

## Two instances (REP 105)

| Instance | world_frame | Sensors | Publishes |
|---|---|---|---|
| local | `odom` | IMUs, wheels, constraints | `odom -> base_footprint`: smooth, drifts, never jumps |
| global | `map` | the same plus GNSS, heading, poses | `map -> odom`, such that `map -> odom -> base_footprint` is its estimate |

The controller follows `map -> base_footprint`; short-term consumers (e.g. perception
tracking obstacles) can use `odom`.

## Fusing several sources of the same quantity

All are fused, each weighted by its covariance (three gyros, two GNSS antennas, ...). Two
rules for the config:
- Do not fuse the same underlying data twice (e.g. `wheel` and a `steering_odom` computed from
  the same wheels): the filter would trust it twice as much as it should.
- With two absolute heading sources that may disagree by a constant offset, use one of them
  in differential mode.

## Tuning and testing

- `FilterStatus` per sensor: innovation, Mahalanobis distance, accepted/rejected/late counts,
  delay. Plot them in Lichtblick while tuning.
- In the simulation the `ground_truth` output publishes position and yaw error against Gazebo.
- The core has no ROS: unit tests per model and estimator, and an offline tool that replays a
  bag through several configs (EKF vs UKF, noise sets) and compares them.

## Implementation order

1. Core: `State`, geodesy (checked against sac_planning), measurement models with numeric
   vs analytic Jacobian tests, `Fuser`.
2. `ekf` + `constant_acceleration`; adapters `imu`, `gnss_position`, `wheel`, `nonholonomic`;
   the node with TF, odometry, fix and status outputs. Replaces `odometry_tf` in the
   simulation (`ground_truth_tf:=false`); measure the error against Gazebo.
3. Local + global instances; late measurements; `initial_pose`, reset.
4. `ukf`, `imu_driven`, `kinematic_bicycle`; compare them on the same bag.
5. Remaining adapters (`odometry`, `twist`, `pose`, `heading`, `magnetometer`,
   `gnss_velocity`, `zero_velocity`), then vendor adapters for the car's GNSS receiver.
