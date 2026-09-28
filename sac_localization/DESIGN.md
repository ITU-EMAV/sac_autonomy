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
| | `dynamic_bicycle` | bicycle with tyre slip (linear tyres, cornering stiffness), backward Euler, blended into the kinematic model below 2 m/s; same inputs |
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

The full simulation configs are `config/sim_local.yaml` and `config/sim_global.yaml`; in short:

```yaml
localization_global:
  ros__parameters:
    frequency: 50.0                 # [Hz] update and output rate
    world_frame: map                # map: global filter, odom: local filter
    base_frame: base_footprint
    odom_frame: odom                # the global filter publishes map -> odom
    history: 1.0                    # [s] how late a measurement may be
    datum: {latitude: 38.1628083, longitude: -122.4579944, altitude: 0.0, heading: 0.83}
    initial_pose_topic: /initialpose

    estimator:
      type: ekf                     # ekf | iekf | ukf
    motion_model:
      type: constant_acceleration   # constant_acceleration | imu_driven | kinematic_bicycle
      acceleration_noise: [3.0, 3.0, 1.0]       # random walk per sqrt(s), x y z
      angular_velocity_noise: [0.3, 0.3, 0.6]
      gyro_bias_noise: 1.0e-4
    initial_state:
      pose_from: gnss               # origin | config | gnss | initial_pose
      gnss_duration: 5.0            # [s] averaging the antennas while standing
    initial_covariance: {position: 0.25, roll_pitch: 0.01, yaw: 0.05}
    recovery_covariance: {position: 4.0, yaw: 0.25}

    sensors:
      names: [middle_imu, wheels, nonholonomic, gnss_front_right, gnss_rear_left]
      middle_imu:
        type: imu
        topic: /sac/sensors/middle_imu/imu
        use: [angular_velocity, linear_acceleration]
        angular_velocity_covariance: [1.0e-5, 1.0e-5, 1.0e-5]
        linear_acceleration_covariance: [0.01, 0.01, 0.01]
        angular_velocity_rejection_threshold: 50.0
      wheels:
        type: wheel
        topic: /joint_states
        covariance: [0.01, 0.001]   # speed [m/s]^2, yaw rate [rad/s]^2
      nonholonomic:
        type: nonholonomic
        lever_arm: [-0.9365, 0.0, 0.0]   # rear axle
      gnss_front_right:
        type: gnss_position
        topic: /sac/sensors/navsat_front_right/navsat
        covariance: [1.0, 1.0, 2.25]     # east, north, up [m^2]
        max_rejections_in_a_row: 10
      gnss_rear_left: {type: gnss_position, topic: /sac/sensors/navsat_rear_left/navsat}

    outputs:
      tf: true
      odometry: /sac/localization/odometry
      fix: /sac/localization/fix
      status: /sac/localization/status
      ground_truth: /sac/ground_truth/pose   # simulation only
      error: /sac/localization/error
```

Common options of every topic-based sensor: `topic`, `frame` (overrides frame_id), `use`,
a covariance per quantity (overrides the message's) or `covariance_scale`,
`rejection_threshold` (and per quantity for the IMU), `max_rejections_in_a_row`,
`max_delay`, `as_input`, `enabled`. The adapters' header
(`sac_localization_adapters/adapters.hpp`) lists each adapter's own parameters.

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

## Status

Implemented and tested (16 unit tests in `test/`, plus the simulation):
- core: state on a manifold, geodesy, all measurement models (analytic Jacobians checked
  against numeric ones), the fuser (late measurements are replayed; too late ones dropped;
  a clock jump back restarts the filter)
- engines: `ekf`, `iekf` (iterated EKF), `ukf`
- motion models: `constant_acceleration`, `imu_driven`, `kinematic_bicycle`, `dynamic_bicycle`
- adapters: `imu`, `gnss_position`, `wheel`, `zero_velocity`, `nonholonomic`, `odometry`,
  `twist`, `pose`
- node: local and global instances, start from the GNSS antennas (position and yaw from the
  baseline), `/initialpose`, `~/reset`, live tuning of adapter noise and the motion model
- outputs: TF, odometry, fix, status, and in the simulation the error against Gazebo
- configs `config/sim_{local,global}.yaml`, overlays `config/models/*.yaml`,
  `launch/localization.launch.py`; `sac_bringup`'s `autonomy.launch.py localization:=true`

Not yet: the `heading`, `magnetometer`, `gnss_velocity`, `geo_pose` and vendor adapters (no
source for them in the simulation), and the real car's `car_*.yaml`.

### Results in the simulation
One recorded lap at 10 m/s (Sonoma, GNSS noise 0.3 m horizontal, rigid car), replayed through every
combination. Error of the global filter against Gazebo's exact pose, after the first 5 s:

| Engine + motion model | Position mean | Position p95 | Position max | Yaw p95 |
|---|---|---|---|---|
| ekf + constant_acceleration | 0.069 m | 0.136 m | 0.22 m | 0.48 deg |
| iekf + constant_acceleration | 0.069 m | 0.136 m | 0.22 m | 0.48 deg |
| ukf + constant_acceleration | 0.069 m | 0.136 m | 0.22 m | 0.48 deg |
| ekf / iekf / ukf + imu_driven | 0.069 m | 0.135 m | 0.22 m | 0.49 deg |
| ekf / iekf / ukf + kinematic_bicycle | 0.088 m | 0.197 m | 0.30 m | 0.56 deg |
| ekf, GNSS arriving 0.2 s late | 0.074 m | 0.146 m | 0.24 m | 0.48 deg |

- Once converged the engines agree to the millimetre: with this much GNSS the problem is
  nearly linear. They differ at the start: from a 30 degree yaw error the yaw is within 2
  degrees after 0.9 s (iekf), 1.0 s (ekf), 1.1 s (ukf).
- `kinematic_bicycle` is the worst: it trusts the wheels for the whole prediction.
- Late GNSS is handled by the replay: 0.2 s late (2 m at 10 m/s) costs 5 mm.
- Defaults: `ekf` + `constant_acceleration` (as good as any, the cheapest).

### Robustness: a bump that made the filter lose itself
Over the crest after Sonoma's first hairpin the simulated car (no suspension) takes off and
lands hard; the landing turns it by ~20 degrees within a quarter of a second and slides it
sideways at 1.4 m/s. The first version lost itself there, twice while driving on it:
1. The gyros measured the real turn, but the motion model did not expect it, so they were
   rejected as outliers (all three IMUs, 0.75 s): 23 degrees of yaw error.
2. With that yaw the GNSS disagreed and was rejected as well, for good: the error grew by
   2.5 m/s and the controller stopped the car (4 m from the path).

Fixes, both general:
- The IMU adapter fuses gyro and accelerometer as separate measurements with their own
  thresholds: gyros high (50 sigma: they measure real rotations), accelerometers 5 sigma
  (impacts spike them).
- `max_rejections_in_a_row` (set for the GNSS): after that many rejections in a row the
  filter widens its position and yaw uncertainty (`recovery_covariance`) so it takes the
  sensor again, instead of rejecting it for ever. On the real car this also covers GNSS
  jumps after tunnels or multipath.

On the same recording the fixed filter stays within 0.56 m and 3.8 degrees through the bump
and is back to ~0.1 m after 3 s.

### Driving on it
The car driving full laps of Sonoma on the global filter's `map -> base_footprint`
(simulation with `ground_truth_tf:=false`, pure pursuit):

| Lap | Localization error mean / p95 / max | Yaw p95 | Distance from the path p95 / max |
|---|---|---|---|
| 10 m/s, GNSS 1 m (a standalone receiver, as on the real car) | 0.16 / 0.37 / 0.63 m | 0.7 deg | 0.36 / 1.26 m |
| 10 m/s, GNSS 0.3 m | 0.07 / 0.15 / 0.34 m | 0.6 deg | 0.25 / 0.73 m |

The first laps (rigid simulated car) failed on the crest after the first hairpin: the car
took off at 10 m/s and the landing threw it 1.5-4 m sideways or rolled it over, whatever
drove it. With the suspension added to sac_description and Gazebo (springs and dampers on
the wheels) the car stays on the ground there. The tables above use GNSS noise of 0.3 m for
the replayed comparison and 1 m for `config/sim_global.yaml` now.

### Dynamic bicycle model (tyre slip)
With the simulated tyres slipping (gazebo_environment's WheelSlip), a lap on wet tyres
replayed through the motion models (GNSS 1 m):

| Motion model | Position mean / p95 | Yaw mean / p95 |
|---|---|---|
| constant_acceleration | 0.24 / 0.52 m | 0.57 / 1.63 deg |
| kinematic_bicycle | 0.25 / 0.52 m | 0.59 / 1.64 deg |
| dynamic_bicycle (dry tyre values) | 0.24 / 0.51 m | 0.38 / 0.95 deg |
| dynamic_bicycle (wet tyre values) | 0.25 / 0.52 m | 0.37 / 0.93 deg |

Modelling the slip cuts the yaw error by 40 %, even with the wrong tyre values.

Driving on the first version failed (the car was stopped over the crest after the first
hairpin, or left the track). A recorded failing run showed three problems, none in the tyre
model itself:
1. **The TF chain.** The global filter computed map -> odom with the local filter's latest
   odom -> base when the one at its own stamp was not there yet; the chain the controller
   reads was then off by the car's motion in between, up to 1.4 m (5.4 m live) while the
   estimate itself was 0.14 m off. The controller's 4 m safety stop fired. Fix: the global
   filter buffers the local filter's odometry (`outputs.local_odometry`) and interpolates it,
   or extrapolates it a few ms with its twist, to its own stamp; the controller now stops only
   if the car stays off the path for 0.5 s.
2. **The wheel speed as the model's input.** Braking (after that stop) locked the rear wheels:
   they read 4 m/s while the car still slid at 7 m/s, and an input cannot be rejected, so the
   estimate stopped 3.5 m before the car. Fix: `speed_from: state`; the speed is a state and
   the wheel speed a measurement (`as_input` for the steering, `also_measure`).
3. **Wheel data during slip.** Even as a measurement, a locking wheel pulls the estimate
   along step by step (each step a small innovation), and the standstill detector fired on
   the locked wheels. Fix: the wheel adapter skips its data while the wheels' acceleration
   differs from the IMU's (`slip_check_imu`), and standstill needs a still IMU too
   (`imu_topic`). Those checks are in the configs for every motion model.

On the failing recording: error through the braking slide 0.52 m instead of 3.7 m; the whole
run 0.13 m mean, 0.61 m max instead of 0.22 / 3.8 m. Driving on it, a full lap at 10 m/s
(dry, GNSS 1 m):

| Motion model | Position mean / p95 / max | Yaw p95 | Distance from the path p95 / max |
|---|---|---|---|
| dynamic_bicycle | 0.21 / 0.42 / 0.69 m | 0.8 deg | 0.47 / 1.19 m |
| constant_acceleration | 0.21 / 0.40 / 0.84 m | 1.4 deg | 0.41 / 1.02 m |

constant_acceleration stays the default as it needs no car parameters; dynamic_bicycle
(`motion_model:=dynamic_bicycle`) needs the car's mass, yaw inertia, axle distances and
cornering stiffness (config/models/dynamic_bicycle.yaml), to be measured on the real car.

## Implementation order (see Status for what is done)

1. Core: `State`, geodesy (checked against sac_planning), measurement models with numeric
   vs analytic Jacobian tests, `Fuser`.
2. `ekf` + `constant_acceleration`; adapters `imu`, `gnss_position`, `wheel`, `nonholonomic`;
   the node with TF, odometry, fix and status outputs. Replaces `odometry_tf` in the
   simulation (`ground_truth_tf:=false`); measure the error against Gazebo.
3. Local + global instances; late measurements; `initial_pose`, reset.
4. `ukf`, `imu_driven`, `kinematic_bicycle`; compare them on the same bag.
5. Remaining adapters (`odometry`, `twist`, `pose`, `heading`, `magnetometer`,
   `gnss_velocity`, `zero_velocity`), then vendor adapters for the car's GNSS receiver.
