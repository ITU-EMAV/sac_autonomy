# Safety-boundary actuator command. All physical values use SI units.
# source_stamp is the ROS/source time used for cross-node freshness checks.
# source_steady_time_ns is CLOCK_MONOTONIC time since boot and is only comparable
# between processes on this host/boot. validity_duration bounds command lifetime.

uint8 MODE_DISABLED=0
uint8 MODE_AUTONOMOUS=1
uint8 MODE_CONTROLLED_STOP=2
uint8 MODE_EMERGENCY_STOP=3

uint32 VALID_STEERING=1
uint32 VALID_THROTTLE=2
uint32 VALID_BRAKE=4
uint32 VALID_TARGET_SPEED=8
uint32 INTEGRITY_OK=2147483648

builtin_interfaces/Time source_stamp
builtin_interfaces/Time publication_stamp
uint64 source_steady_time_ns
uint64 sequence
builtin_interfaces/Duration validity_duration
uint8 mode
bool armed
bool enable
float64 steering_angle_rad
float64 steering_rate_radps
float64 throttle_normalized
float64 drive_torque_nm
float64 brake_normalized
float64 target_speed_mps
float64 target_acceleration_mps2
uint32 validity_flags
string source_id
