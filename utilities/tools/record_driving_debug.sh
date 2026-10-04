#!/usr/bin/env bash
# TF/EKF/NDT and controller diagnosis. Host entry point for the sac container.
set -eo pipefail

# Pass script and QoS contents as arguments; no host/container path assumption.
if [[ "${DRIVING_DEBUG_IN_DOCKER:-0}" != 1 && "${1:-}" != --help && "${1:-}" != -h ]]; then
  script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
  docker_flags=(-i)
  if [[ -t 0 && -t 1 ]]; then docker_flags=(-it); fi
  exec docker exec "${docker_flags[@]}" -e DRIVING_DEBUG_IN_DOCKER=1 \
    -e "DRIVING_DEBUG_QOS=$(cat "$script_dir/record_driving_debug_qos.yaml")" \
    sac bash -c "$(cat "$script_dir/record_driving_debug.sh")" record_driving_debug "$@"
fi

usage() {
  cat <<'EOF'
Usage: record_driving_debug.sh [--sensors] [--dry-run] [OUTPUT_DIRECTORY]

Default: lightweight TF, localization, path, speed constraints, commands,
diagnostics, logs and timing topics. Ctrl+C finishes the bag.
--sensors: also record raw/filtered LiDAR, camera and road masks (larger bag).
IMU and wheel-speed feedback are included even without --sensors.
--dry-run: print the recording command without starting it.

Bag receipt timestamps use wall time, deliberately, so /clock pauses remain
visible. /clock and original message timestamps are recorded unchanged.
An existing output directory is never overwritten.
EOF
}

sensors=false
dry_run=false
output=""
while (($#)); do
  case "$1" in
    --sensors) sensors=true ;;
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    --*) echo "Unknown option: $1" >&2; exit 2 ;;
    *)
      if [[ -n "$output" ]]; then usage >&2; exit 2; fi
      output="$1"
      ;;
  esac
  shift
done

workspace_dir=/smart_car_ws
output=${output:-"$workspace_dir/bags/driving_debug_$(date +%Y%m%d_%H%M%S)"}
if [[ -e "$output" ]]; then
  echo "Output already exists: $output" >&2
  exit 2
fi

topics=(
  /clock /tf /tf_static /rosout /diagnostics /parameter_events
  /initialpose /initialpose3d
  /localization/ekf_localizer/pose_with_cov
  /localization/ekf_localizer/odom
  /localization/ekf_localizer/twist_with_cov
  /localization/ndt_localizer/pose_with_cov
  /localization/ndt_localizer/initial_pose
  /localization/twist_with_covariance /vehicle/twist_with_covariance
  /localization/online/pose /zed/zed_node/imu/data /zed/zed_node/odom
  /sensing/gnss/gnss_pose_with_cov
  /debug/processing_time_ms
  /localization/ndt_localizer/debug/exe_time_ms
  /localization/ndt_localizer/debug/iteration_num
  /localization/ndt_localizer/debug/transform_probability
  /localization/ndt_localizer/debug/nearest_voxel_transformation_likelihood
  /localization/ndt_localizer/debug/initial_to_result_distance
  /localization/ndt_localizer/debug/initial_to_result_relative_pose
  /reactive_planner/trajectory /trajectory_planner/trajectory
  /control/controller_state /control/controller_command /control/timing
  /control/longitudinal_command /cmd_vel
  /controller/lookahead
  /planning/traffic/speed_constraint /planning/road_hazard/speed_constraint
  /planning/map/speed_constraint /planning/dynamic/speed_constraint
  /vehicle/speed /encoder_speed /vehicle/actuator_command
  /safety/command_guardian/status /yolo_detections /stop
  /perception/road/timing /perception/signs/timing
  /perception/dynamic_objects/timing /perception/road_hazards/timing
  /perception/tracking/timing
)
if "$sensors"; then
  topics+=(
    /velodyne_points /sensing/points_filtered
    /zed/zed_node/left/image_rect_color /zed/zed_node/left/camera_info
    /perception/road/drivable_mask /perception/road/lane_mask
  )
fi

# Keep discovery enabled: nodes/topics started after recording are included.
# No compression during capture, to avoid adding CPU load to the diagnosis.
qos_file=$(mktemp /tmp/driving_debug_qos.XXXXXX.yaml)
printf '%s\n' "$DRIVING_DEBUG_QOS" > "$qos_file"
trap 'rm -f -- "$qos_file"' EXIT
command=(ros2 bag record --storage sqlite3 --max-bag-size 1073741824
  --max-cache-size 104857600 --qos-profile-overrides-path
  "$qos_file" -o "$output" "${topics[@]}")
if "$dry_run"; then
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

if [[ -r /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -r "$workspace_dir/install/setup.bash" ]]; then
  source "$workspace_dir/install/setup.bash"
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS 2 unavailable. Run this script inside the sac container." >&2
  exit 1
fi
mkdir -p -- "$(dirname -- "$output")"
echo "Recording to: $output"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; stop with Ctrl+C."
"${command[@]}"
