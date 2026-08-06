#!/usr/bin/env bash
set -euo pipefail

replace_agent=false
if [[ "${1:-}" == "--replace-agent" ]]; then
  replace_agent=true
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--replace-agent]" >&2
  exit 2
fi

ssh_host="${STM32_JETSON_HOST:-damiao}"
device="${STM32_CDC_DEVICE:-/dev/ttyACM0}"

ssh "$ssh_host" bash -s -- "$device" "$replace_agent" <<'REMOTE'
set -euo pipefail
device="$1"
replace_agent="$2"

test -c "$device" || { echo "Missing CDC device: $device" >&2; exit 1; }
mapfile -t agents < <(pgrep -x micro_ros_agent || true)
if (( ${#agents[@]} > 0 )); then
  if [[ "$replace_agent" != true ]]; then
    echo "An existing micro_ros_agent is running; inspect it or rerun with --replace-agent." >&2
    pgrep -a micro_ros_agent >&2
    exit 1
  fi
  pkill -x micro_ros_agent
  sleep 1
fi

set +u
source /opt/ros/humble/setup.bash
source /home/jialiang/Project/microros_ws/install/setup.bash
set -u
agent_prefix="$(ros2 pkg prefix micro_ros_agent)"
agent_bin="$agent_prefix/lib/micro_ros_agent/micro_ros_agent"
log_file=/tmp/rebotarm_microros_agent.log
: > "$log_file"
"$agent_bin" serial --dev "$device" -v4 >"$log_file" 2>&1 &
agent_pid=$!
cleanup() { kill "$agent_pid" 2>/dev/null || true; wait "$agent_pid" 2>/dev/null || true; }
trap cleanup EXIT

sleep 8
nodes="$(ros2 node list)"
topics="$(ros2 topic list -t)"
printf 'NODES\n%s\nTOPICS\n%s\n' "$nodes" "$topics"
grep -qx '/stm32h723_motor_bridge' <<<"$nodes"
grep -q '/stm32/joint_commands \[microros_stm32h723_bridge/msg/JointCommand\]' <<<"$topics"
grep -q '/stm32/joint_states \[sensor_msgs/msg/JointState\]' <<<"$topics"
grep -q '/stm32/status \[microros_stm32h723_bridge/msg/HardwareStatus\]' <<<"$topics"

topic_info="$(ros2 topic info /stm32/joint_states)"
publisher_count="$(awk '/Publisher count:/ {print $3}' <<<"$topic_info")"
if [[ "$publisher_count" != 1 ]]; then
  echo "Refusing rate validation: /stm32/joint_states has $publisher_count publishers." >&2
  echo "Stop mock/duplicate publishers, then rerun." >&2
  ros2 topic info -v /stm32/joint_states >&2
  exit 1
fi

echo "JOINT_STATE_SAMPLE"
timeout 5 ros2 topic echo --once /stm32/joint_states
echo "JOINT_STATE_HZ"
timeout 6 ros2 topic hz /stm32/joint_states || true
REMOTE
