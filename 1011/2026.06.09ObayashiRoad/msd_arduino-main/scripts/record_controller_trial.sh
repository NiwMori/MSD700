#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC="${1:-8}"
OUT_ROOT="${2:-${HOME}/msd_ws/controller_trials}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/controller_trial_${STAMP}"

TOPICS=(
  /msd/sbus/cmd_vel
  /msd/sbus/arm_cmd
  /msd/cmd_vel
  /msd/arm_cmd
  /msd/motor/encoder_tick
  /msd/motor/state
  /msd/cylinder/tick
  /msd/system_status
  /msd/hw_estop
)

source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/msd_ws/install/setup.bash" ]]; then
  source "${HOME}/msd_ws/install/setup.bash"
fi

mkdir -p "${OUT_ROOT}"

{
  echo "started_at: $(date --iso-8601=seconds)"
  echo "duration_sec: ${DURATION_SEC}"
  echo "output_dir: ${OUT_DIR}"
  echo "topics:"
  for topic in "${TOPICS[@]}"; do
    echo "  - ${topic}"
  done
  echo
  echo "ros_nodes:"
  ros2 node list || true
  echo
  echo "topic_info:"
  for topic in "${TOPICS[@]}"; do
    echo "--- ${topic}"
    ros2 topic info "${topic}" || true
  done
} | tee "${OUT_DIR}.txt"

echo "Recording controller trial rosbag to ${OUT_DIR}"
set +e
timeout --foreground --signal=INT --kill-after=5s "${DURATION_SEC}" \
  ros2 bag record -o "${OUT_DIR}" "${TOPICS[@]}" 2>&1 | tee "${OUT_DIR}.record.log"
record_status=${PIPESTATUS[0]}
set -e

if [[ "${record_status}" -ne 0 && "${record_status}" -ne 124 && "${record_status}" -ne 130 ]]; then
  echo "ros2 bag record failed with status ${record_status}" | tee -a "${OUT_DIR}.txt"
  exit "${record_status}"
fi

if [[ ! -d "${OUT_DIR}" ]]; then
  echo "ros2 bag directory was not created: ${OUT_DIR}" | tee -a "${OUT_DIR}.txt"
  echo "Check ${OUT_DIR}.record.log for rosbag errors." | tee -a "${OUT_DIR}.txt"
  exit 1
fi

echo "finished_at: $(date --iso-8601=seconds)" | tee -a "${OUT_DIR}.txt"
echo "Saved: ${OUT_DIR}"
