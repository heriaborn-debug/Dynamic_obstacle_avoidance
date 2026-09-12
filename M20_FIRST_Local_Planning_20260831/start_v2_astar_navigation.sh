#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/nvidia/m20_v2_nx
PID_FILE=${ROOT}/run/v2_astar_navigation.pid
LOG_FILE=${ROOT}/logs/v2_astar_navigation.log
MOTION_PID_FILE=/home/nvidia/m20_motion/bridge.pid

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/m20_fastdds.xml

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/autoware_humble_official_full/install/setup.bash
source ${ROOT}/install/setup.bash
set -u

mkdir -p ${ROOT}/run ${ROOT}/logs /home/nvidia/m20_motion

for topic in /map /lio/robo/odom /m20/local_mapping/traversability/obstacle_cloud; do
  count=$(ros2 topic info "${topic}" 2>/dev/null | awk '/Publisher count:/ {print $3}')
  if [[ ! "${count:-0}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: required input has no publisher: ${topic}" >&2
    exit 1
  fi
done

tf_probe=$(timeout 3 ros2 run tf2_ros tf2_echo world base_link_dog 2>&1 || true)
if ! grep -q 'Translation:' <<<"${tf_probe}"; then
  echo 'ERROR: TF world -> base_link_dog is unavailable' >&2
  exit 1
fi

if [[ -f ${PID_FILE} ]] && kill -0 "$(<${PID_FILE})" 2>/dev/null; then
  echo "V2 A* navigation is already running (pid $(<${PID_FILE}))"
  exit 0
fi

# The production bridge is part of the navigation stack. Keep it disabled
# while the planners are starting so a stale command can never reach the dog.
if ! pgrep -f '[m]20_ros_bridge.py' >/dev/null; then
  nohup setsid /home/nvidia/m20_motion/navigation_control.sh start \
    >/home/nvidia/m20_motion/bridge.log 2>&1 </dev/null &
  echo $! >${MOTION_PID_FILE}
  sleep 2
fi
timeout 5 ros2 service call /m20/enable_motion std_srvs/srv/SetBool '{data: false}' \
  >/dev/null 2>&1 || true

nohup setsid ros2 launch m20_navigation_integration \
  m20_v2_astar_navigation.launch.py \
  cmd_topic:=/cmd_vel \
  >${LOG_FILE} 2>&1 </dev/null &
pid=$!
echo ${pid} >${PID_FILE}
sleep 5
if ! kill -0 ${pid} 2>/dev/null; then
  rm -f ${PID_FILE}
  echo 'ERROR: integrated navigation failed to start' >&2
  tail -n 100 ${LOG_FILE} >&2
  exit 1
fi

# Put the dog in the same navigation/agile control state validated during
# motion integration, then enable forwarding of planner /cmd_vel commands.
prepare_result=$(timeout 12 ros2 service call /m20/prepare_navigation \
  std_srvs/srv/Trigger '{}' 2>&1 || true)
if ! grep -q 'success=True' <<<"${prepare_result}"; then
  timeout 5 ros2 service call /m20/enable_motion \
    std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true
  echo 'ERROR: failed to prepare dog navigation mode/gait; motion remains disabled' >&2
  echo "${prepare_result}" >&2
  exit 1
fi

enable_result=$(timeout 5 ros2 service call /m20/enable_motion \
  std_srvs/srv/SetBool '{data: true}' 2>&1 || true)
if ! grep -q 'success=True' <<<"${enable_result}"; then
  echo 'ERROR: planners are running but motion bridge could not be enabled' >&2
  echo "${enable_result}" >&2
  exit 1
fi

echo "V2+A* navigation started (pid ${pid}); navigation mode/agile gait prepared; motion ENABLED"
