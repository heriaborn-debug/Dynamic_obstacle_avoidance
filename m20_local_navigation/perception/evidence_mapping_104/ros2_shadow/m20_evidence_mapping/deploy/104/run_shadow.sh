#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/foxy/setup.bash
source /home/user/m20_evidence_ws_104/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/m20_evidence_ws_104/deploy/fastdds_edge104.xml
unset FASTDDS_BUILTIN_TRANSPORTS

exec taskset -c 0-5 ros2 run m20_evidence_mapping evidence_mapping_node \
  --ros-args \
  -r __node:=m20_evidence_mapping_104 \
  --params-file /home/user/m20_evidence_ws_104/deploy/m20_evidence_mapping_104_shadow.yaml
