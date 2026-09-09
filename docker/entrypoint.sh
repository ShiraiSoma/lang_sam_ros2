#!/bin/bash
set -e

echo "NODE_IP: $(hostname -I)"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/lang_sam_ws/docker/server_profile.xml

source /opt/ros/humble/setup.bash
source /opt/lang_sam_ws/install/setup.bash

exec ros2 run lang_sam_detector lang_sam_detector_node.py --ros-args \
    -p sam_model:="${SAM_MODEL:-sam2.1_hiera_small}" \
    -p text_prompt:="${TEXT_PROMPT:-red pylon.}" \
    -p box_threshold:="${BOX_THRESHOLD:-0.3}" \
    -p text_threshold:="${TEXT_THRESHOLD:-0.25}" \
    -p request_topic:="${REQUEST_TOPIC:-/lang_sam/detect_request}" \
    -p response_topic:="${RESPONSE_TOPIC:-/lang_sam/detections}" \
    -p visualize:=false
