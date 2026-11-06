#!/bin/bash
set -e

echo "NODE_IP: $(hostname -I)"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/lang_sam_ws/docker/server_profile.xml

source /opt/ros/humble/setup.bash
source /opt/lang_sam_ws/install/setup.bash

PARAMS_FILE=/opt/lang_sam_ws/install/lang_sam_executor/share/lang_sam_executor/config/params.yaml

# 既定値はイメージに同梱したparams.yaml。環境変数が設定されたものだけ上書きする
ARGS=(--params-file "${PARAMS_FILE}"
      -p cutie_weights:=/opt/cutie/cutie-base-mega.pth
      -p visualize:=false)
add_param() {
    if [ -n "${2}" ]; then
        ARGS+=(-p "${1}:=${2}")
    fi
}
# 検出(lang_sam_detector)
add_param sam_model "${SAM_MODEL}"
add_param text_prompt "${TEXT_PROMPT}"
add_param box_threshold "${BOX_THRESHOLD}"
add_param text_threshold "${TEXT_THRESHOLD}"
# 追跡(lang_sam_tracker)
add_param detection_interval_sec "${DETECTION_INTERVAL_SEC}"
add_param cutie_max_internal_size "${CUTIE_MAX_INTERNAL_SIZE}"
add_param cutie_use_amp "${CUTIE_USE_AMP}"

exec ros2 run lang_sam_executor lang_sam_server_node.py --ros-args "${ARGS[@]}"
