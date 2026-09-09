#!/bin/bash
# 計算ノードのIPを受け取り、TCP専用のFast-DDSプロファイルを生成してから
# lang_sam_tracker(+person_following)をロボットPC側で起動する。
set -e

NODE_IP="${1:?Usage: $0 <compute_node_ip>}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SETUP="${LANG_SAM_WS_SETUP:-/home/shiraisoma/lang_sam_server_ws/install/setup.bash}"
LANG_SAM_VENV="${LANG_SAM_VENV:-/home/shiraisoma/venv/lang_sam}"

CLIENT_PROFILE="${SCRIPT_DIR}/client_profile.generated.xml"
sed "s/__COMPUTE_NODE_IP__/${NODE_IP}/" "${SCRIPT_DIR}/client_profile.xml.template" > "${CLIENT_PROFILE}"

echo "== 接続先: ${NODE_IP}:42100 (TCP) =="
echo "== 生成したプロファイル: ${CLIENT_PROFILE} =="

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${CLIENT_PROFILE}"

ros2 daemon stop 2>/dev/null || true

source /opt/ros/humble/setup.bash
source "${WORKSPACE_SETUP}"
source "${LANG_SAM_VENV}/bin/activate"

exec ros2 launch lang_sam_executor lang_sam_tracker.launch.py
