#!/bin/bash
# gpumng2上でlang_sam_serverジョブを投入し、割り当てられた計算ノードのIPを表示する。
# ロボットPC側で実行する。パスワードの入力は最初の1回だけで済む(SSH接続を使い回す)。
set -e

REMOTE_USER="${1:?Usage: $0 <MARINE_username>}"
REMOTE_HOST="gpumng2.cle.it-chiba.ac.jp"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTRL_SOCK="/tmp/ssh_ctrl_lang_sam_$$"

cleanup() {
    ssh -o ControlPath="${CTRL_SOCK}" -O exit "${REMOTE_USER}@${REMOTE_HOST}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "== gpumng2へ接続します(パスワードの入力は最初の1回だけです) =="
ssh -MNf -o ControlPath="${CTRL_SOCK}" -o ControlPersist=15m "${REMOTE_USER}@${REMOTE_HOST}"

RSSH() { ssh -o ControlPath="${CTRL_SOCK}" "${REMOTE_USER}@${REMOTE_HOST}" "$@"; }
RSCP() { scp -o ControlPath="${CTRL_SOCK}" "$@"; }

echo "== ジョブスクリプトをgpumng2へ転送 =="
RSSH "mkdir -p ~/lang_sam_run"
RSCP "${SCRIPT_DIR}/lang_sam_server.job" "${REMOTE_USER}@${REMOTE_HOST}:~/lang_sam_run/lang_sam_server.job"

echo "== ジョブ投入 =="
JOBID=$(RSSH "cd ~/lang_sam_run && sbatch --parsable lang_sam_server.job" | grep -E '^[0-9]+$' | tail -1)

if ! [[ "${JOBID}" =~ ^[0-9]+$ ]]; then
    echo "ジョブID取得に失敗しました(取得した値: '${JOBID}')。squeueで手動確認してください。"
    exit 1
fi
echo "JobID: ${JOBID}"

echo "== ジョブがRUNNINGになるまで待機 =="
STATE=""
for i in $(seq 1 60); do
    STATE=$(RSSH "squeue -j ${JOBID} -h -o %T" 2>/dev/null | head -1 | tr -d '[:space:]')
    echo "  状態: ${STATE:-不明} (${i}/60)"
    if [ "${STATE}" = "RUNNING" ]; then
        break
    fi
    sleep 5
done

if [ "${STATE}" != "RUNNING" ]; then
    echo "タイムアウト: ジョブがRUNNINGになりませんでした。squeueで手動確認してください。"
    exit 1
fi

NODE_IP=""
for i in $(seq 1 15); do
    NODE_IP=$(RSSH "cat ~/lang_sam_run/slurm-${JOBID}.out 2>/dev/null" | grep -m1 "NODE_IP:" | awk '{print $2}')
    if [ -n "${NODE_IP}" ]; then
        break
    fi
    sleep 3
done

if [ -z "${NODE_IP}" ]; then
    echo "計算ノードのIP取得に失敗しました。手動で確認してください:"
    echo "  ssh ${REMOTE_USER}@${REMOTE_HOST} 'cat ~/lang_sam_run/slurm-${JOBID}.out'"
    exit 1
fi

echo ""
echo "=========================================="
echo " JobID:   ${JOBID}"
echo " NodeIP:  ${NODE_IP}"
echo "=========================================="
echo ""
echo "ロボット側の起動には次を実行してください:"
echo "  ${SCRIPT_DIR}/connect_client.sh ${NODE_IP}"
echo ""
echo "ジョブを止めるときは:"
echo "  ssh ${REMOTE_USER}@${REMOTE_HOST} 'scancel ${JOBID}'"
