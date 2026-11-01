#!/usr/bin/env bash
# 학습한 정책을 로봇에서 실행(ACT_PLAN.md 3.10). 사용: bash act/rollout.sh <학습 이름> [test|eval] [평가 데이터셋 꼬리]
#   test: 30초 실행, 녹화 없음.  eval: 20회 녹화, 리셋 때 리더로 팔을 홈으로 되돌리고 플러그를 거치대에 다시 놓는다
#   예: bash act/rollout.sh act_v2 test,  bash act/rollout.sh act_v2 eval static
#   비상 정지: 팔로워 전원 스위치. Ctrl+C로 끝내면 토크가 풀릴 수 있으니 팔을 손으로 받친다
set -euo pipefail
source "$(dirname "$0")/env.sh"
NAME="${1:?학습 이름(act_runs 아래 폴더)}"; MODE="${2:-test}"; TAG="${3:-static}"
CKPT="$RUNS/$NAME/checkpoints/last/pretrained_model"
[ -d "$CKPT" ] || { echo "$CKPT 없음"; exit 1; }
need_cams
if [ "$MODE" = test ]; then
  need_port "$F_PORT"
  exec "$LEROBOT_BIN/lerobot-rollout" --strategy.type=base --policy.path="$CKPT" \
    --robot.type=so101_follower --robot.port="$F_PORT" --robot.id="$F_ID" --robot.cameras="$CAMS" \
    --task="$TASK" --duration=30 --device=cuda --display_data=true
fi
need_port "$F_PORT" "$L_PORT"
exec "$LEROBOT_BIN/lerobot-rollout" --strategy.type=episodic --policy.path="$CKPT" --policy.n_action_steps=10 \
  --robot.type=so101_follower --robot.port="$F_PORT" --robot.id="$F_ID" --robot.cameras="$CAMS" \
  --teleop.type=so101_leader --teleop.port="$L_PORT" --teleop.id="$L_ID" \
  --dataset.repo_id="$HF_USER/rollout_${NAME}_${TAG}" --dataset.single_task="$TASK" \
  --dataset.num_episodes=20 --dataset.episode_time_s=25 --dataset.reset_time_s=30 \
  --dataset.push_to_hub=false --device=cuda
