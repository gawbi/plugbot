#!/usr/bin/env bash
# 시범 녹화(ACT_PLAN.md 3.5). 사용: bash act/record.sh static|moving [에피소드 수=25]
#   static: 정지 소켓(25 s), moving: 흔들림 갑판(30 s). 이미 있으면 이어서 녹화한다(수는 '추가할 개수').
#   녹화 중 키: → 다음(일찍 끝냄), ← 다시, Esc 종료(터미널 입력이면 n, r, q)
set -euo pipefail
source "$(dirname "$0")/env.sh"
KIND="${1:?static 또는 moving}"; N="${2:-25}"
case "$KIND" in static) SEC=25 ;; moving) SEC=30 ;; *) echo "static 또는 moving"; exit 1 ;; esac
need_port "$F_PORT" "$L_PORT"; need_cams
REPO="$HF_USER/plug_$KIND"
ROOT="$HOME/.cache/huggingface/lerobot/$REPO"
EXTRA=()
if [ -d "$ROOT" ]; then
  if [ -f "$ROOT/meta/info.json" ]; then EXTRA=(--resume=true --dataset.root="$ROOT"); echo "이어서 녹화: $ROOT (+$N)"
  else echo "$ROOT 에 에피소드가 없는 빈 폴더가 있다(연결 실패로 끝난 녹화). 지우고 다시: rm -r $ROOT"; exit 1; fi
fi
exec "$LEROBOT_BIN/lerobot-record" \
  --robot.type=so101_follower --robot.port="$F_PORT" --robot.id="$F_ID" --robot.cameras="$CAMS" \
  --teleop.type=so101_leader --teleop.port="$L_PORT" --teleop.id="$L_ID" \
  --dataset.repo_id="$REPO" --dataset.no_stamp=true --dataset.single_task="$TASK" \
  --dataset.num_episodes="$N" --dataset.episode_time_s="$SEC" --dataset.reset_time_s=20 \
  --dataset.streaming_encoding=true --dataset.encoder_threads=2 \
  --dataset.push_to_hub="$PUSH" --dataset.private=true --display_data=false "${EXTRA[@]}"
