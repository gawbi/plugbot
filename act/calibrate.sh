#!/usr/bin/env bash
# 처음 한 번: 모터 ID 설정과 보정(ACT_PLAN.md 3.3). 사용: bash act/calibrate.sh follower|leader [motors]
#   motors: 새 모터에 ID를 매긴다(조립 전, 모터를 하나씩 꽂으라는 안내를 따른다). 이미 조립해 움직여 본 팔은 하지 않는다
#   보정 결과: ~/.cache/huggingface/lerobot/calibration/{robots/so_follower,teleoperators/so_leader}/<id>.json
#   보정은 녹화를 시작하기 전에 한 번만 한다. 그 뒤에 다시 하면 학습한 정책의 관절값이 어긋난다
set -euo pipefail
source "$(dirname "$0")/env.sh"
WHO="${1:?follower 또는 leader}"; WHAT="${2:-calibrate}"
case "$WHO" in
  follower) need_port "$F_PORT"; DEV=(--robot.type=so101_follower --robot.port="$F_PORT"); ID=(--robot.id="$F_ID")
            CAL=~/.cache/huggingface/lerobot/calibration/robots/so_follower/$F_ID.json ;;
  leader)   need_port "$L_PORT"; DEV=(--teleop.type=so101_leader --teleop.port="$L_PORT"); ID=(--teleop.id="$L_ID")
            CAL=~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/$L_ID.json ;;
  *) echo "follower 또는 leader"; exit 1 ;;
esac
if [ "$WHAT" = motors ]; then
  exec "$LEROBOT_BIN/lerobot-setup-motors" "${DEV[@]}"
fi
if [ -f "$CAL" ] && [ "${FORCE:-0}" != 1 ]; then
  echo "이미 보정 파일이 있다: $CAL"
  echo "녹화한 데이터가 있으면 다시 보정하지 않는다. 그래도 하려면 FORCE=1 bash act/calibrate.sh $WHO"; exit 1
fi
exec "$LEROBOT_BIN/lerobot-calibrate" "${DEV[@]}" "${ID[@]}"
