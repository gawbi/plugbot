#!/usr/bin/env bash
# 원격조종 확인(ACT_PLAN.md 3.4): 리더로 팔로워를 움직이고 카메라 화면만 보고 꽂아 본다. 끝내기 Ctrl+C
set -euo pipefail
source "$(dirname "$0")/env.sh"
need_port "$F_PORT" "$L_PORT"; need_cams
exec "$LEROBOT_BIN/lerobot-teleoperate" \
  --robot.type=so101_follower --robot.port="$F_PORT" --robot.id="$F_ID" --robot.cameras="$CAMS" \
  --teleop.type=so101_leader --teleop.port="$L_PORT" --teleop.id="$L_ID" --display_data=true
