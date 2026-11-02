#!/usr/bin/env bash
# 원격조종 확인(ACT_PLAN.md 3.4): 리더로 팔로워를 움직이고 카메라 화면만 보고 꽂아 본다. 끝내기 Ctrl+C
set -euo pipefail
source "$(dirname "$0")/env.sh"
need_port "$F_PORT" "$L_PORT"
CAM_ARGS=(--robot.cameras="$CAMS")
if [ -z "$FRONT_CAM" ] || [ -z "$WRIST_CAM" ]; then  # 카메라를 달기 전: 팔만 움직여 본다
  echo "카메라 설정이 없어 팔만 원격조종한다(FRONT_CAM, WRIST_CAM을 넣으면 화면도 뜬다)"; CAM_ARGS=()
fi
exec "$LEROBOT_BIN/lerobot-teleoperate" \
  --robot.type=so101_follower --robot.port="$F_PORT" --robot.id="$F_ID" "${CAM_ARGS[@]}" \
  --teleop.type=so101_leader --teleop.port="$L_PORT" --teleop.id="$L_ID" --display_data=true
