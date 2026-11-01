#!/usr/bin/env bash
# 실물 ACT 준비 상태 점검: GPU, LeRobot, 보정 파일, 로봇 포트, 카메라. 아무것도 움직이지 않는다.
# 사용: bash act/check.sh
set -uo pipefail
source "$(dirname "$0")/env.sh"
ok() { echo "  [O] $*"; }
no() { echo "  [X] $*"; }

echo "== GPU, LeRobot"
"$LEROBOT_BIN/python" - <<'EOF' 2>/dev/null || echo "  [X] $LEROBOT_BIN 에 lerobot이 없다(ACT_PLAN.md 3.8)"
import torch, importlib.metadata as md
print(f"  [{'O' if torch.cuda.is_available() else 'X'}] torch {torch.__version__}, CUDA {torch.cuda.is_available()}",
      torch.cuda.get_device_name(0) if torch.cuda.is_available() else "")
print(f"  [O] lerobot {md.version('lerobot')}")
EOF

echo "== 보정 파일(처음 한 번 bash act/calibrate.sh follower, leader. 녹화를 시작한 뒤에는 다시 보정하지 않는다)"
CAL=~/.cache/huggingface/lerobot/calibration
for f in "robots/so_follower/$F_ID.json" "teleoperators/so_leader/$L_ID.json"; do
  [ -f "$CAL/$f" ] && ok "$CAL/$f" || no "$CAL/$f 없음: bash act/calibrate.sh follower(또는 leader)"
done

echo "== 로봇 포트(팔로워 $F_PORT, 리더 $L_PORT)"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sed 's/^/  /' || true
for p in "$F_PORT" "$L_PORT"; do
  if [ ! -e "$p" ]; then no "$p 없음"; elif [ -r "$p" ] && [ -w "$p" ]; then ok "$p 읽기, 쓰기 가능"; else no "$p 권한 없음(sudo usermod -aG dialout $USER 후 다시 로그인)"; fi
done
echo "  어느 쪽이 팔로워인지 모르면: $LEROBOT_BIN/lerobot-find-port (안내대로 USB를 하나 뽑는다)"

echo "== 카메라(/dev/v4l/by-id). HP True Vision은 노트북 내장이라 쓰지 않는다"
ls -l /dev/v4l/by-id/ 2>/dev/null | awk 'NR>1 {print "  " $9 " -> " $11}'
[ -n "$FRONT_CAM" ] && [ -e "$FRONT_CAM" ] && ok "front $FRONT_CAM" || no "FRONT_CAM 미설정 또는 없음"
[ -n "$WRIST_CAM" ] && [ -e "$WRIST_CAM" ] && ok "wrist $WRIST_CAM" || no "WRIST_CAM 미설정 또는 없음"
echo "  화면 확인과 사진: $LEROBOT_BIN/lerobot-find-cameras opencv  (사진은 outputs/captured_images/)"

echo "== 데이터셋(~/.cache/huggingface/lerobot/$HF_USER)"
ls ~/.cache/huggingface/lerobot/"$HF_USER" 2>/dev/null | sed 's/^/  /' || echo "  (없음)"
