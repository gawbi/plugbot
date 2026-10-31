#!/usr/bin/env bash
# GPU 노트북(Ubuntu, Isaac Sim 5.1) 첫 설정 → 정지 소켓 1회 확인 → 발표 영상 1개 렌더.
# 사용: ISAAC_SIM=~/isaacsim bash plugbot_demo/isaac/setup_gpu.sh
set -euo pipefail

ISAAC="${ISAAC_SIM:-$HOME/isaacsim}"
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$ISAAC/python.sh"
[ -x "$PY" ] || { echo "Isaac Sim 5.1 폴더를 찾을 수 없다: $ISAAC (ISAAC_SIM=설치폴더 로 지정)"; exit 1; }

echo "== 1/4 의존성 (Isaac Sim 파이썬). numpy는 Isaac Sim 것을 그대로 쓴다"
"$PY" -m pip install "mujoco>=3.3" imageio imageio-ffmpeg pillow

echo "== 2/4 로봇 USD (한 번)"
if [ ! -f "$HERE/so101/so101_plugbot.usd" ]; then
  "$PY" "$HERE/import_robot.py"
fi
[ -f "$HERE/so101/so101_plugbot.usd" ] || { echo "USD 변환 실패. isaac/README.md의 GUI 방법으로 만든 뒤 다시 실행"; exit 1; }

echo "== 3/4 정지 소켓 1회 (화면 없이)"
"$PY" "$HERE/run_plugbot_isaac.py" --hs 0 --strategy predictive --trials 1 --headless

echo "== 4/4 발표 영상: 파고 6 cm, 예측 전략 1회 녹화 (outputs/isaac_video_hs6_predictive_*.mp4)"
"$PY" "$HERE/run_plugbot_isaac.py" --hs 0.06 --strategy predictive --trials 1 --headless --video

echo "끝. 성공률 실험은 렌더링 없이: $PY $HERE/run_plugbot_isaac.py --hs 0.06 --strategy predictive --trials 30 --headless"
