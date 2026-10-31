#!/usr/bin/env bash
# GPU 노트북(Ubuntu, Isaac Sim 5.1) 첫 설정 → 정지 소켓 1회 확인 → 발표 영상 1개 렌더.
# 사용: ISAAC_SIM=~/isaacsim bash isaac/setup_gpu.sh   (pip 설치면 ISAAC_PY=~/env_isaacsim/bin/python bash isaac/setup_gpu.sh)
set -euo pipefail

# Isaac Sim 파이썬: 압축 설치면 <설치폴더>/python.sh, pip 설치(가상환경)면 ISAAC_PY=그 파이썬 경로
ISAAC="${ISAAC_SIM:-$HOME/isaacsim}"
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${ISAAC_PY:-$ISAAC/python.sh}"
[ -x "$PY" ] || command -v "$PY" >/dev/null || { echo "Isaac Sim 파이썬을 찾을 수 없다: $PY (ISAAC_SIM=설치폴더 또는 ISAAC_PY=파이썬)"; exit 1; }

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
