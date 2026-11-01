#!/usr/bin/env bash
# GPU 노트북(Ubuntu, Isaac Sim 5.1) 첫 설정 → 정지 소켓 1회 확인 → 발표 영상 1개 렌더.
# 사용: bash isaac/setup_gpu.sh   (python.sh 설치폴더나 pip 가상환경 ~/isaacsim_env를 스스로 찾는다. 다르면 ISAAC_SIM= 또는 ISAAC_PY=)
set -euo pipefail

# Isaac Sim 파이썬: 압축 설치면 <설치폴더>/python.sh, pip 설치(가상환경)면 ISAAC_PY=그 파이썬 경로
ISAAC="${ISAAC_SIM:-$HOME/isaacsim}"
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${ISAAC_PY:-}"
if [ -z "$PY" ]; then  # 지정이 없으면 압축 설치(python.sh) → pip 가상환경(~/isaacsim_env, ~/env_isaacsim) 순서로 찾는다
  for c in "$ISAAC/python.sh" "$HOME/isaacsim_env/bin/python" "$HOME/env_isaacsim/bin/python"; do
    [ -x "$c" ] && { PY="$c"; break; }
  done
fi
[ -n "$PY" ] && [ -x "$PY" ] || { echo "Isaac Sim 파이썬을 찾을 수 없다(ISAAC_SIM=설치폴더 또는 ISAAC_PY=파이썬 경로로 지정)"; exit 1; }
[ -d "$HOME/isaacsim_compat/lib" ] && export LD_LIBRARY_PATH="$HOME/isaacsim_compat/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"  # Ubuntu 26.04: 24.04의 libxml2.so.2, libGLU를 풀어 둔 곳(URDF 가져오기에 필요)
export OMNI_KIT_ACCEPT_EULA=YES  # pip 설치판은 첫 실행 때 사용권 동의를 키보드로 묻는다. 화면 없이 돌리려면 미리 동의
"$PY" -c "import isaacsim" 2>/dev/null || { echo "$PY 에서 isaacsim을 import할 수 없다"; exit 1; }
echo "Isaac Sim 파이썬: $PY"

echo "== 1/5 의존성 (Isaac Sim 파이썬). numpy는 Isaac Sim 것을 그대로 쓴다"
"$PY" -m pip install "mujoco>=3.3" imageio imageio-ffmpeg pillow scipy

echo "== 2/5 로봇 USD (배치마다 한 번. side = 본선 L자 플러그)"
for L in side top; do
  S=$([ "$L" = side ] && echo _side || echo "")
  [ -f "$HERE/so101/so101_plugbot$S.usd" ] || "$PY" "$HERE/import_robot.py" --layout "$L"
done
[ -f "$HERE/so101/so101_plugbot_side.usd" ] || { echo "USD 변환 실패. isaac/README.md의 GUI 방법으로 만든 뒤 다시 실행"; exit 1; }

echo "== 3/5 미리보기 사진 (outputs/isaac_preview_side_{Webcam,ShowCam,Wrist}.png): 장면이 맞는지 먼저 눈으로"
"$PY" "$HERE/run_plugbot_isaac.py" --preview --headless

echo "== 4/5 정지 소켓 1회 (화면 없이)"
"$PY" "$HERE/run_plugbot_isaac.py" --hs 0 --strategy predictive --trials 1 --headless

echo "== 5/5 발표 영상: 파고 6 cm, 예측 전략 1회 녹화 (outputs/isaac_video_side_hs6_predictive_*.mp4)"
"$PY" "$HERE/run_plugbot_isaac.py" --hs 0.06 --strategy predictive --trials 1 --headless --video

echo "끝. 성공률 실험(렌더링 없이): $PY $HERE/run_plugbot_isaac.py --hs 0.06 --strategy predictive --trials 30 --headless"
echo "    학습 데이터 50개:      $PY $HERE/run_plugbot_isaac.py --hs 0.06 --dataset 50 --headless"
