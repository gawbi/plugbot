#!/usr/bin/env bash
# GPU 노트북: 발표용 Isaac Sim 영상 묶음. 같은 파도(시드)에서 추종과 예측을 찍고, 정지 소켓 장면도 찍은 뒤 묶는다.
# 사용: bash isaac/make_showreel.sh   (setup_gpu.sh가 끝난 뒤. 파이썬은 setup_gpu.sh와 같은 방법으로 찾는다)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${ISAAC_PY:-}"
if [ -z "$PY" ]; then
  for c in "${ISAAC_SIM:-$HOME/isaacsim}/python.sh" "$HOME/isaacsim_env/bin/python" "$HOME/env_isaacsim/bin/python"; do
    [ -x "$c" ] && { PY="$c"; break; }
  done
fi
[ -n "$PY" ] || { echo "Isaac Sim 파이썬을 찾을 수 없다(ISAAC_PY=로 지정)"; exit 1; }
[ -d "$HOME/isaacsim_compat/lib" ] && export LD_LIBRARY_PATH="$HOME/isaacsim_compat/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"  # Ubuntu 26.04: 24.04의 libxml2.so.2, libGLU를 풀어 둔 곳(URDF 가져오기에 필요)
export OMNI_KIT_ACCEPT_EULA=YES
SEED="${SEED:-2}"  # MuJoCo에서 Hs 6 cm 추종 실패, 예측 성공인 시드(make_media.py). Isaac에서 다르면 바꿔 본다

for s in reactive predictive; do
  echo "== 파고 6 cm, $s, 시드 $SEED"
  "$PY" "$HERE/run_plugbot_isaac.py" --hs 0.06 --strategy "$s" --trials 1 --seed "$SEED" --headless --video --tag video
done
echo "== 정지 소켓(파고 0), 예측"
"$PY" "$HERE/run_plugbot_isaac.py" --hs 0 --strategy predictive --trials 1 --headless --video --tag video
echo "== 묶기"
"$PY" "$HERE/../tools/make_showreel.py" --layout side --hs 6
echo "끝: outputs/showreel_side_hs6_{compare,views}.mp4, outputs/showreel_side_hs6_compare.gif"
