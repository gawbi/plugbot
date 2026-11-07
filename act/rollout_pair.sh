#!/usr/bin/env bash
# 로봇 혼자 꽂기(ACT) 실행 → "실물 | UR10e" 나란히 영상(outputs/rollout_pair_<실행>.mp4).
#   bash act/rollout_pair.sh                 '꽂힘' 판정이고 아직 영상이 없는 실행을 모두
#   bash act/rollout_pair.sh 20261106_191627  이 실행만(판정 없어도 됨. 소켓은 기준 소켓)
# Isaac 렌더는 GPU와 CPU를 많이 쓴다. 로봇 실행 중에는 돌리지 않는다(제어가 느려져 성공률이 떨어질 수 있다).
# 측정이 한 묶음 끝난 뒤나 쉬는 시간에 돌린다. 실행 하나에 몇 분.
set -u
cd "$(dirname "$0")/.."
LR=~/lerobot_env/bin/python
ISAAC=~/isaacsim_env/bin/python

if pgrep -f "plugbot-entry=rollout|record.sh" >/dev/null; then
  echo "로봇 실행 중이라 멈춤. 실행이 끝난 뒤 다시."
  exit 1
fi

if [ $# -ge 1 ]; then
  RUNS="$1"
else
  RUNS=$(for f in data/rollouts/*/result.json; do
    r=$(basename "$(dirname "$f")")
    grep -q '"ok": true' "$f" && [ ! -f "outputs/rollout_pair_$r.mp4" ] && echo "$r"
  done)
fi
[ -z "$RUNS" ] && { echo "만들 실행이 없다('꽂힘' 판정 + 영상 없음)"; exit 0; }

for r in $RUNS; do
  echo "== $r"
  $LR isaac/scaleup/rollout_traj.py "$r" || continue
  fd="data/scaleup_real_frames/rollout_$r"
  rm -rf "$fd"
  LD_LIBRARY_PATH=~/isaacsim_compat/lib OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 \
    $ISAAC isaac/scaleup/run_scaleup.py --replay "data/scaleup_real/rollout_$r.npz" --headless --hs-model 0 \
    --frames-dir "$fd" > "$fd.log" 2>&1 || { echo "Isaac 렌더 실패: $fd.log"; continue; }
  $LR tools/make_rollout_pair.py "$r"
done
