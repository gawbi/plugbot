#!/usr/bin/env bash
# 데이터셋 합치기(ACT_PLAN.md 3.7). 사용: bash act/merge.sh <새 이름> <데이터셋 이름>...
#   예(선상, 승선 뒤 녹화를 plug_static과 plug_moving에 이어 붙인 뒤):
#     bash act/merge.sh plug_train_v5 plug_static plug_moving_part1 plug_moving_part2 plug_moving
#   0.6.1은 --new_repo_id가 꼭 있어야 한다. 카메라 이름, 해상도, 상태 차원이 같아야 합쳐진다.
#   합치기 전후 에피소드 수를 찍어 준다(합이 맞는지 눈으로 확인).
set -euo pipefail
source "$(dirname "$0")/env.sh"
NEW="${1:?새 이름}"; shift
[ $# -ge 2 ] || { echo "합칠 데이터셋을 두 개 이상 준다"; exit 1; }
DROOT="$HOME/.cache/huggingface/lerobot/$HF_USER"
[ -e "$DROOT/$NEW" ] && { echo "$DROOT/$NEW 가 이미 있다. 다른 이름을 준다"; exit 1; }
count() { python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['total_episodes'])" "$DROOT/$1/meta/info.json"; }
total=0
for d in "$@"; do
  n=$(count "$d"); echo "  $d: $n개"; total=$((total + n))
done
IDS=$(printf "'%s/%s', " "$HF_USER" "$@"); IDS="[${IDS%, }]"
RTS=$(printf "'$DROOT/%s', " "$@"); RTS="[${RTS%, }]"   # 로컬 경로를 직접 준다(허브에서 찾지 않게)
"$LEROBOT_BIN/lerobot-edit-dataset" --new_repo_id="$HF_USER/$NEW" --new_root="$DROOT/$NEW" --operation.type=merge \
  --operation.repo_ids "$IDS" --operation.roots "$RTS"
echo "$NEW: $(count "$NEW")개 (합 $total개여야 함)"
