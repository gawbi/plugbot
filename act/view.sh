#!/usr/bin/env bash
# 녹화한 에피소드를 Rerun 창에 다시 띄워 본다(카메라 영상, 관절, 명령). 사용: bash act/view.sh static|moving [에피소드 번호=0]
#   카메라가 가렸는지, 흔들렸는지, 빛이 번쩍이는지, 꽂는 순간이 화면에 잡혔는지 본다. 나쁜 에피소드는 ACT_PLAN.md 3.7로 지운다
set -euo pipefail
source "$(dirname "$0")/env.sh"
KIND="${1:?static 또는 moving}"; EP="${2:-0}"
exec "$LEROBOT_BIN/lerobot-dataset-viz" --repo-id "$HF_USER/plug_$KIND" --episode-index "$EP"
