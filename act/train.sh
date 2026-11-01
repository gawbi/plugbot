#!/usr/bin/env bash
# ACT 학습(ACT_PLAN.md 3.9). 사용: bash act/train.sh <데이터셋 이름> [smoke|full] [이름]
#   예: bash act/train.sh plug_static smoke        (2000스텝, 약 10분: 파이프라인 확인)
#       bash act/train.sh plug_all full act_v2     (8만 스텝, 이 노트북 RTX 5070 Laptop에서 약 6시간 반)
#   이어서: RESUME=1 bash act/train.sh plug_all full act_v2
#   데이터셋 이름은 $HF_USER/ 뒤 부분. 시뮬레이션 데이터는 HF_USER=local, 이름 plugbot_sim_side
set -euo pipefail
source "$(dirname "$0")/env.sh"
DS="${1:?데이터셋 이름}"; MODE="${2:-smoke}"; NAME="${3:-act_${DS}_${MODE}}"
OUTD="$RUNS/$NAME"
if [ "${RESUME:-0}" = 1 ]; then
  exec "$LEROBOT_BIN/lerobot-train" --config_path="$OUTD/checkpoints/last/pretrained_model/train_config.json" --resume=true
fi
case "$MODE" in
  smoke) ARGS=(--steps=2000 --save_freq=1000) ;;
  full) ARGS=(--steps=80000 --save_freq=10000 --num_workers=4 --dataset.image_transforms.enable=true) ;;
  *) echo "smoke 또는 full"; exit 1 ;;
esac
[ -e "$OUTD" ] && { echo "$OUTD 가 이미 있다. 다른 이름을 주거나 RESUME=1"; exit 1; }
exec "$LEROBOT_BIN/lerobot-train" --dataset.repo_id="$HF_USER/$DS" --policy.type=act --policy.device=cuda \
  --policy.push_to_hub=false --output_dir="$OUTD" --job_name="$NAME" --batch_size=8 --log_freq=200 \
  --wandb.enable=false "${ARGS[@]}"
