#!/usr/bin/env bash
# 원인 분리 대조 학습(E1-3d): 보정 전 트윈 모델 act_twin_v2(2만 스텝)에서 1천 스텝만 더 학습한다.
#   plug_twin_c1000  : 보정한 카메라 렌더(data/twin_render3), 증강 시드 1000
#   plug_twin_nc1000 : 보정 전 카메라 렌더(data/twin_render2의 n 변형), 증강 시드 1000
# 두 데이터셋은 같은 에피소드, 같은 증강 시드로 만들어 '카메라 보정' 하나만 다르다. 학습 시드 3개(1000, 2000, 3000).
# act_twin_v4(최종 모델)와 같은 설정: v2 last에서 이어 학습, 배치 8, 학습률은 체크포인트 설정(1e-5), 이미지 변환 켬.
# 실행: bash tools/research/train_controls.sh   (Isaac 렌더가 끝난 뒤. GPU 하나를 차례로 쓴다)
set -euo pipefail
cd "$(dirname "$0")/../.."
source act/env.sh
BASE="$HOME/act_runs/act_twin_v2/checkpoints/020000/pretrained_model"
for DS in plug_twin_c1000 plug_twin_nc1000; do
  for SEED in 1000 2000 3000; do
    OUT="$HOME/act_runs/ctl_${DS#plug_twin_}_s${SEED}"
    [ -e "$OUT/checkpoints/001000" ] && { echo "있음: $OUT"; continue; }
    rm -rf "$OUT"
    echo "== $DS seed $SEED -> $OUT"
    "$LEROBOT_BIN/lerobot-train" --dataset.repo_id="$HF_USER/$DS" --policy.path="$BASE" --policy.device=cuda \
      --policy.push_to_hub=false --output_dir="$OUT" --job_name="ctl_${DS}_s${SEED}" --batch_size=8 --log_freq=200 \
      --wandb.enable=false --steps=1000 --save_freq=1000 --num_workers=4 --dataset.image_transforms.enable=true --seed="$SEED"
  done
done
echo "대조 학습 끝"
