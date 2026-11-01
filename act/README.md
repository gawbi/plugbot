# 실물 ACT (우분투 GPU 노트북)

`ACT_PLAN.md` 3절의 명령을 이 노트북용 스크립트로 묶었다. 설정은 `act/env.sh` 한 곳(포트, 보정 id, 카메라, HF 아이디).

## 이 노트북에서 확인한 것 (2026-11-01, 로봇 없이)

| 항목 | 결과 |
|---|---|
| 환경 | `~/lerobot_env`(uv, Python 3.12), lerobot 0.6.1, torch 2.11.0+cu130, RTX 5070 Laptop sm_120 인식 |
| 형식 | MuJoCo 시범 20개(`make_sim_dataset.py --randomize`) → `tools/to_lerobot.py` → `local/plugbot_sim_side`(1378프레임, front, wrist 640 × 480) 다시 읽기 |
| 학습 | ACT 52M, 배치 8, 2000스텝 9분 46초(3.5스텝/s, 계산이 병목). 손실 6.0 → 1.17. 본 학습 8만 스텝이면 약 6시간 반 |
| 추론 | 체크포인트 불러와 100스텝 청크 12.5 ms(CUDA) |
| Isaac 시범 | `isaac/run_plugbot_isaac.py --dataset 3` 3개 모두 성공, 31초 |

로봇, 카메라, 보정 파일이 없어 녹화(`record.sh`), 원격조종, 실행(`rollout.sh`)은 아직 돌리지 않았다.
`torchcodec`은 시스템 FFmpeg가 없어 못 쓰고 pyav로 읽는다(학습 속도에는 영향이 거의 없었다: 데이터 0.004 s/스텝).

## 순서

```bash
# 0. 한 번: 시리얼 포트 권한(다시 로그인해야 적용), 보정 파일을 맥에서 복사(다시 보정하지 않는다)
sudo usermod -aG dialout $USER
scp -r <맥>:~/.cache/huggingface/lerobot/calibration ~/.cache/huggingface/lerobot/

# 1. 로봇 두 대와 카메라 두 대를 꽂고 점검. 나온 /dev/v4l/by-id 경로를 act/env.sh의 FRONT_CAM, WRIST_CAM에 적는다
bash act/check.sh
~/lerobot_env/bin/lerobot-find-cameras opencv      # 어느 쪽이 앞, 손목인지 사진으로 확인

# 2. 원격조종으로 화면만 보고 꽂아 본다
bash act/teleop.sh

# 3. 녹화(이어서 녹화도 같은 명령). 정지 50회, 흔들림 60~80회
bash act/record.sh static 25
bash act/record.sh moving 30

# 4. 학습: 시험 10분 → 본 학습 밤새
bash act/train.sh plug_static smoke
bash act/train.sh plug_static full act_v1          # 끊기면 RESUME=1 bash act/train.sh plug_static full act_v1

# 5. 로봇에서 실행: 30초 시험 → 20회 평가 녹화
bash act/rollout.sh act_v1 test
bash act/rollout.sh act_v1 eval static
```

- 맥에서 Hub에 올린 데이터를 쓰려면 `~/lerobot_env/bin/hf auth login` 뒤 `HF_USER=<아이디> bash act/train.sh plug_static full`.
- 나쁜 에피소드 삭제와 합치기는 `ACT_PLAN.md` 3.7절(`lerobot-edit-dataset`).
- 시뮬레이션 시범은 실물 50회로 먼저 학습한 뒤 섞을지 정한다(`ACT_PLAN.md` 2절).
  MuJoCo: `.venv/bin/python make_sim_dataset.py --episodes 50 --randomize`, Isaac: `isaac/run_plugbot_isaac.py --hs 0.06 --dataset 50 --headless`.
