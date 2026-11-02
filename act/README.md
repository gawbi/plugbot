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

## 시연 환경 (2026-11-01 결정)

- 65 × 65 × 65 cm 암막 포토부스 안에 흔들림 갑판, 로봇, 앞 카메라를 넣는다. 물 위 시연은 하지 않는다.
- 녹화, 학습 평가, 본선 시연을 모두 같은 부스, 같은 조명, 같은 카메라 자리에서 한다(ACT는 녹화한 화면만 배운다).

## 본선 체험 구성 (2026-11-01 결정)

1. **사람 대 로봇**: 같은 파도에서 심사위원이 리더 팔로 직접 꽂아 본다(텔레옵, 20초). 그다음 로봇이 혼자 꽂는다.
2. **바다 조종기**: 심사위원이 파도 세기(잔잔, 보통, 출렁)를 고르고 "배 도착, 충전 시작" 버튼을 누르면 로봇이 꽂는다.
   노트북이 갑판 아두이노에 `s`(정현파), `+`/`-`(진폭), `h`(멈춤)를 보낸다. 고를 수 있는 파도는 리허설 20회 이상에서 거의 다 성공한 것만.
3. **로봇의 눈**: 앞 카메라 영상에 소켓 마커, 지금 위치, 예측 위치, '잔잔한 순간' 표시를 겹쳐 옆 화면에 띄운다.
- 갑판은 좌우로 약하게 흔든다(정현파 8 mm 기본, 시뮬레이션 100%). 꽂고 3초 뒤 정해진 동작으로 빼서 홈으로 돌아가 사람 손 없이 반복한다.
- 로봇이 혼자 움직일 때 사람 손은 부스 밖. ACT가 실패하면 고전 예측 제어기로 바로 전환, 그래도 안 되면 백업 영상.
- 꽂힘 센서는 쓰지 않는다(2026-11-02 결정). 성공은 팀원이 체험 화면에서 스페이스로 판정한다. 끝 맞춤은 깔때기 + 자석 래치(소켓 바닥 자석, 핀 끝 철 조각 권장)로 쉽게 한다.

체험 화면(사람 대 로봇, 파도 버튼, 로봇의 눈): `~/lerobot_env/bin/python -m uvicorn demo.server:app --port 8100` → http://127.0.0.1:8100
(갑판 `SHAKER_PORT=/dev/ttyUSB0`, 카메라 `EYE_CAM=...`. 없으면 가짜로 동작. 로봇은 지금 MuJoCo 가짜, 실물 연결 날 고전 제어기와 ACT를 붙인다)

## 순서

```bash
# 0. 한 번: 시리얼 포트 권한(다시 로그인해야 적용)
sudo usermod -aG dialout $USER

# 0-1. 처음 한 번: 보정(실물 로봇은 이 노트북에서 처음 다룬다). 새 모터면 먼저 ID 설정(motors)
bash act/calibrate.sh follower motors     # 새 모터일 때만(조립 전, 안내대로 모터를 하나씩 꽂는다)
bash act/calibrate.sh leader motors
bash act/calibrate.sh follower            # 녹화 시작 뒤에는 다시 보정하지 않는다
bash act/calibrate.sh leader

# 1. 로봇 두 대와 카메라 두 대를 꽂고 점검. 나온 /dev/v4l/by-id 경로를 act/env.sh의 FRONT_CAM, WRIST_CAM에 적는다
bash act/check.sh
~/lerobot_env/bin/lerobot-find-cameras opencv      # 어느 쪽이 앞, 손목인지 사진으로 확인

# 2. 원격조종으로 화면만 보고 꽂아 본다
bash act/teleop.sh

# 3. 녹화(이어서 녹화도 같은 명령). 정지 50회, 흔들림 60~80회. Rerun 창에 카메라가 떠서 보면서 녹화한다
#    끝에 나오는 Cadence가 30 Hz 가까이인지 확인(느리면 SHOW=false). 녹화 뒤 bash act/view.sh static 0 으로 다시 본다
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
