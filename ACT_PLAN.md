# PlugBot ACT 학습 계획 (SO-101 ,  LeRobot 0.6.1)

작성 2026-10-23. 시연 준비 기간 10/27~11/7.

- 기준 버전: PyPI 최신 `lerobot==0.6.1`(2026-08-03 배포). GitHub main은 0.6.2 개발판이라 문서가 조금 앞서 있다. 맥과 우분투 모두 0.6.1로 고정한다.
- 표기
  - [실행확인]: 이 맥(M5)에 0.6.1을 설치해 로봇 없이 직접 돌려 봤다. 가짜 포트와 합성 데이터로 인자 해석, 학습 20스텝, 모델 로드, 데이터 합치기, 삭제까지 확인했다.
  - [코드확인]: v0.6.1 소스나 공식 문서에서 확인했다.
  - [미검증]: 확인하지 못했다. 현장에서 확인해야 한다.

## 1. 결론: ACT vs SmolVLA, VLA

**ACT로 간다.** 이 과제는 한 장면에서 하는 한 가지 일이고, 1.5 mm 여유로 꽂아야 하며, 움직이는 소켓 때문에 지연에 민감하다.

| 항목 | ACT | SmolVLA | Pi0 / Pi0.5 |
|---|---|---|---|
| 크기 | 52M(우리 설정, 학습 로그 기준) [실행확인]. 문서에는 약 80M | 450M | Pi0 3.5B(SmolVLA 논문 표) |
| 학습 VRAM (BS 8) | 2~6 GB | 10~16 GB(RTX 4080 이상 권장) | 24~40 GB. 12 GB로는 불가 |
| 학습 시간(공식) | A100, BS 64에서 100k 스텝 약 1.5시간. 4090에서 5 epoch(4.5만 프레임) 30~60분 | A100, BS 64에서 20k 스텝 약 4~5시간 | 해당 없음 |
| 데이터 | 약 50회부터 시작 | 약 50회 권장(25회는 부족했다고 문서에 적힘) | - |
| 추론 | 순전파 한 번에 100스텝 청크를 낸다. 이 맥에서 카메라 2대(640×480) 기준 MPS 39 ms, CPU 101 ms [실행확인] | flow matching 10단계. 공식 문서는 느린 VLA에 RTC나 비동기 추론을 권한다 | 같음 |
| 언어 지시 | 필요 없음 | 쓰지만 이 과제에는 필요 없음 | - |

- SmolVLA는 사전학습 덕에 일반화가 더 좋다. 논문의 SO101 레고 집기에서 ACT는 분포 안 70%, 밖 40%, SmolVLA는 90%, 50%였다. 하지만 이 과제는 카메라와 장면이 고정이라 일반화 이득이 작다. 반대로 지연과 12 GB 메모리 한계는 바로 손해로 이어진다.
- ACT 논문은 배터리 끼우기 같은 정밀 과제 6개를 10분 분량 시범으로 배워 80~90% 성공했다.
- SmolVLA는 ACT가 정지 소켓에서도 60%를 넘지 못하고 시간이 남을 때만 시험한다. 그때는 배치 4 이하로 돌린다 [미검증].
- **소켓 예측값을 추가 관측으로 넣는 법**
  - ACT 자체는 `observation.environment_state`를 별도 입력으로 받는다.
  - 하지만 LeRobot의 녹화, 실행 CLI는 로봇이 내는 숫자 관측을 모두 `observation.state` 하나로 묶는다.
  - 그래서 가장 간단한 우회는 "SO-101 로봇 플러그인이 ArUco, 예측값을 상태 벡터 뒤에 붙이는 것"이다(5.3절).
  - 1차 모델에는 넣지 않는다. 고정 카메라에 마커가 보이므로 ACT가 이미지에서 배운다.

## 2. 준비물

- **로봇**
  - SO-101 리더, 팔로워와 각각의 전원.
  - 서보 전원을 한 번에 끌 수 있는 스위치 멀티탭.
- **카메라 2대**
  - `front`: 고정. 선기 님 RealSense D435의 컬러 영상(맥에서 OpenCV로 안 열리면 C270). 로봇 오른쪽 앞 책상 위 40 cm에
    세운 셀카봉에서 선박 소켓과 자동차 충전구 사이를 본다(real/README.md 2절). 세운 판 마커와 L자 플러그가 함께 보인다.
  - `wrist`: 키트 카메라를 공식 손목 거치대로 그리퍼에 단다. 옆으로 꽂을 때는 그리퍼가 아래를 봐서 깔때기 입구가 거의
    안 보인다(플러그 몸통과 판 윗면이 보인다). 그래도 접근 단계 정보가 있어 넣는다.
  - 두 카메라 모두 640×480, 30 fps.
- **시뮬레이션 시범(선택, 2026-10-31 추가)**: `make_sim_dataset.py`(MuJoCo, 맥) 또는 `isaac/run_plugbot_isaac.py --dataset 50`
  (Isaac, GPU 노트북)이 같은 카메라 이름(front, wrist)과 단위로 에피소드를 만든다. `tools/to_lerobot.py`로 LeRobot 데이터셋이
  된다(lerobot 0.6.1로 변환과 다시 읽기 확인). 실물 데이터와 섞을지는 실물 50회로 먼저 학습해 본 뒤 정한다.
- **고정 장치**
  - 로봇 받침, 갑판, 카메라를 테이프, 클램프로 고정한다. 녹화를 시작한 뒤에는 시연 때까지 움직이지 않는다.
  - 플러그 거치대: 에피소드 시작 위치다.
  - 레버 각도 눈금과 메트로놈 앱(흔들림 주기 고정용).
  - 조명 스탠드(현장 조명 차이 대비).
- **소프트웨어**
  - Python 3.12, miniforge(conda), ffmpeg, Hugging Face 계정과 write 토큰.
  - wandb는 선택이다.
- **우분투 GPU**
  - RTX 5070 Ti Laptop은 Blackwell 세대라 CUDA 12.8 이상으로 빌드된 PyTorch가 필요하다.
  - 드라이버가 580.65 이상이면 PyPI 기본 휠(cu130)을 쓴다. 570.86~580이면 cu128 휠을 쓴다(설치 문서).
- **기록표**(공유 스프레드시트): 에피소드 번호, 데이터셋, 조건(정지 각도 또는 흔들림 주기, 진폭, 시작 위상), 결과, 메모.

## 3. 단계별 명령

### 3.1 [맥] 설치 (오늘)

```bash
conda create -y -n lerobot python=3.12 && conda activate lerobot
conda install -y ffmpeg -c conda-forge   # 없으면 torchcodec이 못 올라와 pyav로 대체된다 [실행확인]
pip install 'lerobot[core_scripts,feetech]==0.6.1'
hf auth login                            # write 토큰
export HF_USER=$(NO_COLOR=1 hf auth whoami | awk -F': *' 'NR==1 {print $2}'); echo $HF_USER
```

### 3.2 [맥] 포트, 카메라 찾기

```bash
lerobot-find-port              # 안내대로 팔 USB를 하나씩 뽑아 포트를 확인
lerobot-find-cameras opencv    # 인덱스와 기본 해상도 확인. 재연결, 재부팅 후 바뀔 수 있다
F_PORT=/dev/tty.usbmodemXXXX   # 팔로워
L_PORT=/dev/tty.usbmodemYYYY   # 리더
CAMS='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}'
```

- 시스템 설정 > 개인정보 보호에서 터미널에 카메라 권한을 준다 [미검증, macOS 일반 사항].
- iPhone 연속성 카메라가 카메라 인덱스를 차지할 수 있으니 끈다 [미검증].

### 3.3 [맥] 모터 설정과 보정

모터 ID 설정은 이미 원격조종을 해 봤으면 건너뛴다. 모터를 교체했을 때만 한다.

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=$F_PORT
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=$L_PORT
```

보정은 다음 원칙을 따른다.

- 0.6.1은 보정 파일을 아래 경로에서 찾는다 [실행확인].
  - `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json`
  - `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<id>.json`
- 파일이 없으면 지금 한 번 보정한다.
- 그 뒤에는 수집부터 시연까지 다시 보정하지 않는다. id도 끝까지 같은 값을 쓴다.

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=$F_PORT --robot.id=plug_follower
lerobot-calibrate --teleop.type=so101_leader --teleop.port=$L_PORT --teleop.id=plug_leader
```

### 3.4 [맥] 원격조종 확인

```bash
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=$F_PORT --robot.id=plug_follower \
  --robot.cameras="$CAMS" \
  --teleop.type=so101_leader --teleop.port=$L_PORT --teleop.id=plug_leader \
  --display_data=true
```

화면만 보고 꽂아 본다. 공식 팁대로 사람이 카메라 영상만 보고 할 수 있어야 정책도 배운다.

### 3.5 [맥] 데이터 녹화

**맥에서 녹화가 되는가: 된다.**
- 공식 문서 예시가 macOS 포트(`/dev/tty.usbmodem…`)와 AVFOUNDATION 카메라 백엔드를 쓴다.
- 커뮤니티 사례로 M4 MacBook에서 50회를 녹화한 기록이 있다.
- 키 입력은 손쉬운 사용 권한이 없으면 자동으로 터미널 입력으로 넘어간다. 터미널을 앞에 두고 `n`(다음), `r`(다시), `q`(종료)를 쓴다.
- 이 맥에서는 로봇 없이 인자 해석까지만 확인했다 [실행확인].

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=$F_PORT --robot.id=plug_follower \
  --robot.cameras="$CAMS" \
  --teleop.type=so101_leader --teleop.port=$L_PORT --teleop.id=plug_leader \
  --dataset.repo_id=$HF_USER/plug_static --dataset.no_stamp=true \
  --dataset.single_task="Insert the plug into the socket" \
  --dataset.num_episodes=25 --dataset.episode_time_s=25 --dataset.reset_time_s=20 \
  --dataset.streaming_encoding=true --dataset.encoder_threads=2 \
  --dataset.private=true --display_data=false
```

- **기본값**: 에피소드 60 s, 리셋 60 s, 50회. 위처럼 줄이고, 끝나면 `→`로 일찍 넘긴다.
  - 정지 소켓: `episode_time_s=25`.
  - 흔들림: `--dataset.repo_id=$HF_USER/plug_moving --dataset.episode_time_s=30`.
- **`--dataset.no_stamp=true`**: 0.6.1은 기본으로 이름 뒤에 날짜, 시각을 붙인다(예: `plug_static_20261024_143000`) [코드확인]. 이름을 고정하려면 켠다.
- **이어서 녹화**: 같은 명령에 아래를 붙인다. `num_episodes`는 총수가 아니라 추가할 개수다.

  ```bash
  --resume=true --dataset.root=$HOME/.cache/huggingface/lerobot/$HF_USER/plug_static --dataset.num_episodes=<추가 개수>
  ```

- **연결 실패로 0회에서 끝난 경우**: 빈 폴더가 남아 다음 실행이 `FileExistsError`로 멈춘다 [실행확인].
  - 에피소드가 하나도 없으면 그 폴더를 지운다.
  - 에피소드가 있으면 `--resume=true`로 잇는다.
- **녹화 끝에 나오는 Cadence 보고**: 30 Hz 가까이 나와야 한다.
  - 밀리면 `--display_data=false`를 유지하고, `--dataset.rgb_encoder.vcodec=auto`(맥은 VideoToolbox)를 쓴다.
  - 커뮤니티 사례에서는 녹화 중 화면 표시를 켜면 30 Hz가 9.8 Hz로 떨어졌다.
- **업로드**: 기본으로 Hub에 올라간다. 우분투에서 같은 계정으로 로그인하면 `repo_id`로 받는다.

### 3.6 [맥/우분투] 확인: 시각화, 재생, 정보

```bash
lerobot-dataset-viz --repo-id $HF_USER/plug_static --episode-index 0      # rerun 창 [실행확인]
# 웹: https://huggingface.co/spaces/lerobot/visualize_dataset (비공개 데이터는 로그인)
lerobot-replay --robot.type=so101_follower --robot.port=$F_PORT --robot.id=plug_follower \
  --dataset.repo_id=$HF_USER/plug_static --dataset.episode=0
lerobot-edit-dataset --repo_id $HF_USER/plug_static --operation.type info  # 에피소드, 프레임 수 [실행확인]
```

replay는 정지 소켓 에피소드로만 해 본다. 흔들림 에피소드는 소켓 위상이 달라 의미가 없다.

### 3.7 [맥/우분투] 나쁜 에피소드 삭제와 데이터 합치기

```bash
lerobot-edit-dataset --repo_id $HF_USER/plug_static --new_repo_id $HF_USER/plug_static_clean \
  --operation.type delete_episodes --operation.episode_indices "[3, 17]"
lerobot-edit-dataset --new_repo_id $HF_USER/plug_all --operation.type merge \
  --operation.repo_ids "['$HF_USER/plug_static_clean', '$HF_USER/plug_moving_clean']"
```

- 공식 문서의 merge 예시는 `--repo_id`를 쓴다. 하지만 0.6.1은 `--new_repo_id`가 없으면 오류를 낸다 [실행확인].
- 합칠 데이터는 카메라 이름, 해상도, 상태 차원이 같아야 한다.
- 합친 데이터를 우분투로 옮기는 방법은 두 가지다.
  - `--push_to_hub true`를 붙여 Hub로 올린다. 공개 여부는 [미검증].
  - `~/.cache/huggingface/lerobot/$HF_USER/plug_all` 폴더를 우분투의 같은 경로로 복사한다.

### 3.8 [우분투] 설치와 이전 (10/27)

```bash
nvidia-smi                                   # 드라이버 버전 확인
conda create -y -n lerobot python=3.12 && conda activate lerobot
conda install -y ffmpeg -c conda-forge
# 드라이버 570.86~580이면 먼저: pip install --index-url https://download.pytorch.org/whl/cu128 torch torchvision
pip install 'lerobot[core_scripts,training,feetech]==0.6.1'
python -c "import torch;print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0), torch.cuda.get_arch_list())"
# 출력에 sm_120이 있어야 한다 [미검증]
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1     # 공식 문서 방식. 재부팅하면 다시 해야 한다
hf auth login
```

- **보정 파일 복사(필수)**: 맥의 `~/.cache/huggingface/lerobot/calibration/`을 우분투의 같은 경로로 통째로 복사한다(rsync나 USB). 같은 id를 쓴다.
  - 우분투에서 다시 보정하면 관절값이 조금 달라진다. 그러면 1.5 mm 여유의 삽입이 깨질 수 있다.
- **포트와 카메라**: `lerobot-find-port`와 `lerobot-find-cameras opencv`로 확인한다.
  - 카메라 이름(`front`, `wrist`), 해상도, fps는 녹화 때와 같아야 한다.
  - 실행할 때 이미지 키가 다르면 오류가 난다 [코드확인].

  ```bash
  F_PORT=/dev/ttyACM0; L_PORT=/dev/ttyACM1
  CAMS='{ front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}}'
  ```

- 3.4의 원격조종 명령을 우분투에서 한 번 돌려 보정이 맞는지 본다.

### 3.9 [우분투] 학습

설정 근거는 5절에 있다.

```bash
# 파이프라인 시험(약 수 분): 2000스텝, Hub 업로드 끔
lerobot-train --dataset.repo_id=$HF_USER/plug_static_clean --policy.type=act --policy.device=cuda \
  --policy.push_to_hub=false --output_dir=outputs/train/act_smoke --job_name=act_smoke \
  --batch_size=8 --steps=2000 --save_freq=1000 --wandb.enable=false

# 본 학습(밤새)
lerobot-train \
  --dataset.repo_id=$HF_USER/plug_all \
  --policy.type=act --policy.device=cuda \
  --policy.repo_id=$HF_USER/act_plug_v2 --policy.private=true \
  --output_dir=outputs/train/act_plug_v2 --job_name=act_plug_v2 \
  --batch_size=8 --steps=80000 --save_freq=10000 --num_workers=4 \
  --dataset.image_transforms.enable=true \
  --wandb.enable=false

# 끊겼을 때 이어서
lerobot-train --config_path=outputs/train/act_plug_v2/checkpoints/last/pretrained_model/train_config.json --resume=true
```

- 같은 형태의 명령(합성 데이터, MPS, 20스텝, `--dataset.eval_split`, `--eval_steps` 포함)이 이 맥에서 돌았다 [실행확인].
- `--policy.repo_id`를 주지 않으려면 `--policy.push_to_hub=false`가 필요하다. 그렇지 않으면 오류가 난다 [코드확인].

### 3.10 [우분투] 로봇에서 실행과 평가 녹화

0.6.1에서는 정책 실행이 `lerobot-rollout` 전용이다.
- `lerobot-record`는 정책을 받지 않는다.
- `lerobot-record`는 `eval_`로 시작하는 이름도 거부한다 [코드확인].

```bash
CKPT=outputs/train/act_plug_v2/checkpoints/last/pretrained_model

# 빠른 시험(녹화 없음)
lerobot-rollout --strategy.type=base --policy.path=$CKPT \
  --robot.type=so101_follower --robot.port=$F_PORT --robot.id=plug_follower --robot.cameras="$CAMS" \
  --task="Insert the plug into the socket" --duration=30 --device=cuda --display_data=true

# 평가 20회 녹화(리셋 때 리더로 팔을 홈으로 되돌리고 플러그를 거치대에 다시 놓는다)
lerobot-rollout --strategy.type=episodic --policy.path=$CKPT --policy.n_action_steps=10 \
  --robot.type=so101_follower --robot.port=$F_PORT --robot.id=plug_follower --robot.cameras="$CAMS" \
  --teleop.type=so101_leader --teleop.port=$L_PORT --teleop.id=plug_leader \
  --dataset.repo_id=$HF_USER/rollout_plug_v2_static \
  --dataset.single_task="Insert the plug into the socket" \
  --dataset.num_episodes=20 --dataset.episode_time_s=25 --dataset.reset_time_s=30 \
  --dataset.push_to_hub=false --device=cuda
```

- **평가 데이터 이름은 반드시 `rollout_`로 시작해야 한다.** 0.6.1 코드가 이를 강제한다 [코드확인].
  - 공식 문서 예시의 `eval_so100`, `my_eval_data`는 이 규칙에 걸린다.
  - 날짜 접미사가 붙으니 `--dataset.no_stamp=true`를 붙이거나, 출력 로그에서 실제 이름을 확인한다.
- `--policy.n_action_steps`와 `--policy.temporal_ensemble_coeff`는 실행할 때 덮어쓸 수 있다 [실행확인].
- **리셋 동안 리더 팔이 스스로 움직인다.** 팔로워 자세로 맞춘 뒤 토크를 푸는 동작이다 [코드확인]. 손을 조심한다.
- 맥에서도 `--device=mps`로 실행할 수 있다. 다만 ACT 한 번에 39 ms가 걸리므로 `n_action_steps`는 5 이상만 쓴다. 커뮤니티 사례에서 맥 MPS(약 15 Hz)는 40%, CUDA(30 Hz)는 90%였다.

## 4. 데이터 수집 요령

**에피소드 정의**는 항상 같게 한다. 시작은 팔 홈 자세에 플러그가 거치대에 있는 상태다.
1. 플러그를 집는다.
2. 깔때기 위로 옮긴다.
3. 꽂는다(자석 래치가 붙는다).
4. 놓는다.
5. 홈으로 돌아온다.
6. `→`로 에피소드를 끝낸다.

집기가 계속 문제면 "그리퍼에 쥔 채 시작"으로 단순화한다.

**몇 회 녹화하나**

| 데이터셋 | 횟수 | 조건 |
|---|---|---|
| `plug_static` | 50 | 갑판 정지. 레버 눈금 5단계(예: −2, −1, 0, +1, +2) × 10회 |
| `plug_moving` | 60~80 | 느린 흔들림. 메트로놈으로 주기를 고정하고 진폭 눈금은 1~2단계. 시작 위상은 무작위 |
| 보완(10/30) | 30~40 | 1차 평가에서 실패한 조건 위주로, 회복 시범 포함 |

- 근거
  - 공식 팁: 최소 50회, 위치당 10회.
  - SmolVLA 문서: 25회로는 부족했다.
- 소요 시간: 1회에 30~60초다. 커뮤니티 사례에서 50회에 약 45분이 걸렸다.

**다양화 원칙**: 조금씩 늘린다. 공식 문서도 변화를 한꺼번에 많이 넣으면 성능이 떨어진다고 경고한다.
- 1차: 갑판 각도만 바꾼다.
- 2차: 흔들림을 넣는다.
- 3차: 시작 자세와 거치대 위치를 ±1 cm 정도 바꾼다.
- 카메라, 조명, 배경은 고정한다.

**일관된 시범**
- 한 사람이 조작한다.
- 흔들림에서는 같은 전략을 쓴다. 소켓 3~5 cm 위에서 따라가다가, 갑판이 끝점에서 멈칫하는 정온 구간에 한 번에 내려 꽂는다(README 시뮬레이션의 예측 전략과 같다).
- 망설임과 멈춤을 줄인다.
- 레버를 흔드는 사람의 손은 `front` 화면 밖에 둔다.

**실패 시범 처리**
- 녹화 중에 깔때기를 놓치거나, 플러그를 떨어뜨리거나, 오래 망설이면 즉시 `←`(또는 `r`)로 다시 녹화한다.
- 끝난 뒤에 발견하면 기록표에 표시한다. visualizer로 확인하고 `delete_episodes`로 지운다(3.7). ACT는 시범을 그대로 흉내 내므로 실패 시범은 학습에 넣지 않는다.
- **회복 시범은 넣는다(전체의 10~15%).** 예를 들어 깔때기 벽에 닿은 뒤 미끄러져 들어가는 시범, 빗나간 뒤 위로 올라가 다시 꽂는 시범이다. 실행 중 작은 실수에서 돌아오는 법을 배우게 한다.
- 선택(11/1~2): `lerobot-rollout --strategy.type=dagger`로 정책이 실패하려 할 때 리더로 개입한 구간만 모은다. 키는 스페이스(일시정지), 탭(개입)이다 [코드확인]. 데모 데이터와 합칠 때 `intervention` 열 때문에 merge가 되는지는 [미검증].

**매 녹화 세션 후**: Cadence 약 30 Hz를 확인하고, 첫 에피소드를 visualizer로 보고, replay를 한 번 한다.

## 5. 학습 설정

### 5.1 핵심 하이퍼파라미터

| 항목 | 값 | 메모 |
|---|---|---|
| `policy.type` | `act` | ResNet18(ImageNet) + 트랜스포머, VAE 사용(기본) |
| `chunk_size` | 100(기본) | 30 fps에서 3.3 s다. 반응성은 실행할 때 `n_action_steps`로 조절한다(5.2). 시간이 남으면 50으로 한 번 비교한다 |
| `batch_size` | 8(기본) | 가이드상 ACT는 BS 8에서 2~6 GB다. 16도 12 GB에 들어갈 가능성이 높지만 [미검증], `nvidia-smi`로 확인한다 |
| `steps` | 데이터 기준 5~10 epoch | 예: 150회 × 15 s × 30 fps ≈ 6.8만 프레임. BS 8로 10 epoch면 약 8.4만 스텝. 기본값은 10만 |
| 학습률 | 1e-5(기본, 스케줄러 없음) | 바꾸지 않는다 |
| `save_freq` | 10000 | 4만, 6만, 8만 스텝 체크포인트 중 정지 5회 예비 평가로 고른다 |
| `num_workers` | 4 | RAM 16 GB가 모자라 스왑이 생기면 2로 줄인다 |
| `dataset.image_transforms.enable` | true | 밝기, 대비, 채도 등을 바꿔 현장 조명 변화에 대비한다 |
| `policy.use_amp` | false(기본) | 메모리가 모자랄 때만 true. 정확도 영향은 [미검증] |
| `dataset.eval_split` / `eval_steps` | 첫 모델만 0.1 / 5000 | 검증 손실을 본다. 실제 성공률과의 상관은 약하므로 최종 학습에서는 끈다 |

- 시간 추정: 첫 1000스텝 로그의 속도로 계산한다.
  - 공식 기준: 4090에서 4.5만 프레임 5 epoch에 30~60분.
  - 커뮤니티 기준: DGX Spark에서 BS 8, 10만 스텝에 약 7시간(3.9 it/s).
  - 5070 Ti Laptop에서는 [미검증]. 하루 한 번 밤새 학습하도록 일정을 짠다.
- 노트북은 전원을 연결하고 절전을 끈다.

### 5.2 실행 시 청크 설정 (재학습 없이 비교)

| 설정 | 플래그 | 특징 |
|---|---|---|
| A | 없음(`n_action_steps=100`) | 3.3 s 동안 개루프로 움직인다. 정지 소켓에는 충분하고 흔들림에는 불리하다 |
| B | `--policy.n_action_steps=10` | 0.33 s마다 다시 계획한다. 흔들림 1순위 후보 |
| C | `--policy.temporal_ensemble_coeff=0.01 --policy.n_action_steps=1` | 매 스텝 추론해 겹친 예측을 평균한다(ACT 원 논문 값 0.01). 부드럽지만 오래된 예측에 가중치가 커서 움직이는 목표에는 늦을 수 있다 |

- 흔들림 본평가 전에 A, B, C를 각 5회 해 보고 하나를 고른다.
- rollout 로그의 Cadence가 30 Hz를 유지하는지 본다. C는 GPU에서 매 스텝 추론이 33 ms 안에 끝나야 한다 [미검증].

### 5.3 소켓 예측값을 관측으로 넣기 (선택, 10/30에 결정)

- **ACT가 받는 입력** [코드확인]: 이미지, `observation.state`, `observation.environment_state`(별도 토큰). 근거는 `configuration_act.py`와 `modeling_act.py`다.
- **CLI의 한계** [코드확인]
  - `lerobot-record`와 `lerobot-rollout`은 로봇이 내는 숫자 관측을 모두 `observation.state`로 묶는다.
  - rollout은 그중 이름이 `.pos`나 `.vel`로 끝나는 것만 정책에 넘긴다.
  - 그래서 CLI만으로는 `environment_state`를 채울 수 없다. 별도의 Python 녹화, 추론 루프가 필요한데, 이번 일정에는 추천하지 않는다.
- **가장 간단한 우회: 로봇 플러그인**
  - SO-101을 감싸 `front` 영상에서 ArUco를 찾는다.
  - 마커 중심(u, v), 0.3 s 뒤 예측(u, v), 검출 여부를 `.pos` 이름으로 상태에 붙인다. 상태는 6차원에서 11차원이 된다.
  - 좌표는 화면 비율이라 카메라 보정이 필요 없다. 대신 카메라를 움직이면 무효가 된다.
- **이 맥에서 확인한 것** [실행확인]
  - 플러그인이 등록된다.
  - `lerobot-record --robot.type=plugbot_follower`의 인자가 해석된다.
  - 만들어진 데이터셋의 상태가 11차원이다.
  - 합성 영상에서 마커를 검출하고 예측까지 약 1 ms가 걸린다.
  - 실제 로봇에서의 동작과 성공률 효과는 [미검증].

패키지 구조는 `lerobot_robot_plugbot/pyproject.toml`(name = "lerobot_robot_plugbot", 접두사 필수)과 `lerobot_robot_plugbot/lerobot_robot_plugbot/{__init__.py, config_plugbot_follower.py, plugbot_follower.py}`다. `__init__.py`는 두 클래스를 import한다.

```python
# config_plugbot_follower.py
from dataclasses import dataclass
from lerobot.robots.config import RobotConfig
from lerobot.robots.so_follower import SOFollowerRobotConfig

@RobotConfig.register_subclass("plugbot_follower")
@dataclass
class PlugbotFollowerConfig(SOFollowerRobotConfig):
    marker_cam: str = "front"
    marker_id: int = 0
    predict_s: float = 0.3

# plugbot_follower.py
import collections, time
from functools import cached_property
import cv2, numpy as np
from lerobot.robots.so_follower import SOFollower
from .config_plugbot_follower import PlugbotFollowerConfig

EXTRA = ["sock_u.pos", "sock_v.pos", "sock_u_pred.pos", "sock_v_pred.pos", "sock_ok.pos"]

class PlugbotFollower(SOFollower):
    config_class = PlugbotFollowerConfig
    name = "plugbot_follower"

    def __init__(self, config):
        super().__init__(config)
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        self._hist = collections.deque(maxlen=90)
        self._last = (0.5, 0.5)

    @cached_property
    def observation_features(self):
        return {**self._motors_ft, **{k: float for k in EXTRA}, **self._cameras_ft}

    def _predict(self, t):  # 자리표시: 1차 외삽. 팀의 AR(6) 예측기로 바꾼다
        h = np.array(self._hist) if self._hist else np.zeros((0, 3))
        h = h[h[:, 0] > t - 0.3]
        if len(h) < 2:
            return self._last
        s = self.config.predict_s
        return tuple(float(np.polyval(np.polyfit(h[:, 0] - t, h[:, i], 1), s)) for i in (1, 2))

    def get_observation(self):
        obs = super().get_observation()
        img = obs[self.config.marker_cam]
        t = time.perf_counter()
        corners, ids, _ = self._det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
        ok = 0.0
        if ids is not None and self.config.marker_id in ids.flatten():
            c = corners[list(ids.flatten()).index(self.config.marker_id)][0]
            self._last = (float(c[:, 0].mean() / img.shape[1]), float(c[:, 1].mean() / img.shape[0]))
            self._hist.append((t, *self._last))
            ok = 1.0
        obs.update(dict(zip(EXTRA, [*self._last, *self._predict(t), ok])))
        return obs
```

- **설치**: 맥과 우분투 둘 다에서 `pip install -e lerobot_robot_plugbot`.
- **명령 바꾸기**: `--robot.type=so101_follower` 대신 아래를 쓴다.

  ```bash
  --robot.type=plugbot_follower --robot.calibration_dir=$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower
  ```

  보정 파일을 재사용하려면 `calibration_dir`가 필요하다. 없으면 `robots/plugbot_follower/`에서 찾는다 [실행확인].
- **주의**
  - 상태 차원이 바뀌므로 기존 데이터와 합칠 수 없다. 이 플러그인으로 다시 녹화해야 한다(정지 40 + 흔들림 60, 약 2시간).
  - 마커가 가려지면 마지막 값과 `sock_ok=0`을 낸다.

## 6. 평가 방법

**공통 조건**
- 우분투와 CUDA를 쓴다. 같은 체크포인트와 같은 실행 설정(5.2에서 고른 것)을 쓴다.
- `lerobot-rollout --strategy.type=episodic`으로 영상을 녹화해 증거로 남긴다.
- 시행 순서는 무작위로 한다. 사람이 로봇이나 플러그를 건드리면 실패다.

**성공 기준**(모두 만족해야 성공)
1. 핀이 보어 바닥까지 들어가 자석이 붙는다. 플러그 몸통에 그은 깊이 선이 소켓 입구 아래로 사라지는 것으로 판정한다.
2. 그리퍼가 놓고 팔이 물러난다. 그 뒤 플러그가 빠지거나 기울지 않는다.
3. 시간 안에 끝난다. 정지 소켓은 25 s, 흔들림은 30 s다.
4. 갑판이 밀릴 정도의 충돌, 서보 과부하 정지, 사람 개입이 없다.

**부분 성공**(따로 기록): 깔때기에는 들어갔지만 끝까지 들어가지 않은 경우.

**정지 소켓 20회**
- 학습 범위 안의 갑판 각도 5단계 × 4회를 무작위 순서로 한다.

**흔들림 20회**
- 학습 때와 같은 메트로놈 주기와 진폭 눈금을 쓴다.
- 정책 시작 시점을 0~1주기 사이에서 무작위로 늦춰 시작 위상을 섞는다.
- 실제 진폭과 주기는 녹화된 `front` 영상에서 ArUco로 시행마다 사후 계산해 표에 적는다.

**기록 항목**: 성공, 부분, 실패, 삽입까지 걸린 시간, 실패 유형(소켓 놓침, 선체 찌름, 걸림, 놓기 실패, 집기 실패, 시간 초과).

**보고 방식**: k/20과 95% 신뢰구간(Wilson)을 함께 적는다.

| 결과 | 성공률 | 95% 신뢰구간 |
|---|---|---|
| 18/20 | 90% | 70~97% |
| 16/20 | 80% | 58~92% |
| 12/20 | 60% | 39~78% |
| 10/20 | 50% | 30~70% |

20회로는 10%p 차이를 가를 수 없다는 점을 발표에서 밝힌다.

**목표(팀 제안)**: 정지 소켓 16/20 이상, 흔들림 10/20 이상. 고전 기준선(ArUco + AR 예측 제어기)도 같은 20+20 절차로 재서 표 하나로 비교한다.

## 7. 일정 (10/27~11/7)

**사전(10/23 금~10/26 월, 맥)**
- 설치하고, 0.6.1 보정 파일을 확인하고, 리그를 고정한다.
- 시험 녹화 10회 → 확인 → `plug_static` 50회 → `plug_moving` 30~40회.
- 선택: 맥 MPS로 2000스텝 시험 학습을 하고 `rollout --strategy.type=base --device=mps`로 파이프라인을 미리 확인한다.

| 날짜 | 할 일 | 판단, 산출물 |
|---|---|---|
| 10/27 화 | 우분투 설치, GPU(sm_120) 확인, 보정 복사, 원격조종 확인, 2000스텝 시험 학습. 밤: v1(`plug_static_clean`) 학습 | 학습 속도(it/s)로 이후 스텝 수를 정한다 |
| 10/28 수 | v1 정지 소켓 예비 10회. 흔들림 데이터를 총 60~80회로 채운다. 밤: v2(`plug_all`) 학습 | v1 정지가 50% 미만이면 카메라 시야와 시범 일관성부터 점검 |
| 10/29 목 | v2 정지 20회, 흔들림 A/B/C 각 5회. 실패 유형 분석 | 실패 조건 목록 |
| 10/30 금 | 보완 녹화 30~40회(실패 조건, 회복 시범). 밤: v3 학습 | 5.3 플러그인을 할지 결정 |
| 10/31 토 | v3 정식 평가 20+20 | 1차 수치 |
| 11/1 일~11/2 월 | 개선 한 가지만: (a) dagger 보정 데이터, (b) 플러그인으로 재녹화, (c) 데이터 추가 중 택1. 밤: v4 학습 | |
| 11/3 화 | v4와 v3 비교 평가. 최종 모델을 고정하고 이후 데이터, 코드를 동결 | 최종 체크포인트(로컬과 Hub 둘 다 보관) |
| 11/4 수 | 고전 기준선 20+20 | 비교표 |
| 11/5 목 | 현장 조건 리허설(조명, 철거 후 재설치, 카메라 위치 복원). 백업 영상 촬영 | 설치 체크리스트 |
| 11/6 금 | 최종 평가 20+20(발표 수치), 발표 자료 | |
| 11/7 토 | 시연 | |

## 8. 위험과 대안

| 위험 | 대안 |
|---|---|
| 우분투에서 Blackwell용 torch나 드라이버가 안 맞음 | cu128 휠을 다시 설치하고 드라이버를 올린다. 그래도 안 되면 HF Jobs(`--job.target=a10g-small`, 유료)나 맥 MPS(가이드상 ACT BS 4로 5 epoch에 6~14시간) |
| 맥과 우분투의 보정 불일치 | 보정 폴더를 복사하고 같은 id를 쓴다. 다시 보정하지 않는다. 모터를 교체했다면 다시 보정하고 데이터 일부를 재녹화한다 |
| 카메라 인덱스가 바뀌어 `front`와 `wrist`가 뒤바뀜 | 이름만 검사하므로 오류 없이 엉뚱하게 동작한다. 매 세션 `lerobot-find-cameras`와 `--display_data=true`로 화면을 확인한다 |
| 맥 녹화 프레임 밀림 | 화면 표시를 끄고, `--dataset.rgb_encoder.vcodec=auto`를 쓰고, Cadence 보고를 확인한다 |
| 현장 조명, 배경이 다름 | `image_transforms`를 켜고, 조명 스탠드와 가림막을 쓴다. 가능하면 현장에서 10~20회를 추가 녹화해 합친 뒤 밤새 재학습한다 |
| 흔들림 성공률이 낮음 | 흔들림을 더 느리고 작게 한다. 5.2의 B, C를 쓰고, 회복 시범을 늘리고, 5.3 플러그인을 쓴다. 최후에는 정지 소켓은 ACT, 흔들림은 고전 제어기로 나눠 시연하고 수치를 그대로 보고한다 |
| 서보 과열, 과부하(꽂은 채 계속 미는 경우) | 래치가 잡으면 놓는 동작을 시범에 넣는다. 시행 사이에 쉬고, 전원 스위치를 손 닿는 곳에 둔다. `--robot.max_relative_target`으로 한 스텝 이동량을 제한할 수 있다 [코드확인, 적정값은 미검증] |
| 학습 중 RAM 16 GB 부족 | `--num_workers=2`로 줄이고 다른 프로그램을 끈다 |
| 현장에 네트워크가 없음 | 데이터와 모델을 로컬 경로로 쓴다(`--policy.path=outputs/...`, `--dataset.push_to_hub=false`) |
| 시범 품질이 들쭉날쭉 | 조작자를 한 사람으로 정하고, 4절 절차를 인쇄해 옆에 둔다 |
| 문서와 버전 차이 | 0.6.1로 고정한다. main 문서와 다를 때는 `--help`로 확인한다. 이번에 찾은 차이: 평가 이름에 `rollout_` 필수, merge에 `--new_repo_id` 필수, ACT 크기 52M |

## 9. 출처

- LeRobot 문서(0.6.1과 일치 확인)
  - 설치: https://huggingface.co/docs/lerobot/installation
  - SO-101: https://huggingface.co/docs/lerobot/so101
  - 실물 모방학습(녹화, 재생, 학습, rollout): https://huggingface.co/docs/lerobot/il_robots
  - ACT: https://huggingface.co/docs/lerobot/act
  - SmolVLA: https://huggingface.co/docs/lerobot/smolvla
  - 추론, rollout 전략: https://huggingface.co/docs/lerobot/inference
  - 카메라: https://huggingface.co/docs/lerobot/cameras
  - 데이터 도구: https://huggingface.co/docs/lerobot/using_dataset_tools
  - 스트리밍 인코딩: https://huggingface.co/docs/lerobot/streaming_video_encoding
  - 연산 자원 가이드(VRAM, 시간): https://huggingface.co/docs/lerobot/hardware_guide
  - 노트북(학습 시간): https://huggingface.co/docs/lerobot/notebooks
  - 장치 플러그인: https://huggingface.co/docs/lerobot/integrate_hardware
  - HIL, DAgger: https://huggingface.co/docs/lerobot/hil_data_collection
- GitHub v0.6.1 소스: https://github.com/huggingface/lerobot/tree/v0.6.1
  - `src/lerobot/policies/act/configuration_act.py`, `modeling_act.py`
  - `src/lerobot/scripts/lerobot_record.py`, `lerobot_rollout.py`, `lerobot_edit_dataset.py`
  - `src/lerobot/configs/dataset.py`, `configs/train.py`
  - `src/lerobot/rollout/configs.py`, `rollout/context.py`
  - `src/lerobot/utils/feature_utils.py`
- PyPI lerobot 0.6.1: https://pypi.org/project/lerobot/
- ACT 논문(Zhao et al., 2023): https://arxiv.org/abs/2304.13705
- SmolVLA 논문(2025): https://arxiv.org/abs/2506.01844
- 커뮤니티 사례(맥 녹화, 학습 시간, 맥과 CUDA 추론 성공률): https://dev.classmethod.jp/en/articles/lerobot-so-arm101-act-training-eval/
- 이 저장소: `README.md`(시뮬레이션 결과), `cad/PRINT_GUIDE.md`(마커, 자석 래치)
