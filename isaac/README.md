# Isaac Sim 5.1 (GPU 노트북: RTX 5070 Ti, Ubuntu)

MuJoCo 실험(파고별 삽입 성공률)을 Isaac Sim에서 다시 돌리고, 발표용 고화질 영상과 ACT 학습 데이터를 만든다.
기본 배치는 본선과 같은 **side**(현측에 세운 소켓, L자 플러그, 부두에 옆으로 댄 배의 롤)다. 예선 배치는 `--layout top`.
이 폴더의 자산과 스크립트는 맥에서 만들었다. Isaac Sim이 필요한 두 스크립트는 **실제 Isaac에서 실행해 보지 못했다**.

## 맥에서 만들고 확인한 것

| 파일 | 내용 | 확인 |
|---|---|---|
| `so101/so101_new_calib.urdf`, `so101/assets/` | 공식 SO-101 URDF(TheRobotStudio/SO-ARM100, LeRobot 보정 기준) | 받음 |
| `so101/so101_plugbot_side.urdf`, `so101_plugbot.urdf` | 위 URDF + 그리퍼에 고정한 플러그(L자, 일자) + 손목 카메라 거치대와 `wrist_cam_link` | 무작위 자세 20개에서 MuJoCo와 플러그 끝 0.004 mm, 손목 카메라 0.003 mm, 0.001° |
| `scene/plugbot_scene_side.usda`, `plugbot_scene.usda` | 부두, 바다, 선박(키네마틱 강체), 소켓(보이는 메시 + 볼록 충돌 17개), side는 현측 판. 카메라 Webcam(실물 앞 카메라 자리), ShowCam | 소켓 입구, 축이 MuJoCo와 같음(차 0) |
| `build_assets.py` | 위 자산을 다시 만드는 스크립트 | 맥에서 실행 |
| `run_plugbot_isaac.py` | 실험, 미리보기, 영상, 학습 데이터 스크립트(아래) | isaacsim 자리에 MuJoCo를 끼운 가짜 모듈로 끝까지 실행(실험, `--preview`, `--video`, `--dataset`) |

## GPU 노트북에서 할 일

0. 한 번에: `gh repo clone gawbi/plugbot && cd plugbot && bash isaac/setup_gpu.sh`
   (비공개 저장소라 `gh auth login` 필요. pip 설치판은 파이썬 `~/isaacsim_env/bin/python`을 스스로 찾는다.)
   단계: 의존성 → 로봇 USD(side, top) → 미리보기 사진 → 정지 소켓 1회 → 발표 영상 1개.
1. **미리보기부터 본다**: `outputs/isaac_preview_side_{Webcam,ShowCam,Wrist}.png`
   - Webcam: 로봇 오른쪽 앞 위에서 본 세운 판 소켓(실물 앞 카메라와 같은 자리, 화각 60°)
   - ShowCam: 로봇과 선박이 함께 보이는 넓은 시점
   - Wrist: 손목 카메라. 그리퍼가 아래를 봐서 플러그 몸통과 판 윗면이 보인다
   - 로봇이 부두 위 원점, 세운 판 소켓 입구가 정면 30 cm, 3 cm 높이에서 로봇을 보면 맞다.
2. 실험(렌더링 없이):
   ```bash
   PY=~/isaacsim_env/bin/python
   $PY isaac/run_plugbot_isaac.py --hs 0 --strategy predictive --trials 3 --headless     # 정지 소켓부터
   $PY isaac/run_plugbot_isaac.py --hs 0.06 --strategy predictive --trials 30 --headless
   $PY isaac/run_plugbot_isaac.py --hs 0.06 --strategy reactive --trials 30 --headless
   ```
   결과 `outputs/isaac_side_hs*_*.json`. 같은 조건의 MuJoCo 성공률(`outputs/experiment_side.json`)을 함께 출력한다.
   이 스크립트는 충돌을 실패로 따로 세지 않는다(시간 안에 꽂으면 성공). 그래서 MuJoCo보다 조금 높게 나올 수 있다.
3. 발표 영상: `--video`(화면 없이도 된다). `outputs/isaac_video_side_*_{Webcam,ShowCam,Wrist}.mp4`.
4. ACT 학습 데이터: `--hs 0.06 --dataset 50 --headless` → `data/isaac_side/episode_XXXX/{front,wrist}.mp4, data.npz`.
   파고는 에피소드마다 0~6 cm. LeRobot 데이터셋으로: `python tools/to_lerobot.py data/isaac_side --repo-id <HF아이디>/plugbot_isaac`
   (lerobot 0.6.1 환경. 맥에서 MuJoCo 데이터로 변환과 다시 읽기를 확인했다.)

## 막힐 때

- **`isaacsim.core...` import 오류**: 5.1에서 모듈 이름이 바뀌었으면 스크립트의 `[ISAAC]` 표시 줄만 고친다.
  계산(예측, IK, 채점, 선박 자세)은 `plugbot/`의 검증된 코드라 바꿀 필요가 없다.
- **팔이 목표를 못 따라감(시간 초과만 남)**: `--kp`를 3000~10000으로 올린다. URDF 가져오기 때 드라이브 강성이 낮게 잡혔을 수 있다.
- **플러그가 소켓을 뚫고 지나감**: 물리 간격을 줄인다(`PHYSICS_DT = 0.001`), 또는 충돌 근사를 확인(소켓 Wall00~15, Cap은 convexHull).
- **선박 경로**: 참조로 불러오면 `/World/Env/World/Ship`. 다르면 스크립트의 `SingleXFormPrim` 경로를 바꾼다.
- **손목 카메라가 안 나옴**: 로봇 USD 안에 `wrist_cam_link` 프림이 있어야 한다(Merge Fixed Joints 끔). 없으면 경고만 내고 손목 없이 진행한다.
- **카메라 영상이 한 프레임 늦거나 첫 장이 검다**: Isaac 렌더 결과는 몇 번 그린 뒤에 나온다. 학습 데이터는 빈 프레임을 건너뛴다.
