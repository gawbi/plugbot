# Isaac Sim 5.1 (GPU 노트북: RTX 5070 Ti, Ubuntu)

MuJoCo 예선 실험(파고별 삽입 성공률)을 Isaac Sim에서 다시 돌리고, 발표용 고화질 영상을 만든다.
이 폴더의 자산과 스크립트는 맥에서 만들었다. Isaac Sim이 필요한 두 스크립트는 **맥에서 실행해 보지 못했다**.

## 맥에서 만들고 확인한 것

| 파일 | 내용 | 확인 |
|---|---|---|
| `so101/so101_new_calib.urdf`, `so101/assets/` | 공식 SO-101 URDF(TheRobotStudio/SO-ARM100, LeRobot 보정 기준) | 받음 |
| `so101/so101_plugbot.urdf` | 위 URDF + 그리퍼에 고정한 플러그, `plug_tip` 링크 | MuJoCo 모델과 플러그 끝 위치 차 0.002 mm |
| `urdf_fk.py` | URDF 순기구학(numpy) | 무작위 자세 20개에서 MuJoCo와 그리퍼 자세 차 0.004 mm |
| `scene/plugbot_scene.usda` | 부두, 바다, 선박(키네마틱 강체), 소켓(보이는 메시 + 볼록 충돌 17개), 웹캠 카메라 | 치수와 입구 높이가 MuJoCo와 같음 |
| `build_assets.py` | 위 두 자산을 다시 만드는 스크립트 | 맥에서 실행 |
| `run_plugbot_isaac.py` | 실험·영상 스크립트(아래) | 문법, 선박·소켓 계산이 MuJoCo와 같음(차 0) |

## GPU 노트북에서 할 일

0. 한 번에: `git clone https://github.com/gawbi/plugbot.git && cd plugbot && ISAAC_SIM=~/isaacsim bash isaac/setup_gpu.sh`
   (아래 1~5를 순서대로 한다. 막히면 단계별로 직접 실행)
1. 저장소를 받는다: `git clone https://github.com/gawbi/plugbot.git`
2. Isaac Sim 파이썬에 의존성:
   ```bash
   cd ~/isaacsim   # Isaac Sim 5.1 설치 폴더
   ./python.sh -m pip install "mujoco>=3.3" imageio imageio-ffmpeg pillow   # numpy는 Isaac Sim 것을 쓴다
   ```
3. 로봇 USD 만들기(한 번):
   ```bash
   ./python.sh ~/plugbot/isaac/import_robot.py
   ```
   오류가 나면 GUI로 한다: File > Import > `so101_plugbot.urdf`, Fix Base Link 켬, Merge Fixed Joints 끔,
   Joint Drive Type = Position, Collision = Convex Decomposition, 저장 `isaac/so101/so101_plugbot.usd`.
4. 장면이 맞는지 눈으로: Isaac Sim GUI에서 `scene/plugbot_scene.usda`를 열고 `so101_plugbot.usd`를 끌어다 놓는다.
   로봇이 부두 위 원점, 소켓이 앞쪽 약 22 cm, 5.5 cm 아래에 있으면 맞다.
5. 실험:
   ```bash
   ./python.sh ~/plugbot/isaac/run_plugbot_isaac.py --hs 0 --strategy predictive --trials 3     # 정지 소켓부터
   ./python.sh ~/plugbot/isaac/run_plugbot_isaac.py --hs 0.06 --strategy predictive --trials 10
   ./python.sh ~/plugbot/isaac/run_plugbot_isaac.py --hs 0.06 --strategy reactive --trials 10
   ```
   결과는 `outputs/isaac_hs*_*.json`. MuJoCo 결과(6 cm: 예측 90%, 추종 37%)와 비교해 발표에 쓴다.
6. 발표 영상: 위 명령에 `--video`(화면 없이도 된다). `outputs/isaac_video_*_Webcam.mp4`, `*_ShowCam.mp4`.
   성공률 실험에는 렌더링이 필요 없다(`--headless`가 빠르다).

## 막힐 때

- **`isaacsim.core...` import 오류**: 5.1에서 모듈 이름이 바뀌었으면 스크립트의 `[ISAAC]` 표시 줄만 고친다.
  계산(예측, IK, 채점)은 `plugbot/`의 검증된 코드라 바꿀 필요가 없다.
- **팔이 목표를 못 따라감(시간 초과만 남)**: `--kp`를 3000~10000으로 올린다. URDF 가져오기 때 드라이브 강성이 낮게 잡혔을 수 있다.
- **플러그가 소켓을 뚫고 지나감**: 물리 간격을 줄인다(`PHYSICS_DT = 0.001`), 또는 충돌 근사를 확인(소켓 Wall00~15, Cap은 convexHull).
- **선박 경로**: 참조로 불러오면 `/World/Env/World/Ship`. 다르면 스크립트의 `SingleXFormPrim` 경로를 바꾼다.
