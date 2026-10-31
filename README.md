# PlugBot: 흔들리는 선박 충전 소켓에 플러그 꽂기 (MuJoCo, Isaac Sim 5.1, SO-101)

전기차 충전구에서 배운 '꽂기'를 흔들리는 전기 선박 소켓으로 옮기는 아이디어를 시뮬레이션과 3D 프린트 키트로 검증한다.

## 실행

## 바로 시작: GPU 노트북(Ubuntu, Isaac Sim 5.1)

```bash
gh repo clone gawbi/plugbot && cd plugbot   # 비공개 저장소: 먼저 gh auth login
bash isaac/setup_gpu.sh   # 의존성 → 로봇 USD → 정지 1회 확인 → 발표 영상 1개
```

- 성공률 실험은 렌더링 없이(`--headless`) 돌린다. 렌더링은 물리 결과에 영향이 없고 느리기만 하다.
- 발표 영상이 필요할 때만 `--video`(첫 시행을 웹캠과 발표용 두 시점으로 mp4 저장).
- 자세한 순서와 막힐 때: [isaac/README.md](isaac/README.md). 실물 SO-101 시연: [real/README.md](real/README.md).

## 맥(또는 GPU 없는 PC): MuJoCo 실험

```bash
uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python mujoco numpy matplotlib pillow cadquery \
    opencv-contrib-python trimesh usd-core

.venv/bin/python cad/make_parts.py   # CAD: 부품 STEP/STL, 충돌용 소켓 조각, 조립본
.venv/bin/python run_experiments.py --layout side  # 파고 5단계 × 전략 3개 × 30회 (약 1분). top도 같은 방법
.venv/bin/python render_demo.py      # 장면 렌더, 비교 GIF, CAD 렌더
.venv/bin/python make_figures.py     # 기획안 그림(photos/ 필요, 저장소에는 없음)
.venv/bin/python -m real.dry_run    # 실물용 소프트웨어를 가짜 웹캠, 로봇으로 시험
.venv/bin/python isaac/build_assets.py  # Isaac Sim용 URDF와 장면 USD 다시 만들기
```

## 무엇을 시험했나

- 장면: 부두 위 SO-101(Menagerie 모델) + 플러그 + 깔때기 소켓(입구 지름 50 mm, 포착 반경 19 mm, 반경 여유 2 mm).
- 배치 두 가지(`--layout`)
  - side(본선 채택): 현측에 세운 판의 소켓에 L자 플러그(핀 끝이 몸통에서 65 mm)로 옆에서 꽂는다. 자동차는 세운 충전구.
  - top(예선): 갑판 위 소켓에 일자 플러그로 위에서 꽂는다. 자동차는 45° 충전구.
- 선박 운동: JONSWAP(γ = 3.3, 첨두 주기 2 s) 불규칙파로 상하, 좌우, 앞뒤, 롤, 피치. 배는 부두에 옆으로 댄 것으로
  보고 롤 축을 부두와 나란히 둔다(`ShipMotion(berth="alongside")`). 크기는 모형 유의파고 Hs.
- 센서, 제어: 카메라 30 Hz, 지연 0.1 s, 잡음 0.5 mm. 제어 50 Hz, 물리 500 Hz. 서보는 Menagerie STS3215 위치 액추에이터(토크 한계 포함).
- 전략
  - static: 처음 본 소켓 위치로만 간다
  - reactive: 지연된 최신 위치를 따라간다
  - predictive: 최근 3 s 관측으로 AR(6) 모델을 맞춰 0.3 s 앞을 예측하고, 속도가 작아지는 정온 구간에 삽입을 시작한다
- 성공: 핀 끝이 보어 안 목표 깊이까지 들어가 0.2 s 유지. 실패: 소켓 밖 선체에 부딪힘, 그리퍼가 소켓 테두리에 닿음,
  로봇 몸체 충돌, 12 s 안에 못 꽂음.

## 결과 (조건당 30회)

side 배치, 본선 채택 (`outputs/experiment_side.json`)

| 모형 Hs | static | reactive | predictive |
|---|---|---|---|
| 0 cm | 100% | 100% | 100% |
| 2 cm | 96.7% | 100% | 100% |
| 4 cm | 66.7% | 93.3% | 100% |
| 6 cm | 20% | 50% | 86.7% |
| 8 cm | 10% | 6.7% | 80% |

top 배치, 같은 파도 (`outputs/experiment_top.json`): predictive 100, 100, 100, 93.3, 83.3%

- 삽입 중 접촉력(95백분위 중앙값), side Hs 6 cm: static 38 N, reactive 452 N, predictive 38 N
- 자동차(정지): side 세운 충전구, top 45° 충전구 모두 15/15 성공
- L자 플러그의 핀을 처음에 49 mm로 했을 때는 깊이 꽂히는 순간 그리퍼 턱이 깔때기 테두리에 닿아 Hs 6 cm에서 43%였다
  (`experiment_side_old.json`). 65 mm로 늘리고 소켓을 로봇 앞 30 cm로 옮겨 해결했다. 75 mm 이상은 대기 자세에서 손목 카메라
  거치대가 어깨에 걸린다.
- 남은 실패는 대부분 '소켓을 놓쳐 선체를 찌름'이다. 지연을 보상하지 않으면 깔때기를 빗나간다.
- 로봇이 꽂은 뒤 4 s 동안 계속 쥐고 있으면 세 전략 모두 과부하, 이탈로 실패했다(`Params.keep`). 그래서 '래치가 잡으면 놓는' 구조를 기본으로 했다.

예선 결과(작은 소켓: 포착 반경 12 mm, top 배치, 선수 계류, `outputs/experiment_small_socket.json`):
predictive 100, 100, 100, 90, 57%. 소켓만 키운 같은 조건은 `experiment_top_bow.json`(96.7, 93.3%).

## 코드 구조

- `cad/make_parts.py`: 플러그(일자, L자), 소켓, 흔들림 갑판, 세운 판, 충전구 블록, 조립본 (출력 가이드: `cad/PRINT_GUIDE.md`)
- `plugbot/scene.py`: MuJoCo 장면(선박은 mocap, 소켓 충돌은 볼록 조각 16개, 배치 top/side)
- `plugbot/waves.py`: JONSWAP 불규칙파 선체 운동, 실물 힌지 갑판 운동(`HingeDeckMotion`)
- `plugbot/control.py`: IK(top은 위치 우선, side는 위치와 핀 방향 동시), 소켓 추정(지연, 잡음, AR 예측), 삽입 상태기계, 시행 평가
- `run_experiments.py --layout side|top`, `compare_layouts.py`, `render_demo.py`, `make_figures.py`
- `real/`: 실물 SO-101 시연(마커 추적, 보정, 삽입), `real/shaker/`: 흔들림 갑판 아두이노 코드

## 한계

- 시뮬레이션 결과다. 실물 SO-101과의 일치는 본선 전 키트로 측정해야 한다.
- 선박 운동은 실제 RAO가 아닌 근사이고, 축척 1:10(Froude 상사)은 실선 환산을 위한 가정이다.
- 접촉력은 강체 접촉 모델 값이라 절대값보다 전략 간 비교로 읽는다.
- 소켓 위치는 시뮬레이터가 알려 주는 값에 지연, 잡음을 섞은 것이다. 실물에서는 ArUco 마커나 비전 모델로 추정해야 한다.
- 학습 정책(ACT 등)은 아직 없다. 지금의 예측 제어기는 비교 기준이자 시뮬레이션 시범 생성기다.
- 5축 SO-101로 옆으로 꽂을 때는 그리퍼가 늘 수직이라, 플러그를 핀 축 둘레로 돌리는 각도를 따로 맞출 수 없다(늘 0°).
  실제 충전구처럼 핀이 여러 개인 비대칭 커넥터라면 소켓이 핀 축 둘레로 기울지 않아야 한다.
- Isaac Sim 스크립트(`isaac/`)는 아직 top 배치다(소켓은 키운 것으로 다시 만듦).
