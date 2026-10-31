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
.venv/bin/python run_experiments.py  # 파고 5단계 × 전략 3개 × 30회 (약 1~2분)
.venv/bin/python render_demo.py      # 장면 렌더, 비교 GIF, CAD 렌더
.venv/bin/python make_figures.py     # 기획안 그림(photos/ 필요, 저장소에는 없음)
.venv/bin/python -m real.dry_run    # 실물용 소프트웨어를 가짜 웹캠, 로봇으로 시험
.venv/bin/python isaac/build_assets.py  # Isaac Sim용 URDF와 장면 USD 다시 만들기
```

## 무엇을 시험했나

- 장면: 부두 위 SO-101(Menagerie 모델) + CAD 플러그. 부두보다 5.5 cm 낮은 선박 갑판의 깔때기 소켓(포착 반경 12 mm, 반경 여유 1.5 mm). 자동차 장면은 45° 기울어진 충전구.
- 선박 운동: JONSWAP(γ = 3.3, 첨두 주기 2 s) 불규칙파로 상하, 좌우, 앞뒤, 롤, 피치. 크기는 모형 유의파고 Hs.
- 센서, 제어: 카메라 30 Hz, 지연 0.1 s, 잡음 0.5 mm. 제어 50 Hz, 물리 500 Hz. 서보는 Menagerie STS3215 위치 액추에이터(토크 한계 포함).
- 전략
  - static: 처음 본 소켓 위치로만 간다
  - reactive: 지연된 최신 위치를 따라간다
  - predictive: 최근 3 s 관측으로 AR(6) 모델을 맞춰 0.3 s 앞을 예측하고, 상하 속도가 작아지는 정온 구간에 삽입을 시작한다
- 성공: 핀 끝이 보어 안 목표 깊이까지 들어가 0.2 s 유지. 실패: 소켓 밖 선체에 부딪힘, 로봇 몸체 충돌, 12 s 안에 못 꽂음.

## 결과 (outputs/experiment.json)

| 모형 Hs | static | reactive | predictive |
|---|---|---|---|
| 0 cm | 100% | 100% | 100% |
| 2 cm | 100% | 100% | 100% |
| 4 cm | 90% | 90% | 100% |
| 6 cm | 33% | 37% | 90% |
| 8 cm | 13% | 7% | 57% |

- 삽입 중 접촉력(95백분위 중앙값), Hs 6 cm: static 70 N, reactive 249 N, predictive 41 N
- 자동차 45° 충전구(정지): 15/15 성공
- 실패는 대부분 '소켓을 놓쳐 선체를 찌름'이다. 지연을 보상하지 않으면 깔때기를 빗나간다.
- 로봇이 꽂은 뒤 4 s 동안 계속 쥐고 있으면 세 전략 모두 과부하, 이탈로 실패했다(`Params.keep`). 그래서 '래치가 잡으면 놓는' 구조를 기본으로 했다.

## 코드 구조

- `cad/make_parts.py`: 플러그, 소켓, 흔들림 갑판, 45° 충전구 블록, 조립본 (출력 가이드: `cad/PRINT_GUIDE.md`)
- `plugbot/scene.py`: MuJoCo 장면(선박은 mocap, 소켓 충돌은 볼록 조각 16개)
- `plugbot/waves.py`: JONSWAP 불규칙파 선체 운동
- `plugbot/control.py`: 위치 우선 IK, 소켓 추정(지연, 잡음, AR 예측), 삽입 상태기계, 시행 평가
- `run_experiments.py`, `render_demo.py`, `make_figures.py`

## 한계

- 시뮬레이션 결과다. 실물 SO-101과의 일치는 본선 전 키트로 측정해야 한다.
- 선박 운동은 실제 RAO가 아닌 근사이고, 축척 1:10(Froude 상사)은 실선 환산을 위한 가정이다.
- 접촉력은 강체 접촉 모델 값이라 절대값보다 전략 간 비교로 읽는다.
- 소켓 위치는 시뮬레이터가 알려 주는 값에 지연, 잡음을 섞은 것이다. 실물에서는 ArUco 마커나 비전 모델로 추정해야 한다.
- 학습 정책(ACT 등)은 아직 없다. 지금의 예측 제어기는 비교 기준이자 시뮬레이션 시범 생성기다.
