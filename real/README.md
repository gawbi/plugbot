# 실물 시연 진행 순서 (SO-101, 맥에서 실행, GPU 불필요)

웹캠 영상 → 마커로 소켓 자세 → 카메라→로봇 보정 → AR 예측·정온 구간 → IK → SO-101.
MuJoCo 실험과 같은 전략과 상태 기계를 쓴다(`plugbot/control.py`).

## 0. 실물 없이 검증한 결과 (`python -m real.dry_run`, 2026-10-31)

MuJoCo로 웹캠 영상을 렌더링해 아래 코드를 그대로 돌렸다(조건당 10회, 정답 위치로 채점).

| 조건 | 추종 | 예측(정온 구간) |
|---|---|---|
| 정지 소켓 | 100% | 100% |
| 모형 파고 4 cm | 50% | 100% |
| 모형 파고 6 cm | 30% | 100% |

- 보정(갑판 6곳, 핀 끝 1 mm 잡음): 카메라 위치 오차 4.0 mm, 회전 0.5°.
- 소켓 위치 추정: 마커 하나(25 mm)는 0.3~0.5 m 거리에서 오차가 중앙값 5 mm, 최대 18 mm였다(합성 영상).
  그래서 소켓 둘레에 마커 4개를 더 붙이는 갑판 시트(04)를 만들었고, 오차가 중앙값 1.3 mm로 줄었다.

## 1. 살 것 (철물점·다이소·온라인, 2~3만 원)

| 품목 | 수량 | 용도 |
|---|---|---|
| PLA 필라멘트 | 1 kg 1롤(약 340 g 사용) | 출력 |
| M5 전산볼트 160 mm 이상 + 너트 | 1 + 2 | 갑판 힌지 축 |
| M4 × 12 볼트·너트 | 8쌍 | 소켓 2개 고정 |
| 네오디뮴 자석 Ø6 × 3 mm | 2개(여분 포함 4개) | 래치(핀 끝, 보어 바닥) |
| USB 웹캠(720p 이상) | 1~2대 | 소켓 추적(front). ACT를 하면 손목용 1대 더 |
| 순간접착제, 양면테이프 | | 자석, 마커 시트 |

로봇 받침(높이 100 mm)은 나무 블록이나 책 묶음으로 대신한다. 출력판은 220 mm 이상이어야 한다(갑판 베이스).

## 2. 출력과 조립

- `cad/PRINT_GUIDE.md` 그대로. 소켓은 2개.
- 인쇄(실제 크기 100%, 눈금자로 확인): `real/print/`
  - `01_소켓마커.pdf`: id 0. 소켓 탭 홈에, 마커 오른쪽이 탭 바깥쪽을 향하게.
  - `04_갑판마커.pdf`: 갑판판에 깔고 볼트 구멍을 뚫어 소켓을 그 위에 고정. 힌지(로봇) 쪽이 아래.
  - `03_카메라보정판.pdf`: 평평한 판에 붙인다.
  - `02_보정시트.pdf`: 예비용.

## 3. 소프트웨어 (맥)

```bash
# LeRobot 0.6.1 환경(ACT_PLAN.md 3.1). 그다음 이 저장소 의존성
pip install mujoco opencv-contrib-python numpy scipy
cd plugbot_demo
python -m real.robots write-default-map          # config/joint_map.json 생성
```

## 4. 순서 (처음 한 번)

1. 웹캠 보정: `python -m real.vision calib --cam 0` → 보정판을 여러 각도로 15장. RMS 1 px 안팎이면 된다.
2. 추적 확인: `python -m real.vision show --cam 0` → 갑판을 움직이면 숫자가 따라 바뀌는지.
3. **관절 매핑 확인(필수)**: `python -m real.robots check --port /dev/tty.usbmodemXXXX`
   토크가 꺼진 팔을 손으로 움직여 화면 속 모델이 같은 자세인지 본다. 다른 관절은 `config/joint_map.json`의
   `sign`(-1)이나 `offset`(rad)을 고친다. 공식 URDF(so101_new_calib)와 LeRobot 보정 기준이 같아 대부분 그대로일 것이다.
4. 카메라→로봇 보정: `python -m real.calib --port ... --cam 0`
   토크를 끈 팔로 플러그를 깔때기에 끝까지 꽂고(자석이 붙게) Enter. 갑판 위치·기울기를 바꿔 5~6번. RMS 4 mm 이하.
5. 정지 소켓 10회: `python -m real.insert --port ... --cam 0 --trials 10 --strategy predictive`
6. 흔들림: 레버를 손이나 메트로놈에 맞춰 천천히 흔들며 같은 명령. `--strategy reactive`로도 돌려 비교한다.

## 안전

- 첫 시험은 `config/joint_map.json`의 `max_step_deg`를 3으로 낮춰 천천히.
- 마커를 0.5초 넘게 못 보면 그 자리에 멈추고, 3초가 넘으면 시행을 끝낸다. Ctrl+C로 언제든 중단.
- `--cam-delay`(기본 0.05 s)는 웹캠 지연. 흔들림에서 늘 늦게 꽂히면 0.08~0.12로 올려 본다.
