# Isaac Sim 5.1 실험 결과 (side 배치, 2026-11-01)

MuJoCo에서 만든 꽂기 실험(`plugbot.control.run_trial`)을 Isaac Sim 5.1(PhysX)에서 같은 배치, 같은 파도 시드(0~29),
같은 제어 코드(소켓 관측 지연 0.1 s, AR 예측, 정온 구간, IK)로 다시 돌렸다. 조건당 30회, `--headless`, 기본 드라이브 kp 1000, kd 30.

- 실행 환경: HP OMEN 16, RTX 5070 Laptop GPU 8 GB, Ubuntu 26.04.1, NVIDIA 드라이버 580.178(open), Isaac Sim 5.1.0(pip)
- 결과 파일: `outputs/isaac_side_hs{0,4,6,8}_{static,reactive,predictive}.json`(시행별 성공, 이유, 삽입 시각, 첫 충돌)
- MuJoCo 기준: `outputs/experiment_side.json`(같은 배치, 같은 시드, 조건당 30회)

## 성공률

괄호는 95% 신뢰구간(Wilson). 'Isaac'은 이 스크립트의 원래 판정(제한 시간 안에 핀 끝이 목표 깊이를 0.2 s 유지하면 성공)이다.
MuJoCo는 꽂기 전에 플러그가 소켓 밖 선체에 닿거나 그리퍼, 팔이 소켓이나 선체에 닿으면 그 자리에서 실패로 끝낸다.
같은 기준으로 비교하려고 Isaac에서도 접촉을 기록해 '충돌을 실패로 센 성공률'을 따로 계산했다(아래 '충돌 판정').
차이는 '충돌을 실패로 센 성공률'에서 MuJoCo를 뺀 값이다.

| 모형 Hs | 전략 | Isaac | Isaac, 충돌을 실패로 셈 | MuJoCo | 차이(%p) |
|---|---|---|---|---|---|
| 0 cm | predictive | 30/30, 100% (89~100) | 100% (89~100) | 100% | 0 |
| 4 cm | static | 23/30, 77% (59~88) | 77% (59~88) | 67% | +10 |
| 4 cm | reactive | 30/30, 100% (89~100) | 100% (89~100) | 93% | +7 |
| 4 cm | predictive | 30/30, 100% (89~100) | 100% (89~100) | 100% | 0 |
| 6 cm | static | 20/30, 67% (49~81) | 63% (46~78) | 20% | +43 |
| 6 cm | reactive | 30/30, 100% (89~100) | 100% (89~100) | 50% | +50 |
| 6 cm | predictive | 29/30, 97% (83~99) | 97% (83~99) | 87% | +10 |
| 8 cm | static | 14/30, 47% (30~64) | 43% (27~61) | 10% | +33 |
| 8 cm | reactive | 30/30, 100% (89~100) | 100% (89~100) | 7% | +93 |
| 8 cm | predictive | 26/30, 87% (70~95) | 87% (70~95) | 80% | +7 |

삽입 시각 중앙값(Hs 6 cm, 성공한 시행): Isaac static 2.73 s, reactive 2.24 s, predictive 2.12 s / MuJoCo 2.91 s, 2.80 s, 2.40 s.

### 읽는 법

- **MuJoCo와 같은 것**: 정지 소켓은 두 엔진 모두 100%. 정지(static) 전략은 파고가 클수록 떨어지고 세 전략 중 가장 낮다.
  예측(predictive)은 6 cm 97%, 8 cm 87%로 MuJoCo(87%, 80%)와 신뢰구간이 겹친다.
- **MuJoCo와 다른 것(물리 엔진 갭)**: 추종(reactive)이 Isaac에서는 모든 파고에서 30/30이고 선체 충돌이 한 번도 없다.
  MuJoCo에서는 6 cm 50%, 8 cm 7%이고 실패는 거의 전부 '플러그가 소켓 밖 선체에 부딪힘'이다.
  그래서 MuJoCo의 핵심 결론인 '예측이 추종보다 낫다'는 **Isaac에서는 재현되지 않았다**(6 cm 97% 대 100%).
  정지 전략도 Isaac이 33~43%p 높다.
- 숫자를 맞추려고 파라미터를 바꾸지 않았다. 아래 원인 확인 시험은 본 결과와 따로 적는다.

## 원인 확인 시험 (Hs 6 cm, 본 결과와 별도)

추종 전략의 차이가 서보 모델 때문인지 보려고, Isaac 관절 드라이브를 MuJoCo SO-101 모델(`third_party/robotstudio_so101/so101.xml`)의
STS3215 값으로 맞춰 다시 돌렸다. 결과 파일 `*_mjservo.json`, `*_mjservo_arm.json`.

| 드라이브 | static | reactive | predictive |
|---|---|---|---|
| 기본(kp 1000, kd 30, 최대 10 N m) | 63% | 100% | 97% |
| MuJoCo 서보 값(kp 998, kd 3.33 = kv 2.73 + 관절 감쇠 0.6, 최대 2.94 N m) | 57% | 100% | 90% |
| 위 + armature 0.028 kg m² | (안 함) | 100% | (안 함) |
| MuJoCo | 20% | 50% | 87% |

(충돌을 실패로 센 성공률. 30회씩.) 서보 토크 한계, 감쇠, armature를 MuJoCo와 같게 해도 추종은 30/30이었다.
**서보 모델은 추종 전략 차이의 원인이 아니다.** 원인은 확인하지 못했다(아래 '남은 문제').

## 발표 영상

`SEED=2 bash isaac/make_showreel.sh`로 찍었다. 영상은 팔을 세운 홈 자세에서 출발해 2.5초 동안 소켓 앞으로 다가간 뒤 꽂는다(`--reach 2.5`,
성공률 실험은 MuJoCo와 같게 소켓 앞에서 시작). 시드 2에서 추종 4.84 s, 예측 7.44 s, 정지 소켓 4.24 s에 모두 성공(충돌 없음).
결과 `outputs/showreel_side_hs6_compare.gif`(9.2 MB, README에 넣음), `_compare.mp4`, `_views.mp4`(mp4는 커밋하지 않음).
'추종 실패, 예측 성공' 장면은 얻지 못했다. 6 cm 추종이 시드 0~29 모두 성공했으므로 시드를 바꿔 다시 찍지 않았다.
(6 cm에서 정지 전략은 시드 9, 11, 12, 14, 17, 18, 23, 25, 27, 29에서 실패, 예측은 시드 7에서만 실패.
'정지 실패, 예측 성공' 비교라면 이 시드로 찍을 수 있다.)

## 충돌 판정

Isaac의 PhysX 접촉 API(`RigidPrim` 접촉 뷰)는 이 환경에서 쓰지 못했다. 센서 경로를 여러 개 주면 뷰가 만들어지지 않고,
접촉 보고를 켜면(`prepare_contact_sensors` 또는 `PhysxContactReportAPI`) `isaacsim.sensors.physics` 플러그인에서 세그폴트가 났다.
대신 제어 주기(50 Hz)마다 Isaac의 관절각과 선박 자세를 MuJoCo 장면에 넣고 MuJoCo 실험과 같은 함수
(`plugbot.control.contacts`)로 판정했다. 물리 진행은 Isaac이 하고 판정 규칙과 충돌 형상은 MuJoCo와 같다.
Isaac 판정은 충돌 뒤에도 시행을 계속하므로, 'Isaac' 열에는 부딪힌 뒤 다시 꽂은 경우가 성공으로 들어 있다(6 cm static 1회, 8 cm static 1회).

## 고친 것 (`isaac/run_plugbot_isaac.py`의 [ISAAC] 부분, 셸 스크립트)

| 증상 | 원인 | 고침 |
|---|---|---|
| Isaac 시작 때 `libxml2.so.2`, `libOpenGL.so.0` 없음, URDF 가져오기 실패 | Ubuntu 26.04(Isaac 5.1 지원 OS 밖)에 없는 라이브러리 | Ubuntu 24.04의 libxml2, libicu74, libGLU와 26.04 libopengl0를 `~/isaacsim_compat/lib`에 풀고 `LD_LIBRARY_PATH`에 넣음(`setup_gpu.sh`, `make_showreel.sh`). 시스템은 건드리지 않음 |
| 앱 시작 직후 `librtx.scenedb` 세그폴트 | 드라이버 595 | 드라이버를 580-open으로 내림(사용자가 설치) |
| pip 설치판이 사용권 동의에서 멈춤 | `setup_gpu.sh`가 동의 환경변수보다 먼저 `import isaacsim` 확인 | 순서를 바꿈 |
| Webcam, ShowCam 사진이 엉뚱한 곳을 찍음, 선박이 안 움직일 수 있음 | 참조한 장면의 기본 프림 `World`는 `/World/Env` 바로 아래로 붙는데 스크립트는 `/World/Env/World/...`를 씀 | 경로를 `/World/Env/...`로 |
| 정지 소켓도 시간 초과, 팔이 0.27 rad 처짐 | (1) inertial 없는 좌표 링크 `plug_tip`, `wrist_cam_link`에 PhysX 기본 질량 1 kg씩 (2) URDF 가져오기 기본 드라이브가 acceleration 방식(강성 × 관성)이라 가벼운 팔에서 강성이 약 3 N m/rad | 두 링크 질량을 1e-6 kg으로, 드라이브를 force 방식으로(MuJoCo position 액추에이터와 같음). 오차 0.27 → 0.0005 rad, 정지 소켓 0/1 → 30/30 |
| 영상 1회 실행이 30회 결과 json을 덮어씀 | 같은 파일 이름 | `--tag` 옵션, `make_showreel.sh`는 `--tag video` |
| `print` 출력이 로그에 안 남음 | 앱 종료 때 버퍼가 버려짐 | 실행할 때 `PYTHONUNBUFFERED=1` |

추가 옵션: `--max-effort`, `--armature`(원인 확인용, 기본값은 이전과 같음), `--tag`, `--reach`(영상, 학습 데이터에서 홈 자세부터 출발).
학습 데이터와 영상의 첫 장에 이전 자세가 흐리게 찍히던 것은 시행 시작 전에 몇 장 그려 두는 것으로 고쳤다.

## 남은 문제

- **추종 전략이 Isaac에서 MuJoCo보다 훨씬 잘 되는 원인을 모른다.** 파도 시드, 제어 코드, 판정 규칙, 서보 값은 같게 했다.
  남은 후보는 접촉 처리(PhysX의 볼록 껍질 소켓 벽 17개와 접촉 오프셋, MuJoCo의 접촉 모델)다.
  확인 방법: 같은 시드에서 두 엔진의 플러그 끝 궤적을 시간별로 겹쳐 보고, MuJoCo에서 처음 선체에 닿는 순간 Isaac 궤적이 어디 있는지 본다.
  원인이 밝혀지기 전에는 '예측이 추종보다 낫다'는 주장의 근거로 Isaac 결과를 쓰지 않는다.
- PhysX 접촉 API를 쓰지 못해 Isaac 쪽 접촉력(MuJoCo의 '삽입 접촉력 p95')은 재지 못했다.
- Ubuntu 26.04와 드라이버 580은 Isaac Sim 5.1이 공식 시험한 조합이 아니다(라이브러리를 24.04 것으로 보충함).
- 장면이 있는 실행에서 앱 종료 때 가끔 세그폴트가 난다(결과 저장 뒤라 결과에는 영향 없음).
- Webcam 미리보기는 1280 × 720 화면비에 맞추느라 세로 화각이 설계(4:3, 60°)보다 좁다. 학습 데이터용 카메라(640 × 480)는 영향 없음.
- top 배치, `--dataset`(ACT 학습 데이터)은 이번에 돌리지 않았다.
