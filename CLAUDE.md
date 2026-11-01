# PlugBot: GPU 노트북 작업 지침 (Claude Code가 먼저 읽는다)

## 상황
- 「2026 동남권 멀티도메인 모빌리티 비즈니스 해커톤」 개인부문 **본선 진출**. 본선 2026.11.8~11.10(부산~오사카 선상).
- 팀장 김광빈(이 저장소 주인). 작품: 전기차 충전구에서 배운 '꽂기'를 흔들리는 전기 선박 소켓으로 옮기는 로봇팔.
- 맥에서 MuJoCo 실험, 실물 시연 소프트웨어(`real/`), Isaac Sim 자산과 스크립트(`isaac/`)를 만들어 두었다.
  Isaac Sim 스크립트는 2026-11-01 이 노트북(RTX 5070 Laptop 8 GB, Ubuntu 26.04, Isaac Sim 5.1)에서 처음 돌렸다.
  결과와 고친 것: `outputs/ISAAC_RESULTS.md`.

## 이 노트북의 환경 (2026-11, 사용자가 알려 줌)
- HP OMEN 16, **Ubuntu 26.04**(Isaac 5.1 지원 OS 밖), GPU RTX 5070 Laptop 8 GB, 모드 `prime-select nvidia`.
- 드라이버는 **580-open**이어야 한다. 595에서는 Isaac 시작 직후 `librtx.scenedb` 세그폴트. 닫힌 모듈은 이 GPU를 못 쓴다.
- 26.04에 없는 라이브러리(libxml2.so.2, libicu74, libGLU, libOpenGL)를 `~/isaacsim_compat/lib`에 풀어 두었다.
  셸 스크립트는 스스로 `LD_LIBRARY_PATH`에 넣는다. 파이썬을 직접 부를 때는
  `LD_LIBRARY_PATH=~/isaacsim_compat/lib OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1`을 붙인다.
- Isaac Sim 5.1.0은 **pip로 `~/isaacsim_env`(Python 3.11, uv 가상환경)에 설치**했다. 설치 폴더와 `python.sh`가 없다.
  - 파이썬: `~/isaacsim_env/bin/python`. 직접 부르면 `source activate`는 필요 없다.
  - `isaac/setup_gpu.sh`는 이 경로를 스스로 찾는다. README의 `./python.sh ...` 명령은 이 파이썬으로 바꿔 실행한다.
  - 사용자가 `~/isaacsim/python.sh`(위 파이썬을 실행하는 파일)를 만들어 두었을 수도 있다. 그러면 그대로 써도 된다.
  - 화면 없이 돌릴 때는 `OMNI_KIT_ACCEPT_EULA=YES`를 붙인다(pip 설치판은 첫 실행에 사용권 동의를 묻는다).
- GUI 실행과 URDF 불러오기는 다른 노트북에서 성공했다. `ROS2 Bridge startup failed` 경고는 무시한다.
- 메모리를 아끼게 실험은 `--headless`로.

## 이 노트북에서 할 일 (순서대로. 사용자가 "시작"이라고 하면 1~6단계를 끝까지 진행한다. 단계마다 한두 줄로 알리고, 같은 오류로 세 번 막히면 멈추고 정리해서 묻는다)

배치는 본선과 같은 **side**(현측에 세운 소켓, L자 플러그, 부두에 옆으로 댄 배의 롤)가 기본이다. 예선 배치는 `--layout top`.

1. **환경 확인**: `nvidia-smi`(드라이버, GPU), Isaac Sim 5.1 파이썬 찾기.
   - 압축 설치: `~/isaacsim/python.sh` 등. 없으면 `find ~ -maxdepth 3 -name python.sh -path "*isaac*"`.
   - pip 설치: Isaac Sim이 깔린 가상환경의 python(`python -c "import isaacsim"`가 되는 것).
   - 찾은 경로를 사용자에게 한 줄로 알리고 진행한다.
2. **설정과 첫 시험**: `bash isaac/setup_gpu.sh` (파이썬을 못 찾으면 `ISAAC_PY=~/isaacsim_env/bin/python`).
   단계: 의존성 → `import_robot.py --layout side`, `--layout top`(URDF → USD) → 미리보기 사진 → 정지 소켓 1회 → 파고 6 cm 영상 1개.
   미리보기 `outputs/isaac_preview_side_*.png`를 열어 사용자에게 보여 준다(장면이 맞는지 확인).
3. **실패하면 고친다** (아래 '고칠 수 있는 것'). 한 단계씩 다시 돌린다.
4. **본 실험(렌더링 없이 `--headless`)**: 각 30회
   - `--hs 0 --strategy predictive`
   - `--hs 0.06 --strategy predictive`, `--hs 0.06 --strategy reactive`, `--hs 0.06 --strategy static`
   - 여유가 있으면 `--hs 0.04`, `--hs 0.08`도 세 전략
5. **발표 영상**: `bash isaac/make_showreel.sh`. 같은 파도(시드 2)로 추종과 예측을 찍고, 정지 소켓도 찍은 뒤 묶는다.
   결과 `outputs/showreel_side_hs6_compare.mp4`(나란히), `_views.mp4`(세 시점), `_compare.gif`(README용).
   - Isaac에서 시드 2가 '추종 실패, 예측 성공'이 아니면 `SEED=3 bash isaac/make_showreel.sh`처럼 시드를 바꿔 다시 찍는다
     (각 영상 끝 줄의 성공, 실패를 보고 판단. 최대 다섯 번).
   - GIF가 나오면 README.md의 'Isaac Sim 5.1' 절 맨 위에 `<img src="outputs/showreel_side_hs6_compare.gif" width="900">`와
     한 줄 설명(Isaac Sim 실제 렌더, 시드, 결과)을 넣고, 그 아래 MuJoCo 대체 그림 설명은 그대로 둔다.
   - mp4는 커밋하지 않는다(.gitignore). GIF가 15 MB를 넘으면 `tools/make_showreel.py`의 가로 폭을 720으로 줄여 다시 만든다.
6. **정리**: `outputs/ISAAC_RESULTS.md`에 표로 쓴다(조건, 성공률, 95% 신뢰구간(Wilson), MuJoCo 결과와 차이, 고친 것, 남은 문제).
   커밋은 아래 규칙대로 하고, `git push`는 사용자에게 물어본 뒤 한다.
7. (사용자가 원하면) **ACT 학습 데이터**: `--hs 0.06 --dataset 50 --headless` → `data/isaac_side/`.
   LeRobot 변환은 `tools/to_lerobot.py`(lerobot 0.6.1 환경, ACT_PLAN.md). data/는 커밋하지 않는다.

## 비교 기준 (MuJoCo, `outputs/experiment_side.json`, 조건당 30회)
같은 배치, 같은 파도(side, 옆으로 댄 배)의 MuJoCo 결과다. 실행 스크립트가 끝에 같은 조건의 MuJoCo 성공률을 함께 출력한다.
Isaac 스크립트는 충돌을 실패로 따로 세지 않으므로(시간 안에 꽂으면 성공) MuJoCo보다 조금 높게 나올 수 있다. 그 점을 결과표에 적는다.

| 모형 Hs | static | reactive | predictive |
|---|---|---|---|
| 0 cm | 100% | 100% | 100% |
| 4 cm | 67% | 93% | 100% |
| 6 cm | 20% | 50% | 87% |
| 8 cm | 10% | 7% | 80% |

top 배치는 `outputs/experiment_top.json`(predictive 6 cm 93%, 8 cm 83%).

## 고칠 수 있는 것 / 고치지 않을 것
- 고친다: `isaac/run_plugbot_isaac.py`, `isaac/import_robot.py`의 `[ISAAC]` 표시 줄(Isaac Sim 5.1 API 이름, 프림 경로,
  드라이브 강성 `--kp/--kd`, `PHYSICS_DT`). 5.1 API가 의심되면 설치 폴더의 확장 소스에서 확인한다
  (예: `grep -rn "class SingleArticulation" <isaac 설치폴더>/exts`). 추측으로 여러 번 바꾸지 말고 소스를 본다.
- 고치지 않는다: `plugbot/`(MuJoCo 실험, 예측, IK), `real/`(실물 시연, 맥에서 검증), `cad/`. 계산은 Isaac 스크립트가 이것을 그대로 쓴다.
- `isaac/so101/so101_plugbot*.urdf`, `isaac/scene/*.usda`는 `isaac/build_assets.py`로만 다시 만든다(맥 또는 usd-core, scipy가 있는 파이썬).

## 알아 둘 사실 (맥에서 확인함)
- 공식 SO-101 URDF(`so101_new_calib`)와 MuJoCo 모델은 기구학이 같다(그리퍼 자세 차 0.004 mm). 관절 이름과 각도를 그대로 쓴다.
- SO-101은 팔 5축 + 그리퍼. 그래서 top 배치는 선박 갑판을 부두보다 5.5 cm 낮췄고(수직 삽입), 자동차 충전구는 45°로 기울였다.
  side 배치는 핀이 그리퍼 옆으로 나온 L자 플러그로 수평 삽입한다(그리퍼는 수직, 위치와 핀 방향을 함께 푸는 IK).
- 플러그는 그리퍼에 고정(fixed joint). '놓기'는 실물에서 자석 래치가 맡고 시뮬레이션에서는 다루지 않는다.
- 선박은 키네마틱 강체(`/World/Env/World/Ship`). 매 물리 스텝 `plugbot.scene.ship_pose`(ShipMotion)로 자세를 넣는다.
- 손목 카메라는 URDF의 `wrist_cam_link` 아래에 스크립트가 만든다(MuJoCo wrist_cam과 같은 자세, 세로 화각 61°).
- 성공 = 플러그 끝이 보어 안 목표 깊이를 0.2 s 유지(MuJoCo와 같은 기준).

## 규칙
- 사용자에게는 한국어로, 짧게. 진행 중에는 단계마다 한두 줄로 알린다.
- 글에 가운데점(U+00B7)을 쓰지 않는다. "A와 B" 또는 "A, B"로 쓴다.
- 커밋 작성자는 김광빈만: 처음에 `git config user.name "김광빈"`, `git config user.email "a30115964@gmail.com"`.
  커밋 메시지에 `Co-Authored-By` 같은 Claude 표시를 넣지 않는다.
- 영상(mp4), USD 변환 결과, 학습 데이터(data/)는 `.gitignore` 대상이라 커밋하지 않는다. 결과 표(md, json), 미리보기 PNG, README용 GIF는 커밋한다.
- 이 저장소의 주제는 Sim-to-Real 갭 극복이다(README.md). Isaac 결과를 쓸 때 MuJoCo와의 차이를 '물리 엔진 갭'으로 사실대로 적는다.
- 실패나 미확인 사항을 성공처럼 쓰지 않는다.

## 그다음(사용자가 요청할 때)
- ACT 학습: `ACT_PLAN.md` 3.8~3.9절(우분투 설치와 학습). 데이터는 맥에서 실물 SO-101로 모은 뒤 가져온다.
- 실물 시연 순서: `real/README.md`.
