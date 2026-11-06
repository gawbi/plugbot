# 다른 노트북: Isaac Sim 이종 원격조종(SO-101 리더 팔 → UR10e) 설정 프롬프트

> 이 파일 전체를 다른 노트북의 Claude Code에 붙여 넣는다. 빈 폴더에서 시작해도 된다.
> 작성 2026-11-06, 팀장 김광빈. 모델은 Opus, 노력 수준 high 이상을 권한다. 이 노트북: Isaac Sim 6.2(설치돼 있음), 램 16 GB, 스왑 있음.

---

## 너(Claude)에게

너는 팀 Land2Sea(PlugBot)의 이 노트북에서 **SO-101 리더 팔을 손으로 움직이면 Isaac Sim 안의 실제 크기 UR10e가 실시간으로 따라 움직여
전기 페리(팬스타 무늬) 현측 소켓에 플러그를 꽂는 장면**을 띄운다. 본선 질의응답 때 심사위원이 리더 팔로 직접 해 보는 체험용이다.
코드는 이미 저장소에 있다. 너의 일은 **이 노트북에 환경을 만들고, 돌려 보고, 막히면 고치는 것**이다.

**사용자에게 하나하나 묻지 말고 끝까지 알아서 진행한다.** 설치, 수정, 시험, 실패하면 원인 찾아 고치기, 대체안 선택까지 네가 판단한다.
진행 상황은 단계마다 한두 줄로 알리기만 한다. 같은 오류로 다섯 번 막히거나 사용자 손이 꼭 필요할 때(아래 '사용자에게 넘길 것')만 멈춘다.

### 지킬 규칙 (팀 공통)
- 사용자에게는 한국어로, 짧게.
- 글에 가운데점(U+00B7)을 쓰지 않는다. "A와 B" 또는 "A, B".
- 커밋 작성자는 김광빈만: `git config user.name "김광빈"`, `git config user.email "a30115964@gmail.com"`. 커밋 메시지에 Claude나 Co-Authored-By 표시를 넣지 않는다.
- mp4, USD 변환 결과, data/는 커밋하지 않는다. **git push는 사용자에게 물어본 뒤** 한다. 브랜치는 `teleop-laptop2`를 새로 만들어 쓴다(main에 직접 커밋하지 않는다).
- 결과를 부풀리지 않는다. 안 된 것은 안 됐다고 쓴다.
- sudo가 필요한 일(dialout 그룹, 드라이버, 패키지 설치)은 비밀번호가 필요하면 실행할 명령을 그대로 보여 주고 사용자가 치게 한다. `chmod 777`로 권한을 우회하지 않는다.
- 사용자에게 넘길 것: sudo 비밀번호, GitHub 로그인(비공개 저장소), 리더 팔 전원과 USB 꽂기, git push 여부. 이것 말고는 묻지 않는다.
- 선상 인터넷은 하루 2.5 GB다. **큰 다운로드(Isaac Sim 약 10 GB 이상, 자산)는 반드시 육지에서(11/7까지)** 끝낸다.

## 1. 저장소 받기

```bash
mkdir -p ~/work && cd ~/work
git clone https://github.com/gawbi/plugbot.git      # 비공개 저장소: gh auth login 또는 사용자에게 권한 확인
cd plugbot && git checkout -b teleop-laptop2
```
먼저 읽을 것: `CLAUDE.md`(GPU 노트북 지침, Isaac 5.1 pip 설치와 호환 라이브러리 방법), `isaac/scaleup/README.md`,
그리고 이 작업의 핵심 파일 세 개:
- `isaac/scaleup/run_scaleup.py`: 실제 크기 장면(UR10e, 페리, 소켓). `--live-leader`, `--live-seconds`, `--live-cam`, `--lowmem` 옵션이 이번에 추가됐다
- `isaac/scaleup/leader_source.py`: 리더 팔 관절 → 그리퍼 끝의 소켓 기준 위치(MuJoCo 기구학). `fake:<npz>`로 팔 없이 시험 가능
- `isaac/scaleup/teleop_socket_ref.json`, `isaac/scaleup/calib/plug_leader.json`, `plug_follower.json`: 소켓 기준과 우리 팔 보정값(GPU 노트북에서 만듦)

## 2. 환경 확인 (먼저, 결과를 사용자에게 표로)

```bash
uname -a; lsb_release -a 2>/dev/null; nvidia-smi; free -h; swapon --show; df -h ~; python3 --version
```
- **Isaac Sim은 이 노트북에 6.2가 이미 설치돼 있다(사용자 확인).** 새로 설치하지 말고, 어디에 어떻게 깔렸는지(pip 가상환경인지, 설치 폴더와 python.sh인지)부터 찾아 그 파이썬을 쓴다.
  `find ~ -maxdepth 4 \( -name python.sh -o -name "isaacsim*" \) 2>/dev/null | head`, 그 파이썬에서 `python -c "import isaacsim, importlib.metadata as m; print(m.version('isaacsim'))"`로 버전을 확인한다.
- **GPU**: NVIDIA RTX 계열이어야 Isaac Sim이 돈다. 드라이버 버전 확인(우분투 26.04면 580-open, CLAUDE.md 참고). GPU가 없거나 RTX가 아니면 7절 대체안으로 간다.
- **램 16 GB**: Isaac Sim 5.1 권장은 32 GB다. 스왑 파일은 사용자가 이미 만들어 두었다(`swapon --show`로 확인만 한다). 3절의 메모리 대책을 차례로 쓴다.
- 디스크 여유 50 GB 이상(Isaac pip 설치와 캐시).
- Windows 노트북이면 사용자에게 알리고, Isaac Sim 5.1 Windows pip 설치로 같은 순서를 따르되 경로와 포트 이름(COM3 등)을 바꾼다.

## 3. 램 16 GB 대책 (순서대로, 될 때까지만)

1. **다른 프로그램 끄기**: 크롬, VS Code의 큰 확장, 다른 Claude 창. `free -h`로 Isaac 실행 전 '사용 가능'이 10 GB 이상인지 본다.
2. **가벼운 실행**: `run_scaleup.py --lowmem`(창 960x540, 안티에일리어싱 끔, RaytracedLighting). `--fps 20`으로 낮춘다.
3. **자산을 로컬로**: `isaac/scaleup/fetch_assets.py`로 UR10e와 하늘 HDR을 받아 두면 실행 때 자산 서버를 오가지 않는다(선상 인터넷도 아낀다).
4. 그래도 메모리 부족으로 죽으면(`Killed`, `std::bad_alloc`, 창이 멈춤) 7절 대체안.
- 실행 중 `watch -n 2 free -h`를 옆 터미널에 띄워 두고, 최대 사용량을 사용자에게 알려 준다.

## 4. Isaac Sim 6.2에 맞추기 (설치는 하지 않는다)

- 저장소 코드는 **Isaac Sim 5.1**에서 만들고 돌렸다. 6.2에서는 확장 이름이나 API가 바뀌었을 수 있다. `[ISAAC]` 표시가 붙은 줄이 Isaac API를 쓰는 곳이다.
  - 특히 확인할 것: `isaacsim.core.api.World`, `isaacsim.core.prims.SingleArticulation`, `SingleXFormPrim`, `isaacsim.robot_motion.motion_generation`(Lula, `interface_config_loader`의 `"UR10e"` 설정 이름),
    `isaacsim.storage.native.get_assets_root_path`, `isaacsim.sensors.camera.Camera`, `omni.kit.viewport.utility.get_active_viewport`, 자산 경로(`/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd` 등 5.1 경로)
  - 오류가 나면 추측으로 바꾸지 말고, 6.2 설치 폴더의 확장 소스를 grep해서 새 이름을 찾는다(예: `grep -rn "class SingleArticulation\|def compute_inverse_kinematics" <isaac 설치 폴더>/exts | head`).
  - 고친 줄에는 `# [ISAAC 6.2]` 주석을 붙여, 5.1 노트북과 다른 곳을 나중에 알 수 있게 한다. 5.1에서도 돌아가게 둘 다 지원할 수 있으면 `try/except ImportError`로 둔다.
- 추가 패키지(그 Isaac 파이썬에): `pip install feetech-servo-sdk mujoco scipy`(uv 환경이면 `uv pip install --python <그 파이썬>`).
- 첫 실행은 `OMNI_KIT_ACCEPT_EULA=YES`를 붙인다. 우분투 26.04처럼 지원 밖 OS에서 라이브러리 오류가 나면 CLAUDE.md의 호환 라이브러리 방법을 참고한다.
- 아래 명령의 `~/isaacsim_env/bin/python`은 이 노트북에서 찾은 Isaac 6.2 파이썬(또는 `python.sh`)으로 바꿔 쓴다.

## 5. 팔 없이 먼저 시험 (fake 입력)

```bash
cd ~/work/plugbot
OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 ~/isaacsim_env/bin/python isaac/scaleup/run_scaleup.py \
  --live-leader fake:isaac/scaleup/sample_traj_ep54.npz --live-seconds 20 --lowmem --live-cam /World/CamClose
```
- 성공 기준: GUI 창이 뜨고, UR10e가 실물 시범(54번)과 같은 타이밍으로 소켓에 다가가 꽂히며, 터미널에 `[live] 연결 N s`가 찍힌다.
- 이 실시간 모드는 **GPU 노트북(Isaac 5.1)에서도 아직 돌려 보지 않았고, 6.2에서는 처음이다**(그 노트북 GPU가 학습 중이었다). 같은 계산을 쓰는 녹화 재생(`--replay`)은 GPU 노트북에서 성공했다(9.9 s에 연결).
  그러니 Isaac API 이름이나 시점 설정에서 오류가 나면 `[ISAAC]` 표시 줄부터 확인해서 고친다. 추측으로 여러 번 바꾸지 말고 설치 폴더의 확장 소스를 grep해서 본다.
- `episode_live()`는 `episode_replay()`와 같은 대응식(grow, socket_basis, TAU_CAPTURED, 관절 속도와 가속 제한)을 쓴다. 두 함수를 비교하며 고친다.

## 6. 리더 팔 연결 (실물)

1. 리더 팔 5 V 전원과 USB를 꽂는다. `ls /dev/serial/by-id/`, `ls /dev/ttyACM* /dev/ttyUSB*`로 포트를 찾는다.
2. 권한: `groups`에 dialout이 없으면 `sudo usermod -aG dialout $USER`를 사용자에게 치게 하고, 다시 로그인이 필요하다고 알린다.
   CH340 계열이 안 보이면 `dmesg | tail`로 brltty 점유 여부를 확인하고, 점유 중이면 `sudo systemctl stop brltty`(또는 제거) 명령을 사용자에게 보여 준다.
3. Isaac 없이 먼저: `~/isaacsim_env/bin/python isaac/scaleup/leader_source.py /dev/ttyACM0`
   - 리더 팔을 접은 홈 자세에서 '소켓까지' 약 250~300 mm, 플러그를 꽂는 자세(팔을 앞으로 뻗어 손목을 내림)로 가면 깊이가 + 쪽으로 커지면 정상.
   - 리더 팔은 GPU 노트북에서 보정한 그 팔이어야 한다(`calib/plug_leader.json`). 다른 리더 팔이면 LeRobot으로 다시 보정하고 그 JSON으로 바꾼다.
4. 실시간: `run_scaleup.py --live-leader /dev/ttyACM0 --lowmem --live-cam /World/CamWide`(Ctrl+C로 끝).
   - 너무 느리면(UR10e가 한참 늦게 따라옴) `--fps 20`, 창이 버벅이면 `--lowmem` 확인.
   - 체험용 팁: 심사위원은 시점이 넓은 `/World/CamWide`, 꽂는 순간 확대는 `/World/CamClose`.

## 7. 대체안: Isaac이 이 노트북에서 안 될 때 (GPU 없음, 메모리 부족)

MuJoCo 뷰어로 같은 체험을 만든다(수백 MB). `leader_source.py`의 `LeaderSource`를 그대로 쓰고,
UR10e 모델은 MuJoCo Menagerie(`universal_robots_ur10e`)를 받아 소켓 판 하나를 더한 장면에서 IK(감쇠 최소제곱)로 따라가게 한다.
새 파일 `isaac/scaleup/live_mujoco_ur10e.py`로 만들고, 화면에 "MuJoCo 대체 장면"이라고 표시한다. 팬스타 무늬는 없어도 된다.

## 8. 끝나면

- 사용자에게: 된 것, 안 된 것, 최대 메모리 사용량, 체험 순서(명령 한 줄)를 짧게 정리.
- `isaac/scaleup/README.md`에 '실시간 이종 원격조종' 절을 추가(명령, 메모리 대책, 확인한 것과 안 한 것). 브랜치 `teleop-laptop2`에 커밋.
- push는 사용자에게 묻는다(팀 규칙). 그 밖의 결정은 네가 한다.

## 참고: 이 장면이 무엇인지 (발표에서 정확히 말하기)
- 리더 팔 실시간 입력 → 그리퍼 끝의 소켓 기준 위치(모형 크기) → 커넥터 5배로 키워 UR10e 목표 → IK. 소켓 위치는 시뮬레이션 참값, 충돌 계산 없는 운동학 시각화.
- 배는 파도(모형 6 cm를 프루드 상사로 키운 것)로 흔들리고, 사람은 소켓 기준으로만 움직인다. 배 운동 보정은 시스템이 한다는 구성이다.
- "학습한 정책이 UR10e를 움직인다"가 아니다. 사람이 작은 팔로 큰 팔을 움직이는 **이종 원격조종**이다.
