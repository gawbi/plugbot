# 실물 키트 문제 해결 기록 (우분투 GPU 노트북, 2026-11-02 ~)

실물 SO-101, 카메라, 그리퍼 덮개, 녹화 화면을 다루며 실제로 겪은 문제와 해결. 그림은 `docs/troubleshooting/`.
짧은 표는 `act/README.md`의 '문제 해결'에도 있다. 새 문제가 생기면 여기에 증상, 원인, 해결, 그림을 이어 쓴다.

## 로봇 연결

### 1. 원격조종이 켜지자마자 끊김 (`There is no status packet`, `No such device`) [2026-11-02]
- 원인: 리더, 팔로워 보드를 카드 리더 달린 USB 허브에 꽂았다. 팔로워에 힘이 들어가는 순간 허브가 통째로 리셋됐다(커널 기록에 허브 재연결 수십 번).
- 해결: 두 보드를 **노트북 본체 USB 포트에 하나씩 직접**. 허브는 카메라에만.

### 2. 팔로워 USB가 아예 안 잡힘 [2026-11-05]
- 증상: `/dev/serial/by-id/`에 리더 하나만 있다. `lsusb`에도 `1a86:55d3`(QinHeng USB Single Serial)이 하나뿐. 커널 기록에 새 장치 연결 흔적이 없다.
- 원인: 케이블 또는 단자. 이 USB 칩은 USB 전원만으로 잡히므로 12 V 어댑터와는 상관없다.
- 해결: 케이블을 다시 끝까지 꽂아 해결(충전 전용 케이블이면 데이터선 있는 것으로 바꾼다. 다른 본체 포트도 시험).
- 확인 명령: `ls -l /dev/serial/by-id/` (팔로워 `...5AE6083026`, 리더 `...5B79015054`)

### 3. 리더 포트는 있는데 모터 0개 응답 [2026-11-05]
- 원인: 리더 5 V 어댑터 전원이 꺼져 있었다.
- 해결: 어댑터, 멀티탭 스위치 확인. 정상이면 리더 5.2 V, 팔로워 12.3 V, 각각 모터 6개 응답.

### 4. 그리퍼(6번 모터) 과부하로 녹화가 죽음 [2026-11-05]
```
RuntimeError: Failed to write 'Torque_Enable' on id_=6 with '0' after 6 tries. [RxPacketError] Overload error!
```
- 원인: 그리퍼 덮개를 문 턱은 그리퍼 값 약 8(0~100 눈금, 0 = 완전히 닫힘)에서 칸막이에 닿는다.
  리더 손잡이를 닫으면 팔로워가 0까지 닫으려고 계속 눌러 과부하 보호가 걸렸다(LeRobot은 그리퍼 힘을 50%, 오래 누르면 25%로 제한).
- 해결: 리더 그리퍼 값을 **하한 6 아래로 내려가지 않게** 자른다(`act/record_preview.py`, `GRIP_MIN=6`).
  기록되는 행동과 보내는 명령이 같고, 칸막이는 살짝만 누른다. 로봇이 혼자 실행할 때(rollout)도 같은 하한을 쓴다.
- 과부하 중에 찍힌 에피소드 1개는 `~/.cache/huggingface/lerobot/local/plug_static_bad_overload_20261105`로 옮겨 쓰지 않는다.
- 확인: 6번 모터 Status(65) = 0, Torque_Enable(40), Present_Position(56) 원값 2148 = 그리퍼 약 8.

## 카메라

### 5. 옆면 카메라에 은색 벽 반사, 소켓이 화면 밖 [2026-11-03]
| 처음(은색 벽, 소켓 없음) | 소켓이 화면 밖 | 고정한 자리 |
|---|---|---|
| ![](troubleshooting/01_side_silver_wall.jpg) | ![](troubleshooting/02_side_socket_out.jpg) | ![](troubleshooting/03_side_fixed.jpg) |
- 해결: 30~40 cm 떨어뜨리고 위로 들어 소켓 전체가 왼쪽 위 1/3에 들어오게. 배경에 검은 천. 기준 사진 `outputs/camera_ref/side_ref_20261105.png`.

### 6. 손목 카메라에 엔드이펙터가 안 보임 [2026-11-03]
| 원통만 집었을 때(아래 끝에 보라색만 조금) | 덮개를 끼운 뒤(결합 링이 아래 1/3) |
|---|---|
| ![](troubleshooting/04_wrist_no_tool.jpg) | ![](troubleshooting/05_wrist_with_cover.png) |
- 원인: 키트의 손목 카메라 받침이 위로 젖혀져 있어 턱 끝이 화면 바로 아래에 걸린다.
- 해결: 원통을 턱 끝보다 52~58 mm 앞으로 빼는 그리퍼 덮개(`cad/make_gripper_cover.py`). ACT가 접점과 소켓을 한 화면에서 본다.

### 7. 손목 카메라가 너무 밝음, 수동 노출은 검은 화면 [2026-11-03]
| 수동 노출 60(거의 검정) | 자동 노출 |
|---|---|
| ![](troubleshooting/06_wrist_manual_exposure_black.jpg) | ![](troubleshooting/07_wrist_auto_exposure.jpg) |
- OpenCV로 `CAP_PROP_AUTO_EXPOSURE=1`, `EXPOSURE` 30~250을 줘도 평균 밝기 1~3(거의 검정). 이 카메라의 수동 노출 단위를 아직 못 맞췄다(v4l-utils 미설치).
- 지금: 자동 노출 그대로. 소켓 가운데가 하얗게 날아가면 조명을 소켓에서 비켜 둔다. 녹화와 실행 때 조명을 같게.

### 8. 녹화 시작하면 `Failed to open OpenCVCamera(... C270 ...)` [2026-11-05]
- 원인: 녹화 화면의 평소 미리보기가 카메라를 붙잡은 채로 녹화 프로그램이 같은 카메라를 열려고 했다.
  (미리보기를 스레드로 돌렸더니 끈 뒤에도 장치를 놓지 않았고, 예전 서버 프로세스가 몇 개 남아 있었다.)
- 해결: 미리보기를 따로 프로세스(`act/cam_preview.py`)로 돌리고, 녹화 전에 끝내고 `fuser`로 장치가 비었는지 확인한 뒤 녹화를 시작한다.
- 확인 명령: `fuser /dev/video4 /dev/video6` (출력이 없으면 비어 있음). 녹화 화면 서버는 하나만: `pgrep -af "uvicorn act.record_gui"`

## 그리퍼 덮개

### 9. 덮개가 마찰로만 끼워져 빠질 수 있음 → 그리퍼가 무는 구조로 [2026-11-03]
- 처음 덮개는 안쪽 사방 0.5 mm 여유. 뽑을 때 덮개가 빠질 수 있었다.
- 해결: 고정 턱 칸과 움직이는 턱 칸을 나누고 사이에 쐐기 칸막이. 그리퍼를 닫으면 칸막이를 물어 고정.

### 10. 벽이 얇다는 지적 → 보강 [2026-11-04]
| USB 파일 단면 확인(구멍 2개 = 칸막이 있음) | 칸막이 단면 |
|---|---|
| ![](troubleshooting/08_cover_section_check.png) | ![](troubleshooting/09_cover_septum.png) |
- 바깥벽 최소 2.35 → 4.4 mm, 칸막이 최소 3.1 → 4.3 mm(`WALL=4.5`, `Q_CLAMP=-0.135`). 지름 약 73 → 79 mm.
- STL을 float32로 저장할 때 넓이 0인 삼각형 조각이 생겨 '닫힌 몸체' 검사에 걸림 → manifold3d로 정리(`_simplify`).

## 소프트웨어

### 11. `Failed to find Rerun Viewer executable in PATH` [2026-11-02]
- 해결: `act/env.sh`가 `~/lerobot_env/bin`을 PATH에 넣는다. 녹화 화면에서는 Rerun을 끄고 화면 안에 카메라를 띄운다.

### 12. 보정 때 팔로워가 안 움직임(토크 걸림) [2026-11-02]
- 기존 보정 파일이 있으면 LeRobot이 먼저 '쓸지 새로 할지' 묻는데, 그동안 토크가 켜져 있다. 그 질문에 `c` + Enter(팔을 받친 채).

### 13. 녹화 프로그램이 키 입력을 못 받음(Wayland) [2026-11-05]
- LeRobot은 Wayland에서 전역 키 입력(pynput)을 못 쓰고, 표준 입력이 터미널이면 터미널에서 n(다음), r(다시), q(끝)를 받는다.
- 녹화 화면은 녹화 프로그램을 가짜 터미널(pty) 안에서 돌리고 버튼이 n, r, q를 보낸다(`act/record_gui.py`).
