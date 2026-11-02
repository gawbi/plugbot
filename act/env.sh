# PlugBot 실물 ACT 공통 설정(우분투 GPU 노트북). 다른 스크립트가 source 한다. 근거: ACT_PLAN.md 3절.
# 로봇과 카메라를 꽂은 뒤 `bash act/check.sh`로 포트, 카메라를 확인하고 아래 값을 고친다(또는 같은 이름의 환경변수로 덮어쓴다).

# LeRobot 0.6.1 환경(uv 가상환경). conda를 쓰면 LEROBOT_BIN=$CONDA_PREFIX/bin
LEROBOT_BIN="${LEROBOT_BIN:-$HOME/lerobot_env/bin}"

# 허깅페이스 아이디. 'local'이면 Hub에 올리지 않고 ~/.cache/huggingface/lerobot/local/ 에만 저장한다.
# 맥에서 Hub에 올린 데이터를 받으려면 `$LEROBOT_BIN/hf auth login` 뒤 HF_USER=<아이디>
HF_USER="${HF_USER:-local}"

# 포트(lerobot-find-port로 확인). 보정 id는 맥에서 쓴 것과 같아야 한다(보정 파일 이름)
# USB 고유번호 경로(꽂는 순서가 바뀌어도 같다). 2026-11-02 서보 전압으로 확인: 팔로워 12.3 V, 리더 5.1 V
F_PORT="${F_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6083026-if00}"
L_PORT="${L_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79015054-if00}"
F_ID="${F_ID:-plug_follower}"
L_ID="${L_ID:-plug_leader}"

# 카메라. 번호(/dev/videoN)는 꽂는 순서로 바뀌므로 /dev/v4l/by-id/ 경로를 권한다. 노트북 내장 카메라(HP True Vision)는 쓰지 않는다
FRONT_CAM="${FRONT_CAM:-}"
WRIST_CAM="${WRIST_CAM:-}"
CAMS="{ front: {type: opencv, index_or_path: ${FRONT_CAM}, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: ${WRIST_CAM}, width: 640, height: 480, fps: 30}}"

TASK="Insert the plug into the socket"
RUNS="${RUNS:-$HOME/act_runs}"  # 학습 결과(체크포인트)
PUSH=$([ "$HF_USER" = local ] && echo false || echo true)

need_cams() {
  if [ -z "$FRONT_CAM" ] || [ -z "$WRIST_CAM" ]; then
    echo "FRONT_CAM, WRIST_CAM이 비어 있다. bash act/check.sh로 찾은 경로를 act/env.sh에 적거나 환경변수로 준다"; exit 1
  fi
}
need_port() {
  for p in "$@"; do
    [ -e "$p" ] || { echo "$p 없음(로봇 USB, 전원 확인. bash act/check.sh)"; exit 1; }
    [ -r "$p" ] && [ -w "$p" ] || { echo "$p 권한 없음: sudo usermod -aG dialout $USER 후 다시 로그인(또는 sudo chmod 666 $p)"; exit 1; }
  done
}
