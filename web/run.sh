#!/usr/bin/env bash
# PlugBot 웹 시연 서버(기본은 이 컴퓨터 안에서만: http://127.0.0.1:8000, 발표장 화면은 ?auto=1&hs=6&seed=2)
# 선상처럼 인터넷이 없을 때: 노트북 핫스팟을 켜고 HOST=0.0.0.0 bash web/run.sh → 심사위원 휴대폰이 핫스팟에 붙어
#   http://<노트북 핫스팟 주소>:8000 으로 접속(우분투 핫스팟은 보통 10.42.0.1)
# 준비(한 번): uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python mujoco numpy scipy pillow \
#              imageio imageio-ffmpeg fastapi "uvicorn[standard]"
set -euo pipefail
cd "$(dirname "$0")/.."
export MUJOCO_GL="${MUJOCO_GL:-egl}"  # 화면 없는 렌더(맥은 MUJOCO_GL=cgl 또는 비움)
exec .venv/bin/python -m uvicorn web.server:app --host "${HOST:-127.0.0.1}" --port "${PORT:-8000}"
