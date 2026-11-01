#!/usr/bin/env bash
# PlugBot 웹 시연 서버(이 컴퓨터 안에서만 열림: http://127.0.0.1:8000, 발표장 화면은 ?auto=1&hs=6&seed=2)
# 준비(한 번): uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python mujoco numpy scipy pillow \
#              imageio imageio-ffmpeg fastapi "uvicorn[standard]"
set -euo pipefail
cd "$(dirname "$0")/.."
export MUJOCO_GL="${MUJOCO_GL:-egl}"  # 화면 없는 렌더(맥은 MUJOCO_GL=cgl 또는 비움)
exec .venv/bin/python -m uvicorn web.server:app --host 127.0.0.1 --port "${PORT:-8000}"
