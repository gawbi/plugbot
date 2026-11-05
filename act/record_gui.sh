#!/usr/bin/env bash
# 시범 녹화 화면: 브라우저에서 녹화 시작, 다음, 다시, 끝을 누른다(act/record_gui.py). 끝내기 Ctrl+C
set -euo pipefail
source "$(dirname "$0")/env.sh"
cd "$(dirname "$0")/.."
(sleep 2; xdg-open http://127.0.0.1:8200 >/dev/null 2>&1 || true) &
exec "$LEROBOT_BIN/python" -m uvicorn act.record_gui:app --host 127.0.0.1 --port 8200
