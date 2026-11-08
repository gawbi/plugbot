#!/usr/bin/env bash
# 녹화 화면(8200)과 부스 화면(8300)을 지금 모습 그대로 캡처해 발표 자료 현장사진 폴더에 넣는다(인터넷 불필요).
#   bash act/capture_screens.sh      → ~/Downloads/PlugBot_본선준비/현장사진/gui.jpg, booth_screen.jpg
# 로봇과 카메라를 켜고, 녹화나 흔들기를 하는 중에 찍으면 현장감이 있다. 찍은 뒤 덱을 다시 만들면 그 칸에 들어간다.
set -u
D=~/Downloads/PlugBot_본선준비/현장사진; mkdir -p "$D"
C=$(command -v google-chrome || command -v chromium)
for pair in "8200 gui 1400,1300" "8300 booth_screen 1920,1080"; do
  set -- $pair
  curl -s -o /dev/null -w "%{http_code}" "127.0.0.1:$1/" | grep -q 200 || { echo "$1 화면이 꺼져 있음"; continue; }
  "$C" --headless=new --disable-gpu --window-size="$3" --virtual-time-budget=3000 --screenshot="$D/$2.png" "http://127.0.0.1:$1" >/dev/null 2>&1
  ~/deck_py/bin/python -c "from PIL import Image; Image.open('$D/$2.png').convert('RGB').save('$D/$2.jpg', quality=88)" && rm -f "$D/$2.png" && echo "저장: $D/$2.jpg"
done
