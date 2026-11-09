"""앞(옆면) 카메라 다시 맞추기: 실시간 화면을 기준 화면(잘 되던 때)과 겹쳐 보여 주고, 몇 픽셀 어긋났는지 글자로 띄운다.

팔은 시작(접은) 자세로 둔다(기준 화면도 실행 시작 장면). 녹화나 실행 중에는 카메라를 같이 못 쓰니 멈춘 뒤 실행한다.
실행: ~/lerobot_env/bin/python tools/align_front_cam.py [--ref data/rollouts/20261110_023309/front/00005.jpg]
끝내기: q 또는 Esc
"""

import argparse
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--ref", default="/home/cho/plugbot/data/rollouts/20261110_023309/front/00005.jpg")
ap.add_argument("--cam", default="/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0")
a = ap.parse_args()

ref = cv2.imread(a.ref)
cap = cv2.VideoCapture(int(os.path.realpath(a.cam).replace("/dev/video", "")), cv2.CAP_V4L2)  # by-id 이름은 V4L2가 못 받는다
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
rg = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32)
win = cv2.createHanningWindow((640, 480), cv2.CV_32F)
while True:
    ok, fr = cap.read()
    if not ok:
        print("카메라를 못 읽음(실행 중인지 확인)")
        break
    g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    (dx, dy), resp = cv2.phaseCorrelate(rg, g, win)
    blend = cv2.addWeighted(ref, 0.5, fr, 0.5, 0)
    for x in range(0, 640, 80):
        cv2.line(blend, (x, 0), (x, 480), (255, 255, 0), 1)
    for y in range(0, 480, 60):
        cv2.line(blend, (0, y), (640, y), (255, 255, 0), 1)
    good = abs(dx) < 5 and abs(dy) < 5
    col = (0, 200, 0) if good else (0, 0, 255)
    cv2.putText(blend, f"dx {dx:+.0f}px  dy {dy:+.0f}px", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)
    hint = "OK" if good else ("tilt camera DOWN" if dy > 0 else "tilt camera UP") if abs(dy) >= abs(dx) else ("turn camera RIGHT" if dx > 0 else "turn camera LEFT")
    cv2.putText(blend, hint, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)
    view = np.hstack([fr, blend])
    cv2.imshow("front camera align (left: now, right: overlay with reference)  q=quit", view)
    if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
        break
cap.release()
cv2.destroyAllWindows()
