"""앞(옆면) 카메라 다시 맞추기: 실시간 화면을 기준 화면(잘 되던 때)과 겹쳐 보여 주고, 몇 픽셀 어긋났는지 글자로 띄운다.

팔은 시작(접은) 자세로 둔다(기준 화면도 실행 시작 장면). 녹화나 실행 중에는 카메라를 같이 못 쓰니 멈춘 뒤 실행한다.
실행: ~/lerobot_env/bin/python tools/align_front_cam.py [--ref data/rollouts/20261110_023309/front/00005.jpg]
끝내기: touch /dev/shm/plugbot_align.stop (브라우저 탭은 그냥 닫는다)
"""

import argparse
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--ref", default="/home/cho/plugbot/data/rollouts/20261110_023309/front/00005.jpg")
ap.add_argument("--cam", default="/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0")
ap.add_argument("--bottom", type=float, default=0.0, help="아래쪽 이 비율만 비교(손목 카메라: 덮개 링만 보게 0.4)")
a = ap.parse_args()

ref = cv2.imread(a.ref)
cap = cv2.VideoCapture(int(os.path.realpath(a.cam).replace("/dev/video", "")), cv2.CAP_V4L2)  # by-id 이름은 V4L2가 못 받는다
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
def ring_top(img):
    """보라색 덮개 링의 가장 높은 점(x, y). 손목 카메라와 덮개는 같은 그리퍼에 붙어 있어 늘 같은 자리에 보여야 한다"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 0] > 115) & (hsv[..., 0] < 165) & (hsv[..., 1] > 60) & (hsv[..., 2] > 40)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    m[:200] = 0
    tops = {x: int(np.nonzero(m[:, x])[0].min()) for x in range(0, 640, 4) if m[:, x].any()}
    if not tops:
        return None
    x = min(tops, key=tops.get)
    return x, tops[x]


y0 = int(480 * (1 - a.bottom)) if a.bottom > 0 else 0  # 손목 카메라는 배경이 자세마다 달라 덮개 링만 비교
rg = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:]
win = cv2.createHanningWindow((640, 480 - y0), cv2.CV_32F)
OUT, STOP, PAGE = "/dev/shm/plugbot_align.jpg", "/dev/shm/plugbot_align.stop", "/dev/shm/plugbot_align.html"
if os.path.exists(STOP):
    os.remove(STOP)
with open(PAGE, "w") as f:
    f.write('<html><body style="margin:0;background:#111"><img id=i style="width:100%"><script>'
            'setInterval(()=>{i.src="plugbot_align.jpg?"+Date.now()},150)</script></body></html>')
os.system(f"xdg-open {PAGE} >/dev/null 2>&1 &")
while True:
    ok, fr = cap.read()
    if not ok:
        print("카메라를 못 읽음(실행 중인지 확인)")
        break
    g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:]
    (dx, dy), resp = cv2.phaseCorrelate(rg, g, win)
    if a.bottom > 0:  # 손목: 덮개 링 꼭대기 위치로 잰다(배경은 자세마다 달라 믿을 수 없다)
        rt, nt = ring_top(ref), ring_top(fr)
        if rt and nt:
            dx, dy = nt[0] - rt[0], nt[1] - rt[1]
    blend = cv2.addWeighted(ref, 0.5, fr, 0.5, 0)
    for x in range(0, 640, 80):
        cv2.line(blend, (x, 0), (x, 480), (255, 255, 0), 1)
    for y in range(0, 480, 60):
        cv2.line(blend, (0, y), (640, y), (255, 255, 0), 1)
    if a.bottom > 0:
        rt, nt = ring_top(ref), ring_top(fr)
        if rt:
            cv2.drawMarker(blend, rt, (0, 255, 0), cv2.MARKER_CROSS, 40, 3)
        if nt:
            cv2.drawMarker(blend, nt, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 30, 3)
    good = abs(dx) < 5 and abs(dy) < 5
    col = (0, 200, 0) if good else (0, 0, 255)
    cv2.putText(blend, f"dx {dx:+.0f}px  dy {dy:+.0f}px", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)
    hint = "OK" if good else ("tilt camera DOWN" if dy > 0 else "tilt camera UP") if abs(dy) >= abs(dx) else ("turn camera RIGHT" if dx > 0 else "turn camera LEFT")
    cv2.putText(blend, hint, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)
    view = np.hstack([fr, blend])
    cv2.imwrite(OUT + ".tmp.jpg", view)  # 이 노트북의 OpenCV는 창을 못 띄워서 브라우저로 본다
    os.replace(OUT + ".tmp.jpg", OUT)
    if os.path.exists(STOP):
        break
cap.release()
