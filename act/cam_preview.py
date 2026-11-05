"""녹화 화면의 평소 미리보기(따로 도는 프로세스). 카메라 두 대를 열어 /dev/shm/plugbot_cam/<이름>.jpg에 쓴다.
녹화를 시작하면 record_gui.py가 이 프로세스를 끝내 카메라를 확실히 놓는다. 사용: python act/cam_preview.py <front 경로> <wrist 경로>"""

import sys
import time
from pathlib import Path

import cv2

CAM = Path("/dev/shm/plugbot_cam")
CAM.mkdir(parents=True, exist_ok=True)
caps = {}
for name, path in (("front", sys.argv[1]), ("wrist", sys.argv[2])):
    c = cv2.VideoCapture(path, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    caps[name] = c
try:
    while True:
        for name, c in caps.items():
            ok, f = c.read()
            if ok:
                ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    (CAM / f"{name}.tmp").write_bytes(jpg.tobytes())
                    (CAM / f"{name}.tmp").replace(CAM / f"{name}.jpg")
        time.sleep(1 / 15)
finally:
    for c in caps.values():
        c.release()
