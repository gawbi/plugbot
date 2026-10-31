"""카메라를 다는 여러 방법을 MuJoCo로 그려 비교한다(바닥판 없이 세워 둔 배치).

실행: .venv/bin/python -m real.compare_setups
결과: outputs/setup_compare.png
각 경우: 전체 모습, 플러그 10 mm 삽입 자세에서 카메라 화면(찾은 마커는 초록), 대기, 입구, 삽입 자세별 마커 수와 입구 위치 오차.
"""

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import IK

from . import draw_setup as D
from . import geometry as G
from .vision import Intrinsics, SocketTracker

OUT = D.ROOT / "outputs/setup_compare.png"
LOOK = np.array([0.21, -0.01, -0.06])
CASES = [
    # (이름, 설명, 카메라 위치, 기둥 위치, 받침 거치대(없으면 클램프), 수평 화각, 해상도)
    ("A", "C920, 갑판 옆 앞 클램프 거치대", (0.43, -0.29, 0.27), (0.50, -0.34), None, 78, (1280, 720)),
    ("B", "C920, 로봇 받침 뒤 셀카봉(키트 사진 그대로)", (-0.10, 0.06, 0.40), (-0.10, 0.06), (-0.075, 0.03), 78, (1280, 720)),
    ("C", "C920, 로봇 받침 옆 셀카봉(소켓 탭 쪽)", (-0.02, -0.14, 0.38), (-0.02, -0.14), (-0.01, -0.065), 78, (1280, 720)),
    ("D", "C920, 로봇 맞은편 갑판 너머 삼각대", (0.52, 0.02, 0.30), (0.58, 0.02), None, 78, (1280, 720)),
    ("E", "키트 광각(110°, 640×480 가정), C와 같은 자리", (-0.02, -0.14, 0.38), (-0.02, -0.14), (-0.01, -0.065), 110, (640, 480)),
    ("F", "키트 광각, 소켓 옆 가까이(약 15 cm) 짧은 거치대", (0.30, -0.15, 0.06), (0.36, -0.19), None, 110, (640, 480)),
]
POSES = (("대기", 0.03), ("입구", 0.0), ("삽입", -0.010))
TW, TH = 760, 428  # 그림 칸 크기


def evaluate(case):
    name, desc, cam, pole, bracket, hfov, (w, h) = case
    D.CAM, D.CAM_LOOK, D.POLE, D.BRACKET, D.BOARD, D.HFOV_C920, D.W, D.H = (
        np.array(cam), LOOK, np.array(pole), bracket, False, hfov, w, h)
    s = D.KitScene("car")
    d, m = s.data, s.model
    ik = IK(s)
    cid = m.camera("c920_view").id
    fy = h / 2 / np.tan(np.deg2rad(m.cam_fovy[cid]) / 2)
    tr = SocketTracker(Intrinsics(np.array([[fy, 0, w / 2], [0, fy, h / 2], [0, 0, 1.0]]), np.zeros(5), (w, h)))
    ent = D.socket_point(G.ENTRANCE_IN_SOCKET)
    q = np.array([0.0, -1.0, 0.8, 1.0, 0.0])
    stats, view = [], None
    for tag, off in POSES:
        q = ik.solve(q, ent + [0, 0, off], np.array([0, 0, -1.0]), iters=200)
        d.qpos[s.qadr] = q
        mujoco.mj_forward(m, d)
        img = cv2.cvtColor(D.render(s, "c920_view", w, h), cv2.COLOR_RGB2BGR)
        found = {i: c for i, c in tr.detect(img).items() if i in (0, 5, 6, 7, 8)}
        pose = tr.socket_pose(img)
        err = float("nan")
        if pose is not None:
            R = d.cam_xmat[cid].reshape(3, 3) @ np.diag([1, -1, -1])
            est = R @ (pose[1] + pose[0] @ G.ENTRANCE_IN_SOCKET) + d.cam_xpos[cid]
            err = np.linalg.norm(est - ent) * 1000
        stats.append((tag, len(found), err))
        if tag == "삽입":
            for c in found.values():
                cv2.polylines(img, [c.reshape(4, 2).astype(int)], True, (40, 200, 40), max(2, w // 400))
            view = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    D.W, D.H = 1280, 720
    over = Image.fromarray(D.render(s, "overview", 1280, 720))
    return over, view, stats


def main():
    f_t = ImageFont.truetype(D.FONT, 28)
    f_s = ImageFont.truetype(D.FONT, 23)
    rows = []
    for case in CASES:
        over, view, stats = evaluate(case)
        row = Image.new("RGB", (TW * 2 + 420, TH + 20), "white")
        row.paste(over.resize((TW, TH)), (0, 10))
        row.paste(view.resize((TW, int(TW * view.height / view.width))) if view.width / view.height > TW / TH
                  else view.resize((int(TH * view.width / view.height), TH)), (TW + 10, 10))
        dr = ImageDraw.Draw(row)
        x = TW * 2 + 30
        dr.text((x, 20), f"({case[0]})", font=f_t, fill=(0, 0, 0))
        y = 60
        for line in _wrap(case[1], 15):
            dr.text((x, y), line, font=f_s, fill=(0, 0, 0))
            y += 30
        y += 14
        for tag, n, err in stats:
            e = "못 찾음" if np.isnan(err) else f"{err:.1f} mm"
            dr.text((x, y), f"{tag}: 마커 {n}개, 오차 {e}", font=f_s, fill=(0, 0, 0))
            y += 32
        rows.append(row)
        print(case[0], stats)
    head = Image.new("RGB", (rows[0].width, 60), "white")
    hd = ImageDraw.Draw(head)
    hd.text((20, 14), "전체 모습", font=f_t, fill=(0, 0, 0))
    hd.text((TW + 30, 14), "카메라 화면(플러그 10 mm 삽입 중, 찾은 마커 초록)", font=f_t, fill=(0, 0, 0))
    hd.text((TW * 2 + 30, 14), "입구 위치 오차", font=f_t, fill=(0, 0, 0))
    out = Image.new("RGB", (rows[0].width, 60 + sum(r.height for r in rows)), "white")
    out.paste(head, (0, 0))
    y = 60
    for r in rows:
        out.paste(r, (0, y))
        y += r.height
    out.save(OUT)
    print(OUT)


def _wrap(text, n):
    out, cur = [], ""
    for word in text.split(" "):
        if len(cur) + len(word) + 1 > n and cur:
            out.append(cur)
            cur = word
        else:
            cur = f"{cur} {word}".strip()
    return out + [cur]


if __name__ == "__main__":
    main()
