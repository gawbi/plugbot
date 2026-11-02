"""실물 키트의 앞 카메라(D435) 자리를 3D로 그린다: 셀카봉 위 카메라, 보이는 범위, 65 cm 부스, 그 자리에서 찍히는 화면.

카메라 자리는 plugbot/scene.py CAMERAS_SIDE["front"](로봇 받침 위 기준, 책상은 -0.10 m)와 같다. MuJoCo로 그린다.
실행: MUJOCO_GL=egl ~/plugbot/.venv/bin/python tools/draw_camera_setup_3d.py   결과: outputs/camera_setup_3d.png
"""

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import home_pose
from plugbot.scene import CAMERAS_SIDE, Scene

DESK = -0.10
BOOTH = (-0.17, 0.48, -0.33, 0.32)  # 부스 안쪽 x0, x1, y0, y1(m), 높이 0.65
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def add(scn, typ, size, pos, mat, rgba):
    mujoco.mjv_initGeom(scn.geoms[scn.ngeom], typ, np.asarray(size, float), np.asarray(pos, float),
                        np.asarray(mat, float).reshape(-1), np.asarray(rgba, np.float32))
    scn.ngeom += 1


def line(scn, a, b, w, rgba):
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).reshape(-1), np.asarray(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.asarray(a, float), np.asarray(b, float))
    scn.ngeom += 1


def overlay(scn, cam, look):
    f = look - cam
    f /= np.linalg.norm(f)
    right = np.cross(f, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    pink, pink2 = (0.84, 0.2, 0.42, 1), (0.84, 0.2, 0.42, 0.55)
    add(scn, mujoco.mjtGeom.mjGEOM_BOX, (0.0125, 0.045, 0.015), cam, np.stack([f, right, up], 1), pink)  # D435 몸체
    add(scn, mujoco.mjtGeom.mjGEOM_SPHERE, (0.009, 0, 0), cam + f * 0.014, np.eye(3), (0.1, 0.1, 0.1, 1))
    line(scn, (cam[0], cam[1], DESK), cam - up * 0.016, 0.006, (0.42, 0.45, 0.5, 1))  # 셀카봉
    add(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, (0.05, 0.004, 0), (cam[0], cam[1], DESK + 0.004), np.eye(3), (0.3, 0.3, 0.33, 1))
    th, tv = np.tan(np.deg2rad(69 / 2)), np.tan(np.deg2rad(42 / 2))  # D435 컬러 화각
    L = np.linalg.norm(look - cam) * 1.05
    c = [cam + L * (f + sx * th * right + sy * tv * up) for sx in (-1, 1) for sy in (-1, 1)]
    for p in c:
        line(scn, cam, p, 0.0018, pink2)
    for a, b in ((0, 1), (1, 3), (3, 2), (2, 0)):
        line(scn, c[a], c[b], 0.0018, pink2)
    add(scn, mujoco.mjtGeom.mjGEOM_SPHERE, (0.012, 0, 0), look, np.eye(3), pink)  # 바라보는 점
    x0, x1, y0, y1 = BOOTH
    P = [np.array([x, y, z]) for x in (x0, x1) for y in (y0, y1) for z in (DESK, DESK + 0.65)]
    for i in range(8):
        for j in range(i + 1, 8):
            if np.sum(np.abs(P[i] - P[j]) > 1e-9) == 1:
                line(scn, P[i], P[j], 0.003, (0.07, 0.09, 0.15, 1))


def main():
    s = Scene("ship", "side")
    m, d = s.model, s.data
    s.set_ship(np.zeros(3), 0.0, 0.0)
    d.qpos[:] = 0
    d.qpos[s.qadr] = home_pose(s)
    mujoco.mj_forward(m, d)
    cam, look, _ = CAMERAS_SIDE["front"]
    r = mujoco.Renderer(m, 760, 1000)
    m.vis.global_.fovy = 40
    vc = mujoco.MjvCamera()
    vc.lookat[:], vc.distance, vc.azimuth, vc.elevation = [0.15, 0.0, 0.12], 1.55, -125, -28
    r.update_scene(d, vc)
    overlay(r.scene, cam, look)
    big = Image.fromarray(r.render())
    r2 = mujoco.Renderer(m, 480, 640)
    r2.update_scene(d, camera="front")
    view = Image.fromarray(r2.render())
    fb, fr = ImageFont.truetype(FONT_B, 26), ImageFont.truetype(FONT_R, 21)
    can = Image.new("RGB", (1670, 900), (246, 247, 249))
    dr = ImageDraw.Draw(can)
    dr.text((20, 14), "앞 카메라(D435) 설치 자리: 65 cm 부스 안", font=fb, fill=(17, 24, 39))
    can.paste(big, (10, 70))
    dr.text((1030, 70), "이 자리에서 카메라에 찍히는 화면(시뮬레이션)", font=fb, fill=(214, 51, 108))
    can.paste(view, (1020, 115))
    lines = ["분홍 상자: D435(셀카봉 위), 분홍 선: 보이는 범위, 분홍 공: 바라보는 점",
             "검은 선: 부스 안쪽 650 × 650 × 650 mm", "",
             "로봇 받침 중심에서", "  정면 6 cm, 오른쪽 22 cm, 책상 위 40 cm",
             "  선박 소켓과 자동차 충전구 사이를 보게", "  아래로 약 36° 숙임", "",
             "고정한 뒤에는 움직이지 않는다(보정이 틀어진다)"]
    for i, t in enumerate(lines):
        dr.text((1030, 615 + i * 29), t, font=fr, fill=(55, 65, 81))
    can.save("outputs/camera_setup_3d.png")
    print("outputs/camera_setup_3d.png")


if __name__ == "__main__":
    main()
