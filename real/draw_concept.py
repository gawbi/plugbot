"""최종 구상 그림: 바닥판 없이 세운 배치, C270(로봇 받침 옆 셀카봉), 손목 카메라(키트 카메라 이동), 서보 흔들림 갑판.

실행: .venv/bin/python -m real.draw_concept
결과: outputs/concept_setup.png  (a) 전체 (b) 흔들림 장치 (c) C270 화면 (d) 손목 카메라 화면
"""

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import IK

from . import draw_setup as D
from . import geometry as G
from .vision import Intrinsics, SocketTracker

OUT = D.ROOT / "outputs/concept_setup.png"
W, H = 960, 600


def deck_point(p_mm):
    """갑판 부품 좌표(mm) → 월드."""
    return D.DECK_ORIGIN + D.R_DECK @ (np.asarray(p_mm) * 0.001)


class ConceptScene(D.KitScene):
    def _add_world(self, spec):
        super()._add_world(spec)
        w = spec.worldbody
        q = D._quat_z(D.DECK_YAW)
        blue, dark, metal = [0.10, 0.25, 0.60, 1], [0.10, 0.10, 0.11, 1], [0.75, 0.76, 0.78, 1]
        # 서보(MG996R 40 × 20 × 38 mm): 레버 끝 아래 책상 위, 축은 힌지와 나란히
        w.add_geom(name="servo", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.019, 0.010, 0.020],
                   pos=deck_point([22, 70, 20]), quat=q, rgba=dark, contype=0, conaffinity=0)
        w.add_geom(name="horn", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0015, 0.025, 0.003],
                   pos=deck_point([2, 70, 30]), quat=q, rgba=[0.95, 0.95, 0.95, 1], contype=0, conaffinity=0)
        w.add_geom(name="rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0015, 0, 0],
                   fromto=np.concatenate([deck_point([0, 90, 30]), deck_point([0, 90, 42])]), rgba=metal,
                   contype=0, conaffinity=0)
        # 아두이노 나노 + 전원 어댑터 선
        w.add_geom(name="arduino", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.022, 0.009, 0.003],
                   pos=deck_point([70, 110, 3]), quat=q, rgba=blue, contype=0, conaffinity=0)
        w.add_geom(name="wire1", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0012, 0, 0],
                   fromto=np.concatenate([deck_point([50, 108, 4]), deck_point([30, 75, 10])]), rgba=[0.8, 0.2, 0.1, 1],
                   contype=0, conaffinity=0)
        w.add_geom(name="adapter", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02, 0.015, 0.012],
                   pos=deck_point([110, 140, 12]), quat=q, rgba=dark, contype=0, conaffinity=0)
        # 갑판 장식: 배 모양 테두리 띠
        w.add_geom(name="hull_band", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0875, 0.064, 0.004],
                   pos=deck_point([0, 0, 38]), quat=q, rgba=[0.75, 0.16, 0.14, 1], contype=0, conaffinity=0)


def main():
    D.CAM, D.CAM_LOOK = np.array([-0.02, -0.14, 0.38]), np.array([0.21, -0.01, -0.06])
    D.POLE, D.BRACKET, D.BOARD, D.HFOV_C920 = np.array([-0.02, -0.14]), (-0.01, -0.065), False, 60.0
    s = ConceptScene("car")
    m, d = s.model, s.data
    entrance = D.socket_point(G.ENTRANCE_IN_SOCKET)
    q = IK(s).solve(np.array([0.0, -1.0, 0.8, 1.0, 0.0]), entrance + [0, 0, 0.015], np.array([0, 0, -1.0]), iters=200)
    d.qpos[s.qadr] = q
    mujoco.mj_forward(m, d)

    def free_view(lookat, dist, az, el):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        r = mujoco.Renderer(m, H, W)
        r.update_scene(d, cam)
        return Image.fromarray(r.render())

    over = free_view([0.12, -0.05, 0.12], 1.25, 228, -22)
    shaker = free_view(deck_point([10, 80, 25]), 0.30, 200, -18)
    rc = mujoco.Renderer(m, 720, 1280)
    rc.update_scene(d, camera="c920_view")
    c270 = cv2.cvtColor(rc.render(), cv2.COLOR_RGB2BGR)
    fy = 360 / np.tan(np.deg2rad(m.cam_fovy[m.camera("c920_view").id]) / 2)
    tr = SocketTracker(Intrinsics(np.array([[fy, 0, 640], [0, fy, 360], [0, 0, 1.0]]), np.zeros(5), (1280, 720)))
    found = tr.detect(c270)
    for c in found.values():
        cv2.polylines(c270, [c.reshape(4, 2).astype(int)], True, (40, 200, 40), 3)
    c270 = Image.fromarray(cv2.cvtColor(c270, cv2.COLOR_BGR2RGB)).resize((W, H))
    rw = mujoco.Renderer(m, H, W)
    rw.update_scene(d, camera="wrist_cam")
    wrist = Image.fromarray(rw.render())

    font = ImageFont.truetype(D.FONT, 24)
    caps = ["(a) 전체: 로봇, C270(받침 옆 셀카봉), 손목 카메라, 흔들림 갑판",
            "(b) 흔들림 장치: 서보 → 연결 막대 → 갑판 레버, 아두이노",
            f"(c) C270 화면: 마커 {len([i for i in found if i in (0, 5, 6, 7, 8)])}개 인식(초록)",
            "(d) 손목 카메라 화면: 플러그와 깔때기 입구"]
    gap, cap_h = 16, 44
    canvas = Image.new("RGB", (W * 2 + gap, (H + cap_h) * 2 + gap), "white")
    dr = ImageDraw.Draw(canvas)
    for k, im in enumerate((over, shaker, c270, wrist)):
        x, y = (k % 2) * (W + gap), (k // 2) * (H + cap_h + gap)
        canvas.paste(im, (x, y))
        dr.text((x + 8, y + H + 8), caps[k], font=font, fill=(0, 0, 0))
    canvas.save(OUT)
    print(OUT, "마커", sorted(found))


if __name__ == "__main__":
    main()
