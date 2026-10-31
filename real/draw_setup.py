"""실물 시연 배치도: SO-101, 흔들림 갑판(CAD 부품), 소켓과 마커 시트, 클램프 거치대의 C920을 한 바닥판 위에 그린다.

실행: .venv/bin/python -m real.draw_setup
결과: outputs/setup_overview.png (전체 모습 + C920에 보이는 화면, 마커 추적 결과 표시)

좌표: 로봇 베이스 중심이 원점, +x가 로봇 정면, z 위(m). 책상(바닥판 윗면)은 z = -0.10(받침 높이 100 mm).
갑판은 힌지(갑판 -y)가 로봇 쪽을 보게 놓고, 소켓 중심이 로봇 앞 220 mm, 베이스보다 55 mm 아래에 온다(PRINT_GUIDE).
"""

from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import IK
from plugbot.scene import MM, Scene

from . import geometry as G
from .sim_world import TEX, _marker_texture, _xyaxes
from .vision import Intrinsics, SocketTracker

ROOT = Path(__file__).resolve().parents[1]
CAD = ROOT / "cad/out"
OUT = ROOT / "outputs/setup_overview.png"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"

TABLE_Z = -0.10
DECK_ORIGIN = np.array([0.205, 0.0, TABLE_Z])  # 갑판 부품 좌표 원점(베이스 바닥 중심)
DECK_YAW = -np.pi / 2  # 갑판 +y(레버 쪽)가 로봇 정면(+x), 갑판 +x(소켓 탭 쪽)가 로봇 오른쪽(-y)
SOCKET_ON_DECK = np.array([0.0, 0.015, 0.045])
CAM = np.array([0.43, -0.29, 0.27])  # C920 위치: 책상 위 약 37 cm, 소켓 오른쪽 앞에서 비스듬히
CAM_LOOK = np.array([0.21, -0.01, -0.06])
POLE = np.array([0.50, -0.34])  # 클램프 거치대 기둥(바닥판 가장자리)
BOARD = True  # 공통 바닥판을 그릴지
BRACKET = None  # 로봇 받침에 셀카봉을 붙일 때 받침 옆면 위치 (x, y). None이면 클램프
HFOV_C920 = 78.0
W, H = 1280, 720


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _quat_z(a):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([0, 0, 1.0]), a)
    return q


R_DECK = _rotz(DECK_YAW)
SOCKET_POS = DECK_ORIGIN + R_DECK @ SOCKET_ON_DECK


def socket_point(p_socket):
    """소켓 좌표계 점 → 월드."""
    return SOCKET_POS + R_DECK @ p_socket


class KitScene(Scene):
    def _add_world(self, spec):
        w = spec.worldbody
        spec.add_texture(name="wood", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT,
                         rgb1=[0.80, 0.68, 0.50], width=8, height=8)
        w.add_light(pos=[0.2, -0.3, 1.2], dir=[0, 0.2, -1], diffuse=[0.75, 0.75, 0.75], specular=[0.1, 0.1, 0.1])
        w.add_light(pos=[0.9, 0.5, 0.8], dir=[-0.6, -0.4, -0.6], diffuse=[0.35, 0.35, 0.35])
        w.add_geom(name="desk", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], pos=[0, 0, TABLE_Z - 0.012],
                   rgba=[0.93, 0.93, 0.92, 1])
        # 공통 바닥판(합판 600 × 420 × 12 mm): 로봇, 갑판, 카메라를 함께 고정
        if BOARD:
            w.add_geom(name="board", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.30, 0.21, 0.006],
                       pos=[0.22, -0.06, TABLE_Z - 0.006], rgba=[0.80, 0.68, 0.50, 1])
        w.add_geom(name="block", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.065, 0.065, 0.05], pos=[-0.01, 0, -0.05],
                   rgba=[0.62, 0.50, 0.36, 1])
        for name in ("deck_base", "deck_plate"):
            spec.add_mesh(name=name, file=str(CAD / f"{name}.stl"), scale=[MM] * 3)
        rgba = {"deck_base": [0.70, 0.72, 0.75, 1], "deck_plate": [0.86, 0.87, 0.89, 1]}
        for name in ("deck_base", "deck_plate"):
            w.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, pos=DECK_ORIGIN,
                       quat=_quat_z(DECK_YAW), rgba=rgba[name], contype=0, conaffinity=0)
        w.add_geom(name="socket", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="socket_vis", pos=SOCKET_POS,
                   quat=_quat_z(DECK_YAW), rgba=[0.93, 0.55, 0.12, 1], contype=0, conaffinity=0)
        # 마커: 탭(id 0, 30 mm 종이) + 갑판 시트(id 5~8, 36 mm 종이)
        papers = [(G.SOCKET_MARKER_ID, 0.025, 0.030, G.MARKER_IN_SOCKET + [0, 0, 0.0002])]
        papers += [(mid, 0.030, 0.036, np.array([x, y, 1.2]) * MM) for mid, (x, y) in G.DECK_MARKERS.items()]
        for mid, size, paper, center in papers:
            path = TEX.replace("_sim_socket_marker", f"_sim_marker_{mid}")
            _marker_texture(mid, round(300 * (paper / size - 1) / 2), path)
            spec.add_texture(name=f"aruco{mid}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=path)
            spec.add_material(name=f"aruco{mid}", textures=["", f"aruco{mid}"])
            h = paper / 2
            spec.add_mesh(name=f"quad{mid}", uservert=[-h, -h, 0, h, -h, 0, h, h, 0, -h, h, 0],
                          userface=[0, 1, 2, 0, 2, 3], usertexcoord=[0, 1, 1, 1, 1, 0, 0, 0],
                          inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
            w.add_geom(name=f"aruco{mid}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"quad{mid}", material=f"aruco{mid}",
                       pos=socket_point(center), quat=_quat_z(DECK_YAW), contype=0, conaffinity=0)
        # 갑판 마커 시트(흰 종이) 바탕
        w.add_geom(name="sheet", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.085, 0.06, 0.0002],
                   pos=socket_point(np.array([0, -0.015, 0.0008])), quat=_quat_z(DECK_YAW),
                   rgba=[0.97, 0.97, 0.96, 1], contype=0, conaffinity=0)
        # C920 거치대: 바닥판에 물린 클램프 + 기둥 + 가로 팔 + 카메라
        dark = [0.12, 0.12, 0.13, 1]
        if BRACKET is None:
            w.add_geom(name="clamp", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.025, 0.02, 0.02],
                       pos=[POLE[0], POLE[1], TABLE_Z + 0.008], rgba=dark)
        else:  # 받침 옆면에서 기둥까지 이어 주는 거치대(키트의 받침 홀더)
            mid = (np.array(BRACKET) + POLE) / 2
            w.add_geom(name="bracket", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.012, 0.012, 0.015],
                       pos=[mid[0], mid[1], TABLE_Z + 0.015], rgba=[0.45, 0.25, 0.75, 1])
            w.add_geom(name="bracket_link", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.008, 0, 0],
                       fromto=[BRACKET[0], BRACKET[1], TABLE_Z + 0.02, POLE[0], POLE[1], TABLE_Z + 0.02],
                       rgba=[0.45, 0.25, 0.75, 1])
        w.add_geom(name="pole", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.007, 0, 0],
                   fromto=[POLE[0], POLE[1], TABLE_Z, POLE[0], POLE[1], CAM[2] + 0.03], rgba=dark)
        w.add_geom(name="arm", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.005, 0, 0],
                   fromto=[POLE[0], POLE[1], CAM[2] + 0.03, CAM[0], CAM[1], CAM[2] + 0.025], rgba=dark)
        xy = _xyaxes(CAM, CAM_LOOK)
        Rc = np.stack([xy[:3], xy[3:], np.cross(xy[:3], xy[3:])], axis=1)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, Rc.flatten())
        w.add_geom(name="c920", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.047, 0.0145, 0.012], pos=CAM + Rc @ [0, 0, 0.012],
                   quat=q, rgba=dark)
        w.add_geom(name="c920_lens", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.009, 0.002, 0], pos=CAM,
                   quat=q, rgba=[0.25, 0.30, 0.40, 1])
        fovy = np.rad2deg(2 * np.arctan(np.tan(np.deg2rad(HFOV_C920) / 2) * H / W))
        w.add_camera(name="c920_view", pos=CAM, xyaxes=xy, fovy=fovy)
        ov = np.array([-0.30, -0.95, 0.62])
        w.add_camera(name="overview", pos=ov, xyaxes=_xyaxes(ov, np.array([0.20, -0.10, 0.04])), fovy=42)


def render(scene, cam, w=W, h=H):
    r = mujoco.Renderer(scene.model, h, w)
    r.update_scene(scene.data, camera=cam)
    return r.render()


def project(scene, cam, p, w=W, h=H):
    m, d = scene.model, scene.data
    cid = m.camera(cam).id
    R = d.cam_xmat[cid].reshape(3, 3)
    v = R.T @ (np.asarray(p) - d.cam_xpos[cid])
    f = h / 2 / np.tan(np.deg2rad(m.cam_fovy[cid]) / 2)
    return w / 2 + f * v[0] / -v[2], h / 2 - f * v[1] / -v[2]


def main():
    s = KitScene("car")
    d = s.data
    # 팔: 소켓 입구 위 30 mm에서 꽂으려는 자세
    entrance = socket_point(G.ENTRANCE_IN_SOCKET)
    q = IK(s).solve(np.array([0.0, -1.0, 0.8, 1.0, 0.0]), entrance + [0, 0, 0.03], np.array([0, 0, -1.0]), iters=200)
    d.qpos[s.qadr] = q
    mujoco.mj_forward(s.model, d)

    over = Image.fromarray(render(s, "overview"))
    view = render(s, "c920_view")
    # C920 화면에서 실제 추적 코드로 소켓 찾기
    fy = H / 2 / np.tan(np.deg2rad(s.model.cam_fovy[s.model.camera("c920_view").id]) / 2)
    intr = Intrinsics(np.array([[fy, 0, W / 2], [0, fy, H / 2], [0, 0, 1.0]]), np.zeros(5), (W, H))
    bgr = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
    tr = SocketTracker(intr)
    found = tr.detect(bgr)
    pose = tr.socket_pose(bgr)
    for mid, c in found.items():
        cv2.polylines(bgr, [c.reshape(4, 2).astype(int)], True, (40, 200, 40), 2)
    if pose is not None:
        R, t = pose
        pts = np.array([t + R @ G.ENTRANCE_IN_SOCKET, t + R @ (G.ENTRANCE_IN_SOCKET + [0, 0, 0.05])])
        uv, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), intr.K, intr.dist)
        a, b = uv.reshape(2, 2).astype(int)
        cv2.circle(bgr, tuple(a), 7, (0, 0, 230), 2)
        cv2.arrowedLine(bgr, tuple(a), tuple(b), (0, 0, 230), 2, tipLength=0.25)
    view = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    f_lab = ImageFont.truetype(FONT, 26)
    f_cap = ImageFont.truetype(FONT, 30)
    dr = ImageDraw.Draw(over)
    labels = [
        ("1", np.array([0.0, 0.0, 0.16]), "SO-101 팔로워 (받침 100 mm 위)"),
        ("2", socket_point(np.array([0, 0, 0.04])), "소켓 + 갑판 마커 시트"),
        ("3", DECK_ORIGIN + R_DECK @ np.array([0, 0.085, 0.045]), "레버: 손으로 흔든다"),
        ("4", CAM + [0, 0, 0.02], "C920 (소켓 위 약 33 cm)"),
        ("5", np.array([POLE[0], POLE[1], TABLE_Z + 0.06]), "클램프 거치대"),
        ("6", np.array([0.45, 0.10, TABLE_Z]), "공통 바닥판(합판)"),
    ]
    for num, p, _ in labels:
        u, v = project(s, "overview", p)
        dr.ellipse([u - 17, v - 17, u + 17, v + 17], fill=(255, 255, 255), outline=(0, 0, 0), width=2)
        dr.text((u - 7, v - 15), num, font=f_lab, fill=(0, 0, 0))
    legend = Image.new("RGB", (W, 150), "white")
    dl = ImageDraw.Draw(legend)
    for k, (num, _, text) in enumerate(labels):
        x, y = 30 + (k % 3) * 420, 18 + (k // 3) * 60
        dl.ellipse([x, y, x + 34, y + 34], outline=(0, 0, 0), width=2)
        dl.text((x + 10, y + 1), num, font=f_lab, fill=(0, 0, 0))
        dl.text((x + 46, y + 1), text, font=f_lab, fill=(0, 0, 0))
    gap = 30
    canvas = Image.new("RGB", (W * 2 + gap, H + 150 + 60), "white")
    canvas.paste(over, (0, 0))
    canvas.paste(view, (W + gap, 0))
    canvas.paste(legend, (0, H + 10))
    dc = ImageDraw.Draw(canvas)
    dc.text((20, H + 160), "(a) 전체 배치: 로봇, 갑판, 카메라를 바닥판 하나에 고정", font=f_cap, fill=(0, 0, 0))
    n = len([i for i in found if i in (0, 5, 6, 7, 8)])
    dc.text((W + gap + 20, H + 160), f"(b) C920 화면: 추적 코드가 마커 {n}개를 찾고 입구와 축(빨강)을 계산", font=f_cap,
            fill=(0, 0, 0))
    canvas.save(OUT)
    print(OUT, "마커", sorted(found), "소켓 자세", "있음" if pose is not None else "없음")


if __name__ == "__main__":
    main()
