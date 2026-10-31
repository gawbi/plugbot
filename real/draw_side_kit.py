"""본선 실물 키트 구상도(side 배치): 흔들림 갑판 위 세운 판 소켓, 왼쪽 자동차 충전구, L자 플러그, 앞 카메라, 손목 카메라.

실행: .venv/bin/python -m real.draw_side_kit
결과: outputs/side_kit.png  (a) 전체 (b) 흔들림 장치 (c) 앞 카메라 화면과 추적 (d) 손목 카메라 화면
"""

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import IK, home_pose
from plugbot.scene import CAMERAS_SIDE, MM, SIDE_AXIS, SIDE_QUAT, SOCKET_H, Scene, _xyaxes

from . import geometry as G
from .draw_setup import CAD, FONT, ROOT, TEX, _marker_texture
from .vision import Intrinsics, SocketTracker

OUT = ROOT / "outputs/side_kit.png"
TABLE_Z = -0.10
DECK_TOP = 0.045  # 책상에서 갑판판 윗면
W, H = 960, 600
CW, CH = 1280, 720


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q


R_DECK = _rotz(-np.pi / 2)  # 갑판 +y(레버 쪽)가 로봇 정면(+x)
R_SOCK = np.zeros(9)
mujoco.mju_quat2Mat(R_SOCK, SIDE_QUAT)
R_SOCK = R_SOCK.reshape(3, 3)  # 소켓 좌표 → (정면을 보는) 월드


def deck_point(p_mm):
    return G.SIDE_DECK_ORIGIN + R_DECK @ (np.asarray(p_mm) * MM)


def mounts():
    """(이름, 세운 판 원점, 판 회전, 소켓 원점, 소켓 회전, 받침 높이)"""
    ship_mount = deck_point([0, 15, DECK_TOP * 1000])
    out = [("ship", ship_mount, R_DECK, ship_mount + [0, 0, 0.085], R_SOCK, 0.0)]
    Rz = _rotz(G.CAR_PORT_ANGLE)
    base = G.PAN_AXIS_XY + Rz @ (np.array([ship_mount[0], 0.0, TABLE_Z]) - G.PAN_AXIS_XY)  # 어깨 회전축 둘레로
    out.append(("car", base, Rz @ R_DECK, base + [0, 0, DECK_TOP + 0.085], Rz @ R_SOCK, DECK_TOP))
    return out


class SideKitScene(Scene):
    def _add_world(self, spec):
        w = spec.worldbody
        w.add_light(pos=[0.2, -0.3, 1.2], dir=[0, 0.2, -1], diffuse=[0.75, 0.75, 0.75], specular=[0.1, 0.1, 0.1])
        w.add_light(pos=[0.9, 0.5, 0.8], dir=[-0.6, -0.4, -0.6], diffuse=[0.35, 0.35, 0.35])
        w.add_geom(name="desk", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], pos=[0, 0, TABLE_Z],
                   rgba=[0.93, 0.93, 0.92, 1], contype=0, conaffinity=0)
        w.add_geom(name="block", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.065, 0.065, 0.05], pos=[-0.01, 0, -0.05],
                   rgba=[0.62, 0.50, 0.36, 1], contype=0, conaffinity=0)
        for name in ("deck_base", "deck_plate", "side_mount", "car_port_side"):
            spec.add_mesh(name=name, file=str(CAD / f"{name}.stl"), scale=[MM] * 3)
        grey = {"deck_base": [0.70, 0.72, 0.75, 1], "deck_plate": [0.86, 0.87, 0.89, 1]}
        for name, rgba in grey.items():
            w.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, pos=G.SIDE_DECK_ORIGIN,
                       quat=_quat(R_DECK), rgba=rgba, contype=0, conaffinity=0)
        papers = [(G.SOCKET_MARKER_ID, 0.025, 0.030, G.MARKER_IN_SOCKET + [0, 0, 0.0002])]
        papers += [(mid, 0.030, 0.036, np.array([x, y, 0.4]) * MM) for mid, (x, y) in G.PANEL_MARKERS.items()]
        for off in G.TARGET_ID_OFFSET.values():
            for mid0, size, paper, _ in papers:
                mid = mid0 + off
                path = TEX.replace("_sim_socket_marker", f"_sim_marker_{mid}")
                _marker_texture(mid, round(300 * (paper / size - 1) / 2), path)
                spec.add_texture(name=f"aruco{mid}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=path)
                spec.add_material(name=f"aruco{mid}", textures=["", f"aruco{mid}"])
                h = paper / 2
                spec.add_mesh(name=f"quad{mid}", uservert=[-h, -h, 0, h, -h, 0, h, h, 0, -h, h, 0],
                              userface=[0, 1, 2, 0, 2, 3], usertexcoord=[0, 1, 1, 1, 1, 0, 0, 0],
                              inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
        for tag, mpos, mR, spos, sR, _ in mounts():
            w.add_geom(name=f"{tag}_mount", type=mujoco.mjtGeom.mjGEOM_MESH,
                       meshname="side_mount" if tag == "ship" else "car_port_side", pos=mpos, quat=_quat(mR),
                       rgba=[0.86, 0.87, 0.89, 1] if tag == "ship" else [0.20, 0.33, 0.52, 1], contype=0, conaffinity=0)
            w.add_geom(name=f"{tag}_socket", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="socket_vis", pos=spos,
                       quat=_quat(sR), rgba=[0.93, 0.55, 0.12, 1], contype=0, conaffinity=0)
            w.add_geom(name=f"{tag}_sheet", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0595, 0.085, 0.0002],
                       pos=spos + sR @ np.array([-0.0195, 0, 0.0002]), quat=_quat(sR), rgba=[0.97, 0.97, 0.96, 1],
                       contype=0, conaffinity=0)
            for mid0, _, _, center in papers:
                mid = mid0 + G.TARGET_ID_OFFSET[tag]
                w.add_geom(name=f"{tag}_aruco{mid}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"quad{mid}",
                           material=f"aruco{mid}", pos=spos + sR @ (center + [0, 0, 0.0004]), quat=_quat(sR),
                           contype=0, conaffinity=0)
        # 흔들림 장치: 레버 아래 서보, 연결 막대, 아두이노, 어댑터
        q = _quat(R_DECK)
        dark, blue, metal = [0.10, 0.10, 0.11, 1], [0.10, 0.25, 0.60, 1], [0.75, 0.76, 0.78, 1]
        w.add_geom(name="servo", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.019, 0.010, 0.020], pos=deck_point([22, 100, 20]),
                   quat=q, rgba=dark, contype=0, conaffinity=0)
        w.add_geom(name="horn", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0015, 0.025, 0.003], pos=deck_point([2, 100, 30]),
                   quat=q, rgba=[0.95, 0.95, 0.95, 1], contype=0, conaffinity=0)
        w.add_geom(name="rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0015, 0, 0],
                   fromto=np.concatenate([deck_point([0, 120, 30]), deck_point([0, 120, 42])]), rgba=metal,
                   contype=0, conaffinity=0)
        w.add_geom(name="arduino", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.022, 0.009, 0.003],
                   pos=deck_point([-70, 120, 3]), quat=q, rgba=blue, contype=0, conaffinity=0)
        w.add_geom(name="adapter", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02, 0.015, 0.012],
                   pos=deck_point([-110, 150, 12]), quat=q, rgba=dark, contype=0, conaffinity=0)
        # 앞 카메라: 책상에 세운 셀카봉 + 카메라
        cpos, look, hfov = CAMERAS_SIDE["front"]
        w.add_geom(name="pole", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.007, 0, 0],
                   fromto=[cpos[0], cpos[1] - 0.03, TABLE_Z, cpos[0], cpos[1] - 0.03, cpos[2] + 0.02], rgba=dark,
                   contype=0, conaffinity=0)
        w.add_geom(name="pole_foot", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.05, 0.004, 0],
                   pos=[cpos[0], cpos[1] - 0.03, TABLE_Z + 0.004], rgba=dark, contype=0, conaffinity=0)
        xy = _xyaxes(cpos, look)
        Rc = np.stack([xy[:3], xy[3:], np.cross(xy[:3], xy[3:])], axis=1)
        w.add_geom(name="cam_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.036, 0.015, 0.012],
                   pos=cpos + Rc @ [0, 0, 0.013], quat=_quat(Rc), rgba=dark, contype=0, conaffinity=0)
        w.add_camera(name="front_view", pos=cpos, xyaxes=xy, fovy=float(np.rad2deg(
            2 * np.arctan(np.tan(np.deg2rad(hfov) / 2) * CH / CW))))


def main():
    s = SideKitScene("car", "side")
    m, d = s.model, s.data
    ship = mounts()[0]
    ent = ship[3] + SIDE_AXIS * SOCKET_H  # 소켓 바닥에서 축 방향(로봇 쪽)으로 46 mm
    q = home_pose(s)
    ik = IK(s)
    for back in (0.02, 0.0, -0.015):  # 입구 앞 대기에서 15 mm 꽂은 자세까지
        for _ in range(4):
            q = ik.solve(q, ent + SIDE_AXIS * back, -SIDE_AXIS, iters=200)
    d.qpos[s.qadr] = q
    mujoco.mj_forward(m, d)

    def free_view(lookat, dist, az, el):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        r = mujoco.Renderer(m, H, W)
        r.update_scene(d, cam)
        return Image.fromarray(r.render())

    rc = mujoco.Renderer(m, CH, CW)  # 추적에 쓰는 화면을 먼저 만든다(한 프로세스에서 렌더러를 여러 개 만들면 뒤의 것이 어긋날 수 있다)
    rc.update_scene(d, camera="front_view")
    front = cv2.cvtColor(rc.render(), cv2.COLOR_RGB2BGR)
    fy = CH / 2 / np.tan(np.deg2rad(m.cam_fovy[m.camera("front_view").id]) / 2)
    intr = Intrinsics(np.array([[fy, 0, CW / 2], [0, fy, CH / 2], [0, 0, 1.0]]), np.zeros(5), (CW, CH))
    found = SocketTracker(intr).detect(front)
    # 실제 추적 코드로 두 소켓 입구를 각각 구해 정답과 비교(테두리를 그리기 전에)
    R_cam = d.cam_xmat[m.camera("front_view").id].reshape(3, 3) @ np.diag([1, -1, -1])
    t_cam = d.cam_xpos[m.camera("front_view").id]
    errs = {}
    for tag, _, _, spos, sR, _ in mounts():
        est = SocketTracker(intr, tag).socket(front)
        truth = spos + sR @ G.ENTRANCE_IN_SOCKET
        errs[tag] = None if est is None else float(np.linalg.norm(R_cam @ est[0] + t_cam - truth) * 1000)
    for mid, c in found.items():  # 선박 마커 초록, 자동차 마커 파랑
        cv2.polylines(front, [c.reshape(4, 2).astype(int)], True, (40, 200, 40) if mid < 10 else (230, 120, 30), 3)
    front = Image.fromarray(cv2.cvtColor(front, cv2.COLOR_BGR2RGB)).resize((W, H))
    rw = mujoco.Renderer(m, H, W)
    rw.update_scene(d, camera="wrist_cam")
    wrist = Image.fromarray(rw.render())
    over = free_view([0.20, 0.06, -0.02], 1.10, 35, -28)
    shaker = free_view(deck_point([0, 70, 40]), 0.45, 100, -18)

    font = ImageFont.truetype(FONT, 24)
    caps = ["(a) 전체: 흔들림 갑판 위 세운 판(선박), 왼쪽 자동차 충전구, 앞 카메라",
            "(b) 흔들림 장치: 서보 → 연결 막대 → 갑판 레버 (힌지는 로봇 쪽)",
            f"(c) 앞 카메라: 선박(초록) {sum(i < 10 for i in found)}개, 자동차(파랑) {sum(i >= 10 for i in found)}개 마커 인식",
            "(d) 손목 카메라: 그리퍼가 아래를 봐서 플러그 몸통과 판 윗면만 보인다"]
    gap, cap_h = 16, 44
    canvas = Image.new("RGB", (W * 2 + gap, (H + cap_h) * 2 + gap), "white")
    dr = ImageDraw.Draw(canvas)
    for k, im in enumerate((over, shaker, front, wrist)):
        x, y = (k % 2) * (W + gap), (k // 2) * (H + cap_h + gap)
        canvas.paste(im, (x, y))
        dr.text((x + 8, y + H + 8), caps[k], font=font, fill=(0, 0, 0))
    canvas.save(OUT)
    print(OUT, "마커", sorted(found), "입구 추적 오차 mm", errs)


if __name__ == "__main__":
    main()
