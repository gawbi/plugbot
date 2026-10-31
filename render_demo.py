"""시뮬레이션 장면, 비교 GIF, CAD 렌더.

실행: .venv/bin/python render_demo.py
결과(outputs/): sim_ship.png, sim_car.png, compare_hs6.gif, cad_parts.png, cad_kit.png
"""

from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import HOME, Params, run_trial
from plugbot.scene import MM, SO101_XML, Scene
from plugbot.waves import ShipMotion

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "outputs"
CAD = ROOT / "cad/out"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"


def font(size, bold=False):
    return ImageFont.truetype(FONT, size, index=6 if bold else 0)


def label(img, text, sub=None, color=(30, 30, 30)):
    im = Image.fromarray(img) if isinstance(img, np.ndarray) else img
    band = 92 if sub else 56
    out = Image.new("RGB", (im.width, im.height + band), "white")
    out.paste(im, (0, band))
    d = ImageDraw.Draw(out)
    d.text((18, 10), text, font=font(30, True), fill=color)
    if sub:
        d.text((18, 52), sub, font=font(22), fill=(80, 80, 80))
    return out


def camera(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def replay_frame(scene, rec_item):
    t, qpos, mpos, mquat = rec_item[:4]
    scene.data.qpos[:] = qpos
    if scene.mode == "ship":
        scene.data.mocap_pos[:] = mpos
        scene.data.mocap_quat[:] = mquat
    mujoco.mj_forward(scene.model, scene.data)


def find_seed(scene, hs, want):
    """want = {전략: 성공 여부}를 만족하는 첫 시드."""
    for k in range(60):
        ok = all(run_trial(scene, s, ShipMotion(hs, seed=k), seed=k).inserted == v for s, v in want.items())
        if ok:
            return k
    return 0


def sim_stills():
    ship = Scene("ship")
    k = find_seed(ship, 0.06, {"predictive": True})
    rec = []
    run_trial(ship, "predictive", ShipMotion(0.06, seed=k), seed=k, record=rec)
    r = mujoco.Renderer(ship.model, 760, 1000)
    i = max(0, len(rec) - 12)
    replay_frame(ship, rec[i])
    r.update_scene(ship.data, camera([0.16, 0.0, 0.0], 0.68, 150, -22))
    label(r.render(), "해상, 흔들리는 갑판 소켓에 삽입", "모형 유의파고 6 cm, AR 예측 + 정온 구간 삽입").save(OUT / "sim_ship.png")
    r.close()

    car = Scene("car")
    rec = []
    run_trial(car, "static", None, seed=0, record=rec)
    r = mujoco.Renderer(car.model, 760, 1000)
    replay_frame(car, rec[max(0, len(rec) - 12)])
    r.update_scene(car.data, camera([0.17, 0.0, 0.05], 0.68, 150, -18))
    label(r.render(), "육상, 전기차 45° 충전구에 삽입", "같은 로봇, 같은 소켓 규격, 정지 소켓 15/15 성공").save(OUT / "sim_car.png")
    r.close()


def compare_gif(hs=0.06, seconds=4.0, fps=20):
    scene = Scene("ship")
    k = find_seed(scene, hs, {"reactive": False, "predictive": True})
    recs = {}
    for s in ("reactive", "predictive"):
        rec = []
        tr = run_trial(scene, s, ShipMotion(hs, seed=k), seed=k, record=rec)
        recs[s] = (rec, tr)
    r = mujoco.Renderer(scene.model, 420, 520)
    cam = camera([0.20, 0.0, -0.035], 0.42, 140, -22)
    frames = []
    n = int(seconds * fps)
    for j in range(n):
        tiles = []
        for s, title in (("reactive", "최신 위치 추종"), ("predictive", "AR 예측 + 정온 구간 삽입")):
            rec, tr = recs[s]
            t_want = j / fps
            idx = min(range(len(rec)), key=lambda i: abs(rec[i][0] - t_want))
            replay_frame(scene, rec[idx])
            r.update_scene(scene.data, cam)
            if rec[idx] is rec[-1] or idx == len(rec) - 1:
                status = "성공" if tr.inserted else "실패: " + tr.reason
            else:
                status = f"t = {rec[idx][0]:.1f} s"
            tiles.append(label(r.render(), title, status))
        w = sum(t.width for t in tiles) + 8
        frame = Image.new("RGB", (w, tiles[0].height), "white")
        x = 0
        for t in tiles:
            frame.paste(t, (x, 0))
            x += t.width + 8
        frames.append(frame.quantize(colors=128, method=Image.Quantize.MEDIANCUT))
    frames[0].save(OUT / f"compare_hs{int(hs * 100)}.gif", save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    return k


def _mesh_spec(parts, extra=None):
    """parts: [(이름, stl, 위치(m), quat, rgba)] → 렌더용 MjSpec."""
    spec = mujoco.MjSpec.from_file(str(SO101_XML)) if extra == "robot" else mujoco.MjSpec()
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1800, 1400
    spec.add_texture(name="sky2", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[1, 1, 1], rgb2=[0.86, 0.89, 0.93], width=256, height=256)
    w = spec.worldbody
    w.add_light(pos=[0.2, -0.5, 1.2], dir=[-0.1, 0.4, -1], diffuse=[0.8, 0.8, 0.8], specular=[0.2, 0.2, 0.2])
    w.add_light(pos=[0.8, 0.6, 0.9], dir=[-0.6, -0.5, -0.6], diffuse=[0.35, 0.35, 0.35])
    for name, stl, pos, quat, rgba in parts:
        spec.add_mesh(name=f"m_{name}", file=str(stl), scale=[MM] * 3)
        w.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"m_{name}", pos=pos, quat=quat, rgba=rgba)
    return spec


def _quat(axis, ang):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array(axis, float), ang)
    return q


def cad_parts():
    ident = [1, 0, 0, 0]
    parts = [
        ("plug", CAD / "plug.stl", [0.00, 0.0, 0.0], ident, [0.12, 0.30, 0.62, 1]),
        ("socket", CAD / "socket.stl", [0.12, 0.0, 0.0], ident, [0.93, 0.55, 0.12, 1]),
        ("car_port", CAD / "car_port.stl", [0.32, -0.04, 0.0], ident, [0.25, 0.38, 0.58, 1]),
        ("deck_base", CAD / "deck_base.stl", [0.05, -0.27, 0.0], ident, [0.62, 0.64, 0.68, 1]),
        ("deck_plate", CAD / "deck_plate.stl", [0.33, -0.27, -0.020], ident, [0.80, 0.82, 0.86, 1]),
    ]
    spec = _mesh_spec(parts)
    spec.worldbody.add_geom(name="bed", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01], pos=[0, 0, -0.0005],
                            rgba=[0.95, 0.95, 0.94, 1])
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 1100, 1600)
    r.update_scene(d, camera([0.19, -0.12, 0.0], 0.72, 115, -50))
    img = Image.fromarray(r.render())
    dr = ImageDraw.Draw(img)
    for txt, xy in (("플러그 (핀 Ø12, 자석 포켓)", (250, 160)), ("깔때기 소켓 (입구 Ø36 → 보어 Ø15)", (560, 110)),
                    ("자동차 충전구 블록 (45°)", (1080, 130)), ("흔들림 갑판 베이스", (200, 800)), ("흔들림 갑판판 + 레버", (900, 900))):
        dr.text(xy, txt, font=font(30, True), fill=(30, 30, 30))
    label(img, "3D 프린트 부품 (PLA, STEP/STL 제공)", "cad/out: plug, socket, car_port, deck_base, deck_plate").save(OUT / "cad_parts.png")


def cad_kit():
    """실물 시연 배치: SO-101(높이 100 mm 받침 위) + 흔들림 갑판 + 소켓 + 카메라. 시뮬레이션과 같은 높이 차."""
    table = -0.10
    deck_q = _quat([0, 0, 1], -np.pi / 2)  # 갑판 x → 월드 -y, 갑판 y → 월드 +x
    deck_origin = np.array([0.205, 0.0, table])
    sock = deck_origin + np.array([0.015, 0.0, 0.045])
    parts = [
        ("deck_base", CAD / "deck_base.stl", deck_origin, deck_q, [0.62, 0.64, 0.68, 1]),
        ("deck_plate", CAD / "deck_plate.stl", deck_origin, deck_q, [0.82, 0.84, 0.88, 1]),
        ("socket", CAD / "socket.stl", sock, deck_q, [0.93, 0.55, 0.12, 1]),
    ]
    spec = _mesh_spec(parts, extra="robot")
    w = spec.worldbody
    w.add_geom(name="riser", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.09, 0.09, 0.05], pos=[-0.01, 0, -0.05],
               rgba=[0.55, 0.57, 0.60, 1])
    w.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01], pos=[0, 0, table],
               rgba=[0.93, 0.92, 0.89, 1])
    # ArUco 마커(갑판 x → 월드 -y 방향 탭 위)
    tab = sock + np.array([0.0, -0.055, 0.0041])
    w.add_geom(name="marker", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0125, 0.0125, 0.0003], pos=tab, rgba=[0.05, 0.05, 0.05, 1])
    w.add_geom(name="marker_in", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.006, 0.004, 0.0004], pos=tab + [0.002, 0.003, 0],
               rgba=[1, 1, 1, 1])
    # 카메라(웹캠) 스탠드
    cam_pos = np.array([0.16, -0.24, 0.10])
    w.add_geom(name="cam_pole", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.004, (cam_pos[2] - table) / 2, 0],
               pos=[cam_pos[0], cam_pos[1], (cam_pos[2] + table) / 2], rgba=[0.3, 0.3, 0.3, 1])
    w.add_geom(name="cam_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.03, 0.012, 0.015], pos=cam_pos,
               quat=_quat([0, 0, 1], np.deg2rad(60)), rgba=[0.12, 0.12, 0.12, 1])
    spec.body("gripper").add_geom(name="plug", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="m_plug_kit",
                                  pos=[0.002, 0, -0.150], rgba=[0.12, 0.30, 0.62, 1])
    spec.add_mesh(name="m_plug_kit", file=str(CAD / "plug.stl"), scale=[MM] * 3)
    m = spec.compile()
    d = mujoco.MjData(m)
    sc = Scene("ship")
    from plugbot.control import IK
    ik = IK(sc)
    q = ik.solve(HOME.copy(), np.array([0.22, 0.0, -0.055 + 0.0395 + 0.03]), np.array([0, 0, -1.0]), iters=300)
    d.qpos[:5] = q
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 1100, 1500)
    r.update_scene(d, camera([0.13, 0.0, -0.04], 0.75, 145, -26))
    label(r.render(), "실물 시연 키트 조립도", "SO-101(높이 100 mm 받침=부두), 흔들림 갑판, 깔때기 소켓, ArUco 마커, 웹캠").save(OUT / "cad_kit.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    sim_stills()
    print("stills ok")
    k = compare_gif()
    print("gif ok, seed", k)
    cad_parts()
    cad_kit()
    print("cad ok")
