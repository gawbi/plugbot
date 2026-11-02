"""[발표용 부가 장면] 실제 크기 확장 시각화: UR10e가 실제 크기 전기 페리의 현측 충전 소켓에 플러그를 꽂는다(Isaac Sim 5.1).

메인 주제(실물 SO-101 + ACT)의 검증 결과가 아니다. 모형 실험의 방법(지연 관측, AR 예측, 정온 구간 삽입)을 실제 크기로 키워 보이는 그림이다.
- 로봇: Isaac 기본 자산 UR10e. 관절은 물리 대신 운동학으로 움직인다(관절 속도 제한). 플러그와 소켓 사이 충돌은 계산하지 않는다
- 파도: plugbot.waves.ShipMotion(모형 Hs 6 cm, berth alongside)을 Froude 상사로 키운다(길이 ×10, 시간 ×√10, 각도 그대로)
- 커넥터: 모형 핀 Ø12 mm를 5배(핀 Ø60 mm), 소켓은 cad/make_parts.py 깔때기를 같은 5배. 치수와 이유는 README.md
- 제어: plugbot.control.SocketEstimator(예측 전략)와 같은 상태 기계(접근 → 정온 구간 대기 → 삽입). 시간 인자는 ×√10
- IK: Isaac Lula(UR10e 설명 파일), 끝 프레임 tool0

실행: OMNI_KIT_ACCEPT_EULA=YES LD_LIBRARY_PATH=~/isaacsim_compat/lib ~/isaacsim_env/bin/python isaac/scaleup/run_scaleup.py
        [--seed 8 --hs-model 0.06 --headless --frames-dir data/scaleup_frames]
결과: --frames-dir에 넓은 시점, 근접 시점 PNG와 run.json(시행 기록). 자막과 mp4, GIF는 isaac/scaleup/make_media.py
--trials N(렌더 없이): 같은 운동학 시각화를 N번 돌려 횟수 그대로 센다(물리, 충돌 없는 시각화 기준)
"""

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=2)
ap.add_argument("--hs-model", type=float, default=0.06, help="모형 유의파고(m). 실제는 ×10")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--frames-dir", default=str(ROOT / "data" / "scaleup_frames"))
ap.add_argument("--trials", type=int, default=0, help="0이면 영상 1회. N이면 렌더 없이 시드 0..N-1을 센다")
ap.add_argument("--fps", type=int, default=30)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless or args.trials > 0, "width": 1280, "height": 720})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.robot_motion.motion_generation import interface_config_loader  # noqa: E402
from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from plugbot.control import Params, SocketEstimator  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

# ---------- 치수(m). 근거는 README.md ----------
K_CONN = 5.0  # 커넥터 배율: 모형 핀 Ø12 mm → Ø60 mm
K_LEN, K_TIME = 10.0, np.sqrt(10.0)  # 파도, 선박 운동: Froude 상사(길이 ×10, 시간 ×√10)
PIN_R, PIN_L = 0.030, 0.250
BODY_R, BODY_L = 0.080, 0.200
ADAPTER_R, ADAPTER_L = 0.050, 0.020  # UR10e 플랜지(ISO 9409-1-50-4-M6, Ø63)에 맞춘 어댑터
TIP_L = ADAPTER_L + BODY_L + PIN_L  # tool0 → 핀 끝
BORE_R, BORE_D = 8.0e-3 * K_CONN, 25.0e-3 * K_CONN  # cad/make_parts.py × 5
CONE_TOP_R, OUTER_R, CAP_T = 25.0e-3 * K_CONN, 31.0e-3 * K_CONN, 4.0e-3 * K_CONN
CONE_H = CONE_TOP_R - BORE_R
SOCKET_H = CAP_T + BORE_D + CONE_H
DEPTH_GOAL = (SOCKET_H - (CAP_T + BORE_D)) + 12e-3 * K_CONN  # 입구에서 보어 안쪽(모형 29 mm × 5)

ROBOT_BASE = np.array([0.0, 0.0, 0.60])  # 부두 위 받침(높이 0.6 m)
ENTRANCE0 = np.array([1.20, 0.0, 1.00])  # 쉴 때 소켓 입구(로봇 앞 1.2 m, 받침보다 0.4 m 위)
AXIS0 = np.array([-1.0, 0.0, 0.0])  # 소켓 축(입구 바깥, 로봇 쪽)
HULL_X = ENTRANCE0[0] + SOCKET_H  # 현측 판 면
BEAM, HULL_LEN, DECK_Z, KEEL_Z, WATER_Z = 12.0, 40.0, 2.2, -3.5, -1.2
SHIP_O = np.array([HULL_X + BEAM / 2, 0.0, WATER_Z])  # 선박 운동 기준(수선면 선체 중심)
QUAY_EDGE = 1.00

S = K_TIME
# 시간 인자는 모두 ×√10(관측 주기 포함: AR이 모형과 같은 걸음 수로 앞을 본다). lead는 서보 지연 보상인데, 여기 팔은 운동학이라 한 주기만
P = replace(Params(), latency=0.10 * S, lead=1.0 / 30, ar_window=3.0 * S, gate_speed=0.012 * S, gate_timeout=3.0 * S,
            timeout=12.0 * S, hold=0.2 * S, hover=0.030 * K_CONN, align_tol=0.004 * K_CONN, descend_speed=0.04 * S,
            depth_goal=DEPTH_GOAL, noise=0.0005 * K_LEN, cam_hz=30.0 / S)
QD_MAX = 1.5  # 관절 속도 제한(rad/s). UR10e 정격(어깨 120°/s = 2.1 rad/s)보다 느리게
Q_HOME = np.array([0.0, -2.0, 2.0, -1.57, -1.57, 0.0])


def ship_motion(seed):
    sm = ShipMotion(args.hs_model, seed=seed, berth="alongside")

    def f(t):
        disp, rx, ry = sm(t / K_TIME)
        R = Rotation.from_euler("xyz", [rx, ry, 0.0]).as_matrix()
        return disp * K_LEN, R
    return f


def socket_pose(motion, t):
    disp, R = motion(t)
    o = SHIP_O + disp
    return o, R, o + R @ (ENTRANCE0 - SHIP_O), R @ AXIS0


# ---------- 장면 ----------
world = World(physics_dt=1.0 / 60, rendering_dt=1.0 / args.fps, stage_units_in_meters=1.0)
world.get_physics_context().set_gravity(0.0)  # 팔은 운동학으로 움직인다(중력 처짐 없음)
stage = get_current_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)


def material(name, rgb, rough=0.6, metal=0.0):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    sh = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


MAT = {k: material(k, *v) for k, v in {
    "water": ((0.05, 0.22, 0.32), 0.15), "concrete": ((0.55, 0.55, 0.53), 0.9), "hull": ((0.92, 0.93, 0.95), 0.4),
    "stripe": ((0.75, 0.10, 0.12), 0.5), "hull_low": ((0.10, 0.18, 0.32), 0.5), "cabin": ((0.85, 0.87, 0.90), 0.4),
    "window": ((0.05, 0.08, 0.12), 0.1), "panel": ((0.22, 0.24, 0.27), 0.5), "socket": ((0.95, 0.55, 0.10), 0.45),
    "plug": ((0.18, 0.19, 0.21), 0.35), "pin": ((0.80, 0.80, 0.82), 0.25), "fender": ((0.06, 0.06, 0.06), 0.8),
    "pedestal": ((0.35, 0.37, 0.40), 0.5), "marking": ((0.95, 0.78, 0.10), 0.6)}.items()}


def box(path, center, size, mat, rot=None):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if rot is not None:
        x.AddOrientOp().Set(Gf.Quatf(*map(float, rot)))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(MAT[mat])
    return c


def cylinder(path, center, radius, height, mat, axis="Z"):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(height))
    c.CreateAxisAttr(axis)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(MAT[mat])
    return c


def revolve_mesh(path, profile, n=64, mat="socket"):
    """(r, z) 닫힌 단면을 z축으로 돌린 메시(보이는 것만)."""
    pts, faces, counts = [], [], []
    m = len(profile)
    for i in range(n):
        a = 2 * np.pi * i / n
        for r, z in profile:
            pts.append(Gf.Vec3f(float(r * np.cos(a)), float(r * np.sin(a)), float(z)))
    for i in range(n):
        j = (i + 1) % n
        for k in range(m):
            k2 = (k + 1) % m
            faces += [i * m + k, j * m + k, j * m + k2, i * m + k2]
            counts.append(4)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(MAT[mat])
    return mesh


# 바다, 부두, 받침, 펜더
box("/World/Water", (0, 0, WATER_Z - 0.05), (600, 600, 0.1), "water")
box("/World/Quay", ((QUAY_EDGE - 15) / 2, 0, -1.6), (QUAY_EDGE + 15, 60, 3.2), "concrete")
box("/World/QuayLine", (QUAY_EDGE - 0.25, 0, 0.005), (0.12, 60, 0.01), "marking")
cylinder("/World/Pedestal", (ROBOT_BASE[0], ROBOT_BASE[1], ROBOT_BASE[2] / 2), 0.22, ROBOT_BASE[2], "pedestal")
for y in (-6.0, -2.5, 2.5, 6.0):
    cylinder(f"/World/Fender_{int(y * 10) + 100}", (QUAY_EDGE + 0.18, y, -0.6), 0.18, 1.6, "fender")

# 선박(움직이는 Xform). 자식은 선박 기준점 SHIP_O를 원점으로 둔 좌표
ship_xf = UsdGeom.Xform.Define(stage, "/World/Ship")


def ship_box(name, center_world, size, mat):
    return box(f"/World/Ship/{name}", np.asarray(center_world) - SHIP_O, size, mat)


ship_box("Hull", (HULL_X + BEAM / 2, 0, (DECK_Z + KEEL_Z) / 2), (BEAM, HULL_LEN, DECK_Z - KEEL_Z), "hull")
ship_box("HullLow", (HULL_X + BEAM / 2, 0, (WATER_Z + 0.4 + KEEL_Z) / 2), (BEAM + 0.02, HULL_LEN + 0.02, WATER_Z + 0.4 - KEEL_Z), "hull_low")
ship_box("Stripe", (HULL_X + BEAM / 2, 0, DECK_Z - 0.35), (BEAM + 0.03, HULL_LEN + 0.03, 0.18), "stripe")
ship_box("Cabin", (HULL_X + BEAM / 2 + 0.6, -3.0, DECK_Z + 1.5), (BEAM - 2.0, 24.0, 3.0), "cabin")
ship_box("Windows", (HULL_X + 1.58, -3.0, DECK_Z + 1.9), (0.04, 22.0, 0.9), "window")
ship_box("Bridge", (HULL_X + BEAM / 2 + 1.5, 6.5, DECK_Z + 3.9), (6.0, 5.0, 1.8), "cabin")
ship_box("BridgeWin", (HULL_X + 3.48, 6.5, DECK_Z + 4.1), (0.04, 4.6, 0.7), "window")
ship_box("Panel", (HULL_X - 0.01, 0, ENTRANCE0[2]), (0.04, 1.2, 1.2), "panel")
ship_box("PanelMark", (HULL_X - 0.035, 0, ENTRANCE0[2] + 0.7), (0.01, 1.2, 0.12), "marking")
# 소켓: 단면은 cad/make_parts.py._wall_profile() × 5. 지역 z축(입구 바깥)을 선박의 -x로 돌린다
sock = UsdGeom.Xform.Define(stage, "/World/Ship/Socket")
sx = UsdGeom.Xformable(sock)
sx.AddTranslateOp().Set(Gf.Vec3d(*map(float, np.array([HULL_X, 0, ENTRANCE0[2]]) - SHIP_O)))
sx.AddOrientOp().Set(Gf.Quatf(*map(float, Rotation.from_euler("y", -90, degrees=True).as_quat()[[3, 0, 1, 2]])))
revolve_mesh("/World/Ship/Socket/Wall", [(BORE_R, CAP_T), (OUTER_R, CAP_T), (OUTER_R, SOCKET_H), (CONE_TOP_R, SOCKET_H),
                                         (BORE_R, SOCKET_H - CONE_H)])
cylinder("/World/Ship/Socket/Cap", (0, 0, CAP_T / 2), OUTER_R, CAP_T, "panel")

# 빛
UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(900.0)
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 20.0, -30.0))

# UR10e(Isaac 기본 자산)
UR_USD = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
add_reference_to_stage(UR_USD, "/World/UR10e")
stage.GetPrimAtPath("/World/UR10e").GetAttribute("xformOp:translate").Set(Gf.Vec3d(*map(float, ROBOT_BASE)))  # 참조가 이미 translate를 갖는다
root = next(p.GetPath().pathString for p in stage.Traverse()
            if p.GetPath().pathString.startswith("/World/UR10e") and p.HasAPI(UsdPhysics.ArticulationRootAPI))

ik = LulaKinematicsSolver(**interface_config_loader.load_supported_lula_kinematics_solver_config("UR10e"))
ik.set_robot_base_pose(ROBOT_BASE, np.array([1.0, 0.0, 0.0, 0.0]))
JOINTS = ik.get_joint_names()


def fk_tool(q):
    pos, R = ik.compute_forward_kinematics("tool0", q)
    return np.asarray(pos), np.asarray(R)


robot = world.scene.add(SingleArticulation(prim_path=root, name="ur10e"))
ship = SingleXFormPrim("/World/Ship", name="ship")


def cam(path, pos, look, focal):
    c = UsdGeom.Camera.Define(stage, path)
    c.CreateFocalLengthAttr(float(focal))
    c.CreateHorizontalApertureAttr(20.955)
    c.CreateVerticalApertureAttr(20.955 * 9 / 16)
    c.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000.0))
    z = np.asarray(pos) - np.asarray(look)
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = Rotation.from_matrix(np.stack([x, y, z], 1)).as_quat()
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    xf.AddOrientOp().Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
    return path


CAMS = {"wide": cam("/World/CamWide", (-1.3, -3.4, 2.3), (1.25, 0.6, 1.25), 15.0),
        "close": cam("/World/CamClose", (0.55, -1.25, 1.45), (1.15, 0.0, 0.98), 22.0)}

world.reset()
dof = [robot.get_dof_index(j) for j in JOINTS]


def attach_plug():
    """tool0 프레임(z가 플랜지 바깥)에 어댑터, 몸통, 핀을 단다. wrist_3_link 기준 tool0 자세는 URDF 기구학(Lula FK)으로 구한다."""
    w3 = next(p for p in stage.Traverse() if p.GetPath().pathString.startswith("/World/UR10e") and p.GetName() == "wrist_3_link")

    def T(frame):
        pos, R = ik.compute_forward_kinematics(frame, Q_HOME)
        M = np.eye(4)
        M[:3, :3], M[:3, 3] = np.asarray(R), np.asarray(pos)
        return M
    L = np.linalg.inv(T("wrist_3_link")) @ T("tool0")
    plug = UsdGeom.Xform.Define(stage, w3.GetPath().AppendChild("ChargingPlug"))
    UsdGeom.Xformable(plug).AddTransformOp().Set(Gf.Matrix4d(L.T.tolist()))
    base = plug.GetPath().pathString
    cylinder(base + "/Adapter", (0, 0, ADAPTER_L / 2), ADAPTER_R, ADAPTER_L, "pedestal")
    cylinder(base + "/Body", (0, 0, ADAPTER_L + BODY_L / 2), BODY_R, BODY_L, "plug")
    cylinder(base + "/Pin", (0, 0, ADAPTER_L + BODY_L + (PIN_L - 0.015) / 2), PIN_R, PIN_L - 0.015, "pin")
    cone = UsdGeom.Cone.Define(stage, base + "/Tip")
    cone.CreateRadiusAttr(PIN_R)
    cone.CreateHeightAttr(0.015)
    cone.CreateAxisAttr("Z")
    UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(0, 0, TIP_L - 0.0075))
    UsdShade.MaterialBindingAPI.Apply(cone.GetPrim()).Bind(MAT["pin"])
    cylinder(base + "/Grip", (0, 0, ADAPTER_L + 0.04), BODY_R + 0.006, 0.03, "marking")


def tool_target(tip, axis_out):
    """핀 끝 목표와 소켓 축(바깥) → tool0 위치와 자세(wxyz). tool0 z = 소켓 안쪽, x = 위쪽에 가깝게."""
    z = -axis_out / np.linalg.norm(axis_out)
    x = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z) * z
    x /= np.linalg.norm(x)
    R = np.stack([x, np.cross(z, x), z], 1)
    q = Rotation.from_matrix(R).as_quat()
    return tip - z * TIP_L, np.array([q[3], q[0], q[1], q[2]])


def episode(seed, render=True, frames_dir=None):
    """한 번 시행. 렌더하면 프레임 PNG를 남긴다. (성공, 삽입 시각, 기록)"""
    motion = ship_motion(seed)
    est = SocketEstimator("predictive", P, np.random.default_rng(seed))
    q = Q_HOME.copy()
    dt = 1.0 / args.fps
    phase, depth_cmd, gate_t, hold_t, t_ins = "approach", -P.hover, None, 0.0, None
    captured, miss = None, None  # 핀 끝이 입구 면을 지날 때의 옆 오차(포착 판정)
    log, t, frame, ik_fail = [], 0.0, 0, 0
    t_end = P.timeout
    while t < t_end:
        o, R, ent, axis = socket_pose(motion, t)
        ship.set_world_pose(position=o, orientation=Rotation.from_matrix(R).as_quat()[[3, 0, 1, 2]])  # [ISAAC]
        est.capture(t, ent, axis)
        e = est.estimate(t)
        p_t, R_t = fk_tool(q)
        tip = p_t + R_t[:, 2] * TIP_L
        if e is not None:  # 연결 뒤에도 순응 손목이 소켓을 따라 꽂힌 상태를 유지한다(실제 장치는 래치가 잡고 팔은 놓는다)
            ent_hat, ax_hat, speed = e
            rel = tip - ent_hat
            lat = np.linalg.norm(rel - np.dot(rel, ax_hat) * ax_hat)
            if phase == "approach" and lat < P.align_tol and abs(np.dot(rel, ax_hat) - P.hover) < P.align_tol:
                phase, gate_t = "gate", t
            if phase == "gate" and (speed < P.gate_speed or t - gate_t > P.gate_timeout):
                phase = "insert"
            if phase == "insert":
                depth_cmd = min(depth_cmd + P.descend_speed * dt, P.depth_goal + 0.02)
            if captured:  # 가정: 깔때기와 순응 손목(RCC)이 옆 오차를 흡수해 핀이 실제 소켓 축을 따른다(모형은 물리 접촉으로 흡수)
                ent_hat, ax_hat = ent, axis
            pos, quat = tool_target(ent_hat - ax_hat * depth_cmd, ax_hat)
            q_ik, ok = ik.compute_inverse_kinematics("tool0", pos, quat, warm_start=q)
            if ok:
                q = q + np.clip(np.asarray(q_ik) - q, -QD_MAX * dt, QD_MAX * dt)
            else:
                ik_fail += 1
        # 채점(모형 실험과 같은 기준: 목표 깊이, 보어 안 반경 여유를 hold 동안 유지). 충돌은 계산하지 않는다
        rel = tip - ent
        depth = -float(np.dot(rel, axis))
        lat = float(np.linalg.norm(rel + depth * axis))
        if captured is None and phase == "insert" and depth >= 0.0:
            captured = lat < CONE_TOP_R - PIN_R  # 깔때기 포착 반경(모형 19 mm × 5 = 95 mm)
            if not captured:
                miss, t_end = lat, min(t_end, t + 2.0)
        if t_ins is None and captured:
            if depth >= P.depth_goal and lat < BORE_R - PIN_R + 0.002:
                hold_t += dt
                if hold_t >= P.hold:
                    t_ins, phase, t_end = t, "connected", t + 3.0
            else:
                hold_t = 0.0
        log.append({"t": round(t, 3), "phase": phase, "captured": captured, "depth": round(depth, 4), "lateral": round(lat, 4), "ik_fail": ik_fail})
        if render:
            robot.set_joint_positions(q, joint_indices=np.array(dof))  # [ISAAC] 운동학 재생
            world.step(render=True)  # [ISAAC]
            if frames_dir is not None:
                save_frames(frames_dir, frame, t, phase)
            frame += 1
        t += dt
    return t_ins is not None, t_ins, log


def save_frames(d, k, t, phase):
    from PIL import Image
    for name, c in cam_objs.items():
        rgba = c.get_rgba()  # [ISAAC]
        if rgba is None or rgba.size == 0:
            continue
        Image.fromarray(np.ascontiguousarray(rgba[..., :3]).astype(np.uint8)).save(d / name / f"{k:05d}.png")
    meta.append({"k": k, "t": round(t, 3), "phase": phase})


attach_plug()
if args.trials:
    rows = []
    for s in range(args.trials):
        ok, t_ins, lg = episode(s, render=False)
        ph = {k: sum(1 for x in lg if x["phase"] == k) for k in ("approach", "gate", "insert", "connected")}
        print("   ", ph, "last", lg[-1], "min lateral", min(x["lateral"] for x in lg), "max depth", max(x["depth"] for x in lg))
        rows.append({"seed": s, "connected": ok, "t_connect_s": None if t_ins is None else round(t_ins, 2)})
        print(f"[{s + 1}/{args.trials}] seed {s} {'연결' if ok else '실패'} {rows[-1]['t_connect_s']}")
    n = sum(r["connected"] for r in rows)
    print(f"운동학 시각화 기준(충돌 없음): {n}/{args.trials}")
    (HERE / "trials.json").write_text(json.dumps({"note": "물리 충돌 없는 운동학 시각화 기준. 검증 결과가 아니다",
                                                  "hs_real_m": args.hs_model * K_LEN, "rows": rows,
                                                  "connected": n, "trials": args.trials}, ensure_ascii=False, indent=1))
else:
    from isaacsim.sensors.camera import Camera  # noqa: E402  [ISAAC]
    cam_objs = {k: Camera(prim_path=v, resolution=(1280, 720)) for k, v in CAMS.items()}
    for c in cam_objs.values():
        c.initialize()  # [ISAAC]
    fd = Path(args.frames_dir)
    for k in CAMS:
        (fd / k).mkdir(parents=True, exist_ok=True)
    meta = []
    robot.set_joint_positions(Q_HOME, joint_indices=np.array(dof))
    for _ in range(10):  # 렌더 결과가 따라오도록 몇 장 그린다
        world.step(render=True)  # [ISAAC]
    ok, t_ins, log = episode(args.seed, render=True, frames_dir=fd)
    (fd / "run.json").write_text(json.dumps({"seed": args.seed, "hs_real_m": args.hs_model * K_LEN, "connected": ok,
                                             "t_connect_s": t_ins, "frames": meta}, ensure_ascii=False))
    print(f"seed {args.seed}: {'연결' if ok else '실패'} {t_ins}, 프레임 {len(meta)}장 → {fd}")
sys.stdout.flush()
app.close()
