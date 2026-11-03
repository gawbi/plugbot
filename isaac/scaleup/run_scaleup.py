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
ap.add_argument("--livery", choices=["panstar", "plain"], default="panstar",
                help="panstar: 팬스타 미라클호 사진을 흉내 낸 현측 무늬(make_livery.py), plain: 예전 흰 선체와 빨간 띠")
ap.add_argument("--max-frames", type=int, default=0, help="0보다 크면 이만큼만 렌더(구도 확인용)")
ap.add_argument("--replay", default="", help="SO-101 시범 에피소드 폴더(data/isaac_side/episode_XXXX). 그 궤적을 UR10e로 옮겨 재생한다\n"
                "(이종 원격조종 시각화: 리더 팔 대신 녹화된 시범)")
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
from plugbot.dataset import JOINT_NAMES  # noqa: E402
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
HY = 14.0  # 선체 길이 방향 중심(소켓에서 뱃머리 쪽으로). 선미 끝이 소켓 6 m 옆에 와서 넓은 시점에 바다가 보인다

S = K_TIME
# 시간 인자는 모두 ×√10(관측 주기 포함: AR이 모형과 같은 걸음 수로 앞을 본다). lead는 서보 지연 보상인데, 여기 팔은 운동학이라 한 주기만
P = replace(Params(), latency=0.10 * S, lead=1.0 / 30, ar_window=3.0 * S, gate_speed=0.012 * S, gate_timeout=3.0 * S,
            timeout=12.0 * S, hold=0.2 * S, hover=0.030 * K_CONN, align_tol=0.004 * K_CONN, descend_speed=0.04 * S,
            depth_goal=DEPTH_GOAL, noise=0.0005 * K_LEN, cam_hz=30.0 / S)
QD_MAX = 1.5  # 관절 속도 제한(rad/s). UR10e 정격(어깨 120°/s = 2.1 rad/s)보다 느리게
QDD_MAX = 6.0  # 관절 가속도 제한(rad/s²): 산업용 팔처럼 매끄럽게 가속, 감속한다
KP_JOINT = 6.0  # 관절 목표를 따라가는 속도 이득(1/s)
TAU_TARGET, TAU_INSERT, TAU_CAPTURED = 0.25, 0.12, 0.08  # 목표 필터 시정수(s): 관측 잡음과 9.5 Hz 계단을 펴서 떨림을 없앤다. 삽입, 포착 뒤는 빠르게
LOCAL_ASSETS = Path.home() / "isaacsim_assets" / "Assets" / "Isaac" / "5.1"  # isaac/scaleup/fetch_assets.py가 받아 둔 곳


def asset(rel):
    local = LOCAL_ASSETS / rel.lstrip("/")
    return str(local) if local.exists() else get_assets_root_path() + rel  # 없으면 Isaac 자산 서버(인터넷)
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
    "water": ((0.02, 0.10, 0.16), 0.30), "concrete": ((0.36, 0.36, 0.35), 0.9), "hull": ((0.92, 0.93, 0.95), 0.4),
    "stripe": ((0.75, 0.10, 0.12), 0.5), "hull_low": ((0.10, 0.18, 0.32), 0.5), "cabin": ((0.85, 0.87, 0.90), 0.4),
    "window": ((0.05, 0.08, 0.12), 0.1), "panel": ((0.22, 0.24, 0.27), 0.5), "socket": ((0.95, 0.55, 0.10), 0.45),
    "plug": ((0.18, 0.19, 0.21), 0.35), "pin": ((0.80, 0.80, 0.82), 0.25), "fender": ((0.06, 0.06, 0.06), 0.8),
    "pedestal": ((0.35, 0.37, 0.40), 0.5), "marking": ((0.95, 0.78, 0.10), 0.6)}.items()}
MAT["orange"] = material("orange", (0.91, 0.50, 0.16), 0.45)
MAT["glass_blue"] = material("glass_blue", (0.16, 0.36, 0.62), 0.15)


def texture_material(name, png):
    """PNG를 diffuseColor로 쓰는 재질(메시 primvar st)."""
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    sh = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    st = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/St")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(png))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


def emissive(name, rgb):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    sh = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(5 * np.asarray(rgb))))
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


UsdShade.Shader.Get(stage, "/World/Looks/water/S").CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.15)  # 비스듬히 볼 때 하늘이 덜 비친다
MAT["led_wait"] = emissive("led_wait", (1.0, 0.45, 0.05))
MAT["led_on"] = emissive("led_on", (0.1, 1.0, 0.35))
MAT["cable"] = material("cable", (0.03, 0.03, 0.035), 0.55)


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
box("/World/Water", (0, 0, WATER_Z - 0.25), (2000, 2000, 0.1), "water")
SEA_N, SEA_L = 90, 140.0  # 출렁이는 바다 메시(가로세로 칸 수, 한 변 m)
_g = np.linspace(-SEA_L / 2, SEA_L / 2, SEA_N)
SEA_X, SEA_Y = np.meshgrid(_g + SEA_L / 2 - 2.0, _g)
_sea_rng = np.random.default_rng(7)
SEA_W = [(2 * np.pi / T, np.deg2rad(d), a, _sea_rng.uniform(0, 2 * np.pi)) for T, d, a in
         [(6.3, 200, 0.12), (4.6, 170, 0.07), (8.1, 225, 0.09), (3.2, 190, 0.035), (2.4, 160, 0.02)]]  # 주기(s), 방향(°), 진폭(m)


def sea_height(x, y, t):
    h = np.zeros_like(x)
    for w, d, a, ph in SEA_W:
        k = w * w / 9.81  # 깊은 물 분산 관계
        h += a * np.sin(k * (x * np.cos(d) + y * np.sin(d)) - w * t + ph)
    return h


sea = UsdGeom.Mesh.Define(stage, "/World/Sea")
_idx = np.arange(SEA_N * SEA_N).reshape(SEA_N, SEA_N)
sea.CreateFaceVertexIndicesAttr(Vt.IntArray(np.stack([_idx[:-1, :-1], _idx[:-1, 1:], _idx[1:, 1:], _idx[1:, :-1]], -1).reshape(-1).tolist()))
sea.CreateFaceVertexCountsAttr(Vt.IntArray([4] * (SEA_N - 1) ** 2))
sea.CreateSubdivisionSchemeAttr("none")
UsdShade.MaterialBindingAPI.Apply(sea.GetPrim()).Bind(MAT["water"])


def update_sea(t):
    h = sea_height(SEA_X, SEA_Y, t)
    gy, gx = np.gradient(h, _g, _g)
    n = np.stack([-gx, -gy, np.ones_like(h)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    pts = np.stack([SEA_X, SEA_Y, WATER_Z + h], -1).reshape(-1, 3)
    sea.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    sea.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(n.reshape(-1, 3).astype(np.float32)))


sea.CreatePointsAttr()
sea.CreateNormalsAttr()
sea.SetNormalsInterpolation("vertex")
update_sea(0.0)
box("/World/Quay", ((QUAY_EDGE - 15) / 2, 0, -1.6), (QUAY_EDGE + 15, 60, 3.2), "concrete")
box("/World/QuayLine", (QUAY_EDGE - 0.25, 0, 0.005), (0.12, 60, 0.01), "marking")
cylinder("/World/Pedestal", (ROBOT_BASE[0], ROBOT_BASE[1], ROBOT_BASE[2] / 2), 0.22, ROBOT_BASE[2], "pedestal")
for y in (-6.0, -2.5, 2.5, 6.0):
    cylinder(f"/World/Fender_{int(y * 10) + 100}", (QUAY_EDGE + 0.18, y, -0.6), 0.18, 1.6, "fender")
for y in (-8.0, 4.5, 9.0):  # 계선주
    cylinder(f"/World/Bollard_{int(y * 10) + 100}", (QUAY_EDGE - 0.5, y, 0.25), 0.16, 0.5, "fender")
CABINET = np.array([-0.85, 1.7, 0.0])  # 로봇 뒤 왼쪽(넓은 시점을 가리지 않게)
box("/World/Cabinet", CABINET + (0, 0, 0.8), (0.5, 0.7, 1.6), "panel")
box("/World/CabinetTop", CABINET + (0, 0, 1.62), (0.52, 0.72, 0.05), "marking")
box("/World/CabinetFace", CABINET + (0.255, 0, 1.05), (0.01, 0.5, 0.35), "window")
CABLE_OUT = CABINET + np.array([0.27, -0.12, 0.55])

# 선박(움직이는 Xform). 자식은 선박 기준점 SHIP_O를 원점으로 둔 좌표
ship_xf = UsdGeom.Xform.Define(stage, "/World/Ship")


def ship_box(name, center_world, size, mat):
    return box(f"/World/Ship/{name}", np.asarray(center_world) - SHIP_O, size, mat)


ship_box("Hull", (HULL_X + BEAM / 2, HY, (DECK_Z + KEEL_Z) / 2), (BEAM, HULL_LEN, DECK_Z - KEEL_Z), "hull")
ship_box("HullLow", (HULL_X + BEAM / 2, HY, (WATER_Z + 0.4 + KEEL_Z) / 2), (BEAM + 0.02, HULL_LEN + 0.02, WATER_Z + 0.4 - KEEL_Z), "hull_low")
if args.livery == "plain":
    ship_box("Stripe", (HULL_X + BEAM / 2, HY, DECK_Z - 0.35), (BEAM + 0.03, HULL_LEN + 0.03, 0.18), "stripe")
else:  # 부두 쪽 현측 전체에 무늬 그림(왼쪽 = 뱃머리 +y, 오른쪽 = 선미 -y). 반대쪽 현측은 보이지 않는다
    MAT["livery"] = texture_material("livery", HERE / "livery_panstar.png")
    _lx = HULL_X - 0.006
    _y0, _y1 = HY + HULL_LEN / 2, HY - HULL_LEN / 2
    _q = [(_lx, _y0, DECK_Z), (_lx, _y1, DECK_Z), (_lx, _y1, KEEL_Z), (_lx, _y0, KEEL_Z)]
    liv = UsdGeom.Mesh.Define(stage, "/World/Ship/Livery")
    liv.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, np.asarray(v) - SHIP_O)) for v in _q]))
    liv.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    liv.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    liv.CreateSubdivisionSchemeAttr("none")
    liv.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(liv).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(
        Vt.Vec2fArray([Gf.Vec2f(0, 1), Gf.Vec2f(1, 1), Gf.Vec2f(1, 0), Gf.Vec2f(0, 0)]))
    UsdShade.MaterialBindingAPI.Apply(liv.GetPrim()).Bind(MAT["livery"])
ship_box("Cabin", (HULL_X + BEAM / 2 + 0.6, HY - 3.0, DECK_Z + 1.5), (BEAM - 2.0, 24.0, 3.0), "cabin")
ship_box("Windows", (HULL_X + 1.58, HY - 3.0, DECK_Z + 1.9), (0.04, 22.0, 0.9), "window")
ship_box("Bridge", (HULL_X + BEAM / 2 + 1.5, HY + 6.5, DECK_Z + 3.9), (6.0, 5.0, 1.8), "cabin")
ship_box("BridgeWin", (HULL_X + 3.48, HY + 6.5, DECK_Z + 4.1), (0.04, 4.6, 0.7), "window")
ship_box("Panel", (HULL_X - 0.01, 0, ENTRANCE0[2]), (0.04, 1.2, 1.2), "panel")
ship_box("PanelMark", (HULL_X - 0.035, 0, ENTRANCE0[2] + 0.7), (0.01, 1.2, 0.12), "marking")
for i, y in enumerate(np.arange(HY - 17.0, HY + 17.5, 1.6) if args.livery == "plain" else []):  # 둥근 창(현측)
    if abs(y) > 1.0:
        c = cylinder(f"/World/Ship/Port_{i}", np.array([HULL_X - 0.005, y, DECK_Z - 1.0]) - SHIP_O, 0.17, 0.02, "window", axis="X")
for i, y in enumerate(np.arange(HY - 19.5, HY + 20.0, 1.5)):  # 난간 기둥
    cylinder(f"/World/Ship/Rail_{i}", np.array([HULL_X + 0.15, y, DECK_Z + 0.5]) - SHIP_O, 0.02, 1.0, "pedestal")
box("/World/Ship/RailTop", np.array([HULL_X + 0.15, HY, DECK_Z + 1.0]) - SHIP_O, (0.05, 39.0, 0.05), "pedestal")
if args.livery == "panstar":  # 사진처럼 흰 상부 갑판 두 층, 파란 창 띠, 선미 쪽 주황 구조물
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath("/World/Ship/Windows")).Bind(MAT["glass_blue"])
    ship_box("Cabin2", (HULL_X + BEAM / 2 + 1.0, HY - 1.0, DECK_Z + 3.6), (BEAM - 3.0, 20.0, 1.4), "cabin")
    ship_box("Windows2", (HULL_X + 2.48, HY - 1.0, DECK_Z + 3.65), (0.04, 19.0, 0.6), "glass_blue")
    ship_box("AftOrange", (HULL_X + BEAM / 2 + 1.0, HY - 15.0, DECK_Z + 3.6), (BEAM - 4.0, 4.0, 2.4), "orange")
    ship_box("AftOrangeTop", (HULL_X + BEAM / 2 + 1.0, HY - 15.0, DECK_Z + 4.9), (BEAM - 5.0, 3.0, 0.3), "glass_blue")
cylinder("/World/Ship/Mast", np.array([HULL_X + BEAM / 2 + 1.5, HY + 6.5, DECK_Z + 6.8]) - SHIP_O, 0.12, 4.0, "cabin")
LED_PANEL = cylinder("/World/Ship/PanelLed", np.array([HULL_X - 0.04, 0.45, ENTRANCE0[2] + 0.45]) - SHIP_O, 0.05, 0.02, "led_wait", axis="X")
LED_CAB = box("/World/CabinetLed", CABINET + (0.26, 0.0, 1.38), (0.02, 0.18, 0.06), "led_wait")
cable = UsdGeom.BasisCurves.Define(stage, "/World/Cable")
cable.CreateTypeAttr("linear")
cable.CreateCurveVertexCountsAttr(Vt.IntArray([32]))
cable.CreateWidthsAttr(Vt.FloatArray([0.045] * 32))
cable.SetWidthsInterpolation("vertex")
cable.CreatePointsAttr()
UsdShade.MaterialBindingAPI.Apply(cable.GetPrim()).Bind(MAT["cable"])


def update_cable(p_end, out_dir):
    """부두 캐비닛에서 플러그 뒤까지 늘어진 케이블(3차 베지에)."""
    p0 = CABLE_OUT
    p1 = p0 + np.array([0.25, 0.0, -0.45])
    p2 = p_end - out_dir * 0.45 + np.array([0.0, 0.0, -0.55])
    u = np.linspace(0, 1, 32)[:, None]
    pts = (1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * p1 + 3 * (1 - u) * u ** 2 * p2 + u ** 3 * p_end
    pts[:, 2] = np.maximum(pts[:, 2], 0.03)  # 부두 바닥 위
    cable.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))


def set_led(on):
    for g in (LED_PANEL, LED_CAB):
        UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(MAT["led_on" if on else "led_wait"])
# 소켓: 단면은 cad/make_parts.py._wall_profile() × 5. 지역 z축(입구 바깥)을 선박의 -x로 돌린다
sock = UsdGeom.Xform.Define(stage, "/World/Ship/Socket")
sx = UsdGeom.Xformable(sock)
sx.AddTranslateOp().Set(Gf.Vec3d(*map(float, np.array([HULL_X, 0, ENTRANCE0[2]]) - SHIP_O)))
sx.AddOrientOp().Set(Gf.Quatf(*map(float, Rotation.from_euler("y", -90, degrees=True).as_quat()[[3, 0, 1, 2]])))
revolve_mesh("/World/Ship/Socket/Wall", [(BORE_R, CAP_T), (OUTER_R, CAP_T), (OUTER_R, SOCKET_H), (CONE_TOP_R, SOCKET_H),
                                         (BORE_R, SOCKET_H - CONE_H)])
cylinder("/World/Ship/Socket/Cap", (0, 0, CAP_T / 2), OUTER_R, CAP_T, "panel")

# 빛
sky = UsdLux.DomeLight.Define(stage, "/World/Sky")
sky.CreateIntensityAttr(700.0)
sky.CreateTextureFileAttr(asset("/NVIDIA/Assets/Skies/Cloudy/champagne_castle_1_4k.hdr"))
sky.CreateTextureFormatAttr("latlong")
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(1800.0)
sun.CreateAngleAttr(0.6)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.85))
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-32.0, 18.0, -55.0))

# UR10e(Isaac 기본 자산)
UR_USD = asset("/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd")
print("UR10e:", UR_USD)
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


def look_quat(pos, look):
    z = np.asarray(pos, float) - np.asarray(look, float)
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1.0], z)
    x /= np.linalg.norm(x)
    q = Rotation.from_matrix(np.stack([x, np.cross(z, x), z], 1)).as_quat()
    return Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2]))


def move_cam(path, pos, look):
    p = stage.GetPrimAtPath(path)
    p.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*map(float, pos)))
    p.GetAttribute("xformOp:orient").Set(look_quat(pos, look))


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


WIDE_PATH = [((-1.6, -9.5, 3.4), (1.2, 1.0, 1.0)), ((-0.7, -6.2, 2.5), (1.1, 0.3, 1.0))]  # 부두 쪽 선미 근처에서 로봇 어깨 너머로 천천히 다가간다
CAMS = {"wide": cam("/World/CamWide", *WIDE_PATH[0], 20.0),
        "close": cam("/World/CamClose", (0.55, -1.25, 1.45), (1.15, 0.0, 0.98), 22.0),
        "sea": cam("/World/CamSea", (1.7, -17.0, 3.6), (0.5, 0.0, 1.0), 20.0)}  # 부두와 선체 사이 틈의 연장선, 선미 바깥 바다 위
if args.livery == "panstar":  # 부두 안쪽 높은 곳에서 현측 무늬와 로봇을 함께(편집본 첫 장면)
    CAMS["ship"] = cam("/World/CamShip", (-13.0, -1.5, 6.5), (1.4, 5.0, 1.4), 16.0)

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
    q, qd = Q_HOME.copy(), np.zeros(6)
    q_goal, tgt = Q_HOME.copy(), None  # IK 목표 관절, 필터한 tool0 목표(위치, 자세)
    qd_ff = np.zeros(6)  # 목표 관절의 속도(앞먹임): 비례 추종만으로는 움직이는 소켓을 v/Kp만큼 늦게 따른다
    dt = 1.0 / args.fps
    set_led(False)
    tgt_started = False
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
            if tgt is None:
                tgt = (pos, Rotation.from_quat(quat[[1, 2, 3, 0]]))
            a = 1.0 - np.exp(-dt / (TAU_CAPTURED if captured else TAU_INSERT if phase == "insert" else TAU_TARGET))
            r_new = Rotation.from_quat(quat[[1, 2, 3, 0]])
            r_f = tgt[1] * Rotation.from_rotvec(a * (tgt[1].inv() * r_new).as_rotvec())
            tgt = (tgt[0] + a * (pos - tgt[0]), r_f)
            qf = r_f.as_quat()
            q_ik, ok = ik.compute_inverse_kinematics("tool0", tgt[0], np.array([qf[3], qf[0], qf[1], qf[2]]), warm_start=q_goal)
            if ok:
                q_ik = np.asarray(q_ik)
                qd_ff = (q_ik - q_goal) / dt if tgt_started else np.zeros(6)
                q_goal, tgt_started = q_ik, True
            else:
                ik_fail += 1
        # 관절: 속도와 가속도를 제한해 목표를 따라간다(산업용 팔처럼 매끄럽게)
        qd_cmd = np.clip(qd_ff + KP_JOINT * (q_goal - q), -QD_MAX, QD_MAX)
        qd = qd + np.clip(qd_cmd - qd, -QDD_MAX * dt, QDD_MAX * dt)
        q = q + qd * dt
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
                    set_led(True)
            else:
                hold_t = 0.0
        log.append({"t": round(t, 3), "phase": phase, "captured": captured, "depth": round(depth, 4), "lateral": round(lat, 4), "ik_fail": ik_fail})
        if render:
            update_sea(t)
            p_t, R_t = fk_tool(q)
            update_cable(p_t + R_t[:, 2] * (ADAPTER_L + 0.06) - R_t[:, 0] * BODY_R, R_t[:, 2])
            u = min(1.0, t / max(P.timeout * 0.5, 1e-6))
            u = u * u * (3 - 2 * u)
            move_cam("/World/CamWide", *(np.array(WIDE_PATH[0][i]) + u * (np.array(WIDE_PATH[1][i]) - np.array(WIDE_PATH[0][i])) for i in (0, 1)))
            robot.set_joint_positions(q, joint_indices=np.array(dof))  # [ISAAC] 운동학 재생
            world.step(render=True)  # [ISAAC]
            if frames_dir is not None:
                save_frames(frames_dir, frame, t, phase)
            frame += 1
            if args.max_frames and frame >= args.max_frames:
                break
        t += dt
    return t_ins is not None, t_ins, log


def socket_basis(axis):
    """소켓 좌표계(열: x = 위쪽에 가깝게, y, z = 축 바깥)."""
    z = axis / np.linalg.norm(axis)
    x = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z) * z
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)


def grow(local):
    """모형 소켓 기준 상대 위치 → 실제 크기. 소켓 5 cm 안은 커넥터 배율(×5) 그대로, 먼 곳은 덜 키워 UR10e 작업 범위에 넣는다."""
    r = np.linalg.norm(local)
    if r <= 0.05:
        return local * K_CONN
    return local / r * (0.05 * K_CONN + 2.0 * (r - 0.05))


def replay_targets(ep_dir):
    """SO-101 시범(관절, 30 fps) → 각 시각의 (핀 끝의 모형 소켓 기준 상대 위치, 핀 방향) — 소켓 좌표계."""
    import mujoco
    from plugbot.scene import Scene, ship_pose
    meta = json.loads((Path(ep_dir) / "meta.json").read_text())
    d = np.load(Path(ep_dir) / "data.npz")
    kin = Scene("car", "side")
    kd = mujoco.MjData(kin.model)
    sm = ShipMotion(meta["hs"], seed=meta["seed"], berth="alongside") if meta["hs"] > 0 else (lambda t: (np.zeros(3), 0.0, 0.0))
    out = []
    for t, st in zip(d["timestamp"], d["state"]):
        kd.qpos[:] = 0
        kd.qpos[kin.qadr] = np.deg2rad(st[:5])
        mujoco.mj_kinematics(kin.model, kd)
        tip = kd.site_xpos[kin.tip_site].copy()
        pin = kd.xmat[kin.gripper_body].reshape(3, 3) @ kin.pin_axis
        _, _, ent, axis = ship_pose("side", sm, float(t))
        B = socket_basis(axis)
        out.append((float(t), B.T @ (tip - ent), B.T @ pin))
    return meta, out


def episode_replay(ep_dir, seed, frames_dir=None):
    """녹화된 SO-101 시범을 실제 크기 소켓 기준으로 옮겨 UR10e가 따라 한다. 소켓 위치는 시뮬레이션의 참값을 쓴다
    (실제로는 카메라로 재야 한다). 사람은 소켓 기준으로만 움직이고, 배 운동 보정은 시스템이 한다는 구성이다."""
    ep_meta, traj = replay_targets(ep_dir)
    motion = ship_motion(seed)
    q, qd, q_goal, qd_ff, tgt, started = Q_HOME.copy(), np.zeros(6), Q_HOME.copy(), np.zeros(6), None, False
    dt = 1.0 / args.fps
    set_led(False)
    hold_t, t_ins, t, frame, k, captured = 0.0, None, 0.0, 0, 0, None
    # 시작 동기화: UR10e를 SO-101 첫 자세에 대응하는 자세에 두고 시작한다(멀리서 따라오느라 늦지 않게)
    _, _, ent0, axis0 = socket_pose(motion, 0.0)
    B0 = socket_basis(axis0)
    z0 = B0 @ traj[0][2]
    x0 = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z0) * z0
    x0 /= np.linalg.norm(x0)
    r0 = Rotation.from_matrix(np.stack([x0, np.cross(z0, x0), z0], 1)).as_quat()
    q0, ok0 = ik.compute_inverse_kinematics("tool0", ent0 + B0 @ grow(traj[0][1]) - z0 * TIP_L,
                                            np.array([r0[3], r0[0], r0[1], r0[2]]), warm_start=Q_HOME)
    if ok0:
        q, q_goal = np.asarray(q0), np.asarray(q0)
    t_end = traj[-1][0] + 2.5
    while t < t_end:
        o, R, ent, axis = socket_pose(motion, t)
        ship.set_world_pose(position=o, orientation=Rotation.from_matrix(R).as_quat()[[3, 0, 1, 2]])  # [ISAAC]
        while k + 1 < len(traj) and traj[k + 1][0] <= t:
            k += 1
        _, rel, pin = traj[k]
        B = socket_basis(axis)
        g = grow(rel)
        if captured:  # 자동 모드와 같은 가정: 포착 뒤 옆 방향은 깔때기와 순응 손목이 맞추고, 깊이는 사람 입력을 따른다
            g, pin = np.array([0.0, 0.0, g[2]]), np.array([0.0, 0.0, -1.0])
        tip_goal = ent + B @ g
        z = B @ pin  # 핀 방향(실제 소켓 좌표계로 옮김) = tool0 z
        x = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z) * z
        x /= np.linalg.norm(x)
        rot = Rotation.from_matrix(np.stack([x, np.cross(z, x), z], 1))
        pos = tip_goal - z * TIP_L
        if tgt is None:
            tgt = (pos, rot)
        a = 1.0 - np.exp(-dt / TAU_CAPTURED)  # 사람 입력은 바로 따른다(시범 궤적 자체가 매끄럽다)
        tgt = (tgt[0] + a * (pos - tgt[0]), tgt[1] * Rotation.from_rotvec(a * (tgt[1].inv() * rot).as_rotvec()))
        qf = tgt[1].as_quat()
        q_ik, ok = ik.compute_inverse_kinematics("tool0", tgt[0], np.array([qf[3], qf[0], qf[1], qf[2]]), warm_start=q_goal)
        if ok:
            q_ik = np.asarray(q_ik)
            qd_ff = (q_ik - q_goal) / dt if started else np.zeros(6)
            q_goal, started = q_ik, True
        qd_cmd = np.clip(qd_ff + KP_JOINT * (q_goal - q), -QD_MAX, QD_MAX)
        qd = qd + np.clip(qd_cmd - qd, -QDD_MAX * dt, QDD_MAX * dt)
        q = q + qd * dt
        p_t, R_t = fk_tool(q)
        tip = p_t + R_t[:, 2] * TIP_L
        rel_t = tip - ent
        depth = -float(np.dot(rel_t, axis))
        lat = float(np.linalg.norm(rel_t + depth * axis))
        if captured is None and depth >= 0.0:
            captured = lat < CONE_TOP_R - PIN_R  # 깔때기 포착 반경 95 mm
        phase = "connected" if t_ins is not None else ("insert" if depth > -0.02 else "teleop")
        if t_ins is None and captured and depth >= P.depth_goal and lat < BORE_R - PIN_R + 0.002:
            hold_t += dt
            if hold_t >= P.hold:
                t_ins = t
                t_end = max(t_end, t + 2.5)
                set_led(True)
        elif t_ins is None:
            hold_t = 0.0
        if frames_dir is not None:
            update_sea(t)
            update_cable(p_t + R_t[:, 2] * (ADAPTER_L + 0.06) - R_t[:, 0] * BODY_R, R_t[:, 2])
            robot.set_joint_positions(q, joint_indices=np.array(dof))  # [ISAAC] 운동학 재생
            world.step(render=True)  # [ISAAC]
            save_frames(frames_dir, frame, t, phase)
            meta[-1]["so101_frame"] = k
            frame += 1
        t += dt
    return t_ins is not None, t_ins, ep_meta


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
    if args.replay:
        move_cam("/World/CamWide", *WIDE_PATH[1])
        ok, t_ins, ep_meta = episode_replay(args.replay, args.seed, frames_dir=fd)
        extra = {"replay": str(args.replay), "so101_episode": ep_meta}
    else:
        ok, t_ins, log = episode(args.seed, render=True, frames_dir=fd)
        extra = {}
    (fd / "run.json").write_text(json.dumps({"seed": args.seed, "hs_real_m": args.hs_model * K_LEN, "connected": ok,
                                             "t_connect_s": t_ins, "frames": meta, **extra}, ensure_ascii=False))
    print(f"seed {args.seed}: {'연결' if ok else '실패'} {t_ins}, 프레임 {len(meta)}장 → {fd}")
sys.stdout.flush()
app.close()
