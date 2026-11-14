"""포토박스 트윈 2단계(Isaac Sim 5.1): 실물 정지 소켓 시범 궤적을 트윈 안 SO-101로 다시 그려 앞, 손목 카메라를 렌더한다.

트윈: 회색 천 포토박스, 검은 바닥판과 세운 판, 흰 소켓과 검은 받침(에피소드마다 실물 꽂힌 자세에서 계산한 자리),
은색 L자 받침, 보라색 팔과 홈 있는 원통 덮개. 물리 없이 관절각을 그대로 넣는 운동학 재생이다.
도메인 무작위화(--rand): 조명 세기와 색, 벽과 판 밝기, 앞 카메라 위치 ±1 cm와 방향 ±2도, 팔 색조.

실행(미리보기): LD_LIBRARY_PATH=~/isaacsim_compat/lib OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim_env/bin/python isaac/twin/run_twin.py --preview 54
실행(렌더):     ... run_twin.py --render [--rand] [--limit N]
결과: data/twin_render/ep_XXX_{front,wrist}.mp4, 미리보기 outputs/twin_preview_*.png
"""

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("--preview", type=int, default=-1, help="이 에피소드의 처음과 끝 장면만 렌더")
ap.add_argument("--render", action="store_true")
ap.add_argument("--rand", action="store_true")
ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--variants", default="n,h", help="n: 보통, h: 가혹 조건")
ap.add_argument("--src", default=str(ROOT / "data/twin_src"))
ap.add_argument("--out", default=str(ROOT / "data/twin_render"))
ap.add_argument("--cam", default="", help="앞 카메라 eye와 target 9개 숫자 ex,ey,ez,tx,ty,tz,fov,plate_z,panel_dx")
ap.add_argument("--wrist", default="0,0,0", help="손목 카메라 추가 회전(도) rx,ry,rz")
ap.add_argument("--try_cams", default="", help="카메라 후보 json [[front dict, wrist list], ...]을 실물 꽂힌 장면 옆에 나란히 그린다")
ap.add_argument("--real", default="", help="--try_cams와 함께: 실물 꽂힌 장면 npz(eps, front, wrist)")
a = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.kit.commands  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

W, H = 640, 480
USD = HERE / "so101_cam.usd"
if not USD.exists():
    ok, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False
    cfg.fix_base = True
    cfg.make_default_prim = True
    cfg.self_collision = False
    cfg.distance_scale = 1.0
    cfg.create_physics_scene = False
    omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(ROOT / "isaac/so101/so101_new_calib_camera.urdf"),
                              import_config=cfg, dest_path=str(USD))

world = World(physics_dt=1 / 60, rendering_dt=1 / 30, stage_units_in_meters=1.0)
stage = get_current_stage()
UsdPhysics.Scene.Get(stage, "/physicsScene") or UsdPhysics.Scene.Define(stage, "/physicsScene")
add_reference_to_stage(str(USD), "/World/SO101")
for p in stage.Traverse():  # 중력 끄기(운동학 재생)
    if p.GetPath().pathString.startswith("/World/SO101") and p.HasAPI(UsdPhysics.RigidBodyAPI):
        p.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)


def mat(name, rgb, rough=0.7, metal=0.0):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    sh = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m, sh


def bind(prim, m):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(m, UsdShade.Tokens.strongerThanDescendants)


def box(path, c, size, m):
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*c))
    x.AddScaleOp().Set(Gf.Vec3d(*size))
    bind(g.GetPrim(), m)
    return g


def cyl(path, r, h, m, axis="Z"):
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(r)
    g.CreateHeightAttr(h)
    g.CreateAxisAttr(axis)
    bind(g.GetPrim(), m)
    return g


MESH = np.load(HERE / "twin_meshes.npz")  # make_twin_meshes.py: 실제 출력 덮개 STL(그리퍼 좌표), 흰 소켓
M = {k: mat(k, *v) for k, v in {"paper": ((0.018, 0.02, 0.018), 0.95), "black": ((0.025, 0.025, 0.03), 0.85),
                                   "white": ((0.93, 0.93, 0.95), 0.35), "silver": ((0.72, 0.73, 0.75), 0.25, 0.9),
                                   "purple": ((0.20, 0.06, 0.58), 0.5), "servo": ((0.05, 0.05, 0.06), 0.4)}.items()}
for _ in range(6):  # 인스턴스를 풀어야 색을 바꿀 수 있다(겹쳐 있으면 여러 번)
    inst = [p for p in stage.Traverse() if p.GetPath().pathString.startswith("/World/SO101") and p.IsInstance()]
    if not inst:
        break
    for p in inst:
        p.SetInstanceable(False)
for p in stage.Traverse():  # 가져온 URDF 재질 중 밝은 색(노란 출력 부품)을 보라색으로
    if p.GetPath().pathString.startswith("/World/SO101") and p.IsA(UsdShade.Shader):
        for inp in UsdShade.Shader(p).GetInputs():
            if inp.GetBaseName() in ("diffuseColor", "diffuse_color_constant", "base_color"):
                v = inp.Get()
                if v is not None and sum(v) > 1.2:
                    inp.Set(Gf.Vec3f(0.20, 0.06, 0.58))
for p in stage.Traverse():  # 출력 부품은 보라색, 서보와 카메라는 검은색(실물과 같게)
    ps = p.GetPath().pathString
    if ps.startswith("/World/SO101") and p.IsA(UsdGeom.Gprim) and "/Cover" not in ps:
        dark = any(k in ps.lower() for k in ("sts3215", "wrist_camera_so101", "waveshare"))
        UsdShade.MaterialBindingAPI.Apply(p).Bind(M["servo" if dark else "purple"][0], UsdShade.Tokens.strongerThanDescendants)


def mesh(path, v, f, m):
    g = UsdGeom.Mesh.Define(stage, path)
    g.CreatePointsAttr([Gf.Vec3f(*map(float, x)) for x in v])
    g.CreateFaceVertexCountsAttr([3] * len(f))
    g.CreateFaceVertexIndicesAttr([int(x) for x in np.asarray(f).ravel()])
    g.CreateSubdivisionSchemeAttr("none")
    bind(g.GetPrim(), m)
    return g


# 그리퍼 덮개: 실제 출력한 STL(그리퍼 좌표 그대로)
glink = next(p for p in stage.Traverse() if p.GetPath().pathString.startswith("/World/SO101") and p.GetName() == "gripper_link")
mesh(str(glink.GetPath().AppendChild("Cover")), MESH["cover_v"], MESH["cover_f"], M["purple"][0])
PIN_TIP = MESH["pin_tip"].astype(float)
INSERT = 0.020  # 꽂힌 자세에서 핀이 소켓 구멍에 들어간 깊이
# 소켓 묶음(에피소드마다 옮긴다). 로컬 x = 판 쪽(꽂는 방향), 앞면 x = 0, z = 위
UsdGeom.Xform.Define(stage, "/World/Socket")  # 먼저 Xform으로 정의해야 옮길 수 있다
mesh("/World/Socket/White", MESH["sock_v"], MESH["sock_f"], M["white"][0])
mesh("/World/Socket/Base", MESH["base_v"], MESH["base_f"], M["black"][0])
PANEL_X = 0.038  # 소켓 앞면에서 판 앞면까지(소켓 30 mm + 받침 8 mm)
box("/World/Socket/Panel", (PANEL_X + 0.004, 0.0, 0.03), (0.008, 0.40, 0.30), M["black"][0])
PLATE_Z = -0.12  # 소켓 중심이 바닥판보다 약 12 cm 위(실물 사진: 소켓 아래로 L자 받침이 다 보인다)
box("/World/Socket/Plate", (PANEL_X - 0.15, 0.0, PLATE_Z - 0.004), (0.32, 0.40, 0.008), M["black"][0])
for k, sy in enumerate((-0.10, 0.10)):  # L자 받침 두 개(소켓 좌우, 판 앞, 발은 로봇 쪽)
    box(f"/World/Socket/BrV{k}", (PANEL_X - 0.003, sy, PLATE_Z + 0.045), (0.003, 0.025, 0.09), M["silver"][0])
    box(f"/World/Socket/BrH{k}", (PANEL_X - 0.033, sy, PLATE_Z + 0.0015), (0.06, 0.025, 0.003), M["silver"][0])
    for h, (bx, bz) in enumerate(((PANEL_X - 0.005, PLATE_Z + 0.07), (PANEL_X - 0.005, PLATE_Z + 0.04))):
        c = cyl(f"/World/Socket/BrB{k}{h}", 0.0035, 0.004, M["black"][0], "X")
        UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(bx, sy, bz))
for k, (bx, by) in enumerate(((-0.26, -0.17), (-0.26, -0.13), (-0.24, -0.17), (-0.26, 0.15), (-0.26, 0.17), (-0.05, -0.18), (-0.03, -0.18))):
    c = cyl(f"/World/Socket/Bolt{k}", 0.004, 0.012, M["black"][0])  # 바닥판 볼트
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(PANEL_X + bx, by, PLATE_Z + 0.006))
# 포토박스(검은 종이)
box("/World/Socket/Back", (PANEL_X + 0.06, 0.0, 0.2), (0.01, 0.70, 0.70), M["paper"][0])
box("/World/Socket/WallR", (PANEL_X - 0.45, -0.33, 0.2), (1.10, 0.01, 0.80), M["paper"][0])
box("/World/Socket/WallL", (PANEL_X - 0.45, 0.33, 0.2), (1.10, 0.01, 0.80), M["paper"][0])
box("/World/Socket/Floor", (PANEL_X - 0.45, 0.0, PLATE_Z - 0.03), (1.10, 0.70, 0.01), M["paper"][0])
box("/World/Socket/Top", (PANEL_X - 0.45, 0.0, 0.55), (1.10, 0.70, 0.01), M["paper"][0])
sock_root = stage.GetPrimAtPath("/World/Socket")
UsdGeom.Xformable(sock_root).AddTransformOp()
# 조명(포토박스 위 LED: 밝고 고르게)
dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
dome.CreateIntensityAttr(250)
rect = UsdLux.RectLight.Define(stage, "/World/Top")
rect.CreateIntensityAttr(14000)
rect.CreateWidthAttr(0.5)
rect.CreateHeightAttr(0.5)
rect_x = UsdGeom.Xformable(rect).AddTranslateOp()
# 카메라
UsdGeom.Camera.Define(stage, "/World/FrontCam")
wlink = next(p for p in stage.Traverse() if p.GetPath().pathString.startswith("/World/SO101") and p.GetName() == "wrist_camera_link")


def cam_lens(prim, vfov):
    c = UsdGeom.Camera(prim)
    c.CreateHorizontalApertureAttr(20.955)
    c.CreateVerticalApertureAttr(20.955 * H / W)
    c.CreateFocalLengthAttr(float(20.955 * H / W / 2 / np.tan(np.deg2rad(vfov) / 2)))
    c.CreateClippingRangeAttr(Gf.Vec2f(0.005, 10))


# 손목 카메라: gripper_link 아래, 실물 카메라 마운트 자리(URDF wrist_camera_link 위치)에서 턱 방향(-z)을 본다. 위치는 첫 자세 뒤에 정한다
ap2 = __import__("os").environ.get("TWIN_WRIST", "")  # 손목 카메라 후보 "tilt,vfov,z,h;..." (맞출 때만)
WRIST = [tuple(map(float, c.split(","))) for c in ap2.split(";")] if ap2 else [(10, 44, -0.05, 0.058)]  # (아래로 기울기 도, 세로 화각, 카메라 z, 축에서 높이[, 옆 m, 돌림 도])
CALIB_F = HERE / "calib_cam.json"  # --calib으로 실물 꽂힌 장면에 맞춘 카메라 값(있으면 기본값으로 쓴다)
CALIB = json.loads(CALIB_F.read_text()) if CALIB_F.exists() and not a.try_cams and not __import__("os").environ.get("TWIN_NOCALIB") else None
if CALIB and not ap2:
    WRIST = [tuple(CALIB["wrist"])]
WRIST = [tuple(w) + (0.0, 0.0)[len(w) - 4:] if len(w) < 6 else tuple(w) for w in WRIST]
wcams = []
for k, (tilt, vf, cz, ch, _lx, _rl) in enumerate(WRIST):
    c = UsdGeom.Camera.Define(stage, glink.GetPath().AppendChild(f"WristCam{k}"))
    cam_lens(c.GetPrim(), vf)
    wcams.append((c, tilt, (cz, ch)))

root = next(p.GetPath().pathString for p in stage.Traverse()
            if p.GetPath().pathString.startswith("/World/SO101") and p.HasAPI(UsdPhysics.ArticulationRootAPI))
robot = world.scene.add(SingleArticulation(prim_path=root, name="so101"))
cams = {"front": Camera(prim_path="/World/FrontCam", resolution=(W, H))}
if a.preview >= 0:
    UsdGeom.Camera.Define(stage, "/World/Overview")
    cams["over"] = Camera(prim_path="/World/Overview", resolution=(W, H))
for k, (c, _, _r) in enumerate(wcams):
    cams["wrist" if k == 0 else f"wv{k}"] = Camera(prim_path=str(c.GetPath()), resolution=(W, H))
world.reset()
for c in cams.values():
    c.initialize()
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
dof = np.array([robot.get_dof_index(n) for n in NAMES])


def world_tf(prim):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def setup_wrist(jit=None):
    """실물 손목 카메라: 덮개 결합 링 위쪽(꽂힌 자세에서 하늘 방향) 약 6 cm, 링 뒤에서 턱 방향(-z)을 본다. 그리퍼에 고정"""
    if "up" not in SOCK:  # 카메라는 그리퍼에 고정: 위쪽은 기준 자세(에피소드 54 끝)에서 한 번만 정한다
        up = np.array(world_tf(glink).GetInverse().TransformDir(Gf.Vec3d(0, 0, 1)))
        up[2] = 0.0
        SOCK["up"] = up / np.linalg.norm(up)
    up = SOCK["up"]
    pass
    yaw = np.degrees(np.arctan2(-up[0], up[1]))
    side_dir = np.cross(up, [0, 0, 1.0])
    for k, (c, tilt, (cz, ch)) in enumerate(wcams):
        t = np.array([PIN_TIP[0], PIN_TIP[1], cz]) + up * ch + side_dir * WRIST[k][4]
        roll = WRIST[k][5]
        if jit is not None:  # 가혹 조건: 위치 ±5 mm, 기울기와 돌림 ±3도, 화각 ±5도
            t = t + jit[:3]
            tilt, roll = tilt + jit[3], roll + jit[4]
            cam_lens(c.GetPrim(), WRIST[k][1] + jit[5])
        r = Gf.Rotation(Gf.Vec3d(1, 0, 0), float(-tilt)) * Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw + roll))
        qq = r.GetQuat()
        cams["wrist" if k == 0 else f"wv{k}"].set_local_pose(translation=t, orientation=np.array([qq.GetReal(), *qq.GetImaginary()]), camera_axes="usd")


def set_front(eye, tgt, fov):
    v = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*tgt), Gf.Vec3d(0, 0, 1))
    q = v.GetInverse().ExtractRotationQuat()
    cams["front"].set_world_pose(position=np.array(eye, float), orientation=np.array([q.GetReal(), *q.GetImaginary()]), camera_axes="usd")
    cam_lens(stage.GetPrimAtPath("/World/FrontCam"), fov * H / W * 1.3)


SOCK = {"ent": np.array([0.42, 0.03, 0.13])}


def place_socket(ent, axis):
    """소켓 앞면을 입구에, 로컬 +x를 꽂는 방향으로. 판, 받침, 바닥판, 포토박스가 같이 움직인다(위는 +z 유지)"""
    x = -np.asarray(SOCK.get("axis_ref", axis), float)  # 판과 카메라는 실물처럼 고정: 방향은 기준 에피소드(54) 하나로
    x[2] = 0.0  # 판은 세워 둔다(실물: 수직 판)
    x /= np.linalg.norm(x)
    z = np.array([0, 0, 1.0])
    y = np.cross(z, x)
    stage.GetPrimAtPath("/World/Socket").GetAttribute("xformOp:transform").Set(Gf.Matrix4d(*x, 0, *y, 0, *z, 0, *ent, 1))
    rect_x.Set(Gf.Vec3d(float(ent[0] - 0.15), float(ent[1]), float(ent[2] + 0.30)))
    SOCK["ent"] = np.asarray(ent, float)
    SOCK["axis"] = np.asarray(axis, float)


# 앞 카메라: 왼쪽(+y) 벽 위쪽, 판 가까이에서 로봇 쪽을 비스듬히 내려다본다(소켓 기준 상대 위치)
FC = dict(dx=0.0, dy=0.20, dz=0.025, tx=-0.12, ty=-0.03, tz=-0.01, fov=62.0)
if CALIB:
    FC.update(CALIB["front"])


def front_from_socket(fc, jit=None):
    e = SOCK["ent"]
    j = np.zeros(7) if jit is None else jit
    set_front((e[0] + fc["dx"] + j[0], e[1] + fc["dy"] + j[1], e[2] + fc["dz"] + j[2]),
              (e[0] + fc["tx"] + j[3], e[1] + fc["ty"] + j[4], e[2] + fc["tz"] + j[5]), fc["fov"] + j[6])


side = UsdLux.SphereLight.Define(stage, "/World/Side")  # 가혹: 옆빛, 야간 점광원
side.CreateRadiusAttr(0.02)
side.CreateIntensityAttr(0.0)
side_x = UsdGeom.Xformable(side).AddTranslateOp()
HARSH = ["dark", "bright", "side", "warm", "cool", "night", "fabric", "foil"]


def randomize(rng, harsh=False):
    """보통: 조명, 벽 밝기, 팔 색조, 앞 카메라를 조금. 가혹: 아래 모드 1~2개 + 카메라를 크게 흔든다. 돌려주는 값은 고른 모드"""
    dome.GetIntensityAttr().Set(float(rng.uniform(150, 450)))
    rect.GetIntensityAttr().Set(float(rng.uniform(9000, 20000)))
    rect.CreateColorAttr(Gf.Vec3f(*rng.uniform(0.9, 1.0, 3)))
    side.GetIntensityAttr().Set(0.0)
    g = rng.uniform(0.012, 0.035)
    sh = M["paper"][1]
    sh.GetInput("diffuseColor").Set(Gf.Vec3f(g, g, g * rng.uniform(1.0, 1.06)))
    sh.GetInput("metallic").Set(0.0)
    sh.GetInput("roughness").Set(0.95)
    pu = np.array([0.20, 0.06, 0.58]) * rng.uniform(0.85, 1.15, 3)
    M["purple"][1].GetInput("diffuseColor").Set(Gf.Vec3f(*np.clip(pu, 0, 1)))
    modes = []
    if harsh:
        modes = list(rng.choice(HARSH, size=int(rng.integers(1, 3)), replace=False))
        e = SOCK["ent"]
        for m in modes:
            if m == "dark":
                dome.GetIntensityAttr().Set(float(rng.uniform(20, 70))); rect.GetIntensityAttr().Set(float(rng.uniform(1500, 3500)))
            elif m == "bright":
                dome.GetIntensityAttr().Set(float(rng.uniform(900, 1600))); rect.GetIntensityAttr().Set(float(rng.uniform(35000, 60000)))
            elif m == "side":
                side_x.Set(Gf.Vec3d(float(e[0] - 0.1), float(e[1] + rng.choice([-0.25, 0.25])), float(e[2] + 0.1)))
                side.GetIntensityAttr().Set(float(rng.uniform(30000, 80000)))
            elif m == "warm":
                rect.CreateColorAttr(Gf.Vec3f(1.0, 0.78, 0.55))
            elif m == "cool":
                rect.CreateColorAttr(Gf.Vec3f(0.7, 0.82, 1.0))
            elif m == "night":
                dome.GetIntensityAttr().Set(5.0); rect.GetIntensityAttr().Set(0.0)
                side_x.Set(Gf.Vec3d(float(e[0] - 0.25), float(e[1]), float(e[2] + 0.25)))
                side.GetIntensityAttr().Set(float(rng.uniform(20000, 50000)))
            elif m == "fabric":
                v = rng.uniform(0.25, 0.4); sh.GetInput("diffuseColor").Set(Gf.Vec3f(v, v, v))
            elif m == "foil":
                sh.GetInput("diffuseColor").Set(Gf.Vec3f(0.6, 0.6, 0.62)); sh.GetInput("metallic").Set(0.9); sh.GetInput("roughness").Set(0.35)
        front_from_socket(FC, np.r_[rng.uniform(-0.02, 0.02, 3), rng.uniform(-0.04, 0.04, 3), rng.uniform(-5, 5)])
        setup_wrist(np.r_[rng.uniform(-0.005, 0.005, 3), rng.uniform(-3, 3, 2), rng.uniform(-5, 5)])
    else:
        front_from_socket(FC, np.r_[rng.normal(0, 0.008, 3), rng.normal(0, 0.015, 3), rng.uniform(-3, 3)])
        setup_wrist(np.r_[rng.normal(0, 0.0015, 3), rng.normal(0, 1.0, 2), rng.uniform(-1.5, 1.5)])
    return modes


def show(q, renders=1):
    robot.set_joint_positions(q, joint_indices=dof)
    robot.apply_action(ArticulationAction(joint_positions=q, joint_indices=dof))
    world.step(render=True)
    for _ in range(renders):  # 카메라 버퍼가 늦게 따라오지 않게 더 그린다
        world.render()


def socket_from_end(q_end):
    """Isaac 팔로 꽂힌 마지막 자세를 만들고, 덮개 핀 끝에서 INSERT만큼 뒤를 소켓 입구로 둔다"""
    for _ in range(3):
        show(q_end)
    G = world_tf(glink)
    tip = np.array(G.Transform(Gf.Vec3d(*PIN_TIP)))
    jaw = np.array(G.TransformDir(Gf.Vec3d(0, 0, -1)))
    jaw /= np.linalg.norm(jaw)
    place_socket(tip - jaw * INSERT, -jaw)


def grab():
    out = {}
    for k, c in cams.items():
        rgba = c.get_rgba()
        out[k] = None if rgba is None or rgba.size == 0 else np.ascontiguousarray(rgba[..., :3]).astype(np.uint8)
    return out


_e = np.load(ROOT / "data/twin_src/ep_054.npz")  # 판과 카메라 기준은 늘 원본 54번
for _ in range(3):
    show(_e["q"][-1])
setup_wrist()
socket_from_end(_e["q"][-1])
SOCK["axis_ref"] = SOCK["axis"].copy()
src = Path(a.src)
if a.try_cams:
    # 카메라 후보를 실물 꽂힌 장면 옆에 나란히 그린다. 사람이 보고 골라 calib_cam.json에 적는다.
    # (겹침 점수로 고르는 자동 탐색은 2026-11-10 두 번 모두 덮개 안이나 옆면을 보는 값을 골라 지웠다)
    import imageio.v2 as iio
    R = np.load(a.real)
    cands = [(dict(FC, **f), tuple(w)) for f, w in json.loads(Path(a.try_cams).read_text())]
    rows = []
    for ri in range(len(R["eps"]))[-2:]:  # 실물 장면 마지막 두 개
        e = np.load(ROOT / f"data/twin_src/ep_{int(R['eps'][ri]):03d}.npz")
        socket_from_end(e["q"][-1])
        for _ in range(3):
            show(e["q"][-1])
        for fc, wr in cands:
            front_from_socket(fc)
            WRIST[0] = wr
            cam_lens(wcams[0][0].GetPrim(), wr[1])
            wcams[0] = (wcams[0][0], wr[0], (wr[2], wr[3]))
            setup_wrist()
            for _ in range(6):  # 카메라를 바꾼 뒤 화면이 늦게 따라온다
                world.render()
            g = grab()
            rows.append(np.concatenate([R["wrist"][ri], g["wrist"], R["front"][ri], g["front"]], 1)[::2, ::2])
    iio.imwrite(ROOT / "outputs/twin_try_cams.png", np.concatenate(rows, 0))
    print("저장 outputs/twin_try_cams.png (행: 후보, 열: 실물 손목 | 트윈 손목 | 실물 앞 | 트윈 앞)")
elif a.preview >= 0:
    import imageio.v2 as iio
    e = np.load(src / f"ep_{a.preview:03d}.npz")
    socket_from_end(e["q"][-1])
    front_from_socket(FC)
    v = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0.15, -0.55, 0.55), Gf.Vec3d(0.35, 0.0, 0.08), Gf.Vec3d(0, 0, 1))
    qo = v.GetInverse().ExtractRotationQuat()
    cams["over"].set_world_pose(position=np.array([0.15, -0.55, 0.55]), orientation=np.array([qo.GetReal(), *qo.GetImaginary()]), camera_axes="usd")
    for nm in ("/World/Socket/White", "/World/Socket/Panel", "/World/Socket/Back", "/World/Socket/WallL", "/World/Socket/Floor"):
        bb = UsdGeom.Imageable(stage.GetPrimAtPath(nm)).ComputeWorldBound(Usd.TimeCode.Default(), "default").ComputeAlignedRange()
        print("DBG", nm, list(np.round(bb.GetMin(), 3)), list(np.round(bb.GetMax(), 3)))
    print("DBG ent", SOCK["ent"].round(3))
    for which, i in (("start", 0), ("end", len(e["q"]) - 1)):
        for _ in range(8):
            show(e["q"][i], 2)
        g = grab()
        for k, im in g.items():
            if im is not None:
                iio.imwrite(ROOT / f"outputs/twin_preview_{which}_{k}.png", im)
elif a.render:
    import imageio.v2 as iio
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    files = sorted(src.glob("ep_*.npz"))[: a.limit or None]
    import time
    t0 = time.time()
    jobs = [(f, v) for f in files for v in a.variants.split(",")]
    for n, (f, v) in enumerate(jobs):
        stem = f"{f.stem}_{v}"
        if (out / f"{stem}_wrist.mp4").exists():
            continue
        e = np.load(f)
        rng = np.random.default_rng(int(f.stem[3:]) * 10 + (1 if v == "h" else 0))
        socket_from_end(e["q"][-1])
        modes = randomize(rng, harsh=(v == "h")) if a.rand else (front_from_socket(FC) or [])
        for _ in range(3):
            show(e["q"][0])
        ws = {k: iio.get_writer(out / f"{stem}_{k}.mp4", fps=30, codec="libx264", quality=8, macro_block_size=8) for k in cams}
        for q in e["q"]:
            show(q)
            g = grab()
            for k, im in g.items():
                ws[k].append_data(im if im is not None else np.zeros((H, W, 3), np.uint8))
        for w in ws.values():
            w.close()
        (out / f"{stem}.json").write_text(json.dumps({"variant": v, "modes": [str(m) for m in modes]}))
        print(f"{n + 1}/{len(jobs)} {stem} {len(e['q'])}장면 {modes}, 누적 {time.time() - t0:.0f}s", flush=True)
app.close()
