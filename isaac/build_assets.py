"""Isaac Sim 5.1용 자산 만들기(이 맥에서 실행, GPU 불필요).

실행: .venv/bin/python isaac/build_assets.py
결과:
- isaac/so101/so101_plugbot.urdf : 공식 SO-101 URDF(so101_new_calib) + 그리퍼에 고정한 플러그
  (MuJoCo 모델과 기구학이 같음을 urdf_fk.py로 확인: gripper_link = MuJoCo gripper, 오차 0.004 mm)
- isaac/scene/plugbot_scene.usda : 부두, 바다, 선박(키네마틱 강체) + 소켓(보이는 메시 + 볼록 충돌 조각 16개),
  웹캠 카메라(Webcam), 발표 영상용 카메라(ShowCam), 조명. 좌표와 치수는 plugbot/scene.py(MuJoCo)와 같다. 단위 m, Z 위.
"""

import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from plugbot.scene import N_SEG, PLUG_TIP, SHIP_HOME, SOCKET_H, SOCKET_ON_SHIP  # noqa: E402

ISAAC = ROOT / "isaac"
CAD = ROOT / "cad/out"
CAM_POS, CAM_LOOK = (0.36, -0.20, 0.30), (0.24, 0.0, -0.03)  # real/sim_world.py와 같은 웹캠 위치
SHOW_POS, SHOW_LOOK = (0.62, -0.58, 0.36), (0.14, 0.0, -0.01)  # 발표 영상용 넓은 시점(로봇과 선박이 함께)

PLUG_LINK = f"""
  <!-- PlugBot 플러그: 그리퍼에 고정(MuJoCo 모델과 같다). 핀 끝 = gripper_link 기준 {PLUG_TIP.tolist()} m -->
  <link name="plug_link">
    <visual>
      <origin xyz="{PLUG_TIP[0]} {PLUG_TIP[1]} {PLUG_TIP[2]}" rpy="0 0 0"/>
      <geometry><mesh filename="assets/plug.stl" scale="0.001 0.001 0.001"/></geometry>
      <material name="plug_blue"><color rgba="0.12 0.30 0.62 1"/></material>
    </visual>
    <collision>
      <origin xyz="{PLUG_TIP[0]} {PLUG_TIP[1]} {PLUG_TIP[2] + 0.020}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.006" length="0.040"/></geometry>
    </collision>
    <collision>
      <origin xyz="{PLUG_TIP[0]} {PLUG_TIP[1]} {PLUG_TIP[2] + 0.055}" rpy="0 0 0"/>
      <geometry><box size="0.016 0.018 0.030"/></geometry>
    </collision>
    <inertial><mass value="0.03"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-6"/></inertial>
  </link>
  <joint name="plug_fixed" type="fixed">
    <parent link="gripper_link"/><child link="plug_link"/><origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="plug_tip"/>
  <joint name="plug_tip_fixed" type="fixed">
    <parent link="plug_link"/><child link="plug_tip"/>
    <origin xyz="{PLUG_TIP[0]} {PLUG_TIP[1]} {PLUG_TIP[2]}" rpy="0 0 0"/>
  </joint>
</robot>"""


def build_urdf():
    src = ISAAC / "so101/so101_new_calib.urdf"
    shutil.copy2(CAD / "plug.stl", ISAAC / "so101/assets/plug.stl")
    text = src.read_text(encoding="utf-8")
    assert text.rstrip().endswith("</robot>")
    out = ISAAC / "so101/so101_plugbot.urdf"
    out.write_text(text.rstrip()[: -len("</robot>")] + PLUG_LINK + "\n", encoding="utf-8")
    return out


def _mesh(stage, path, stl, color=None, collision=False, visible=True):
    m = trimesh.load(str(stl))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*(v * 0.001)) for v in m.vertices])
    mesh.CreateFaceVertexCountsAttr([3] * len(m.faces))
    mesh.CreateFaceVertexIndicesAttr(m.faces.flatten().tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    if color is not None:
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    if not visible:
        mesh.CreatePurposeAttr("guide")  # 충돌 전용(화면에 안 그림)
    return mesh


def _box(stage, path, size, center, color, collision=True):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def _look_at_quat(pos, look):
    """USD 카메라는 -Z를 보고 +Y가 위. 월드 Z 위를 기준으로 방향을 만든다."""
    f = np.array(look) - np.array(pos)
    f /= np.linalg.norm(f)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    R = np.stack([x, y, -f], axis=1)
    m = Gf.Matrix3d(*R.T.flatten().tolist())
    return m.ExtractRotation().GetQuat()


def build_scene():
    out = ISAAC / "scene/plugbot_scene.usda"
    out.parent.mkdir(exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))

    # 부두(로봇 받침): MuJoCo quay와 같은 상자
    _box(stage, "/World/Quay", (0.28, 0.60, 0.15), (-0.02, 0, -0.075), (0.62, 0.63, 0.64))
    _box(stage, "/World/Water", (3.0, 3.0, 0.002), (0, 0, -0.101), (0.18, 0.42, 0.62), collision=False)

    # 선박: 키네마틱 강체. 실행 스크립트가 매 스텝 파랑 운동으로 자세를 넣는다
    ship = UsdGeom.Xform.Define(stage, "/World/Ship")
    UsdGeom.Xformable(ship).AddTranslateOp().Set(Gf.Vec3d(*SHIP_HOME))
    UsdGeom.Xformable(ship).AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
    rb = UsdPhysics.RigidBodyAPI.Apply(ship.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    _box(stage, "/World/Ship/Hull", (0.32, 0.26, 0.07), (0, 0, -0.035), (0.86, 0.87, 0.89))
    _box(stage, "/World/Ship/Stripe", (0.322, 0.262, 0.012), (0, 0, -0.050), (0.75, 0.16, 0.14), collision=False)
    _box(stage, "/World/Ship/Cabin", (0.10, 0.16, 0.06), (0.07, 0, 0.03), (0.95, 0.95, 0.96))

    sock = UsdGeom.Xform.Define(stage, "/World/Ship/Socket")
    UsdGeom.Xformable(sock).AddTranslateOp().Set(Gf.Vec3d(*SOCKET_ON_SHIP))
    _mesh(stage, "/World/Ship/Socket/Visual", CAD / "socket.stl", color=(0.93, 0.55, 0.12))
    for k in range(N_SEG):
        _mesh(stage, f"/World/Ship/Socket/Wall{k:02d}", CAD / f"socket_wall_{k:02d}.stl", collision=True, visible=False)
    _mesh(stage, "/World/Ship/Socket/Cap", CAD / "socket_cap.stl", collision=True, visible=False)

    # 소켓 표면 마찰(MuJoCo와 같은 0.3)
    mat = UsdShade.Material.Define(stage, "/World/Looks/SocketPhysics")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.3)
    pm.CreateDynamicFrictionAttr(0.3)
    pm.CreateRestitutionAttr(0.0)
    for k in range(N_SEG):
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(f"/World/Ship/Socket/Wall{k:02d}")).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    for name, pos, look, focal in (("Webcam", CAM_POS, CAM_LOOK, 18.0), ("ShowCam", SHOW_POS, SHOW_LOOK, 24.0)):
        cam = UsdGeom.Camera.Define(stage, f"/World/{name}")
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10))
        UsdGeom.Xformable(cam).AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdGeom.Xformable(cam).AddOrientOp().Set(Gf.Quatf(_look_at_quat(pos, look)))

    stage.GetRootLayer().Save()
    return out


def check_scene(path):
    """만든 USD를 다시 열어 구조와 치수를 확인한다."""
    stage = Usd.Stage.Open(str(path))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "guide"])
    sock = stage.GetPrimAtPath("/World/Ship/Socket/Visual")
    bb = cache.ComputeWorldBound(sock).ComputeAlignedRange()
    size = bb.GetSize()
    top = bb.GetMax()[2]
    n_col = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
    kin = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath("/World/Ship")).GetKinematicEnabledAttr().Get()
    print(f"소켓 크기 {size[0] * 1000:.1f} × {size[1] * 1000:.1f} × {size[2] * 1000:.1f} mm(기대 110 × 70 × {SOCKET_H * 1000:.1f}), "
          f"입구 높이 {top:.4f} m(기대 {SHIP_HOME[2] + SOCKET_H:.4f}), 충돌체 {n_col}개, 선박 키네마틱 {kin}")


if __name__ == "__main__":
    print(build_urdf())
    p = build_scene()
    print(p)
    check_scene(p)
