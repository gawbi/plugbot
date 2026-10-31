"""Isaac Sim 5.1용 자산 만들기(이 맥에서 실행, GPU 불필요).

실행: .venv/bin/python isaac/build_assets.py
결과(배치마다 하나씩, side가 본선):
- isaac/so101/so101_plugbot_side.urdf, so101_plugbot.urdf(top)
  공식 SO-101 URDF(so101_new_calib) + 그리퍼에 고정한 플러그(side: L자, top: 일자) + 손목 카메라.
  손목 카메라 거치대 메시, 충돌 상자, 카메라 자리는 MuJoCo 모델(Menagerie camera_mount, wrist_cam)과 같다.
  (MuJoCo 모델과 기구학이 같음을 urdf_fk.py로 확인: gripper_link = MuJoCo gripper, 오차 0.004 mm)
- isaac/scene/plugbot_scene_side.usda, plugbot_scene.usda(top)
  부두, 바다, 선박(키네마틱 강체) + 소켓(보이는 메시 + 볼록 충돌 조각 16개), side는 현측에 세운 판.
  카메라: Webcam(앞 카메라, 실물과 같은 자리), ShowCam(발표 영상용). 좌표와 치수는 plugbot/scene.py(MuJoCo)와 같다.
"""

import shutil
import sys
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from plugbot.scene import (CAMERAS, CAMERAS_SIDE, HULL_HALF, L_BODY, L_REACH, L_TIP, N_SEG, PIN_AXIS_L,  # noqa: E402
                           PIN_R, PLUG_TIP, SHIP_HOME, SHIP_HOME_SIDE, SIDE_PANEL_T, SIDE_QUAT, SOCKET_H,
                           SOCKET_ON_SHIP, SOCKET_SIDE_ON_SHIP, SO101_XML, socket_on_ship_frame)

ISAAC = ROOT / "isaac"
CAD = ROOT / "cad/out"
MENAGERIE = SO101_XML.parent / "assets"
SUFFIX = {"top": "", "side": "_side"}
SHOW = {"top": ((0.62, -0.58, 0.36), (0.14, 0.0, -0.01)),  # 발표 영상용 넓은 시점(로봇과 선박이 함께)
        "side": ((0.58, -0.62, 0.40), (0.20, 0.02, 0.0))}
WEBCAM = {"top": ((0.36, -0.20, 0.30), (0.24, 0.0, -0.03), 60.0),  # 예선 웹캠 자리(real/sim_world.py top)
          "side": tuple(CAMERAS_SIDE["front"])}
APERTURE = 20.955  # USD 카메라 가로 조리개(mm). 초점거리 = 조리개 / 2 / tan(수평 화각 / 2)


def _rpy(quat_wxyz):
    w, x, y, z = quat_wxyz
    return Rotation.from_quat([x, y, z, w]).as_euler("xyz")  # URDF rpy(고정축 X, Y, Z 순)


def _fmt(v):
    return " ".join(f"{float(a):.6g}" for a in v)


def wrist_parts():
    """MuJoCo 모델의 손목 카메라 거치대(메시, 충돌 상자)와 카메라 자세(그리퍼 좌표계). camera_mount 몸체 = gripper."""
    m = mujoco.MjModel.from_xml_path(str(SO101_XML))
    cam = m.camera("wrist_cam")
    box = m.geom("camera_box1")
    fovy = float(m.cam_fovy[cam.id])
    shutil.copy2(MENAGERIE / "wrist_roll_follower_so101_camera_mount.stl",
                 ISAAC / "so101/assets/wrist_roll_follower_so101_camera_mount.stl")
    return f"""
  <!-- 손목 카메라 거치대(MuJoCo Menagerie camera_mount와 같음) -->
  <link name="wrist_camera_mount_link">
    <visual>
      <origin xyz="0 -0.000218 0.00095" rpy="{np.pi} 0 0"/>
      <geometry><mesh filename="assets/wrist_roll_follower_so101_camera_mount.stl"/></geometry>
      <material name="mount_white"><color rgba="0.9 0.9 0.9 1"/></material>
    </visual>
    <collision>
      <origin xyz="{_fmt(box.pos)}" rpy="0 0 0"/>
      <geometry><box size="{_fmt(2 * box.size)}"/></geometry>
    </collision>
    <inertial><mass value="0.012"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
  </link>
  <joint name="wrist_camera_mount_fixed" type="fixed">
    <parent link="gripper_link"/><child link="wrist_camera_mount_link"/><origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <!-- 손목 카메라 광학 좌표계(MuJoCo wrist_cam과 같음: -z가 보는 방향, +y가 위, 세로 화각 {fovy:.0f}°).
       run_plugbot_isaac.py가 이 링크 아래에 USD 카메라를 만든다 -->
  <link name="wrist_cam_link"/>
  <joint name="wrist_cam_fixed" type="fixed">
    <parent link="gripper_link"/><child link="wrist_cam_link"/>
    <origin xyz="{_fmt(cam.pos)}" rpy="{_fmt(_rpy(cam.quat))}"/>
  </joint>
"""


def plug_link(layout):
    if layout == "top":
        shutil.copy2(CAD / "plug.stl", ISAAC / "so101/assets/plug.stl")
        return f"""
  <!-- PlugBot 일자 플러그: 그리퍼에 고정(MuJoCo 모델과 같다). 핀 끝 = gripper_link 기준 {PLUG_TIP.tolist()} m -->
  <link name="plug_link">
    <visual>
      <origin xyz="{_fmt(PLUG_TIP)}" rpy="0 0 0"/>
      <geometry><mesh filename="assets/plug.stl" scale="0.001 0.001 0.001"/></geometry>
      <material name="plug_blue"><color rgba="0.12 0.30 0.62 1"/></material>
    </visual>
    <collision>
      <origin xyz="{_fmt(PLUG_TIP + [0, 0, 0.020])}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.006" length="0.040"/></geometry>
    </collision>
    <collision>
      <origin xyz="{_fmt(PLUG_TIP + [0, 0, 0.055])}" rpy="0 0 0"/>
      <geometry><box size="0.016 0.018 0.030"/></geometry>
    </collision>
    <inertial><mass value="0.03"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-6"/></inertial>
  </link>""", PLUG_TIP
    # L자 플러그: CAD plug_L 좌표 = 그리퍼 좌표를 L_BODY만큼 옮긴 것(회전 없음). 핀은 그리퍼 -y.
    # 충돌: 몸통 상자 + 핀(원기둥 + 끝 구 = MuJoCo 캡슐)
    shutil.copy2(CAD / "plug_L.stl", ISAAC / "so101/assets/plug_L.stl")
    root = L_BODY + np.array([0.0, 0.0, -0.010]) + PIN_AXIS_L * 0.009
    sphere = L_TIP - PIN_AXIS_L * PIN_R
    cyl_len = float(np.linalg.norm(sphere - root))
    cyl_mid = (root + sphere) / 2
    return f"""
  <!-- PlugBot L자 플러그: 그리퍼에 고정(MuJoCo 모델과 같다). 핀 끝 = gripper_link 기준 {np.round(L_TIP, 4).tolist()} m,
       핀 방향 그리퍼 -y, 몸통 중심에서 {L_REACH * 1000:.0f} mm -->
  <link name="plug_link">
    <visual>
      <origin xyz="{_fmt(L_BODY)}" rpy="0 0 0"/>
      <geometry><mesh filename="assets/plug_L.stl" scale="0.001 0.001 0.001"/></geometry>
      <material name="plug_blue"><color rgba="0.12 0.30 0.62 1"/></material>
    </visual>
    <collision>
      <origin xyz="{_fmt(L_BODY)}" rpy="0 0 0"/>
      <geometry><box size="0.016 0.018 0.030"/></geometry>
    </collision>
    <collision>
      <origin xyz="{_fmt(cyl_mid)}" rpy="{np.pi / 2} 0 0"/>
      <geometry><cylinder radius="{PIN_R}" length="{cyl_len:.6f}"/></geometry>
    </collision>
    <collision>
      <origin xyz="{_fmt(sphere)}" rpy="0 0 0"/>
      <geometry><sphere radius="{PIN_R}"/></geometry>
    </collision>
    <inertial><mass value="0.03"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-6"/></inertial>
  </link>""", L_TIP


def build_urdf(layout):
    src = ISAAC / "so101/so101_new_calib.urdf"
    text = src.read_text(encoding="utf-8")
    assert text.rstrip().endswith("</robot>")
    link, tip = plug_link(layout)
    extra = link + f"""
  <joint name="plug_fixed" type="fixed">
    <parent link="gripper_link"/><child link="plug_link"/><origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="plug_tip"/>
  <joint name="plug_tip_fixed" type="fixed">
    <parent link="plug_link"/><child link="plug_tip"/>
    <origin xyz="{_fmt(tip)}" rpy="0 0 0"/>
  </joint>
""" + wrist_parts() + "</robot>"
    out = ISAAC / f"so101/so101_plugbot{SUFFIX[layout]}.urdf"
    out.write_text(text.rstrip()[: -len("</robot>")] + extra + "\n", encoding="utf-8")
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
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def _look_at_quat(pos, look):
    """USD 카메라는 -Z를 보고 +Y가 위. 월드 Z 위를 기준으로 방향을 만든다."""
    f = np.array(look, dtype=float) - np.array(pos, dtype=float)
    f /= np.linalg.norm(f)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    R = np.stack([x, y, -f], axis=1)
    m = Gf.Matrix3d(*R.T.flatten().tolist())
    return m.ExtractRotation().GetQuat()


def build_scene(layout):
    out = ISAAC / f"scene/plugbot_scene{SUFFIX[layout]}.usda"
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

    # 부두(로봇 받침): MuJoCo quay와 같은 상자(side는 로봇 쪽으로 짧다)
    qx = 0.14 if layout == "top" else 0.11
    _box(stage, "/World/Quay", (2 * qx, 0.60, 0.15), (0.12 - qx, 0, -0.075), (0.62, 0.63, 0.64))
    _box(stage, "/World/Water", (3.0, 3.0, 0.002), (0, 0, -0.101), (0.18, 0.42, 0.62), collision=False)

    # 선박: 키네마틱 강체. 실행 스크립트가 매 스텝 파랑 운동으로 자세를 넣는다
    home = SHIP_HOME if layout == "top" else SHIP_HOME_SIDE
    ship = UsdGeom.Xform.Define(stage, "/World/Ship")
    UsdGeom.Xformable(ship).AddTranslateOp().Set(Gf.Vec3d(*map(float, home)))
    UsdGeom.Xformable(ship).AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
    rb = UsdPhysics.RigidBodyAPI.Apply(ship.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    h = HULL_HALF
    _box(stage, "/World/Ship/Hull", 2 * h, (0, 0, -0.035), (0.86, 0.87, 0.89))
    _box(stage, "/World/Ship/Stripe", 2 * h + [0.002, 0.002, -0.058], (0, 0, -0.050), (0.75, 0.16, 0.14),
         collision=False)
    _box(stage, "/World/Ship/Cabin", (0.10, 0.16, 0.06), (0.07, 0, 0.03), (0.95, 0.95, 0.96))

    sock = UsdGeom.Xform.Define(stage, "/World/Ship/Socket")
    if layout == "top":
        UsdGeom.Xformable(sock).AddTranslateOp().Set(Gf.Vec3d(*map(float, SOCKET_ON_SHIP)))
    else:  # 현측 끝에 세운 판(두께 6 mm, 갑판 위 15 cm) + 판 앞면에서 로봇 쪽을 보는 소켓
        b = SOCKET_SIDE_ON_SHIP
        _box(stage, "/World/Ship/SidePanel", (SIDE_PANEL_T, 0.17, 0.15), (b[0] + SIDE_PANEL_T / 2, 0, 0.075),
             (0.86, 0.87, 0.89))
        UsdGeom.Xformable(sock).AddTranslateOp().Set(Gf.Vec3d(*map(float, b)))
        UsdGeom.Xformable(sock).AddOrientOp().Set(Gf.Quatf(*map(float, SIDE_QUAT)))
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

    wpos, wlook, whfov = WEBCAM[layout]
    spos, slook = SHOW[layout]
    for name, pos, look, focal in (("Webcam", wpos, wlook, APERTURE / 2 / np.tan(np.deg2rad(whfov) / 2)),
                                   ("ShowCam", spos, slook, 24.0)):
        cam = UsdGeom.Camera.Define(stage, f"/World/{name}")
        cam.CreateFocalLengthAttr(float(focal))
        cam.CreateHorizontalApertureAttr(APERTURE)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10))
        UsdGeom.Xformable(cam).AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        UsdGeom.Xformable(cam).AddOrientOp().Set(Gf.Quatf(_look_at_quat(pos, look)))

    stage.GetRootLayer().Save()
    return out


def check_scene(path, layout):
    """만든 USD를 다시 열어 소켓 입구와 축이 MuJoCo 계산(plugbot.scene)과 같은지 확인한다."""
    stage = Usd.Stage.Open(str(path))
    cache = UsdGeom.XformCache()
    M = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Ship/Socket"))
    origin = np.array(M.Transform(Gf.Vec3d(0, 0, 0)))
    ent_usd = np.array(M.Transform(Gf.Vec3d(0, 0, SOCKET_H)))
    axis_usd = (ent_usd - origin) / SOCKET_H
    home = SHIP_HOME if layout == "top" else SHIP_HOME_SIDE
    ent_ref, axis_ref = socket_on_ship_frame(layout, home, np.eye(3))
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "guide"]).ComputeWorldBound(
        stage.GetPrimAtPath("/World/Ship/Socket/Visual")).ComputeAlignedRange().GetSize()
    n_col = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
    kin = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath("/World/Ship")).GetKinematicEnabledAttr().Get()
    print(f"[{layout}] 소켓 입구 차 {np.abs(ent_usd - ent_ref).max() * 1000:.3f} mm, 축 차 {np.abs(axis_usd - axis_ref).max():.4f}, "
          f"소켓 상자 {bb[0] * 1000:.0f} × {bb[1] * 1000:.0f} × {bb[2] * 1000:.0f} mm, 충돌체 {n_col}개, 선박 키네마틱 {kin}")


if __name__ == "__main__":
    for layout in ("side", "top"):
        print(build_urdf(layout))
        p = build_scene(layout)
        print(p)
        check_scene(p, layout)
