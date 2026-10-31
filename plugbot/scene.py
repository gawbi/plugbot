"""SO-101 + 플러그 + (선박 갑판 소켓 | 자동차 측면 충전구) MuJoCo 장면.

배치(layout) 두 가지:
- "top": 소켓이 위를 보고(선박 갑판 위, 자동차는 45° 경사) 일자 플러그를 위에서 꽂는다(예선 실험).
- "side": 소켓이 로봇을 마주 보게 세우고(선체 옆면 판, 세운 자동차 충전구) L자 플러그를 옆으로 꽂는다.
  5축 SO-101은 일자 플러그로 수평 삽입을 못 하지만(손목 한계), 핀이 그리퍼 옆으로 나온 L자(충전건 모양)는 된다.

좌표 단위 m. 로봇 베이스는 부두 위 원점. 선박은 mocap 몸체라 파도 운동을 매 스텝 직접 넣는다.
소켓 충돌은 CAD에서 16등분한 볼록 조각으로 계산하고, 화면에는 원본 메시를 그린다.
"""

from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SO101_XML = ROOT / "third_party/robotstudio_so101/so101.xml"  # MuJoCo Menagerie(Apache 2.0)에서 복사
CAD = ROOT / "cad/out"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MM = 0.001

# 플러그(그리퍼 좌표계). 그리퍼 몸체의 -z가 플러그 축, 핀 끝이 가장 아래.
PLUG_TIP = np.array([0.002, 0.0, -0.150])
PIN_R = 6.0 * MM

# L자 플러그(그리퍼 좌표계): 몸통은 일자 플러그와 같은 자리, 핀은 그리퍼 -y(집게 경첩 축 방향)로 나온다.
# 핀 끝은 몸통 중심에서 L_REACH. 49 mm였을 때는 깊이 꽂히는 순간 집게가 깔때기 테두리(반지름 31 mm)에 닿았다.
# 더 길면(75 mm 이상) 대기 자세에서 팔을 더 접어야 해서 손목 카메라 거치대가 어깨에 걸린다
PIN_AXIS_L = np.array([0.0, -1.0, 0.0])
L_BODY = np.array([0.002, 0.0, -0.095])
L_REACH = 0.065
L_TIP = L_BODY + np.array([0.0, 0.0, -0.010]) + PIN_AXIS_L * L_REACH

# 소켓 규격(cad/make_parts.py와 같음). 본선용으로 키움: 포착 반경 19 mm, 반경 여유 2 mm
SOCKET_H = 46.0 * MM  # 소켓 바닥에서 깔때기 입구까지
BORE_TOP = (4.0 + 25.0) * MM  # 보어가 시작하는 높이(원뿔 아래 끝)
BORE_R = 8.0 * MM
N_SEG = 16

# 선박: 몸체 원점 = 갑판 위 선체 중심. 소켓은 부두 쪽 가장자리 근처 갑판 위.
# 갑판은 부두보다 5.5 cm 낮다(썰물 때처럼). SO-101 손목 한계 안에서 수직 삽입이 되는 높이다.
SHIP_HOME = np.array([0.32, 0.0, -0.055])
SOCKET_ON_SHIP = np.array([-0.10, 0.0, 0.0])

# 자동차: 바깥 위쪽으로 45° 기울어진 충전구. 수평 충전구는 5축 SO-101의 손목 한계 밖이다.
CAR_INCLINE = np.deg2rad(45)
CAR_AXIS = np.array([-np.cos(CAR_INCLINE), 0.0, np.sin(CAR_INCLINE)])
CAR_ENTRANCE = np.array([0.27, 0.0, 0.06])

# 세운 소켓(side): 입구가 로봇 정면 30 cm, 베이스보다 3 cm 높은 곳에서 로봇 쪽(-x)을 본다.
# 배는 부두에 옆으로 댄 것으로 보고, 소켓 판을 부두 쪽 현측(선체 끝)에 둔다. 롤 축(선체 중심)은 판에서 16 cm.
# 핀 65 mm일 때 대기(입구 앞 30 mm), 입구, 끝까지 꽂은 자세가 모두 충돌 없이 되는 자리(26~30 cm, 높이 3~5 cm를 비교해 고름).
# 더 가까우면 팔을 접는 대기 자세에서 손목 카메라 거치대가 어깨에 닿는다
SIDE_ENTRANCE = np.array([0.30, 0.0, 0.03])
SIDE_AXIS = np.array([-1.0, 0.0, 0.0])
SIDE_QUAT = np.array([np.cos(-np.pi / 4), 0.0, np.sin(-np.pi / 4), 0.0])  # y축 -90°: 소켓 +z → -x, 탭(+x) → 위
HULL_HALF = np.array([0.16, 0.13, 0.035])  # 선체 상자 반치수
SIDE_PANEL_T = 0.006
SHIP_HOME_SIDE = np.array([SIDE_ENTRANCE[0] + SOCKET_H + SIDE_PANEL_T + HULL_HALF[0], 0.0, SHIP_HOME[2]])
SOCKET_SIDE_ON_SHIP = SIDE_ENTRANCE - SIDE_AXIS * SOCKET_H - SHIP_HOME_SIDE  # 선박 좌표계의 소켓 바닥(플랜지) 중심

# 학습 데이터용 카메라 3대(실물 배치와 같게). 물리 계산에는 영향이 없다.
# - wrist: Menagerie SO-101 모델에 원래 있는 손목 카메라 "wrist_cam"(거치대 부품 포함)을 그대로 쓴다
# - front: 셀카봉의 앞 카메라(C270 또는 RealSense 컬러, 화각 약 60°), 로봇 받침 옆 위. ArUco 추적과 ACT 앞 시점을 겸한다
# - top: 키트 셀카봉 카메라, 로봇 받침 뒤 위(선택. 키트 카메라를 손목으로 옮기면 없음)
CAMERAS = {  # 이름: (위치, 바라보는 점, 수평 화각 deg). 해상도는 모두 640 × 480
    "top": (np.array([-0.10, 0.06, 0.40]), np.array([0.20, 0.0, -0.04]), 110.0),
    "front": (np.array([-0.02, -0.14, 0.38]), np.array([0.22, 0.0, -0.04]), 60.0),
}
# side: 앞 카메라를 로봇 받침 오른쪽 앞 책상에 세운 셀카봉(받침 거치대)에 단다. 받침 옆(위 front 자리)에서는
# 삽입 때 팔이 판 마커를 가려 추적 오차가 3.8 mm까지 커졌고, 이 자리는 대기와 삽입 모두 0.6 mm 이하였다(real 모의 영상).
# 보정 뒤에는 카메라를 움직일 수 없으므로 선박 소켓과 로봇 왼쪽 자동차 충전구(real.geometry.CAR_PORT_ANGLE) 사이를 본다
CAMERAS_SIDE = {**CAMERAS, "front": (np.array([0.06, -0.22, 0.30]), np.array([0.26, 0.10, 0.03]), 60.0)}
CAM_W, CAM_H = 640, 480


def _fovy(hfov):
    return float(np.rad2deg(2 * np.arctan(np.tan(np.deg2rad(hfov) / 2) * CAM_H / CAM_W)))


def _xyaxes(pos, look):
    f = (look - pos) / np.linalg.norm(look - pos)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    return np.concatenate([x, np.cross(x, f)])


def _add_socket(body, pos, quat, prefix):
    body.add_geom(name=f"{prefix}_vis", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="socket_vis",
                  pos=pos, quat=quat, contype=0, conaffinity=0, group=1, rgba=[0.93, 0.55, 0.12, 1])
    for k in range(N_SEG):
        body.add_geom(name=f"{prefix}_wall{k:02d}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"socket_wall_{k:02d}",
                      pos=pos, quat=quat, group=3, friction=[0.3, 0.005, 0.0001])
    body.add_geom(name=f"{prefix}_cap", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="socket_cap",
                  pos=pos, quat=quat, group=3)


class Scene:
    def __init__(self, mode="ship", layout="top"):
        assert mode in ("ship", "car") and layout in ("top", "side")
        self.mode, self.layout = mode, layout
        self.pin_axis = np.array([0.0, 0.0, -1.0]) if layout == "top" else PIN_AXIS_L  # 그리퍼 좌표계의 핀 방향
        self.ship_home = SHIP_HOME if layout == "top" else SHIP_HOME_SIDE
        spec = mujoco.MjSpec.from_file(str(SO101_XML))
        spec.option.timestep = 0.002
        spec.visual.global_.offwidth = 1600
        spec.visual.global_.offheight = 1200
        self._assets(spec)
        self._add_plug(spec)
        self._add_world(spec)
        self._add_cameras(spec)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

        m = self.model
        self.qadr = np.array([m.joint(j).qposadr[0] for j in ARM_JOINTS])
        self.dadr = np.array([m.joint(j).dofadr[0] for j in ARM_JOINTS])
        self.qlo = np.array([m.joint(j).range[0] for j in ARM_JOINTS])
        self.qhi = np.array([m.joint(j).range[1] for j in ARM_JOINTS])
        self.act = np.array([m.actuator(j).id for j in ARM_JOINTS])
        self.grip_act = m.actuator("gripper").id
        self.gripper_body = m.body("gripper").id
        self.tip_site = m.site("plug_tip").id
        self.plug_geoms = {m.geom(n).id for n in ("plug_pin", "plug_body")}
        base_root = m.body("base").id
        self.robot_geoms = {g for g in range(m.ngeom)
                            if m.body_rootid[m.geom_bodyid[g]] == base_root and m.geom_contype[g]}
        self.socket_geoms = {g for g in range(m.ngeom) if m.geom(g).name.startswith("sock_") and m.geom_contype[g]}
        self.env_geoms = {g for g in range(m.ngeom)
                          if m.body_rootid[m.geom_bodyid[g]] != base_root and m.geom_contype[g]}
        if mode == "ship":
            self.ship_body = m.body("ship").id
            self.mocap = m.body_mocapid[self.ship_body]
        self.home = None  # 시행 시작 자세. side는 plugbot.control.home_pose가 구해 넣는다

    def _assets(self, spec):
        spec.add_mesh(name="socket_vis", file=str(CAD / "socket.stl"), scale=[MM] * 3)
        spec.add_mesh(name="plug_vis", file=str(CAD / "plug.stl"), scale=[MM] * 3)
        for k in range(N_SEG):
            spec.add_mesh(name=f"socket_wall_{k:02d}", file=str(CAD / f"socket_wall_{k:02d}.stl"), scale=[MM] * 3)
        spec.add_mesh(name="socket_cap", file=str(CAD / "socket_cap.stl"), scale=[MM] * 3)
        spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                         rgb1=[1, 1, 1], rgb2=[0.84, 0.88, 0.93], width=256, height=256)
        spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                         rgb1=[0.62, 0.63, 0.64], rgb2=[0.58, 0.59, 0.60], width=64, height=64)
        spec.add_material(name="concrete", textures=["", "grid"], texrepeat=[8, 8])

    def _add_plug(self, spec):
        g = spec.body("gripper")
        if self.layout == "side":  # L자 플러그: 몸통(박스) + 옆으로 나온 핀(캡슐)
            blue = [0.12, 0.30, 0.62, 1]
            root = L_BODY + np.array([0.0, 0.0, -0.010]) + PIN_AXIS_L * 0.009
            g.add_geom(name="plug_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.008, 0.009, 0.015], pos=L_BODY,
                       group=1, mass=0.01, rgba=blue)
            g.add_geom(name="plug_pin", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[PIN_R, 0, 0],
                       fromto=np.concatenate([root, L_TIP - PIN_AXIS_L * PIN_R]), group=1, mass=0.02,
                       friction=[0.3, 0.005, 0.0001], rgba=blue)
            g.add_site(name="plug_tip", pos=L_TIP, size=[0.002, 0, 0], group=3)
            return
        g.add_geom(name="plug_vis", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="plug_vis", pos=PLUG_TIP,
                   contype=0, conaffinity=0, group=1, rgba=[0.12, 0.30, 0.62, 1], mass=0)
        # 충돌: 핀은 캡슐(둥근 끝이 모따기 역할), 몸통은 박스
        g.add_geom(name="plug_pin", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[PIN_R, 0, 0],
                   fromto=np.concatenate([PLUG_TIP + [0, 0, PIN_R], PLUG_TIP + [0, 0, 0.040]]),
                   group=3, mass=0.02, friction=[0.3, 0.005, 0.0001])
        g.add_geom(name="plug_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.008, 0.009, 0.015],
                   pos=PLUG_TIP + [0, 0, 0.055], group=3, mass=0.01)
        g.add_site(name="plug_tip", pos=PLUG_TIP, size=[0.002, 0, 0], group=3)

    def _add_cameras(self, spec):
        for name, (pos, look, hfov) in (CAMERAS if self.layout == "top" else CAMERAS_SIDE).items():
            spec.worldbody.add_camera(name=name, pos=pos, xyaxes=_xyaxes(pos, look), fovy=_fovy(hfov))

    def _add_world(self, spec):
        w = spec.worldbody
        w.add_light(pos=[0.2, -0.4, 1.0], dir=[-0.1, 0.4, -1], diffuse=[0.8, 0.8, 0.8], specular=[0.15, 0.15, 0.15])
        w.add_light(pos=[0.8, 0.6, 0.8], dir=[-0.6, -0.5, -0.6], diffuse=[0.35, 0.35, 0.35])
        qx = 0.14 if self.layout == "top" else 0.11  # side: 부두 끝을 로봇 쪽으로(실물은 13 cm 받침뿐)
        w.add_geom(name="quay", type=mujoco.mjtGeom.mjGEOM_BOX, size=[qx, 0.30, 0.075], pos=[0.12 - qx, 0, -0.075],
                   material="concrete")
        if self.mode == "ship":
            w.add_geom(name="water", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], pos=[0, 0, -0.10],
                       rgba=[0.18, 0.42, 0.62, 1], contype=0, conaffinity=0)
            ship = w.add_body(name="ship", pos=self.ship_home, mocap=True)
            ship.add_geom(name="hull", type=mujoco.mjtGeom.mjGEOM_BOX, size=HULL_HALF, pos=[0, 0, -0.035],
                          rgba=[0.86, 0.87, 0.89, 1])
            ship.add_geom(name="hull_stripe", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.161, 0.131, 0.006],
                          pos=[0, 0, -0.050], rgba=[0.75, 0.16, 0.14, 1], contype=0, conaffinity=0)
            ship.add_geom(name="cabin", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.08, 0.03], pos=[0.07, 0, 0.03],
                          rgba=[0.95, 0.95, 0.96, 1])
            if self.layout == "top":
                _add_socket(ship, SOCKET_ON_SHIP, [1, 0, 0, 0], "sock")
            else:  # 현측 끝에 세운 판(두께 6 mm, 갑판 위 15 cm) + 판 앞면의 소켓
                b = SOCKET_SIDE_ON_SHIP
                ship.add_geom(name="side_panel", type=mujoco.mjtGeom.mjGEOM_BOX, size=[SIDE_PANEL_T / 2, 0.085, 0.075],
                              pos=[b[0] + SIDE_PANEL_T / 2, 0, 0.075], rgba=[0.86, 0.87, 0.89, 1])
                _add_socket(ship, b, SIDE_QUAT, "sock")
        elif self.layout == "side":
            w.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], pos=[0, 0, -0.15],
                       material="concrete", contype=0, conaffinity=0)
            car = w.add_body(name="car", pos=[0, 0, 0])
            base = SIDE_ENTRANCE - SIDE_AXIS * SOCKET_H
            car.add_geom(name="car_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.12, 0.22, 0.12],
                         pos=[base[0] + 0.12, 0, -0.03], rgba=[0.20, 0.33, 0.52, 1])
            _add_socket(car, base, SIDE_QUAT, "sock")
        else:
            w.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], pos=[0, 0, -0.15],
                       material="concrete", contype=0, conaffinity=0)
            car = w.add_body(name="car", pos=[0, 0, 0])
            base = CAR_ENTRANCE - CAR_AXIS * SOCKET_H
            q = np.zeros(4)
            mujoco.mju_axisAngle2Quat(q, np.array([0, 1.0, 0]), -CAR_INCLINE)  # 소켓 +z → CAR_AXIS
            # 차체와 충전구가 붙은 경사면
            car.add_geom(name="car_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.12, 0.22, 0.09],
                         pos=[base[0] + 0.14, 0, -0.06], rgba=[0.20, 0.33, 0.52, 1])
            car.add_geom(name="car_panel", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.045, 0.06, 0.004],
                         pos=base - CAR_AXIS * 0.004, quat=q, rgba=[0.16, 0.27, 0.44, 1])
            _add_socket(car, base, q, "sock")

    # 선박 운동: 갑판 기준 변위(dx, dy, dz)와 자세(x축 회전, y축 회전). 무엇이 롤인지는 waves.ShipMotion.berth
    def set_ship(self, disp, rx, ry):
        q = np.zeros(4)
        mujoco.mju_euler2Quat(q, np.array([rx, ry, 0.0]), "xyz")
        self.data.mocap_pos[self.mocap] = self.ship_home + disp
        self.data.mocap_quat[self.mocap] = q

    def socket_frame(self):
        """소켓 입구 중심(월드)과 소켓 축(입구 바깥 방향, 단위 벡터)."""
        if self.mode == "ship":
            R = self.data.xmat[self.ship_body].reshape(3, 3)
            if self.layout == "top":
                base, axis = self.data.xpos[self.ship_body] + R @ SOCKET_ON_SHIP, R[:, 2]
            else:
                base, axis = self.data.xpos[self.ship_body] + R @ SOCKET_SIDE_ON_SHIP, R @ SIDE_AXIS
        elif self.layout == "side":
            return SIDE_ENTRANCE.copy(), SIDE_AXIS.copy()
        else:
            return CAR_ENTRANCE.copy(), CAR_AXIS.copy()
        return base + axis * SOCKET_H, axis
