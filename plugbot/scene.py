"""SO-101 + 플러그 + (선박 갑판 소켓 | 자동차 측면 충전구) MuJoCo 장면.

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

# 소켓 규격(cad/make_parts.py와 같음)
SOCKET_H = 39.5 * MM  # 소켓 바닥에서 깔때기 입구까지
BORE_TOP = (4.0 + 25.0) * MM  # 보어가 시작하는 높이(원뿔 아래 끝)
BORE_R = 7.5 * MM
N_SEG = 16

# 선박: 몸체 원점 = 갑판 위 선체 중심. 소켓은 부두 쪽 가장자리 근처 갑판 위.
# 갑판은 부두보다 5.5 cm 낮다(썰물 때처럼). SO-101 손목 한계 안에서 수직 삽입이 되는 높이다.
SHIP_HOME = np.array([0.32, 0.0, -0.055])
SOCKET_ON_SHIP = np.array([-0.10, 0.0, 0.0])

# 자동차: 바깥·위쪽으로 45° 기울어진 충전구. 수평 충전구는 5축 SO-101의 손목 한계 밖이다.
CAR_INCLINE = np.deg2rad(45)
CAR_AXIS = np.array([-np.cos(CAR_INCLINE), 0.0, np.sin(CAR_INCLINE)])
CAR_ENTRANCE = np.array([0.27, 0.0, 0.06])


def _add_socket(body, pos, quat, prefix):
    body.add_geom(name=f"{prefix}_vis", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="socket_vis",
                  pos=pos, quat=quat, contype=0, conaffinity=0, group=1, rgba=[0.93, 0.55, 0.12, 1])
    for k in range(N_SEG):
        body.add_geom(name=f"{prefix}_wall{k:02d}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"socket_wall_{k:02d}",
                      pos=pos, quat=quat, group=3, friction=[0.3, 0.005, 0.0001])
    body.add_geom(name=f"{prefix}_cap", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="socket_cap",
                  pos=pos, quat=quat, group=3)


class Scene:
    def __init__(self, mode="ship"):
        assert mode in ("ship", "car")
        self.mode = mode
        spec = mujoco.MjSpec.from_file(str(SO101_XML))
        spec.option.timestep = 0.002
        spec.visual.global_.offwidth = 1600
        spec.visual.global_.offheight = 1200
        self._assets(spec)
        self._add_plug(spec)
        self._add_world(spec)
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
        g.add_geom(name="plug_vis", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="plug_vis", pos=PLUG_TIP,
                   contype=0, conaffinity=0, group=1, rgba=[0.12, 0.30, 0.62, 1], mass=0)
        # 충돌: 핀은 캡슐(둥근 끝이 모따기 역할), 몸통은 박스
        g.add_geom(name="plug_pin", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[PIN_R, 0, 0],
                   fromto=np.concatenate([PLUG_TIP + [0, 0, PIN_R], PLUG_TIP + [0, 0, 0.040]]),
                   group=3, mass=0.02, friction=[0.3, 0.005, 0.0001])
        g.add_geom(name="plug_body", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.008, 0.009, 0.015],
                   pos=PLUG_TIP + [0, 0, 0.055], group=3, mass=0.01)
        g.add_site(name="plug_tip", pos=PLUG_TIP, size=[0.002, 0, 0], group=3)

    def _add_world(self, spec):
        w = spec.worldbody
        w.add_light(pos=[0.2, -0.4, 1.0], dir=[-0.1, 0.4, -1], diffuse=[0.8, 0.8, 0.8], specular=[0.15, 0.15, 0.15])
        w.add_light(pos=[0.8, 0.6, 0.8], dir=[-0.6, -0.5, -0.6], diffuse=[0.35, 0.35, 0.35])
        w.add_geom(name="quay", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.14, 0.30, 0.075], pos=[-0.02, 0, -0.075],
                   material="concrete")
        if self.mode == "ship":
            w.add_geom(name="water", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], pos=[0, 0, -0.10],
                       rgba=[0.18, 0.42, 0.62, 1], contype=0, conaffinity=0)
            ship = w.add_body(name="ship", pos=SHIP_HOME, mocap=True)
            ship.add_geom(name="hull", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.16, 0.13, 0.035], pos=[0, 0, -0.035],
                          rgba=[0.86, 0.87, 0.89, 1])
            ship.add_geom(name="hull_stripe", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.161, 0.131, 0.006],
                          pos=[0, 0, -0.050], rgba=[0.75, 0.16, 0.14, 1], contype=0, conaffinity=0)
            ship.add_geom(name="cabin", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.08, 0.03], pos=[0.07, 0, 0.03],
                          rgba=[0.95, 0.95, 0.96, 1])
            _add_socket(ship, SOCKET_ON_SHIP, [1, 0, 0, 0], "sock")
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

    # 선박 운동: 갑판 기준 변위(dx, dy, dz)와 자세(roll, pitch)
    def set_ship(self, disp, roll, pitch):
        q = np.zeros(4)
        mujoco.mju_euler2Quat(q, np.array([roll, pitch, 0.0]), "xyz")
        self.data.mocap_pos[self.mocap] = SHIP_HOME + disp
        self.data.mocap_quat[self.mocap] = q

    def socket_frame(self):
        """소켓 입구 중심(월드)과 소켓 축(입구 바깥 방향, 단위 벡터)."""
        if self.mode == "ship":
            R = self.data.xmat[self.ship_body].reshape(3, 3)
            base = self.data.xpos[self.ship_body] + R @ SOCKET_ON_SHIP
            axis = R[:, 2]
        else:
            return CAR_ENTRANCE.copy(), CAR_AXIS.copy()
        return base + axis * SOCKET_H, axis
