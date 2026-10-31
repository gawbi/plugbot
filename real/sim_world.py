"""실물 없이 실물용 소프트웨어 전체(웹캠 영상 → 마커 추적 → 보정 변환 → 예측 → IK → 팔)를 시험하는 MuJoCo 세계.

- 소켓 탭 홈에 실제 인쇄용과 같은 ArUco 마커(30 mm 흰 종이 위 25 mm 마커)를 붙이고, 배치(G.LAYOUT)에 따라
  갑판 시트(top) 또는 세운 판 앞면(side)에 30 mm 마커 4개를 붙인다.
- 웹캠 위치에 카메라를 두고 30 Hz로 렌더링한다. side는 로봇 받침 옆 셀카봉의 앞 카메라(1280 × 720, 화각 60°). 렌더링한 영상은 latency 뒤에야 쓸 수 있다(실물 웹캠 지연 흉내).
- SimRobot은 robots.RealRobot과 같은 인터페이스다. 팔은 위치 제어 서보(모델의 액추에이터)로 움직인다.
"""

import cv2
import mujoco
import numpy as np

from plugbot.control import CONTROL_DT
from plugbot.scene import CAMERAS_SIDE, SIDE_QUAT, SOCKET_ON_SHIP, SOCKET_SIDE_ON_SHIP, Scene

from . import geometry as G
from .markers import marker_image
from .robots import Kinematics
from .vision import Intrinsics

TEX = G.__file__.replace("geometry.py", "print/_sim_socket_marker.png")
if G.LAYOUT == "top":
    CAM_POS = np.array([0.36, -0.20, 0.30])  # 웹캠: 선박 바깥쪽 위에서 소켓을 비스듬히 내려다본다
    CAM_LOOK = np.array([0.24, 0.0, -0.03])
    W, H, FOVY = 640, 480, 50.0
else:
    CAM_POS, CAM_LOOK, _hfov = CAMERAS_SIDE["front"]
    W, H = 1280, 720
    FOVY = float(np.rad2deg(2 * np.arctan(np.tan(np.deg2rad(_hfov) / 2) * H / W)))


def _marker_texture(mid, margin_px, path):
    """흰 종이 가운데 마커(인쇄물과 같은 모양). 마커는 300 px."""
    from PIL import Image
    paper = Image.new("RGB", (300 + 2 * margin_px,) * 2, "white")
    paper.paste(marker_image(mid, 25.4), (margin_px, margin_px))
    paper.save(path)


def _xyaxes(pos, look):
    f = (look - pos) / np.linalg.norm(look - pos)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return np.concatenate([x, y])


class MarkerScene(Scene):
    def _add_world(self, spec):
        super()._add_world(spec)
        # (id, 마커 한 변, 종이 한 변, 소켓 좌표계 중심): 탭 마커는 30 mm 종이 위 25 mm, 갑판 마커는 36 mm 종이 위 30 mm
        papers = [(G.SOCKET_MARKER_ID, 0.025, 0.030, G.MARKER_IN_SOCKET + [0, 0, 0.0002])]
        papers += [(mid, 0.030, 0.036, np.array([x, y, 0.4]) * G.MM) for mid, (x, y) in G.BOARD_MARKERS.items()]
        ship = spec.body("ship") if self.mode == "ship" else None
        if self.layout == "top":
            base, quat = SOCKET_ON_SHIP, np.array([1.0, 0, 0, 0])
        else:
            base, quat = SOCKET_SIDE_ON_SHIP, SIDE_QUAT
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        for mid, size, paper_size, center in papers:
            path = TEX.replace("_sim_socket_marker", f"_sim_marker_{mid}")
            _marker_texture(mid, round(300 * (paper_size / size - 1) / 2), path)
            spec.add_texture(name=f"aruco{mid}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=path)
            spec.add_material(name=f"aruco{mid}", textures=["", f"aruco{mid}"])
            h = paper_size / 2
            spec.add_mesh(name=f"aruco_quad{mid}", uservert=[-h, -h, 0, h, -h, 0, h, h, 0, -h, h, 0],
                          userface=[0, 1, 2, 0, 2, 3], usertexcoord=[0, 1, 1, 1, 1, 0, 0, 0],
                          inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
            if ship is not None:
                ship.add_geom(name=f"aruco{mid}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"aruco_quad{mid}",
                              material=f"aruco{mid}", pos=base + R @ center, quat=quat, contype=0, conaffinity=0,
                              group=1)
        spec.worldbody.add_camera(name="webcam", pos=CAM_POS, xyaxes=_xyaxes(CAM_POS, CAM_LOOK), fovy=FOVY)


def sim_intrinsics():
    f = H / 2 / np.tan(np.deg2rad(FOVY) / 2)
    return Intrinsics(np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]]), np.zeros(5), (W, H))


class SimWorld:
    """가짜 로봇 + 가짜 웹캠. motion(t) -> (변위, roll, pitch)가 None이면 선박이 정지해 있다."""

    def __init__(self, motion=None, latency=0.10, cam_hz=30.0, seed=0):
        self.scene = MarkerScene("ship", G.LAYOUT)
        self.m, self.d = self.scene.model, self.scene.data
        self.motion = motion or (lambda t: (np.zeros(3), 0.0, 0.0))
        self.latency, self.cam_dt = latency, 1.0 / cam_hz
        self.renderer = mujoco.Renderer(self.m, H, W)
        self.cam_id = self.m.camera("webcam").id
        self.kin = Kinematics()
        self.rng = np.random.default_rng(seed)
        self.reset()

    # ----- 세계 -----
    def reset(self, q0=None):
        mujoco.mj_resetData(self.m, self.d)
        q0 = self.kin.home if q0 is None else q0
        self.d.qpos[self.scene.qadr] = q0
        self.d.ctrl[self.scene.act] = q0
        self.scene.set_ship(*self.motion(0.0))
        mujoco.mj_forward(self.m, self.d)
        self.t, self.frames, self.next_cap = 0.0, [], 0.0
        self._capture()

    def _capture(self):
        self.renderer.update_scene(self.d, camera=self.cam_id)
        img = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        self.frames.append((self.t, img))
        self.frames = self.frames[-8:]
        self.next_cap = self.t + self.cam_dt

    def true_cam_pose(self):
        """정답 확인용: OpenCV 카메라 좌표 → 월드(=로봇 베이스) 변환 (R, t)."""
        R = self.d.cam_xmat[self.cam_id].reshape(3, 3) @ np.diag([1, -1, -1])
        return R, self.d.cam_xpos[self.cam_id].copy()

    def socket_truth(self):
        return self.scene.socket_frame()

    # ----- 웹캠 인터페이스 -----
    def latest(self):
        """latency 이전에 찍힌 가장 최근 영상 (촬영 시각, 영상)."""
        ok = [f for f in self.frames if f[0] <= self.t - self.latency + 1e-9]
        return ok[-1] if ok else self.frames[0]

    # ----- 로봇 인터페이스 -----
    def now(self):
        return self.t

    def wait_until(self, t_target):
        n = max(1, int(round((t_target - self.t) / self.m.opt.timestep)))
        for _ in range(n):
            self.scene.set_ship(*self.motion(self.t))
            mujoco.mj_step(self.m, self.d)
            self.t += self.m.opt.timestep
            if self.t + 1e-9 >= self.next_cap:
                self._capture()

    def q(self):
        return self.d.qpos[self.scene.qadr].copy()

    def command(self, q):
        self.d.ctrl[self.scene.act] = q

    def gripper(self, open_):
        pass  # 시뮬레이션에서는 플러그가 그리퍼에 붙어 있다(놓기는 실물에서 자석 래치가 맡는다)

    def tip_position(self):
        return self.d.site_xpos[self.scene.tip_site].copy()

    def set_arm(self, q):
        """보정 시험용: 팔을 해당 관절각에 순간 이동(사람이 손으로 꽂은 자세 흉내)."""
        self.d.qpos[self.scene.qadr] = q
        self.d.ctrl[self.scene.act] = q
        mujoco.mj_forward(self.m, self.d)
        self._capture()
        self.frames = [(self.t - self.latency - 1e-6, self.frames[-1][1])]


CONTROL = CONTROL_DT
