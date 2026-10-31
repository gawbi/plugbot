"""로봇 계층. 제어 루프는 아래 인터페이스만 쓴다(실물과 MuJoCo 가짜 로봇이 같다).

    now() -> 초                     q() -> 팔 관절 5개(rad, MuJoCo 모델 기준)
    command(q)                      gripper(open: bool)
    wait_until(t)                   tip_position() -> 핀 끝(로봇 베이스 좌표, m)

RealRobot은 LeRobot 0.6.1의 SO-101 팔로워를 감싼다. LeRobot 관절값(use_degrees=True면 도)과
MuJoCo 모델 관절각 사이의 부호·영점은 config/joint_map.json으로 맞춘다. 처음 연결하면 반드시
`python -m real.robots check --port ...`로 실물 자세와 화면 속 모델 자세가 같은지 확인한다.
"""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from plugbot.control import IK
from plugbot.scene import ARM_JOINTS, Scene

CONFIG = Path(__file__).resolve().parent / "config"
DEFAULT_MAP = {
    # MuJoCo 관절각(rad) = sign × LeRobot 값(도) × π/180 + offset(rad)
    "joints": {j: {"sign": 1.0, "offset": 0.0} for j in ARM_JOINTS},
    "gripper_open": 40.0,   # LeRobot gripper.pos(0~100). 실물에서 플러그를 놓는 값
    "gripper_close": 2.0,   # 플러그 몸통 홈을 쥐는 값
    "max_step_deg": 6.0,    # 제어 주기(20 ms)당 관절 이동 한계. 갑작스러운 큰 움직임 방지
}


class Kinematics:
    """MuJoCo SO-101 모델(플러그를 쥔 상태)로 순·역기구학만 계산한다."""

    def __init__(self):
        self.scene = Scene("car")  # 장면 종류와 무관하게 팔과 플러그 끝 위치만 쓴다
        self.ik = IK(self.scene)
        self._d = mujoco.MjData(self.scene.model)

    def tip(self, q):
        s, d = self.scene, self._d
        d.qpos[:] = 0
        d.qpos[s.qadr] = q
        mujoco.mj_kinematics(s.model, d)
        return d.site_xpos[s.tip_site].copy()

    def solve(self, q0, target, tool_dir):
        return self.ik.solve(q0, target, tool_dir)

    @property
    def limits(self):
        return self.scene.qlo, self.scene.qhi


def load_map(path=CONFIG / "joint_map.json"):
    if Path(path).exists():
        return {**DEFAULT_MAP, **json.loads(Path(path).read_text())}
    return DEFAULT_MAP


class RealRobot:
    def __init__(self, port, robot_id="plugbot_follower", calibration_dir=None):
        try:
            from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        except ImportError as e:  # LeRobot 버전마다 모듈 이름이 다를 수 있다
            raise SystemExit(f"LeRobot(0.6.1)을 찾을 수 없다: {e}. ACT_PLAN.md 3.1대로 설치한다.")
        kw = dict(port=port, id=robot_id, use_degrees=True)
        if calibration_dir:
            kw["calibration_dir"] = Path(calibration_dir)
        self.robot = SOFollower(SOFollowerRobotConfig(**kw))
        self.robot.connect()
        self.map = load_map()
        self.kin = Kinematics()
        self._q_last = self.q()

    # LeRobot(도) ↔ MuJoCo(rad)
    def _to_model(self, obs):
        m = self.map["joints"]
        return np.array([m[j]["sign"] * np.deg2rad(obs[f"{j}.pos"]) + m[j]["offset"] for j in ARM_JOINTS])

    def _to_robot(self, q):
        m = self.map["joints"]
        return {f"{j}.pos": float(np.rad2deg((q[k] - m[j]["offset"]) / m[j]["sign"])) for k, j in enumerate(ARM_JOINTS)}

    def now(self):
        return time.monotonic()

    def wait_until(self, t):
        dt = t - time.monotonic()
        if dt > 0:
            time.sleep(dt)

    def q(self):
        return self._to_model(self.robot.get_observation())

    def command(self, q):
        lo, hi = self.kin.limits
        step = np.deg2rad(self.map["max_step_deg"])
        q = np.clip(np.clip(q, self._q_last - step, self._q_last + step), lo, hi)
        action = self._to_robot(q)
        action["gripper.pos"] = self._grip
        self.robot.send_action(action)
        self._q_last = q

    _grip = DEFAULT_MAP["gripper_close"]

    def gripper(self, open_):
        self._grip = self.map["gripper_open"] if open_ else self.map["gripper_close"]
        self.command(self._q_last)

    def torque(self, on):
        bus = self.robot.bus
        bus.enable_torque() if on else bus.disable_torque()

    def tip_position(self):
        return self.kin.tip(self.q())

    def close(self):
        self.robot.disconnect()


def check(port):
    """토크를 끈 실물 팔을 손으로 움직이면 MuJoCo 모델이 같은 자세로 따라 그려진다. 다르면 joint_map.json을 고친다."""
    import cv2
    robot = RealRobot(port)
    robot.torque(False)
    s = robot.kin.scene
    d = mujoco.MjData(s.model)
    renderer = mujoco.Renderer(s.model, 480, 640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.12, 0, 0.08]
    cam.distance, cam.azimuth, cam.elevation = 0.75, 135, -20
    print("q로 끝낸다. 실물과 화면 자세가 다르면 해당 관절의 sign(-1) 또는 offset(rad)을 config/joint_map.json에 적는다.")
    try:
        while True:
            q = robot.q()
            d.qpos[:] = 0
            d.qpos[s.qadr] = q
            mujoco.mj_forward(s.model, d)
            renderer.update_scene(d, cam)
            img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            txt = " ".join(f"{np.rad2deg(v):6.1f}" for v in q)
            cv2.putText(img, "deg: " + txt, (10, 25), 0, 0.55, (0, 0, 255), 2)
            tip = d.site_xpos[s.tip_site] * 1000
            cv2.putText(img, "tip mm: " + " ".join(f"{v:6.1f}" for v in tip), (10, 50), 0, 0.55, (0, 0, 255), 2)
            cv2.imshow("model pose (real.robots check)", img)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        robot.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["check", "write-default-map"])
    ap.add_argument("--port")
    a = ap.parse_args()
    if a.cmd == "write-default-map":
        CONFIG.mkdir(exist_ok=True)
        (CONFIG / "joint_map.json").write_text(json.dumps(DEFAULT_MAP, indent=1))
        print(CONFIG / "joint_map.json")
    else:
        check(a.port)
