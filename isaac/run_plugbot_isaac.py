"""[Isaac Sim 5.1, GPU 노트북] MuJoCo 실험(plugbot.control.run_trial)을 Isaac Sim에서 다시 돌린다.

- 물리: Isaac(PhysX) 500 Hz. 선박은 키네마틱 강체라 매 스텝 파랑 운동(plugbot.waves.ShipMotion)으로 자세를 넣는다.
- 제어: 50 Hz. 소켓 관측(지연 0.1 s, 잡음 0.5 mm), AR 예측, 정온 구간, IK는 MuJoCo 실험과 같은 코드
  (plugbot.control.SocketEstimator, plugbot.control.IK)를 쓴다. MuJoCo는 기구학 계산에만 쓴다.
  URDF와 MuJoCo 모델의 기구학이 같음은 isaac/urdf_fk.py로 확인했다(오차 0.004 mm).
- 채점: 플러그 끝(관절값 → 순기구학)이 보어 안 목표 깊이를 0.2 s 유지하면 성공.

준비(한 번): ./python.sh -m pip install mujoco numpy   그리고 import_robot.py로 USD를 만든다.
실행: ./python.sh <저장소>/plugbot_demo/isaac/run_plugbot_isaac.py --hs 0.06 --strategy predictive --trials 10
      화면 없이: --headless

※ 이 맥에서는 실행해 보지 못했다(GPU 없음). Isaac API를 쓰는 곳은 [ISAAC] 표시로 모아 두었다.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

ap = argparse.ArgumentParser()
ap.add_argument("--hs", type=float, default=0.06, help="모형 유의파고(m), 0이면 정지")
ap.add_argument("--strategy", default="predictive", choices=["static", "reactive", "predictive"])
ap.add_argument("--trials", type=int, default=10)
ap.add_argument("--headless", action="store_true")
ap.add_argument("--video", action="store_true",
                help="첫 시행을 웹캠과 발표용 두 시점으로 녹화해 outputs/isaac_video_*.mp4로 저장(화면 없이도 된다)")
ap.add_argument("--video-fps", type=int, default=25)
ap.add_argument("--kp", type=float, default=1000.0, help="관절 위치 제어 강성(추종이 느리면 올린다)")
ap.add_argument("--kd", type=float, default=30.0)
args = ap.parse_args()

# [ISAAC] 앱은 다른 omni 모듈보다 먼저 만든다
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import mujoco  # noqa: E402
from isaacsim.core.api import World  # noqa: E402  [ISAAC]
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402  [ISAAC]
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402  [ISAAC]
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402  [ISAAC]
from pxr import UsdPhysics  # noqa: E402

from plugbot.control import CONTROL_DT, HOME, IK, Params, SocketEstimator  # noqa: E402
from plugbot.scene import ARM_JOINTS, BORE_R, PIN_R, SHIP_HOME, SOCKET_H, SOCKET_ON_SHIP, Scene  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

PHYSICS_DT = 0.002
N_SUB = int(round(CONTROL_DT / PHYSICS_DT))

# ---------- 장면 ----------
world = World(physics_dt=PHYSICS_DT, rendering_dt=CONTROL_DT, stage_units_in_meters=1.0)  # [ISAAC]
add_reference_to_stage(str(HERE / "scene/plugbot_scene.usda"), "/World/Env")  # [ISAAC]
add_reference_to_stage(str(HERE / "so101/so101_plugbot.usd"), "/World/SO101")  # [ISAAC]
stage = get_current_stage()
root = next((p.GetPath().pathString for p in stage.Traverse()
             if p.GetPath().pathString.startswith("/World/SO101") and p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
if root is None:
    raise SystemExit("로봇 USD에서 ArticulationRoot를 못 찾았다. import_robot.py 결과를 확인한다.")
robot = world.scene.add(SingleArticulation(prim_path=root, name="so101"))  # [ISAAC]
cams = {}
if args.video:
    from isaacsim.sensors.camera import Camera  # noqa: E402  [ISAAC]
    for cname in ("Webcam", "ShowCam"):
        cams[cname] = Camera(prim_path=f"/World/Env/World/{cname}", resolution=(1280, 720))  # [ISAAC] 기존 카메라 프림
ship = SingleXFormPrim("/World/Env/World/Ship", name="ship")  # [ISAAC] 참조 안의 경로
world.reset()  # [ISAAC]
for c in cams.values():
    c.initialize()  # [ISAAC]
dof = [robot.get_dof_index(j) for j in ARM_JOINTS]  # [ISAAC]
robot.get_articulation_controller().set_gains(  # [ISAAC]
    kps=np.full(robot.num_dof, args.kp), kds=np.full(robot.num_dof, args.kd))

kin = Scene("car")  # MuJoCo는 기구학만
ik = IK(kin)
kd = mujoco.MjData(kin.model)


def tip_of(q):
    kd.qpos[:] = 0
    kd.qpos[kin.qadr] = q
    mujoco.mj_kinematics(kin.model, kd)
    return kd.site_xpos[kin.tip_site].copy()


def ship_pose(motion, t):
    """(위치, 쿼터니언 wxyz, 소켓 입구, 소켓 축). MuJoCo Scene.set_ship/socket_frame과 같은 계산."""
    disp, roll, pitch = motion(t)
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.array([roll, pitch, 0.0]), "xyz")
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    R = R.reshape(3, 3)
    pos = SHIP_HOME + disp
    axis = R[:, 2]
    return pos, q, pos + R @ SOCKET_ON_SHIP + axis * SOCKET_H, axis


def set_arm(q):
    robot.apply_action(ArticulationAction(joint_positions=q, joint_indices=np.array(dof)))  # [ISAAC]


def arm_q():
    return np.asarray(robot.get_joint_positions())[dof]  # [ISAAC]


class VideoOut:
    """카메라별 mp4 스트림(프레임을 메모리에 쌓지 않는다). imageio-ffmpeg가 없으면 PNG로 저장."""

    def __init__(self, tag):
        self.writers, self.tag, self.n = {}, tag, 0
        out = HERE.parent / "outputs"
        out.mkdir(exist_ok=True)
        try:
            import imageio.v2 as imageio
            for name in cams:
                self.writers[name] = imageio.get_writer(str(out / f"isaac_video_{tag}_{name}.mp4"),
                                                        fps=args.video_fps, codec="libx264", quality=8)
        except Exception as e:  # noqa: BLE001
            print("mp4 저장 불가, PNG로 저장:", e)
            self.png_dir = out / f"isaac_frames_{tag}"
            self.png_dir.mkdir(exist_ok=True)

    def grab(self):
        for name, cam in cams.items():
            rgba = cam.get_rgba()  # [ISAAC]
            if rgba is None or rgba.size == 0:
                continue
            rgb = np.ascontiguousarray(rgba[..., :3])
            if name in self.writers:
                self.writers[name].append_data(rgb)
            else:
                from PIL import Image
                Image.fromarray(rgb).save(self.png_dir / f"{name}_{self.n:05d}.png")
        self.n += 1

    def close(self):
        for w in self.writers.values():
            w.close()
        print(f"영상 {self.n}장 저장: outputs/isaac_video_{self.tag}_*.mp4")


def run_trial(motion, strategy, p=Params(), seed=0, video=None):
    """plugbot.control.run_trial과 같은 상태 기계. (성공, 이유, 삽입 시각). video가 있으면 녹화하고 성공 뒤 1초 더 찍는다."""
    rng = np.random.default_rng(seed)
    world.reset()  # [ISAAC]
    robot.set_joint_positions(HOME, joint_indices=np.array(dof))  # [ISAAC]
    set_arm(HOME)
    est = SocketEstimator(strategy, p, rng)
    phase, depth_cmd, gate_t, hold_t, t = "approach", -p.hover, None, 0.0, 0.0
    every = max(1, int(round(1.0 / args.video_fps / CONTROL_DT)))
    tick, done, tail = 0, None, 0
    while t < p.timeout + (1.0 if done else 0.0):
        _, _, entrance, axis = ship_pose(motion, t)
        est.capture(t, entrance, axis)
        e = est.estimate(t)
        q = arm_q()
        if e is not None:
            ent_hat, ax_hat, speed = e
            rel = tip_of(q) - ent_hat
            lateral = np.linalg.norm(rel - np.dot(rel, ax_hat) * ax_hat)
            if phase == "approach" and lateral < p.align_tol and abs(np.dot(rel, ax_hat) - p.hover) < p.align_tol:
                phase, gate_t = "gate", t
            if phase == "gate" and (strategy != "predictive" or speed < p.gate_speed or t - gate_t > p.gate_timeout):
                phase = "insert"
            if phase == "insert":
                depth_cmd = min(depth_cmd + p.descend_speed * CONTROL_DT, p.depth_goal + 0.004)
            set_arm(ik.solve(q, ent_hat - ax_hat * depth_cmd, -ax_hat))
        for k in range(N_SUB):
            pos, quat, _, _ = ship_pose(motion, t)
            ship.set_world_pose(position=pos, orientation=quat)  # [ISAAC] 키네마틱 선박
            world.step(render=(k == N_SUB - 1 and (not args.headless or video is not None)))  # [ISAAC] 제어 주기마다 한 번
            t += PHYSICS_DT
        tick += 1
        if video is not None and tick % every == 0:
            video.grab()
        if done is not None:
            tail += 1
            if video is None or tail * CONTROL_DT >= 1.0:
                return done
            continue
        _, _, entrance, axis = ship_pose(motion, t)
        rel = tip_of(arm_q()) - entrance
        depth = -float(np.dot(rel, axis))
        lateral = float(np.linalg.norm(rel + depth * axis))
        if depth >= p.depth_goal and lateral < BORE_R - PIN_R + 0.001:
            hold_t += CONTROL_DT
            if hold_t >= p.hold:
                done = (True, "", t)
        else:
            hold_t = 0.0
    return done or (False, "시간 초과(걸림, 정렬 실패)", float("nan"))


def main():
    motion_of = (lambda k: ShipMotion(args.hs, seed=k)) if args.hs > 0 else (lambda k: (lambda t: (np.zeros(3), 0.0, 0.0)))
    rows = []
    for k in range(args.trials):
        video = VideoOut(f"hs{args.hs * 100:.0f}_{args.strategy}") if (args.video and k == 0) else None
        ok, why, t_ins = run_trial(motion_of(k), args.strategy, seed=k, video=video)
        if video is not None:
            video.close()
        rows.append({"시드": k, "성공": ok, "이유": why, "삽입 시각 s": t_ins})
        print(f"[{k + 1}/{args.trials}] {'성공' if ok else '실패'} {why}")
    rate = float(np.mean([r["성공"] for r in rows]))
    print(f"Isaac Sim, Hs {args.hs * 100:.0f} cm, {args.strategy}: 성공률 {rate * 100:.0f}% "
          f"(MuJoCo 예선 결과: 6 cm 예측 90%, 추종 37%)")
    out = HERE.parent / "outputs" / f"isaac_hs{args.hs * 100:.0f}_{args.strategy}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"hs": args.hs, "strategy": args.strategy, "성공률": rate, "시행": rows},
                              ensure_ascii=False, indent=1))
    app.close()


if __name__ == "__main__":
    main()
