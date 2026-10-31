"""[Isaac Sim 5.1, GPU 노트북] MuJoCo 실험(plugbot.control.run_trial)을 Isaac Sim에서 다시 돌린다.

- 배치: --layout side(기본, 본선: 현측에 세운 소켓, L자 플러그) 또는 top(예선: 갑판 위 소켓, 일자 플러그).
- 물리: Isaac(PhysX) 500 Hz. 선박은 키네마틱 강체라 매 스텝 파랑 운동(plugbot.waves.ShipMotion)으로 자세를 넣는다.
  파도는 기본으로 부두에 옆으로 댄 배(--berth alongside, 롤 축이 부두와 나란함). MuJoCo 실험과 같다.
- 제어: 50 Hz. 소켓 관측(지연 0.1 s, 잡음 0.5 mm), AR 예측, 정온 구간, IK는 MuJoCo 실험과 같은 코드
  (plugbot.control.SocketEstimator, plugbot.control.IK, plugbot.scene.ship_pose)를 쓴다. MuJoCo는 기구학 계산에만 쓴다.
  URDF와 MuJoCo 모델의 플러그 끝, 손목 카메라 자세가 같음은 맥에서 확인했다(0.004 mm).
- 채점: 플러그 끝(관절값 → 순기구학)이 보어 안 목표 깊이를 0.2 s 유지하면 성공.
- --video: 첫 시행을 Webcam(앞 카메라), ShowCam(발표용), Wrist(손목) 시점으로 mp4 저장.
- --preview: 시작 자세에서 세 시점 사진만 찍고 끝낸다(장면이 맞는지 처음 확인할 때).
- --dataset N: 성공한 시행 N개를 ACT 학습 데이터로 녹화(앞, 손목 640 × 480, 30 fps, plugbot.dataset 형식).
  에피소드마다 파고를 0~--hs에서 고른다. LeRobot 데이터셋으로: tools/to_lerobot.py.

준비(한 번): <Isaac 파이썬> -m pip install mujoco imageio imageio-ffmpeg pillow scipy, import_robot.py로 USD를 만든다.
실행: <Isaac 파이썬> isaac/run_plugbot_isaac.py --hs 0.06 --strategy predictive --trials 10 --headless

※ 이 맥에서는 실행해 보지 못했다(GPU 없음). Isaac API를 쓰는 곳은 [ISAAC] 표시로 모아 두었다.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

ap = argparse.ArgumentParser()
ap.add_argument("--layout", default="side", choices=["side", "top"])
ap.add_argument("--berth", default="alongside", choices=["alongside", "bow"], help="파도 방향(MuJoCo 실험과 같게)")
ap.add_argument("--hs", type=float, default=0.06, help="모형 유의파고(m), 0이면 정지")
ap.add_argument("--strategy", default="predictive", choices=["static", "reactive", "predictive"])
ap.add_argument("--trials", type=int, default=10)
ap.add_argument("--seed", type=int, default=0, help="첫 시행의 파도 시드(영상 비교 때 전략끼리 같은 파도를 쓰려고)")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--video", action="store_true", help="첫 시행을 세 시점으로 녹화해 outputs/isaac_video_*.mp4로 저장")
ap.add_argument("--video-fps", type=int, default=25)
ap.add_argument("--preview", action="store_true", help="시작 자세 사진만 찍고 끝낸다(outputs/isaac_preview_*.png)")
ap.add_argument("--dataset", type=int, default=0, help="성공 시행 N개를 학습 데이터로 녹화(data/isaac_<layout>/)")
ap.add_argument("--kp", type=float, default=1000.0, help="관절 위치 제어 강성(추종이 느리면 올린다)")
ap.add_argument("--kd", type=float, default=30.0)
args = ap.parse_args()
if args.dataset and args.video:
    raise SystemExit("--dataset과 --video는 따로 실행한다(카메라 해상도가 다르다)")
SUFFIX = "_side" if args.layout == "side" else ""

# [ISAAC] 앱은 다른 omni 모듈보다 먼저 만든다
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import mujoco  # noqa: E402
from isaacsim.core.api import World  # noqa: E402  [ISAAC]
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: E402  [ISAAC]
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402  [ISAAC]
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402  [ISAAC]
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

from plugbot.control import CONTROL_DT, HOME, IK, Params, SocketEstimator, home_pose  # noqa: E402
from plugbot.dataset import EpisodeWriter, to_lerobot_vector  # noqa: E402
from plugbot.scene import ARM_JOINTS, BORE_R, CAM_H, CAM_W, PIN_R, Scene, ship_pose  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

PHYSICS_DT = 0.002
N_SUB = int(round(CONTROL_DT / PHYSICS_DT))
WRIST_FOVY = 61.0  # MuJoCo wrist_cam 세로 화각
OUT = ROOT / "outputs"

# ---------- 장면 ----------
world = World(physics_dt=PHYSICS_DT, rendering_dt=CONTROL_DT, stage_units_in_meters=1.0)  # [ISAAC]
add_reference_to_stage(str(HERE / f"scene/plugbot_scene{SUFFIX}.usda"), "/World/Env")  # [ISAAC]
robot_usd = HERE / f"so101/so101_plugbot{SUFFIX}.usd"
if not robot_usd.exists():
    raise SystemExit(f"{robot_usd.name}가 없다. 먼저: import_robot.py --layout {args.layout}")
add_reference_to_stage(str(robot_usd), "/World/SO101")  # [ISAAC]
stage = get_current_stage()
root = next((p.GetPath().pathString for p in stage.Traverse()
             if p.GetPath().pathString.startswith("/World/SO101") and p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
if root is None:
    raise SystemExit("로봇 USD에서 ArticulationRoot를 못 찾았다. import_robot.py 결과를 확인한다.")


def add_wrist_camera():
    """URDF의 wrist_cam_link(MuJoCo wrist_cam과 같은 자세) 아래에 USD 카메라를 만든다. 경로를 돌려준다."""
    link = next((p for p in stage.Traverse()
                 if p.GetPath().pathString.startswith("/World/SO101") and p.GetName() == "wrist_cam_link"), None)
    if link is None:
        print("wrist_cam_link를 못 찾아 손목 카메라 없이 진행한다(URDF를 build_assets.py로 다시 만들었는지 확인)")
        return None
    cam = UsdGeom.Camera.Define(stage, link.GetPath().AppendChild("WristCam"))
    v_ap = 20.955 * CAM_H / CAM_W  # 4:3 영상의 세로 조리개
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(v_ap)
    cam.CreateFocalLengthAttr(float(v_ap / 2 / np.tan(np.deg2rad(WRIST_FOVY) / 2)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.005, 10))
    return cam.GetPath().pathString


robot = world.scene.add(SingleArticulation(prim_path=root, name="so101"))  # [ISAAC]
cams, ds_cams = {}, {}
if args.video or args.preview or args.dataset:
    from isaacsim.sensors.camera import Camera  # noqa: E402  [ISAAC]
    wrist_path = add_wrist_camera()
    if args.dataset:  # 실물 ACT 녹화와 같은 이름, 해상도
        ds_cams["front"] = Camera(prim_path="/World/Env/World/Webcam", resolution=(CAM_W, CAM_H))  # [ISAAC]
        if wrist_path:
            ds_cams["wrist"] = Camera(prim_path=wrist_path, resolution=(CAM_W, CAM_H))  # [ISAAC]
    else:
        for cname in ("Webcam", "ShowCam"):
            cams[cname] = Camera(prim_path=f"/World/Env/World/{cname}", resolution=(1280, 720))  # [ISAAC]
        if wrist_path:
            cams["Wrist"] = Camera(prim_path=wrist_path, resolution=(CAM_W, CAM_H))  # [ISAAC]
ship = SingleXFormPrim("/World/Env/World/Ship", name="ship")  # [ISAAC] 참조 안의 경로
world.reset()  # [ISAAC]
for c in list(cams.values()) + list(ds_cams.values()):
    c.initialize()  # [ISAAC]
dof = [robot.get_dof_index(j) for j in ARM_JOINTS]  # [ISAAC]
robot.get_articulation_controller().set_gains(  # [ISAAC]
    kps=np.full(robot.num_dof, args.kp), kds=np.full(robot.num_dof, args.kd))

kin = Scene("car", args.layout)  # MuJoCo는 기구학만(플러그 모양이 배치를 따른다)
ik = IK(kin)
kd = mujoco.MjData(kin.model)
START = HOME if args.layout == "top" else home_pose(kin)  # side: 소켓 입구 앞 4 cm, 충돌 없는 자세


def tip_of(q):
    kd.qpos[:] = 0
    kd.qpos[kin.qadr] = q
    mujoco.mj_kinematics(kin.model, kd)
    return kd.site_xpos[kin.tip_site].copy()


def set_arm(q):
    robot.apply_action(ArticulationAction(joint_positions=q, joint_indices=np.array(dof)))  # [ISAAC]


def arm_q():
    return np.asarray(robot.get_joint_positions())[dof]  # [ISAAC]


def rgb_of(cam):
    rgba = cam.get_rgba()  # [ISAAC]
    if rgba is None or rgba.size == 0:
        return None
    return np.ascontiguousarray(rgba[..., :3]).astype(np.uint8)


class VideoOut:
    """카메라별 mp4 스트림(프레임을 메모리에 쌓지 않는다). imageio-ffmpeg가 없으면 PNG로 저장."""

    def __init__(self, tag):
        self.writers, self.tag, self.n = {}, tag, 0
        OUT.mkdir(exist_ok=True)
        try:
            import imageio.v2 as imageio
            for name in cams:
                self.writers[name] = imageio.get_writer(str(OUT / f"isaac_video_{tag}_{name}.mp4"),
                                                        fps=args.video_fps, codec="libx264", quality=8,
                                                        macro_block_size=1)
        except Exception as e:  # noqa: BLE001
            print("mp4 저장 불가, PNG로 저장:", e)
            self.png_dir = OUT / f"isaac_frames_{tag}"
            self.png_dir.mkdir(exist_ok=True)

    def grab(self):
        for name, cam in cams.items():
            rgb = rgb_of(cam)
            if rgb is None:
                continue
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


class Recorder:
    """학습 데이터: 물리 스텝 격자에서 30 fps로 앞, 손목 영상과 관절 상태, 명령을 쌓는다."""

    def __init__(self, writer, fps=30):
        self.w, self.dt, self.next_t = writer, 1.0 / fps, 0.0

    def due(self, t):
        return t + 1e-9 >= self.next_t

    def grab(self, t, q, q_cmd):
        self.next_t += self.dt
        images = {name: rgb_of(cam) for name, cam in ds_cams.items()}
        if any(v is None for v in images.values()):
            return  # 렌더 결과가 아직 없으면(첫 프레임 등) 건너뛴다
        self.w.add(t, images, to_lerobot_vector(q), to_lerobot_vector(q_cmd))


def reset_robot():
    world.reset()  # [ISAAC]
    robot.set_joint_positions(START, joint_indices=np.array(dof))  # [ISAAC]
    set_arm(START)


def run_trial(motion, strategy, p=Params(), seed=0, video=None, rec=None):
    """plugbot.control.run_trial과 같은 상태 기계. (성공, 이유, 삽입 시각). video가 있으면 녹화하고 성공 뒤 1초 더 찍는다."""
    rng = np.random.default_rng(seed)
    reset_robot()
    est = SocketEstimator(strategy, p, rng)
    phase, depth_cmd, gate_t, hold_t, t = "approach", -p.hover, None, 0.0, 0.0
    every = max(1, int(round(1.0 / args.video_fps / CONTROL_DT)))
    tick, done, tail, q_cmd = 0, None, 0, START.copy()
    while t < p.timeout + (1.0 if done else 0.0):
        _, _, entrance, axis = ship_pose(args.layout, motion, t)
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
            q_cmd = ik.solve(q, ent_hat - ax_hat * depth_cmd, -ax_hat)
            set_arm(q_cmd)
        for k in range(N_SUB):
            pos, quat, _, _ = ship_pose(args.layout, motion, t)
            ship.set_world_pose(position=pos, orientation=quat)  # [ISAAC] 키네마틱 선박
            grab = rec is not None and rec.due(t + PHYSICS_DT)
            last = k == N_SUB - 1 and (not args.headless or video is not None)
            world.step(render=grab or last)  # [ISAAC]
            t += PHYSICS_DT
            if grab:
                rec.grab(t, arm_q(), q_cmd)
        tick += 1
        if video is not None and tick % every == 0:
            video.grab()
        if done is not None:
            tail += 1
            if video is None or tail * CONTROL_DT >= 1.0:
                return done
            continue
        _, _, entrance, axis = ship_pose(args.layout, motion, t)
        rel = tip_of(arm_q()) - entrance
        depth = -float(np.dot(rel, axis))
        lateral = float(np.linalg.norm(rel + depth * axis))
        if depth >= p.depth_goal and lateral < BORE_R - PIN_R + 0.001:
            hold_t += CONTROL_DT
            if hold_t >= p.hold:
                done = (True, "", t)
                if video is None:
                    return done
        else:
            hold_t = 0.0
    return done or (False, "시간 초과(걸림, 정렬 실패)", float("nan"))


def motion_for(hs, seed):
    if hs <= 0:
        return lambda t: (np.zeros(3), 0.0, 0.0)
    return ShipMotion(hs, seed=seed, berth=args.berth)


def mujoco_reference():
    """같은 조건의 MuJoCo 결과(조건당 30회). 없으면 None."""
    name = f"experiment_{args.layout}.json" if args.berth == "alongside" else "experiment_top_bow.json"
    path = OUT / name
    if not path.exists() or (args.berth == "bow" and args.layout == "side"):
        return None
    res = json.loads(path.read_text())["결과"]
    key = next((k for k in res if k.startswith(f"Hs={args.hs * 100:.0f}cm")), None)
    return None if key is None else res[key][args.strategy]["성공률"]


def preview():
    reset_robot()
    for _ in range(6):  # 렌더 결과가 나오기까지 몇 번 그린다
        world.step(render=True)  # [ISAAC]
    from PIL import Image
    OUT.mkdir(exist_ok=True)
    for name, cam in cams.items():
        rgb = rgb_of(cam)
        if rgb is not None:
            path = OUT / f"isaac_preview_{args.layout}_{name}.png"
            Image.fromarray(rgb).save(path)
            print("저장:", path)


def record_dataset():
    out = ROOT / "data" / f"isaac_{args.layout}"
    rng = np.random.default_rng(0)
    kept, k = 0, 0
    while kept < args.dataset and k < args.dataset * 3:
        hs = float(rng.uniform(0.0, args.hs))
        writer = EpisodeWriter(out, kept, ds_cams, fps=30)
        ok, why, _ = run_trial(motion_for(hs, 1000 + k), args.strategy, seed=1000 + k, rec=Recorder(writer))
        if ok and len(writer) > 0:
            writer.close({"layout": args.layout, "hs": hs, "seed": 1000 + k, "strategy": args.strategy, "success": True,
                          "sim": "isaac", "task": "Insert the plug into the socket"})
            kept += 1
            print(f"[{kept}/{args.dataset}] Hs {hs * 100:.1f} cm 성공, {len(writer)}프레임")
        else:
            writer.discard()
            print(f"  (버림) Hs {hs * 100:.1f} cm {why}")
        k += 1
    print(f"{out}: 에피소드 {kept}개. LeRobot으로: python tools/to_lerobot.py {out} --repo-id <HF아이디>/plugbot_isaac")


def main():
    if args.preview:
        preview()
        app.close()
        return
    if args.dataset:
        record_dataset()
        app.close()
        return
    rows = []
    for k in range(args.seed, args.seed + args.trials):
        video = VideoOut(f"{args.layout}_hs{args.hs * 100:.0f}_{args.strategy}") if (args.video and k == args.seed) else None
        ok, why, t_ins = run_trial(motion_for(args.hs, k), args.strategy, seed=k, video=video)
        if video is not None:
            video.close()
        rows.append({"시드": k, "성공": ok, "이유": why, "삽입 시각 s": t_ins})
        print(f"[{k - args.seed + 1}/{args.trials}] 시드 {k} {'성공' if ok else '실패'} {why}")
    rate = float(np.mean([r["성공"] for r in rows]))
    ref = mujoco_reference()
    ref_txt = "" if ref is None else f" (MuJoCo 같은 조건 30회: {ref * 100:.0f}%)"
    print(f"Isaac Sim, {args.layout}, Hs {args.hs * 100:.0f} cm, {args.strategy}: 성공률 {rate * 100:.0f}%{ref_txt}")
    out = OUT / f"isaac_{args.layout}_hs{args.hs * 100:.0f}_{args.strategy}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"layout": args.layout, "berth": args.berth, "hs": args.hs, "strategy": args.strategy,
                               "성공률": rate, "MuJoCo 성공률": ref, "시행": rows}, ensure_ascii=False, indent=1))
    app.close()


if __name__ == "__main__":
    main()
