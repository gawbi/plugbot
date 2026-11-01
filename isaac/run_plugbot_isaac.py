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
ap.add_argument("--max-effort", type=float, default=None, help="관절 최대 토크 N m(기본: URDF effort 10). MuJoCo 서보는 2.94")
ap.add_argument("--reach", type=float, default=0.0,
                help="0보다 크면 팔을 세운 홈 자세(plugbot.control.HOME)에서 출발해 이 시간(s) 동안 소켓 앞 시작 자세로 다가간 뒤 꽂는다"
                     "(영상, 학습 데이터용. 성공률 실험은 0으로 MuJoCo와 같게)")
ap.add_argument("--armature", type=float, default=None, help="관절 armature kg m^2(기본: 없음). MuJoCo SO-101 모델은 0.028")
ap.add_argument("--tag", default="", help="결과 파일 이름 뒤에 붙인다(본 결과를 덮어쓰지 않게)")
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
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

from plugbot.control import CONTROL_DT, HOME, IK, Params, SocketEstimator, contacts, home_pose  # noqa: E402
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
# [ISAAC] URDF에 inertial이 없는 좌표용 링크(plug_tip, wrist_cam_link)는 PhysX가 기본 질량 1 kg을 넣어 팔이 처진다.
# MuJoCo에서는 질량 없는 site이므로 거의 0으로 둔다(URDF의 gripper_frame_link와 같은 방식).
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/World/SO101") and p.GetName() in ("plug_tip", "wrist_cam_link") \
            and p.HasAPI(UsdPhysics.RigidBodyAPI):
        mass = UsdPhysics.MassAPI.Apply(p)
        mass.CreateMassAttr(1e-6)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-12, 1e-12, 1e-12))
    # [ISAAC] URDF 가져오기 기본 드라이브는 acceleration(강성 × 관성)이라 가벼운 SO-101은 중력에 0.1 rad 넘게 처진다.
    # MuJoCo position 액추에이터와 같게 force(토크 = kp × 오차)로 바꾼다.
    if p.GetPath().pathString.startswith("/World/SO101") and p.HasAPI(UsdPhysics.DriveAPI):
        UsdPhysics.DriveAPI.Get(p, "angular").GetTypeAttr().Set("force")
        if args.max_effort is not None:
            UsdPhysics.DriveAPI.Get(p, "angular").GetMaxForceAttr().Set(args.max_effort)
        if args.armature is not None:
            PhysxSchema.PhysxJointAPI.Apply(p).CreateArmatureAttr(args.armature)


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
        ds_cams["front"] = Camera(prim_path="/World/Env/Webcam", resolution=(CAM_W, CAM_H))  # [ISAAC]
        if wrist_path:
            ds_cams["wrist"] = Camera(prim_path=wrist_path, resolution=(CAM_W, CAM_H))  # [ISAAC]
    else:
        for cname in ("Webcam", "ShowCam"):
            cams[cname] = Camera(prim_path=f"/World/Env/{cname}", resolution=(1280, 720))  # [ISAAC]
        if wrist_path:
            cams["Wrist"] = Camera(prim_path=wrist_path, resolution=(CAM_W, CAM_H))  # [ISAAC]
ship = SingleXFormPrim("/World/Env/Ship", name="ship")  # [ISAAC] 참조 안의 경로

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


# 충돌 판정: PhysX 접촉 API(RigidPrim 접촉 뷰)는 이 노트북에서 세그폴트가 나서, MuJoCo 실험과 같은 함수
# (plugbot.control.contacts)를 Isaac의 관절각과 선박 자세에 적용한다. 물리 진행은 Isaac, 판정 규칙은 MuJoCo와 같다.
judge = Scene("ship", args.layout)
GRIP = robot.get_dof_index("gripper")  # [ISAAC]
GRIP_ADR = judge.model.joint("gripper").qposadr[0]


def collision(motion, t):
    """MuJoCo plugbot.control.run_trial과 같은 분류. 접촉이 없으면 None."""
    q = np.asarray(robot.get_joint_positions())  # [ISAAC]
    judge.data.qpos[judge.qadr] = q[dof]
    judge.data.qpos[GRIP_ADR] = q[GRIP]
    judge.set_ship(*motion(t))
    mujoco.mj_forward(judge.model, judge.data)
    body_hit, plug_hull, _ = contacts(judge)
    if body_hit:
        return "그리퍼가 소켓 테두리에 닿음" if body_hit == "socket" else "로봇 몸체 충돌"
    if plug_hull:
        return "플러그가 소켓 밖 선체에 부딪힘"
    return None


def reset_robot():
    q0 = HOME if args.reach > 0 else START
    world.reset()  # [ISAAC]
    robot.set_joint_positions(q0, joint_indices=np.array(dof))  # [ISAAC]
    set_arm(q0)


def run_trial(motion, strategy, p=Params(), seed=0, video=None, rec=None):
    """plugbot.control.run_trial과 같은 상태 기계. (성공, 이유, 삽입 시각, 첫 충돌(이유, 시각) 또는 None).
    video가 있으면 녹화하고 성공 뒤 1초 더 찍는다. 충돌은 기록만 하고 시행을 끝내지 않는다(MuJoCo는 끝낸다)."""
    rng = np.random.default_rng(seed)
    reset_robot()
    if video is not None or rec is not None:  # [ISAAC] RTX 렌더가 한두 장 늦게 따라와 첫 장에 이전 자세가 흐리게 찍힌다
        pos, quat, _, _ = ship_pose(args.layout, motion, 0.0)
        ship.set_world_pose(position=pos, orientation=quat)
        for _ in range(6):  # 팔은 START를 유지하고 선박은 t = 0 자세로 둔 채 몇 장 그린다(물리 12 ms, 시각 t는 0부터)
            world.step(render=True)
    est = SocketEstimator(strategy, p, rng)
    phase, depth_cmd, gate_t, hold_t, t = "approach", -p.hover, None, 0.0, 0.0
    hit = None
    every = max(1, int(round(1.0 / args.video_fps / CONTROL_DT)))
    tick, done, tail, q_cmd = 0, None, 0, START.copy()
    while t < p.timeout + (1.0 if done else 0.0):
        _, _, entrance, axis = ship_pose(args.layout, motion, t)
        est.capture(t, entrance, axis)
        e = est.estimate(t)
        q = arm_q()
        if t < args.reach:  # 홈 → 시작 자세(관절 공간 smoothstep). 이 경로는 MuJoCo에서 충돌 없음을 확인했다
            u = t / args.reach
            q_cmd = HOME + (START - HOME) * (u * u * (3 - 2 * u))
            set_arm(q_cmd)
        elif e is not None:
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
        if hit is None and done is None:  # MuJoCo도 제어 주기마다 접촉을 본다
            c = collision(motion, t)
            if c is not None:
                hit = (c, t)
        tick += 1
        if video is not None and tick % every == 0:
            video.grab()
        if done is not None:
            tail += 1
            if video is None or tail * CONTROL_DT >= 1.0:
                return done + (hit,)
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
                    return done + (hit,)
        else:
            hold_t = 0.0
    return (done or (False, "시간 초과(걸림, 정렬 실패)", float("nan"))) + (hit,)


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
        ok, why, _, _ = run_trial(motion_for(hs, 1000 + k), args.strategy, seed=1000 + k, rec=Recorder(writer))
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
        ok, why, t_ins, hit = run_trial(motion_for(args.hs, k), args.strategy, seed=k, video=video)
        if video is not None:
            video.close()
        ok_mj = ok and (hit is None or hit[1] > t_ins)  # MuJoCo는 꽂기 전에 부딪히면 그 자리에서 실패로 끝낸다
        rows.append({"시드": k, "성공": ok, "이유": why, "삽입 시각 s": t_ins,
                     "첫 충돌": None if hit is None else hit[0], "첫 충돌 시각 s": None if hit is None else hit[1],
                     "MuJoCo 기준 성공": ok_mj})
        hit_txt = "" if hit is None else f" (충돌 {hit[1]:.2f} s: {hit[0]})"
        print(f"[{k - args.seed + 1}/{args.trials}] 시드 {k} {'성공' if ok else '실패'} {why}{hit_txt}")
    rate = float(np.mean([r["성공"] for r in rows]))
    rate_mj = float(np.mean([r["MuJoCo 기준 성공"] for r in rows]))
    ref = mujoco_reference()
    ref_txt = "" if ref is None else f" (MuJoCo 같은 조건 30회: {ref * 100:.0f}%)"
    print(f"Isaac Sim, {args.layout}, Hs {args.hs * 100:.0f} cm, {args.strategy}: 성공률 {rate * 100:.0f}%, "
          f"충돌을 실패로 세면 {rate_mj * 100:.0f}%{ref_txt}")
    tag = f"_{args.tag}" if args.tag else ""
    out = OUT / f"isaac_{args.layout}_hs{args.hs * 100:.0f}_{args.strategy}{tag}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"layout": args.layout, "berth": args.berth, "hs": args.hs, "strategy": args.strategy,
                               "kp": args.kp, "kd": args.kd, "max_effort": args.max_effort, "armature": args.armature,
                               "성공률": rate, "충돌을 실패로 센 성공률": rate_mj, "MuJoCo 성공률": ref,
                               "시행": rows}, ensure_ascii=False, indent=1))
    app.close()


if __name__ == "__main__":
    main()
