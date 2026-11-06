"""실물 SO-101 시범(LeRobot 데이터셋 에피소드) → UR10e 재생용 궤적(소켓 기준 상대 위치, 핀 방향). run_scaleup.py --replay-traj 로 재생.

변환:
1. 관절: LeRobot 보정 눈금(-100..100) → 모터 원값(보정 파일 range_min..range_max) → 각도((원값 - 2048) × 360/4096).
   보정 때 팔을 가운데 자세에 두면 원값이 2048이 되므로, 공식 URDF(so101_new_calib)의 0 자세와 같다.
   확인(2026-11-06): plug_static 54~68번의 꽂힌 위치가 자리별로 4~5 mm 안에 모이고, 왼쪽 +22 mm, 오른쪽 -12 mm(옆), 높이는 같다.
2. 끝점: MuJoCo 기구학(plugbot.scene)의 gripperframe. 핀 방향: 그리퍼 몸체 원점 → gripperframe(턱이 가리키는 쪽).
3. 소켓: 실물 소켓 위치는 따로 재지 않았다. 꽂힌 마지막 자세를 '보어 안 DEPTH_MODEL까지 꽂힘'으로 놓고 소켓 입구와 축을 정한다.
   그래서 정지 소켓 시범에서만 정확하다(흔들림 시범은 소켓이 움직여 이 가정이 틀린다).
실행: ~/lerobot_env/bin/python isaac/scaleup/real_episode_traj.py <에피소드 번호> [--dataset plug_static]
결과: data/scaleup_real/<데이터셋>_ep<번호>.npz (t, rel, pin) 와 .json
"""

import argparse
import glob
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from plugbot.scene import Scene  # noqa: E402

DEPTH_MODEL = 0.031  # 모형 크기에서 꽂힌 깊이. run_scaleup.DEPTH_GOAL(모형 29 mm × 5)보다 조금 깊게
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

ap = argparse.ArgumentParser()
ap.add_argument("episode", type=int)
ap.add_argument("--dataset", default="plug_static")
args = ap.parse_args()

cal_path = glob.glob(str(Path.home() / ".cache/huggingface/lerobot/calibration/robots/*/plug_follower.json"))[0]
CAL = json.loads(Path(cal_path).read_text())


def to_deg(n):
    out = []
    for v, j in zip(n, J):
        c = CAL[j]
        raw = c["range_min"] + (v + 100) / 200 * (c["range_max"] - c["range_min"])
        out.append((raw - 2048) * 360 / 4096)
    return np.array(out)


kin = Scene("car", "side")
m = kin.model
d = mujoco.MjData(m)
site = m.site("gripperframe").id


def fk(n):
    d.qpos[:] = 0
    d.qpos[kin.qadr] = np.deg2rad(to_deg(n))
    mujoco.mj_kinematics(m, d)
    tip = d.site_xpos[site].copy()
    jaw = tip - d.xpos[kin.gripper_body]
    return tip, jaw / np.linalg.norm(jaw)


from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402

root = Path.home() / ".cache/huggingface/lerobot/local" / args.dataset
ds = LeRobotDataset(f"local/{args.dataset}", root=root)
e = ds.meta.episodes
a, b = int(e["dataset_from_index"][args.episode]), int(e["dataset_to_index"][args.episode])
state = np.stack(ds.hf_dataset["observation.state"][a:b])
ts = np.asarray(ds.hf_dataset["timestamp"][a:b], dtype=float)
ts -= ts[0]

tips, jaws = zip(*(fk(s) for s in state))
tips, jaws = np.array(tips), np.array(jaws)
axis = -jaws[-1]                      # 소켓 축(바깥쪽) = 마지막 핀 방향의 반대
ent = tips[-1] + axis * DEPTH_MODEL   # 꽂힌 끝점에서 축 바깥으로 DEPTH_MODEL = 소켓 입구
z = axis / np.linalg.norm(axis)
x = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z) * z
x /= np.linalg.norm(x)
B = np.stack([x, np.cross(z, x), z], 1)  # run_scaleup.socket_basis와 같은 정의
rel = (tips - ent) @ B
pin = jaws @ B

out = ROOT / "data" / "scaleup_real"
out.mkdir(parents=True, exist_ok=True)
stem = f"{args.dataset}_ep{args.episode}"
np.savez(out / f"{stem}.npz", t=ts, rel=rel, pin=pin)
info = {"dataset": args.dataset, "episode": args.episode, "frames": int(len(ts)), "duration_s": round(float(ts[-1]), 2),
        "start_dist_mm": round(float(np.linalg.norm(rel[0]) * 1000), 1), "depth_model_m": DEPTH_MODEL,
        "note": "실물 시범 관절 → 기구학 → 소켓 기준(꽂힌 마지막 자세에서 정함). 정지 소켓 시범에서만 정확"}
(out / f"{stem}.json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
print(info)
print("깊이(mm, 축 안쪽 +) 처음, 중간, 끝:", np.round(-rel[[0, len(rel) // 2, -1], 2] * 1000, 1))
