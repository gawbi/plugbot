"""이종 원격조종용 소켓 기준(teleop_socket_ref.json): 학습 데이터 '가운데' 자리의 꽂힌 마지막 자세 평균으로 소켓 입구와 축을 정한다.
실행(GPU 노트북, lerobot 환경): ~/lerobot_env/bin/python isaac/scaleup/make_socket_ref.py
real_episode_traj.py와 같은 기구학, 같은 깊이(DEPTH_MODEL 31 mm).
"""

import json
import sys
from pathlib import Path

import numpy as np

import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from plugbot.scene import Scene  # noqa: E402

DEPTH_MODEL = 0.031  # real_episode_traj.py와 같다
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAL = json.loads((HERE / "calib" / "plug_follower.json").read_text())
kin = Scene("car", "side")
m, d = kin.model, mujoco.MjData(kin.model)
site = m.site("gripperframe").id


def fk(n):
    deg = [(CAL[j]["range_min"] + (v + 100) / 200 * (CAL[j]["range_max"] - CAL[j]["range_min"]) - 2048) * 360 / 4096 for v, j in zip(n, J)]
    d.qpos[:] = 0
    d.qpos[kin.qadr] = np.deg2rad(deg)
    mujoco.mj_kinematics(m, d)
    tip = d.site_xpos[site].copy()
    jaw = tip - d.xpos[kin.gripper_body]
    return tip, jaw / np.linalg.norm(jaw)

from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402

root = Path.home() / ".cache/huggingface/lerobot/local/plug_static"
ds = LeRobotDataset("local/plug_static", root=root)
e = ds.meta.episodes
pos = json.loads((root / "plugbot_positions.json").read_text())
s = np.stack(ds.hf_dataset["observation.state"])
eps = [i for i in range(54, ds.num_episodes) if pos.get(str(i)) == "가운데"]
tips, jaws = zip(*(fk(s[int(e["dataset_to_index"][i]) - 1]) for i in eps))
tip, jaw = np.mean(tips, 0), np.mean(jaws, 0)
jaw /= np.linalg.norm(jaw)
axis = -jaw
ent = tip + axis * DEPTH_MODEL
z = axis
x = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z) * z
x /= np.linalg.norm(x)
B = np.stack([x, np.cross(z, x), z], 1)
out = {"note": "SO-101 베이스 기준(m). 가운데 자리 꽂힌 자세 평균으로 정한 소켓 입구와 좌표계(열: x 위쪽, y, z 축 바깥)",
       "episodes": eps, "entrance_m": ent.round(5).tolist(), "basis": B.round(6).tolist(),
       "spread_mm": round(float(np.std(np.array(tips), 0).mean() * 1000), 2), "depth_model_m": DEPTH_MODEL}
(HERE / "teleop_socket_ref.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
print(out)
