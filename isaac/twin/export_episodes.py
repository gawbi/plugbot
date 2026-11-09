"""포토박스 트윈 1단계: 실물 정지 소켓 시범(LeRobot plug_static) → 에피소드별 npz.

넣는 것: 실물 상태와 동작(LeRobot 눈금 그대로, 학습 데이터에 그대로 쓴다), URDF 관절각(라디안, 트윈 재생용),
소켓 자리는 트윈(run_twin.py)이 Isaac 팔의 꽂힌 마지막 자세로 정한다(MuJoCo를 쓰지 않는다).
실행: ~/lerobot_env/bin/python isaac/twin/export_episodes.py [--dataset plug_static] [--out data/twin_src]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DEPTH = 0.031  # 꽂힌 깊이(모형), real_episode_traj.DEPTH_MODEL과 같다

ap = argparse.ArgumentParser()
ap.add_argument("--dataset", default="plug_static")
ap.add_argument("--out", default=str(ROOT / "data/twin_src"))
a = ap.parse_args()
D = Path.home() / ".cache/huggingface/lerobot/local" / a.dataset
CAL = json.loads((ROOT / "isaac/scaleup/calib/plug_follower.json").read_text())
out = Path(a.out)
out.mkdir(parents=True, exist_ok=True)


def to_rad(s):
    q = []
    for v, j in zip(s[:5], J):
        c = CAL[j]
        raw = c["range_min"] + (v + 100) / 200 * (c["range_max"] - c["range_min"])
        q.append((raw - 2048) * 360 / 4096)
    c = CAL["gripper"]  # 그리퍼 눈금은 0~100
    raw = c["range_min"] + s[5] / 100 * (c["range_max"] - c["range_min"])
    q.append((raw - 2048) * 360 / 4096)
    return np.deg2rad(q)


frames = pd.concat([pd.read_parquet(f) for f in sorted((D / "data").rglob("*.parquet"))]).sort_values("index")
eps = pd.read_parquet(sorted((D / "meta/episodes").rglob("*.parquet"))[0])
summary = []
for _, r in eps.iterrows():
    e = int(r.episode_index)
    f = frames[frames.episode_index == e]
    st = np.stack(f["observation.state"].to_numpy()).astype(np.float32)
    ac = np.stack(f["action"].to_numpy()).astype(np.float32)
    q = np.stack([to_rad(s) for s in st]).astype(np.float32)
    np.savez(out / f"ep_{e:03d}.npz", state=st, action=ac, q=q, ts=f["timestamp"].to_numpy(np.float32), task=r.tasks[0])
    summary.append((e, len(st)))
print(f"{len(summary)}개, 프레임 {sum(s[1] for s in summary)}")
(out / "summary.json").write_text(json.dumps(summary))
