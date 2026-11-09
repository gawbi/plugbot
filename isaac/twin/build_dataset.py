"""포토박스 트윈 3단계: 렌더 영상 + 실물 상태와 동작 → LeRobot 데이터셋(local/plug_twin).

화면(front, wrist)은 Isaac 트윈 렌더만 쓴다(실물 사진은 한 장도 넣지 않는다). 상태와 동작은 실물 시범 값 그대로(LeRobot 눈금).
이름과 형식은 실물 데이터셋(plug_static)과 같다: observation.state, action(6), observation.images.front, .wrist(480x640).
실행: ~/lerobot_env/bin/python isaac/twin/build_dataset.py [--src data/twin_src] [--render data/twin_render] [--repo plug_twin]
"""

import argparse
import json
import shutil
from pathlib import Path

import imageio.v2 as iio
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser()
ap.add_argument("--src", default=str(ROOT / "data/twin_src"))
ap.add_argument("--render", default=str(ROOT / "data/twin_render"))
ap.add_argument("--repo", default="plug_twin")
a = ap.parse_args()

REAL = Path.home() / ".cache/huggingface/lerobot/local/plug_static"
info = json.loads((REAL / "meta/info.json").read_text())
features = {k: v for k, v in info["features"].items() if k in ("action", "observation.state", "observation.images.front", "observation.images.wrist")}
root = Path.home() / ".cache/huggingface/lerobot/local" / a.repo
if root.exists():
    shutil.rmtree(root)
ds = LeRobotDataset.create(f"local/{a.repo}", fps=info["fps"], features=features, root=root, robot_type=info.get("robot_type", "so_follower"),
                           use_videos=True)
n_ep = 0
for f in sorted(Path(a.src).glob("ep_*.npz")):
    vf, vw = Path(a.render) / f"{f.stem}_front.mp4", Path(a.render) / f"{f.stem}_wrist.mp4"
    if not (vf.exists() and vw.exists()):
        continue
    e = np.load(f)
    fr, wr = iio.get_reader(vf), iio.get_reader(vw)
    for i, (imf, imw) in enumerate(zip(fr, wr)):
        if i >= len(e["state"]):
            break
        ds.add_frame({"observation.state": e["state"][i], "action": e["action"][i],
                      "observation.images.front": imf, "observation.images.wrist": imw, "task": str(e["task"])})
    ds.save_episode()
    n_ep += 1
    print(f"{n_ep} {f.stem}", flush=True)
ds.finalize() if hasattr(ds, "finalize") else None
print("완료", root, n_ep, "에피소드")
