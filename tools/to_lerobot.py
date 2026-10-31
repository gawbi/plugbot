"""에피소드 폴더(plugbot.dataset.EpisodeWriter 형식) → LeRobot 0.6.1 데이터셋.

MuJoCo(make_sim_dataset.py)와 Isaac Sim(isaac/run_plugbot_isaac.py --dataset)이 만든 폴더를 실물 녹화
(lerobot-record, SO-101 팔로워)와 같은 이름과 단위로 바꾼다: observation.state, action은
shoulder_pan.pos ... gripper.pos(도, 그리퍼 0~100), 카메라는 observation.images.{front, wrist}.

실행(lerobot 환경): python tools/to_lerobot.py data/sim_side --repo-id <HF아이디>/plugbot_sim_side [--root 저장폴더]
"""

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="episode_XXXX 폴더들이 있는 곳")
    ap.add_argument("--repo-id", required=True)
    ap.add_argument("--root", default=None, help="기본: ~/.cache/huggingface/lerobot/<repo-id>")
    ap.add_argument("--task", default=None, help="기본: meta.json의 task")
    ap.add_argument("--push", action="store_true", help="허깅페이스 허브에 올린다(hf auth login 필요)")
    a = ap.parse_args()
    eps = sorted(Path(a.src).glob("episode_*"))
    if not eps:
        raise SystemExit(f"{a.src}에 episode_* 폴더가 없다")
    meta0 = json.loads((eps[0] / "meta.json").read_text())
    cams = meta0["cameras"]
    shapes = {c: imageio.get_reader(str(eps[0] / f"{c}.mp4")).get_data(0).shape for c in cams}
    hw = {f"{j}.pos": float for j in meta0["joint_names"]}
    features = {**hw_to_dataset_features(hw, "action"),
                **hw_to_dataset_features({**hw, **{c: tuple(s) for c, s in shapes.items()}}, "observation")}
    ds = LeRobotDataset.create(a.repo_id, fps=meta0["fps"], features=features, root=a.root,
                               robot_type="so_follower", use_videos=True)
    for ep in eps:
        meta = json.loads((ep / "meta.json").read_text())
        z = np.load(ep / "data.npz")
        frames = {c: imageio.get_reader(str(ep / f"{c}.mp4")) for c in cams}
        task = a.task or meta.get("task", "Insert the plug into the socket")
        for i in range(len(z["state"])):
            f = {"observation.state": z["state"][i].astype(np.float32), "action": z["action"][i].astype(np.float32),
                 "task": task}
            for c in cams:
                f[f"observation.images.{c}"] = frames[c].get_data(i)
            ds.add_frame(f)
        ds.save_episode()
        print(f"{ep.name}: {len(z['state'])}프레임", flush=True)
    ds.finalize()
    print(f"완료: {ds.root} (에피소드 {len(eps)}개)")
    if a.push:
        ds.push_to_hub()


if __name__ == "__main__":
    main()
