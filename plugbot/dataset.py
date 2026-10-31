"""학습 데이터(에피소드) 기록기. MuJoCo와 Isaac Sim이 같이 쓴다.

에피소드 하나 = 폴더 하나:
    episode_0000/{카메라}.mp4      카메라별 영상(H.264)
    episode_0000/data.npz          state, action (프레임 × 6, 도), timestamp (s)
    episode_0000/meta.json         조건(파고, 시드, 성공 여부 등)
관절값 순서와 단위는 LeRobot SO-101(use_degrees=True)과 같다: 팔 5관절(도) + 그리퍼(0~100).
tools/to_lerobot.py가 이 폴더들을 LeRobot 데이터셋으로 바꾼다.
"""

import json
import shutil
from pathlib import Path

import numpy as np

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
GRIPPER_CLOSED = 2.0  # 플러그를 쥔 상태(real/robots.py의 gripper_close와 같은 값)


def to_lerobot_vector(q_arm_rad, gripper=GRIPPER_CLOSED):
    """MuJoCo/Isaac 관절각(rad, 5개) → LeRobot 상태 벡터(도 5개 + 그리퍼)."""
    return np.concatenate([np.rad2deg(np.asarray(q_arm_rad, dtype=np.float64)), [gripper]]).astype(np.float32)


class EpisodeWriter:
    def __init__(self, root, index, cameras, fps=30):
        import imageio.v2 as imageio
        self.dir = Path(root) / f"episode_{index:04d}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.fps, self.cameras = fps, list(cameras)
        self.writers = {c: imageio.get_writer(str(self.dir / f"{c}.mp4"), fps=fps, codec="libx264", quality=8,
                                              macro_block_size=1) for c in self.cameras}
        self.state, self.action, self.time = [], [], []

    def add(self, t, images, state, action):
        """images: {카메라: RGB uint8 배열}. state, action: 6개(도, 그리퍼)."""
        for c in self.cameras:
            self.writers[c].append_data(np.ascontiguousarray(images[c]))
        self.state.append(np.asarray(state, np.float32))
        self.action.append(np.asarray(action, np.float32))
        self.time.append(float(t))

    def __len__(self):
        return len(self.time)

    def _close_writers(self):
        for w in self.writers.values():
            w.close()

    def close(self, meta):
        self._close_writers()
        np.savez(self.dir / "data.npz", state=np.stack(self.state), action=np.stack(self.action),
                 timestamp=np.array(self.time) - self.time[0])
        meta = {**meta, "fps": self.fps, "cameras": self.cameras, "frames": len(self), "joint_names": JOINT_NAMES}
        (self.dir / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1))

    def discard(self):
        self._close_writers()
        shutil.rmtree(self.dir, ignore_errors=True)
