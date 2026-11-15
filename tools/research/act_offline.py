"""ACT 정책을 오프라인으로 돌리는 공용 함수(재생 확인, 실물 화면 평가, 설명 영상에서 같이 쓴다).

정책 로딩은 tools/policy_server.py와 같다. 다른 점은 한 스텝(select_action)이 아니라 100스텝 묶음 전체
(predict_action_chunk)를 받는다는 것. ACT 추론은 잠재 변수 z = 0이라 결정적이다.
"""

import json
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors

ROOT = Path(__file__).resolve().parents[2]
RUNS = Path.home() / "act_runs"
TASK = "Insert the plug into the socket"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAL = json.loads((ROOT / "isaac/scaleup/calib/plug_follower.json").read_text())


def ckpt_path(name):
    """'act_twin_v4/001000' → pretrained_model 폴더"""
    model, step = name.split("/")
    return RUNS / model / "checkpoints" / step / "pretrained_model"


class Policy:
    def __init__(self, name, device="cuda"):
        path = ckpt_path(name) if not Path(name).exists() else Path(name)
        cfg = PreTrainedConfig.from_pretrained(path)
        cfg.device = device
        self.name, self.device = name, device
        self.policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg).to(device).eval()
        self.pre, self.post = make_pre_post_processors(self.policy.config, path,
                                                       preprocessor_overrides={"device_processor": {"device": device}})
        self.chunk = self.policy.config.chunk_size

    @torch.inference_mode()
    def chunks(self, front, wrist, state):
        """front, wrist: (B, H, W, 3) uint8 RGB, state: (B, 6) 데이터셋 눈금 → (B, chunk, 6) 데이터셋 눈금"""
        obs = {"observation.images.front": torch.from_numpy(np.ascontiguousarray(front)).permute(0, 3, 1, 2).float() / 255.0,
               "observation.images.wrist": torch.from_numpy(np.ascontiguousarray(wrist)).permute(0, 3, 1, 2).float() / 255.0,
               "observation.state": torch.as_tensor(np.asarray(state), dtype=torch.float32),
               "task": [TASK] * len(state)}
        a = self.policy.predict_action_chunk(self.pre(obs))
        a = self.post(a)
        return a.detach().float().cpu().numpy()


def units_to_rad(s):
    """데이터셋 눈금(관절 -100~100, 그리퍼 0~100) → URDF 라디안. isaac/twin/export_episodes.py의 to_rad와 같다. s: (..., 6)"""
    s = np.asarray(s, float)
    q = np.zeros_like(s)
    for i, j in enumerate(JOINTS):
        c = CAL[j]
        raw = c["range_min"] + (s[..., i] + 100) / 200 * (c["range_max"] - c["range_min"])
        q[..., i] = (raw - 2048) * 360 / 4096
    c = CAL["gripper"]
    raw = c["range_min"] + s[..., 5] / 100 * (c["range_max"] - c["range_min"])
    q[..., 5] = (raw - 2048) * 360 / 4096
    return np.deg2rad(q)


# ---------- 핀 끝 순기구학 (isaac/twin/make_recovery.py와 같은 URDF, 같은 계산. 그 파일은 스크립트라 import할 수 없어 옮겨 둠) ----------
import xml.etree.ElementTree as ET  # noqa: E402

PIN = np.load(ROOT / "isaac/twin/twin_meshes.npz")["pin_tip"].astype(float)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(ax, t):
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


_urdf = ET.parse(ROOT / "isaac/so101/so101_new_calib_camera.urdf").getroot()
_by_child = {j.find("child").get("link"): j for j in _urdf.findall("joint")}
_chain, _link = [], "gripper_link"
while _link in _by_child:
    _j = _by_child[_link]
    _chain.insert(0, _j)
    _link = _j.find("parent").get("link")
_CH = []
for _j in _chain:
    _o = _j.find("origin")
    _T = np.eye(4)
    _T[:3, :3] = _rpy(*map(float, _o.get("rpy").split()))
    _T[:3, 3] = list(map(float, _o.get("xyz").split()))
    _ax = _j.find("axis")
    _CH.append((_j.get("name"), _T, np.array(list(map(float, _ax.get("xyz").split()))) if _ax is not None else None))


def pin_tip(q5):
    """5축 관절(라디안) → 덮개 핀 끝 위치(m, 로봇 기준)"""
    T = np.eye(4)
    for name, To, ax in _CH:
        T = T @ To
        if name in JOINTS:
            R = np.eye(4)
            R[:3, :3] = _axis_rot(ax, q5[JOINTS.index(name)])
            T = T @ R
    return T[:3, :3] @ PIN + T[:3, 3]


def tips_from_units(s):
    """데이터셋 눈금 (N, 6) → 핀 끝 (N, 3) m"""
    q = units_to_rad(s)
    return np.array([pin_tip(x[:5]) for x in q.reshape(-1, 6)]).reshape(*np.shape(s)[:-1], 3)
