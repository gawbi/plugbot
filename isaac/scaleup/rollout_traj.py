"""로봇 혼자 꽂기(ACT) 실행 기록 → UR10e 재생용 궤적(소켓 기준 상대 위치, 핀 방향). run_scaleup.py --replay 로 재생.

입력: data/rollouts/<실행>/state.npy (열: 시각 s, 관절 6개 보정 눈금 -100..100), result.json(녹화 화면 판정, 있으면)
변환: real_episode_traj.py와 같은 기구학(보정 눈금 → 원값 → 각도 → MuJoCo gripperframe).
소켓 위치:
  - 판정이 '꽂힘'이면: 가장 깊이 들어간 순간의 자세를 '보어 안 DEPTH_MODEL까지 꽂힘'으로 놓고 소켓을 정한다
    (사람이 눈으로 꽂힌 것을 확인했으므로 실제 소켓이 그 자리에 있었다).
  - 그 밖(못 꽂음, 판정 없음): 학습 데이터 가운데 자리 평균 소켓(teleop_socket_ref.json)을 쓴다. 빗나갔으면 UR10e도 빗나간다.
자르기: 꽂힌 뒤 팔이 빠지기 시작하면(깊이 8 mm 감소) 0.3 s 뒤에서 자른다. 실행 끝의 '처음 자세로 돌아가기'는 넣지 않는다.
실행: ~/lerobot_env/bin/python isaac/scaleup/rollout_traj.py 20261106_191627
결과: data/scaleup_real/rollout_<실행>.npz (t, rel, pin) 와 .json
"""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from plugbot.scene import Scene  # noqa: E402

DEPTH_MODEL = 0.031  # real_episode_traj.py, make_socket_ref.py와 같다
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAL = json.loads((HERE / "calib" / "plug_follower.json").read_text())
REF = json.loads((HERE / "teleop_socket_ref.json").read_text())

run = sys.argv[1]
d_run = ROOT / "data" / "rollouts" / run
S = np.load(d_run / "state.npy")
res_f = d_run / "result.json"
res = json.loads(res_f.read_text()) if res_f.exists() else None
ok = bool(res and res.get("ok"))

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


t = S[:, 0] - S[0, 0]
tips, jaws = map(np.array, zip(*(fk(s[1:6]) for s in S)))
ent0, B0 = np.array(REF["entrance_m"]), np.array(REF["basis"])
depth_ref = -((tips - ent0) @ B0)[:, 2]
k_in = int(np.argmax(depth_ref))

if ok:
    axis = -jaws[k_in]
    ent = tips[k_in] + axis * DEPTH_MODEL
    z = axis / np.linalg.norm(axis)
    x = np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], z) * z
    x /= np.linalg.norm(x)
    B = np.stack([x, np.cross(z, x), z], 1)
    socket = "꽂힘 판정: 가장 깊은 자세에서 소켓을 정함"
else:
    ent, B = ent0, B0
    socket = "기준 소켓(학습 데이터 가운데 자리 평균)"

rel = (tips - ent) @ B
pin = jaws @ B
depth = -rel[:, 2]
end = len(t)
after = np.nonzero(depth[k_in:] < depth[k_in] - 0.008)[0]
if len(after):
    k_out = k_in + int(after[0])
    end = int(np.searchsorted(t, t[k_out] + 0.3))
t, rel, pin = t[:end], rel[:end], pin[:end]

out = ROOT / "data" / "scaleup_real"
out.mkdir(parents=True, exist_ok=True)
stem = f"rollout_{run}"
np.savez(out / f"{stem}.npz", t=t, rel=rel, pin=pin)
info = {"run": run, "policy": json.loads((d_run / "meta.json").read_text()).get("policy", ""),
        "judged": None if res is None else ("꽂힘" if ok else "못 꽂음"), "cond": None if res is None else res.get("cond"),
        "frames": int(len(t)), "duration_s": round(float(t[-1]), 2), "t_deepest_s": round(float(t[k_in]), 2),
        "depth_ref_mm": round(float(depth_ref[k_in] * 1000), 1), "socket": socket, "depth_model_m": DEPTH_MODEL,
        "note": "로봇 혼자 꽂기(ACT) 실행의 관절 기록 → 기구학 → 소켓 기준. 정지 소켓에서만 정확(흔들림 실행은 소켓이 움직임)"}
(out / f"{stem}.json").write_text(json.dumps(info, ensure_ascii=False, indent=1))
print(json.dumps(info, ensure_ascii=False))
