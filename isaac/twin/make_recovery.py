"""포토박스 트윈 회복 시범 합성: 실물 시범 궤적에 '한 번 빗나감 → 뒤로 빼기 → 바로잡고 다시 꽂기'를 넣는다.

사람이 실제로 한 회복 동작이 아니다(합성). 실물 시범 궤적(data/twin_src/ep_XXX.npz)을 바탕으로:
  1) 접근 구간(소켓 앞 6 cm부터)에 옆으로 8~20 mm 빗나감(절반은 로봇 오른쪽)을 서서히 넣어, 핀이 소켓 앞면(흰 링)에 닿아 멈춘다
  2) 0.3~0.5초 밀고 있다가 3 cm 뒤로 뺀다
  3) 관절 공간에서 실물 궤적의 '소켓 앞 3 cm' 자세로 옮겨 가고, 거기부터는 실물 궤적 그대로 꽂는다
관절값은 URDF 순기구학(numpy)과 감쇠 최소제곱 역기구학(핀 끝 위치 + 그리퍼 자세 유지)으로 푼다.
상태는 관절값을 LeRobot 눈금으로 되돌린 값, 동작은 실물 시범의 '동작이 상태보다 L장면 앞서고 b만큼 더 민다' 관계로 만든다.
소켓 자리는 run_twin.py가 마지막 장면(실물 꽂힌 자세)으로 정하므로 원래 에피소드와 같다.
결과: data/twin_src_rec/ep_1XXX.npz (번호에 1000을 더해 렌더 조명 무작위 시드가 원래 판과 다르게)
실행: ~/lerobot_env/bin/python isaac/twin/make_recovery.py
"""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser()
ap.add_argument("--src", default=str(ROOT / "data/twin_src"))
ap.add_argument("--out", default=str(ROOT / "data/twin_src_rec"))
ap.add_argument("--seed", type=int, default=7)
a = ap.parse_args()

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAL = json.loads((ROOT / "isaac/scaleup/calib/plug_follower.json").read_text())
PIN = np.load(ROOT / "isaac/twin/twin_meshes.npz")["pin_tip"].astype(float)
INSERT = 0.020  # run_twin.py와 같다
FPS = 30

# ---------- URDF 순기구학 (base → gripper_link) ----------
def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rot(ax, t):
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


urdf = ET.parse(ROOT / "isaac/so101/so101_new_calib_camera.urdf").getroot()
joints = {j.get("name"): j for j in urdf.findall("joint")}
by_child = {j.find("child").get("link"): j for j in urdf.findall("joint")}
chain, link = [], "gripper_link"
while link in by_child:
    j = by_child[link]
    chain.insert(0, j)
    link = j.find("parent").get("link")
CH = []
for j in chain:
    o = j.find("origin")
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*map(float, o.get("rpy").split()))
    T[:3, 3] = list(map(float, o.get("xyz").split()))
    ax = j.find("axis")
    CH.append((j.get("name"), T, np.array(list(map(float, ax.get("xyz").split()))) if ax is not None else None))
assert [c[0] for c in CH if c[0] in J] == J, [c[0] for c in CH]


def fk(q5):
    """gripper_link 자세(4x4)"""
    T = np.eye(4)
    for name, To, ax in CH:
        T = T @ To
        if name in J:
            R = np.eye(4)
            R[:3, :3] = axis_rot(ax, q5[J.index(name)])
            T = T @ R
    return T


def tip_jaw(q5):
    G = fk(q5)
    return G[:3, :3] @ PIN + G[:3, 3], -G[:3, 2], G[:3, :3]


def rot_err(Rt, R):
    E = Rt @ R.T
    return 0.5 * np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])


def ik(q0, p_t, R_t, iters=30, lam=1e-3, w_rot=0.05):
    """핀 끝을 p_t로, 그리퍼 자세는 R_t에 가깝게(5축이라 자세는 가중치를 낮춰 최소제곱)"""
    q = q0.copy()
    for _ in range(iters):
        p, _, R = tip_jaw(q)
        e = np.r_[p_t - p, w_rot * rot_err(R_t, R)]
        if np.linalg.norm(e[:3]) < 2e-4:
            break
        Jm = np.zeros((6, 5))
        for k in range(5):
            dq = np.zeros(5); dq[k] = 1e-5
            p2, _, R2 = tip_jaw(q + dq)
            Jm[:3, k] = (p2 - p) / 1e-5
            Jm[3:, k] = w_rot * rot_err(R2, R) / 1e-5
        q = q + Jm.T @ np.linalg.solve(Jm @ Jm.T + lam * np.eye(6), e)
    return q, np.linalg.norm(p_t - tip_jaw(q)[0])


# ---------- LeRobot 눈금 ↔ 라디안 (export_episodes.to_rad의 역) ----------
def to_state(q):
    s = []
    for v, j in zip(np.rad2deg(q[:5]), J):
        c = CAL[j]
        raw = v * 4096 / 360 + 2048
        s.append((raw - c["range_min"]) / (c["range_max"] - c["range_min"]) * 200 - 100)
    c = CAL["gripper"]
    raw = np.rad2deg(q[5]) * 4096 / 360 + 2048
    s.append((raw - c["range_min"]) / (c["range_max"] - c["range_min"]) * 100)
    return np.array(s, np.float32)


def smooth(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


rng = np.random.default_rng(a.seed)
out = Path(a.out)
out.mkdir(parents=True, exist_ok=True)
summary = []
for f in sorted(Path(a.src).glob("ep_*.npz")):
    e = np.load(f)
    q, st, ac = e["q"].astype(float), e["state"], e["action"]
    assert np.abs(to_state(q[10]) - st[10]).max() < 0.05, "눈금 변환이 맞지 않는다"
    tip_end, jaw_end, _ = tip_jaw(q[-1, :5])
    d = jaw_end / np.linalg.norm(jaw_end)  # 꽂는 방향(핀이 향하는 쪽)
    E = tip_end - d * INSERT  # 소켓 입구(앞면 중심)
    P = np.array([tip_jaw(x[:5])[0] for x in q])
    s = (P - E) @ d  # 입구 기준 깊이(음수 = 앞)
    hit = np.nonzero(s > -0.002)[0]
    if len(hit) == 0:
        print(f.stem, "꽂는 구간을 못 찾음, 건너뜀"); continue
    k_hit = int(hit[0])
    before = np.nonzero(s[:k_hit] < -0.06)[0]
    k0 = int(before[-1]) if len(before) else 0
    back = np.nonzero(s[:k_hit] < -0.03)[0]
    k_r = int(back[-1]) if len(back) else k0
    if k_hit - k0 < 8:
        print(f.stem, "접근 구간이 너무 짧음, 건너뜀"); continue
    # 빗나감: 앞면에 닿는 순간 핀 중심이 소켓 축에서 8~20 mm 옆(구멍 반지름 14.5 mm, 핀 13.75 mm라 1 mm만 어긋나도 안 들어간다)
    # 절반은 로봇 기준 오른쪽(-y, ±25도). 왼쪽 앞 카메라의 시선과 거의 같은 방향이라 화면에서 잘 안 보이는 빗나감(실물 v2 시험에서 나온 실패)
    # 나머지 절반은 실물이 이미 옆으로 와 있던 쪽(±60도), 없으면 무작위
    right = np.array([0.0, -1.0, 0.0]) - d * (-d[1])
    right /= np.linalg.norm(right)
    lat_hit = (P[k_hit] - E) - s[k_hit] * d
    if rng.random() < 0.5:
        side = "right"
        v, ang = right, rng.uniform(-np.pi * 25 / 180, np.pi * 25 / 180)
    elif np.linalg.norm(lat_hit) > 0.003:
        side = "real"
        v, ang = lat_hit / np.linalg.norm(lat_hit), rng.uniform(-np.pi / 3, np.pi / 3)
    else:
        side = "rand"
        v = np.cross(d, rng.normal(size=3)); v /= np.linalg.norm(v); ang = 0.0
    u = np.cos(ang) * v + np.sin(ang) * np.cross(d, v)
    mag = float(rng.uniform(0.008, 0.020))
    Lh = u * mag  # 앞면에서의 옆 어긋남(축 기준 절대값)
    # 1) 빗나간 접근: 실물 접근 구간 + 옆으로 delta(앞 60%에서 서서히), 깊이는 앞면에서 멈춤
    syn, n_ramp = [], max(1, int(0.6 * (k_hit - k0)))
    qc = q[k0, :5].copy()
    for t in range(k0, k_hit + 1):
        _, _, Rt = tip_jaw(q[t, :5])
        lat = (P[t] - E) - s[t] * d
        w = smooth((t - k0) / n_ramp)
        tgt = E + lat * (1 - w) + Lh * w + d * min(s[t], -0.001)
        qc, err = ik(qc, tgt, Rt)
        syn.append(qc.copy())
    p_hit, R_hit = tip_jaw(qc)[0], tip_jaw(q[k_hit, :5])[2]
    # 2) 앞면에 대고 밀고 있기(사람이 '어, 안 들어가네' 하는 시간)
    n_hold = int(rng.integers(9, 16))
    for i in range(n_hold):
        qc, _ = ik(qc, p_hit + rng.normal(0, 0.0004, 3), R_hit)
        syn.append(qc.copy())
    # 3) 3 cm 뒤로 빼기
    n_back = int(rng.integers(16, 25))
    for i in range(1, n_back + 1):
        qc, _ = ik(qc, p_hit - d * 0.03 * smooth(i / n_back), R_hit)
        syn.append(qc.copy())
    # 4) 잠깐 멈췄다가 관절 공간에서 실물의 '앞 3 cm' 자세로
    syn += [qc.copy()] * int(rng.integers(3, 8))
    n_fix = int(rng.integers(18, 27))
    q_from = qc.copy()
    for i in range(1, n_fix + 1):
        syn.append(q_from + (q[k_r, :5] - q_from) * smooth(i / n_fix))
    syn = np.array(syn)
    g = q[k_hit, 5]
    q_new = np.concatenate([q[:k0], np.c_[syn, np.full(len(syn), g)], q[k_r + 1:]]).astype(np.float32)
    # 상태: 실물 구간은 원래 값, 합성 구간은 관절값에서
    st_syn = np.array([to_state(x) for x in q_new[k0:k0 + len(syn)]])
    st_new = np.concatenate([st[:k0], st_syn, st[k_r + 1:]]).astype(np.float32)
    # 동작: 실물 시범에서 '동작 ≈ L장면 뒤 상태 + b'의 L, b를 구해 합성 구간에 쓴다
    errs = [np.abs(ac[:len(ac) - L, :5] - st[L:, :5]).mean() for L in range(0, 11)]
    L = int(np.argmin(errs))
    b_app = np.median(ac[k0:k_hit - L] - st[k0 + L:k_hit], axis=0) if k_hit - L > k0 + 2 else np.zeros(6)
    b_push = np.median(ac[-30:] - st[-30:], axis=0)  # 꽂은 채 밀고 있을 때(팔로워가 막힌 만큼 리더가 더 감)
    ac_new = np.concatenate([ac[:k0], np.zeros((len(syn), 6)), ac[k_r + 1:]]).astype(np.float32)
    n_att = k_hit - k0 + 1
    for i in range(len(syn)):
        t = k0 + i
        j = min(t + L, len(st_new) - 1)
        b = b_push if n_att <= i < n_att + n_hold else b_app
        ac_new[t] = st_new[j] + b
    T = len(q_new)
    np.savez(out / f"ep_{1000 + int(f.stem[3:]):04d}.npz", state=st_new, action=ac_new, q=q_new,
             ts=(np.arange(T) / FPS).astype(np.float32), task=e["task"])
    rec = dict(src=f.stem, side=side, k0=k0, k_hit=k_hit, k_r=k_r, miss_mm=round(mag * 1000, 1), lag=L, frames=T, added=T - len(q),
               ik_err_mm=round(float(err) * 1000, 2))
    summary.append(rec)
    print(rec, flush=True)
(out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
print(f"{len(summary)}개, 프레임 {sum(r['frames'] for r in summary)}")
