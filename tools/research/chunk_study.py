"""E3 묶음 실행 시뮬레이션: ACT처럼 H초 분량 계획을 한 번에 세우고 그동안 눈을 감고 실행하면, 움직이는 소켓에서 얼마나 손해인가.

plugbot/control.py를 바꾸지 않고, 같은 장면, 같은 판정(contacts, 성공 기준), 같은 카메라 모형(SocketEstimator)을 쓴다.
다른 점은 제어기다.
  - 다시 계획하는 순간(t0)에만 관측을 본다: 카메라 관측(지연 0.1 s, 30 Hz, 잡음)과 팔의 현재 핀 끝(고유 감각, ACT의 state와 같음).
  - 그 순간 [t0, t0 + H]의 소켓 경로를 정한다
      follow*  : 가장 최근 관측을 그대로 둔다(정지한 것으로 본다)
      predict* : 자기회귀(AR) 모형으로 묶음 끝까지 내다본다(원래 예측 전략과 같은 lead 보정)
  - 단계 판단(접근 → 대기 → 꽂기)도 계획 안에서 한다. t0에 정렬이 안 돼 있으면 남은 거리 / V_CONV초 뒤 정렬된다고 보고,
    같은 묶음 안에서 대기와 꽂기까지 이어서 계획한다(ACT가 접근부터 꽂기까지 한 묶음으로 배우는 것을 흉내).
      *_gate   : 소켓이 잠잠할 때만 꽂기 시작(predict는 예측 속도, follow_gate는 t0에 본 속도)
      나머지    : 정렬되면 바로 꽂는다
  - 계획한 목표마다 IK를 이전 명령에서 이어 풀어 관절 명령 열을 만들고, 그대로 보낸다(실제 핀 끝, 소켓을 안 본다).
  - *_te: 매 스텝 계획하되, 같은 시각을 겨냥한 겹친 계획들의 목표를 평균한다(ACT 시간 앙상블, 계수 0.01).
각본 제어기이지 ACT가 아니다. 엔진(MuJoCo)에 따라 절대값이 달라질 수 있어 같은 시드끼리의 상대 비교만 말한다.
실행: .venv/bin/python tools/research/chunk_study.py [--n 100] [--jobs 12]
결과: outputs/research/chunk_study.json, chunk_study.png
"""

import argparse
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from plugbot.control import CONTROL_DT, IK, Params, SocketEstimator, Trial, ar_predict, contacts, home_pose  # noqa: E402
from plugbot.scene import BORE_R, PIN_R, SOCKET_SIDE_ON_SHIP, Scene  # noqa: E402
from plugbot.waves import HINGE_BELOW_SIDE_SOCKET, HeaveProfile, HingeDeckMotion, ShipMotion  # noqa: E402

V_CONV = 0.05  # 정렬이 안 된 채 계획할 때, 대기 지점까지 남은 거리를 이 속도(m/s)로 좁힌다고 보고 정렬 시각을 정한다
TE_COEF = 0.01
G = {}


def init():
    G["scene"] = Scene("ship", "side")
    G["home"] = home_pose(G["scene"])


def motion_of(wave, seed):
    if wave == "deck_sine":  # 실물 갑판 기본 설정: 정현파 12 mm, 2.5 s, 상하. 시드마다 시작 위상을 다르게
        deck = HingeDeckMotion(HeaveProfile("sine", amp=0.012, period=2.5), socket_on_ship=tuple(SOCKET_SIDE_ON_SHIP),
                               hinge_below=HINGE_BELOW_SIDE_SOCKET)
        off = float(np.random.default_rng(seed + 777).uniform(0, 2.5))
        return lambda t: deck(t + off)
    return ShipMotion(float(wave), seed=seed, berth="alongside")


class Planner:
    def __init__(self, mode, p: Params):
        self.mode, self.p = mode, p
        self.phase, self.depth, self.gate_t0 = "approach", -p.hover, None

    def socket_path(self, est, t0, H):
        """t → (입구 추정, 축 추정, 예측 속도). 관측이 없으면 None"""
        av = est.available(t0)
        if not av:
            return None
        ax = av[-1][2]
        if self.mode.startswith("follow"):
            ent = av[-1][1]
            if self.mode.startswith("follow_gate") and len(av) >= 4:  # t0에 본 속도(마지막 관측 몇 개의 차분)
                v = float(np.linalg.norm(av[-1][1] - av[-4][1]) / (av[-1][0] - av[-4][0]))
            else:
                v = 0.0
            return lambda t: (ent, ax, v)
        p = self.p
        win = [o for o in av if o[0] >= av[-1][0] - p.ar_window]
        P = np.array([o[1] for o in win])
        dt, tl = 1.0 / p.cam_hz, av[-1][0]
        steps = max(2, int(np.ceil((t0 + H + p.lead - tl) / dt)) + 2)
        pred = np.stack([ar_predict(P[:, k], p.ar_order, steps) for k in range(3)], axis=1)  # pred[i] = 시각 tl + (i+1) dt

        def f(t):
            x = (t + p.lead - tl) / dt - 1
            i = int(np.clip(np.floor(x), 0, steps - 2))
            w = float(np.clip(x - i, 0, 1))
            pos = pred[i] * (1 - w) + pred[i + 1] * w
            return pos, ax, float(np.linalg.norm(pred[i + 1] - pred[i]) / dt)
        return f

    def plan(self, est, t0, tip_now, H):
        """[t0, t0 + H)의 목표 핀 위치와 축 목록(CONTROL_DT 간격). 단계 상태는 계획 끝 상태로 넘긴다"""
        f = self.socket_path(est, t0, H)
        if f is None:
            return None
        p, n = self.p, max(1, int(round(H / CONTROL_DT)))
        ent0, ax0, _ = f(t0)
        rel = tip_now - ent0
        lateral = np.linalg.norm(rel - np.dot(rel, ax0) * ax0)
        dist = float(np.hypot(lateral, np.dot(rel, ax0) - p.hover))
        aligned_at = t0 if (lateral < p.align_tol and abs(np.dot(rel, ax0) - p.hover) < p.align_tol) else t0 + dist / V_CONV
        gate_mode = self.mode in ("follow_gate", "predict", "predict_te", "follow_gate_te")
        out = []
        for i in range(n):
            t = t0 + i * CONTROL_DT
            ent, ax, speed = f(t)
            if self.phase == "approach" and t >= aligned_at:
                self.phase, self.gate_t0 = "gate", t
            if self.phase == "gate":
                calm = (not gate_mode) or speed < p.gate_speed or t - self.gate_t0 > p.gate_timeout
                if calm:
                    self.phase = "insert"
            if self.phase == "insert":
                self.depth = min(self.depth + p.descend_speed * CONTROL_DT, p.depth_goal + 0.004)
            out.append((ent - ax * self.depth, ax))
        return out


def run(args):
    mode, H, wave, seed = args
    scene, p = G["scene"], Params()
    m, d = scene.model, scene.data
    rng = np.random.default_rng(seed)
    motion = motion_of(wave, seed)
    mujoco.mj_resetData(m, d)
    d.qpos[scene.qadr] = G["home"]
    d.ctrl[scene.act] = G["home"]
    d.ctrl[scene.grip_act] = 0.0
    scene.set_ship(*motion(0.0))
    mujoco.mj_forward(m, d)
    ik = IK(scene)
    est = SocketEstimator("reactive", p, rng)  # capture()와 available()만 쓴다(잡음 난수 순서가 run_trial과 같다)
    te = mode.endswith("_te")
    planner = Planner(mode, p)
    queue, q_cmd = [], G["home"].copy()
    te_plans = []  # (시작 스텝, 목표 목록)
    tr, step = Trial(), 0
    n_sub = int(round(CONTROL_DT / m.opt.timestep))
    t, t_end, hold_t, f_ins = 0.0, p.timeout, 0.0, []
    while t < t_end:
        entrance, axis = scene.socket_frame()
        est.capture(t, entrance, axis)
        tip = d.site_xpos[scene.tip_site].copy()
        if te:
            snap = (planner.phase, planner.depth, planner.gate_t0)
            pl = planner.plan(est, t, tip, H)
            if pl is not None:
                te_plans.append((step, pl))
                # 다음 스텝의 단계 상태는 한 스텝만 진행한 상태로 되돌린다(매 스텝 다시 계획)
                planner.phase, planner.depth, planner.gate_t0 = snap
                planner.plan(est, t, tip, CONTROL_DT)
            te_plans = [(s0, pl) for s0, pl in te_plans if step - s0 < len(pl)]
            if te_plans:
                ages = np.array([step - s0 for s0, _ in te_plans], float)
                w = np.exp(-TE_COEF * (ages.max() - ages))  # 오래된 계획에 큰 가중치(ACT와 같은 방향)
                tg = sum(wi * pl[step - s0][0] for wi, (s0, pl) in zip(w, te_plans)) / w.sum()
                ax = te_plans[-1][1][0][1]
                q_cmd = ik.solve(q_cmd, tg, -ax)
                d.ctrl[scene.act] = q_cmd
        else:
            if not queue:
                pl = planner.plan(est, t, tip, H)
                if pl is not None:
                    for tg, ax in pl:  # 이전 명령에서 이어 푼다(실제 관절을 안 본다)
                        q_cmd = ik.solve(q_cmd, tg, -ax)
                        queue.append(q_cmd.copy())
            if queue:
                d.ctrl[scene.act] = queue.pop(0)
        for _ in range(n_sub):
            scene.set_ship(*motion(t))
            mujoco.mj_step(m, d)
            t += m.opt.timestep
        step += 1
        body_hit, plug_hull, force = contacts(scene)
        f_ins.append(force)
        if body_hit:
            tr.reason = "body"
            break
        if plug_hull:
            tr.reason = "hull"
            break
        entrance, axis = scene.socket_frame()
        rel = d.site_xpos[scene.tip_site] - entrance
        depth = -np.dot(rel, axis)
        lateral = np.linalg.norm(rel + depth * axis)
        if depth >= p.depth_goal and lateral < BORE_R - PIN_R + 0.001:
            hold_t += CONTROL_DT
            if hold_t >= p.hold:
                tr.inserted, tr.t_insert = True, t
                break
        else:
            hold_t = 0.0
    else:
        tr.reason = "timeout"
    return mode, H, wave, seed, bool(tr.inserted), tr.reason, float(np.percentile(f_ins, 95)) if f_ins else 0.0


def wilson(k, n, z=1.96):
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [round(float(max(0, c - h)) * 100, 1), round(float(min(1, c + h)) * 100, 1)]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--modes", default="follow,follow_gate,predict,predict_nogate,follow_te,predict_te")
    ap.add_argument("--H", default="0.02,0.1,0.33,1.0,3.3")
    ap.add_argument("--waves", default="0.04,0.06,0.08,deck_sine")
    ap.add_argument("--out", default="chunk_study")
    a = ap.parse_args()
    modes, Hs, waves = a.modes.split(","), [float(h) for h in a.H.split(",")], a.waves.split(",")
    jobs = [(mo, H, w, s) for w in waves for mo in modes for H in Hs for s in range(a.seed0, a.seed0 + a.n)
            if not (mo.endswith("_te") and H < 0.3)]  # 앙상블은 묶음이 길 때만 의미가 있다
    t0 = time.time()
    with Pool(a.jobs, initializer=init) as pool:
        res = pool.map(run, jobs, chunksize=4)
    out = {"setup": {"n": a.n, "seeds": f"{a.seed0}~{a.seed0 + a.n - 1}", "v_conv_mps": V_CONV, "te_coef": TE_COEF,
                     "engine": "MuJoCo", "note": "각본 묶음 제어기. 같은 시드 짝 비교만 해석한다."}, "cells": {}}
    for w in waves:
        for mo in modes:
            for H in Hs:
                rs = [r for r in res if r[0] == mo and r[1] == H and r[2] == w]
                if not rs:
                    continue
                k = sum(r[4] for r in rs)
                reasons = {}
                for r in rs:
                    if not r[4]:
                        reasons[r[5]] = reasons.get(r[5], 0) + 1
                out["cells"][f"{w}|{mo}|{H}"] = {"ok": k, "n": len(rs), "pct": round(100 * k / len(rs), 1), "wilson": wilson(k, len(rs)),
                                                  "force_p95_median": round(float(np.median([r[6] for r in rs])), 1),
                                                  "fail_reasons": reasons, "ok_seeds": [r[3] for r in rs if r[4]]}
    out["seconds"] = round(time.time() - t0, 1)
    dst = ROOT / "outputs/research" / f"{a.out}.json"
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(json.dumps(out, ensure_ascii=False, indent=1))
    for kk, v in out["cells"].items():
        print(f"{kk:32s} {v['ok']:3d}/{v['n']}  {v['wilson']}  {v['fail_reasons']}")
    print("->", dst.relative_to(ROOT), out["seconds"], "s")
