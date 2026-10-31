"""실물용 소프트웨어를 MuJoCo 가짜 세계에서 처음부터 끝까지 시험한다(로봇·웹캠 없이).

1) 보정: 갑판을 6곳에 놓고 '손으로 끝까지 꽂은 자세'를 만들어 카메라→로봇 변환을 구한다(핀 끝 위치에 1 mm 잡음).
   정답 카메라 자세와 비교한다.
2) 삽입: 그 보정값으로 정지 소켓, 모형 유의파고 4 cm, 6 cm에서 예측·추종 전략을 시행한다.
   채점은 시뮬레이션 정답(실제 소켓 위치)으로 한다: 목표 깊이 유지 = 성공, 몸체 충돌·선체 찌름 = 실패.

실행: .venv/bin/python -m real.dry_run [--trials 10]
결과: outputs/real_dry_run.json
"""

import argparse
import json
from pathlib import Path

import numpy as np

from plugbot.control import Params, contacts
from plugbot.scene import BORE_R, PIN_R
from plugbot.waves import ShipMotion

from . import geometry as G
from .calib import HandEye, bore_bottom_cam, kabsch
from .insert import run
from .sim_world import SimWorld, sim_intrinsics
from .vision import SocketTracker

OUT = Path(__file__).resolve().parents[1] / "outputs"


def calibrate(world, tracker, n=6, tip_noise=0.001, seed=0):
    rng = np.random.default_rng(seed)
    p_cam, p_base = [], []
    for k in range(n):
        pose = (rng.uniform([-0.03, -0.04, -0.015], [0.03, 0.04, 0.015]), rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06))
        world.motion = lambda t, pose=pose: pose
        world.reset()
        ent, ax = world.socket_truth()
        bottom = ent - ax * (G.ENTRANCE_IN_SOCKET[2] - G.BORE_BOTTOM_IN_SOCKET[2])
        q = world.kin.solve(world.q(), bottom, -ax)
        world.set_arm(q)
        c = bore_bottom_cam(tracker, world.latest()[1])
        if c is None:
            continue
        p_cam.append(c)
        p_base.append(world.kin.tip(q) + rng.normal(0, tip_noise, 3))
    R, t, err = kabsch(p_cam, p_base)
    Rt, tt = world.true_cam_pose()
    rot_err = np.degrees(np.arccos(np.clip((np.trace(Rt.T @ R) - 1) / 2, -1, 1)))
    return HandEye(R, t), {"쌍": len(p_cam), "점별 오차 RMS mm": float(np.sqrt(np.mean(err ** 2)) * 1000),
                           "회전 오차 deg": float(rot_err), "카메라 위치 오차 mm": float(np.linalg.norm(t - tt) * 1000)}


def judge(world, p):
    """시뮬레이션 정답으로 매 주기 채점. 실패 이유 문자열 또는 None. 성공 여부는 world.truth_seated에 남긴다."""
    body_hit, plug_hull, _ = contacts(world.scene)
    if body_hit:
        return "로봇 몸체 충돌"
    if plug_hull:
        return "플러그가 선체를 찌름"
    ent, ax = world.socket_truth()
    rel = world.tip_position() - ent
    depth = -float(np.dot(rel, ax))
    lateral = float(np.linalg.norm(rel + depth * ax))
    seated = depth >= p.depth_goal and lateral < BORE_R - PIN_R + 0.001
    world.truth_hold = world.truth_hold + 0.02 if seated else 0.0
    if world.truth_hold >= p.hold:
        world.truth_seated = True
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10)
    a = ap.parse_args()
    p = Params()
    world = SimWorld()
    tracker = SocketTracker(sim_intrinsics())
    he, cal = calibrate(world, tracker)
    print("보정:", {k: round(v, 2) for k, v in cal.items()})
    out = {"보정": cal, "결과": {}}
    for hs in (0.0, 0.04, 0.06):
        for strategy in ("reactive", "predictive"):
            rows = []
            for k in range(a.trials):
                world.motion = ShipMotion(hs, seed=100 + k) if hs > 0 else (lambda t: (np.zeros(3), 0.0, 0.0))
                world.reset()
                world.truth_hold, world.truth_seated = 0.0, False
                r = run(world, world, tracker, he, p, strategy=strategy, monitor=lambda w: judge(w, p), release=False)
                if r.seated:  # 파이프라인이 '다 꽂았다'고 본 뒤 0.4초 더 지켜본다(실물은 이때 놓고 자석이 잡는다)
                    for _ in range(20):
                        world.wait_until(world.now() + 0.02)
                        why = judge(world, p)
                        if why:
                            r.reason = why
                            break
                rows.append({"성공": bool(world.truth_seated), "파이프라인 판단": r.seated, "이유": r.reason,
                             "놓친 영상": r.lost_frames})
            rate = np.mean([x["성공"] for x in rows])
            print(f"Hs {hs * 100:.0f} cm {strategy:10s} 성공 {rate * 100:5.1f}%  실패 이유: "
                  f"{sorted(set(x['이유'] for x in rows if not x['성공']))}")
            out["결과"][f"hs{hs * 100:.0f}_{strategy}"] = {"성공률": float(rate), "시행": rows}
    OUT.mkdir(exist_ok=True)
    (OUT / "real_dry_run.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
