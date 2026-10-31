"""MuJoCo 시범 데이터 만들기(ACT 사전학습, 형식 확인용). Isaac Sim의 --dataset과 같은 형식이다.

예측 제어기(plugbot.control.run_trial)가 꽂는 장면을 앞 카메라와 손목 카메라로 30 fps 녹화한다.
상태 = 현재 관절각, 행동 = 그 순간 보낸 관절 목표(둘 다 LeRobot SO-101 단위: 도 5개 + 그리퍼 0~100).

실행: .venv/bin/python make_sim_dataset.py --episodes 50 [--layout side --hs-max 0.06 --out data/sim_side]
결과: data/sim_side/episode_XXXX/{front.mp4, wrist.mp4, data.npz, meta.json}
LeRobot 데이터셋으로: python tools/to_lerobot.py data/sim_side --repo-id <HF아이디>/plugbot_sim_side (lerobot 환경)
"""

import argparse
import time
from pathlib import Path

import mujoco
import numpy as np

from plugbot.control import Params, run_trial
from plugbot.dataset import EpisodeWriter, to_lerobot_vector
from plugbot.scene import CAM_H, CAM_W, Scene
from plugbot.waves import ShipMotion

CAMERAS = {"front": "front", "wrist": "wrist_cam"}  # 데이터셋 이름: MuJoCo 카메라 이름


def record(scene, renderer, writer, fps, motion, strategy, p, seed):
    m, d = scene.model, scene.data
    next_t = [0.0]

    def on_step(t, phase):
        if t + 1e-9 < next_t[0]:
            return
        next_t[0] += 1.0 / fps
        images = {}
        for name, cam in CAMERAS.items():
            renderer.update_scene(d, camera=cam)
            images[name] = renderer.render()
        state = to_lerobot_vector(d.qpos[scene.qadr])
        action = to_lerobot_vector(d.ctrl[scene.act])
        writer.add(t, images, state, action)

    return run_trial(scene, strategy, motion, p, seed=seed, on_step=on_step)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--layout", default="side", choices=["side", "top"])
    ap.add_argument("--hs-max", type=float, default=0.06, help="에피소드마다 0~이 값에서 모형 유의파고를 고른다(m)")
    ap.add_argument("--strategy", default="predictive", choices=["static", "reactive", "predictive"])
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--keep-failures", action="store_true", help="실패한 시행도 남긴다(기본은 성공만)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = Path(a.out or f"data/sim_{a.layout}")
    scene = Scene("ship", a.layout)
    renderer = mujoco.Renderer(scene.model, CAM_H, CAM_W)  # 카메라 두 대를 렌더러 하나로 번갈아 그린다
    p = Params()
    rng = np.random.default_rng(0)
    kept, k, t0 = 0, 0, time.time()
    while kept < a.episodes:
        hs = float(rng.uniform(0.0, a.hs_max))
        motion = ShipMotion(hs, seed=1000 + k, berth="alongside")
        writer = EpisodeWriter(out, kept, CAMERAS, fps=a.fps)
        tr = record(scene, renderer, writer, a.fps, motion, a.strategy, p, seed=1000 + k)
        if tr.inserted or a.keep_failures:
            writer.close({"layout": a.layout, "hs": hs, "seed": 1000 + k, "strategy": a.strategy,
                          "success": bool(tr.inserted), "reason": tr.reason, "task": "Insert the plug into the socket"})
            kept += 1
            print(f"[{kept}/{a.episodes}] Hs {hs * 100:.1f} cm {'성공' if tr.inserted else '실패 ' + tr.reason}, "
                  f"{len(writer)}프레임", flush=True)
        else:
            writer.discard()
            print(f"  (버림) Hs {hs * 100:.1f} cm 실패: {tr.reason}", flush=True)
        k += 1
    print(f"{out}: 에피소드 {kept}개, {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
