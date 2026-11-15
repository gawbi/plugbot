"""E1 오프라인 평가: 학습에 안 쓴 실물 에피소드(plug_static 69~93, 트윈이 픽셀도 궤적도 본 적 없음)에서
체크포인트가 내는 다음 1초(30스텝) 동작을 잰다.

조건(같은 장면, 같은 state, 화면만 바꾼다)
  real   실물 카메라 화면
  tc     같은 관절로 다시 그린 트윈 화면, 보정한 카메라(data/twin_render_holdout_calib)
  tn     같은 트윈 화면, 보정 전 카메라(data/twin_render_holdout_nocalib)
  gray   회색 화면(대조: 시각을 안 쓰면 이것과 같아야 한다)
  shuf   다른 에피소드의 같은 진행 시점 실물 화면(대조)
지표
  gt_mm    사람 시범의 실제 다음 동작과 핀 끝 거리(순기구학, 1초 뒤 스텝)
  gt_u     관절 5개 평균 절대 차(LeRobot 눈금, 30스텝 평균)
  gap_mm   같은 state에서 real 화면 묶음과 트윈 화면 묶음의 핀 끝 거리(1초 뒤). state와 정답이 상쇄돼 누설이 없다
신뢰구간: 에피소드 단위 부트스트랩(2000회) 95%.
실행: ~/lerobot_env/bin/python tools/research/offline_eval.py [--ckpts act_twin_v4/001000,...]
결과: outputs/research/offline_eval.json (체크포인트가 늘면 합쳐 쓴다)
"""

import argparse
import json
import sys
import time
from pathlib import Path

import imageio.v2 as iio
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import act_offline as A  # noqa: E402

DEFAULT = (["act_twin_v2/%06d" % s for s in range(2500, 20001, 2500)] + ["act_twin_v4/%06d" % s for s in range(1000, 5001, 1000)]
           + ["act_twin_v1/010000", "act_real_v4/080000"])
ap = argparse.ArgumentParser()
ap.add_argument("--ckpts", default=",".join(DEFAULT))
ap.add_argument("--per_ep", type=int, default=20)
ap.add_argument("--lo", type=float, default=0.30)
ap.add_argument("--H", type=int, default=30)
ap.add_argument("--hi", type=float, default=1.0, help="진행 구간 끝(1.0이면 에피소드 끝 - H)")
ap.add_argument("--tag", default="", help="캐시와 결과 파일 이름 꼬리(구간을 바꿀 때)")
a = ap.parse_args()
ROOT = A.ROOT
DS = Path.home() / ".cache/huggingface/lerobot/local/plug_static"
CACHE = ROOT / f"data/research/holdout_cache{a.tag}.npz"
EPS = list(range(69, 94))


def build_cache():
    meta = pd.concat([pd.read_parquet(f) for f in sorted((DS / "meta/episodes").rglob("*.parquet"))]).set_index("episode_index")
    rows = {k: [] for k in ("ep", "k", "prog", "state", "act")}
    imgs = {k: [] for k in ("real_front", "real_wrist", "tc_front", "tc_wrist", "tn_front", "tn_wrist")}
    for e in EPS:
        src = np.load(ROOT / f"data/twin_src_holdout/ep_{e:03d}.npz")
        T = len(src["state"])
        ks = np.linspace(a.lo * T, min(a.hi * T, T - a.H - 1), a.per_ep).astype(int)
        m = meta.loc[e]
        for cam in ("front", "wrist"):
            f = DS / f"videos/observation.images.{cam}/chunk-{int(m[f'videos/observation.images.{cam}/chunk_index']):03d}" \
                / f"file-{int(m[f'videos/observation.images.{cam}/file_index']):03d}.mp4"
            off = int(round(float(m[f"videos/observation.images.{cam}/from_timestamp"]) * 30))
            r = iio.get_reader(f)
            imgs[f"real_{cam}"] += [r.get_data(off + int(k)) for k in ks]
            r.close()
            for tag, d in (("tc", "twin_render_holdout_calib"), ("tn", "twin_render_holdout_nocalib")):
                r = iio.get_reader(ROOT / f"data/{d}/ep_{e:03d}_n_{cam}.mp4")
                fr = [im for im in r]
                r.close()
                assert len(fr) == T, (e, cam, tag, len(fr), T)
                imgs[f"{tag}_{cam}"] += [fr[int(k)] for k in ks]
        rows["ep"] += [e] * len(ks); rows["k"] += list(ks); rows["prog"] += list(ks / T)
        rows["state"] += [src["state"][k] for k in ks]
        rows["act"] += [src["action"][k: k + a.H] for k in ks]
        print("cache", e, flush=True)
    CACHE.parent.mkdir(parents=True, exist_ok=True)
    np.savez(CACHE, **{k: np.array(v) for k, v in rows.items()}, **{k: np.stack(v).astype(np.uint8) for k, v in imgs.items()})


if not CACHE.exists():
    build_cache()
C = dict(np.load(CACHE))
N = len(C["ep"])
ep = C["ep"]
# 대조 화면
gray = np.full_like(C["real_front"], 128)
order = np.arange(N)
shuf_idx = (order + a.per_ep) % N  # 다음 에피소드의 같은 진행 시점
views = {"real": (C["real_front"], C["real_wrist"]), "tc": (C["tc_front"], C["tc_wrist"]), "tn": (C["tn_front"], C["tn_wrist"]),
         "gray": (gray, gray), "shuf": (C["real_front"][shuf_idx], C["real_wrist"][shuf_idx])}
gt_tip = A.tips_from_units(C["act"][:, a.H - 1])  # (N, 3)


def boot(per_sample, n=2000, seed=0):
    """에피소드 단위 부트스트랩: (평균, [하한, 상한])"""
    rng = np.random.default_rng(seed)
    by = np.array([per_sample[ep == e].mean() for e in EPS])
    bs = rng.choice(by, (n, len(by))).mean(1)
    return round(float(by.mean()), 2), [round(float(np.percentile(bs, 2.5)), 2), round(float(np.percentile(bs, 97.5)), 2)]


out_f = ROOT / f"outputs/research/offline_eval{a.tag}.json"
res = json.loads(out_f.read_text()) if out_f.exists() else {}
res["_setup"] = {"episodes": "plug_static 69~93 (학습에 안 씀)", "samples": int(N), "per_episode": a.per_ep,
                 "progress_from": a.lo, "progress_to": a.hi, "horizon_steps": a.H, "metric_tip": "핀 끝 거리 mm(1초 뒤 스텝, 순기구학)",
                 "ci": "에피소드 단위 부트스트랩 95%"}
for name in a.ckpts.split(","):
    t0 = time.time()
    P = A.Policy(name)
    pred = {}
    for cond, (fr, wr) in views.items():
        out = [P.chunks(fr[i:i + 32], wr[i:i + 32], C["state"][i:i + 32]) for i in range(0, N, 32)]
        pred[cond] = np.concatenate(out)[:, : a.H]
    tip = {c: A.tips_from_units(p[:, a.H - 1]) for c, p in pred.items()}
    r = {}
    for c in views:
        r[f"gt_mm_{c}"] = boot(np.linalg.norm(tip[c] - gt_tip, axis=1) * 1000)
        r[f"gt_u_{c}"] = boot(np.abs(pred[c][..., :5] - C["act"][..., :5]).mean((1, 2)))
    for c in ("tc", "tn", "gray", "shuf"):
        r[f"gap_mm_real_{c}"] = boot(np.linalg.norm(tip["real"] - tip[c], axis=1) * 1000)
    res[name] = r
    out_f.write_text(json.dumps(res, ensure_ascii=False, indent=1))
    print(f"{name:22s} real {r['gt_mm_real'][0]:6.1f}  tc {r['gt_mm_tc'][0]:6.1f}  tn {r['gt_mm_tn'][0]:6.1f}  gray {r['gt_mm_gray'][0]:6.1f}  "
          f"shuf {r['gt_mm_shuf'][0]:6.1f} | gap real-tc {r['gap_mm_real_tc'][0]:5.1f}  real-tn {r['gap_mm_real_tn'][0]:5.1f}  "
          f"({time.time() - t0:.0f}s)", flush=True)
    del P
