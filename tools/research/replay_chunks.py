"""재생 확인(오프라인 분석 검증): 기록된 실물 실행에서 ACT 묶음을 다시 계산해, 실제로 움직인 관절과 맞는지 본다.

실행 중 정책은 관측 k에서 100스텝 묶음을 내고 그것을 끝까지 실행한 뒤 다시 예측했다(n_action_steps 100).
기록(data/rollouts/<run>)에는 정책이 본 화면(JPEG)과 관절만 있고 동작은 없다. 그래서
  1) 묶음 시작 프레임 후보마다 묶음을 다시 계산하고
  2) 실제 관절(state)이 명령을 L 프레임 늦게 따른다고 보고 오차가 가장 작은 시작 위치와 L을 찾는다.
다시 계산한 묶음이 실제 궤적과 맞으면, 실물 화면을 정책에 넣는 오프라인 파이프라인(전처리, 단위)이 실행 때와 같다는 뜻이다.
실행: ~/lerobot_env/bin/python tools/research/replay_chunks.py [--run 20261110_031451]
결과: outputs/research/replay_<run>.json, .png, 묶음 배열 data/research/chunks_<run>.npz(영상용)
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import act_offline as A  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--run", default="20261110_031451")
ap.add_argument("--ckpt", default="", help="비우면 meta.json의 정책")
a = ap.parse_args()
R = A.ROOT / "data/rollouts" / a.run
meta = json.loads((R / "meta.json").read_text())
ckpt = a.ckpt or meta["policy"].split("act_runs/")[-1].replace("/pretrained_model", "").replace("checkpoints/", "")
P = A.Policy(ckpt)
st = np.load(R / "state.npy")  # [t, 6 관절]
t, S = st[:, 0], st[:, 1:7]
N = len(S)


def frame(cam, k):
    return cv2.cvtColor(cv2.imread(str(R / cam / f"{k:05d}.jpg")), cv2.COLOR_BGR2RGB)


# 모든 프레임에서 묶음을 계산해 둔다(시작 위치 탐색, 영상에 쓴다)
allc = []
for k0 in range(0, N, 32):
    ks = list(range(k0, min(N, k0 + 32)))
    allc.append(P.chunks(np.stack([frame("front", k) for k in ks]), np.stack([frame("wrist", k) for k in ks]), S[ks]))
allc = np.concatenate(allc)  # (N, 100, 6)


def score(start, lag):
    """start에서 시작해 100스텝마다 다시 예측했다고 볼 때, 명령과 L 프레임 뒤 실제 관절의 평균 절대 오차(관절 5개, 눈금)"""
    err = []
    for b in range(start, N - lag - 1, P.chunk):
        n = min(P.chunk, N - lag - b - 1)
        err.append(np.abs(allc[b, :n, :5] - S[b + lag + 1: b + lag + 1 + n, :5]))
    return float(np.concatenate(err).mean())


grid = {(s0, lag): score(s0, lag) for s0 in range(0, 12) for lag in range(0, 12)}
(best_s, best_l), best = min(grid.items(), key=lambda kv: kv[1])
base = score(best_s, best_l)
# 대조: 묶음 시작을 반 묶음(50) 어긋나게 두면 오차가 커져야 한다
shifted = score(best_s + P.chunk // 2, best_l)
# 핀 끝 위치 오차(mm): 묶음 마지막 25스텝(꽂기 직전 동작이 들어 있는 구간)
tips_err = []
for b in range(best_s, N - best_l - P.chunk - 1, P.chunk):
    cmd = A.tips_from_units(allc[b, -25:])
    real = A.tips_from_units(S[b + best_l + P.chunk - 25 + 1: b + best_l + P.chunk + 1])
    tips_err.append(float(np.linalg.norm(cmd - real, axis=1).mean() * 1000))
out = {"run": a.run, "ckpt": ckpt, "frames": N, "fps_logged": round(float(1 / np.diff(t).mean()), 1),
       "best_chunk_start": best_s, "best_lag_frames": best_l, "mean_abs_err_units": round(base, 2),
       "mean_abs_err_units_if_boundaries_shifted_50": round(shifted, 2),
       "tip_err_mm_last25_per_chunk": [round(x, 1) for x in tips_err],
       "note": "눈금은 LeRobot 관절 눈금(-100~100). 명령과 실제 관절 사이에는 서보 추종 오차와 접촉이 섞인다."}
dst = A.ROOT / "outputs/research" / f"replay_{a.run}.json"
dst.write_text(json.dumps(out, ensure_ascii=False, indent=1))
(A.ROOT / "data/research").mkdir(parents=True, exist_ok=True)
np.savez_compressed(A.ROOT / "data/research" / f"chunks_{a.run}.npz", chunks=allc.astype(np.float32), state=st,
                    start=best_s, lag=best_l, ckpt=ckpt)
print(json.dumps(out, ensure_ascii=False, indent=1))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=F).get_name()
fig, axs = plt.subplots(5, 1, figsize=(11, 9), dpi=140, sharex=True)
for j, ax in enumerate(axs):
    ax.plot(np.arange(N) / 30, S[:, j], color="#161b26", lw=1.4, label="실제 관절(기록)")
    for b in range(best_s, N, P.chunk):
        n = min(P.chunk, N - b)
        ax.plot((np.arange(n) + b + best_l + 1) / 30, allc[b, :n, j], color="#6b3fd6", lw=1.2, ls="--",
                label="다시 계산한 3.3 s 묶음" if b == best_s else None)
        ax.axvline(b / 30, color="#c9c2e8", lw=0.8)
    ax.set_ylabel(A.JOINTS[j], fontsize=9)
    ax.spines[["top", "right"]].set_visible(False)
axs[0].legend(frameon=False, fontsize=9, loc="upper right")
axs[0].set_title(f"재생 확인: {a.run} ({ckpt}), 묶음 시작 {best_s}, 지연 {best_l} 프레임, 평균 오차 {base:.2f} (어긋나게 두면 {shifted:.2f})", fontsize=10)
axs[-1].set_xlabel("시간 (s, 30 fps 프레임 기준)")
fig.tight_layout()
fig.savefig(dst.with_suffix(".png"))
print("->", dst.relative_to(A.ROOT))
