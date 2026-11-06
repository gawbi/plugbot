"""녹화한 시범 검수: 에피소드마다 '꽂힌 마지막 자세까지 남은 거리' 곡선과 자동 표시, 의심 에피소드의 장면 사진.

표시 기준(관절 5개 각도 거리, 0 = 마지막 꽂힌 자세):
- 다시 꽂기: 거리 5 아래로 갔다가 8 위로 올라간 뒤 다시 5 아래로(잘 꽂힌 뒤 뺀 것인지 장면으로 확인)
- 빗나감 뒤 회복: 15 아래로 왔다가 25 위로 물러난 뒤 꽂음(소켓 앞까지 왔다가 다시 맞춤. 대개 좋은 시범)
- 짧음(5 s 미만), 거의 안 움직임(최대 이동 20 미만), 끝 1.5 s에 움직임(꽂힌 채 멈추지 않음)
결과: outputs/episode_review/<데이터셋>_curves.png, 의심 에피소드는 ep<번호>.jpg(앞, 손목 카메라 네 장면)
실행: ~/lerobot_env/bin/python act/check_episodes.py [static]
"""

import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import matplotlib
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

KIND = sys.argv[1] if len(sys.argv) > 1 else "static"
ROOT = Path.home() / ".cache" / "huggingface" / "lerobot" / "local" / f"plug_{KIND}"
OUT = Path(__file__).resolve().parents[1] / "outputs" / "episode_review"
OUT.mkdir(parents=True, exist_ok=True)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if Path(FONT).exists():
    font_manager.fontManager.addfont(FONT)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()

if not (ROOT / "meta" / "info.json").exists():
    # 없는 데이터셋을 열면 LeRobot이 빈 폴더를 만들고, 그 폴더 때문에 녹화가 시작되지 않는다(2026-11-06)
    sys.exit(f"{ROOT} 에 데이터셋이 없다")
ds = LeRobotDataset(f"local/plug_{KIND}", root=ROOT)
eps = ds.meta.episodes
try:
    pos = json.loads((ROOT / "plugbot_positions.json").read_text())
except OSError:
    pos = {}
state = np.stack(ds.hf_dataset["observation.state"])[:, :5]
epi = np.asarray(ds.hf_dataset["episode_index"])


def arrivals(d, lo, hi):
    out, armed = [], True
    for i, x in enumerate(d):
        if armed and x < lo:
            out.append(i)
            armed = False
        elif x > hi:
            armed = True
    return out


def frames(ep, a, d, picks):
    rows = []
    for idx, lab in picks:
        x = ds[a + int(idx)]
        f = (x["observation.images.front"].permute(1, 2, 0).numpy() * 255).astype(np.uint8)[:, :, ::-1]
        w = (x["observation.images.wrist"].permute(1, 2, 0).numpy() * 255).astype(np.uint8)[:, :, ::-1]
        im = np.hstack([f, w]).copy()
        cv2.putText(im, f"ep{ep} {idx / 30:.1f}s d={d[idx]:.0f} {lab}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        rows.append(cv2.resize(im, (640, 240)))
    cv2.imwrite(str(OUT / f"ep{ep}.jpg"), np.vstack(rows))


curves, report = {}, []
for ep in range(ds.num_episodes):
    a, b = int(eps["dataset_from_index"][ep]), int(eps["dataset_to_index"][ep])
    s = state[a:b]
    d = np.linalg.norm(s - s[-1], axis=1)
    curves[ep] = d
    flags = []
    arr = arrivals(d, 5, 8)
    if len(arr) > 1:
        flags.append(f"다시 꽂기({', '.join(f'{i / 30:.1f}s' for i in arr)})")
    near = arrivals(d, 15, 25)
    if len(near) > 1 and len(arr) <= 1:
        flags.append("빗나감 뒤 회복")
    if (b - a) / 30 < 5:
        flags.append("짧음")
    if np.abs(s - s[0]).max() < 20:
        flags.append("거의 안 움직임")
    if np.abs(s[-45:] - s[-1]).max() > 3:
        flags.append("끝 1.5 s 움직임")
    report.append((ep, pos.get(str(ep), "?"), (b - a) / 30, flags))
    if flags:
        i1 = arr[0] if arr else (near[0] if near else int(np.argmin(d[: len(d) // 2 + 1])))
        k = i1 + int(np.argmax(d[i1:]))
        frames(ep, a, d, [(i1, "first near"), (min(i1 + 15, len(d) - 1), "+0.5s"), (k, "farthest after"), (len(d) - 1, "end")])

n = ds.num_episodes
cols = 6
rows = (n + cols - 1) // cols
fig, axs = plt.subplots(rows, cols, figsize=(18, 2.4 * rows), sharey=True, squeeze=False)
for ax, (ep, p, dur, flags) in zip(axs.flat, report):
    t = np.arange(len(curves[ep])) / 30
    ax.plot(t, curves[ep], lw=1.2, color="#d6336c" if flags else "#2b6cb0")
    ax.axhline(5, color="gray", lw=0.6, ls="--")
    ax.set_ylim(0, 40)
    ax.set_title(f"{ep}번 {p}" + (" ⚠" if flags else ""), fontsize=9)
    ax.grid(alpha=0.2)
for ax in axs.flat[n:]:
    ax.axis("off")
fig.suptitle(f"plug_{KIND}: 에피소드별 꽂힌 자세까지 남은 거리(0 = 꽂힘). 빨강 = 자동 표시", fontsize=13)
fig.tight_layout()
fig.savefig(OUT / f"plug_{KIND}_curves.png", dpi=80)

print(f"plug_{KIND}: {n}개, {ds.num_frames}프레임, 자리별 {dict(Counter(pos.get(str(e), '?') for e in range(n)))}")
for ep, p, dur, flags in report:
    if flags:
        print(f"  {ep}번 ({p}, {dur:.1f}s): {' / '.join(flags)}")
print(f"곡선: {OUT / f'plug_{KIND}_curves.png'}")
