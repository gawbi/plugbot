"""정책이 어느 카메라에 기대나: 배 위 시험 때 정책이 실제로 본 입력(실행 기록)에서 한쪽 카메라를 가리거나 밀어 본다.

배경: 시범은 앞 카메라 시점 A로 녹화했고, 배 위 시험은 시점 B였다(outputs/research/front_view_shift.png).
그런데도 실물 모델과 트윈 모델 모두 꽂았다. 그렇다면 어느 입력에 기대는지 재 볼 만하다.
조건(같은 관절 state, 화면만 바꿈) → 원래 입력의 묶음과 1초 뒤(30스텝) 핀 끝 거리(mm)
  front_gray   앞 카메라를 회색으로
  wrist_gray   손목 카메라를 회색으로
  both_gray    둘 다 회색
  front_up45   앞 화면을 위로 45 px 민 것(시점 A와 B 차이 정도)
  front_dn45   아래로 45 px
실행: ~/lerobot_env/bin/python tools/research/camera_reliance.py
결과: outputs/research/camera_reliance.json, .png
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import act_offline as A  # noqa: E402

ROOT = A.ROOT
MODELS = ["act_real_v4/080000", "act_twin_v4/001000"]
PER_RUN = 12
audit = json.loads((ROOT / "outputs/research/rollout_audit.json").read_text())["runs"]


def load(run, k):
    d = ROOT / "data/rollouts" / run
    f = cv2.cvtColor(cv2.imread(str(d / "front" / f"{k:05d}.jpg")), cv2.COLOR_BGR2RGB)
    w = cv2.cvtColor(cv2.imread(str(d / "wrist" / f"{k:05d}.jpg")), cv2.COLOR_BGR2RGB)
    return f, w


def roll(im, dy):
    out = np.empty_like(im)
    if dy < 0:
        out[:dy] = im[-dy:]; out[dy:] = im[-1:]
    else:
        out[dy:] = im[:-dy]; out[:dy] = im[:1]
    return out


res = {"note": "배 위 시험 실행 기록(판정 있는 실행)의 실제 입력. 값은 원래 입력 대비 1초 뒤 핀 끝 거리(mm), "
               "실행 단위 부트스트랩 95%. 크면 그 입력에 기대는 것."}
for name in MODELS:
    model, step = name.split("/")
    runs = [r for r in audit if r["model"] == model and r["ckpt"] == step and r["ok"] is not None and r["frames"] > 100]
    F, Wr, S, run_id = [], [], [], []
    for i, r in enumerate(runs):
        st = np.load(ROOT / "data/rollouts" / r["run"] / "state.npy")
        n = min(r["frames"], len(st))
        for k in np.linspace(5, n - 1, PER_RUN).astype(int):
            f, w = load(r["run"], k)
            F.append(f); Wr.append(w); S.append(st[k, 1:7]); run_id.append(i)
    F, Wr, S, run_id = np.stack(F), np.stack(Wr), np.stack(S), np.array(run_id)
    gray = np.full_like(F, 128)
    conds = {"base": (F, Wr), "front_gray": (gray, Wr), "wrist_gray": (F, gray), "both_gray": (gray, gray),
             "front_up45": (np.stack([roll(x, -45) for x in F]), Wr), "front_dn45": (np.stack([roll(x, 45) for x in F]), Wr)}
    P = A.Policy(name)
    tip = {}
    for c, (ff, ww) in conds.items():
        ch = np.concatenate([P.chunks(ff[i:i + 32], ww[i:i + 32], S[i:i + 32]) for i in range(0, len(S), 32)])
        tip[c] = A.tips_from_units(ch[:, 29])
    rng = np.random.default_rng(0)
    r = {"runs": len(runs), "samples": int(len(S))}
    for c in conds:
        if c == "base":
            continue
        dist = np.linalg.norm(tip[c] - tip["base"], axis=1) * 1000
        by = np.array([dist[run_id == i].mean() for i in range(len(runs))])
        bs = rng.choice(by, (2000, len(by))).mean(1)
        r[c] = [round(float(by.mean()), 1), [round(float(np.percentile(bs, 2.5)), 1), round(float(np.percentile(bs, 97.5)), 1)]]
    res[name] = r
    print(name, r, flush=True)
    del P
dst = ROOT / "outputs/research/camera_reliance.json"
dst.write_text(json.dumps(res, ensure_ascii=False, indent=1))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

FF = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FF)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FF).get_name()
labels = {"front_gray": "앞 카메라 가림", "wrist_gray": "손목 카메라 가림", "both_gray": "둘 다 가림", "front_up45": "앞 화면 45 px 위로", "front_dn45": "앞 화면 45 px 아래로"}
fig, ax = plt.subplots(figsize=(10, 4.2), dpi=150)
xs = np.arange(len(labels))
for j, (name, col) in enumerate(zip(MODELS, ("#6b3fd6", "#1d7686"))):
    v = [res[name][c][0] for c in labels]
    e = np.array([[res[name][c][0] - res[name][c][1][0], res[name][c][1][1] - res[name][c][0]] for c in labels]).T
    ax.bar(xs + (j - 0.5) * 0.38, v, 0.36, yerr=e, color=col, capsize=3,
           label=("실물 화면으로 학습(act_real_v4)" if "real" in name else "트윈 화면으로만 학습(act_twin_v4 1천)"))
ax.set_xticks(xs)
ax.set_xticklabels(list(labels.values()))
ax.set_ylabel("1초 뒤 동작 변화 (핀 끝 mm)")
ax.set_title("배 위 시험 입력에서 카메라를 가리거나 밀면 정책 동작이 얼마나 바뀌나")
ax.legend(frameon=False)
ax.spines[["top", "right"]].set_visible(False)
fig.tight_layout()
fig.savefig(dst.with_suffix(".png"))
print("->", dst.relative_to(ROOT))
