"""E3 결과 정리: 묶음 길이별 성공률 그림, 같은 시드 짝 검정(McNemar 정확).

입력: outputs/research/chunk_study.json(chunk_study.py)
결과: outputs/research/chunk_report.json, outputs/research/chunk_study.png
"""

import json
from math import comb
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
D = json.loads((ROOT / "outputs/research/chunk_study.json").read_text())
C = D["cells"]
N = D["setup"]["n"]
SEEDS = set(range(1000, 1000 + N))


def ok(w, m, h):
    return set(C[f"{w}|{m}|{h}"]["ok_seeds"])


def mcnemar(a, b):
    """a, b: 성공 시드 집합. (a만, b만, 양측 정확 p)"""
    x, y = len(a - b), len(b - a)
    n, k = x + y, min(x, y)
    p = min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n) if n else 1.0
    return x, y, p


pairs = {}
for w in ("0.04", "0.06", "0.08", "deck_sine"):
    pairs[w] = {
        "predict_vs_follow_closed(0.02)": mcnemar(ok(w, "predict", 0.02), ok(w, "follow", 0.02)),
        "predict_vs_follow_chunk3.3": mcnemar(ok(w, "predict", 3.3), ok(w, "follow", 3.3)),
        "predict_closed_vs_chunk3.3": mcnemar(ok(w, "predict", 0.02), ok(w, "predict", 3.3)),
        "follow_closed_vs_chunk3.3": mcnemar(ok(w, "follow", 0.02), ok(w, "follow", 3.3)),
        "gate_vs_nogate_predict(0.02)": mcnemar(ok(w, "predict", 0.02), ok(w, "predict_nogate", 0.02)),
        "gate_vs_nogate_predict(0.33)": mcnemar(ok(w, "predict", 0.33), ok(w, "predict_nogate", 0.33)),
        "te_vs_chunk_predict(3.3)": mcnemar(ok(w, "predict_te", 3.3), ok(w, "predict", 3.3)),
    }
out = {"note": "같은 시드 짝 비교. (A만 성공, B만 성공, McNemar 정확 양측 p). 각본 묶음 제어기, MuJoCo.", "pairs": pairs}
(ROOT / "outputs/research/chunk_report.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
for w, r in pairs.items():
    print(w)
    for k, (x, y, p) in r.items():
        print(f"   {k:34s} A만 {x:3d}  B만 {y:3d}  p = {p:.2g}")

F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=F).get_name()
HS = [0.02, 0.1, 0.33, 1.0, 3.3]
fig, axs = plt.subplots(1, 3, figsize=(15, 4.8), dpi=150, sharey=True)
for ax, (w, title) in zip(axs, (("0.06", "불규칙파 6 cm (JONSWAP)"), ("0.08", "불규칙파 8 cm"), ("deck_sine", "실물 갑판 설정 (정현파 12 mm, 2.5 s)"))):
    for m, col, lab, ls in (("follow", "#c8781e", "따라가기", "-"), ("predict", "#1d7686", "예측(0.3 s 앞 + AR 내다보기)", "-"),
                            ("follow_te", "#c8781e", "따라가기 + 시간 앙상블", ":"), ("predict_te", "#1d7686", "예측 + 시간 앙상블", ":")):
        xs = [h for h in HS if f"{w}|{m}|{h}" in C]
        ys = [C[f"{w}|{m}|{h}"]["pct"] for h in xs]
        lo = [C[f"{w}|{m}|{h}"]["wilson"][0] for h in xs]
        hi = [C[f"{w}|{m}|{h}"]["wilson"][1] for h in xs]
        ax.plot(xs, ys, ls, color=col, marker="o", lw=2, label=lab)
        if ls == "-":
            ax.fill_between(xs, lo, hi, color=col, alpha=0.12)
    ax.axvline(3.3, color="#6b3fd6", lw=1, alpha=0.5)
    ax.text(3.2, 97, "실물 ACT 묶음 길이", color="#6b3fd6", ha="right", va="top", fontsize=9)
    ax.set_xscale("log")
    ax.set_xticks(HS)
    ax.set_xticklabels(["0.02\n(매 스텝)", "0.1", "0.33", "1.0", "3.3"])
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("다시 계획하는 간격 H (s, 그동안 눈 감고 실행)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_ylim(0, 102)
axs[0].set_ylabel("성공률 (%) , 같은 시드 100개")
axs[0].legend(frameon=False, fontsize=9, loc="lower left")
fig.suptitle("묶음 실행의 비용: 다시 계획하는 간격이 길수록 예측의 이점이 줄어든다 (MuJoCo, 각본 묶음 제어기)", fontsize=12)
fig.tight_layout()
fig.savefig(ROOT / "outputs/research/chunk_study.png")
print("-> outputs/research/chunk_study.png")
