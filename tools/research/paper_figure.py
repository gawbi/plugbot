"""논문(2쪽 요약)용 그림 한 장: (a) 실물 화면 대 트윈 화면 짝 차이(보정 전, 보정 후, 체크포인트별), (b) 다시 계획하는 간격별 성공률.

입력: outputs/research/offline_eval.json(접근과 꽂기 구간), chunk_study.json
결과: outputs/research/paper_fig.png
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams.update({"font.family": font_manager.FontProperties(fname=F).get_name(), "font.size": 8})
R = json.loads((ROOT / "outputs/research/offline_eval.json").read_text())
C = json.loads((ROOT / "outputs/research/chunk_study.json").read_text())["cells"]
K = [f"act_twin_v2/{s:06d}" for s in range(2500, 20001, 2500)] + [f"act_twin_v4/{s:06d}" for s in range(1000, 5001, 1000)]
lab = ["2.5", "5", "7.5", "10", "12.5", "15", "17.5", "20", "+1", "+2", "+3", "+4", "+5"]
fig, (a, b) = plt.subplots(1, 2, figsize=(7.0, 2.3), dpi=300, gridspec_kw={"width_ratios": [1.35, 1]})
x = np.arange(len(K))
for key, c, l in (("gap_mm_real_tn", "#c8781e", "실물 대 보정 전 트윈"), ("gap_mm_real_tc", "#1d7686", "실물 대 보정 후 트윈")):
    v = [R[k][key][0] for k in K]; lo = [R[k][key][1][0] for k in K]; hi = [R[k][key][1][1] for k in K]
    a.plot(x, v, "-o", ms=2.5, lw=1.2, color=c, label=l); a.fill_between(x, lo, hi, color=c, alpha=0.18, lw=0)
a.axvline(7.5, color="#999", lw=0.6)
a.text(3.5, a.get_ylim()[1] * 0.98, "v2 (보정 전 렌더, 천 스텝 단위)", ha="center", va="top", fontsize=7, color="#555")
a.text(10, a.get_ylim()[1] * 0.98, "v4 (보정 후 이어 학습)", ha="center", va="top", fontsize=7, color="#555")
a.set_xticks(x); a.set_xticklabels(lab, fontsize=6.5)
a.set_ylabel("1초 뒤 핀 끝 차이 (mm)")
a.set_title("(a) 같은 state에서 화면만 바꾼 동작 차이", fontsize=8)
a.legend(frameon=False, fontsize=6.5, loc="lower right")
HS = [0.02, 0.1, 0.33, 1.0, 3.3]
for m, c, l in (("predict", "#1d7686", "예측"), ("follow", "#c8781e", "따라가기")):
    v = [C[f"0.06|{m}|{h}"]["pct"] for h in HS]; lo = [C[f"0.06|{m}|{h}"]["wilson"][0] for h in HS]; hi = [C[f"0.06|{m}|{h}"]["wilson"][1] for h in HS]
    b.plot(HS, v, "-o", ms=2.5, lw=1.2, color=c, label=l); b.fill_between(HS, lo, hi, color=c, alpha=0.18, lw=0)
b.set_xscale("log"); b.set_xticks(HS); b.set_xticklabels(["0.02", "0.1", "0.33", "1", "3.3"])
b.set_xlabel("다시 계획하는 간격 H (s)"); b.set_ylabel("성공률 (%)"); b.set_ylim(0, 100)
b.set_title("(b) 묶음 실행 간격 (MuJoCo, 6 cm, n=100)", fontsize=8)
b.legend(frameon=False, fontsize=6.5)
for ax in (a, b):
    ax.spines[["top", "right"]].set_visible(False)
fig.tight_layout(pad=0.4)
fig.savefig(ROOT / "outputs/research/paper_fig.png")
print("-> outputs/research/paper_fig.png")
