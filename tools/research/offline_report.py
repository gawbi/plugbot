"""E1 결과 정리 그림: 두 구간(접근 초반 2~30%, 접근과 꽂기 30~97%)에서
 (가) 같은 state의 실물 화면 대 트윈 화면 동작 차이(보정 후 tc, 보정 전 tn)
 (나) 사람 시범과의 오차: 실물 화면 대 회색 화면(대조)
입력: outputs/research/offline_eval_early.json, offline_eval.json
결과: outputs/research/offline_eval.png, offline_summary.json
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
W = {"early": json.loads((ROOT / "outputs/research/offline_eval_early.json").read_text()),
     "late": json.loads((ROOT / "outputs/research/offline_eval.json").read_text())}
TWIN = [f"act_twin_v2/{s:06d}" for s in range(2500, 20001, 2500)] + [f"act_twin_v4/{s:06d}" for s in range(1000, 5001, 1000)]
LAB = [f"v2 {int(s.split('/')[1]) // 1000 if int(s.split('/')[1]) % 1000 == 0 else int(s.split('/')[1]) / 1000}k" for s in TWIN[:8]] + [f"v4 {i}k" for i in range(1, 6)]
OUT_REAL = {"act_twin_v2/005000": "성공 1/1", "act_twin_v2/007500": "성공 1/1", "act_twin_v2/010000": "빗나감(관찰)",
            "act_twin_v2/012500": "빗나감(관찰)", "act_twin_v2/015000": "빗나감(관찰)", "act_twin_v4/001000": "13/13 + 다른 노트북",
            "act_twin_v4/002000": "실패 0/1", "act_twin_v4/003000": "실패 0/1", "act_twin_v4/004000": "실패 0/1"}

summ = {}
for w, R in W.items():
    tc = [R[c]["gap_mm_real_tc"][0] for c in TWIN]
    tn = [R[c]["gap_mm_real_tn"][0] for c in TWIN]
    real = [R[c]["gt_mm_real"][0] for c in TWIN]
    gray = [R[c]["gt_mm_gray"][0] for c in TWIN]
    summ[w] = {"window": f"{R['_setup']['progress_from']}~{R['_setup'].get('progress_to', 1.0)}",
               "gap_tc_lt_tn_all": all(a < b for a, b in zip(tc, tn)), "gap_tc_range": [min(tc), max(tc)], "gap_tn_range": [min(tn), max(tn)],
               "twin_real_minus_gray_median": round(float(np.median(np.array(real) - np.array(gray))), 1),
               "real_model": {"real": R["act_real_v4/080000"]["gt_mm_real"], "gray": R["act_real_v4/080000"]["gt_mm_gray"]}}
(ROOT / "outputs/research/offline_summary.json").write_text(json.dumps(summ, ensure_ascii=False, indent=1))
print(json.dumps(summ, ensure_ascii=False, indent=1))

F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=F).get_name()
fig, axs = plt.subplots(2, 2, figsize=(15, 8.4), dpi=140, sharex=True)
x = np.arange(len(TWIN))
for col, (w, title) in enumerate((("early", "접근 초반 (진행 2~30%)"), ("late", "접근과 꽂기 (진행 30~97%)"))):
    R = W[w]
    ax = axs[0, col]
    for key, c, lab in (("gap_mm_real_tc", "#1d7686", "실물 화면 대 보정 후 트윈 화면"), ("gap_mm_real_tn", "#c8781e", "실물 화면 대 보정 전 트윈 화면")):
        v = np.array([R[k][key][0] for k in TWIN]); lo = np.array([R[k][key][1][0] for k in TWIN]); hi = np.array([R[k][key][1][1] for k in TWIN])
        ax.plot(x, v, "-o", color=c, label=lab); ax.fill_between(x, lo, hi, color=c, alpha=0.15)
    ax.set_title(f"(가) {title}: 같은 순간 화면만 바꿨을 때 동작 차이", fontsize=11)
    ax.set_ylabel("1초 뒤 핀 끝 차이 (mm)")
    ax.axvline(7.5, color="#bbb", lw=1)
    ax = axs[1, col]
    for key, c, lab in (("gt_mm_real", "#6b3fd6", "실물 화면"), ("gt_mm_gray", "#9aa3ae", "회색 화면(대조)"), ("gt_mm_tc", "#1d7686", "보정 후 트윈 화면")):
        v = np.array([R[k][key][0] for k in TWIN]); lo = np.array([R[k][key][1][0] for k in TWIN]); hi = np.array([R[k][key][1][1] for k in TWIN])
        ax.plot(x, v, "-o", color=c, label=lab, ms=4); ax.fill_between(x, lo, hi, color=c, alpha=0.12)
    rr = R["act_real_v4/080000"]
    ax.axhline(rr["gt_mm_real"][0], color="#6b3fd6", ls=":", lw=1.5)
    ax.text(0, rr["gt_mm_real"][0] + 1.5, "실물 화면으로 학습한 모델, 실물 화면(참고: 이 에피소드로 학습함)", color="#6b3fd6", fontsize=8, ha="left")
    ax.set_title(f"(나) {title}: 사람 시범과의 차이", fontsize=11)
    ax.set_ylabel("1초 뒤 핀 끝 차이 (mm)")
    ax.axvline(7.5, color="#bbb", lw=1)
    for i, k in enumerate(TWIN):
        if k in OUT_REAL:
            ax.text(i, ax.get_ylim()[1], OUT_REAL[k], rotation=90, fontsize=7, color="#555", va="top", ha="center")
for ax in axs.flat:
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_xticks(x)
    ax.set_xticklabels(LAB, rotation=45, fontsize=8)
axs[0, 0].legend(frameon=False, fontsize=9)
axs[1, 0].legend(frameon=False, fontsize=9, loc="upper right")
fig.suptitle("학습에 안 쓴 실물 에피소드 25개(plug_static 69~93), 에피소드 단위 부트스트랩 95% 구간. 세로선 왼쪽 v2(보정 전 렌더), 오른쪽 v4(보정 후 렌더로 이어 학습)", fontsize=10)
fig.tight_layout()
fig.savefig(ROOT / "outputs/research/offline_eval.png")
print("-> outputs/research/offline_eval.png")
