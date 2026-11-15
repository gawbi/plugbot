"""통계 보강: 짝 결과의 McNemar 정확 검정, 2x2 Fisher 정확 검정, Wilson 구간. 외부 라이브러리 없이 계산한다.

- MuJoCo 예측 대 따라가기(같은 시드 100개): outputs/compare_predict_follow.json의 짝 비교 개수
- 트윈 체크포인트: outputs/research/rollout_audit.json(이 노트북 기록)에서 v4 1천 대 2천~4천
- 실물 R2R 대 S2R: outputs/real_results.json(두 노트북 합산)
실행: python3 tools/research/stats.py  →  outputs/research/stats.json
"""

import json
from math import comb, sqrt
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def wilson(k, n, z=1.96):
    if n == 0:
        return [0.0, 100.0]
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [round(max(0, c - h) * 100, 1), round(min(1, c + h) * 100, 1)]


def mcnemar_exact(b, c):
    """짝지은 이진 결과에서 한쪽만 성공한 개수 b, c. 양측 정확 p = 2 * P(X <= min(b,c)), X ~ Bin(b+c, 0.5)."""
    n, k = b + c, min(b, c)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)


def fisher_exact(a, b, c, d, side="two"):
    """2x2 표 [[a, b], [c, d]] (행 = 집단, 열 = 성공, 실패). side: 'two' 또는 'greater'(첫 집단 성공률이 더 큼)."""
    r1, r2, c1, n = a + b, c + d, a + c, a + b + c + d

    def pmf(x):
        return comb(r1, x) * comb(r2, c1 - x) / comb(n, c1)

    lo, hi = max(0, c1 - r2), min(r1, c1)
    p0 = pmf(a)
    if side == "greater":
        return sum(pmf(x) for x in range(a, hi + 1))
    return min(1.0, sum(pmf(x) for x in range(lo, hi + 1) if pmf(x) <= p0 * (1 + 1e-9)))


out = {"note": "정확 검정. MuJoCo는 시뮬레이션, 트윈 체크포인트는 이 노트북 실물 기록, R2R 대 S2R은 두 노트북 합산 실물 기록."}

# 1) MuJoCo 짝 비교
cp = json.loads((ROOT / "outputs/compare_predict_follow.json").read_text())["결과"]
out["mujoco_predict_vs_follow"] = {}
for hs, r in cp.items():
    b, c = r["짝 비교"]["예측만 성공"], r["짝 비교"]["따라가기만 성공"]
    out["mujoco_predict_vs_follow"][hs] = {"only_predict": b, "only_follow": c, "both": r["짝 비교"]["둘 다 성공"],
                                           "neither": r["짝 비교"]["둘 다 실패"], "mcnemar_exact_p": mcnemar_exact(b, c),
                                           "predict_pct": r["predictive"]["성공률"], "follow_pct": r["reactive"]["성공률"]}

# 2) 트윈 v4: 1천 대 2천~4천 (정지 소켓, 이 노트북 기록)
au = json.loads((ROOT / "outputs/research/rollout_audit.json").read_text())["by_model_ckpt_cond"]
k1 = au["act_twin_v4/001000|static"]
ok1, n1 = k1["ok"], k1["ok"] + k1["fail"]
later = [au[f"act_twin_v4/00{s}000|static"] for s in (2, 3, 4)]
ok2, n2 = sum(x["ok"] for x in later), sum(x["ok"] + x["fail"] for x in later)
out["twin_v4_1k_vs_2k_4k_static"] = {"1k": f"{ok1}/{n1}", "2k_4k": f"{ok2}/{n2}",
                                    "fisher_one_sided_p": fisher_exact(ok1, n1 - ok1, ok2, n2 - ok2, "greater"),
                                    "fisher_two_sided_p": fisher_exact(ok1, n1 - ok1, ok2, n2 - ok2),
                                    "caveat": "2천~4천은 각 1회, 판정은 실행 뒤(retro). 연속 시도, 눈가림 없음"}

# 3) 실물 R2R 대 S2R (합산)
rr = json.loads((ROOT / "outputs/real_results.json").read_text())["results"]
tot = {}
for r in rr:
    t = tot.setdefault(r["pipeline"], [0, 0])
    t[0] += r["ok"]; t[1] += r["n"]
(a_, n_a), (c_, n_c) = tot["real_to_real"], tot["sim_to_real"]
out["real_r2r_vs_s2r"] = {"r2r": f"{a_}/{n_a}", "s2r": f"{c_}/{n_c}", "r2r_wilson": wilson(a_, n_a), "s2r_wilson": wilson(c_, n_c),
                          "fisher_two_sided_p": fisher_exact(a_, n_a - a_, c_, n_c - c_),
                          "reading": "차이를 찾지 못했다. 같다는 증거는 아니다"}

dst = ROOT / "outputs/research/stats.json"
dst.write_text(json.dumps(out, ensure_ascii=False, indent=1))
for hs, r in out["mujoco_predict_vs_follow"].items():
    print(f"MuJoCo {hs}: 예측만 {r['only_predict']} 대 따라가기만 {r['only_follow']}, McNemar p = {r['mcnemar_exact_p']:.2e}")
t = out["twin_v4_1k_vs_2k_4k_static"]
print(f"트윈 v4 정지: 1천 {t['1k']} 대 2천~4천 {t['2k_4k']}, Fisher 단측 p = {t['fisher_one_sided_p']:.4f}, 양측 {t['fisher_two_sided_p']:.4f}")
r = out["real_r2r_vs_s2r"]
print(f"실물 R2R {r['r2r']} {r['r2r_wilson']} 대 S2R {r['s2r']} {r['s2r_wilson']}, Fisher 양측 p = {r['fisher_two_sided_p']:.3f}")
