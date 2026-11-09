"""예측 대 따라가기(그리고 정지) MuJoCo 비교, 새 파도 시드로 다시 재기(run_experiments.py와 같은 장면, 제어기, 판정).

같은 시드(같은 파도, 같은 카메라 잡음)로 세 전략을 돌려 짝 비교한다: '예측만 성공', '따라가기만 성공' 횟수.
실행: .venv/bin/python tools/compare_predict_follow.py [--n 100] [--seed0 1000]
결과: outputs/compare_predict_follow.json, outputs/compare_predict_follow.png
"""

import argparse
import json
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from plugbot.control import Params, run_trial  # noqa: E402
from plugbot.scene import Scene  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=100)
ap.add_argument("--seed0", type=int, default=1000, help="run_experiments.py는 0~29를 썼다. 겹치지 않게 1000부터")
ap.add_argument("--hs", default="0.04,0.06,0.08")
a = ap.parse_args()
HS = [float(x) for x in a.hs.split(",")]
S = {"static": "정지(처음 위치로)", "reactive": "따라가기(최신 위치 추종)", "predictive": "예측(0.3초 앞 + 잔잔할 때 삽입)"}


def wilson(k, n, z=1.96):
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [round(max(0, c - h) * 100, 1), round(min(1, c + h) * 100, 1)]


scene = Scene("ship", "side")
p = Params()
res = {"조건": {"반복": a.n, "시드": f"{a.seed0}~{a.seed0 + a.n - 1}", "배치": "side(현측에 세운 소켓, L자 플러그)",
               "카메라": f"{p.cam_hz:.0f} Hz, 지연 {p.latency:.2f} s, 잡음 σ {p.noise * 1000:.1f} mm", "엔진": "MuJoCo"}, "결과": {}}
t0 = time.time()
for hs in HS:
    key = f"{hs * 100:.0f}cm"
    ok = {}
    force = {}
    for s in S:
        tr = [run_trial(scene, s, ShipMotion(hs, seed=k, berth="alongside"), p, seed=k) for k in range(a.seed0, a.seed0 + a.n)]
        ok[s] = np.array([t.inserted for t in tr])
        force[s] = float(np.median([t.f_insert_p95 for t in tr]))
    r = {}
    for s in S:
        k = int(ok[s].sum())
        r[s] = {"성공": k, "회": a.n, "성공률": round(k / a.n * 100, 1), "95% 구간": wilson(k, a.n), "접촉력 p95 중앙값(N)": round(force[s], 1)}
    r["짝 비교"] = {"예측만 성공": int((ok["predictive"] & ~ok["reactive"]).sum()),
                  "따라가기만 성공": int((~ok["predictive"] & ok["reactive"]).sum()),
                  "둘 다 성공": int((ok["predictive"] & ok["reactive"]).sum()),
                  "둘 다 실패": int((~ok["predictive"] & ~ok["reactive"]).sum())}
    res["결과"][key] = r
    print(f"Hs {key}: 정지 {r['static']['성공률']}%  따라가기 {r['reactive']['성공률']}%  예측 {r['predictive']['성공률']}%  "
          f"짝: 예측만 {r['짝 비교']['예측만 성공']}, 따라가기만 {r['짝 비교']['따라가기만 성공']}", flush=True)
res["소요 시간(s)"] = round(time.time() - t0, 1)
(ROOT / "outputs/compare_predict_follow.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))

F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=F).get_name()
fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=200)
xs = np.arange(len(HS))
col = {"static": "#9a9a9a", "reactive": "#c8781e", "predictive": "#6b3fd6"}
for i, s in enumerate(S):
    v = [res["결과"][f"{h * 100:.0f}cm"][s]["성공률"] for h in HS]
    ci = np.array([res["결과"][f"{h * 100:.0f}cm"][s]["95% 구간"] for h in HS])
    b = ax.bar(xs + (i - 1) * 0.27, v, 0.25, color=col[s], label=S[s],
               yerr=[np.array(v) - ci[:, 0], ci[:, 1] - np.array(v)], capsize=3, error_kw={"lw": 1, "ecolor": "#555"})
    for x, y in zip(xs + (i - 1) * 0.27, v):
        ax.text(x, y + 2, f"{y:.0f}%", ha="center", fontsize=10)
ax.set_xticks(xs, [f"모형 파고 {h * 100:.0f} cm\n(실선 약 {h * 10:.1f} m 가정)" for h in HS])
ax.set_ylim(0, 112)
ax.set_ylabel(f"꽂기 성공률(%), 조건당 {a.n}회, 막대 = 95% 구간")
ax.set_title("같은 파도에서 정지, 따라가기, 예측 (MuJoCo 시뮬레이션, 새 시드)", fontsize=13)
ax.legend(loc="upper right", fontsize=9)
ax.grid(axis="y", alpha=0.3)
fig.tight_layout()
fig.savefig(ROOT / "outputs/compare_predict_follow.png")
print("완료", res["소요 시간(s)"], "s")
