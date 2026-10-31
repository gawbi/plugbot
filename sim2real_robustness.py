"""Sim-to-Real 강건성 시험: 실물이 시뮬레이션과 다를 수 있는 값을 하나씩 바꿔 가며 성공률이 어디까지 버티는지 잰다.

바꾸는 값(나머지는 기본값, side 배치, 부두에 옆으로 댄 배, 모형 Hs 4 cm, 조건당 30회):
- 카메라 지연(s): 웹캠, USB, 영상 처리 지연. 시뮬레이션 기본 0.10
- 위치 측정 잡음(mm): 마커 추적의 흔들림. 기본 0.5
- 보정 오차(mm): 카메라→로봇 변환이 틀린 만큼 소켓 위치가 늘 어긋남(소켓 축에 수직 방향)
- 서보 강성 배율: 실물 STS3215 위치 제어가 모델보다 무르거나 단단함
- 소켓 마찰 계수: PLA 표면, 모따기 상태
- 파도 주기(s): 흔들림 장치 주기가 가정과 다름
그림의 회색 띠는 실물에서 예상하는 범위(가정)다. 본선 전 키트에서 실제 값을 재어 이 띠를 바꾼다.

실행: .venv/bin/python sim2real_robustness.py   (6개 묶음을 병렬로, 약 2~4분)
결과: outputs/sim2real_robustness.json, outputs/sim2real_robustness.png
"""

import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "outputs"
HS, N, LAYOUT = 0.04, 30, "side"
STRATEGIES = ("predictive", "reactive")
SWEEPS = {  # 이름: (단위 설명, 값들, 실물 예상 범위(가정), 시뮬레이션 기본값)
    "latency": ("카메라 지연 (s)", [0.05, 0.10, 0.15, 0.20, 0.30], (0.08, 0.15), 0.10),
    "noise": ("위치 측정 잡음 σ (mm)", [0.5, 1.0, 2.0, 3.0, 5.0], (0.5, 1.5), 0.5),
    "calib": ("보정 오차 (mm, 소켓 축에 수직)", [0, 2, 4, 6, 8, 10], (1.0, 4.0), 0),
    "kp": ("서보 강성 배율 (모델 = 1)", [0.4, 0.6, 0.8, 1.0, 1.5], (0.6, 1.2), 1.0),
    "friction": ("소켓 마찰 계수", [0.1, 0.3, 0.6, 1.0], (0.3, 0.6), 0.3),
    "period": ("파도 주기 Tp (s)", [1.5, 2.0, 2.5, 3.0], (2.0, 3.0), 2.0),
}


def run_sweep(name):
    from plugbot.control import Params, run_trial
    from plugbot.randomize import perturb, restore
    from plugbot.scene import Scene
    from plugbot.waves import ShipMotion
    scene = Scene("ship", LAYOUT)
    rows = {}
    for v in SWEEPS[name][1]:
        p, tp = Params(), 2.0
        restore(scene)
        if name == "latency":
            p = replace(p, latency=v)
        elif name == "noise":
            p = replace(p, noise=v / 1000)
        elif name == "calib":
            p = replace(p, calib_bias=(0.0, v / 1000 / np.sqrt(2), v / 1000 / np.sqrt(2)))
        elif name == "kp":
            perturb(scene, kp_scale=v)
        elif name == "friction":
            perturb(scene, friction=v)
        elif name == "period":
            tp = v
        rows[str(v)] = {}
        for s in STRATEGIES:
            trials = [run_trial(scene, s, ShipMotion(HS, tp=tp, seed=k, berth="alongside"), p, seed=k) for k in range(N)]
            fails = {}
            for t in trials:
                if not t.inserted:
                    fails[t.reason] = fails.get(t.reason, 0) + 1
            rows[str(v)][s] = {"성공률": float(np.mean([t.inserted for t in trials])), "실패 원인": fails}
        print(f"{name} {v}: " + ", ".join(f"{s} {rows[str(v)][s]['성공률'] * 100:.0f}%" for s in STRATEGIES), flush=True)
    restore(scene)
    return name, rows


def wilson(p, n, z=1.96):
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return c - h, c + h


def plot(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
    font_manager.fontManager.addfont(font)
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    style = {"predictive": ("#1a8a5c", "^", "-", "예측(AR + 정온 구간)"), "reactive": ("#c8781e", "s", "--", "추종")}
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.4), dpi=150)
    for ax, (name, (label, vals, band, base)) in zip(axes.flat, SWEEPS.items()):
        x = np.array(vals, float)
        ax.axvspan(*band, color="#d9dde2", alpha=0.6, lw=0)
        ax.axvline(base, color="#8a929b", lw=1, ls=":")
        for s in STRATEGIES:
            col, mk, ls, lab = style[s]
            y = np.array([res[name][str(v)][s]["성공률"] for v in vals]) * 100
            lo, hi = zip(*[wilson(v / 100, N) for v in y])
            ax.fill_between(x, np.array(lo) * 100, np.array(hi) * 100, color=col, alpha=0.10, lw=0)
            ax.plot(x, y, color=col, marker=mk, ls=ls, lw=2, ms=7, label=lab)
        ax.set_title(label, fontsize=11, color="#1b1f24")
        ax.set_ylim(-3, 103)
        ax.grid(alpha=0.25)
        ax.tick_params(colors="#5d6670", labelsize=9)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[0, 0].set_ylabel("삽입 성공률 (%)")
    axes[1, 0].set_ylabel("삽입 성공률 (%)")
    h, l = axes[0, 0].get_legend_handles_labels()
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    h += [Patch(color="#d9dde2", label="실물 예상 범위(가정, 본선 전 측정)"),
          Line2D([], [], color="#8a929b", ls=":", label="시뮬레이션 기본값")]
    fig.suptitle(f"Sim-to-Real 강건성: 한 가지씩 실물과 다르게 했을 때 (side 배치, 모형 Hs {HS * 100:.0f} cm, 조건당 {N}회, "
                 "옅은 띠 = 95% 신뢰구간)", y=0.985, fontsize=12)
    fig.legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, 0.955), ncol=4, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(OUT / "sim2real_robustness.png")


def main():
    import sys
    if "--plot-only" in sys.argv:  # 저장된 결과로 그림만 다시
        plot(json.loads((OUT / "sim2real_robustness.json").read_text())["결과"])
        return
    with ProcessPoolExecutor(max_workers=len(SWEEPS)) as ex:
        res = dict(ex.map(run_sweep, SWEEPS))
    OUT.mkdir(exist_ok=True)
    meta = {"배치": LAYOUT, "모형 Hs": HS, "반복": N, "파도": "옆으로 댄 배(롤 축이 부두와 나란함)",
            "바꾼 값": {k: {"설명": v[0], "값": v[1], "실물 예상 범위(가정)": v[2], "기본값": v[3]} for k, v in SWEEPS.items()}}
    (OUT / "sim2real_robustness.json").write_text(json.dumps({"조건": meta, "결과": res}, ensure_ascii=False, indent=1))
    plot(res)
    print(OUT / "sim2real_robustness.png")


if __name__ == "__main__":
    main()
