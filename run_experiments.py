"""흔들리는 선박 소켓 삽입 실험: 파고 × 제어 전략 × 반복.

실행: .venv/bin/python run_experiments.py [--layout top|side]
결과: outputs/experiment_{layout}.json, outputs/exp_{layout}_성공률.png, outputs/exp_{layout}_접촉력.png
      (top: 갑판 위 소켓에 위에서 꽂기, side: 현측에 세운 소켓에 L자 플러그로 옆에서 꽂기.
       배는 부두에 옆으로 댄 것(롤 축이 부두와 나란함, waves.ShipMotion berth="alongside").
       예선 결과는 소켓을 키우기 전 top 배치, 선수 계류이며 experiment_small_socket.json에 남겨 두었다)
"""

import argparse
import json
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

from plugbot.control import Params, run_trial
from plugbot.scene import Scene
from plugbot.waves import ShipMotion

OUT = Path(__file__).resolve().parent / "outputs"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
plt.rcParams["axes.unicode_minus"] = False

HS = [0.0, 0.02, 0.04, 0.06, 0.08]
STRATEGIES = {"static": "처음 위치로(정적)", "reactive": "최신 위치 추종", "predictive": "AR 예측 + 정온 구간 삽입"}
STYLE = {"static": ("#9a9a9a", "o", ":"), "reactive": ("#c8781e", "s", "--"), "predictive": ("#0e5c3e", "^", "-")}
N = 30
SCALE = 10  # 축척 1:10 가정(Froude 상사: 길이 ×10, 시간 ×√10)


def summarize(trials):
    ins = [t for t in trials if t.inserted]
    reasons = {}
    for t in trials:
        if not t.inserted:
            reasons[t.reason] = reasons.get(t.reason, 0) + 1
    f = np.array([t.f_insert_p95 for t in trials])
    return {
        "성공률": len(ins) / len(trials),
        "삽입 시간 중앙값(s)": float(np.median([t.t_insert for t in ins])) if ins else None,
        "삽입 접촉력 p95 중앙값(N)": float(np.median(f)),
        "삽입 접촉력 p95 사분위(N)": [float(np.percentile(f, 25)), float(np.percentile(f, 75))],
        "실패 원인": reasons,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default="top", choices=["top", "side"])
    layout = ap.parse_args().layout
    OUT.mkdir(exist_ok=True)
    scene = Scene("ship", layout)
    p = Params()
    res = {"조건": {"반복": N, "카메라": f"{p.cam_hz:.0f} Hz, 지연 {p.latency:.2f} s, 잡음 σ {p.noise*1000:.1f} mm",
                   "제어/물리": "50 Hz / 500 Hz", "파도": "JONSWAP γ=3.3, Tp 2.0 s, 상하, 좌우, 앞뒤, 롤, 피치(부두에 옆으로 댄 배: 롤 축이 부두와 나란함)",
                   "소켓": "깔때기 포착 반경 19 mm, 반경 여유 2 mm",
                   "배치": "갑판 위 소켓, 위에서 꽂기(일자 플러그)" if layout == "top"
                   else "현측에 세운 소켓, 옆에서 꽂기(L자 플러그, 핀 65 mm)",
                   "축척": f"1:{SCALE} 가정 → 모형 Tp 2.0 s ≈ 실선 {2.0*np.sqrt(SCALE):.1f} s"},
           "결과": {}}
    t0 = time.time()
    for hs in HS:
        key = f"Hs={hs*100:.0f}cm (실선 약 {hs*SCALE:.1f} m)"
        res["결과"][key] = {}
        for s in STRATEGIES:
            trials = [run_trial(scene, s, ShipMotion(hs, seed=k, berth="alongside"), p, seed=k) for k in range(N)]
            res["결과"][key][s] = summarize(trials)
            r = res["결과"][key][s]
            print(f"{key:22s} {s:10s} 성공 {r['성공률']*100:5.1f}%  F p95 {r['삽입 접촉력 p95 중앙값(N)']:7.1f} N", flush=True)
    car = Scene("car", layout)
    car_trials = [run_trial(car, s, None, p, seed=k) for s in STRATEGIES for k in range(5)]
    res["자동차(정지 소켓, " + ("45° 충전구)" if layout == "top" else "세운 충전구)")] = summarize(car_trials)
    res["소요 시간(s)"] = round(time.time() - t0, 1)
    (OUT / f"experiment_{layout}.json").write_text(json.dumps(res, ensure_ascii=False, indent=2))
    plot(res, layout)
    print("완료", res["소요 시간(s)"], "s")


def plot(res, layout):
    keys = list(res["결과"])
    hs_cm = [h * 100 for h in HS]
    fig, ax = plt.subplots(figsize=(6.4, 3.8), dpi=200)
    for s, label in STRATEGIES.items():
        c, mk, ls = STYLE[s]
        ax.plot(hs_cm, [res["결과"][k][s]["성공률"] * 100 for k in keys], color=c, marker=mk, ls=ls, lw=2, ms=7, label=label)
    ax.set_xlabel(f"모형 유의파고 Hs (cm), 축척 1:{SCALE} 가정 시 실선 Hs = ×0.1 m")
    ax.set_ylabel(f"삽입 성공률 (%), 조건당 {N}회")
    ax.set_ylim(-3, 103)
    ax.set_xticks(hs_cm)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower left")
    ax.set_title("흔들리는 선박 소켓 삽입 성공률 (MuJoCo, SO-101 모델)", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / f"exp_{layout}_성공률.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.4, 3.8), dpi=200)
    for s, label in STRATEGIES.items():
        c, mk, ls = STYLE[s]
        med = np.array([res["결과"][k][s]["삽입 접촉력 p95 중앙값(N)"] for k in keys])
        q = np.array([res["결과"][k][s]["삽입 접촉력 p95 사분위(N)"] for k in keys])
        ax.errorbar(hs_cm, np.maximum(med, 0.5), yerr=[np.maximum(med - q[:, 0], 0), q[:, 1] - med],
                    color=c, marker=mk, ls=ls, lw=2, ms=7, capsize=3, label=label)
    ax.set_yscale("log")
    ax.set_xlabel("모형 유의파고 Hs (cm)")
    ax.set_ylabel("삽입 중 접촉력 95백분위 (N, 로그)")
    ax.set_xticks(hs_cm)
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("삽입 충격: 중앙값과 사분위 범위", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / f"exp_{layout}_접촉력.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
