"""흔들림 갑판 움직임 재기 시도: 실행 기록의 앞 카메라 화면에서 흰 소켓의 윗선을 따라간다.

결과(2026-11-15): 앞 카메라가 갑판과 함께 움직이는 포토박스에 붙어 있어서, 화면 속 소켓은 흔들림 실행에서도 정지 실행과
비슷하게 1~2 mm만 움직인다. 그래서 이 방법으로는 갑판 진폭을 잴 수 없다. 흔들림 실행 일부에서 잡히는 약 2.5 s 주기는
카메라 받침의 미세한 상대 운동으로 보이며, 갑판이 설정(정현파 2.5 s)대로 돌았다는 정황만 준다.
진폭을 남기려면 실행마다 /dev/shm/plugbot_shaker.json(명령, 측정 높이)을 함께 저장해야 한다.

팔이 소켓 근처에 오기 전(처음 몇 초)만 쓴다. 정지 실행도 같이 재서 측정 잡음의 기준으로 삼는다.
화소 → mm: 흰 소켓 지름 60 mm(isaac/twin/twin_meshes.npz의 sock_v)와 화면 속 소켓 높이(화소)의 비. 비스듬한 시점이라
환산은 대략값(±20% 정도)이다.
실행: ~/lerobot_env/bin/python tools/research/deck_motion.py
결과: outputs/research/deck_motion.json, outputs/research/deck_motion.png
"""

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SOCKET_MM = 60.0
FPS = 30.0
MAX_FRAMES = 240  # 8 s 안에서
audit = json.loads((ROOT / "outputs/research/rollout_audit.json").read_text())["runs"]


def socket_rows(img):
    """(윗선 y, 무게중심 y, 높이 px). 소켓이 안 보이거나 팔이 가리면 None"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    roi = hsv[150:, :220]
    white = ((roi[..., 2] > 190) & (roi[..., 1] < 70)).astype(np.uint8)
    purple = ((roi[..., 0] > 120) & (roi[..., 0] < 160) & (roi[..., 1] > 80) & (roi[..., 2] > 60)).sum()
    if purple > 400:  # 보라 덮개가 소켓 근처에 들어옴
        return None
    n, lab, st, cen = cv2.connectedComponentsWithStats(white)
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[i, cv2.CC_STAT_AREA] < 3000:
        return None
    ys = np.where(lab == i)[0]
    return float(np.percentile(ys, 1)) + 150, float(cen[i][1]) + 150, float(st[i, cv2.CC_STAT_HEIGHT])


def period_s(x):
    x = x - x.mean()
    if len(x) < 90 or x.std() < 0.5:
        return None
    ac = np.correlate(x, x, "full")[len(x) - 1:]
    lo, hi = int(1.2 * FPS), min(len(ac) - 1, int(4.5 * FPS))
    k = lo + int(np.argmax(ac[lo:hi]))
    return round(k / FPS, 2) if ac[k] > 0.2 * ac[0] else None


rows = []
for r in audit:
    if r["ok"] is None or r["model"] not in ("act_real_v4", "act_twin_v4") or r["frames"] < 60:
        continue
    d = ROOT / "data/rollouts" / r["run"] / "front"
    top, cen, hgt = [], [], []
    for k in range(min(MAX_FRAMES, r["frames"])):
        m = socket_rows(cv2.imread(str(d / f"{k:05d}.jpg")))
        if m is None:
            if k > 30:
                break  # 팔이 들어오면 거기서 끝
            continue
        top.append(m[0]); cen.append(m[1]); hgt.append(m[2])
    if len(top) < 45:
        continue
    top = np.convolve(np.array(top), np.ones(3) / 3, "valid")
    mm_per_px = SOCKET_MM / float(np.median(hgt))
    p2p_px = float(np.percentile(top, 97) - np.percentile(top, 3))
    rows.append({"run": r["run"], "model": r["model"], "ckpt": r["ckpt"], "cond": r["cond"], "ok": r["ok"],
                 "seconds_tracked": round(len(top) / FPS, 1), "p2p_px": round(p2p_px, 1),
                 "p2p_mm_approx": round(p2p_px * mm_per_px, 1), "period_s": period_s(top),
                 "trace_px": [round(float(v), 1) for v in top]})

summary = {}
for cond in ("static", "moving"):
    xs = [x for x in rows if x["cond"] == cond]
    if xs:
        p = np.array([x["p2p_mm_approx"] for x in xs])
        per = [x["period_s"] for x in xs if x["period_s"]]
        summary[cond] = {"runs": len(xs), "p2p_mm_median": round(float(np.median(p)), 1),
                         "p2p_mm_range": [round(float(p.min()), 1), round(float(p.max()), 1)],
                         "period_s_median": round(float(np.median(per)), 2) if per else None, "period_found": len(per)}
out = {"note": "앞 카메라 흰 소켓 윗선의 상하 움직임(팔이 오기 전 구간). 카메라가 갑판과 같이 움직여 갑판 진폭은 재지 못한다. "
               "흔들림과 정지의 분포가 겹치는 것이 그 근거. 주기 약 2.5 s는 흔들림 실행 일부에서만 잡힌다.", "summary": summary,
       "runs": [{k: v for k, v in x.items() if k != "trace_px"} for x in rows]}
dst = ROOT / "outputs/research/deck_motion.json"
dst.write_text(json.dumps(out, ensure_ascii=False, indent=1))
print(json.dumps(summary, ensure_ascii=False, indent=1))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=F).get_name()
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), dpi=150)
for x in rows:
    t = np.arange(len(x["trace_px"])) / FPS
    y = np.array(x["trace_px"]) - np.mean(x["trace_px"])
    ax[0].plot(t, y, color="#6b3fd6" if x["cond"] == "moving" else "#9aa3ae", lw=0.8, alpha=0.6)
ax[0].set(xlabel="시간 (s)", ylabel="소켓 윗선 위치 변화 (px)", title="앞 카메라 속 소켓 (카메라도 갑판과 함께 움직임)")
ax[0].invert_yaxis()
for cond, c in (("static", "#9aa3ae"), ("moving", "#6b3fd6")):
    v = [x["p2p_mm_approx"] for x in rows if x["cond"] == cond]
    ax[1].hist(v, bins=12, color=c, alpha=0.8, label=f"{'정지' if cond == 'static' else '흔들림'} {len(v)}회")
ax[1].set(xlabel="상하 변위 (봉우리-골, mm, 대략)", ylabel="실행 수", title="흔들림과 정지가 겹친다: 이 카메라로는 갑판 진폭을 못 잰다")
ax[1].legend(frameon=False)
for a in ax:
    a.spines[["top", "right"]].set_visible(False)
fig.tight_layout()
fig.savefig(ROOT / "outputs/research/deck_motion.png")
print("->", dst.relative_to(ROOT), len(rows), "runs")
