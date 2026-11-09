"""MuJoCo 비교 실험 장면 렌더: 같은 파도(같은 시드)에서 정지, 따라가기, 예측이 꽂는 모습.

tools/compare_predict_follow.py와 같은 장면(side 배치), 제어기, 판정. 실험 숫자는 그 json(조건당 100회).
결과: outputs/mujoco_compare_hs6_grid.png(세 방식 x 네 순간), outputs/mujoco_compare_hs6_final.png(마지막 순간 크게),
      outputs/mujoco_compare_hs6.mp4(세 칸 나란히, 실제 속도)
실행: MUJOCO_GL=egl .venv/bin/python tools/render_mujoco_compare.py [--seed 1002] [--hs 0.06]
"""

import argparse
import json
import sys
from pathlib import Path

import imageio.v2 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from plugbot.control import Params, run_trial  # noqa: E402
from plugbot.scene import Scene  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1002)
ap.add_argument("--hs", type=float, default=0.06)
a = ap.parse_args()
OUT = ROOT / "outputs"
S = [("static", "정지", "처음 본 위치로 간다"), ("reactive", "따라가기", "지금 보이는 위치를 쫓는다"),
     ("predictive", "예측", "0.3초 앞을 보고 잔잔할 때 꽂는다")]
COL = {"static": (120, 120, 120), "reactive": (200, 120, 30), "predictive": (107, 63, 214)}


def font(sz, bold=True):
    return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc" % ("Bold" if bold else "Regular"), sz, index=1)


sc = Scene("ship", "side")
p = Params()
recs, trs = {}, {}
for s, *_ in S:
    rec = []
    trs[s] = run_trial(sc, s, ShipMotion(a.hs, seed=a.seed, berth="alongside"), p, seed=a.seed, record=rec)
    recs[s] = rec
    print(s, "성공" if trs[s].inserted else "실패: " + trs[s].reason, f"{rec[-1][0]:.2f}s")


def put(item):
    t, qpos, mp, mq = item[:4]
    sc.data.qpos[:] = qpos
    sc.data.mocap_pos[:] = mp
    sc.data.mocap_quat[:] = mq
    mujoco.mj_forward(sc.model, sc.data)


sid = mujoco.mj_name2id(sc.model, mujoco.mjtObj.mjOBJ_SITE, "plug_tip")
put(recs["predictive"][-1])
tip = sc.data.site_xpos[sid].copy()


def cam(dist, az, el, dx=-0.08):
    c = mujoco.MjvCamera()
    c.lookat[:] = tip + np.array([dx, 0, 0])
    c.distance, c.azimuth, c.elevation = dist, az, el
    return c


WIDE = cam(0.6, 90, -20)
CLOSE = cam(0.30, 75, -12, dx=-0.03)
r = mujoco.Renderer(sc.model, 480, 640)


def shot(item, c):
    put(item)
    r.update_scene(sc.data, c)
    return Image.fromarray(r.render())


def status(s, i):
    rec, tr = recs[s], trs[s]
    if i >= len(rec) - 1:
        return ("꽂힘" if tr.inserted else "실패: " + tr.reason), (40, 160, 70) if tr.inserted else (210, 50, 50)
    return f"{rec[i][0]:.1f}초", (90, 90, 90)


# 1) 세 방식 x 네 순간(시작, 1/3, 2/3, 끝)
TW, TH = 480, 360
grid = Image.new("RGB", (220 + 4 * TW, 90 + 3 * (TH + 50)), "white")
d = ImageDraw.Draw(grid)
d.text((20, 20), f"같은 파도(모형 파고 {a.hs * 100:.0f} cm, 시드 {a.seed})에서 세 방식, MuJoCo 시뮬레이션", font=font(34), fill=(20, 20, 20))
for row, (s, name, desc) in enumerate(S):
    rec = recs[s]
    y = 90 + row * (TH + 50)
    d.text((20, y + 120), name, font=font(40), fill=COL[s])
    d.text((20, y + 180), desc[:9], font=font(20, False), fill=(90, 90, 90))
    d.text((20, y + 208), desc[9:], font=font(20, False), fill=(90, 90, 90))
    for col, f in enumerate([0.0, 0.4, 0.75, 1.0]):
        i = min(int(f * (len(rec) - 1)), len(rec) - 1)
        im = shot(rec[i], WIDE if col < 3 else CLOSE).resize((TW, TH))
        grid.paste(im, (220 + col * TW, y))
        txt, c = status(s, i)
        d.text((220 + col * TW + 8, y + TH + 6), txt + (" (가까이)" if col == 3 else ""), font=font(24), fill=c)
grid.save(OUT / "mujoco_compare_hs6_grid.png")

# 2) 마지막 순간 크게
fin = Image.new("RGB", (3 * 640, 480 + 130), "white")
d = ImageDraw.Draw(fin)
for k, (s, name, desc) in enumerate(S):
    fin.paste(shot(recs[s][-1], CLOSE), (k * 640, 110))
    d.text((k * 640 + 20, 10), name, font=font(40), fill=COL[s])
    txt, c = status(s, len(recs[s]) - 1)
    d.text((k * 640 + 20, 62), txt, font=font(26), fill=c)
fin.save(OUT / "mujoco_compare_hs6_final.png")

# 3) 영상: 세 칸 나란히, 실제 속도, 끝난 칸은 마지막 장면 유지
fps = 30
T = max(rec[-1][0] for rec in recs.values()) + 1.5
w = iio.get_writer(OUT / "mujoco_compare_hs6.mp4", fps=fps, codec="libx264", quality=7, macro_block_size=8)
times = {s: np.array([x[0] for x in recs[s]]) for s, *_ in S}
for j in range(int(T * fps)):
    t = j / fps
    frame = Image.new("RGB", (1920, 1080), "white")
    d = ImageDraw.Draw(frame)
    d.text((40, 30), f"같은 파도에서 정지, 따라가기, 예측 (MuJoCo, 모형 파고 {a.hs * 100:.0f} cm, 실제 속도)", font=font(40), fill=(20, 20, 20))
    for k, (s, name, desc) in enumerate(S):
        i = int(np.searchsorted(times[s], t))
        i = min(i, len(recs[s]) - 1)
        frame.paste(shot(recs[s][i], WIDE), (k * 640, 200))
        frame.paste(shot(recs[s][i], CLOSE).resize((640, 300)), (k * 640, 690))
        d.text((k * 640 + 20, 110), name, font=font(44), fill=COL[s])
        d.text((k * 640 + 160, 125), desc, font=font(24, False), fill=(90, 90, 90))
        txt, c = status(s, i)
        d.text((k * 640 + 20, 1000), txt, font=font(30), fill=c)
    w.append_data(np.asarray(frame))
w.close()
r.close()
res = json.loads((OUT / "compare_predict_follow.json").read_text())["결과"].get(f"{a.hs * 100:.0f}cm")
if res:
    print("100회 결과:", {s: res[s]["성공률"] for s, *_ in S})
print("저장 outputs/mujoco_compare_hs6_grid.png, _final.png, .mp4")
