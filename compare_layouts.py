"""두 배치(위에서 꽂기, 옆으로 꽂기) 비교 그림.

실행: .venv/bin/python compare_layouts.py  (run_experiments.py --layout top, --layout side 결과가 먼저 있어야 한다)
결과: outputs/layout_compare.png
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from matplotlib import font_manager
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import IK, home_pose
from plugbot.scene import Scene

OUT = Path(__file__).resolve().parent / "outputs"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
if not Path(FONT).exists():  # 맥 밖(우분투 GPU 노트북)에서는 Noto CJK
    FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
W, H = 900, 640


def snapshot(layout):
    s = Scene("ship", layout)
    m, d = s.model, s.data
    s.set_ship(np.zeros(3), 0.0, 0.0)
    mujoco.mj_forward(m, d)
    ent, ax = s.socket_frame()
    q0 = home_pose(s) if layout == "side" else np.array([0.0, -1.0, 0.8, 1.0, 0.0])
    q = q0
    for back in (0.03, 0.015, 0.008):  # 대기 자세에서 입구 쪽으로 조금씩(같은 갈래 유지)
        q = IK(s).solve(q, ent + ax * back, -ax, iters=200)
    d.qpos[s.qadr] = q
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.15, 0.0, 0.0]
    cam.distance, cam.azimuth, cam.elevation = 0.85, 235, -20
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, cam)
    return Image.fromarray(r.render())


def chart():
    hs = [0, 2, 4, 6, 8]
    small = json.loads((OUT / "experiment_small_socket.json").read_text())
    top = json.loads((OUT / "experiment_top.json").read_text())
    side = json.loads((OUT / "experiment_side.json").read_text())
    rate = lambda r, s: [r["결과"][k][s]["성공률"] * 100 for k in r["결과"]]
    fig, ax = plt.subplots(figsize=(W / 150, H / 150), dpi=150)
    ax.plot(hs, rate(top, "predictive"), color="#0e5c3e", marker="^", lw=2.2, label="위에서 꽂기, 예측")
    ax.plot(hs, rate(top, "reactive"), color="#0e5c3e", marker="^", lw=1.2, ls="--", alpha=0.6, label="위에서 꽂기, 추종")
    side_old = json.loads((OUT / "experiment_side_old.json").read_text())
    ax.plot(hs, rate(side, "predictive"), color="#c8781e", marker="s", lw=2.2, label="옆으로 꽂기(L자 핀 65 mm), 예측")
    ax.plot(hs, rate(side, "reactive"), color="#c8781e", marker="s", lw=1.2, ls="--", alpha=0.6, label="옆으로 꽂기, 추종")
    ax.plot(hs, rate(side_old, "predictive"), color="#c8781e", marker="x", lw=1.0, ls=":", alpha=0.7,
            label="옆으로 꽂기 첫 시도(핀 49 mm), 예측")
    ax.plot(hs, rate(small, "predictive"), color="#888888", marker="o", lw=1.2, ls=":", label="예선(작은 소켓), 예측")
    ax.set_xlabel("모형 유의파고 Hs (cm)")
    ax.set_ylabel("삽입 성공률 (%), 조건당 30회")
    ax.set_xticks(hs)
    ax.set_ylim(-3, 103)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    path = OUT / "_layout_chart.png"
    fig.savefig(path)
    plt.close(fig)
    return Image.open(path).convert("RGB").resize((W, H))


def main():
    panels = [snapshot("top"), snapshot("side"), chart()]
    caps = ["(a) 위에서 꽂기: 갑판 위 소켓, 일자 플러그", "(b) 옆으로 꽂기(채택): 현측 세운 소켓, L자 플러그",
            "(c) 파고별 성공률(입구 지름 50 mm, 옆으로 댄 배의 롤)"]
    f = ImageFont.truetype(FONT, 24)
    gap, cap = 14, 46
    out = Image.new("RGB", (W * 3 + gap * 2, H + cap), "white")
    dr = ImageDraw.Draw(out)
    for k, im in enumerate(panels):
        out.paste(im, (k * (W + gap), 0))
        dr.text((k * (W + gap) + 10, H + 10), caps[k], font=f, fill=(0, 0, 0))
    out.save(OUT / "layout_compare.png")
    print(OUT / "layout_compare.png")


if __name__ == "__main__":
    main()
