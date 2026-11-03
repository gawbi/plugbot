"""그리퍼 덮개 칸막이가 어디에 붙어 있는지: 턱 방향(z)으로 자른 가로 단면 세 장. 실행: ~/cad_env/bin/python tools/draw_gripper_cover_septum.py
결과: cad/out/gripper_cover_septum.png"""
import importlib.util

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(F)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=F).get_name()
spec = importlib.util.spec_from_file_location("g", "/home/cho/plugbot/cad/make_gripper_cover.py")
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)

cover, info = g.build_iec(27.5, onepiece=True)
cover.apply_transform(np.linalg.inv(info["gripper_to_print"]))
fixed, moving = g.jaws(g.Q_CLAMP)
fixed.apply_scale(1000.0)
moving.apply_scale(1000.0)
tip = min(fixed.vertices[:, 2].min(), moving.vertices[:, 2].min())
fig, axs = plt.subplots(1, 3, figsize=(16, 6))
for ax, dz in zip(axs, (4.0, 15.0, 30.0)):
    z = tip + dz
    for m, c, fill in ((cover, "#2f6fb0", True), (fixed, "#c9c9cf", True), (moving, "#9a9aa3", True)):
        s = m.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
        if s is None:
            continue
        p, T = s.to_2D()
        for poly in p.polygons_full:
            P = np.c_[np.asarray(poly.exterior.coords), np.zeros(len(poly.exterior.coords)), np.ones(len(poly.exterior.coords))] @ T.T
            ax.fill(P[:, 0], P[:, 1], color=c, alpha=0.95, lw=0.5, ec="k")
            for hole in poly.interiors:
                H = np.c_[np.asarray(hole.coords), np.zeros(len(hole.coords)), np.ones(len(hole.coords))] @ T.T
                ax.fill(H[:, 0], H[:, 1], color="white", lw=0.5, ec="k")
    ax.set_aspect("equal")
    ax.set_xlim(-40, 40)
    ax.set_ylim(-45, 45)
    ax.set_title(f"턱 끝에서 {dz:.0f} mm 뒤를 가로로 자름", fontsize=12)
    ax.set_xlabel("← 고정 턱 쪽      mm      움직이는 턱 쪽 →")
    ax.grid(alpha=0.2)
fig.suptitle("파랑 = 덮개, 회색 = 턱. 칸막이(두 턱 사이 파랑)는 위아래 벽과 한 덩어리로 이어져 있다", fontsize=14)
fig.tight_layout()
fig.savefig("/home/cho/plugbot/cad/out/gripper_cover_septum.png", dpi=95)
print("ok")
