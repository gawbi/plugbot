"""실물 키트의 앞 카메라(D435) 자리 그림: 위에서 본 배치와 옆에서 본 배치, 65 cm 암막 부스 안.

치수는 real/README.md '놓는 자리'와 plugbot/scene.py CAMERAS_SIDE(시뮬레이션과 같은 자리)를 그대로 쓴다.
좌표: 로봇 받침 중심 기준, x = 정면, y = 왼쪽, 높이는 책상 위(로봇 받침 100 mm).
실행: ~/plugbot/.venv/bin/python tools/draw_camera_layout.py   결과: outputs/camera_layout.png
"""

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle, Wedge  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "outputs" / "camera_layout.png"
for f in ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/System/Library/Fonts/AppleSDGothicNeo.ttc"]:
    if Path(f).exists():
        font_manager.fontManager.addfont(f)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name()
        break

PED = 0.10  # 로봇 받침 높이
CAM = np.array([0.06, -0.22, 0.40])  # 앞 카메라(책상 위 높이)
LOOK = np.array([0.26, 0.10, 0.13])  # 두 소켓 사이
HFOV, VFOV = 69.0, 42.0  # RealSense D435 컬러
ENT = np.array([0.30, 0.0, PED + 0.03])  # 선박 소켓 입구
SHOULDER = np.array([0.039, 0.0])
CAR_ANG = np.deg2rad(50.0)
BOOTH = (-0.17, -0.33, 0.65)  # 부스 안쪽 시작 x, 시작 y, 한 변

C_ROBOT, C_SHIP, C_CAR, C_CAM, C_BOOTH = "#c8781e", "#2b6cb0", "#6b7280", "#d6336c", "#111827"


def rot(v, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def P(x, y):  # 위에서 본 그림: 가로 = 오른쪽(-y), 세로 = 정면(x)
    return (-y, x)


fig, (ax, bx) = plt.subplots(1, 2, figsize=(15, 8.4), gridspec_kw={"width_ratios": [1.05, 1]})

# ---------- 위에서 본 배치 ----------
x0, y0, L = BOOTH
ax.add_patch(Rectangle(P(x0, y0 + L), L, L, fill=False, lw=2.5, ec=C_BOOTH))
ax.text(*P(x0 + L - 0.012, y0 + L - 0.01), "암막 부스 안쪽 650 × 650 mm", va="top", fontsize=10, color=C_BOOTH)
ax.text(*P(x0 + L + 0.012, 0.0), "관람, 출입 쪽(예)", ha="center", va="bottom", fontsize=9, color="#555")
# 로봇
ax.add_patch(Rectangle(P(-0.06, 0.06), 0.12, 0.12, fc="#e5e7eb", ec="#9ca3af"))
ax.add_patch(Circle(P(0, 0), 0.045, fc=C_ROBOT, ec="k", lw=0.8))
ax.add_patch(Circle(P(*SHOULDER), 0.44, fill=False, ls=":", ec=C_ROBOT, lw=1))
ax.text(*P(-0.075, 0.0), "SO-101\n(받침 100 mm)", ha="center", va="top", fontsize=9)
ax.text(*P(0.36, -0.42), "팔 도달\n약 44 cm", fontsize=8, color=C_ROBOT)
# 흔들림 갑판과 선박 소켓
ax.add_patch(Rectangle(P(0.331 - 0.085, 0.085), 0.17, 0.17, fc="#dbeafe", ec=C_SHIP, lw=1))
ax.add_patch(Rectangle(P(0.346, 0.085), 0.17, 0.03, fc=C_SHIP, ec="k", lw=0.6))
ax.add_patch(Rectangle(P(0.300, 0.031), 0.062, 0.046, fc="#f59e0b", ec="k", lw=0.6))
ax.text(*P(0.30, -0.10), "흔들림 갑판 +\n세운 판 소켓\n입구: 정면 300 mm", ha="left", fontsize=9, color=C_SHIP)
# 자동차 충전구(어깨축 둘레 왼쪽 50°)
c_ent = SHOULDER + rot(ENT[:2] - SHOULDER, CAR_ANG)
n = rot(np.array([1.0, 0.0]), CAR_ANG)
t = np.array([-n[1], n[0]])
pts = [c_ent + n * 0.046 + t * 0.085, c_ent + n * 0.046 - t * 0.085, c_ent + n * 0.076 - t * 0.085, c_ent + n * 0.076 + t * 0.085]
ax.add_patch(Polygon([P(*p) for p in pts], fc=C_CAR, ec="k", lw=0.6))
ax.text(*P(0.12, 0.31), "자동차 충전구\n(어깨축 기준\n왼쪽 50°)", ha="left", fontsize=9, color="#374151")
# 앞 카메라와 시야
d = LOOK[:2] - CAM[:2]
yaw = np.degrees(np.arctan2(d[0], -d[1]))  # 위 그림의 가로(-y) 기준 각도
ax.add_patch(Wedge(P(CAM[0], CAM[1]), 0.62, yaw - HFOV / 2, yaw + HFOV / 2, fc=C_CAM, alpha=0.10, ec=C_CAM))
ax.plot(*zip(P(*CAM[:2]), P(*LOOK[:2])), color=C_CAM, lw=1.2, ls="--")
ax.add_patch(Rectangle((-CAM[1] - 0.02, CAM[0] - 0.012), 0.04, 0.024, fc=C_CAM, ec="k"))
ax.plot(*P(*LOOK[:2]), marker="x", color=C_CAM, ms=10, mew=2)
ax.text(0.45, CAM[0] - 0.01, "앞 카메라 D435\n정면 60 mm\n오른쪽 220 mm\n책상 위 400 mm", fontsize=10, color=C_CAM, fontweight="bold", va="center")
ax.text(*P(LOOK[0] - 0.035, LOOK[1] - 0.005), "바라보는 점\n(두 소켓 사이)", fontsize=8, color=C_CAM, ha="left", va="top")
# 치수선
ax.annotate("", xy=P(0.06, 0.0), xytext=P(0.06, -0.22), arrowprops=dict(arrowstyle="<->", color="#555"))
ax.text(*P(0.075, -0.11), "220", fontsize=8, color="#555", ha="center")
ax.annotate("", xy=P(0.0, -0.30), xytext=P(0.06, -0.30), arrowprops=dict(arrowstyle="<->", color="#555"))
ax.text(*P(0.03, -0.315), "60", fontsize=8, color="#555", ha="left", va="center")
ax.set_xlim(-0.36, 0.64)
ax.set_ylim(-0.22, 0.52)
ax.set_aspect("equal")
ax.set_xlabel("← 왼쪽     (m)     오른쪽 →")
ax.set_ylabel("정면 방향 (m)")
ax.set_title("위에서 본 배치", fontsize=14, fontweight="bold")
ax.grid(alpha=0.15)

# ---------- 옆에서 본 배치(카메라에서 바라보는 점을 잇는 면) ----------
dist = np.linalg.norm(LOOK[:2] - CAM[:2])
pitch = np.degrees(np.arctan2(LOOK[2] - CAM[2], dist))
bx.add_patch(Rectangle((-0.05, 0), 0.75, 0.65, fill=False, lw=2.5, ec=C_BOOTH))
bx.text(-0.04, 0.665, "부스 높이 650 mm", ha="left", va="bottom", fontsize=10, color=C_BOOTH)
bx.axhline(0, color="k", lw=1.5)
u = (LOOK[:2] - CAM[:2]) / np.linalg.norm(LOOK[:2] - CAM[:2])
rb = float(np.dot(-CAM[:2], u))  # 로봇 받침 중심의 시선 방향 투영 거리
bx.add_patch(Rectangle((rb - 0.06, 0.0), 0.12, 0.10, fc="#e5e7eb", ec="#9ca3af"))
bx.add_patch(Rectangle((rb - 0.03, 0.10), 0.06, 0.05, fc=C_ROBOT, ec="k", lw=0.8))
bx.text(rb, 0.17, "SO-101(받침 위)", ha="center", fontsize=9)
bx.plot([0.0, 0.0], [0.0, CAM[2]], color="#6b7280", lw=3)
bx.add_patch(Rectangle((-0.02, CAM[2] - 0.012), 0.04, 0.024, fc=C_CAM, ec="k"))
bx.add_patch(Wedge((0.0, CAM[2]), 0.62, pitch - VFOV / 2, pitch + VFOV / 2, fc=C_CAM, alpha=0.10, ec=C_CAM))
bx.plot([0.0, dist], [CAM[2], LOOK[2]], color=C_CAM, ls="--", lw=1.2)
bx.plot(dist, LOOK[2], marker="x", color=C_CAM, ms=10, mew=2)
bx.text(0.02, CAM[2] + 0.03, f"D435 높이 400 mm\n아래로 약 {abs(pitch):.0f}° 숙임", fontsize=10, color=C_CAM, fontweight="bold")
bx.text(dist + 0.01, LOOK[2] + 0.02, "소켓 높이\n(책상 위 130 mm)", fontsize=9, color=C_SHIP)
bx.text(0.0, -0.035, "셀카봉, 삼각대\n(클램프로 고정)", ha="center", va="top", fontsize=9, color="#555")
for xl in (0.12, 0.58):  # 위쪽 LED
    bx.add_patch(Rectangle((xl - 0.05, 0.63), 0.10, 0.015, fc="#fde68a", ec="#ca8a04"))
bx.text(0.35, 0.60, "LED는 위쪽 좌우, 카메라 렌즈로 직접 빛이 들어가지 않게", ha="center", fontsize=9, color="#a16207")
bx.set_xlim(-0.1, 0.72)
bx.set_ylim(-0.1, 0.7)
bx.set_aspect("equal")
bx.set_xlabel("카메라에서 바라보는 점까지 수평 거리 (m)")
bx.set_ylabel("책상 위 높이 (m)")
bx.set_title("옆에서 본 배치(카메라 시선 방향)", fontsize=14, fontweight="bold")
bx.grid(alpha=0.15)

fig.suptitle("PlugBot 실물 키트: 앞 카메라(D435) 자리", fontsize=17, fontweight="bold", y=0.99)
fig.text(0.5, 0.925, "치수는 real/README.md와 시뮬레이션(plugbot/scene.py CAMERAS_SIDE)과 같다. 받침 바로 옆에 두면 팔이 마커를 가려 추적 오차 3.8 mm, "
         "이 자리는 0.6 mm 이하(모의 영상). 보정 뒤에는 카메라를 움직이지 않는다.", ha="center", fontsize=9, color="#444")
fig.tight_layout(rect=(0, 0.0, 1, 0.91))
fig.savefig(OUT, dpi=130)
print(OUT)
