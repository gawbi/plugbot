"""그리퍼 덮개(육상전원 AMP 케이블 플러그 모양). 닫은 SO-101 팔로워 그리퍼 턱에 앞에서 끼우고, 앞 코에 원통(cad/extra/)을 꽂는다. 단위 mm.

- 안쪽: 공식 SO-101 메시(isaac/so101/assets)에서 고정 턱과 움직이는 턱(닫힘 q = -0.20 rad, 두 턱이 닿는 각)을
  그리퍼 좌표로 놓고, 턱 끝에서 COVER_L 길이만큼의 볼록 껍질 + 여유 CLEAR.
- 바깥: 선박 육상전원(AMP) 케이블 플러그를 흉내 낸 둥근 몸통(턱을 다 감싸는 원), 결합 링, 돌기, 앞으로 좁아지는 코.
  특정 선박(예: 팬스타)의 실제 플러그 규격을 확인해 본뜬 것이 아니다.
- 앞 코 끝에 원통 꽂는 홈(깊이 POCKET). 원통 끝은 턱 끝보다 약 45 mm 앞(손목 카메라 화면 아래쪽에 보이게).
- 출력 방향 그대로 저장: 뒤쪽 열린 면이 z = 0(출력판), 코가 +z. 서포트 없이 출력된다.
CadQuery가 아니라 메시(trimesh + manifold3d)로 만든다(턱 모양이 메시라서). 그래서 STEP은 없다.
실행: ~/cad_env/bin/python cad/make_gripper_cover.py   결과: cad/out/gripper_cover_D22.5.stl, gripper_cover_D27.5.stl, 미리보기 png
"""

from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, translation_matrix

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "isaac" / "so101" / "assets"
OUT = ROOT / "cad" / "out"

Q_CLOSED = -0.20  # 두 턱이 닿는 그리퍼 관절각(rad, 메시 단면으로 찾음)
COVER_L = 40.0  # 턱 끝에서 뒤로 덮는 길이
CLEAR = 0.5  # 턱과 덮개 안쪽 여유(한쪽)
WALL = 2.4  # 덮개 벽
NOSE = 30.0  # 턱 끝에서 코 앞면까지
POCKET = 10.0  # 원통 꽂는 홈 깊이
POCKET_CLEAR = 0.15  # 홈 반지름 여유(빡빡하면 사포, 헐거우면 순간접착제)
RIBS = 16  # 몸통 돌기 수
CYLINDERS = {"D22.5": (22.5, 27.5), "D27.5": (27.5, 22.5)}  # cad/extra/ 원통 (지름, 길이)


def _part(name, xyz, rpy):
    m = trimesh.load(ASSETS / name)
    m.apply_transform(translation_matrix(xyz) @ euler_matrix(*rpy, "sxyz"))
    return m


def jaws():
    """닫힌 두 턱(그리퍼 좌표, m). 값은 so101_plugbot_side.urdf 그대로."""
    fixed = _part("wrist_roll_follower_so101_v1.stl", (0, -0.000218214, 0.000949706), (-3.14159, 0, 0))
    moving = _part("moving_jaw_so101_v1.stl", (0, 0, 0.0189), (0, 0, 0))
    moving.apply_transform(translation_matrix((0.0202, 0.0188, -0.0234)) @ euler_matrix(1.5708, 0, 0, "sxyz")
                           @ euler_matrix(0, 0, Q_CLOSED, "sxyz"))
    return fixed, moving


def _grow(pts, d):
    """점 묶음을 d만큼 부풀린다(정육면체 꼭짓점 방향 + 축 방향으로 복사, 볼록 껍질용)."""
    dirs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) / np.sqrt(3)
    dirs = np.vstack([dirs, np.eye(3), -np.eye(3)])
    return (pts[:, None, :] + d * dirs[None]).reshape(-1, 3)


def _circle(r, z, cx, cy, n=96):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[cx + r * np.cos(a), cy + r * np.sin(a), np.full(n, z)]


def build(d_cyl):
    fixed, moving = jaws()
    pts = np.vstack([fixed.vertices, moving.vertices]) * 1000.0  # mm, 그리퍼 좌표(-z가 턱 끝 방향)
    tip = pts[:, 2].min()
    back = tip + COVER_L
    jaw = pts[pts[:, 2] <= back]
    near = jaw[jaw[:, 2] < tip + 4]
    cx, cy = (near[:, 0].min() + near[:, 0].max()) / 2, (near[:, 1].min() + near[:, 1].max()) / 2

    inner = _grow(jaw, CLEAR)
    inner = inner[inner[:, 2] <= back]
    cavity = trimesh.convex.convex_hull(np.vstack([inner, inner[inner[:, 2] > back - 1] + [0, 0, 5]]))

    # 육상전원(AMP) 케이블 플러그 모양: 턱을 감싸는 둥근 몸통 → 결합 링 → 앞으로 좁아지는 코 → 접점(원통)
    r_nose = d_cyl / 2 + POCKET_CLEAR + WALL
    front = tip - NOSE
    grown = _grow(jaw, CLEAR + WALL)
    r_body = np.hypot(grown[:, 0] - cx, grown[:, 1] - cy).max()
    outer_pts = np.vstack([_circle(r_body, back, cx, cy), _circle(r_body, tip - 2, cx, cy),
                           _circle(r_nose + 3, front + 8, cx, cy), _circle(r_nose, front, cx, cy)])
    body = trimesh.convex.convex_hull(outer_pts)

    ring = trimesh.creation.cylinder(radius=r_body + 1.8, height=8.0, sections=128)  # 결합 링(돌려 잠그는 너트 자리)
    ring.apply_translation((cx, cy, tip - 1.0))
    parts = [body, ring]
    for k in range(RIBS):  # 손잡이 쪽 미끄럼 방지 돌기
        a = 2 * np.pi * k / RIBS
        rib = trimesh.creation.box((2.4, 2.4, COVER_L - 14))
        rib.apply_transform(trimesh.transformations.rotation_matrix(a, (0, 0, 1)))
        rib.apply_translation((cx + r_body * np.cos(a), cy + r_body * np.sin(a), back - (COVER_L - 14) / 2 - 1))
        parts.append(rib)
    body = trimesh.boolean.union(parts, engine="manifold")
    for zg in (tip - 4.0, tip + 2.0):  # 결합 링 홈 두 줄
        g = trimesh.creation.annulus(r_min=r_body + 1.0, r_max=r_body + 6, height=1.0)
        g.apply_translation((cx, cy, zg))
        body = trimesh.boolean.difference([body, g], engine="manifold")
    face = trimesh.creation.annulus(r_min=d_cyl / 2 + 1.6, r_max=d_cyl / 2 + 2.4, height=1.6)  # 앞면 접점 둘레 홈
    face.apply_translation((cx, cy, front))
    body = trimesh.boolean.difference([body, face], engine="manifold")

    pocket = trimesh.creation.cylinder(radius=d_cyl / 2 + POCKET_CLEAR, height=POCKET * 2, sections=128)
    pocket.apply_translation((cx, cy, front))
    body = trimesh.boolean.difference([body, cavity, pocket], engine="manifold")

    # 출력 방향: 뒤쪽 열린 면을 z = 0에, 코를 +z로(그리퍼 좌표의 -z를 +z로 뒤집음)
    body.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))
    body.apply_translation(-np.r_[body.bounds[0][:2], body.bounds[0][2]])
    info = {"tip_to_front": NOSE, "cyl_tip_ahead_of_jaw": NOSE - POCKET + CYLINDERS_LEN[d_cyl], "axis_xy": (cx, cy)}
    return body, info


CYLINDERS_LEN = {d: h for d, h in CYLINDERS.values()}


def preview(meshes, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(6 * len(meshes), 6))
    for i, (name, m) in enumerate(meshes.items()):
        ax = fig.add_subplot(1, len(meshes), i + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(m.triangles, facecolor="#7c5cff", edgecolor="#3b2a99", linewidths=0.05, alpha=1.0))
        b = m.bounds
        c, s = b.mean(0), (b[1] - b[0]).max() / 2
        ax.set_xlim(c[0] - s, c[0] + s), ax.set_ylim(c[1] - s, c[1] + s), ax.set_zlim(c[2] - s, c[2] + s)
        ax.view_init(elev=18, azim=-60)
        ax.set_title(f"{name}  {b[1][0] - b[0][0]:.0f} x {b[1][1] - b[0][1]:.0f} x {b[1][2] - b[0][2]:.0f} mm")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    made = {}
    for name, (d, h) in CYLINDERS.items():
        m, info = build(d)
        assert m.is_watertight, name
        m.export(OUT / f"gripper_cover_{name}.stl")
        made[name] = m
        print(f"gripper_cover_{name}.stl  크기 {np.round(m.extents, 1)} mm, 부피 {m.volume / 1000:.1f} cm3, "
              f"원통 끝이 턱 끝보다 {info['cyl_tip_ahead_of_jaw']:.1f} mm 앞")
    preview(made, OUT / "gripper_cover_preview.png")
