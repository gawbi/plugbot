"""그리퍼 덮개(육상전원 AMP 케이블 플러그 모양). 닫은 SO-101 팔로워 그리퍼 턱에 앞에서 끼우고, 앞 코에 원통(cad/extra/)을 꽂는다. 단위 mm.

- 안쪽: 공식 SO-101 메시(isaac/so101/assets)로 고정 턱 칸과 움직이는 턱 칸을 따로 판다(각 턱의 볼록 껍질 + 여유 CLEAR).
  두 칸 사이에 칸막이(쐐기, 턱 끝 쪽 3.2 mm)가 남는다. 그리퍼를 살짝 연 채 끼우고 닫으면 움직이는 턱이 칸막이를
  고정 턱 쪽으로 눌러 덮개를 문다(마찰만으로 끼우는 것이 아니다). 그리퍼를 열면 빠진다.
- 바깥: 선박 육상전원(AMP) 케이블 플러그를 흉내 낸 둥근 몸통(턱을 다 감싸는 원), 결합 링, 돌기, 앞으로 좁아지는 코.
  특정 선박(예: 팬스타)의 실제 플러그 규격을 확인해 본뜬 것이 아니다.
- 앞 코 끝에 원통 꽂는 홈(깊이 POCKET). 원통 끝은 턱 끝보다 약 45 mm 앞(손목 카메라 화면 아래쪽에 보이게).
- 출력 방향 그대로 저장: 뒤쪽 열린 면이 z = 0(출력판), 코가 +z. 서포트 없이 출력된다.
CadQuery가 아니라 메시(trimesh + manifold3d)로 만든다(턱 모양이 메시라서). 그래서 STEP은 없다.
실행: ~/cad_env/bin/python cad/make_gripper_cover.py   결과: cad/out/gripper_cover_{amp,iec,iec_onepiece}_{D22.5,D27.5}.stl, 미리보기 png
- iec_onepiece: iec 덮개 코에 원통 접점을 붙인 한 덩어리(원통을 따로 뽑아 붙이지 않는다). 끝 모따기 1.5 mm.
- amp: 둥근 몸통과 좁은 코(일반 육상전원 플러그 느낌). iec: IEC/IEEE 80005-3 저압 육상전원 플러그(IEC 62613-2) 구조를 흉내(build_iec).
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
CLEAR = 0.4  # 턱과 덮개 안쪽 여유(한쪽)
Q_CLAMP = -0.15  # 두 턱 사이 칸막이(쐐기)에 움직이는 턱이 닿는 각. 그리퍼를 닫으면 이 칸막이를 고정 턱 쪽으로 눌러 덮개를 문다
Q_SLACK = 0.06  # 움직이는 턱 칸을 이만큼 더 열린 각까지 넓힌다(끼울 때 여유, 턱 끝 약 5 mm)
WALL = 2.4  # 덮개 벽
NOSE = 30.0  # 턱 끝에서 코 앞면까지
POCKET = 10.0  # 원통 꽂는 홈 깊이
POCKET_CLEAR = 0.15  # 홈 반지름 여유(빡빡하면 사포, 헐거우면 순간접착제)
PIN_CHAMFER = 1.5  # 한 덩어리형 핀 끝 모따기(소켓에 들어가기 쉽게)
RIBS = 16  # 몸통 돌기 수
CYLINDERS = {"D22.5": (22.5, 27.5), "D27.5": (27.5, 22.5)}  # cad/extra/ 원통 (지름, 길이)


def _part(name, xyz, rpy):
    m = trimesh.load(ASSETS / name)
    m.apply_transform(translation_matrix(xyz) @ euler_matrix(*rpy, "sxyz"))
    return m


def jaws(q=Q_CLOSED):
    """두 턱(그리퍼 좌표, m). 움직이는 턱은 관절각 q. 값은 so101_plugbot_side.urdf 그대로."""
    fixed = _part("wrist_roll_follower_so101_v1.stl", (0, -0.000218214, 0.000949706), (-3.14159, 0, 0))
    moving = _part("moving_jaw_so101_v1.stl", (0, 0, 0.0189), (0, 0, 0))
    moving.apply_transform(translation_matrix((0.0202, 0.0188, -0.0234)) @ euler_matrix(1.5708, 0, 0, "sxyz")
                           @ euler_matrix(0, 0, q, "sxyz"))
    return fixed, moving


def inner():
    """안쪽 빈자리: 고정 턱 칸 + 움직이는 턱 칸(Q_CLAMP ~ Q_CLAMP + Q_SLACK). 두 칸 사이가 칸막이(턱 끝 3.2 mm → 뒤 15 mm 쐐기).
    돌려주는 값: 턱 끝 z, 뒤 z, 몸통 크기용 점(mm, 그리퍼 좌표), 빈자리 메시, 축 중심 cx, cy."""
    fixed, _ = jaws()
    f = fixed.vertices * 1000.0
    m = np.vstack([jaws(q)[1].vertices * 1000.0 for q in np.linspace(Q_CLAMP, Q_CLAMP + Q_SLACK, 4)])
    tip = min(f[:, 2].min(), m[:, 2].min())
    back = tip + COVER_L
    f, m = f[f[:, 2] <= back], m[m[:, 2] <= back]
    near = np.vstack([f, jaws()[1].vertices * 1000.0])
    near = near[near[:, 2] < tip + 4]
    cx, cy = (near[:, 0].min() + near[:, 0].max()) / 2, (near[:, 1].min() + near[:, 1].max()) / 2
    holes = []
    for pts in (f, m):
        g = _grow(pts, CLEAR)
        g = g[g[:, 2] <= back]
        holes.append(trimesh.convex.convex_hull(np.vstack([g, g[g[:, 2] > back - 1] + [0, 0, 5]])))  # 뒤로 열림
    cavity = trimesh.boolean.union(holes, engine="manifold")
    return tip, back, np.vstack([f, m]), cavity, cx, cy


def _grow(pts, d):
    """점 묶음을 d만큼 부풀린다(정육면체 꼭짓점 방향 + 축 방향으로 복사, 볼록 껍질용)."""
    dirs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) / np.sqrt(3)
    dirs = np.vstack([dirs, np.eye(3), -np.eye(3)])
    return (pts[:, None, :] + d * dirs[None]).reshape(-1, 3)


def _circle(r, z, cx, cy, n=96):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[cx + r * np.cos(a), cy + r * np.sin(a), np.full(n, z)]


def build(d_cyl):
    tip, back, jaw, cavity, cx, cy = inner()

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
    return _finish(body, cavity, d_cyl, cx, cy, front)


def build_iec(d_cyl, onepiece=False):
    """IEC/IEEE 80005-3 저압 육상전원 플러그(IEC 62613-2 방식) 구조를 흉내 낸 덮개.
    넓은 앞면(접점 판)에 전력 핀 3, 접지 1, 파일럿 2 구멍, 코 테두리의 방향 맞춤 홈, 돌려 잠그는 결합 링,
    카메라 반대쪽(-y)으로 비스듬히 나오는 케이블 부싱. 치수는 그리퍼에 맞춘 것이고 규격 치수가 아니다."""
    tip, back, jaw, cavity, cx, cy = inner()

    front = tip - NOSE
    grown = _grow(jaw, CLEAR + WALL)
    r_body = np.hypot(grown[:, 0] - cx, grown[:, 1] - cy).max()
    r_face = d_cyl / 2 + 9.0  # 접점 판(손목 카메라가 원통 끝을 보게 몸통보다 작게)
    body = trimesh.convex.convex_hull(np.vstack([
        _circle(r_body, back, cx, cy), _circle(r_body, tip + 6, cx, cy),
        _circle(r_face + 1.5, front + 12, cx, cy), _circle(r_face, front + 10, cx, cy), _circle(r_face, front, cx, cy)]))
    parts = [body]
    ring = trimesh.creation.cylinder(radius=r_body + 2.2, height=12.0, sections=128)  # 결합 링
    ring.apply_translation((cx, cy, tip + 1.0))
    parts.append(ring)
    for k in range(24):  # 결합 링 손잡이 돌기(로렛)
        a = 2 * np.pi * k / 24
        b = trimesh.creation.box((2.0, 2.0, 10.0))
        b.apply_transform(trimesh.transformations.rotation_matrix(a, (0, 0, 1)))
        b.apply_translation((cx + (r_body + 2.2) * np.cos(a), cy + (r_body + 2.2) * np.sin(a), tip + 1.0))
        parts.append(b)
    # 케이블 부싱: 카메라 반대쪽(-y)으로, 뒤쪽(+z)으로 기울여(출력 때 서포트 없이 37°)
    d = np.array([0.0, -0.6, 0.8])
    gl = trimesh.creation.cylinder(radius=8.0, height=34.0, sections=64)
    gl.apply_transform(trimesh.geometry.align_vectors((0, 0, 1), d))
    gl.apply_translation(np.array([cx, cy - r_body + 4.0, tip + 14.0]) + d * 14.0)
    parts.append(gl)
    for k in range(3):  # 부싱 주름
        g = trimesh.creation.cylinder(radius=9.2, height=2.0, sections=64)
        g.apply_transform(trimesh.geometry.align_vectors((0, 0, 1), d))
        g.apply_translation(np.array([cx, cy - r_body + 4.0, tip + 14.0]) + d * (18.0 + 6.0 * k))
        parts.append(g)
    body = trimesh.boolean.union(parts, engine="manifold")

    cuts = []
    pin_r = d_cyl / 2 + 4.6
    for ang, dia in ((30, 3.2), (150, 3.2), (270, 3.2), (210, 4.2), (330, 2.0), (90, 2.0)):  # L1 L2 L3, PE, 파일럿 2
        h = trimesh.creation.cylinder(radius=dia / 2, height=6.0, sections=32)
        a = np.deg2rad(ang)
        h.apply_translation((cx + pin_r * np.cos(a), cy + pin_r * np.sin(a), front))
        cuts.append(h)
    key = trimesh.creation.box((3.0, 4.0, 16.0))  # 방향 맞춤 홈(코 테두리, 아래쪽)
    key.apply_translation((cx, cy - r_face, front + 4.0))
    cuts.append(key)
    g = trimesh.creation.annulus(r_min=r_body + 1.4, r_max=r_body + 8, height=1.0)  # 결합 링 가운데 줄
    g.apply_translation((cx, cy, tip + 1.0))
    cuts.append(g)
    tail = trimesh.creation.box((400, 400, 100))  # 뒤 면(출력판)보다 뒤로 나간 부분 자르기
    tail.apply_translation((cx, cy, back + 50))
    cuts.append(tail)
    body = trimesh.boolean.difference([body] + cuts, engine="manifold")
    return _finish(body, cavity, d_cyl, cx, cy, front, onepiece)


def _finish(body, cavity, d_cyl, cx, cy, front, onepiece=False):
    h_cyl = CYLINDERS_LEN[d_cyl]
    if onepiece:  # 원통(지름 d_cyl, 길이 h_cyl)을 코에 붙여 한 덩어리로. 끝 모따기 PIN_CHAMFER
        r, L, c = d_cyl / 2, h_cyl + 2.0, PIN_CHAMFER
        pin = trimesh.creation.revolve([[0, 0], [r, 0], [r, L - c], [r - c, L], [0, L]], sections=128)
        pin.apply_transform(trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0)))  # 코 앞(-z)으로
        pin.apply_translation((cx, cy, front + 2.0))
        body = trimesh.boolean.union([body, pin], engine="manifold")
        body = trimesh.boolean.difference([body, cavity], engine="manifold")
        tip_ahead = NOSE + h_cyl
    else:
        pocket = trimesh.creation.cylinder(radius=d_cyl / 2 + POCKET_CLEAR, height=POCKET * 2, sections=128)
        pocket.apply_translation((cx, cy, front))
        body = trimesh.boolean.difference([body, cavity, pocket], engine="manifold")
        tip_ahead = NOSE - POCKET + h_cyl

    # 출력 방향: 뒤쪽 열린 면을 z = 0에, 코를 +z로(그리퍼 좌표의 -z를 +z로 뒤집음)
    T = trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0))
    body.apply_transform(T)
    T = translation_matrix(-body.bounds[0]) @ T
    body.apply_translation(-body.bounds[0])
    info = {"tip_to_front": NOSE, "cyl_tip_ahead_of_jaw": tip_ahead, "axis_xy": (cx, cy), "gripper_to_print": T}
    return body, info


CYLINDERS_LEN = {d: h for d, h in CYLINDERS.values()}


def _simplify(m, tol=0.01):
    """불린 연산이 남긴 넓이 0에 가까운 삼각형을 manifold3d로 접어 없앤다(STL float32로 저장해도 닫힌 하나의 몸체)."""
    import manifold3d

    mm = manifold3d.Manifold(manifold3d.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                                             tri_verts=np.asarray(m.faces, np.uint32))).simplify(tol)
    out = mm.to_mesh()
    return trimesh.Trimesh(np.asarray(out.vert_properties)[:, :3], np.asarray(out.tri_verts), process=True)


def preview(meshes, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(5 * len(meshes), 5))
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
    for style, fn in (("amp", build), ("iec", build_iec), ("iec_onepiece", lambda d: build_iec(d, onepiece=True))):
        for name, (d, h) in CYLINDERS.items():
            m, info = fn(d)
            m = _simplify(m)
            assert m.is_watertight, (style, name)
            m.export(OUT / f"gripper_cover_{style}_{name}.stl")
            made[f"{style} {name}"] = m
            print(f"gripper_cover_{style}_{name}.stl  크기 {np.round(m.extents, 1)} mm, 부피 {m.volume / 1000:.1f} cm3, "
                  f"원통 끝이 턱 끝보다 {info['cyl_tip_ahead_of_jaw']:.1f} mm 앞")
    preview(made, OUT / "gripper_cover_preview.png")
