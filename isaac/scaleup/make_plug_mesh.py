"""실제 크기 장면의 UR10e 충전 플러그 메시(그리퍼 덮개 gripper_cover_iec와 같은 구조: 육상전원 표준 IEC 62613 플러그를 흉내).

tool0 좌표(m, z = 플랜지 바깥). 핀(Ø60, 끝 z = 0.47)과 몸통 반지름은 run_scaleup.py의 PIN_R, TIP_L, BODY_R 그대로라 제어와 판정은 바뀌지 않는다.
부품별로 나눠 저장한다(재질이 다르다): body(손잡이와 앞으로 좁아지는 코), ring(돌려 잠그는 결합 링과 돌기),
holes(앞면 핀 구멍 6개: 전력 3, 접지 1, 파일럿 2), pin(접점), gland(케이블 부싱, tool0 -x = 아래쪽으로 비스듬히).
실행: ~/cad_env/bin/python isaac/scaleup/make_plug_mesh.py   결과: isaac/scaleup/plug_iec.npz
특정 선박(팬스타 미라클호 등)의 실제 플러그를 본뜬 것이 아니다(공개 자료 없음).
"""

from pathlib import Path

import numpy as np
import trimesh

OUT = Path(__file__).resolve().parent / "plug_iec.npz"
ADAPTER_L, PIN_R, TIP_L = 0.020, 0.030, 0.470
FACE_Z = 0.220
GLAND_START = np.array([-0.060, 0.0, 0.090])
GLAND_DIR = np.array([-0.8, 0.0, -0.6])  # 아래(tool0 -x)와 뒤(-z)로
GLAND_LEN = 0.110


def revolve(profile, n=96):
    return trimesh.creation.revolve(np.asarray(profile, float), sections=n)


def cyl(r, h, center, axis=(0, 0, 1), n=48):
    c = trimesh.creation.cylinder(radius=r, height=h, sections=n)
    c.apply_transform(trimesh.geometry.align_vectors((0, 0, 1), axis))
    c.apply_translation(center)
    return c


def box(size, center, yaw=0.0):
    b = trimesh.creation.box(size)
    b.apply_transform(trimesh.transformations.rotation_matrix(yaw, (0, 0, 1)))
    b.apply_translation(center)
    return b


def ring_at(r, n):
    return [(r * np.cos(a), r * np.sin(a), a) for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]


body = [revolve([(0, ADAPTER_L), (0.072, ADAPTER_L), (0.072, 0.120), (0.080, 0.120), (0.080, 0.165),
                 (0.058, 0.212), (0.058, FACE_Z), (0, FACE_Z)])]
body += [box((0.006, 0.006, 0.075), (x, y, 0.070), a) for x, y, a in ring_at(0.072, 12)]  # 손잡이 돌기
ring = [revolve([(0.071, 0.120), (0.086, 0.120), (0.086, 0.165), (0.071, 0.165)])]
ring += [box((0.008, 0.008, 0.040), (x, y, 0.1425), a) for x, y, a in ring_at(0.086, 24)]  # 결합 링 돌기
holes = []
for ang, d in ((30, 0.016), (150, 0.016), (270, 0.016), (210, 0.020), (330, 0.010), (90, 0.010)):
    a = np.deg2rad(ang)
    holes.append(cyl(d / 2, 0.004, (0.044 * np.cos(a), 0.044 * np.sin(a), FACE_Z + 0.001)))
holes.append(box((0.010, 0.016, 0.050), (0.0, -0.058, FACE_Z - 0.022)))  # 방향 맞춤 홈(코 테두리)
pin = [revolve([(0, FACE_Z - 0.002), (PIN_R, FACE_Z - 0.002), (PIN_R, TIP_L - 0.006), (PIN_R - 0.006, TIP_L), (0, TIP_L)])]
g = GLAND_DIR / np.linalg.norm(GLAND_DIR)
gland = [cyl(0.026, GLAND_LEN, GLAND_START + g * GLAND_LEN / 2, g)]
gland += [cyl(0.030, 0.010, GLAND_START + g * s, g) for s in (0.045, 0.065, 0.085)]  # 부싱 주름

parts = {"body": body, "ring": ring, "holes": holes, "pin": pin, "gland": gland}
data = {}
for k, ms in parts.items():
    m = trimesh.util.concatenate(ms)
    data[k + "_v"], data[k + "_f"] = m.vertices.astype(np.float32), m.faces.astype(np.int32)
data["gland_end"] = (GLAND_START + g * GLAND_LEN).astype(np.float32)
data["gland_dir"] = g.astype(np.float32)
np.savez_compressed(OUT, **data)
print(OUT, {k: len(v) for k, v in data.items() if k.endswith("_f")})
