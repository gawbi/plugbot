"""포토박스 트윈용 메시: 실제 출력한 그리퍼 덮개 STL을 그리퍼 좌표(m)로 되돌리고, 흰 소켓(구멍 있는 원통)을 만든다.

덮개: cad/out/gripper_cover_iec_onepiece_D27.5.stl(출력 방향 저장). cad/make_gripper_cover.py가 저장 때 쓴 변환(gripper_to_print)을
거꾸로 적용한다. 접점 핀(지름 27.5 mm)이 소켓 구멍에 들어간다.
소켓: 흰 원통(지름 60 mm, 깊이 30 mm)에 핀 구멍(지름 29 mm, 깊이 25 mm), 뒤에 검은 받침 원판. 로컬 +x가 판 쪽, 앞면이 x = 0.
실행: ~/cad_env/bin/python isaac/twin/make_twin_meshes.py   결과: isaac/twin/twin_meshes.npz
"""

import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "cad"))
import make_gripper_cover as G  # noqa: E402

body, info = G.build_iec(27.5, onepiece=True)
T = np.asarray(info["gripper_to_print"])
stl = trimesh.load(ROOT / "cad/out/gripper_cover_iec_onepiece_D27.5.stl")
assert np.allclose(np.sort(stl.vertices[:, 2])[-5:], np.sort(body.vertices[:, 2])[-5:], atol=0.05), "STL이 지금 스크립트와 다르다"
cov = stl.copy()
cov.apply_transform(np.linalg.inv(T))
cov.apply_scale(0.001)  # mm → m, 그리퍼 좌표
cx, cy = info["axis_xy"]
tip = np.array([cx / 1000, cy / 1000, cov.vertices[:, 2].min()])
print("덮개 그리퍼 좌표 범위(m)", cov.bounds.round(4).tolist(), "핀 끝", tip.round(4).tolist())

# 흰 소켓: 축을 x로
outer = trimesh.creation.cylinder(radius=0.030, height=0.030, sections=96)
hole = trimesh.creation.cylinder(radius=0.0145, height=0.050, sections=96)
hole.apply_translation((0, 0, -0.015))  # 앞(-z)에서 25 mm 깊이
sock = trimesh.boolean.difference([outer, hole], engine="manifold")
R = trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0))  # 로컬 z → x
sock.apply_transform(R)
sock.apply_translation((0.015, 0, 0))  # 앞면을 x = 0에
base = trimesh.creation.cylinder(radius=0.033, height=0.008, sections=96)
base.apply_transform(R)
base.apply_translation((0.034, 0, 0))
np.savez(HERE := ROOT / "isaac/twin/twin_meshes.npz", cover_v=cov.vertices.astype(np.float32), cover_f=cov.faces.astype(np.int32),
         pin_tip=tip.astype(np.float32), sock_v=sock.vertices.astype(np.float32), sock_f=sock.faces.astype(np.int32),
         base_v=base.vertices.astype(np.float32), base_f=base.faces.astype(np.int32))
print("저장", HERE, "덮개 면", len(cov.faces), "소켓 면", len(sock.faces))
