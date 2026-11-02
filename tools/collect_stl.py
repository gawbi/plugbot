"""출력용 STL을 한 폴더와 zip으로 모은다(윈도우, 맥, 리눅스 공통. 파이썬 표준 라이브러리만 쓴다).

파일과 폴더 이름은 영문만 쓴다(맥에서 만든 zip의 한글 이름이 윈도우에서 깨지는 문제를 피하려고).
실행(저장소 폴더에서): python tools/collect_stl.py
결과: stl_bundle/ 폴더와 stl_bundle.zip (저장소 폴더 안. .gitignore 대상)
"""

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "stl_bundle"
# (원본, 묶음 안 경로, 수량, 메모)
PARTS = [
    ("cad/out/socket.stl", "1_final/01_socket_x2.stl", 2, "플랜지를 바닥에. 입구 지름 50 mm 깔때기. 채움 40%"),
    ("cad/out/plug_L.stl", "1_final/02_plug_L_x2.stl", 2, "핀 없는 넓은 면을 바닥에(핀이 위로). 1개는 여분"),
    ("cad/out/side_mount.stl", "1_final/03_side_mount_x1.stl", 1, "발판을 바닥에. 갑판판 소켓 구멍 4개에 M4 볼트"),
    ("cad/out/car_port_side.stl", "1_final/04_car_port_side_x1.stl", 1, "받침을 바닥에. 자동차 충전구"),
    ("cad/out/deck_base.stl", "1_final/05_deck_base_x1.stl", 1, "그대로. 출력판 220 mm 이상 필요"),
    ("cad/out/deck_plate.stl", "1_final/06_deck_plate_x1.stl", 1, "윗면을 바닥에(러그가 위로)"),
    ("isaac/so101/assets/wrist_camera_mount_so101_v1.stl", "1_final/07_wrist_camera_mount_x1.stl", 1,
     "SO-101 공식 손목 카메라 거치대. 카메라 기판(약 32 x 32 mm)에 맞는지 먼저 확인"),
    ("cad/extra/cylinder_D27.5_H22.5mm.stl", "2_cylinder/cylinder_D27.5_H22.5mm.stl", 1, "지름 27.5 x 높이 22.5 mm"),
    ("cad/extra/cylinder_D22.5_H27.5mm.stl", "2_cylinder/cylinder_D22.5_H27.5mm.stl", 1, "지름 22.5 x 높이 27.5 mm"),
    ("cad/out/plug.stl", "3_prelim/plug_straight.stl", 0, "예선용 일자 플러그(본선 불필요)"),
    ("cad/out/car_port.stl", "3_prelim/car_port_45deg.stl", 0, "예선용 45도 자동차 충전구(본선 불필요)"),
]
STEP = ["socket", "plug_L", "side_mount", "car_port_side", "deck_base", "deck_plate", "plug", "car_port"]


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    lines = ["PlugBot STL bundle (단위 mm). 권장: PLA, 노즐 0.4, 층 0.2 mm, 벽 3줄, 채움 20~30%, 서포트 없음", ""]
    for src, dst, qty, memo in PARTS:
        s, d = ROOT / src, OUT / dst
        d.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(s, d)
        lines.append(f"{dst:45s} 수량 {qty}  {memo}")
    for name in STEP:
        d = OUT / "4_step" / f"{name}.step"
        d.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "cad/out" / f"{name}.step", d)
    for p in (ROOT / "cad/extra").glob("*.step"):
        shutil.copy2(p, OUT / "4_step" / p.name)
    lines += ["", "4_step/: 치수 수정용 STEP 원본. 철물: M5 x 40 볼트 2(힌지), M4 x 16 볼트와 너트 12쌍, 자석 Ø6 x 3 mm",
              "자세한 안내: cad/PRINT_GUIDE.md"]
    (OUT / "README.txt").write_text("\n".join(lines) + "\n", encoding="utf-8-sig")  # 메모장에서도 한글이 보이게 BOM
    zip_path = shutil.make_archive(str(OUT), "zip", root_dir=ROOT, base_dir=OUT.name)
    n = sum(1 for _ in OUT.rglob("*.stl"))
    print(f"STL {n}개 → {OUT}")
    print(f"zip → {zip_path}")


if __name__ == "__main__":
    main()
