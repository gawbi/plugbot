"""축소 시연 키트 CAD (CadQuery). 단위 mm.

부품(cad/out/에 STEP·STL):
- plug: 그리퍼가 쥐는 플러그. 핀 Ø12, 끝 모따기 2 mm, 그리퍼 턱 홈, 핀 끝 자석 포켓(Ø6×3)
- socket: 깔때기 소켓. 입구 Ø36 → 보어 Ø15(반경 여유 1.5 mm), 45° 원뿔, 바닥 자석 포켓(래치 역할),
  ArUco 마커 탭(25 mm 마커용 30×30 홈), M4 장착 구멍 4개(피치 54)
- deck: 흔들림 갑판. 베이스 + 힌지 블록(M5 축) + 갑판판(소켓 장착) + 레버(손·서보 링크용 구멍 5개)
- car_port: 자동차 충전구 블록. 45° 경사면에 같은 소켓 장착
- assembly.step: 갑판 위 소켓, 꽂힌 플러그, 자동차 충전구 블록 조립본
충돌 계산용 소켓 조각(socket_wall_XX.stl, socket_cap.stl)은 시뮬레이션이 쓴다.
STEP은 솔리드웍스에서 열리고, STL은 3D 프린트용이다.
"""

from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"

# 플러그
PIN_R, PIN_L, PIN_CHAMFER = 6.0, 40.0, 2.0
BODY = (16.0, 18.0, 30.0)
MAGNET_D, MAGNET_H = 6.2, 3.2  # Ø6×3 네오디뮴 자석 + 끼움 여유

# 소켓: 포착 반경 = 원뿔 입구 반지름 - 핀 반지름 = 12 mm
BORE_R, BORE_D = 7.5, 25.0
CONE_TOP_R = 18.0
CONE_H = CONE_TOP_R - BORE_R
OUTER_R = 24.0
CAP_T = 4.0
SOCKET_H = CAP_T + BORE_D + CONE_H
FLANGE = (70.0, 70.0, 4.0)
HOLE_PITCH, HOLE_D = 54.0, 4.5
MARKER_TAB, MARKER_POCKET = 40.0, 30.0
N_SEG = 16


def plug():
    pin = cq.Workplane("XY").circle(PIN_R).extrude(PIN_L).faces("<Z").chamfer(PIN_CHAMFER)
    pin = pin.cut(cq.Workplane("XY").circle(MAGNET_D / 2).extrude(MAGNET_H))
    body = cq.Workplane("XY").box(*BODY).translate((0, 0, PIN_L + BODY[2] / 2)).edges("|Z").fillet(2.0)
    groove = cq.Workplane("XY").box(BODY[0] + 2, 3.0, 14.0).translate((0, BODY[1] / 2, PIN_L + BODY[2] / 2))
    tether = cq.Workplane("XZ").circle(2.5).extrude(-BODY[1]).translate((0, BODY[1] / 2, PIN_L + BODY[2] - 6))
    return pin.union(body).cut(groove).cut(groove.mirror("XZ")).cut(tether)


def _wall_profile():
    # (r, z) 단면. 볼록 다각형이라 원주 방향으로 잘게 나눈 조각이 거의 볼록하다(충돌 계산용).
    return [(BORE_R, CAP_T), (OUTER_R, CAP_T), (OUTER_R, SOCKET_H), (CONE_TOP_R, SOCKET_H), (BORE_R, SOCKET_H - CONE_H)]


def socket_wall(angle_deg=360.0, start_deg=0.0):
    w = cq.Workplane("XZ").polyline(_wall_profile()).close().revolve(angle_deg, (0, 0, 0), (0, 1, 0))
    return w.rotate((0, 0, 0), (0, 0, 1), start_deg)


def socket_cap():
    return cq.Workplane("XY").circle(OUTER_R).extrude(CAP_T)


def socket():
    flange = (cq.Workplane("XY").box(FLANGE[0], FLANGE[1], FLANGE[2]).translate((0, 0, FLANGE[2] / 2))
              .union(cq.Workplane("XY").box(MARKER_TAB, FLANGE[1], FLANGE[2])
                     .translate((FLANGE[0] / 2 + MARKER_TAB / 2, 0, FLANGE[2] / 2)))
              .edges("|Z").fillet(4.0))
    holes = (cq.Workplane("XY").rect(HOLE_PITCH, HOLE_PITCH, forConstruction=True).vertices()
             .circle(HOLE_D / 2).extrude(FLANGE[2]))
    marker = (cq.Workplane("XY").box(MARKER_POCKET, MARKER_POCKET, 0.6)
              .translate((FLANGE[0] / 2 + MARKER_TAB / 2, 0, FLANGE[2] - 0.3)))
    magnet = cq.Workplane("XY").circle(MAGNET_D / 2).extrude(MAGNET_H).translate((0, 0, CAP_T - MAGNET_H))
    return flange.cut(holes).cut(marker).union(socket_cap()).union(socket_wall()).cut(magnet)


HINGE_Y, HINGE_Z = -50.0, 26.0  # 힌지 축: x 방향, 갑판 한쪽 가장자리
DECK_TOP = 45.0
SOCKET_ON_DECK = (0.0, 15.0)  # 힌지 축에서 65 mm 떨어져 레버를 들면 상하·좌우로 움직인다


def _x_hole(part, y, z, d):
    """x 방향 관통 구멍(힌지 축)."""
    return part.cut(cq.Workplane("YZ").center(y, z).circle(d / 2).extrude(400, both=True))


def deck():
    """흔들림 갑판: 베이스 + 힌지 블록 2개 + 갑판판(러그 2개) + 레버. 힌지 축은 M5 봉 하나(길이 160 mm 이상).

    레버 끝을 손이나 서보 링크로 올리면 갑판이 힌지 축을 중심으로 기울어, 소켓이 상하·좌우로 움직인다.
    """
    base = cq.Workplane("XY").box(220, 150, 6).translate((0, 0, 3)).edges("|Z").fillet(6)
    base = base.cut(cq.Workplane("XY").rect(196, 126, forConstruction=True).vertices().circle(2.6).extrude(6))
    hinge = cq.Workplane("XY").box(16, 24, HINGE_Z + 8 - 6).translate((0, HINGE_Y, 6 + (HINGE_Z + 2) / 2))
    hinges = hinge.translate((-78, 0, 0)).union(hinge.translate((78, 0, 0)))
    plate = (cq.Workplane("XY").box(170, 120, 6).translate((0, 0, DECK_TOP - 3)).edges("|Z").fillet(6)
             .faces(">Z").workplane().center(*SOCKET_ON_DECK)
             .rect(HOLE_PITCH, HOLE_PITCH, forConstruction=True).vertices().hole(HOLE_D))
    lug = cq.Workplane("XY").box(12, 18, DECK_TOP - 6 - (HINGE_Z - 9)).translate(
        (0, HINGE_Y, (DECK_TOP - 6 + HINGE_Z - 9) / 2))
    lugs = lug.translate((-64, 0, 0)).union(lug.translate((64, 0, 0)))
    lever = cq.Workplane("XY").box(20, 50, 6).translate((0, 60 + 25, DECK_TOP - 3)).edges("|Z").fillet(4)
    for k in range(4):
        lever = lever.cut(cq.Workplane("XY").circle(1.6).extrude(10, both=True).translate((0, 70 + k * 10, DECK_TOP - 3)))
    moving = _x_hole(plate.union(lugs).union(lever), HINGE_Y, HINGE_Z, 5.3)
    fixed = _x_hole(base.union(hinges), HINGE_Y, HINGE_Z, 5.3)
    return fixed, moving


def car_port():
    """자동차 충전구 블록: 45° 경사면에 소켓 플랜지 장착. 속을 비워(벽 3 mm) 출력 재료를 줄였다."""
    w = 90.0
    block = cq.Workplane("XZ").polyline([(0, 0), (w, 0), (w, w)]).close().extrude(40, both=True)
    block = block.faces("<Z").shell(-3.0)
    holes = (cq.Workplane("XY").rect(HOLE_PITCH, HOLE_PITCH, forConstruction=True).vertices().circle(HOLE_D / 2)
             .extrude(30, both=True).rotate((0, 0, 0), (0, 1, 0), -45).translate((w / 2, 0, w / 2)))
    return block.cut(holes)


def assembly():
    """갑판 위 소켓(플러그 꽂힘) + 자동차 충전구 블록. 위치는 갑판 베이스 중심 기준."""
    fixed, moving = deck()
    a = cq.Assembly(name="plugbot_kit")
    a.add(fixed, name="deck_base", color=cq.Color(0.70, 0.72, 0.75))
    a.add(moving, name="deck_plate", color=cq.Color(0.84, 0.86, 0.89))
    sx, sy = SOCKET_ON_DECK
    a.add(socket(), name="socket_ship", loc=cq.Location((sx, sy, DECK_TOP)), color=cq.Color(0.93, 0.55, 0.12))
    a.add(plug(), name="plug", loc=cq.Location((sx, sy, DECK_TOP + CAP_T)), color=cq.Color(0.12, 0.30, 0.62))
    a.add(car_port(), name="car_port", loc=cq.Location((170, 0, 0)), color=cq.Color(0.20, 0.33, 0.52))
    a.add(socket(), name="socket_car", loc=cq.Location((170 + 45, 0, 45), (0, 1, 0), -45),
          color=cq.Color(0.93, 0.55, 0.12))
    return a


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    deck_base, deck_plate = deck()
    parts = {"plug": plug(), "socket": socket(), "deck_base": deck_base, "deck_plate": deck_plate,
             "car_port": car_port()}
    for name, part in parts.items():
        cq.exporters.export(part, str(OUT / f"{name}.step"))
        cq.exporters.export(part, str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        bb = part.val().BoundingBox()
        print(f"{name:9s} {bb.xlen:6.1f} × {bb.ylen:6.1f} × {bb.zlen:6.1f} mm, 부피 {part.val().Volume() / 1000:6.1f} cm³")
    step = 360.0 / N_SEG
    for k in range(N_SEG):
        cq.exporters.export(socket_wall(step, k * step), str(OUT / f"socket_wall_{k:02d}.stl"), tolerance=0.05)
    cq.exporters.export(socket_cap(), str(OUT / "socket_cap.stl"), tolerance=0.05)
    assembly().save(str(OUT / "assembly.step"))
    print("socket height", SOCKET_H, "capture radius", CONE_TOP_R - PIN_R, "radial clearance", BORE_R - PIN_R)
