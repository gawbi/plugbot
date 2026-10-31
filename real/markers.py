"""인쇄용 마커 시트(A4 PDF, 실제 크기). 인쇄할 때 '실제 크기(100%)'로 뽑고 자로 한 변을 재어 확인한다.

실행: .venv/bin/python -m real.markers
결과(real/print/): 01_소켓마커.pdf, 02_보정시트.pdf(예비), 03_카메라보정판.pdf,
                  04_갑판마커.pdf(top 배치), 05_판마커_선박.pdf, 06_판마커_자동차.pdf(side 배치, 본선)
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import geometry as G

OUT = Path(__file__).resolve().parent / "print"
DPI = 300
A4 = (210, 297)  # mm
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"


def px(mm):
    return int(round(mm / 25.4 * DPI))


def _dict(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_image(marker_id, size_mm, dict_name=G.DICT):
    img = cv2.aruco.generateImageMarker(_dict(dict_name), marker_id, px(size_mm))
    return Image.fromarray(img).convert("RGB")


def _page():
    page = Image.new("RGB", (px(A4[0]), px(A4[1])), "white")
    return page, ImageDraw.Draw(page)


def _text(d, xy_mm, s, size_pt=11):
    d.text((px(xy_mm[0]), px(xy_mm[1])), s, font=ImageFont.truetype(FONT, int(size_pt / 72 * DPI)), fill=(0, 0, 0))


def _ruler(d, x_mm, y_mm, length_mm=50):
    """크기 확인용 50 mm 눈금자."""
    d.line([(px(x_mm), px(y_mm)), (px(x_mm + length_mm), px(y_mm))], fill=(0, 0, 0), width=3)
    for k in range(0, length_mm + 1, 10):
        d.line([(px(x_mm + k), px(y_mm - 2)), (px(x_mm + k), px(y_mm))], fill=(0, 0, 0), width=3)
    _text(d, (x_mm, y_mm + 1.5), f"{length_mm} mm (자로 재어 맞는지 확인)", 9)


def socket_sheet():
    page, d = _page()
    _text(d, (20, 18), "소켓 마커 (ArUco 4x4, 25 mm): 선박 id 0, 자동차 id 10", 16)
    _text(d, (20, 28), "소켓 탭의 30 mm 홈에 붙인다. 마커의 오른쪽이 탭 바깥쪽(소켓에서 먼 쪽)을 향하게.", 10)
    for row, (target, label) in enumerate((("ship", "선박 소켓"), ("car", "자동차 충전구"))):
        mid = G.SOCKET_MARKER_ID + G.TARGET_ID_OFFSET[target]
        _text(d, (20, 45 + row * 55), f"{label} (id {mid})", 11)
        for k in range(2):  # 여분 포함 2장
            x, y = 40 + k * 55, 55 + row * 55
            d.rectangle([px(x - 2.5), px(y - 2.5), px(x + 27.5), px(y + 27.5)], outline=(150, 150, 150), width=2)
            page.paste(marker_image(mid, 25), (px(x), px(y)))
            _text(d, (x - 2, y + 30), "← 깔때기 쪽", 9)
    _text(d, (20, 160), "자르는 선(회색)은 30 mm. 흰 여백을 남기고 잘라 홈에 맞춘다.", 10)
    _ruler(d, 20, 175)
    return page


def deck_sheet():
    """갑판판 위에 깔고 소켓을 그 위에 볼트로 고정하는 마커 시트(실제 크기). 소켓 중심 = 볼트 구멍 4개의 중심."""
    page, d = _page()
    cx, cy = 105, 150  # 소켓 중심(페이지 mm)

    def P(x, y):  # 소켓 좌표(mm) → 페이지 픽셀(위가 소켓 +y)
        return px(cx + x), px(cy - y)

    _text(d, (15, 12), "갑판 마커 시트 (id 5~8, 30 mm) - 소켓 아래에 깐다", 15)
    _text(d, (15, 21), "1) 회색 갑판판 외곽(170 x 120 mm)에 맞춰 붙인다  2) 볼트 구멍 4개를 뚫고 소켓을 이 위에 볼트로 고정", 9)
    _text(d, (15, 27), "3) 소켓 탭(마커 홈)이 오른쪽, 힌지(로봇 쪽)가 아래로 오게. 마커가 가려지지 않게 볼트 머리는 낮은 것", 9)
    px0, py0 = G.PLATE_IN_SOCKET
    d.rectangle([P(px0 - 85, py0 + 60), P(px0 + 85, py0 - 60)], outline=(150, 150, 150), width=3)
    d.rectangle([P(-35, 35), P(35, -35)], outline=(200, 120, 0), width=3)  # 소켓 플랜지
    d.rectangle([P(35, 35), P(75, -35)], outline=(200, 120, 0), width=3)  # 마커 탭
    r = px(4.5 / 2)
    for sx in (-27, 27):
        for sy in (-27, 27):
            x, y = P(sx, sy)
            d.ellipse([x - r, y - r, x + r, y + r], outline=(0, 0, 0), width=4)
            d.line([(x - r * 2, y), (x + r * 2, y)], fill=(0, 0, 0), width=2)
            d.line([(x, y - r * 2), (x, y + r * 2)], fill=(0, 0, 0), width=2)
    for mid, (x, y) in G.DECK_MARKERS.items():
        X, Y = P(x - 15, y + 15)
        page.paste(marker_image(mid, 30), (X, Y))
        _text(d, (cx + x - 6, cy - y + 16), f"id {mid}", 8)
    _text(d, (cx - 28, cy - 2), "소켓 자리(플랜지 70 mm)", 9)
    _text(d, (cx - 20, cy - (py0 - 60) + 3), "↓ 로봇(힌지) 쪽", 11)
    _ruler(d, 15, 240)
    return page


def panel_sheet(target="ship"):
    """세운 판(side_mount) 앞면에 붙이는 마커 시트(실제 크기). 로봇 쪽에서 본 모습 그대로 인쇄된다.
    소켓 좌표 (x 위, y 왼쪽) → 페이지 (오른쪽 = -y, 위 = x). 마커의 오른쪽이 소켓 +x(위)를 보도록 90° 돌려 붙인다."""
    page, d = _page()
    cx, cy = 105, 150

    def P(x, y):  # 소켓 좌표(mm) → 페이지 픽셀
        return px(cx - y), px(cy - x)

    k = G.TARGET_ID_OFFSET[target]
    label = "선박 소켓" if target == "ship" else "자동차 충전구"
    _text(d, (15, 12), f"{label} 세운 판 마커 (id {5 + k}~{8 + k}, 30 mm) - 판 앞면에 붙이고 소켓을 그 위에 볼트로", 13)
    _text(d, (15, 21), "1) 회색 외곽(170 x 119 mm)을 판 앞면(발판 위)에 맞춰 붙인다  2) 볼트 구멍 4개를 뚫는다", 9)
    _text(d, (15, 27), "3) 소켓 탭(마커 홈)이 위로 오게 소켓을 고정한다. 이 종이는 로봇 쪽에서 본 모습이다", 9)
    top, bottom = 125 - 85, 6 - 85  # 판 윗면, 발판 윗면(소켓 x)
    d.rectangle([P(top, 85), P(bottom, -85)], outline=(150, 150, 150), width=3)
    d.rectangle([P(35, 35), P(-35, -35)], outline=(200, 120, 0), width=3)  # 소켓 플랜지
    r = px(4.5 / 2)
    for sx in (-27, 27):
        for sy in (-27, 27):
            x, y = P(sx, sy)
            d.ellipse([x - r, y - r, x + r, y + r], outline=(0, 0, 0), width=4)
            d.line([(x - r * 2, y), (x + r * 2, y)], fill=(0, 0, 0), width=2)
            d.line([(x, y - r * 2), (x, y + r * 2)], fill=(0, 0, 0), width=2)
    for mid, (x, y) in G.PANEL_MARKERS.items():
        X, Y = P(x + 15, y + 15)  # 돌린 마커의 왼쪽 위 = 소켓 (x+15, y+15)
        page.paste(marker_image(mid + k, 30).rotate(90), (X, Y))
        _text(d, (cx - y - 6, cy - x + 16), f"id {mid + k}", 8)
    _text(d, (cx - 22, cy - 2), "소켓 자리(플랜지 70 mm)", 9)
    _text(d, (cx - 14, cy - top - 9), "↑ 위(소켓 탭)", 11)
    _text(d, (cx - 14, cy - bottom + 2), "↓ 발판", 11)
    _ruler(d, 15, 250)
    return page


def calib_sheet():
    page, d = _page()
    _text(d, (20, 18), "카메라-로봇 보정 시트 (id 1~4, 30 mm, 중심 간격 60 mm)", 16)
    _text(d, (20, 28), "(예비용) 로봇 앞 팔이 닿는 평평한 곳에 붙인다. 플러그 핀 끝을 각 마커 중심에 차례로 댄다.", 10)
    ox, oy = 105 - 30, 70
    for k, mid in enumerate(G.CALIB_MARKER_IDS):
        cx, cy = ox + (k % 2) * 60, oy + (k // 2) * 60
        page.paste(marker_image(mid, 30), (px(cx - 15), px(cy - 15)))
        # 중심 표시(마커 안쪽 흰 칸을 해치지 않게 바깥에 짧은 눈금)
        for dx, dy in ((-19, 0), (19, 0), (0, -19), (0, 19)):
            d.line([(px(cx + dx * 0.8), px(cy + dy * 0.8)), (px(cx + dx), px(cy + dy))], fill=(220, 0, 0), width=4)
        _text(d, (cx - 4, cy + 18), f"id {mid}", 10)
    _ruler(d, 20, 210)
    return page


def charuco_sheet():
    page, d = _page()
    board = cv2.aruco.CharucoBoard(G.CHARUCO_SQUARES, G.CHARUCO_SQUARE, G.CHARUCO_MARKER, _dict(G.CHARUCO_DICT))
    w, h = (px(G.CHARUCO_SQUARES[0] * 30), px(G.CHARUCO_SQUARES[1] * 30))
    img = board.generateImage((w, h))
    page.paste(Image.fromarray(img).convert("RGB"), (px(15), px(60)))
    _text(d, (15, 18), "카메라 보정판 (ChArUco 7x5, 칸 30 mm, 마커 22 mm)", 16)
    _text(d, (15, 28), "평평한 판에 붙여 웹캠 앞에서 여러 각도로 15장쯤 찍는다(real.vision calib).", 10)
    _ruler(d, 15, 220)
    return page


def main():
    OUT.mkdir(exist_ok=True)
    for name, page in (("01_소켓마커", socket_sheet()), ("02_보정시트", calib_sheet()), ("03_카메라보정판", charuco_sheet()),
                       ("04_갑판마커", deck_sheet()), ("05_판마커_선박", panel_sheet("ship")),
                       ("06_판마커_자동차", panel_sheet("car"))):
        page.save(OUT / f"{name}.pdf", resolution=DPI)
        page.save(OUT / f"{name}.png")
        print(OUT / f"{name}.pdf")


if __name__ == "__main__":
    main()
