"""실제 크기 장면의 페리 현측 무늬(팬스타 미라클호 사진을 보고 흉내 낸 색과 띠, 글씨). 결과: isaac/scaleup/livery_panstar.png

그림 한 장이 선체 옆면 전체(길이 40 m × 용골~갑판 5.7 m, 1 px = 1 cm)를 덮는다. 왼쪽 끝 = 뱃머리, 오른쪽 끝 = 선미
(부두에서 볼 때 뱃머리가 왼쪽). 실제 도장 도면이 아니라 사진을 보고 비슷하게 그린 것이다.
실행: ~/plugbot/.venv/bin/python isaac/scaleup/make_livery.py
"""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent / "livery_panstar.png"
W, H = 4000, 570  # 40 m × 5.7 m
WATER = 340  # 수선(갑판에서 3.4 m 아래)
FONT = "/usr/share/fonts/opentype/urw-base35/NimbusSans-BoldItalic.otf"
WHITE, ORANGE, BLUE, NAVY, RED = (240, 242, 245), (232, 128, 40), (34, 72, 150), (24, 40, 92), (150, 38, 30)


def smooth(u):
    u = np.clip(u, 0, 1)
    return u * u * (3 - 2 * u)


def band(top, bot, u0, u1):
    us = np.linspace(u0, u1, 200)
    upper = [(u * W, top(u)) for u in us]
    lower = [(u * W, bot(u)) for u in us[::-1]]
    return upper + lower


img = Image.new("RGB", (W, H), WHITE)
d = ImageDraw.Draw(img)
# 파란 띠: 선미 쪽 아래에서 위로 올라간다
d.polygon(band(lambda u: 250 - 190 * smooth((u - 0.45) / 0.55), lambda u: 300 - 120 * smooth((u - 0.45) / 0.55), 0.45, 1.0), fill=BLUE)
# 주황 띠: 뱃머리 아래쪽에서 시작해 선미 위로 휜다
d.polygon(band(lambda u: 250 - 230 * smooth((u - 0.05) / 0.95), lambda u: 300 - 175 * smooth((u - 0.05) / 0.95), 0.05, 1.0), fill=ORANGE)
d.polygon(band(lambda u: 250, lambda u: 300, 0.0, 0.05), fill=ORANGE)
# 글씨(사진처럼 띠 위에 진한 남색, 흰 테두리)
f = ImageFont.truetype(FONT, 190)
d.text((0.45 * W, 205), "PanStar Cruise", font=f, fill=NAVY, anchor="lm", stroke_width=8, stroke_fill=WHITE)
f2 = ImageFont.truetype(FONT, 44)
d.text((0.06 * W, 150), "PanStar Miracle", font=f2, fill=NAVY, anchor="lm")
# 수선 아래 붉은 칠, 수선 띠
d.rectangle((0, WATER - 6, W, H), fill=RED)
d.rectangle((0, WATER - 14, W, WATER - 6), fill=NAVY)
img.save(OUT)
print(OUT, img.size)
