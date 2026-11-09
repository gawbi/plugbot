"""세 단계 영상을 1:1:1로 한 화면에(왼쪽부터 Real-to-Real, Sim-to-Real, 이종 원격조종). 세 칸이 같이 재생되고, 짧은 칸은 마지막 장면에서 멈춘다.
실행: ~/lerobot_env/bin/python tools/make_three_side.py   (먼저 tools/make_three_videos.py)
"""

from pathlib import Path

import imageio.v2 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

V = Path.home() / "Downloads/PlugBot_본선준비/5_발표_사진영상/영상"
W, H, FPS = 1920, 1080, 30
BG, FG, SUB, ACC = (14, 14, 18), (240, 240, 245), (165, 165, 180), (178, 120, 255)


def font(sz, bold=True):
    return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc" % ("Bold" if bold else "Regular"), sz, index=1)


COLS = [  # 파일, 앞에서 자를 초, 뒤에서 자를 초, 단계, 제목 두 줄, 결과 줄들
    ("단계1_RealToReal_사람시범에서_혼자꽂기.mp4", 2, 3, "1  Real-to-Real", ["사람이 원격조종한 시범으로", "배운 로봇이 혼자 꽂는다"],
     ["배 위 20번 중 19번", "정지와 흔들림 섞음, 사람 판정", "95% 구간 76~99%"]),
    ("단계2_SimToReal_아이작트윈에서_실물로.mp4", 2, 4, "2  Sim-to-Real", ["Isaac 트윈 화면만으로 배워", "실물 소켓에 꽂는다"],
     ["정지 소켓 10번 중 10번", "실물 사진 0장으로 학습", "95% 구간 72~100%"]),
    ("단계3_이종원격조종_작은팔에서_산업용로봇으로.mp4", 2, 3, "3  이종 원격조종", ["작은 팔의 엔드이펙터 움직임을", "산업용 로봇 UR10e로"],
     ["Isaac Sim에서 실시간 조종", "실물 혼자 꽂기 동작을 UR10e로 재생", "실물 산업용 로봇은 다음 단계"]),
]

clips = []
for f, a, b, *_ in COLS:
    r = iio.get_reader(V / f)
    fr = [np.asarray(Image.fromarray(x).resize((620, 349))) for x in r]  # 작게 줄여 메모리 아끼기
    clips.append(fr[a * FPS: len(fr) - b * FPS])
T = max(len(c) for c in clips)
out = V / "세단계_1대1대1_RealToReal_SimToReal_이종원격조종.mp4"
w = iio.get_writer(out, fps=FPS, codec="libx264", quality=7, macro_block_size=8)
base = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(base)
for k, (_, _, _, step, title, res) in enumerate(COLS):
    x = k * 640 + 30
    d.text((x, 150), step, font=font(40), fill=ACC)
    for j, ln in enumerate(title):
        d.text((x, 215 + j * 46), ln, font=font(32), fill=FG)
    d.text((x, 760), res[0], font=font(40), fill=FG)
    for j, ln in enumerate(res[1:]):
        d.text((x, 825 + j * 44), ln, font=font(28, False), fill=SUB)
for k in (1, 2):
    d.line([(k * 640, 120), (k * 640, 960)], fill=(50, 50, 60), width=2)
for t in range(T + 3 * FPS):
    c = base.copy()
    for k, cl in enumerate(clips):
        im = Image.fromarray(cl[min(t, len(cl) - 1)])
        c.paste(im, (k * 640 + 10, 350))
    w.append_data(np.asarray(c))
w.close()
print("저장", out, f"{(T + 3 * FPS) / FPS:.1f}초")
