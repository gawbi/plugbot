"""발표용 세 단계 영상: 1/3 Real-to-Real, 2/3 Sim-to-Real, 3/3 이종 원격조종. 같은 틀(제목 2초, 본편, 결과 3초).

재료(발표 폴더 ~/Downloads/PlugBot_본선준비):
  1/3: 5_발표_사진영상/영상/teleop_1992.mp4(사람 원격조종), auto_2079.mp4(ACT 혼자 꽂기) 세로 영상 나란히
  2/3: 5_발표_사진영상/영상/심투리얼_아이작트윈_실물_10번중10번.mp4(tools/make_sim2real_video.py)
  3/3: ~/plugbot/outputs/scaleup_teleop.mp4(SO-101 → UR10e, Isaac), rollout_pair_20261106_191627.mp4(실물 혼자 꽂기 → UR10e)
실행: ~/lerobot_env/bin/python tools/make_three_videos.py
"""

from pathlib import Path

import imageio.v2 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

P = Path.home() / "Downloads/PlugBot_본선준비"
V = P / "5_발표_사진영상/영상"
OUTS = Path.home() / "plugbot/outputs"
W, H, FPS = 1920, 1080, 30
BG, FG, SUB, ACC = (14, 14, 18), (240, 240, 245), (165, 165, 180), (178, 120, 255)


def font(sz, bold=True):
    return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc" % ("Bold" if bold else "Regular"), sz, index=1)


class Out:
    def __init__(self, path):
        self.w = iio.get_writer(path, fps=FPS, codec="libx264", quality=7, macro_block_size=8)
        self.n = 0
        self.path = path

    def put(self, img):
        self.w.append_data(np.asarray(img.convert("RGB")))
        self.n += 1

    def close(self):
        self.w.close()
        print("저장", self.path, f"{self.n / FPS:.1f}초")


def card(o, sec, step, title, lines, big=None):
    for _ in range(int(sec * FPS)):
        c = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(c)
        d.text((160, 220), step, font=font(40), fill=ACC)
        d.text((160, 290), title, font=font(64), fill=FG)
        y = 420
        if big:
            d.text((160, y), big, font=font(56), fill=FG)
            y += 100
        for ln in lines:
            d.text((160, y), ln, font=font(34, False), fill=SUB)
            y += 56
        o.put(c)


def band(c, step, text):
    d = ImageDraw.Draw(c)
    d.rectangle([0, 0, W, 64], fill=(0, 0, 0))
    d.text((24, 10), step, font=font(32), fill=ACC)
    d.text((24 + d.textlength(step, font=font(32)) + 24, 10), text, font=font(32), fill=FG)
    return c


def frames(path):
    return iio.get_reader(path)


def fit(img, w, h):
    im = Image.fromarray(img) if not isinstance(img, Image.Image) else img
    s = min(w / im.width, h / im.height)
    return im.resize((int(im.width * s), int(im.height * s)))


# ---------- 1/3 Real-to-Real ----------
o = Out(str(V / "단계1_RealToReal_사람시범에서_혼자꽂기.mp4"))
S1 = "1/3 Real-to-Real"
card(o, 2, S1, "사람이 보여 주고, 로봇이 배운다", ["작은 리더 팔로 직접 꽂아 보인 시범으로 학습(ACT)"])
tl = [f for f in frames(V / "teleop_1992.mp4")]
au = [f for f in frames(V / "auto_2079.mp4")]
n = len(au)
for i in range(n):
    c = Image.new("RGB", (W, H), BG)
    a = fit(tl[min(int(i * len(tl) / n), len(tl) - 1)], 560, 1000)
    b = fit(au[i], 560, 1000)
    c.paste(a, (120, 70))
    c.paste(b, (740, 70))
    d = ImageDraw.Draw(c)
    d.text((1360, 160), "왼쪽", font=font(36), fill=ACC)
    d.text((1360, 215), "사람이 리더 팔로 원격조종", font=font(32, False), fill=FG)
    d.text((1360, 265), f"시범 134번(정지 94, 흔들림 40), {len(tl) / n:.1f}배속", font=font(26, False), fill=SUB)
    d.text((1360, 380), "오른쪽", font=font(36), fill=ACC)
    d.text((1360, 435), "그 시범으로 배운 ACT가", font=font(32, False), fill=FG)
    d.text((1360, 485), "사람 없이 혼자 꽂는다", font=font(32, False), fill=FG)
    d.text((1360, 535), "실물 SO-101, 실제 속도", font=font(26, False), fill=SUB)
    o.put(band(c, S1, "원격조종 시범 → 학습 → 혼자 꽂기"))
card(o, 3, S1, "배 위에서 20번 중 19번 혼자 꽂았다", ["정지와 흔들림 소켓 섞어서, act_real_v4 8만 스텝, 사람 판정", "95% 신뢰구간 76~99%, 팀원 노트북 실행 기록(본선 중)"])
o.close()

# ---------- 2/3 Sim-to-Real ----------
o = Out(str(V / "단계2_SimToReal_아이작트윈에서_실물로.mp4"))
S2 = "2/3 Sim-to-Real"
card(o, 2, S2, "시뮬레이션 화면으로 배우고, 실물에서 꽂는다", ["실물 시범 관절값을 Isaac 트윈에서 재생해 그린 화면만으로 학습(실물 사진 0장)"])
for f in frames(V / "심투리얼_아이작트윈_실물_10번중10번.mp4"):
    o.put(band(Image.fromarray(f), S2, "Isaac 트윈 학습 화면 → 실물 혼자 꽂기"))
o.close()

# ---------- 3/3 이종 원격조종 ----------
o = Out(str(V / "단계3_이종원격조종_작은팔에서_산업용로봇으로.mp4"))
S3 = "3/3 이종 원격조종"
card(o, 2, S3, "작은 팔로 큰 산업용 로봇을 움직인다", ["SO-101 리더 팔의 엔드이펙터 움직임을 UR10e(실제 크기)로 옮긴다, Isaac Sim"])
for f in frames(OUTS / "scaleup_teleop.mp4"):
    c = Image.new("RGB", (W, H), BG)
    im = fit(f, W, H - 160)
    c.paste(im, ((W - im.width) // 2, 64 + (H - 64 - 96 - im.height) // 2))
    d = ImageDraw.Draw(c)
    d.text((40, H - 80), "입력: SO-101 리더 팔(모형 1:10)  →  출력: UR10e(실제 크기), 페리 현측 소켓. Isaac Sim 시각화", font=font(28, False), fill=SUB)
    o.put(band(c, S3, "리더 팔로 UR10e 원격조종"))
pr = frames(OUTS / "rollout_pair_20261106_191627.mp4")
for k, f in enumerate(pr):
    if k % 3 == 2:  # 1.5배속
        continue
    o.put(band(Image.fromarray(f), S3, "실물이 혼자 꽂은 동작을 UR10e로 (1.5배속)"))
card(o, 3, S3, "같은 시범을 큰 로봇으로 옮길 수 있다", ["SO-101 리더 팔로 Isaac의 UR10e를 실시간 조종해 부두 소켓에 꽂음(팀원 노트북)",
                                                 "실물 혼자 꽂기 관절 기록을 UR10e 엔드이펙터로 옮겨 재생(시뮬레이션, 충돌 계산 없음)",
                                                 "실물 산업용 로봇에서 하는 것은 다음 단계"])
o.close()
