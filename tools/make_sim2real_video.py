"""Sim-to-Real 영상: Isaac 트윈 학습 화면 → 실물 혼자 꽂기(트윈으로만 배운 모델)와 같은 관절값의 Isaac 재생을 나란히.

1) 앞 5초: 학습에 쓴 Isaac 트윈 렌더(실물 시범 관절값을 Isaac에서 재생, 카메라는 실물에 맞춤) 네 칸
2) 본편: 실물 앞, 손목 카메라(data/rollouts/<run>) | 같은 관절값을 Isaac에서 재생한 앞, 손목 화면(1.5배속)
3) 끝 3초: 결과(사람 판정 집계, tools/tally_rollouts.py와 같은 숫자)
실행: ~/lerobot_env/bin/python tools/make_sim2real_video.py --run 20261110_024600 --replay <렌더 폴더> --out <mp4>
"""

import argparse
import glob
from pathlib import Path

import imageio.v2 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument("--run", default="20261110_024600")
ap.add_argument("--replay", required=True, help="run_twin.py로 렌더한 폴더(ep_900_n_front.mp4, _wrist.mp4)")
ap.add_argument("--train", default=str(ROOT / "data/twin_render3"))
ap.add_argument("--ok", type=int, default=10)
ap.add_argument("--n", type=int, default=10)
ap.add_argument("--out", required=True)
a = ap.parse_args()

W, H, FPS = 1920, 1080, 30
F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def font(sz, bold=True):
    return ImageFont.truetype(F if bold else REG, sz, index=1)  # 1 = KR


BG, FG, SUB, ACC = (14, 14, 18), (240, 240, 245), (165, 165, 180), (178, 120, 255)


def label(img, text):
    d = ImageDraw.Draw(img)
    f = font(26)
    w = d.textlength(text, font=f)
    d.rectangle([0, 0, w + 24, 44], fill=(0, 0, 0))
    d.text((12, 4), text, font=f, fill=FG)
    return img


def wilson(k, n, z=1.96):
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0, c - h), min(1, c + h)


lo, hi = wilson(a.ok, a.n)
Path(a.out).parent.mkdir(parents=True, exist_ok=True)
w = iio.get_writer(a.out, fps=FPS, codec="libx264", quality=7, macro_block_size=8)
nfr = 0


def put(c):
    global nfr
    w.append_data(np.asarray(c))
    nfr += 1


# 1) 학습 화면
eps = [5, 20, 40, 54]
readers = [iio.get_reader(Path(a.train) / f"ep_{e:03d}_n_front.mp4") for e in eps]
clips = [[fr for fr in r] for r in readers]
for t in range(5 * FPS):
    c = Image.new("RGB", (W, H), BG)
    for k, cl in enumerate(clips):
        i = int(len(cl) * (0.15 + 0.85 * t / (5 * FPS)))
        im = Image.fromarray(cl[min(i, len(cl) - 1)]).resize((720, 540))
        c.paste(label(im, f"Isaac 트윈 시범 {eps[k]}번"), (k % 2 * 720, k // 2 * 540))
    d = ImageDraw.Draw(c)
    d.text((1470, 120), "학습 데이터", font=font(44), fill=ACC)
    y = 210
    for line in ["사람이 원격조종한", "실물 시범 69개의 관절값을", "Isaac 트윈에서 재생해", "화면만 새로 그렸다", "", "실물 사진: 0장", "카메라: 실물 꽂힌 장면에", "맞춰 보정"]:
        d.text((1470, y), line, font=font(32, bold=False), fill=FG if line else FG)
        y += 52
    put(c)
clips = None

# 2) 실물 | Isaac 재생
R = ROOT / "data/rollouts" / a.run
rf = sorted(glob.glob(str(R / "front/*.jpg")))
rw = sorted(glob.glob(str(R / "wrist/*.jpg")))
sf = [fr for fr in iio.get_reader(Path(a.replay) / "ep_900_n_front.mp4")]
sw = [fr for fr in iio.get_reader(Path(a.replay) / "ep_900_n_wrist.mp4")]
n = min(len(rf), len(rw), len(sf), len(sw))
for t in np.arange(0, n, 1.5):
    i = int(t)
    c = Image.new("RGB", (W, H), BG)
    c.paste(label(Image.open(rf[i]).convert("RGB").resize((720, 540)), "실물 앞 카메라"), (0, 0))
    c.paste(label(Image.fromarray(sf[i]).resize((720, 540)), "Isaac 트윈 앞 카메라"), (720, 0))
    c.paste(label(Image.open(rw[i]).convert("RGB").resize((720, 540)), "실물 손목 카메라"), (0, 540))
    c.paste(label(Image.fromarray(sw[i]).resize((720, 540)), "Isaac 트윈 손목 카메라"), (720, 540))
    d = ImageDraw.Draw(c)
    d.text((1470, 90), "Sim-to-Real", font=font(48), fill=ACC)
    y = 180
    for line, f_, col in [("Isaac 화면만으로 배운 모델이", font(32), FG), ("실물 소켓에 혼자 꽂는다", font(32), FG), ("", font(20), FG),
                          ("왼쪽: 실물(모델이 직접 조종)", font(26, False), SUB), ("오른쪽: 같은 관절값을", font(26, False), SUB),
                          ("Isaac 트윈에서 재생", font(26, False), SUB), ("", font(20), FG),
                          ("모델 act_twin_v4, 1,000스텝", font(26, False), SUB), ("정지 소켓, 1.5배속", font(26, False), SUB)]:
        d.text((1470, y), line, font=f_, fill=col)
        y += 48 if line else 24
    d.text((1470, 860), f"실제 시간 {t / 30:4.1f}초", font=font(28, False), fill=SUB)
    put(c)

# 3) 결과
for t in range(4 * FPS):
    c = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(c)
    d.text((160, 200), "결과 (2026-11-10 배 위, 사람 판정)", font=font(44), fill=SUB)
    d.text((160, 320), "Sim-to-Real", font=font(56), fill=ACC)
    d.text((160, 410), f"Isaac 트윈 화면만으로 학습: 정지 소켓 {a.n}번 중 {a.ok}번", font=font(52), fill=FG)
    d.text((160, 490), f"95% 신뢰구간 {lo * 100:.0f}~{hi * 100:.0f}%", font=font(34, False), fill=SUB)
    d.text((160, 600), "같은 데이터로 더 학습한 2,000~4,000스텝은 각 1번 시도, 모두 실패", font=font(32, False), fill=SUB)
    d.text((160, 650), "(시뮬레이션 화면에 지나치게 맞춰지는 문제. 흔들림 소켓은 아직 재지 않음)", font=font(32, False), fill=SUB)
    put(c)

w.close()
print("저장", a.out, nfr / FPS, "초")
