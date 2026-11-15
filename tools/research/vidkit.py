"""공부용 설명 영상 공용 도구: 색, 글꼴, 제목 카드, 자막, 마무리 카드, mp4 쓰기.

모양새는 하나로 맞춘다: 밤바다 남색 바탕, 보라 = 실물, 청록 = 시뮬레이션, 노랑 = 강조(한 화면에 하나).
영상은 저장소에 커밋하지 않는다(~/Downloads/PlugBot_연구/영상/).
"""

from pathlib import Path

import imageio.v2 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

W, H, FPS = 1920, 1080, 30
BG = (14, 21, 34)
PANEL = (21, 32, 54)
INK = (231, 235, 241)
MUTED = (155, 165, 180)
REAL = (166, 139, 255)
SIM = (92, 184, 200)
HI = (255, 204, 77)
BAD = (240, 113, 103)
OUT = Path.home() / "Downloads/PlugBot_연구/영상"
_F = {}


def font(sz, bold=False):
    k = (sz, bold)
    if k not in _F:
        _F[k] = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc" % ("Bold" if bold else "Regular"), sz, index=1)
    return _F[k]


def ease(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def canvas(color=BG):
    return Image.new("RGB", (W, H), color)


def text(d, xy, s, sz=36, bold=False, fill=INK, anchor="la"):
    d.text(xy, s, font=font(sz, bold), fill=fill, anchor=anchor)


def wrap(s, sz, width):
    """한국어 줄바꿈(어절 단위)"""
    words, lines, cur = s.split(" "), [], ""
    f = font(sz)
    for w in words:
        t = (cur + " " + w).strip()
        if f.getlength(t) > width and cur:
            lines.append(cur)
            cur = w
        else:
            cur = t
    if cur:
        lines.append(cur)
    return lines


def caption(img, s, alpha=1.0, y=H - 96, sz=38):
    """아래쪽 자막 띠. alpha로 들어오고 나간다"""
    if not s or alpha <= 0:
        return img
    lines = wrap(s, sz, W - 360)
    hgt = 26 + len(lines) * (sz + 16)
    band = Image.new("RGBA", (W, hgt), (8, 12, 20, int(205 * alpha)))
    bd = ImageDraw.Draw(band)
    for i, ln in enumerate(lines):
        bd.text((W // 2, 14 + i * (sz + 16)), ln, font=font(sz), fill=INK + (int(255 * alpha),), anchor="ma")
    base = img.convert("RGBA")
    base.alpha_composite(band, (0, y - hgt + 40))
    return base.convert("RGB")


def caption_at(cues, t):
    """cues: [(시작 s, 끝 s, 글)] → (글, 알파)"""
    for a, b, s in cues:
        if a <= t < b:
            return s, min(ease((t - a) / 0.35), ease((b - t) / 0.35))
    return "", 0.0


def title_card(chapter, title, question, k, n):
    """장 번호, 제목, 질문 한 줄. k/n 진행에 따라 천천히 나타난다"""
    img = canvas()
    d = ImageDraw.Draw(img)
    p = ease(k / max(1, n * 0.45))
    x0 = 180
    d.rectangle([x0, 380, x0 + int(8 * p) + 1, 640], fill=REAL)
    text(d, (x0 + 40, 360), chapter, 34, False, tuple(int(c * p + b * (1 - p)) for c, b in zip(MUTED, BG)))
    text(d, (x0 + 40, 420), title, 84, True, tuple(int(c * p + b * (1 - p)) for c, b in zip(INK, BG)))
    q = ease((k - n * 0.35) / max(1, n * 0.4))
    for i, ln in enumerate(wrap(question, 44, 1400)):
        text(d, (x0 + 40, 560 + i * 62), ln, 44, False, tuple(int(c * q + b * (1 - q)) for c, b in zip(HI, BG)))
    return img


def end_card(points, k, n, title="정리"):
    img = canvas()
    d = ImageDraw.Draw(img)
    text(d, (180, 200), title, 56, True, INK)
    for i, s in enumerate(points):
        q = ease((k - i * 12) / 18)
        col = tuple(int(c * q + b * (1 - q)) for c, b in zip(INK, BG))
        d.ellipse([186, 330 + i * 130, 206, 350 + i * 130], fill=tuple(int(c * q + b * (1 - q)) for c, b in zip(REAL, BG)))
        for j, ln in enumerate(wrap(s, 42, 1450)):
            text(d, (240, 318 + i * 130 + j * 56), ln, 42, False, col)
    return img


def fit(im, w, h):
    """비율을 지켜 w x h 안에 맞춘 PIL 이미지"""
    im = im if isinstance(im, Image.Image) else Image.fromarray(im)
    r = min(w / im.width, h / im.height)
    return im.resize((max(1, int(im.width * r)), max(1, int(im.height * r))), Image.LANCZOS)


def panel(img, box, radius=18, color=PANEL):
    d = ImageDraw.Draw(img)
    d.rounded_rectangle(box, radius, fill=color)


def label(d, xy, s, color, sz=28):
    x, y = xy
    d.ellipse([x, y + 8, x + 16, y + 24], fill=color)
    text(d, (x + 26, y), s, sz, False, INK)


class Writer:
    def __init__(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        self.path = OUT / name
        self.w = iio.get_writer(self.path, fps=FPS, codec="libx264", quality=8, macro_block_size=8,
                                ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
        self.n = 0

    def add(self, img):
        self.w.append_data(np.asarray(img))
        self.n += 1

    def close(self):
        self.w.close()
        print(f"-> {self.path} ({self.n / FPS:.1f} s)")


def glow_dot(img, xy, r, color):
    """작은 빛 점(현재 위치 표시)"""
    lay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ld = ImageDraw.Draw(lay)
    x, y = xy
    ld.ellipse([x - r * 2.2, y - r * 2.2, x + r * 2.2, y + r * 2.2], fill=color + (70,))
    lay = lay.filter(ImageFilter.GaussianBlur(r))
    base = img.convert("RGBA")
    base.alpha_composite(lay)
    d = ImageDraw.Draw(base)
    d.ellipse([x - r, y - r, x + r, y + r], fill=color + (255,))
    return base.convert("RGB")
