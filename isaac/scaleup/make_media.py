"""[발표용 부가 장면] run_scaleup.py가 남긴 프레임에 자막을 넣어 mp4와 README용 GIF를 만든다.

모든 프레임에 '실제 크기 확장 시각화(Isaac Sim)'와, 학습 정책이나 검증 결과가 아니라는 설명을 넣는다.
실행: ~/plugbot/.venv/bin/python isaac/scaleup/make_media.py [--frames data/scaleup_frames]
결과: outputs/scaleup_wide.mp4, scaleup_close.mp4, scaleup_side_by_side.mp4(mp4는 커밋하지 않음), outputs/scaleup.gif
"""

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "outputs"
FONTS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", "/System/Library/Fonts/AppleSDGothicNeo.ttc"]
PHASE = {"approach": "접근", "gate": "잔잔한 순간 대기", "insert": "삽입", "connected": "연결됨"}
TITLE = "실제 크기 확장 시각화(Isaac Sim)"
LINE1 = "UR10e, 전기 페리 현측 충전 소켓(입구 Ø250 mm, 핀 Ø60 mm), 파고 0.6 m(모형 6 cm × 10, Froude 상사)"
LINE2 = "규칙 기반 예측 제어(학습 정책 아님). 충돌 없는 운동학 재생, 깔때기 포착 가정. 검증 결과가 아니다"


def font(size):
    for f in FONTS:
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def caption(img, t, phase, scale=1.0):
    im = Image.fromarray(img).convert("RGB")
    d = ImageDraw.Draw(im, "RGBA")
    W, H = im.size
    f_t, f_s, f_p = font(int(30 * scale)), font(int(17 * scale)), font(int(24 * scale))
    pad = int(14 * scale)
    tw = d.textlength(TITLE, font=f_t)
    d.rounded_rectangle((pad, pad, pad * 2 + tw, pad * 2 + 38 * scale), 8, fill=(8, 18, 32, 200))
    d.text((pad * 1.5, pad * 1.15), TITLE, font=f_t, fill=(255, 255, 255))
    status = f"{PHASE.get(phase, phase)}  {t:4.1f} s"
    sw = d.textlength(status, font=f_p)
    d.rounded_rectangle((W - sw - pad * 3, pad, W - pad, pad * 2 + 30 * scale), 8,
                        fill=(34, 160, 110, 220) if phase == "connected" else (8, 18, 32, 200))
    d.text((W - sw - pad * 2, pad * 1.2), status, font=f_p, fill=(255, 255, 255))
    bh = int(58 * scale)
    d.rectangle((0, H - bh, W, H), fill=(8, 18, 32, 190))
    d.text((pad, H - bh + 6 * scale), LINE1, font=f_s, fill=(230, 236, 245))
    d.text((pad, H - bh + 30 * scale), LINE2, font=f_s, fill=(255, 205, 140))
    return np.asarray(im)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default=str(ROOT / "data" / "scaleup_frames"))
    ap.add_argument("--fps", type=int, default=30)
    a = ap.parse_args()
    fd = Path(a.frames)
    meta = json.loads((fd / "run.json").read_text())["frames"]
    writers = {k: imageio.get_writer(str(OUT / f"scaleup_{k}.mp4"), fps=a.fps, codec="libx264", quality=8, macro_block_size=8)
               for k in ("wide", "close", "side_by_side")}
    gif = []
    for i, m in enumerate(meta):
        wide = np.asarray(Image.open(fd / "wide" / f"{m['k']:05d}.png").convert("RGB"))
        close = np.asarray(Image.open(fd / "close" / f"{m['k']:05d}.png").convert("RGB"))
        writers["wide"].append_data(caption(wide, m["t"], m["phase"]))
        writers["close"].append_data(caption(close, m["t"], m["phase"]))
        half = [np.asarray(Image.fromarray(x).resize((960, 540))) for x in (wide, close)]
        both = caption(np.concatenate(half, 1), m["t"], m["phase"], scale=1.0)
        writers["side_by_side"].append_data(both)
        if i % 4 == 0:  # GIF: 7.5 fps, 가로 960(README에 넣을 크기)
            gif.append(Image.fromarray(both).resize((960, 270)))
    for w in writers.values():
        w.close()
    pal = [g.convert("P", palette=Image.ADAPTIVE, colors=96) for g in gif]
    pal[0].save(OUT / "scaleup.gif", save_all=True, append_images=pal[1:], duration=133, loop=0, optimize=True)
    for f in ["scaleup_wide.mp4", "scaleup_close.mp4", "scaleup_side_by_side.mp4", "scaleup.gif"]:
        print(f, round((OUT / f).stat().st_size / 1e6, 1), "MB")


if __name__ == "__main__":
    main()
