"""시뮬레이션 영상 묶기: run_plugbot_isaac.py --video 결과(또는 같은 이름의 MuJoCo 영상)로 발표, README용 파일을 만든다.

- outputs/showreel_{layout}_hs{H}_compare.mp4: 추종(왼쪽)과 예측(오른쪽) ShowCam 시점을 나란히(같은 파도 시드)
- outputs/showreel_{layout}_hs{H}_views.mp4: 예측 전략의 Webcam | ShowCam | Wrist 세 시점
- outputs/showreel_{layout}_hs{H}_compare.gif: README용(가로 960, 12 fps, 8 MB 안팎)

실행: python tools/make_showreel.py [--layout side --hs 6]   (imageio, imageio-ffmpeg, pillow 필요)
"""

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parents[1] / "outputs"
FONTS = ["/System/Library/Fonts/AppleSDGothicNeo.ttc", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
         "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc"]


def font(size):
    for f in FONTS:
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()  # 한글 글꼴이 없으면 영어 라벨만 깨끗하다(우분투: sudo apt install fonts-nanum)


def frames(path):
    r = imageio.get_reader(str(path))
    try:
        for f in r:
            yield f
    finally:
        r.close()


def label(img, text, h=44, size=26):
    im = Image.fromarray(img)
    canvas = Image.new("RGB", (im.width, im.height + h), "white")
    canvas.paste(im, (0, h))
    ImageDraw.Draw(canvas).text((12, 8), text, font=font(size), fill=(20, 20, 20))
    return np.asarray(canvas)


def side_by_side(paths, labels, out, fps, width=None):
    gens = [frames(p) for p in paths]
    last = [None] * len(gens)
    w = imageio.get_writer(str(out), fps=fps, codec="libx264", quality=8, macro_block_size=1)
    gif = []
    while True:
        alive = False
        for i, g in enumerate(gens):
            try:
                last[i] = next(g)
                alive = True
            except StopIteration:
                pass  # 먼저 끝난 쪽은 마지막 장면을 유지
        if not alive:
            break
        h = min(f.shape[0] for f in last)
        tiles = [label(np.asarray(Image.fromarray(f).resize((int(f.shape[1] * h / f.shape[0]), h))), lab)
                 for f, lab in zip(last, labels)]
        row = np.concatenate([np.pad(t, ((0, 0), (0, 8), (0, 0)), constant_values=255) for t in tiles], axis=1)[:, :-8]
        w.append_data(row)
        if width:
            gif.append(Image.fromarray(row).resize((width, int(row.shape[0] * width / row.shape[1]))))
    w.close()
    print(out)
    return gif


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default="side")
    ap.add_argument("--hs", type=int, default=6, help="cm")
    ap.add_argument("--fps", type=int, default=25)
    a = ap.parse_args()
    tag = f"{a.layout}_hs{a.hs}"
    vid = {s: {c: OUT / f"isaac_video_{tag}_{s}_{c}.mp4" for c in ("Webcam", "ShowCam", "Wrist")}
           for s in ("reactive", "predictive")}
    if all(vid[s]["ShowCam"].exists() for s in vid):
        gif = side_by_side([vid["reactive"]["ShowCam"], vid["predictive"]["ShowCam"]],
                           ["추종: 지연된 위치를 따라감", "예측: AR 예측 + 정온 구간"],
                           OUT / f"showreel_{tag}_compare.mp4", a.fps, width=960)
        g = gif[::2]  # 12 fps
        g[0].save(OUT / f"showreel_{tag}_compare.gif", save_all=True, append_images=g[1:], duration=int(2000 / a.fps),
                  loop=0, optimize=True)
        print(OUT / f"showreel_{tag}_compare.gif")
    else:
        print("추종, 예측 ShowCam 영상이 둘 다 있어야 비교 영상을 만든다:", vid["reactive"]["ShowCam"].name)
    views = [p for c, p in vid["predictive"].items() if p.exists()]
    if views:
        side_by_side(views, [p.stem.split("_")[-1] for p in views], OUT / f"showreel_{tag}_views.mp4", a.fps)


if __name__ == "__main__":
    main()
