"""[발표용 부가 장면] run_scaleup.py가 남긴 프레임에 자막을 넣어 mp4와 README용 GIF를 만든다.

모든 프레임에 '실제 크기 확장 시각화(Isaac Sim)'와, 학습 정책이나 검증 결과가 아니라는 설명을 넣는다.
실행: ~/plugbot/.venv/bin/python isaac/scaleup/make_media.py [--frames data/scaleup_frames]
--frames data/scaleup_replay(run_scaleup.py --replay 결과)면 이종 원격조종 영상 outputs/scaleup_teleop.mp4, .gif를 만든다.
결과: outputs/scaleup_edit.mp4(바다 → 넓은 → 근접 → 넓은 편집본), scaleup_{sea,wide,close,side_by_side}.mp4
      (mp4는 커밋하지 않음), outputs/scaleup.gif(편집본)
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
PHASE = {"approach": "접근", "gate": "잔잔한 순간 대기", "insert": "삽입", "connected": "연결됨", "teleop": "원격조종"}
TITLE = "실제 크기 확장 시각화(Isaac Sim)"
LINE1 = "UR10e, 전기 페리 현측 충전 소켓(입구 Ø250 mm, 핀 Ø60 mm), 파고 0.6 m(모형 6 cm × 10, Froude 상사)"
LINE2 = "규칙 기반 예측 제어(학습 정책 아님). 충돌 없는 운동학 재생, 깔때기 포착 가정. 검증 결과가 아니다"


def font(size):
    for f in FONTS:
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


TELEOP = ("이종 원격조종 시각화(Isaac Sim)",
          "왼쪽 SO-101(모형 1:10) 시범 궤적을 소켓 기준 상대 위치로 옮겨(소켓 근처 ×5) 오른쪽 UR10e(실제 크기 페리 소켓)가 따른다",
          "녹화된 시뮬레이션 시범 재생(실시간 리더 팔 아님). 소켓 위치는 시뮬레이션 참값(실제는 카메라로 재야 함). 충돌 없는 운동학, 깔때기 포착 가정")


def caption(img, t, phase, scale=1.0, text=None):
    im = Image.fromarray(img).convert("RGB")
    d = ImageDraw.Draw(im, "RGBA")
    W, H = im.size
    f_t, f_s, f_p = font(int(30 * scale)), font(int(17 * scale)), font(int(24 * scale))
    pad = int(14 * scale)
    title, line1, line2 = text or (TITLE, LINE1, LINE2)
    tw = d.textlength(title, font=f_t)
    d.rounded_rectangle((pad, pad, pad * 2 + tw, pad * 2 + 38 * scale), 8, fill=(8, 18, 32, 200))
    d.text((pad * 1.5, pad * 1.15), title, font=f_t, fill=(255, 255, 255))
    status = f"{PHASE.get(phase, phase)}  {t:4.1f} s"
    sw = d.textlength(status, font=f_p)
    d.rounded_rectangle((W - sw - pad * 3, pad, W - pad, pad * 2 + 30 * scale), 8,
                        fill=(34, 160, 110, 220) if phase == "connected" else (8, 18, 32, 200))
    d.text((W - sw - pad * 2, pad * 1.2), status, font=f_p, fill=(255, 255, 255))
    bh = int(58 * scale)
    d.rectangle((0, H - bh, W, H), fill=(8, 18, 32, 190))
    d.text((pad, H - bh + 6 * scale), line1, font=f_s, fill=(230, 236, 245))
    d.text((pad, H - bh + 30 * scale), line2, font=f_s, fill=(255, 205, 140))
    return np.asarray(im)


def shot_plan(meta):
    """편집본의 장면 순서: 바다(전체 상황) → 넓은 시점(접근, 대기) → 근접(삽입 0.5 s 전부터 연결 1.5 s 뒤까지) → 넓은 시점."""
    t_ins = next((m["t"] for m in meta if m["phase"] == "insert"), None)
    t_con = next((m["t"] for m in meta if m["phase"] == "connected"), None)
    plan = []
    for m in meta:
        if m["t"] < 3.5:
            plan.append("sea")
        elif t_ins is not None and t_ins - 0.5 <= m["t"] and (t_con is None or m["t"] <= t_con + 1.5):
            plan.append("close")
        else:
            plan.append("wide")
    return plan


def label(img, text, f):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im, "RGBA")
    w = d.textlength(text, font=f)
    d.rounded_rectangle((12, 70, 28 + w, 108), 8, fill=(8, 18, 32, 200))
    d.text((20, 72), text, font=f, fill=(255, 255, 255))
    return np.asarray(im)


def teleop(fd, run, fps):
    """이종 원격조종 영상: 왼쪽 SO-101 시범(앞 카메라), 오른쪽 UR10e(넓은 시점, 삽입부터 근접)."""
    meta = run["frames"]
    so = [np.asarray(Image.fromarray(f).resize((960, 720))) for f in imageio.get_reader(str(Path(run["replay"]) / "front.mp4"))]
    f_l = font(24)
    w = imageio.get_writer(str(OUT / "scaleup_teleop.mp4"), fps=fps, codec="libx264", quality=8, macro_block_size=8)
    gif = []
    for i, m in enumerate(meta):
        left = label(so[min(m.get("so101_frame", 0), len(so) - 1)], "입력: SO-101 (모형 1:10)", f_l)
        view = "close" if m["phase"] in ("insert", "connected") else "wide"
        right = label(np.asarray(Image.open(fd / view / f"{m['k']:05d}.png").convert("RGB")), "출력: UR10e (실제 크기)", f_l)
        frame = caption(np.concatenate([left, right], 1), m["t"], m["phase"], text=TELEOP)
        w.append_data(frame)
        if i % 4 == 0:
            gif.append(Image.fromarray(frame).resize((896, 288)))
    w.close()
    pal = [g.convert("P", palette=Image.ADAPTIVE, colors=96) for g in gif]
    pal[0].save(OUT / "scaleup_teleop.gif", save_all=True, append_images=pal[1:], duration=133, loop=0, optimize=True)
    for f in ["scaleup_teleop.mp4", "scaleup_teleop.gif"]:
        print(f, round((OUT / f).stat().st_size / 1e6, 1), "MB")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default=str(ROOT / "data" / "scaleup_frames"))
    ap.add_argument("--fps", type=int, default=30)
    a = ap.parse_args()
    fd = Path(a.frames)
    run = json.loads((fd / "run.json").read_text())
    if run.get("replay"):
        teleop(fd, run, a.fps)
        return
    meta = json.loads((fd / "run.json").read_text())["frames"]
    views = [v for v in ("sea", "wide", "close") if (fd / v).exists()]
    names = ["edit", "side_by_side"] + views
    writers = {k: imageio.get_writer(str(OUT / f"scaleup_{k}.mp4"), fps=a.fps, codec="libx264", quality=8, macro_block_size=8)
               for k in names}
    plan = shot_plan(meta) if "sea" in views else ["wide"] * len(meta)
    gif = []
    for i, m in enumerate(meta):
        img = {v: np.asarray(Image.open(fd / v / f"{m['k']:05d}.png").convert("RGB")) for v in views}
        for v in views:
            writers[v].append_data(caption(img[v], m["t"], m["phase"]))
        edit = caption(img[plan[i]], m["t"], m["phase"])
        writers["edit"].append_data(edit)
        half = [np.asarray(Image.fromarray(img[v]).resize((960, 540))) for v in ("wide", "close")]
        writers["side_by_side"].append_data(caption(np.concatenate(half, 1), m["t"], m["phase"]))
        if i % 4 == 0:  # GIF: 편집본, 7.5 fps, 가로 720(README에 넣을 크기)
            gif.append(Image.fromarray(edit).resize((720, 405)))
    for w in writers.values():
        w.close()
    pal = [g.convert("P", palette=Image.ADAPTIVE, colors=96) for g in gif]
    pal[0].save(OUT / "scaleup.gif", save_all=True, append_images=pal[1:], duration=133, loop=0, optimize=True)
    for f in [f"scaleup_{k}.mp4" for k in names] + ["scaleup.gif"]:
        print(f, round((OUT / f).stat().st_size / 1e6, 1), "MB")


if __name__ == "__main__":
    main()
