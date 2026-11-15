"""설명 영상 V2 '같은 관절, 다른 픽셀': 디지털 트윈이 무엇을 바꾸고 무엇을 그대로 두는지.

1) 학습에 안 쓴 실물 에피소드(plug_static 81)와, 같은 관절로 다시 그린 트윈(보정 후)을 닦아내기로 비교
2) 꽂힌 순간: 실물 위에 보정 전, 보정 후 트윈을 겹쳐 보기(손목, 앞)
3) 반전: 배 위 시험 때 앞 카메라는 녹화 때와 다른 시점이었다. 정책은 어느 카메라에 기댔나(camera_reliance.json)
실행: ~/lerobot_env/bin/python tools/research/video_twin.py
결과: ~/Downloads/PlugBot_연구/영상/V2_같은관절_다른픽셀.mp4
"""

import json
import sys
from pathlib import Path

import imageio.v2 as iio
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vidkit as V  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
EP = 81
DS = Path.home() / ".cache/huggingface/lerobot/local/plug_static"
meta = pd.concat([pd.read_parquet(f) for f in sorted((DS / "meta/episodes").rglob("*.parquet"))]).set_index("episode_index")


def real_frames(cam):
    m = meta.loc[EP]
    f = DS / f"videos/observation.images.{cam}/chunk-{int(m[f'videos/observation.images.{cam}/chunk_index']):03d}/file-{int(m[f'videos/observation.images.{cam}/file_index']):03d}.mp4"
    off, n = int(round(float(m[f"videos/observation.images.{cam}/from_timestamp"]) * 30)), int(m["length"])
    r = iio.get_reader(f)
    out = [np.asarray(r.get_data(off + k)) for k in range(n)]
    r.close()
    return out


def twin(tag, cam):
    r = iio.get_reader(ROOT / f"data/twin_render_holdout_{tag}/ep_{EP:03d}_n_{cam}.mp4")
    out = [np.asarray(im) for im in r]
    r.close()
    return out


RF, RW = real_frames("front"), real_frames("wrist")
CF, CW = twin("calib", "front"), twin("calib", "wrist")
NF, NW = twin("nocalib", "front"), twin("nocalib", "wrist")
T = len(RF)
REL = json.loads((ROOT / "outputs/research/camera_reliance.json").read_text())
SHIFT = Image.open(ROOT / "outputs/research/front_view_shift.png")

w = V.Writer("V2_같은관절_다른픽셀.mp4")
for i in range(100):
    w.add(V.title_card("2장  디지털 트윈과 Sim-to-Real", "같은 관절, 다른 픽셀", "실물 사진 없이 배운 정책이 실물에서 통하려면, 트윈 화면이 무엇을 닮아야 할까?", i, 100))

# 1) 닦아내기
CUES1 = [(0.2, 5.0, "왼쪽은 실물 카메라, 오른쪽은 같은 순간의 관절값으로 Isaac Sim에서 다시 그린 트윈입니다."),
         (5.2, 10.0, "관절 상태와 동작은 실물 시범 그대로이고, 바뀌는 것은 화면(픽셀)뿐입니다. 경계선이 둘을 오갑니다."),
         (10.2, 13.2, "이 에피소드(81번)는 트윈 모델이 학습에 쓰지 않은 것입니다.")]
for k in range(T):
    t = k / V.FPS
    img = V.canvas()
    d = ImageDraw.Draw(img)
    x = 0.5 + 0.42 * np.sin(2 * np.pi * t / 4.0)
    for (rf, cf, title), (X0, Y0) in zip(((RF[k], CF[k], "앞 카메라"), (RW[k], CW[k], "손목 카메라")), ((120, 170), (1000, 170))):
        r = Image.fromarray(rf).resize((800, 600))
        c = Image.fromarray(cf).resize((800, 600))
        cut = int(800 * x)
        comp = Image.new("RGB", (800, 600))
        comp.paste(r.crop((0, 0, cut, 600)), (0, 0))
        comp.paste(c.crop((cut, 0, 800, 600)), (cut, 0))
        img.paste(comp, (X0, Y0))
        d.line([X0 + cut, Y0, X0 + cut, Y0 + 600], fill=V.HI, width=4)
        V.text(d, (X0, Y0 - 50), title, 32, True, V.INK)
        V.text(d, (X0 + 10, Y0 + 10), "실물", 28, True, V.REAL)
        V.text(d, (X0 + 790, Y0 + 10), "트윈(보정 후)", 28, True, V.SIM, "ra")
    s, al = V.caption_at(CUES1, t)
    w.add(V.caption(img, s, al))

# 2) 겹쳐 보기: 꽂힌 순간
k = T - 1
CUES2 = [(0.2, 4.6, "꽂힌 순간에 실물(보라)과 트윈(청록)을 겹쳐 봅니다. 먼저 보정 전 카메라입니다."),
         (4.8, 9.6, "카메라 위치와 각도를 실물 화면에 맞춰 다시 보정하면, 소켓과 덮개가 실물과 더 가깝게 겹칩니다."),
         (9.8, 14.5, "정책 눈으로도 그렇습니다. 같은 순간 실물 화면과 트윈 화면을 넣었을 때 동작 차이가 보정 후 쪽이 더 작았습니다(체크포인트 14개 모두).")]


def tint(im, col):
    a = np.asarray(Image.fromarray(im).convert("L").resize((720, 540))).astype(float) / 255
    return np.stack([a * c for c in col], -1)


for i in range(int(14.5 * V.FPS)):
    t = i / V.FPS
    calib = t > 4.7
    img = V.canvas()
    d = ImageDraw.Draw(img)
    for (rr, cc, nn, title), X0 in (((RW[k], CW[k], NW[k], "손목 카메라"), 160), ((RF[k], CF[k], NF[k], "앞 카메라"), 1040)):
        tw = cc if calib else nn
        mix = np.clip(tint(rr, (166, 139, 255)) * 0.62 + tint(tw, (92, 184, 200)) * 0.62, 0, 255).astype(np.uint8)
        img.paste(Image.fromarray(mix), (X0, 200))
        V.text(d, (X0, 140), title, 32, True, V.INK)
    V.text(d, (V.W // 2, 760), "보정 후" if calib else "보정 전", 54, True, V.HI if calib else V.MUTED, "ma")
    V.label(d, (160, 770), "실물", V.REAL, 28)
    V.label(d, (300, 770), "트윈", V.SIM, 28)
    s, al = V.caption_at(CUES2, t)
    w.add(V.caption(img, s, al))

# 3) 반전: 시험 때 앞 카메라 시점
CUES3 = [(0.2, 5.6, "그런데 기록을 다시 보니, 배 위 시험 때 앞 카메라는 녹화 때와 다른 자리(시점 B)에 있었습니다."),
         (5.8, 11.2, "두 모델 모두 처음 보는 시점에서 꽂았습니다. 시험 입력에서 카메라를 하나씩 가려 보면 이유가 보입니다."),
         (11.4, 17.0, "트윈 모델은 손목 카메라를 가릴 때 동작이 앞 카메라를 가릴 때의 두 배로 바뀝니다. 앞 화면을 45 px 밀어도 거의 그대로입니다.")]
rr, tt = REL["act_real_v4/080000"], REL["act_twin_v4/001000"]
bars = [("앞 카메라 가림", rr["front_gray"][0], tt["front_gray"][0]), ("손목 카메라 가림", rr["wrist_gray"][0], tt["wrist_gray"][0]),
        ("앞 화면 45 px 이동", rr["front_up45"][0], tt["front_up45"][0])]
for i in range(int(17 * V.FPS)):
    t = i / V.FPS
    img = V.canvas()
    d = ImageDraw.Draw(img)
    if t < 5.7:
        img.paste(V.fit(SHIFT, 1500, 830), (210, 90))
    else:
        p = V.ease((t - 5.8) / 2.0)
        V.text(d, (200, 110), "배 위 시험 입력에서 카메라를 바꾸면 1초 뒤 동작이 얼마나 바뀌나 (핀 끝 mm)", 36, True, V.INK)
        for j, (lab, a, b) in enumerate(bars):
            y = 250 + j * 210
            V.text(d, (200, y + 20), lab, 34, False, V.INK)
            for kk, (val, col, nm) in enumerate(((a, V.REAL, "실물 화면 모델"), (b, V.SIM, "트윈 모델"))):
                L = int(30 * val * p)
                d.rounded_rectangle([620, y + kk * 70, 620 + max(8, L), y + kk * 70 + 50], 10, fill=col)
                V.text(d, (640 + L, y + kk * 70 + 6), f"{val * p:.0f} mm  {nm}", 30, False, V.INK)
    s, al = V.caption_at(CUES3, t)
    w.add(V.caption(img, s, al))

points = ["트윈은 관절과 동작은 실물 그대로 두고 화면만 바꾼다. 실물 관절 궤적 69개는 썼다.",
          "카메라 보정 뒤 트윈 화면은 정책 눈에도 실물에 더 가깝다(같은 순간 동작 차이가 작다).",
          "시험 때 앞 카메라 시점은 녹화 때와 달랐다. 트윈 모델은 손목 카메라에 더 기댔다.",
          "최종 모델은 보정과 짧은 학습이 함께 바뀐 것이라, 원인 분리는 대조 학습으로 확인한다."]
for i in range(140):
    w.add(V.end_card(points, i, 140))
w.close()
