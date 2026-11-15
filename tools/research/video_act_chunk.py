"""설명 영상 V1 'ACT 묶음 해부': 실물 실행 하나(트윈 모델, 흔들림 갑판 성공)에서 정책이 낸 3.3초 묶음을 보여 준다.

재료: replay_chunks.py가 만든 data/research/chunks_<run>.npz(기록된 화면으로 다시 계산한 묶음), 실행 기록의 앞, 손목 화면.
실행: ~/lerobot_env/bin/python tools/research/video_act_chunk.py [--run 20261110_031451]
결과: ~/Downloads/PlugBot_연구/영상/V1_ACT_묶음_해부.mp4
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import act_offline as A  # noqa: E402
import vidkit as V  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--run", default="20261110_031451")
a = ap.parse_args()
R = A.ROOT / "data/rollouts" / a.run
Z = np.load(A.ROOT / "data/research" / f"chunks_{a.run}.npz")
chunks, st, start, lag = Z["chunks"], Z["state"], int(Z["start"]), int(Z["lag"])
N, CH = len(st), chunks.shape[1]
tip_real = A.tips_from_units(st[:, 1:7]) * 1000
tip_chunk = A.tips_from_units(chunks) * 1000  # (N, 100, 3)
ref = tip_real[-1]
real_xz = np.stack([tip_real[:, 0] - ref[0], tip_real[:, 2] - ref[2]], 1)
chunk_xz = np.stack([tip_chunk[..., 0] - ref[0], tip_chunk[..., 2] - ref[2]], -1)

# 그래프 영역
PX0, PX1 = 760, 1840
PLOTS = [("핀 끝 앞뒤 위치 (mm, 0 = 꽂힌 자리)", 0, 170, 450), ("핀 끝 높이 (mm, 0 = 꽂힌 자리)", 1, 560, 820)]
TMAX = (N + CH) / V.FPS
rng_ = []
for _, j, _, _ in PLOTS:
    lo = min(real_xz[:, j].min(), chunk_xz[::10, :, j].min())
    hi = max(real_xz[:, j].max(), chunk_xz[::10, :, j].max())
    pad = (hi - lo) * 0.08
    rng_.append((lo - pad, hi + pad))


def X(t):
    return PX0 + (PX1 - PX0) * t / TMAX


def Y(v, j):
    lo, hi = rng_[j]
    _, _, y0, y1 = PLOTS[j]
    return y1 - (y1 - y0) * (v - lo) / (hi - lo)


bg = V.canvas()
d = ImageDraw.Draw(bg)
for title, j, y0, y1 in PLOTS:
    V.panel(bg, [PX0 - 30, y0 - 70, PX1 + 30, y1 + 50])
    V.text(d, (PX0, y0 - 56), title, 28, False, V.MUTED)
    for s in range(0, int(TMAX) + 1, 5):
        d.line([X(s), y0, X(s), y1], fill=(34, 48, 74), width=1)
        V.text(d, (X(s), y1 + 12), f"{s}s", 22, False, V.MUTED, "ma")
    zero = Y(0, j)
    d.line([PX0, zero, PX1, zero], fill=(60, 74, 100), width=1)
V.text(d, (90, 22), "실물 실행 기록: 트윈 화면으로만 학습한 ACT", 24, False, V.MUTED)
V.text(d, (90, 50), "(act_twin_v4 1천 스텝), 흔들림 갑판, 성공", 22, False, V.MUTED)


def dashed(dd, pts, color, width=2, on=6, off=6):
    for i in range(0, len(pts) - 1):
        if (i // on) % 2 == 0 if off else True:
            dd.line([pts[i], pts[i + 1]], fill=color, width=width)


def frame_main(k):
    img = bg.copy()
    dd = ImageDraw.Draw(img)
    t = k / V.FPS
    # 카메라
    fr = Image.open(R / "front" / f"{k:05d}.jpg")
    wr = Image.open(R / "wrist" / f"{k:05d}.jpg")
    img.paste(V.fit(fr, 560, 420), (90, 90))
    img.paste(V.fit(wr, 560, 420), (90, 530))
    V.text(dd, (100, 98), "앞 카메라", 24, True, V.INK)
    V.text(dd, (100, 538), "손목 카메라", 24, True, V.INK)
    # 묶음 진행 막대
    b = start + CH * ((k - start) // CH) if k >= start else None
    if b is not None:
        prog = (k - b) / CH
        x0, x1, y = 760, 1840, 52
        dd.rounded_rectangle([x0, y, x1, y + 14], 7, fill=(34, 48, 74))
        dd.rounded_rectangle([x0, y, x0 + max(14, int((x1 - x0) * prog)), y + 14], 7, fill=V.REAL)
        V.text(dd, (x0, y - 40), f"묶음 {1 + (k - start) // CH}번째 실행 중: 다시 볼 때까지 {(CH - (k - b)) / V.FPS:.1f}초", 28, True, V.INK)
        flash = max(0.0, 1 - (k - b) / 20)
        if flash > 0 and k > start:
            V.text(dd, (x1, y - 40), "새 묶음 예측", 28, True, tuple(int(c * flash + bgc * (1 - flash)) for c, bgc in zip(V.HI, V.BG)), "ra")
    for title, j, y0, y1 in PLOTS:
        # 묶음 경계
        for bb in range(start, N, CH):
            if bb <= k:
                dd.line([X(bb / V.FPS), y0, X(bb / V.FPS), y1], fill=(70, 62, 120), width=2)
        # 다시 계획했다면(지금 프레임의 묶음)
        ts = (np.arange(CH) + k + lag + 1) / V.FPS
        ghost = [(X(tt), Y(v, j)) for tt, v in zip(ts, chunk_xz[k, :, j])]
        dashed(dd, ghost, (120, 132, 160), 2)
        # 실행 중인 묶음
        if b is not None:
            ts = (np.arange(CH) + b + lag + 1) / V.FPS
            dd.line([(X(tt), Y(v, j)) for tt, v in zip(ts, chunk_xz[b, :, j])], fill=V.REAL, width=4)
        # 실제 궤적
        pts = [(X(i / V.FPS), Y(real_xz[i, j], j)) for i in range(k + 1)]
        if len(pts) > 1:
            dd.line(pts, fill=V.INK, width=3)
        img = V.glow_dot(img, pts[-1], 7, V.INK)
        dd = ImageDraw.Draw(img)
    # 범례
    V.label(dd, (PX0, 880), "실제로 움직인 궤적", V.INK, 26)
    V.label(dd, (PX0 + 330, 880), "실행 중인 3.3초 묶음", V.REAL, 26)
    V.label(dd, (PX0 + 680, 880), "지금 다시 계획했다면(기록으로 재계산)", (120, 132, 160), 26)
    return img


CUES = [(0.3, 5.5, "ACT는 지금 보이는 화면 두 장과 관절 6개를 보고, 앞으로 100스텝(3.3초)의 동작을 한 번에 냅니다."),
        (5.8, 11.0, "보라 실선이 실행 중인 묶음입니다. 이 모델은 묶음을 끝까지 실행한 뒤에야 다시 봅니다."),
        (11.3, 16.5, "회색 점선은 '지금 다시 계획했다면' 냈을 묶음입니다. 기록된 화면을 정책에 다시 넣어 계산했습니다."),
        (16.8, 22.0, "접근은 7초 안에 끝나고, 그 뒤로는 새 묶음이 시작될 때마다 조금씩 더 밀어 넣습니다(계단 모양)."),
        (22.3, 26.8, "흔들리는 소켓에서 3.3초 동안 눈을 감는 비용은 3장의 묶음 길이 실험에서 잽니다.")]

w = V.Writer("V1_ACT_묶음_해부.mp4")
n_title = 100
for i in range(n_title):
    w.add(V.title_card("1장  모방학습과 ACT", "ACT 묶음 해부", "로봇은 한 번에 몇 초 앞을 계획하고, 그동안 무엇을 보지 않을까?", i, n_title))
for k in range(N):
    s, al = V.caption_at(CUES, k / V.FPS)
    w.add(V.caption(frame_main(k), s, al))
points = ["ACT는 화면과 관절을 보고 3.3초(100스텝) 묶음을 한 번에 낸다.",
          "이 모델은 묶음을 끝까지 실행한 뒤 다시 본다. 시간 앙상블은 쓰지 않았다.",
          "기록된 화면을 다시 넣으면 그때의 계획을 복원할 수 있다(추론이 결정적).",
          "움직이는 표적에서는 '눈 감는 시간'이 비용이 된다. 3장에서 잰다."]
for i in range(130):
    w.add(V.end_card(points, i, 130))
w.close()
