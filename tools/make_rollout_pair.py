"""로봇 혼자 꽂기(ACT) 실행 | 같은 관절 기록을 실제 크기 UR10e로(Isaac Sim) 나란히 영상.

순서(act/rollout_pair.sh가 한 번에 한다):
  1. isaac/scaleup/rollout_traj.py <실행>       → data/scaleup_real/rollout_<실행>.npz
  2. run_scaleup.py --replay <npz> --frames-dir data/scaleup_real_frames/rollout_<실행>
  3. 이 스크립트                                   → outputs/rollout_pair_<실행>.mp4 와 .png(미리보기)
왼쪽: data/rollouts/<실행>/front(옆면 카메라, 실행 중 저장), 오른쪽: Isaac 근접 시점. 두 칸은 같은 시각(실행 기록의 시각).
실행: ~/lerobot_env/bin/python tools/make_rollout_pair.py 20261106_191627
"""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
run = sys.argv[1]
cam = sys.argv[2] if len(sys.argv) > 2 else "close"
FPS = 30
W, H = 1920, 1080
PW, PH = 880, 660
GAP = 40
X0 = (W - 2 * PW - GAP) // 2
Y0 = 190
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BG, FG, MUT, ACC, LINE = (12, 15, 20), (236, 240, 244), (140, 150, 162), (255, 150, 40), (60, 70, 82)


def font(size, bold=True):
    return ImageFont.truetype(FONT if bold else FONT_R, size)


def fit43(im):
    w, h = im.size
    if w / h > 4 / 3:
        nw = int(h * 4 / 3)
        im = im.crop(((w - nw) // 2, 0, (w - nw) // 2 + nw, h))
    else:
        nh = int(w * 3 / 4)
        im = im.crop((0, (h - nh) // 2, w, (h - nh) // 2 + nh))
    return im.resize((PW, PH), Image.LANCZOS)


d_run = ROOT / "data" / "rollouts" / run
S = np.load(d_run / "state.npy")
t_real = S[:, 0] - S[0, 0]
fronts = sorted((d_run / "front").glob("*.jpg"))
info = json.loads((ROOT / "data" / "scaleup_real" / f"rollout_{run}.json").read_text())
traj = np.load(ROOT / "data" / "scaleup_real" / f"rollout_{run}.npz")
dist, depth = np.linalg.norm(traj["rel"], axis=1), -traj["rel"][:, 2]
side = np.linalg.norm(traj["rel"][:, :2], axis=1)  # 축에서 옆으로 벗어난 거리. 크면 소켓 옆을 찌른 것
fr = ROOT / "data" / "scaleup_real_frames" / f"rollout_{run}"
rj = json.loads((fr / "run.json").read_text())
files = sorted((fr / cam).glob("*.png"))
rows = rj.get("frames") or []
t_isaac = np.array([r["t"] for r in rows] if len(rows) == len(files) else [i / FPS for i in range(len(files))])

ck = "/".join(Path(info["policy"]).parts[-4::2]) if info.get("policy") else "?"
judged = info.get("judged") or "판정 없음"
cond = {"static": "정지 소켓", "moving": "흔들림 갑판"}.get(info.get("cond"), "")
PHASES = ["다가가기", "입구 맞추기", "꽂기", "연결"]


def phase(k):
    k = min(k, len(dist) - 1)
    if side[k] > 0.006:  # 입구 반지름(모형 약 6 mm) 밖이면 '꽂기'나 '연결'로 표시하지 않는다
        return 1 if dist[k] < 0.06 else 0
    return 3 if depth[k] >= 0.025 else 2 if depth[k] > -0.004 else 1 if dist[k] < 0.06 else 0


def at(ts, t):
    return int(max(0, min(np.searchsorted(ts, t, side="right") - 1, len(ts) - 1)))


T = float(traj["t"][-1])
n = int((T + 1.5) * FPS)
out = ROOT / "outputs" / f"rollout_pair_{run}.mp4"
enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                        "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-movflags", "+faststart", str(out)],
                       stdin=subprocess.PIPE)
F_T, F_S, F_L, F_X = font(46), font(24, False), font(28), font(18, False)
preview = None
for f in range(n):
    t = min(f / FPS, T)
    can = Image.new("RGB", (W, H), BG)
    dr = ImageDraw.Draw(can)
    dr.text((X0, 50), "작은 팔이 혼자 꽂은 동작을, 실제 크기 산업용 로봇으로", font=F_T, fill=FG)
    dr.text((X0, 118), f"같은 시각, 같은 관절 기록   |   {ck}   |   {cond} {judged}", font=F_S, fill=MUT)
    left = fit43(Image.open(fronts[min(at(t_real, t), len(fronts) - 1)]).convert("RGB"))
    right = fit43(Image.open(files[at(t_isaac, t)]).convert("RGB"))
    for i, (im, lab, sub) in enumerate(((left, "실물 SO-101, 로봇 혼자 (ACT)", "모형 크기, 옆면 카메라, 실제 속도"),
                                        (right, "UR10e, 전기 페리 현측 소켓", "Isaac Sim, 커넥터 5배, 충돌 계산 없는 재생"))):
        x = X0 + i * (PW + GAP)
        can.paste(im, (x, Y0))
        dr.rectangle((x - 1, Y0 - 1, x + PW, Y0 + PH), outline=LINE, width=2)
        dr.text((x, Y0 + PH + 16), lab, font=F_L, fill=FG)
        dr.text((x, Y0 + PH + 56), sub, font=F_X, fill=MUT)
    ph = phase(at(traj["t"], t))
    bx, by, bw = X0, Y0 + PH + 100, 2 * PW + GAP - 160
    seg = bw // 4
    for j, name in enumerate(PHASES):
        on = j == ph
        dr.rectangle((bx + j * seg + 3, by, bx + (j + 1) * seg - 3, by + 6), fill=ACC if on else LINE)
        dr.text((bx + j * seg + 3, by + 14), name, font=F_X, fill=FG if on else MUT)
    dr.text((X0 + 2 * PW + GAP, by + 2), f"{t:5.1f} s", font=F_L, fill=FG, anchor="ra")
    dr.text((X0, H - 44), "오른쪽은 왼쪽 실행의 관절 기록을 소켓 기준으로 옮겨 실제 크기 UR10e로 재생한 시각화입니다(실시간 연동 아님). "
            f"소켓 위치: {info['socket']}.", font=font(16, False), fill=MUT)
    enc.stdin.write(np.asarray(can).tobytes())
    if preview is None and ph == 2:
        preview = can.copy()
enc.stdin.close()
enc.wait()
(preview or can).save(out.with_suffix(".png"))
print(out, f"{n / FPS:.1f} s")
