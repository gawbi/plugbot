"""발표용 1:1:1 비교 영상: 실물 SO-101 원격조종 시범(= ACT 학습 데이터) | 이종 원격조종: 같은 궤적을 실제 크기 UR10e로(Isaac Sim) | 현대차 ACR 공식 영상.

왼쪽과 가운데는 같은 시각(실물 시범의 타임스탬프)으로 맞춘다. 오른쪽은 참고 영상이라 시각을 맞추지 않고 꽂는 구간만 잘라 붙인다.
입력:
  실물: LeRobot 데이터셋 에피소드의 옆면 카메라(front)
  Isaac: run_scaleup.py --replay data/scaleup_real/<..>.npz --frames-dir <폴더> 의 PNG와 run.json
  ACR: data/ref_video/hyundai_acr_official_20230320.mp4(커밋 안 함), 출처 표기
실행: ~/lerobot_env/bin/python tools/make_triptych.py --episode 54 --isaac data/scaleup_real_frames/ep54 [--isaac-cam wide]
결과: outputs/triptych_ep<번호>.mp4(커밋 안 함), 미리보기 outputs/triptych_ep<번호>.png
"""

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument("--episode", type=int, default=54)
ap.add_argument("--dataset", default="plug_static")
ap.add_argument("--isaac", default="", help="run_scaleup.py --frames-dir 폴더. 비우면 가운데 칸은 '렌더 전' 표시")
ap.add_argument("--isaac-cam", default="wide")
ap.add_argument("--acr", default=str(ROOT / "data/ref_video/hyundai_acr_official_20230320.mp4"))
ap.add_argument("--acr-start", type=float, default=16.0, help="ACR 영상에서 로봇이 다가와 꽂기까지 시작 시각(s)")
ap.add_argument("--acr-len", type=float, default=15.0)
ap.add_argument("--fps", type=int, default=30)
args = ap.parse_args()

W, H = 1920, 1080
PW, PH = 600, 450  # 칸(4:3)
GAP = 24
X0 = (W - 3 * PW - 2 * GAP) // 2
Y0 = 250
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def font(size, bold=True):
    try:
        return ImageFont.truetype(FONT if bold else FONT_R, size)
    except OSError:
        return ImageFont.load_default()


F_TITLE, F_SUB, F_LAB, F_SMALL, F_PH = font(54), font(28, False), font(30), font(20, False), font(34)


def fit43(im):
    """가운데를 4:3으로 잘라 칸 크기로."""
    w, h = im.size
    if w / h > 4 / 3:
        nw = int(h * 4 / 3)
        im = im.crop(((w - nw) // 2, 0, (w - nw) // 2 + nw, h))
    else:
        nh = int(w * 3 / 4)
        im = im.crop((0, (h - nh) // 2, w, (h - nh) // 2 + nh))
    return im.resize((PW, PH), Image.LANCZOS)


# 1. 실물 시범(옆면 카메라)과 단계(소켓 기준 궤적에서)
from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402

root = Path.home() / ".cache/huggingface/lerobot/local" / args.dataset
ds = LeRobotDataset(f"local/{args.dataset}", root=root)
e = ds.meta.episodes
a, b = int(e["dataset_from_index"][args.episode]), int(e["dataset_to_index"][args.episode])
real = [Image.fromarray((ds[i]["observation.images.front"].permute(1, 2, 0).numpy() * 255).astype(np.uint8)) for i in range(a, b)]
traj = np.load(ROOT / "data/scaleup_real" / f"{args.dataset}_ep{args.episode}.npz")
dist = np.linalg.norm(traj["rel"], axis=1)
depth = -traj["rel"][:, 2]


def phase(k):
    k = min(k, len(dist) - 1)
    if depth[k] >= 0.025:
        return 3
    if depth[k] > -0.004:
        return 2
    if dist[k] < 0.06:
        return 1
    return 0


PHASES = ["다가가기", "입구 맞추기", "꽂기", "연결"]

# 2. Isaac 렌더(run.json의 프레임 시각)
isaac = []
if args.isaac:
    run = json.loads((Path(args.isaac) / "run.json").read_text())
    rows = run.get("frames") or run.get("meta") or []
    files = sorted((Path(args.isaac) / args.isaac_cam).glob("*.png"))
    ts = [r["t"] for r in rows] if rows and len(rows) == len(files) else [i / args.fps for i in range(len(files))]
    isaac = list(zip(ts, files))


def isaac_at(t):
    if not isaac:
        return None
    k = int(np.searchsorted([x[0] for x in isaac], t, side="right")) - 1
    return Image.open(isaac[max(0, min(k, len(isaac) - 1))][1]).convert("RGB")


# 3. ACR 구간 → 30 fps 프레임
acr_raw = subprocess.run(["ffmpeg", "-v", "error", "-ss", str(args.acr_start), "-t", str(args.acr_len), "-i", args.acr,
                          "-vf", f"fps={args.fps},scale=1280:720", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                         capture_output=True, check=True).stdout
acr = np.frombuffer(acr_raw, np.uint8).reshape(-1, 720, 1280, 3)

# 4. 합치기
n = len(real) + int(2.0 * args.fps)  # 끝에 2초 머무름
BG, FG, MUT, ACC = (14, 18, 26), (240, 244, 248), (150, 160, 175), (255, 138, 31)
out = ROOT / "outputs" / f"triptych_ep{args.episode}.mp4"
enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(args.fps),
                        "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(out)], stdin=subprocess.PIPE)
LABELS = [("원격조종 시범 = ACT 학습 데이터", "SO-101 리더 팔 → 팔로워 팔 (실물, 모형 크기)"),
          ("이종 원격조종: 같은 동작을 실제 크기로", "UR10e + 전기 페리 현측 소켓 (Isaac Sim)"),
          ("영감: 자동차 충전 로봇", "Hyundai Motor Group ACR (공식 영상)")]
preview = None
# 소개 화면(3.5 s): 발표에서 이 영상을 소개하는 말과 같은 순서
INTRO = [("1", "작은 팔(SO-101)로 원격조종해 꽂는 시범을 모은다", "이 시범이 ACT의 학습 데이터"),
         ("2", "같은 동작을 실제 크기 산업용 로봇으로 옮긴다", "이종 원격조종: SO-101 → UR10e, 전기 페리 현측 소켓"),
         ("3", "출발점은 자동차 충전 로봇", "현대차 ACR이 주차된 차에 하던 일을, 흔들리는 배에서")]
for f in range(int(3.5 * args.fps)):
    can = Image.new("RGB", (W, H), BG)
    dr = ImageDraw.Draw(can)
    dr.text((W // 2, 230), "자동차에서 배운 '꽂기'를, 흔들리는 배로", font=F_TITLE, fill=FG, anchor="mm")
    for j, (num, a1, a2) in enumerate(INTRO):
        if f < int((0.4 + 0.8 * j) * args.fps):
            continue
        y = 380 + j * 170
        dr.ellipse((300, y, 380, y + 80), fill=ACC)
        dr.text((340, y + 40), num, font=F_LAB, fill=BG, anchor="mm")
        dr.text((420, y + 2), a1, font=font(40), fill=FG)
        dr.text((420, y + 56), a2, font=F_SUB, fill=MUT)
    enc.stdin.write(np.asarray(can).tobytes())
    if f == int(3.4 * args.fps):
        can.save(ROOT / "outputs" / f"triptych_ep{args.episode}_intro.png")
for f in range(n):
    t = f / args.fps
    k = min(f, len(real) - 1)
    can = Image.new("RGB", (W, H), BG)
    dr = ImageDraw.Draw(can)
    dr.text((X0, 60), "자동차에서 배운 '꽂기'를, 흔들리는 배로", font=F_TITLE, fill=FG)
    dr.text((X0, 140), "작은 팔로 원격조종한 시범이 ACT의 학습 데이터가 되고, 같은 동작은 실제 크기 산업용 로봇으로 옮겨집니다. 왼쪽과 가운데는 같은 시각입니다.", font=F_SUB, fill=MUT)
    ims = [fit43(real[k]), isaac_at(t), fit43(Image.fromarray(acr[min(f, len(acr) - 1)]))]
    for i, (im, (lab, sub)) in enumerate(zip(ims, LABELS)):
        x = X0 + i * (PW + GAP)
        if im is None:
            dr.rectangle((x, Y0, x + PW, Y0 + PH), fill=(30, 36, 48))
            dr.text((x + PW // 2, Y0 + PH // 2), "Isaac Sim 렌더 전", font=F_PH, fill=MUT, anchor="mm")
        else:
            can.paste(im if im.size == (PW, PH) else fit43(im), (x, Y0))
        dr.rectangle((x, Y0, x + PW, Y0 + PH), outline=(ACC if i < 2 else (90, 100, 115)), width=3)
        dr.text((x, Y0 + PH + 18), lab, font=F_LAB, fill=FG)
        dr.text((x, Y0 + PH + 60), sub, font=F_SMALL, fill=MUT)
    # 단계 막대(왼쪽 두 칸 아래): 실물 궤적에서 계산
    ph = phase(k)
    bx, by, bw = X0, Y0 + PH + 110, 2 * PW + GAP
    seg = bw // 4
    for j, name in enumerate(PHASES):
        on = j == ph
        dr.rounded_rectangle((bx + j * seg + 4, by, bx + (j + 1) * seg - 4, by + 46), 10,
                             fill=(ACC if on else (36, 42, 56)))
        dr.text((bx + j * seg + seg // 2, by + 23), name, font=F_SMALL, fill=(BG if on else MUT), anchor="mm")
    dr.text((bx + bw + GAP, by + 10), f"{t:5.1f} s", font=F_SUB, fill=FG)
    dr.text((X0, H - 70), "가운데: 왼쪽 원격조종 시범의 손끝 궤적(소켓 기준)을 실제 크기 UR10e로 옮겨 재생한 시뮬레이션입니다(녹화한 시범의 재생, 실시간 조종 아님).  "
            "오른쪽 출처: Hyundai Motor Group, \"Newly Developed Automatic Charging Robot (ACR)\" (2023)", font=font(17, False), fill=MUT)
    enc.stdin.write(np.asarray(can).tobytes())
    if preview is None and ph == 2:
        preview = can.copy()
enc.stdin.close()
enc.wait()
(preview or can).save(ROOT / "outputs" / f"triptych_ep{args.episode}.png")
print(out, f"{n / args.fps + 3.5:.1f} s")
