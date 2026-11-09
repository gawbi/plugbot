"""포토박스 트윈 3단계: 렌더 영상 + 실물 상태와 동작 → LeRobot 데이터셋.

화면(front, wrist)은 Isaac 트윈 렌더만 쓴다(실물 사진은 한 장도 넣지 않는다). 상태와 동작은 실물 시범 값 그대로(LeRobot 눈금).
이름과 형식은 실물 데이터셋(plug_static)과 같다: observation.state, action(6), observation.images.front, .wrist(480x640).

렌더 폴더 형식 두 가지를 읽는다: ep_XXX_{front,wrist}.mp4(예전), ep_XXX_{n,h}_{front,wrist}.mp4(보통, 가혹 판).
가혹 조건 후처리(--aug): 가혹 판은 1~3가지, 보통 판은 30% 확률로 1가지.
  blur 초점 흐림, motion 흔들림 번짐, noise 센서 노이즈, jpeg 압축 깨짐, flicker 노출 출렁임,
  fog 안개(대비 저하), drops 렌즈 물방울, latency 화면이 관절보다 1~3장면 늦음(실물 카메라 지연 흉내)
  shake 카메라 흔들림(가혹 판 60%, 보통 판 25%. 2~6 Hz, 3~12 px, 0.3~1.5도, 손목은 1.5배)
기록: <데이터셋>/meta/twin_aug.json (에피소드마다 판, Isaac 조명/재질 모드, 후처리)
실행: ~/lerobot_env/bin/python isaac/twin/build_dataset.py --render data/twin_render2 --repo plug_twin_v2 --aug
"""

import argparse
import io
import json
import shutil
from pathlib import Path

import imageio.v2 as iio
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from PIL import Image, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser()
ap.add_argument("--src", default=str(ROOT / "data/twin_src"))
ap.add_argument("--render", default=str(ROOT / "data/twin_render"))
ap.add_argument("--repo", default="plug_twin")
ap.add_argument("--aug", action="store_true")
ap.add_argument("--seed", type=int, default=1000, help="후처리 무작위 시드 시작(데이터셋마다 다르게)")
a = ap.parse_args()

EFFECTS = ["blur", "motion", "noise", "jpeg", "flicker", "fog", "drops", "latency"]
SHAKE_P = {"h": 0.6, "n": 0.25}  # 카메라 흔들림을 넣을 확률(가혹 판, 보통 판). 다른 효과와 따로 뽑는다


def make_aug(rng, harsh):
    if not a.aug:
        return {}
    if harsh:
        names = list(rng.choice(EFFECTS, size=int(rng.integers(1, 4)), replace=False))
    elif rng.random() < 0.3:
        names = [str(rng.choice(EFFECTS))]
    else:
        names = []
    p = {}
    if rng.random() < SHAKE_P["h" if harsh else "n"]:  # 카메라 흔들림: 진폭(px), 돌림(도), 떨림 주파수(Hz), 위상
        p["shake"] = [float(rng.uniform(3, 12)), float(rng.uniform(0.3, 1.5)), float(rng.uniform(2, 6)), *rng.uniform(0, 6.28, 3).tolist()]
    for nm in names:
        if nm == "blur":
            p[nm] = float(rng.uniform(0.6, 2.0))
        elif nm == "motion":
            p[nm] = int(rng.choice([5, 7, 9, 11]))
        elif nm == "noise":
            p[nm] = float(rng.uniform(4, 12))
        elif nm == "jpeg":
            p[nm] = int(rng.integers(15, 40))
        elif nm == "flicker":
            p[nm] = float(rng.uniform(0.08, 0.2))
        elif nm == "fog":
            p[nm] = float(rng.uniform(0.15, 0.4))
        elif nm == "drops":
            p[nm] = int(rng.integers(5, 15))
        elif nm == "latency":
            p[nm] = int(rng.integers(1, 4))
    return p


def drop_mask(rng, n, w=640, h=480):
    """렌즈 물방울: 반투명하고 번진 동그라미들(밝기 조금 올리고 흐림)"""
    m = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(m)
    for _ in range(n):
        r = int(rng.uniform(8, 40))
        x, y = int(rng.uniform(0, w)), int(rng.uniform(0, h))
        d.ellipse([x - r, y - r, x + r, y + r], fill=int(rng.uniform(120, 230)))
    return m.filter(ImageFilter.GaussianBlur(4))


def shake(im, sh, k, cam):
    """카메라 흔들림: 장면마다 조금씩 밀리고 돌아간다(손목은 팔 떨림이라 1.5배). 가장자리가 비지 않게 10% 확대"""
    amp, rot, f, a1, a2, a3 = sh
    amp, rot = (amp * 1.5, rot * 1.5) if cam == "wrist" else (amp, rot)
    t = k / 30.0
    w = 2 * np.pi * f * t
    dx = amp * (np.sin(w + a1) + 0.35 * np.sin(2.3 * w + a2))
    dy = amp * (np.sin(1.3 * w + a2) + 0.35 * np.sin(2.9 * w + a3))
    r = rot * np.sin(0.9 * w + a3)
    W, H = im.size
    im = im.rotate(r, resample=Image.BILINEAR, translate=(dx, dy))
    m = 0.05
    return im.crop((int(W * m), int(H * m), int(W * (1 - m)), int(H * (1 - m)))).resize((W, H), Image.BILINEAR)


def apply(img, p, rng, k, drops, cam="front"):
    im = Image.fromarray(img)
    if "shake" in p:
        im = shake(im, p["shake"], k, cam)
    if "blur" in p:
        im = im.filter(ImageFilter.GaussianBlur(p["blur"]))
    if "motion" in p:
        n = p["motion"]
        ker = np.zeros((n, n)); ker[n // 2, :] = 1.0
        im = im.filter(ImageFilter.Kernel((n, n), ker.ravel().tolist(), scale=n)) if n <= 5 else im.filter(ImageFilter.BoxBlur(n // 3))
    if drops is not None:
        blurred = im.filter(ImageFilter.GaussianBlur(6))
        bright = Image.eval(blurred, lambda v: min(255, int(v * 1.15 + 10)))
        im = Image.composite(bright, im, drops)
    x = np.asarray(im).astype(np.float32)
    if "fog" in p:
        x = x * (1 - p["fog"]) + 175.0 * p["fog"]
    if "flicker" in p:
        x = x * (1 + p["flicker"] * np.sin(k * 0.37 + rng.uniform(0, 0.3)))
    if "noise" in p:
        x = x + rng.normal(0, p["noise"], x.shape)
    x = np.clip(x, 0, 255).astype(np.uint8)
    if "jpeg" in p:
        buf = io.BytesIO()
        Image.fromarray(x).save(buf, "JPEG", quality=p["jpeg"])
        x = np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB"))
    return x


REAL = Path.home() / ".cache/huggingface/lerobot/local/plug_static"
info = json.loads((REAL / "meta/info.json").read_text())
features = {k: v for k, v in info["features"].items() if k in ("action", "observation.state", "observation.images.front", "observation.images.wrist")}
root = Path.home() / ".cache/huggingface/lerobot/local" / a.repo
if root.exists():
    shutil.rmtree(root)
ds = LeRobotDataset.create(f"local/{a.repo}", fps=info["fps"], features=features, root=root, robot_type=info.get("robot_type", "so_follower"),
                           use_videos=True)
R = Path(a.render)
jobs = []
for f in sorted(Path(a.src).glob("ep_*.npz")):
    if (R / f"{f.stem}_front.mp4").exists():
        jobs.append((f, f.stem, "n"))
    for v in ("n", "h"):
        if (R / f"{f.stem}_{v}_front.mp4").exists() and (R / f"{f.stem}_{v}_wrist.mp4").exists():
            jobs.append((f, f"{f.stem}_{v}", v))
log = []
for n, (f, stem, v) in enumerate(jobs):
    e = np.load(f)
    rng = np.random.default_rng(a.seed + n)
    p = make_aug(rng, v == "h")
    meta = json.loads((R / f"{stem}.json").read_text()) if (R / f"{stem}.json").exists() else {}
    frames = {c: [im for im in iio.get_reader(R / f"{stem}_{c}.mp4")] for c in ("front", "wrist")}
    drops = {c: (drop_mask(rng, p["drops"] if c == "wrist" else max(2, p["drops"] // 3)) if "drops" in p else None) for c in frames}
    lat = p.get("latency", 0)
    T = min(len(e["state"]), len(frames["front"]), len(frames["wrist"]))
    for i in range(T):
        j = max(0, i - lat)
        obs = {c: apply(frames[c][j], p, rng, i, drops[c], c) if p else frames[c][j] for c in frames}
        ds.add_frame({"observation.state": e["state"][i], "action": e["action"][i],
                      "observation.images.front": obs["front"], "observation.images.wrist": obs["wrist"], "task": str(e["task"])})
    ds.save_episode()
    log.append({"episode": n, "source": stem, "variant": v, "isaac_modes": meta.get("modes", []), "post": p})
    print(f"{n + 1}/{len(jobs)} {stem} {v} {meta.get('modes', [])} {p}", flush=True)
if hasattr(ds, "finalize"):
    ds.finalize()
(root / "meta/twin_aug.json").write_text(json.dumps(log, ensure_ascii=False, indent=1))
print("완료", root, len(jobs), "에피소드")
