"""포토박스 트윈 3단계: 렌더 영상 + 실물 상태와 동작 → LeRobot 데이터셋.

화면(front, wrist)은 Isaac 트윈 렌더만 쓴다(실물 사진은 한 장도 넣지 않는다). 상태와 동작은 실물 시범 값 그대로(LeRobot 눈금).
이름과 형식은 실물 데이터셋(plug_static)과 같다: observation.state, action(6), observation.images.front, .wrist(480x640).

렌더 폴더 형식 두 가지를 읽는다: ep_XXX_{front,wrist}.mp4(예전), ep_XXX_{n,h}_{front,wrist}.mp4(보통, 가혹 판).
가혹 조건 후처리(--aug): 가혹 판은 1~3가지, 보통 판은 30% 확률로 1가지.
  blur 초점 흐림, motion 흔들림 번짐, noise 센서 노이즈, jpeg 압축 깨짐, flicker 노출 출렁임,
  fog 안개(대비 저하), drops 렌즈 물방울, latency 화면이 관절보다 1~3장면 늦음(실물 카메라 지연 흉내)
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
a = ap.parse_args()

EFFECTS = ["blur", "motion", "noise", "jpeg", "flicker", "fog", "drops", "latency"]


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


def apply(img, p, rng, k, drops):
    im = Image.fromarray(img)
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
    rng = np.random.default_rng(1000 + n)
    p = make_aug(rng, v == "h")
    meta = json.loads((R / f"{stem}.json").read_text()) if (R / f"{stem}.json").exists() else {}
    frames = {c: [im for im in iio.get_reader(R / f"{stem}_{c}.mp4")] for c in ("front", "wrist")}
    drops = {c: (drop_mask(rng, p["drops"] if c == "wrist" else max(2, p["drops"] // 3)) if "drops" in p else None) for c in frames}
    lat = p.get("latency", 0)
    T = min(len(e["state"]), len(frames["front"]), len(frames["wrist"]))
    for i in range(T):
        j = max(0, i - lat)
        obs = {c: apply(frames[c][j], p, rng, i, drops[c]) if p else frames[c][j] for c in frames}
        ds.add_frame({"observation.state": e["state"][i], "action": e["action"][i],
                      "observation.images.front": obs["front"], "observation.images.wrist": obs["wrist"], "task": str(e["task"])})
    ds.save_episode()
    log.append({"episode": n, "source": stem, "variant": v, "isaac_modes": meta.get("modes", []), "post": p})
    print(f"{n + 1}/{len(jobs)} {stem} {v} {meta.get('modes', [])} {p}", flush=True)
if hasattr(ds, "finalize"):
    ds.finalize()
(root / "meta/twin_aug.json").write_text(json.dumps(log, ensure_ascii=False, indent=1))
print("완료", root, len(jobs), "에피소드")
