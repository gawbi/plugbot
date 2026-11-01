"""PlugBot 웹 시연: 같은 파도에서 정지, 추종, 예측 세 로봇이 동시에 꽂기를 시도한다(MuJoCo, side 배치).

- POST /api/duel {"hs_cm": 0~8, "seed": 0~999}: 세 전략을 같은 파도로 돌리고 2 × 2 영상(세 로봇 + 예측 로봇의 손목 카메라)과
  소켓 움직임, 결과를 돌려준다. 계산은 plugbot.control.run_trial(MuJoCo 실험과 같은 코드)을 그대로 쓴다.
- GET /api/score: 관객이 돌린 판 수와 전략별 성공 수
- GET /api/isaac: Isaac Sim 결과(outputs/isaac_side_hs*_*.json)와 같은 조건의 MuJoCo 결과
- 정적 파일: web/static/(화면), /media/(outputs의 그림, GIF)

실행: MUJOCO_GL=egl .venv/bin/python -m uvicorn web.server:app --host 0.0.0.0 --port 8000
공개 서버로 열 것을 생각해 입력 범위를 묶고, 렌더는 한 번에 하나만(스레드 1개), 같은 파도는 캐시에서 준다.
"""

import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from fastapi import FastAPI, HTTPException  # noqa: E402
from fastapi.responses import FileResponse  # noqa: E402
from fastapi.staticfiles import StaticFiles  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pydantic import BaseModel, Field  # noqa: E402

from plugbot.control import CONTROL_DT, run_trial  # noqa: E402
from plugbot.scene import Scene, ship_pose  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
CACHE = HERE / "cache"
CACHE.mkdir(exist_ok=True)
OUT = ROOT / "outputs"

STRATEGIES = [("static", "정지", "처음 본 위치로 꽂는다"), ("reactive", "추종", "0.1초 늦게 본 위치를 따라간다"),
              ("predictive", "예측", "AR 예측 + 잔잔한 순간에 꽂는다")]
W, H = 480, 360  # 칸 하나(4:3). 영상은 2 × 2 = 960 × 720
FPS = 25
CAM_DELAY = 0.1  # plugbot.control.Params.cam_delay와 같음(그래프의 '카메라가 본 위치')
MAX_CACHE = 300
FONTS = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
         "/System/Library/Fonts/AppleSDGothicNeo.ttc"]


def font(size):
    for f in FONTS:
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def free_cam(pos, look):
    v = look - pos
    dist = float(np.linalg.norm(v))
    c = mujoco.MjvCamera()
    c.lookat[:] = look
    c.distance = dist
    c.azimuth = float(np.degrees(np.arctan2(v[1], v[0])))
    c.elevation = float(np.degrees(np.arcsin(v[2] / dist)))
    return c


class Worker:
    """MuJoCo 장면과 EGL 렌더러는 만든 스레드에서만 쓴다. 그래서 작업은 모두 이 스레드 하나에서 돈다."""

    def __init__(self):
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.scene = self.renderer = None

    def _init(self):
        if self.scene is None:
            self.scene = Scene("ship", "side")
            self.scene.model.vis.global_.fovy = 45
            self.renderer = mujoco.Renderer(self.scene.model, H, W)
            self.cam = free_cam(np.array([0.10, -0.40, 0.26]), np.array([0.30, 0.02, 0.02]))  # make_media.py와 같은 시점
            self.f_big, self.f_small = font(24), font(17)

    def duel(self, hs_cm, seed):
        return self.pool.submit(self._duel, hs_cm, seed).result()

    def _duel(self, hs_cm, seed):
        key = f"side_hs{hs_cm:.1f}_seed{seed}"
        meta_path = CACHE / f"{key}.json"
        if meta_path.exists():
            return json.loads(meta_path.read_text())
        self._init()
        s, m, d = self.scene, self.scene.model, self.scene.data
        hs = hs_cm / 100
        motion = ShipMotion(hs, seed=seed, berth="alongside") if hs > 0 else (lambda t: (np.zeros(3), 0.0, 0.0))
        recs, results = {}, {}
        for name, label, _ in STRATEGIES:
            rec = []
            tr = run_trial(s, name, motion, seed=seed, record=rec)
            recs[name] = rec
            results[name] = {"success": bool(tr.success), "reason": tr.reason or "",
                             "t_end": float(rec[-1][0]) if rec else 0.0,
                             "t_insert": None if np.isnan(tr.t_insert) else float(tr.t_insert)}
        t_end = max(r["t_end"] for r in results.values()) + 1.0
        video = self._render(key, recs, results, t_end)
        # 그래프: 소켓 입구의 실제 위치와 카메라가 본(0.1초 늦은) 위치, 쉴 때 기준 mm
        ts = np.arange(0.0, t_end, 0.02)
        ent0 = ship_pose("side", lambda t: (np.zeros(3), 0.0, 0.0), 0.0)[2]
        ent = np.array([ship_pose("side", motion, t)[2] for t in ts]) - ent0
        seen = np.array([ship_pose("side", motion, max(0.0, t - CAM_DELAY))[2] for t in ts]) - ent0
        meta = {"key": key, "hs_cm": hs_cm, "seed": seed, "video": f"/cache/{video.name}", "duration": t_end,
                "results": results, "wave": {"t": ts.round(3).tolist(), "y": (ent[:, 1] * 1000).round(2).tolist(),
                                             "z": (ent[:, 2] * 1000).round(2).tolist(),
                                             "z_seen": (seen[:, 2] * 1000).round(2).tolist()}}
        meta_path.write_text(json.dumps(meta, ensure_ascii=False))
        self._trim_cache()
        return meta

    def _render(self, key, recs, results, t_end):
        s, m, d, r = self.scene, self.scene.model, self.scene.data, self.renderer
        path = CACHE / f"{key}.mp4"
        writer = imageio.get_writer(str(path), fps=FPS, codec="libx264", quality=7, macro_block_size=8,
                                    ffmpeg_params=["-movflags", "+faststart"])
        n = int(round(t_end * FPS))
        per = int(round(1.0 / FPS / CONTROL_DT))
        for i in range(n):
            canvas = Image.new("RGB", (2 * W, 2 * H), (8, 18, 32))
            for j, (name, label, _) in enumerate(STRATEGIES):
                rec = recs[name]
                k = min(i * per, len(rec) - 1)
                t, qpos, mpos, mquat, phase, depth, lat, force = rec[k]
                d.qpos[:] = qpos
                d.mocap_pos[:] = mpos
                d.mocap_quat[:] = mquat
                mujoco.mj_forward(m, d)
                r.update_scene(d, self.cam)
                tile = Image.fromarray(r.render())
                self._label(tile, label, results[name], i / FPS)
                canvas.paste(tile, ((j % 2) * W, (j // 2) * H))
                if name == "predictive":  # 넷째 칸: 예측 로봇의 손목 카메라
                    r.update_scene(d, camera="wrist_cam")
                    wrist = Image.fromarray(r.render())
                    self._label(wrist, "예측 로봇 손목 카메라", None, i / FPS)
                    canvas.paste(wrist, (W, H))
            writer.append_data(np.asarray(canvas))
        writer.close()
        return path

    def _label(self, tile, label, res, t):
        dr = ImageDraw.Draw(tile, "RGBA")
        dr.rounded_rectangle((10, 10, 18 + dr.textlength(label, font=self.f_big), 46), 8, fill=(8, 18, 32, 190))
        dr.text((14, 11), label, font=self.f_big, fill=(240, 244, 248))
        if res is None or t < res["t_end"]:
            return
        ok = res["success"]
        text = "성공" if ok else "실패"
        color = (34, 197, 130, 230) if ok else (239, 83, 80, 230)
        w = dr.textlength(text, font=self.f_big)
        dr.rounded_rectangle((W - w - 34, 10, W - 10, 46), 8, fill=color)
        dr.text((W - w - 22, 11), text, font=self.f_big, fill=(255, 255, 255))
        if not ok and res["reason"]:
            dr.rounded_rectangle((10, H - 40, 18 + dr.textlength(res["reason"], font=self.f_small), H - 10), 6,
                                 fill=(8, 18, 32, 200))
            dr.text((14, H - 38), res["reason"], font=self.f_small, fill=(255, 205, 205))

    def _trim_cache(self):
        metas = sorted(CACHE.glob("side_*.json"), key=lambda p: p.stat().st_mtime)
        for p in metas[:-MAX_CACHE]:
            p.unlink(missing_ok=True)
            p.with_suffix(".mp4").unlink(missing_ok=True)


worker = Worker()
score_lock = threading.Lock()
SCORE = CACHE / "score.json"


def load_score():
    if SCORE.exists():
        return json.loads(SCORE.read_text())
    return {"rounds": 0, "wins": {n: 0 for n, _, _ in STRATEGIES}}


app = FastAPI(title="PlugBot")


class DuelIn(BaseModel):
    hs_cm: float = Field(6.0, ge=0.0, le=8.0)
    seed: int = Field(2, ge=0, le=999)


@app.post("/api/duel")
def duel(body: DuelIn):
    hs_cm = round(body.hs_cm * 2) / 2  # 0.5 cm 간격(캐시가 잘 맞게)
    meta = worker.duel(hs_cm, body.seed)
    with score_lock:
        sc = load_score()
        sc["rounds"] += 1
        for n, r in meta["results"].items():
            sc["wins"][n] += int(r["success"])
        SCORE.write_text(json.dumps(sc))
    return {**meta, "score": sc}


@app.get("/api/score")
def score():
    with score_lock:
        return load_score()


@app.get("/api/isaac")
def isaac():
    mj = json.loads((OUT / "experiment_side.json").read_text())["결과"]
    rows = []
    for hs in (0, 4, 6, 8):
        mj_key = next((k for k in mj if k.startswith(f"Hs={hs}cm")), None)
        for n, label, _ in STRATEGIES:
            f = OUT / f"isaac_side_hs{hs}_{n}.json"
            if not f.exists():
                continue
            r = json.loads(f.read_text())
            rows.append({"hs_cm": hs, "strategy": n, "label": label, "n": len(r["시행"]),
                         "isaac": r.get("충돌을 실패로 센 성공률", r["성공률"]),
                         "mujoco": None if mj_key is None else mj[mj_key][n]["성공률"]})
    return {"rows": rows}


MEDIA = {"isaac_gif": "showreel_side_hs6_compare.gif", "robustness": "sim2real_robustness.png",
         "kit": "side_kit.png", "isaac_preview": "isaac_preview_side_ShowCam.png"}


@app.get("/media/{name}")
def media(name: str):
    if name not in MEDIA:  # 정해 둔 파일만(경로를 받지 않는다)
        raise HTTPException(404)
    return FileResponse(OUT / MEDIA[name])


app.mount("/cache", StaticFiles(directory=CACHE), name="cache")
app.mount("/", StaticFiles(directory=HERE / "static", html=True), name="static")
