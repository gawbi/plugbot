"""README, 발표용 그림(MuJoCo, 맥).

- outputs/side_compare_hs6.gif: 같은 파도(모형 Hs 6 cm, 같은 시드)에서 추종(왼쪽)과 예측(오른쪽). 옆으로 꽂기 배치
- outputs/domain_randomization.png: 학습 데이터 무작위화 예(앞 카메라, 손목 카메라)
- outputs/isaac_viewpoints_mujoco.png: Isaac 장면의 세 카메라 자리를 MuJoCo로 그린 것(실제 Isaac 화면이 아니다)

실행: .venv/bin/python make_media.py
"""

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plugbot.control import Params, home_pose, run_trial
from plugbot.randomize import randomize, restore
from plugbot.scene import Scene
from plugbot.waves import ShipMotion

OUT = __import__("pathlib").Path(__file__).resolve().parent / "outputs"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
SHOW = (np.array([0.06, -0.48, 0.30]), np.array([0.28, 0.03, 0.02]))  # isaac/build_assets.py ShowCam(side), 초점 18 mm


def free_cam(pos, look):
    v = look - pos
    dist = float(np.linalg.norm(v))
    c = mujoco.MjvCamera()
    c.lookat[:] = look
    c.distance = dist
    c.azimuth = float(np.degrees(np.arctan2(v[1], v[0])))
    c.elevation = float(np.degrees(np.arcsin(v[2] / dist)))
    return c


def pick_seed(scene, hs):
    for k in range(60):
        a = run_trial(scene, "reactive", ShipMotion(hs, seed=k, berth="alongside"), seed=k)
        b = run_trial(scene, "predictive", ShipMotion(hs, seed=k, berth="alongside"), seed=k)
        if b.inserted and not a.inserted:
            return k, a.reason
    raise RuntimeError("추종 실패, 예측 성공인 시드가 없다")


def compare_gif(hs=0.06, w=560, h=400, fps=15):
    scene = Scene("ship", "side")
    m, d = scene.model, scene.data
    k, why = pick_seed(scene, hs)
    recs = {}
    for s in ("reactive", "predictive"):
        rec = []
        run_trial(scene, s, ShipMotion(hs, seed=k, berth="alongside"), seed=k, record=rec)
        recs[s] = rec
    r = mujoco.Renderer(m, h, w)
    m.vis.global_.fovy = 45
    cam = free_cam(np.array([0.10, -0.40, 0.26]), np.array([0.30, 0.02, 0.02]))  # 소켓 앞면이 보이는 로봇 오른쪽 앞
    f_t, f_s = ImageFont.truetype(FONT, 26), ImageFont.truetype(FONT, 20)
    n = max(len(v) for v in recs.values())
    step = max(1, int(round(1 / fps / 0.02)))
    frames = []
    for i in range(0, n + 25, step):
        panel = Image.new("RGB", (2 * w + 8, h + 64), "white")
        dr = ImageDraw.Draw(panel)
        for j, (s, label) in enumerate((("reactive", "추종: 지연된 위치를 따라감"), ("predictive", "예측: AR 예측 + 정온 구간"))):
            rec = recs[s]
            t, qpos, mpos, mquat, phase, depth, lat, force = rec[min(i, len(rec) - 1)]
            d.qpos[:] = qpos
            d.mocap_pos[:] = mpos
            d.mocap_quat[:] = mquat
            mujoco.mj_forward(m, d)
            r.update_scene(d, cam)
            panel.paste(Image.fromarray(r.render()), (j * (w + 8), 64))
            ended = i >= len(rec) - 1
            ok = s == "predictive"
            dr.text((j * (w + 8) + 12, 6), label, font=f_t, fill=(20, 20, 20))
            status = ("성공" if ok else f"실패: {why}") if ended else f"t = {t:4.1f} s  깊이 {depth * 1000:5.1f} mm"
            dr.text((j * (w + 8) + 12, 38), status, font=f_s,
                    fill=(26, 138, 92) if ended and ok else (200, 120, 30) if ended else (90, 96, 104))
        frames.append(panel)
    frames[0].save(OUT / "side_compare_hs6.gif", save_all=True, append_images=frames[1:], duration=int(1000 / fps),
                   loop=0, optimize=True)
    print(OUT / "side_compare_hs6.gif", f"시드 {k}, {len(frames)}장")


def dr_figure(n=4):
    scene = Scene("ship", "side")
    m, d = scene.model, scene.data
    d.qpos[scene.qadr] = home_pose(scene)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 240, 320)
    rng = np.random.default_rng(3)
    W, H = 320, 240
    canvas = Image.new("RGB", (n * W + (n - 1) * 6, 2 * H + 6 + 40), "white")
    for i in range(n):
        if i:
            randomize(scene, rng)
        mujoco.mj_forward(m, d)
        for row, cam in enumerate(("front", "wrist_cam")):
            r.update_scene(d, camera=cam)
            canvas.paste(Image.fromarray(r.render()), (i * (W + 6), row * (H + 6)))
    restore(scene)
    ImageDraw.Draw(canvas).text((8, 2 * H + 14), "왼쪽 끝 = 기본, 나머지 = 조명, 색, 앞 카메라 자리(±1.5 cm, ±2°)를 무작위로 바꾼 학습 장면. "
                                "위: 앞 카메라, 아래: 손목 카메라", font=ImageFont.truetype(FONT, 17), fill=(30, 30, 30))
    canvas.save(OUT / "domain_randomization.png")
    print(OUT / "domain_randomization.png")


def isaac_viewpoints():
    scene = Scene("ship", "side")
    m, d = scene.model, scene.data
    d.qpos[scene.qadr] = home_pose(scene)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 540, 960)
    imgs = []
    for cam in ("front", None, "wrist_cam"):
        if cam is None:
            m.vis.global_.fovy = float(np.degrees(2 * np.arctan(20.955 / 2 / 18 * 540 / 960)))
            r.update_scene(d, free_cam(*SHOW))
        else:
            r.update_scene(d, camera=cam)
        imgs.append(Image.fromarray(r.render()))
    f = ImageFont.truetype(FONT, 26)
    caps = ["Webcam: 앞 카메라 자리(화각 60°)", "ShowCam: 발표 영상 시점", "Wrist: 손목 카메라"]
    canvas = Image.new("RGB", (960 * 3 + 32, 540 + 52), "white")
    dr = ImageDraw.Draw(canvas)
    for k, im in enumerate(imgs):
        canvas.paste(im, (k * 976, 0))
        dr.text((k * 976 + 10, 550), caps[k], font=f, fill=(0, 0, 0))
    canvas.save(OUT / "isaac_viewpoints_mujoco.png")
    print(OUT / "isaac_viewpoints_mujoco.png")


if __name__ == "__main__":
    compare_gif()
    dr_figure()
    isaac_viewpoints()
