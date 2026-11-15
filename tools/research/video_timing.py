"""설명 영상 V4 '쫓지 말고 내다보기': 같은 파도에서 따라가기와 예측의 차이가 어디서 오는지(MuJoCo).

- 위: 같은 시드의 따라가기, 예측 시행을 나란히(본편이 22초가 되게 느리게)
- 아래: 소켓 실제 높이, 따라가기가 본 위치(0.1 s 늦은 관측), 예측이 본 위치(0.3 s 앞 내다보기), 꽂기 시작 순간
추정값은 run_trial과 같은 시드로 SocketEstimator를 다시 돌려 그대로 재현한다(잡음 난수 순서가 같다).
실행: .venv/bin/python tools/research/video_timing.py [--seed 1002 --hs 0.06]
결과: ~/Downloads/PlugBot_연구/영상/V4_쫓지말고_내다보기.mp4
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import vidkit as V  # noqa: E402
from plugbot.control import CONTROL_DT, Params, SocketEstimator, run_trial  # noqa: E402
from plugbot.scene import Scene  # noqa: E402
from plugbot.waves import ShipMotion  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1002)
ap.add_argument("--hs", type=float, default=0.06)
a = ap.parse_args()
sc, p = Scene("ship", "side"), Params()
STRATS = (("reactive", "따라가기", (200, 120, 30)), ("predictive", "예측", V.SIM))
recs, trs = {}, {}
for s, _, _ in STRATS:
    rec = []
    trs[s] = run_trial(sc, s, ShipMotion(a.hs, seed=a.seed, berth="alongside"), p, seed=a.seed, record=rec)
    recs[s] = rec
    print(s, trs[s].inserted, trs[s].reason, round(rec[-1][0], 2))


def put(item):
    _, qpos, mp, mq = item[:4]
    sc.data.qpos[:] = qpos
    sc.data.mocap_pos[:] = mp
    sc.data.mocap_quat[:] = mq
    mujoco.mj_forward(sc.model, sc.data)


# 소켓 실제 높이와 두 추정값을 제어 주기마다 다시 계산(run_trial과 같은 시드, 같은 호출 순서)
def estimates(strategy):
    rec = recs[strategy]
    est = SocketEstimator(strategy, p, np.random.default_rng(a.seed))
    ts, true_z, est_z, phase = [], [], [], []
    # run_trial의 기록은 제어 주기(0.02 s)마다 하나. 제어 c번째에 쓴 상태는 바로 앞 기록(c-1)의 상태다
    for i in range(len(rec)):
        item = rec[i - 1] if i > 0 else rec[0]
        put(item)
        t = i * CONTROL_DT
        ent, ax = sc.socket_frame()
        est.capture(t, ent, ax)
        e = est.estimate(t)
        ts.append(t); true_z.append(ent[2]); est_z.append(e[0][2] if e else np.nan); phase.append(rec[min(i, len(rec) - 1)][4])
    return np.array(ts), np.array(true_z) * 1000, np.array(est_z) * 1000, phase


E = {s: estimates(s) for s, _, _ in STRATS}
tip_site = mujoco.mj_name2id(sc.model, mujoco.mjtObj.mjOBJ_SITE, "plug_tip")
put(recs["predictive"][-1])
look = sc.data.site_xpos[tip_site].copy()
cam = mujoco.MjvCamera()
cam.lookat[:] = look + np.array([-0.03, 0, 0])
cam.distance, cam.azimuth, cam.elevation = 0.30, 75, -12
R = mujoco.Renderer(sc.model, 480, 640)
TEND = max(r[-1][0] for r in recs.values()) + 0.6
SLOW = TEND / 22.0  # 본편이 자막 길이(22 s)에 맞게 느리게
z0 = np.nanmean(E["predictive"][1])
PX0, PX1, PY0, PY1 = 140, 1780, 640, 900
lo = min(np.nanmin(E[s][1]) for s in E) - z0 - 4
hi = max(np.nanmax(E[s][1]) for s in E) - z0 + 4


def X(t):
    return PX0 + (PX1 - PX0) * t / TEND


def Y(z):
    return PY1 - (PY1 - PY0) * (z - z0 - lo) / (hi - lo)


def insert_time(s):
    ts, _, _, ph = E[s]
    for t, x in zip(ts, ph):
        if x in ("insert", "keep"):
            return t
    return None


CUES = [(0.2, 5.5, "같은 파도(모형 6 cm), 같은 카메라 잡음입니다. 카메라 관측은 0.1초 늦게 들어오고, 팔은 명령을 0.2초쯤 늦게 따릅니다."),
        (5.7, 11.0, "따라가기(주황)는 늦게 본 위치를 쫓습니다. 소켓이 움직이는 동안 늘 한 박자 뒤에 있습니다."),
        (11.2, 16.5, "예측(청록)은 최근 3초를 AR 모형으로 맞춰 0.3초 앞 위치를 겨눕니다. 늦음을 미리 보정하는 것입니다."),
        (16.7, 22.0, "원래 설계에는 '소켓이 잠잠할 때 꽂기' 대기도 있지만, 대기를 꺼도 같은 파도 300개에서 결과가 하나도 달라지지 않았습니다.")]
w = V.Writer("V4_쫓지말고_내다보기.mp4")
for i in range(100):
    w.add(V.title_card("3장  움직이는 표적", "쫓지 말고 내다보기", "흔들리는 소켓을 쫓으면 왜 늦고, 예측은 정확히 무엇을 고치나?", i, 100))
n = int(TEND / SLOW * V.FPS)
for k in range(n):
    t = k / V.FPS * SLOW
    img = V.canvas()
    d = ImageDraw.Draw(img)
    for j, (s, name, col) in enumerate(STRATS):
        rec = recs[s]
        idx = min(int(t / CONTROL_DT), len(rec) - 1)
        put(rec[idx])
        R.update_scene(sc.data, cam)
        im = Image.fromarray(R.render()).resize((720, 540))
        X0 = 180 + j * 840
        img.paste(im, (X0, 60))
        V.text(d, (X0, 18), name, 34, True, col)
        if idx >= len(rec) - 1:
            ok = trs[s].inserted
            V.text(d, (X0 + 720, 18), "꽂힘" if ok else "실패: 소켓 밖 선체에 부딪힘", 30, True, (90, 200, 120) if ok else V.BAD, "ra")
    V.panel(img, [PX0 - 40, PY0 - 50, PX1 + 40, PY1 + 40])
    d = ImageDraw.Draw(img)
    V.text(d, (PX0, PY0 - 44), "소켓 높이 (mm)", 24, False, V.MUTED)
    ts, tz, _, _ = E["predictive"]
    m = ts <= t
    d.line([(X(x), Y(z)) for x, z in zip(ts[m], tz[m])], fill=V.INK, width=3)
    for s, name, col in STRATS:
        ts2, _, ez, _ = E[s]
        m2 = (ts2 <= t) & ~np.isnan(ez)
        if m2.sum() > 1:
            d.line([(X(x), Y(z)) for x, z in zip(ts2[m2], ez[m2])], fill=col, width=3)
        ti = insert_time(s)
        if ti is not None and ti <= t:
            d.line([X(ti), PY0, X(ti), PY1], fill=col, width=2)
            V.text(d, (X(ti) + 6, PY0 + (0 if s == "reactive" else 30)), f"{name} 꽂기 시작", 22, False, col)
    d.line([X(t), PY0, X(t), PY1], fill=(80, 90, 115), width=1)
    V.label(d, (PX0 + 330, PY0 - 46), "소켓 실제 높이", V.INK, 24)
    V.label(d, (PX0 + 590, PY0 - 46), "따라가기가 본 위치(0.1 s 늦음)", (200, 120, 30), 24)
    V.label(d, (PX0 + 1030, PY0 - 46), "예측이 본 위치(0.3 s 앞)", V.SIM, 24)
    s_, al = V.caption_at(CUES, k / V.FPS)
    w.add(V.caption(img, s_, al, y=V.H - 30, sz=34))
points = ["카메라와 서보 때문에 로봇은 늘 0.3초쯤 늦게 본다. 따라가기는 그 늦음을 그대로 안고 간다.",
          "예측은 최근 움직임으로 0.3초 앞 위치를 겨눠 늦음을 보정한다(MuJoCo 6 cm에서 88% 대 46%).",
          "'잠잠할 때 기다리기'는 이 시뮬레이션에서 결과를 바꾸지 않았다. 이점은 내다보기에서 온다.",
          "Isaac(PhysX)에서는 따라가기도 100%라 이 우위가 재현되지 않았다(물리 엔진 갭)."]
for i in range(140):
    w.add(V.end_card(points, i, 140))
w.close()
