"""시범 녹화 화면(브라우저). act/record.sh(lerobot-record)를 가짜 터미널(pty) 안에서 돌리고, 버튼으로 n(다음), r(다시), q(끝)를 보낸다.

- 카메라 두 대를 이 화면에 띄운다. 평소에는 [카메라 켜기]로 이 화면이 카메라를 직접 열고, 녹화 중에는 act/record_preview.py가 /dev/shm에 쓴 영상. Rerun 창은 끈다.
- 녹화를 시작할 때 고른 '소켓 자리'를 에피소드 번호마다 기록한다: <데이터셋>/plugbot_positions.json.
  나중에 자리별 개수를 맞추고, 녹화하지 않은 자리에서 시험할 때 쓴다.
실행: bash act/record_gui.sh → http://127.0.0.1:8200
"""

import json
import os
import pty
import re
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from pydantic import BaseModel

ACT = Path(__file__).resolve().parent
DATA = Path.home() / ".cache" / "huggingface" / "lerobot" / os.environ.get("HF_USER", "local")
SEC = {"static": 25, "moving": 30}
RESET_S = 20
POSITIONS = ["가운데", "왼쪽 2 cm", "오른쪽 2 cm", "로봇 쪽 1.5 cm", "멀리 1.5 cm"]
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")

app = FastAPI()
# 마지막으로 고른 종류(정지, 흔들림)를 기억한다. 서버를 다시 켤 때마다 '정지 소켓'으로 돌아가서
# 흔들림 시범이 두 번이나 plug_static에 섞였다(2026-11-06)
KIND_FILE = Path.home() / ".cache" / "plugbot_gui_kind"
try:
    _KIND0 = KIND_FILE.read_text().strip() if KIND_FILE.read_text().strip() in ("static", "moving") else "static"
except OSError:
    _KIND0 = "static"
S = {"proc": None, "fd": None, "kind": _KIND0, "position": POSITIONS[0], "phase": "대기", "episode": None,
     "phase_t0": 0.0, "added": 0, "target": 0, "log": deque(maxlen=200), "error": "", "cam_on": False, "rollout": None}
LOCK = threading.Lock()


def ds_root(kind):
    return DATA / f"plug_{kind}"


def saved_count(kind):
    try:
        return json.loads((ds_root(kind) / "meta" / "info.json").read_text())["total_episodes"]
    except Exception:
        return 0


def positions_file(kind):
    return ds_root(kind) / "plugbot_positions.json"


def load_positions(kind):
    try:
        return json.loads(positions_file(kind).read_text())
    except Exception:
        return {}


def tag_episode(kind, ep, pos):
    if not ds_root(kind).exists():
        return
    d = load_positions(kind)
    d[str(ep)] = pos
    positions_file(kind).write_text(json.dumps(d, ensure_ascii=False, indent=1))
    # 그 에피소드 동안의 갑판 흔들기(없음, 보통 12 mm, 세게 16 mm): <데이터셋>/plugbot_shaker.json
    f = ds_root(kind) / "plugbot_shaker.json"
    try:
        sd = json.loads(f.read_text())
    except Exception:
        sd = {}
    sd[str(ep)] = SHAKER.get("mode") if shaker_running() else "없음"
    f.write_text(json.dumps(sd, ensure_ascii=False, indent=1))


def reader(fd, proc):
    buf = ""
    while True:
        try:
            data = os.read(fd, 4096)
        except OSError:
            break
        if not data:
            break
        buf += data.decode(errors="ignore")
        *lines, buf = re.split(r"[\r\n]+", buf)
        for line in lines:
            line = ANSI.sub("", line).strip()
            if not line:
                continue
            with LOCK:
                S["log"].append(line)
                m = re.search(r"Recording episode (\d+)", line)
                if m:
                    S["episode"] = int(m.group(1))
                    S["phase"], S["phase_t0"] = "녹화 중", time.time()
                    tag_episode(S["kind"], S["episode"], S["position"])
                elif "Reset the environment" in line:
                    S["phase"], S["phase_t0"] = "정리 시간(소켓 옮기기)", time.time()
                elif "Re-record episode" in line:
                    S["phase"] = "다시 찍기"
                elif S.get("rollout") and S["phase"] == "로봇 실행 준비" and re.search(r"(?i)rollout|running|start", line):
                    S["phase"], S["phase_t0"] = "로봇 실행 중", time.time()
                elif "Stop recording" in line:
                    S["phase"] = "저장 중"
                elif "Error" in line or "Traceback" in line:
                    S["error"] = line
    proc.wait()
    with LOCK:
        S["phase"] = "대기" if proc.returncode == 0 else f"끝남(코드 {proc.returncode})"
        S["proc"], S["fd"] = None, None
        S["rollout"] = None
    if S["cam_on"]:
        idle_start()  # 녹화 전에 카메라를 켜 두었으면 다시 미리보기


class Start(BaseModel):
    kind: str = "static"
    n: int = 10
    position: str = POSITIONS[0]


@app.post("/start")
def start(req: Start):
    with LOCK:
        if S["proc"] is not None:
            return JSONResponse({"ok": False, "msg": "이미 녹화 중"}, 409)
        idle_stop()  # 카메라를 녹화 프로그램에 넘긴다
        master, slave = pty.openpty()
        env = dict(os.environ, XDG_SESSION_TYPE="wayland", PYTHONUNBUFFERED="1", PREVIEW="1", SHOW="false")  # 키 입력을 터미널(pty)로 받게
        proc = subprocess.Popen(["bash", str(ACT / "record.sh"), req.kind, str(req.n)], stdin=slave, stdout=slave,
                                stderr=slave, cwd=ACT.parent, env=env, start_new_session=True)
        os.close(slave)
        try:
            KIND_FILE.write_text(req.kind)
        except OSError:
            pass
        S.update(proc=proc, fd=master, kind=req.kind, position=req.position, phase="연결 중", episode=None,
                 phase_t0=time.time(), target=req.n, added=saved_count(req.kind), error="")
        S["log"].clear()
    threading.Thread(target=reader, args=(master, proc), daemon=True).start()
    return {"ok": True}


@app.post("/delete_last")
def delete_last():
    """마지막 저장 에피소드를 지운다(녹화 중이 아닐 때). 원본은 <데이터셋>_backup_<시각>으로 옮겨 둔다."""
    with LOCK:
        if S["proc"] is not None:
            return JSONResponse({"ok": False, "msg": "녹화 중에는 [다시 찍기]를 쓴다"}, 409)
        kind = S["kind"]
    root = ds_root(kind)
    n = saved_count(kind)
    if n == 0:
        return JSONResponse({"ok": False, "msg": "지울 에피소드가 없다"}, 400)
    last = n - 1
    tmp = root.with_name(root.name + "_tmp")
    bak = root.with_name(root.name + time.strftime("_backup_%H%M%S"))
    r = subprocess.run([str(Path(sys.executable).parent / "lerobot-edit-dataset"), "--repo_id", f"local/{root.name}", "--root", str(root),
                        "--new_repo_id", f"local/{tmp.name}", "--new_root", str(tmp), "--operation.type", "delete_episodes",
                        "--operation.episode_indices", f"[{last}]"], capture_output=True, text=True)
    if r.returncode != 0 or not (tmp / "meta" / "info.json").exists():
        return JSONResponse({"ok": False, "msg": (r.stderr or r.stdout)[-300:]}, 500)
    pos = load_positions(kind)
    root.rename(bak)
    tmp.rename(root)
    pos.pop(str(last), None)
    positions_file(kind).write_text(json.dumps(pos, ensure_ascii=False, indent=1))
    with LOCK:
        S["log"].append(f"{last}번 에피소드를 지웠다(원본 백업: {bak.name})")
    return {"ok": True, "deleted": last, "backup": bak.name}


RUNS = Path.home() / "act_runs"


@app.get("/ckpts")
def ckpts():
    """학습 결과 체크포인트 목록(최근 것 먼저)."""
    out = []
    for run in sorted(RUNS.glob("*/checkpoints"), key=lambda p: p.stat().st_mtime, reverse=True):
        for c in sorted(run.glob("[0-9]*"), reverse=True):
            if (c / "pretrained_model" / "config.json").exists():
                out.append(f"{run.parent.name}/{c.name}")
    return out


class Roll(BaseModel):
    ckpt: str
    duration: float = 20.0
    replan: int = 100  # 몇 스텝마다 다시 예측할지(100 = 학습 설정 그대로 3.3 s, 1 = 매 스텝 예측을 겹쳐 평균)


@app.post("/rollout/start")
def rollout_start(req: Roll):
    path = RUNS / req.ckpt.split("/")[0] / "checkpoints" / req.ckpt.split("/")[1] / "pretrained_model"
    if not path.exists():
        return JSONResponse({"ok": False, "msg": f"{path} 없음"}, 404)
    with LOCK:
        if S["proc"] is not None:
            return JSONResponse({"ok": False, "msg": "이미 실행 중"}, 409)
    idle_stop()
    cams = (f"{{ front: {{type: opencv, index_or_path: {_env_cam('FRONT_CAM')}, width: 640, height: 480, fps: 30}}, "
            f"wrist: {{type: opencv, index_or_path: {_env_cam('WRIST_CAM')}, width: 640, height: 480, fps: 30}}}}")
    env_txt = (ACT / "env.sh").read_text()

    def env_default(name):
        m = re.search(name + r'="\$\{' + name + r':-([^}]*)\}"', env_txt)
        return os.environ.get(name) or (m.group(1) if m else "")
    cmd = [sys.executable, str(ACT / "record_preview.py"), "--plugbot-entry=rollout", "--strategy.type=base",
           f"--policy.path={path}", "--robot.type=so101_follower", f"--robot.port={env_default('F_PORT')}",
           f"--robot.id={env_default('F_ID')}", f"--robot.cameras={cams}", "--task=Insert the plug into the socket",
           f"--duration={req.duration}", "--device=cuda", "--display_data=false"]
    if req.replan < 100:
        # 출발 때 한 번 예측한 3.3 s를 눈 감고 따라가면 소켓 가까이에서 위치를 못 고친다(2026-11-06 왼쪽에서 가운데를 찌름)
        cmd.append(f"--policy.n_action_steps={max(1, req.replan)}")
        if req.replan <= 1:
            cmd.append("--policy.temporal_ensemble_coeff=0.01")
    master, slave = pty.openpty()
    env = dict(os.environ, XDG_SESSION_TYPE="wayland", PYTHONUNBUFFERED="1")
    proc = subprocess.Popen(cmd, stdin=slave, stdout=slave, stderr=slave, cwd=ACT.parent, env=env, start_new_session=True)
    os.close(slave)
    with LOCK:
        S.update(proc=proc, fd=master, phase="로봇 실행 준비", episode=None, phase_t0=time.time(), target=0,
                 added=saved_count(S["kind"]), error="", rollout=req.duration)
        S["log"].clear()
    threading.Thread(target=reader, args=(master, proc), daemon=True).start()
    return {"ok": True}


class Mark(BaseModel):
    ok: bool
    cond: str = "static"  # static(정지 소켓) 또는 moving(흔들림 갑판)


@app.post("/rollout/mark")
def rollout_mark(req: Mark):
    """마지막 로봇 실행(data/rollouts/<시각>/)에 사람이 본 판정(꽂힘, 못 꽂음)을 붙인다. 집계: tools/tally_rollouts.py"""
    d = ACT.parent / "data" / "rollouts"
    runs = sorted(x for x in d.iterdir() if (x / "meta.json").exists()) if d.exists() else []
    if not runs:
        return JSONResponse({"ok": False, "msg": "로봇 실행 기록이 없다"}, 404)
    last = runs[-1]
    meta = json.loads((last / "meta.json").read_text())
    res = {"ok": req.ok, "cond": req.cond, "policy": meta.get("policy", ""), "marked": time.strftime("%Y-%m-%d %H:%M:%S")}
    (last / "result.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
    n = sum(1 for x in runs if (x / "result.json").exists())
    return {"ok": True, "run": last.name, "marked_total": n}


@app.post("/rollout/stop")
def rollout_stop():
    with LOCK:
        p = S["proc"]
    if p is not None:
        p.send_signal(2)  # Ctrl+C: 정상 종료
    return {"ok": True}


@app.post("/key/{k}")
def key(k: str):
    if k not in ("n", "r", "q"):
        return JSONResponse({"ok": False}, 400)
    with LOCK:
        if S["fd"] is None:
            return JSONResponse({"ok": False, "msg": "녹화 중이 아님"}, 409)
        os.write(S["fd"], k.encode())
    return {"ok": True}


@app.post("/position")
def set_position(req: Start):
    with LOCK:
        S["position"] = req.position
    return {"ok": True}


@app.get("/state")
def state():
    with LOCK:
        kind = S["kind"]
        dur = (SEC.get(kind, 25) if S["phase"] == "녹화 중" else RESET_S if S["phase"].startswith("정리")
               else S["rollout"] if S["phase"] == "로봇 실행 중" else None)
        left = max(0.0, dur - (time.time() - S["phase_t0"])) if dur else None
        pos = load_positions(kind)
        saved = saved_count(kind)
        counts = {}
        for ep, p in pos.items():
            if int(ep) < saved:
                counts[p] = counts.get(p, 0) + 1
        return {"running": S["proc"] is not None, "kind": kind, "phase": S["phase"], "episode": S["episode"],
                "left": left, "saved": saved, "session": saved - S["added"] if S["proc"] else None, "target": S["target"],
                "position": S["position"], "counts": counts, "error": S["error"], "log": list(S["log"])[-14:],
                "positions": POSITIONS, "root": str(ds_root(kind)), "cam_on": S["cam_on"], "rollout": S["rollout"],
                "shaker": shaker_running(), "shaker_mode": SHAKER["mode"] if shaker_running() else None, "shaker_log": list(SHAKER["log"])[-3:]}


CAM = Path("/dev/shm/plugbot_cam")


def _env_cam(name):
    """act/env.sh의 FRONT_CAM, WRIST_CAM 기본 경로(환경변수가 있으면 그것)."""
    if os.environ.get(name):
        return os.environ[name]
    m = re.search(name + r'="\$\{' + name + r':-([^}]*)\}"', (ACT / "env.sh").read_text())
    return m.group(1) if m else ""


IDLE = {"proc": None}


def _cams_free(timeout=5.0):
    """카메라 장치를 아무도 안 쓸 때까지 기다린다(fuser)."""
    devs = [os.path.realpath(_env_cam(v)) for v in ("FRONT_CAM", "WRIST_CAM")]
    t0 = time.time()
    while time.time() - t0 < timeout:
        if subprocess.run(["fuser", *devs], capture_output=True).returncode != 0:
            return True
        time.sleep(0.2)
    return False


def idle_start():
    """평소 미리보기를 따로 프로세스로 켠다(끌 때 카메라를 확실히 놓게)."""
    if IDLE["proc"] is None or IDLE["proc"].poll() is not None:
        IDLE["proc"] = subprocess.Popen([sys.executable, str(ACT / "cam_preview.py"), _env_cam("FRONT_CAM"), _env_cam("WRIST_CAM")],
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def idle_stop():
    p = IDLE["proc"]
    if p is not None and p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    IDLE["proc"] = None
    _kill_stray_previews()
    _cams_free()


def _kill_stray_previews():
    """이전 서버가 남긴 cam_preview.py를 끈다. 미리보기를 켠 채 서버를 다시 켜면 미리보기가 주인 없이 남아
    카메라를 잡고, 녹화가 'Failed to open OpenCVCamera'로 멈췄다(2026-11-06, 트러블슈팅 19)."""
    me = IDLE["proc"].pid if IDLE["proc"] is not None else None
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) == me:
            continue
        try:
            cmd = (d / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if any(c.endswith(b"act/cam_preview.py") for c in cmd):
            try:
                os.kill(int(d.name), 15)
            except OSError:
                pass
    time.sleep(0.5)


_kill_stray_previews()  # 서버가 켜질 때도


# 흔들림 갑판(act/deck_shaker.py, ~/shaker_env). [흔들기] = 프로그램을 켠다(켜자마자 정현파 12 mm, 2.5 s, t=0에서 가운데라 튀지 않음).
# [멈추기] = q를 보내 가운데로 되돌리고 토크를 끄고 끝낸다. 녹화와 따로 돌아서 에피소드마다 시작 위상이 달라진다
SHAKER = {"proc": None, "log": deque(maxlen=20), "mode": None}
SHAKER_PY = Path.home() / "shaker_env" / "bin" / "python"


def _stop_stray_shakers():
    """이전 서버가 남긴 deck_shaker.py에 SIGINT(가운데로 되돌리고 토크 끔)."""
    import signal
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            cmd = (d / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if any(c.endswith(b"act/deck_shaker.py") for c in cmd):
            try:
                os.kill(int(d.name), signal.SIGINT)
            except OSError:
                pass


_stop_stray_shakers()


def _shaker_reader(proc):
    for line in proc.stdout:
        SHAKER["log"].append(line.rstrip())


def shaker_running():
    return SHAKER["proc"] is not None and SHAKER["proc"].poll() is None


class Shake(BaseModel):
    strong: bool = False


@app.post("/shaker/start")
def shaker_start(req: Shake = Shake()):
    if shaker_running():
        return JSONResponse({"ok": False, "msg": "이미 흔드는 중. [갑판 멈추기] 뒤 다시"}, 409)
    SHAKER["log"].clear()
    SHAKER["mode"] = "세게 16 mm" if req.strong else "보통 12 mm"
    env = dict(os.environ, SHAKER_AMP="16" if req.strong else "12", SHAKER_PERIOD="2.5")
    proc = subprocess.Popen([str(SHAKER_PY), "-u", str(ACT / "deck_shaker.py")], env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    SHAKER["proc"] = proc
    threading.Thread(target=_shaker_reader, args=(proc,), daemon=True).start()
    time.sleep(1.0)
    if proc.poll() is not None:
        return JSONResponse({"ok": False, "msg": " / ".join(SHAKER["log"]) or "흔들기 시작 실패"}, 500)
    return {"ok": True}


@app.post("/shaker/stop")
def shaker_stop():
    proc = SHAKER["proc"]
    if proc is None or proc.poll() is not None:
        return {"ok": True}
    try:
        proc.stdin.write("q\n")
        proc.stdin.flush()
        proc.wait(timeout=3)
    except Exception:
        import signal
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
    return {"ok": True}


@app.post("/camera/{onoff}")
def camera(onoff: str):
    with LOCK:
        if S["proc"] is not None:
            return JSONResponse({"ok": False, "msg": "녹화 중에는 늘 켜져 있다"}, 409)
        S["cam_on"] = onoff == "on"
    idle_start() if S["cam_on"] else idle_stop()
    return {"ok": True}


@app.get("/cam/{name}")
def cam(name: str):
    def gen():
        last = None
        while True:
            try:
                f = CAM / f"{name}.jpg"
                m = f.stat().st_mtime
                if m != last:
                    last = m
                    yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + f.read_bytes() + b"\r\n"
            except (FileNotFoundError, OSError):
                pass
            time.sleep(1 / 20)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=f")


@app.get("/ref/{name}")
def ref(name: str):
    # 학습 데이터(plug_static 20번 에피소드 첫 장면)의 카메라 화면. 카메라가 밀렸을 때 겹쳐 보며 되돌린다(2026-11-06 옆면 52 px 밀림)
    f = ACT.parent / "outputs" / "camera_ref" / f"dataset_{name}.jpg"
    if name not in ("front", "wrist") or not f.exists():
        return Response(status_code=404)
    return Response(f.read_bytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})


_DRIFT = {}  # 이번 세션 기준 화면(옆면)의 특징점


def _orb(img):
    import cv2
    g = cv2.cvtColor(cv2.resize(img, (640, 480)), cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(1500)
    return orb.detectAndCompute(cv2.equalizeHist(g), None)


def _cur_front():
    import cv2
    import numpy as np
    f = CAM / "front.jpg"
    if not f.exists() or time.time() - f.stat().st_mtime > 3:
        return None
    return cv2.imdecode(np.frombuffer(f.read_bytes(), np.uint8), cv2.IMREAD_COLOR)


@app.post("/drift/reset")
def drift_reset():
    # 지금 옆면 화면을 이번 세션 기준으로 삼고 outputs/camera_ref/session_<시각>.jpg로 남긴다(접고 펼 때마다 카메라 위치 기록)
    import cv2
    cur = _cur_front()
    if cur is None:
        return JSONResponse({"ok": False, "msg": "카메라 영상 없음"}, 409)
    _DRIFT.update(kp=_orb(cur), t=time.strftime("%Y%m%d_%H%M%S"))
    cv2.imwrite(str(ACT.parent / "outputs" / "camera_ref" / f"session_{_DRIFT['t']}.jpg"), cur)
    return {"ok": True, "t": _DRIFT["t"]}


@app.get("/drift")
def drift():
    # (화면에는 안 쓴다. 갑판, 소켓 움직임과 구분이 안 됨) 옆면 카메라가 이번 세션 기준 화면에서 얼마나 움직였는지. 특징점(ORB) 맞춤 + RANSAC이라 팔이 화면에 들어와도 덜 흔들린다.
    # 2026-11-06: 처음에는 학습 데이터 화면과 위상 상관으로 쟀는데, 카메라가 가까워져 크기가 바뀌자 19~190 px로 튀었다
    import cv2
    import numpy as np
    cur = _cur_front()
    if cur is None:
        return {"ok": False}
    if "kp" not in _DRIFT:
        drift_reset()
    k0, d0 = _DRIFT["kp"]
    k1, d1 = _orb(cur)
    if d0 is None or d1 is None:
        return {"ok": False}
    m = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(d0, d1)
    if len(m) < 12:
        return {"ok": True, "bad": True, "since": _DRIFT["t"]}
    p0 = np.float32([k0[x.queryIdx].pt for x in m])
    p1 = np.float32([k1[x.trainIdx].pt for x in m])
    M, inl = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC, ransacReprojThreshold=3)
    if M is None or inl.sum() < 12:
        return {"ok": True, "bad": True, "since": _DRIFT["t"]}
    # 화면 네 귀퉁이가 평균 몇 픽셀 옮겨졌는지(이동, 회전, 크기 변화를 한 숫자로)
    c = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
    moved = c @ M[:, :2].T + M[:, 2]
    px = float(np.linalg.norm(moved - c, axis=1).mean())
    return {"ok": True, "px": round(px, 1), "inliers": int(inl.sum()), "since": _DRIFT["t"]}


@app.get("/frame/{name}")
def frame(name: str):
    """가장 최근 한 장. 화면이 0.1초마다 새로 받아 간다(긴 스트림은 녹화와 미리보기가 바뀔 때 멈춰 버렸다)."""
    try:
        data = (CAM / f"{name}.jpg").read_bytes()
    except OSError:
        return Response(status_code=404)
    return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/", response_class=HTMLResponse)
def index():
    return PAGE


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><title>PlugBot 시범 녹화</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1720;--card:#18222e;--fg:#e8eef5;--mut:#93a4b8;--acc:#7c5cff;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444}
body{margin:0;background:var(--bg);color:var(--fg);font-family:'Noto Sans KR',system-ui,sans-serif}
main{max-width:1200px;margin:0 auto;padding:18px}
h1{font-size:22px;margin:0 0 12px}.card{background:var(--card);border-radius:14px;padding:16px;margin-bottom:14px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}label{color:var(--mut);font-size:14px}
select,input{background:#0b121a;color:var(--fg);border:1px solid #2b3a4b;border-radius:8px;padding:8px;font-size:16px}
button{border:0;border-radius:12px;padding:14px 18px;font-size:18px;font-weight:700;cursor:pointer;color:#fff}
.start{background:var(--acc)}.camb{background:#2b3a4b}.camb.on{background:#0e7490}.delb{background:#7f1d1d;font-size:15px}.next{background:var(--ok);flex:2}.redo{background:var(--warn);flex:1}.quit{background:var(--bad);flex:1}
button:disabled{opacity:.35;cursor:default}
.phase{font-size:34px;font-weight:800}.big{font-size:56px;font-weight:800;font-variant-numeric:tabular-nums}
.bar{height:12px;background:#0b121a;border-radius:6px;overflow:hidden}.bar>div{height:100%;background:var(--acc);transition:width .2s}
.pos{padding:10px 14px;border-radius:10px;background:#0b121a;border:2px solid #2b3a4b;cursor:pointer;font-size:16px}
.pos.on{border-color:var(--acc);background:#241d4d}
pre{background:#0b121a;border-radius:10px;padding:10px;font-size:12px;color:var(--mut);white-space:pre-wrap;max-height:220px;overflow:auto;margin:0}
.err{color:var(--bad)}.cams{display:grid;grid-template-columns:1fr 1fr;gap:10px}.cams figure{margin:0;position:relative}.cams img.ref{position:absolute;left:0;top:0;opacity:var(--refop,.35);pointer-events:none;display:none}.refon .cams img.ref{display:block}
.cams img{width:100%;aspect-ratio:4/3;background:#000;border-radius:10px;display:block}.cams figcaption{color:var(--mut);font-size:13px;margin-top:4px}
@media(max-width:640px){.cams{grid-template-columns:1fr}}.mut{color:var(--mut);font-size:14px}
</style></head><body><main>
<h1>PlugBot 시범 녹화 (ACT)</h1>
<div class="card"><div class="row">
 <label>종류 <select id="kind"><option value="static">정지 소켓 (25초)</option><option value="moving">흔들림 갑판 (30초)</option></select></label>
 <label>이번에 추가할 개수 <input id="n" type="number" value="10" min="1" max="100" style="width:80px"></label>
 <button class="start" id="start">녹화 시작</button> <button class="camb" id="camb">카메라 켜기</button> <button class="camb" id="shk">갑판 흔들기</button> <button class="camb" id="shkstop">갑판 멈추기</button> <span id="shkst" class="mut"></span> <label class="mut"><input type="checkbox" id="refchk"> 학습 때 화면 겹쳐 보기</label> <label class="mut">진하기 <input type="range" id="refop" min="10" max="80" value="35" style="width:90px;vertical-align:middle"></label> <button onclick="camRec()">카메라 위치 기록</button> <span id="drift" class="mut"></span>
 <button class="delb" id="delb">마지막 에피소드 지우기</button></div>
 <p class="mut">카메라 화면은 아래에 나온다. 같은 종류로 다시 시작하면 이어서 녹화된다. 그리퍼는 덮개를 살짝만 물게 하한(6)을 걸었다.</p></div>
<div class="card"><div class="cams"><figure><img id="img_front" alt=""><img class="ref" src="/ref/front" alt=""><figcaption>옆면 카메라 (front)</figcaption></figure>
 <figure><img id="img_wrist" alt=""><img class="ref" src="/ref/wrist" alt=""><figcaption>손목 카메라 (wrist)</figcaption></figure></div>
 <p class="mut">[카메라 켜기]로 평소에 보고, 녹화를 시작하면 몇 초 끊겼다가 녹화 영상으로 이어진다(녹화 중에는 늘 나온다).</p></div>
<div class="card"><div class="mut" style="margin-bottom:8px">소켓 자리 (바꾸면 다음 에피소드부터 이 자리로 기록)</div><div class="row" id="poss"></div>
 <div class="mut" id="counts" style="margin-top:10px"></div></div>
<div class="card"><div class="row" style="justify-content:space-between"><div><div class="phase" id="phase">대기</div>
 <div class="mut" id="sub"></div></div><div class="big" id="left"></div></div>
 <div class="bar" style="margin:12px 0"><div id="bar" style="width:0"></div></div>
 <div class="row"><button class="next" id="next">다음 (성공, 저장) [→ 또는 스페이스]</button>
 <button class="redo" id="redo">다시 찍기 [←]</button><button class="quit" id="quit">끝내기 [Esc]</button></div>
 <p class="mut">녹화 중: 홈에서 출발, 소켓 앞에서 멈춰 맞추고, 천천히 끝까지 꽂는다. <b>꽂힌 채로 2~3초 가만히</b> 있다가 [다음](충전 중 상태로 끝). 실패나 빈 녹화는 [다시 찍기].<br>
 정리 시간(녹화 안 됨): 빼서 홈으로 돌려놓고, 소켓을 표시한 자리로 옮기고 위 '소켓 자리'를 맞게 누른다. 준비되면 [다음]으로 바로 다음 녹화.</p>
 <div class="err" id="err"></div></div>
<div class="card"><div class="mut" style="margin-bottom:8px">로봇 혼자 꽂기 (학습한 정책 시험, 녹화 안 함)</div>
 <div class="row"><label>체크포인트 <select id="ckpt"></select></label>
 <label>다시 계획 <select id="replan"><option value="100" selected>3.3초마다(학습 그대로)</option><option value="30">1초마다</option><option value="10">0.33초마다</option><option value="1">매 순간(평균)</option></select></label>
 <label>시간(초) <input id="rdur" type="number" value="20" min="5" max="120" style="width:70px"></label>
 <button class="start" id="rstart">로봇 실행</button><button class="quit" id="rstop">멈추기</button></div>
 <div class="row" style="margin-top:8px"><label>판정 조건 <select id="mcond"><option value="static">정지 소켓</option><option value="moving">흔들림 갑판</option></select></label>
 <button class="next" id="mok">꽂힘 (성공)</button><button class="redo" id="mfail">못 꽂음 (실패)</button> <span id="mres" class="mut"></span></div>
 <p class="mut">실행 전: 리더 팔은 그대로 두고(이때는 리더를 안 씀), 팔로워를 홈 근처에, 소켓을 표시한 자리에. 사람 손은 부스 밖.
 비상 시 [멈추기] 또는 팔로워 12 V 전원. 끝나면 꽂힌 채로 멈춘다(빼기는 손으로 받치고 다음 녹화나 원격조종 때).</p></div>
<div class="card"><div class="mut" style="margin-bottom:6px">로그</div><pre id="log"></pre></div>
</main><script>
const $=id=>document.getElementById(id);let st=null,lastPos=null;
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:b?JSON.stringify(b):null});return r.json()}
$('start').onclick=()=>{const k=$('kind').value;
 // 2026-11-06: 갑판을 흔들면서 종류가 '정지 소켓'인 채로 10개를 찍어 plug_static에 섞였다. 맞지 않으면 묻는다
 if(st&&st.shaker&&k==='static'&&!confirm('갑판이 흔들리는 중인데 종류가 [정지 소켓]입니다. 그대로 정지 데이터에 녹화할까요? (흔들림이면 취소 후 종류를 [흔들림 갑판]으로)'))return;
 if(st&&!st.shaker&&k==='moving'&&!confirm('종류가 [흔들림 갑판]인데 갑판이 멈춰 있습니다. 그대로 녹화할까요?'))return;
 post('/start',{kind:k,n:+$('n').value,position:lastPos||'가운데'})};
$('delb').onclick=async()=>{if(!st||st.saved<1)return;if(!confirm(`${st.saved-1}번(마지막) 에피소드를 지울까요? 원본은 백업됩니다.`))return;
 $('delb').disabled=true;$('delb').textContent='지우는 중...';const r=await post('/delete_last');alert(r.ok?`${r.deleted}번을 지웠습니다.`:'실패: '+r.msg);$('delb').textContent='마지막 에피소드 지우기'};
async function loadCkpts(){const l=await (await fetch('/ckpts')).json();const sel=$('ckpt');const cur=sel.value;sel.innerHTML='';
 l.forEach(c=>{const o=document.createElement('option');o.value=o.textContent=c;sel.appendChild(o)});if(cur&&l.includes(cur))sel.value=cur}
loadCkpts();setInterval(loadCkpts,30000);
$('rstart').onclick=()=>{if(!$('ckpt').value){alert('체크포인트가 아직 없습니다');return}
 if(confirm(`${$('ckpt').value} 로 ${$('rdur').value}초 동안 로봇을 움직입니다. 손을 부스 밖으로!`))post('/rollout/start',{ckpt:$('ckpt').value,duration:+$('rdur').value,replan:+$('replan').value})};
$('rstop').onclick=()=>post('/rollout/stop');
// 실행이 끝나면 사람이 보고 판정한다(마지막 실행 기록에 붙음). 발표 9장 숫자는 이 판정을 센다(tools/tally_rollouts.py)
async function mark(ok){const r=await post('/rollout/mark',{ok:ok,cond:$('mcond').value});$('mres').textContent=r.ok?`${r.run}: ${ok?'성공':'실패'} 기록 (판정 ${r.marked_total}개)`:(r.msg||'실패')}
$('mok').onclick=()=>mark(true);$('mfail').onclick=()=>mark(false);
$('camb').onclick=()=>post('/camera/'+(st&&st.cam_on?'off':'on'));
$('shk').onclick=async()=>{if(!confirm('갑판을 흔듭니다(정현파 12 mm, 2.5초). 갑판 주변에서 손을 빼세요'))return;const r=await post('/shaker/start',{strong:false});if(!r.ok)alert(r.msg)};
// [세게 흔들기](16 mm) 버튼은 뺐다: 진폭은 12 mm 하나로 통일(2026-11-06 결정). 서버의 strong 옵션은 남겨 둠
$('shkstop').onclick=()=>post('/shaker/stop');
$('next').onclick=()=>post('/key/n');$('redo').onclick=()=>post('/key/r');$('quit').onclick=()=>{if(confirm('녹화를 끝낼까요? (지금 에피소드는 저장되지 않을 수 있음)'))post('/key/q')};
document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();post('/key/n')}
 else if(e.key==='ArrowLeft')post('/key/r');else if(e.key==='Escape')$('quit').click()});
function renderPos(list,cur){const b=$('poss');if(b.childElementCount!==list.length){b.innerHTML='';list.forEach(p=>{const d=document.createElement('div');d.className='pos';d.textContent=p;
 d.onclick=()=>{lastPos=p;post('/position',{position:p});[...b.children].forEach(c=>c.classList.toggle('on',c.textContent===p))};b.appendChild(d)})}
 [...b.children].forEach(c=>c.classList.toggle('on',c.textContent===cur))}
async function tick(){try{st=await (await fetch('/state')).json()}catch(e){return}
 if(!window._kindSynced){window._kindSynced=true;$('kind').value=st.kind}
 $('shk').classList.toggle('on',!!st.shaker);$('shk').textContent=st.shaker?('흔드는 중: '+(st.shaker_mode||'')):'갑판 흔들기';$('shk').disabled=!!st.shaker;$('shkstop').disabled=!st.shaker;
 $('shkst').textContent=(!st.shaker&&st.shaker_log&&st.shaker_log.length)?st.shaker_log[st.shaker_log.length-1]:'';
 if(!lastPos)lastPos=st.position;renderPos(st.positions,st.running?st.position:lastPos);
 $('phase').textContent=st.phase;$('phase').style.color=st.phase==='녹화 중'||st.phase==='로봇 실행 중'?'#22c55e':st.phase.startsWith('정리')?'#f59e0b':'';
 $('sub').textContent=(st.episode!==null?`에피소드 ${st.episode}  `:'')+`저장된 에피소드 ${st.saved}`+(st.running?`  (이번 ${st.session}/${st.target})`:'')+`  자리: ${st.running?st.position:lastPos}`;
 $('left').textContent=st.left!==null?Math.ceil(st.left)+'s':'';
 const dur=st.phase==='로봇 실행 중'?st.rollout:st.phase==='녹화 중'?(st.kind==='moving'?30:25):20;$('bar').style.width=st.left!==null?(100*(1-st.left/dur))+'%':'0';
 $('counts').textContent='자리별 저장 개수: '+(Object.entries(st.counts).map(([k,v])=>`${k} ${v}`).join(', ')||'아직 없음')+`   (${st.root})`;
 ['next','redo','quit'].forEach(i=>$(i).disabled=!st.running||!!st.rollout);
 $('rstart').disabled=st.running;$('rstop').disabled=!(st.running&&st.rollout);
 $('camb').textContent=st.running?'카메라: 녹화 중':(st.cam_on?'카메라 끄기':'카메라 켜기');$('camb').classList.toggle('on',st.running||st.cam_on);$('camb').disabled=st.running;
 if($('delb').textContent!=='지우는 중...')$('delb').disabled=st.running||st.saved<1;$('start').disabled=st.running;$('kind').disabled=st.running;
 $('err').textContent=st.error||'';$('log').textContent=st.log.join('\\n');}
setInterval(tick,300);tick();
// 겹쳐 보기는 기본으로 켠다(녹화 중에도). 끈 것은 이 브라우저에 기억
{const c=$('refchk');let on=true;try{on=localStorage.getItem('refon')!=='0'}catch(e){}
 c.checked=on;document.body.classList.toggle('refon',on);
 c.onchange=()=>{document.body.classList.toggle('refon',c.checked);try{localStorage.setItem('refon',c.checked?'1':'0')}catch(e){}}}
// 옆면 카메라 위치 기록: 접고 펼 때마다 [카메라 위치 기록]. 실시간 밀림 숫자는 뺐다(2026-11-06: 벽에 무늬가 없어
// 소켓, 브래킷, 볼트로만 재게 되는데 이것들은 소켓 자리 바꾸기와 갑판 흔들림에 같이 움직여 카메라 움직임과 구분이 안 됨)
async function camRec(){const r=await post('/drift/reset');$('drift').textContent=r.ok?('기록함 '+r.t):(r.msg||'실패')}
{const r=$('refop');try{const v=localStorage.getItem('refop');if(v)r.value=v}catch(e){}
 const ap=()=>{document.body.style.setProperty('--refop',r.value/100);try{localStorage.setItem('refop',r.value)}catch(e){}};r.oninput=ap;ap()}
function pull(name){const im=$('img_'+name);const next=()=>setTimeout(()=>pull(name),100);
 const n=new Image();n.onload=()=>{im.src=n.src;next()};n.onerror=()=>setTimeout(()=>pull(name),500);n.src='/frame/'+name+'?t='+Date.now()}
pull('front');pull('wrist');
</script></body></html>"""
