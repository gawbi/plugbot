"""본선 체험 서버: '사람 대 로봇'(리더 팔 텔레옵 20초 도전 → 로봇 차례), '바다 조종기'(파도 세기, 충전 시작 버튼),
'로봇의 눈'(앞 카메라 영상). 구성은 act/README.md '본선 체험 구성'.

장치(없으면 가짜로 동작해 화면과 흐름을 미리 시험할 수 있다):
- 갑판: SHAKER_PORT(예 /dev/ttyUSB0)의 아두이노(real/shaker). s=정현파, +/-=진폭 1 mm, p<초>=주기, h=멈춤, ?=상태
- 꽂힘 판정: 아두이노가 'c1'(꽂힘), 'c0'(빠짐)을 보내면 자동(센서는 아직 없음). 없으면 화면의 '성공' 버튼이나 스페이스
- 로봇: ROBOT=fake(기본)는 MuJoCo로 같은 갑판 운동을 계산해 결과와 시간을 낸다(plugbot.control.run_trial).
  ROBOT=classic(real.insert), ROBOT=act(학습 정책)는 실물을 연결하는 날 붙인다
- 카메라: EYE_CAM(번호 또는 /dev/v4l/by-id/... 경로)의 영상을 그대로 보낸다

실행: cd ~/plugbot && ~/lerobot_env/bin/python -m uvicorn demo.server:app --host 127.0.0.1 --port 8100
      (예: SHAKER_PORT=/dev/ttyUSB0 EYE_CAM=/dev/v4l/by-id/... ROBOT=fake ...)  화면: http://127.0.0.1:8100
"""

import os
import threading
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from fastapi import FastAPI, HTTPException  # noqa: E402
from fastapi.responses import StreamingResponse  # noqa: E402
from fastapi.staticfiles import StaticFiles  # noqa: E402
from pydantic import BaseModel  # noqa: E402

HERE = Path(__file__).resolve().parent
CHALLENGE_S = 20.0  # 사람 도전 시간
HOLD_S = 0.2  # 꽂힘 유지 판정(시뮬레이션과 같은 기준)
# 파도 세기: 정현파 진폭(mm), 주기(s). '보통'은 시뮬레이션 갑판 운동에서 100%(README). 나머지는 실물 리허설로 확인해 고친다
WAVES = {"calm": (4.0, 3.0), "normal": (8.0, 2.5), "rough": (12.0, 2.0)}


class Shaker:
    """흔들림 갑판 아두이노. 포트가 없으면 상태만 들고 있는 가짜."""

    def __init__(self, port):
        self.port, self.ser, self.level, self.amp = port, None, "stop", 8.0
        self.lock = threading.Lock()
        self.listeners = []  # 아두이노가 보낸 줄을 받을 함수(꽂힘 판정)
        if port:
            import serial
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2.0)  # 아두이노가 연결 직후 다시 시작한다
            threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            for f in self.listeners:
                f(line)

    def _send(self, s):
        if self.ser:
            self.ser.write(s.encode())
            time.sleep(0.05)

    def set(self, level):
        with self.lock:
            if level == "stop":
                self._send("h")
            else:
                amp, period = WAVES[level]
                step = int(round(amp - self.amp))  # 스케치는 진폭을 1 mm씩만 바꾼다
                self._send(("+" if step > 0 else "-") * abs(step))
                self._send(f"p{period}\n")
                self._send("s")
                self.amp = amp
            self.level = level


class Judge:
    """꽂힘 판정. 아두이노 'c1'이 HOLD_S 넘게 이어지면 꽂힘. 센서가 없으면 manual()로."""

    def __init__(self):
        self.on_since, self.callbacks = None, []

    def line(self, s):
        if s == "c1" and self.on_since is None:
            self.on_since = time.time()
        elif s == "c0":
            self.on_since = None

    def seated(self):
        return self.on_since is not None and time.time() - self.on_since >= HOLD_S


class FakeRobot:
    """MuJoCo로 같은 갑판 운동(plugbot.waves.HingeDeckMotion)을 계산하고, 결과 시각까지 실제 시간으로 기다린다."""

    name = "fake"

    def __init__(self):
        from plugbot.scene import Scene
        self.scene = Scene("ship", "side")

    def insert(self, level, seed):
        from plugbot.control import run_trial
        from plugbot.scene import SOCKET_SIDE_ON_SHIP
        from plugbot.waves import HINGE_BELOW_SIDE_SOCKET, HeaveProfile, HingeDeckMotion
        amp, period = WAVES.get(level, (0.0, 2.5))
        motion = HingeDeckMotion(HeaveProfile("sine", amp / 1000, period), tuple(SOCKET_SIDE_ON_SHIP),
                                 HINGE_BELOW_SIDE_SOCKET)
        tr = run_trial(self.scene, "predictive", motion, seed=seed)
        t = tr.t_insert if tr.success else 12.0
        time.sleep(min(t, 12.0))
        return tr.success, (t if tr.success else None), tr.reason


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.human = None  # {"start", "result": None|{"ok","t"}}
        self.robot = None  # {"start", "running", "result"}
        self.rounds = []  # [{"human": t|None, "robot": t|None}]
        self.round = {}


shaker = Shaker(os.environ.get("SHAKER_PORT"))
judge = Judge()
shaker.listeners.append(judge.line)
robot = FakeRobot() if os.environ.get("ROBOT", "fake") == "fake" else None
st = State()
app = FastAPI(title="PlugBot 체험")


class WaveIn(BaseModel):
    level: str


@app.post("/api/wave")
def wave(body: WaveIn):
    if body.level not in (*WAVES, "stop"):
        raise HTTPException(400)
    shaker.set(body.level)
    return {"level": shaker.level}


def _finish_human(ok):
    h = st.human
    if h and h["result"] is None:
        h["result"] = {"ok": ok, "t": round(time.time() - h["start"], 2) if ok else None}
        st.round["human"] = h["result"]["t"]


@app.post("/api/human/start")
def human_start():
    with st.lock:
        st.round = {}
        st.human = {"start": time.time(), "result": None}
        st.robot = None
    return {"ok": True}


@app.post("/api/human/success")
def human_success():  # 센서가 없을 때 팀원이 누르는 판정 버튼(스페이스)
    with st.lock:
        if st.human and time.time() - st.human["start"] <= CHALLENGE_S:
            _finish_human(True)
    return {"ok": True}


@app.post("/api/robot/start")
def robot_start():
    if robot is None:
        raise HTTPException(503, "ROBOT=classic, act는 실물 연결 날 붙인다")
    with st.lock:
        if st.robot and st.robot["running"]:
            return {"ok": False}
        st.robot = {"start": time.time(), "running": True, "result": None}

    def work():
        ok, t, why = robot.insert(shaker.level, seed=len(st.rounds))
        with st.lock:
            st.robot.update(running=False, result={"ok": ok, "t": round(t, 2) if t else None, "why": why})
            st.round["robot"] = st.robot["result"]["t"]
            st.rounds.append(dict(st.round))
    threading.Thread(target=work, daemon=True).start()
    return {"ok": True}


@app.get("/api/state")
def state():
    with st.lock:
        now = time.time()
        h = st.human
        if h and h["result"] is None:
            if judge.seated():
                _finish_human(True)
            elif now - h["start"] > CHALLENGE_S:
                _finish_human(False)
        return {"wave": shaker.level, "waves": WAVES, "shaker": bool(shaker.ser), "robot_backend": robot.name if robot else None,
                "sensor": os.environ.get("SENSOR") == "1", "challenge_s": CHALLENGE_S,
                "human": None if h is None else {"elapsed": round(now - h["start"], 2), "result": h["result"]},
                "robot": None if st.robot is None else {"elapsed": round(now - st.robot["start"], 2),
                                                         "running": st.robot["running"], "result": st.robot["result"]},
                "rounds": st.rounds[-8:],
                "totals": {"human": sum(r.get("human") is not None for r in st.rounds),
                           "robot": sum(r.get("robot") is not None for r in st.rounds), "n": len(st.rounds)}}


def _frames():
    src = os.environ.get("EYE_CAM")
    cap = cv2.VideoCapture(int(src) if src and src.isdigit() else src) if src else None
    while True:
        ok, img = cap.read() if cap is not None else (False, None)
        if not ok:
            img = np.full((480, 640, 3), 24, np.uint8)
            cv2.putText(img, "NO CAMERA (EYE_CAM)", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)
            time.sleep(0.2)
        # TODO(실물 연결 날): real.vision.SocketTracker로 마커, 입구 위치, 예측 위치, '잔잔한 순간' 막대를 겹친다
        cv2.putText(img, f"wave: {shaker.level}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 220, 200), 2)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"


@app.get("/api/eye.mjpg")
def eye():
    return StreamingResponse(_frames(), media_type="multipart/x-mixed-replace; boundary=f")


app.mount("/", StaticFiles(directory=HERE / "static", html=True), name="static")
