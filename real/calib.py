"""카메라 → 로봇 베이스 좌표 변환(T_base_cam) 보정.

방법(권장, 소켓 꽂기): 로봇 토크를 끄고 손으로 팔을 움직여 플러그를 깔때기에 끝까지(자석이 붙을 때까지) 꽂는다.
깔때기가 핀을 가운데로 모아 주므로 핀 끝(관절값 → 순기구학) = 보어 바닥(마커 → 카메라 좌표)인 점 한 쌍이 생긴다.
갑판을 책상 위 4~6곳으로 옮기거나 레버로 기울여 가며 반복하면, 점 쌍들로 회전과 이동을 구한다(Kabsch).

실행: .venv/bin/python -m real.calib --port /dev/tty.usbmodemXXXX --cam 0
결과: real/config/hand_eye.json
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from . import geometry as G

CONFIG = Path(__file__).resolve().parent / "config"


def kabsch(p_cam, p_base):
    """p_base ≈ R @ p_cam + t 를 최소제곱으로 푼다. (R, t, 점별 오차 m)"""
    A, B = np.asarray(p_cam), np.asarray(p_base)
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = cb - R @ ca
    err = np.linalg.norm((R @ A.T).T + t - B, axis=1)
    return R, t, err


class HandEye:
    def __init__(self, R, t):
        self.R, self.t = np.asarray(R), np.asarray(t)

    def point(self, p_cam):
        return self.R @ p_cam + self.t

    def vector(self, v_cam):
        return self.R @ v_cam

    def save(self, path=CONFIG / "hand_eye.json", rms=None):
        Path(path).parent.mkdir(exist_ok=True)
        Path(path).write_text(json.dumps({"R": self.R.tolist(), "t": self.t.tolist(), "rms_mm": rms}, indent=1))

    @classmethod
    def load(cls, path=CONFIG / "hand_eye.json"):
        d = json.loads(Path(path).read_text())
        return cls(d["R"], d["t"])


def bore_bottom_cam(tracker, img):
    """소켓 마커로 본 보어 바닥 중심(카메라 좌표). 마커가 없으면 None."""
    pose = tracker.socket_pose(img)
    return None if pose is None else pose[1] + pose[0] @ G.BORE_BOTTOM_IN_SOCKET


def averaged(cam, fn, seconds=0.6):
    """정지 상태에서 여러 장을 평균해 잡음을 줄인다."""
    vals, t_end, last = [], time.monotonic() + seconds, None
    while time.monotonic() < t_end:
        t, img = cam.latest()
        if t != last:
            last = t
            v = fn(img)
            if v is not None:
                vals.append(v)
        time.sleep(0.005)
    return np.mean(vals, axis=0) if len(vals) >= 3 else None


def main():
    from .robots import RealRobot
    from .vision import Camera, Intrinsics, SocketTracker
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--cam", type=int, default=0)
    a = ap.parse_args()
    intr = Intrinsics.load() if (CONFIG / "camera.json").exists() else Intrinsics.from_fov(640, 480)
    tracker, cam, robot = SocketTracker(intr), Camera(a.cam), RealRobot(a.port)
    robot.torque(False)
    pairs = []
    print("토크를 껐다. 팔을 손으로 움직여 플러그를 깔때기에 끝까지 꽂고(자석이 붙게) 손을 뗀 뒤 Enter.")
    print("갑판 위치·기울기를 바꿔 4~6번 반복. 끝내려면 q + Enter.")
    try:
        while True:
            if input(f"[{len(pairs)}쌍] Enter=기록, q=계산: ").strip().lower() == "q":
                break
            p_cam = averaged(cam, lambda img: bore_bottom_cam(tracker, img))
            if p_cam is None:
                print("  마커가 안 보인다. 카메라와 마커를 확인하고 다시.")
                continue
            p_base = robot.tip_position()
            pairs.append((p_cam, p_base))
            print(f"  카메라 {np.round(p_cam * 1000, 1)} mm → 핀 끝 {np.round(p_base * 1000, 1)} mm")
    finally:
        cam.close()
        robot.close()
    if len(pairs) < 4:
        raise SystemExit("4쌍 이상 필요하다")
    R, t, err = kabsch([p for p, _ in pairs], [q for _, q in pairs])
    rms = float(np.sqrt(np.mean(err ** 2)) * 1000)
    HandEye(R, t).save(rms=rms)
    print(f"점별 오차(mm): {np.round(err * 1000, 1)}, RMS {rms:.1f} mm → {CONFIG / 'hand_eye.json'}")
    if rms > 4:
        print("RMS가 4 mm를 넘는다. 깔때기 포착 반경(12 mm) 안이지만 카메라 보정이나 관절 매핑을 먼저 확인한다.")


if __name__ == "__main__":
    main()
