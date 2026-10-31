"""실물 삽입 제어 루프(시뮬레이션 plugbot.control.run_trial과 같은 전략과 상태 기계).

웹캠 영상 → 소켓 자세(SocketTracker) → 로봇 좌표(HandEye) → AR 예측과 정온 구간 판단 → IK → 팔.
상태: approach(입구 위 30 mm로) → gate(예측 상하 속도가 작아질 때까지) → insert(축을 따라 내려감)
     → release(그리퍼 열기, 자석 래치가 잡는다) → retract(축 방향으로 60 mm 물러남).

실물:   .venv/bin/python -m real.insert --port /dev/tty.usbmodemXXXX --cam 0 --trials 10
가짜:   real/dry_run.py가 같은 run()을 SimWorld로 부른다.
"""

import argparse
import time
from dataclasses import dataclass, field

import numpy as np

from plugbot.control import CONTROL_DT, HOME, Params, ar_predict

from .calib import HandEye

DEPTH_MARGIN = 0.002


class RealEstimator:
    """촬영 시각이 고르지 않은 관측을 30 Hz 격자로 다시 맞춘 뒤 AR로 앞을 예측한다."""

    def __init__(self, p: Params):
        self.p, self.obs = p, []  # (촬영 시각, 입구, 축)

    def add(self, t, entrance, axis):
        self.obs.append((t, entrance, axis))
        self.obs = [o for o in self.obs if o[0] >= t - self.p.ar_window - 0.5]

    def last_time(self):
        return self.obs[-1][0] if self.obs else -np.inf

    def estimate(self, now, strategy):
        """(예측 입구, 축, 예측 속도 크기) 또는 None."""
        if not self.obs:
            return None
        axis = np.mean([o[2] for o in self.obs[-5:]], axis=0)
        axis /= np.linalg.norm(axis)
        if strategy == "static":
            return self.obs[0][1], self.obs[0][2], 0.0
        if strategy == "reactive" or len(self.obs) < self.p.ar_order * 3:
            return self.obs[-1][1], axis, 0.0
        dt = 1.0 / self.p.cam_hz
        T = np.array([o[0] for o in self.obs])
        P = np.array([o[1] for o in self.obs])
        grid = np.arange(T[-1], max(T[0], T[-1] - self.p.ar_window) - 1e-9, -dt)[::-1]
        G = np.stack([np.interp(grid, T, P[:, k]) for k in range(3)], axis=1)
        ahead = now - T[-1] + self.p.lead
        steps = max(1, int(round(ahead / dt)))
        pred = np.stack([ar_predict(G[:, k], self.p.ar_order, steps + 1) for k in range(3)], axis=1)
        speed = np.linalg.norm(pred[-1] - pred[-2]) / dt
        return pred[-2] if steps > 1 else pred[0], axis, speed


@dataclass
class Result:
    seated: bool = False  # 파이프라인 자신의 판단(추정 입구 기준 목표 깊이 유지)
    reason: str = ""
    t_seated: float = float("nan")
    lost_frames: int = 0
    log: list = field(default_factory=list)


def run(robot, camera, tracker, hand_eye: HandEye, p: Params = Params(), strategy="predictive", cam_delay=0.0,
        monitor=None, timeout=None, release=True):
    """한 번 꽂는다. monitor(robot)가 문자열을 돌려주면 그 이유로 중단한다(시뮬레이션 채점·안전용)."""
    est, res = RealEstimator(p), Result()
    phase, depth_cmd, gate_t, hold_t = "approach", -p.hover, None, 0.0
    t0 = robot.now()
    t_end = t0 + (timeout or p.timeout)
    last_frame, t_next = None, t0
    while robot.now() < t_end:
        now = robot.now()
        t_img, img = camera.latest()
        if t_img != last_frame:
            last_frame = t_img
            s = tracker.socket(img)
            if s is not None:
                est.add(t_img - cam_delay, hand_eye.point(s[0]), hand_eye.vector(s[1]))
            else:
                res.lost_frames += 1
        if now - est.last_time() > 3.0:
            res.reason = "마커를 3초 넘게 못 봄"
            break
        e = est.estimate(now, strategy)
        q = robot.q()
        if e is not None and now - est.last_time() < 0.5:  # 0.5초 넘게 못 보면 그 자리에 멈춘다
            ent, ax, speed = e
            tip = robot.tip_position()
            rel = tip - ent
            along = float(np.dot(rel, ax))
            lateral = float(np.linalg.norm(rel - along * ax))
            if phase == "approach" and lateral < p.align_tol and abs(along - p.hover) < p.align_tol:
                phase, gate_t = "gate", now
            if phase == "gate" and (strategy != "predictive" or speed < p.gate_speed or now - gate_t > p.gate_timeout):
                phase = "insert"
            if phase == "insert":
                # 보정 오차(수 mm)를 감안해 목표보다 조금 더 깊이 누르고, 판단도 2 mm 여유를 둔다
                depth_cmd = min(depth_cmd + p.descend_speed * CONTROL_DT, p.depth_goal + DEPTH_MARGIN + 0.006)
                if -along >= p.depth_goal + DEPTH_MARGIN and lateral < 0.004:
                    hold_t += CONTROL_DT
                    if hold_t >= p.hold:
                        res.seated, res.t_seated = True, now - t0
                        break
                else:
                    hold_t = 0.0
            robot.command(robot.kin.solve(q, ent - ax * depth_cmd, -ax) if hasattr(robot, "kin") else q)
            res.log.append((now - t0, phase, *ent, speed, along, lateral))
        if monitor is not None:
            why = monitor(robot)
            if why:
                res.reason = why
                break
        t_next += CONTROL_DT
        robot.wait_until(t_next)
    else:
        res.reason = res.reason or "시간 초과"
    if res.seated and release:
        robot.gripper(True)
        _retract(robot, est, 0.06)
    return res


def _retract(robot, est, dist, speed=0.05):
    """소켓 축 방향으로 물러난다(놓은 뒤 플러그를 건드리지 않게)."""
    if not est.obs:
        return
    ax = est.obs[-1][2]
    start = robot.tip_position()
    n = max(1, int(dist / speed / CONTROL_DT))
    t_next = robot.now()
    for k in range(1, n + 1):
        robot.command(robot.kin.solve(robot.q(), start + ax * dist * k / n, -ax))
        t_next += CONTROL_DT
        robot.wait_until(t_next)


def go_home(robot, seconds=2.0):
    q0, n = robot.q(), int(seconds / CONTROL_DT)
    t_next = robot.now()
    for k in range(1, n + 1):
        robot.command(q0 + (HOME - q0) * k / n)
        t_next += CONTROL_DT
        robot.wait_until(t_next)


def main():
    from .robots import RealRobot
    from .vision import CONFIG, Camera, Intrinsics, SocketTracker
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--strategy", default="predictive", choices=["static", "reactive", "predictive"])
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--cam-delay", type=float, default=0.05, help="영상을 받은 시각과 실제 촬영 시각의 차(s)")
    a = ap.parse_args()
    intr = Intrinsics.load() if (CONFIG / "camera.json").exists() else Intrinsics.from_fov(640, 480)
    tracker, cam, robot = SocketTracker(intr), Camera(a.cam), RealRobot(a.port)
    he = HandEye.load()
    robot.torque(True)
    ok = 0
    try:
        for k in range(a.trials):
            go_home(robot)
            robot.gripper(False)
            input(f"[{k + 1}/{a.trials}] 플러그를 그리퍼에 물리고(홈) 갑판을 준비한 뒤 Enter (Ctrl+C로 중단)")
            r = run(robot, cam, tracker, he, strategy=a.strategy, cam_delay=a.cam_delay)
            ok += r.seated
            print(f"  {'성공' if r.seated else '실패'} {r.reason} 시간 {r.t_seated:.1f}s, 마커 놓친 영상 {r.lost_frames}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("중단: 현재 자세에서 멈춘다")
    finally:
        go_home(robot)
        cam.close()
        robot.close()
    print(f"{ok}/{a.trials} 성공")


if __name__ == "__main__":
    main()
