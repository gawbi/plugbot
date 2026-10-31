"""플러그 삽입 제어기와 시행 평가.

제어 50 Hz, 물리 500 Hz. 카메라는 소켓 입구 위치를 30 Hz로 지연(latency)과 잡음을 섞어 알려 준다.
전략:
- static: 처음 본 소켓 위치로만 간다
- reactive: 지연된 최신 위치를 그대로 따라간다
- predictive: 최근 관측으로 자기회귀(AR) 모델을 맞춰 지연+서보 지연만큼 앞을 예측하고,
  상하 속도가 작아지는 순간(정온 구간)에 삽입을 시작한다
"""

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .scene import BORE_R, BORE_TOP, PIN_R, SIDE_AXIS, SIDE_ENTRANCE, SOCKET_H, Scene

HOME = np.array([0.0, -1.5, -0.29, 1.6, 0.0])
CONTROL_DT = 0.02


@dataclass
class Params:
    latency: float = 0.10
    cam_hz: float = 30.0
    noise: float = 0.0005
    lead: float = 0.20  # 서보 추종 지연 보상(예측 전략)
    hover: float = 0.030  # 입구 위 대기 높이
    descend_speed: float = 0.04
    depth_goal: float = SOCKET_H - BORE_TOP + 0.012  # 입구에서 보어 안쪽 12 mm까지
    align_tol: float = 0.004
    gate_speed: float = 0.012  # 예측 전략: 이 속도 이하일 때 삽입 시작
    gate_timeout: float = 3.0
    timeout: float = 12.0
    hold: float = 0.2  # 목표 깊이를 이 시간 이상 유지하면 삽입 성공
    # 삽입 후 연결 유지 시험 시간. 기본 0: 실제 자동 연결 장치처럼 소켓 래치가 플러그를 잡고 로봇은 놓는다고 가정한다.
    # 로봇이 계속 쥐고 있으면(예: 4초) 위치 제어 팔은 선체 운동을 못 따라가 과부하, 이탈이 난다(실험으로 확인).
    keep: float = 0.0
    force_limit: float = 20.0  # 유지 중 접촉력 95백분위 한계(N), 커넥터와 소켓 보호 기준(가정)
    ar_order: int = 6
    ar_window: float = 3.0


# ---------- IK ----------

def _dpinv(J, lam):
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(J.shape[0]))


class IK:
    def __init__(self, scene: Scene):
        self.s = scene
        self.data = mujoco.MjData(scene.model)  # 계산 전용 복사본
        self.jacp = np.zeros((3, scene.model.nv))
        self.jacr = np.zeros((3, scene.model.nv))

    def solve(self, q0, target, tool_dir, iters=40, k_dir=0.6, lam=0.01):
        if self.s.layout == "side":
            return self._solve_full(q0, target, tool_dir, iters, lam)
        s, d = self.s, self.data
        q = q0.copy()
        eye = np.eye(len(q))
        for _ in range(iters):
            d.qpos[:] = 0
            d.qpos[s.qadr] = q
            mujoco.mj_kinematics(s.model, d)
            mujoco.mj_comPos(s.model, d)
            e_p = target - d.site_xpos[s.tip_site]
            axis = d.xmat[s.gripper_body].reshape(3, 3) @ s.pin_axis  # 핀 방향(일자: 그리퍼 -z, L자: 그리퍼 -y)
            e_o = np.cross(axis, tool_dir)
            mujoco.mj_jacSite(s.model, d, self.jacp, self.jacr, s.tip_site)
            Jp, Jo = self.jacp[:, s.dadr], self.jacr[:, s.dadr]
            Jp_inv = _dpinv(Jp, lam)
            dq = Jp_inv @ e_p + (eye - Jp_inv @ Jp) @ (_dpinv(Jo, lam) @ (k_dir * e_o))
            q = np.clip(q + np.clip(dq, -0.2, 0.2), s.qlo, s.qhi)
            if np.linalg.norm(e_p) < 2e-4 and np.linalg.norm(dq) < 1e-4:
                break
        return q


def home_pose(scene: Scene, back=0.04, n=40, seed=0):
    """side 배치의 시작 자세: 정지 소켓 입구 앞 back(m)에서 핀이 소켓 축과 나란한 자세.

    관절 한계 안 무작위 시작점 n개에서 최적화하고, 로봇이 자기 몸이나 환경과 부딪히지 않는 해 가운데 가장 좋은 것을 쓴다
    (단순 IK는 손목 회전 갈래를 놓친다). 한 번 구해 둔다.
    """
    if scene.home is not None:
        return scene.home
    from scipy.optimize import least_squares
    s, d = scene, mujoco.MjData(scene.model)
    if s.mode == "ship":
        d.mocap_pos[s.mocap] = s.ship_home
        d.mocap_quat[s.mocap] = [1, 0, 0, 0]
    mujoco.mj_kinematics(s.model, d)
    ent, ax = (SIDE_ENTRANCE, SIDE_AXIS)
    target, tool = ent + ax * back, -ax

    def resid(q):
        d.qpos[s.qadr] = q
        mujoco.mj_kinematics(s.model, d)
        R = d.xmat[s.gripper_body].reshape(3, 3)
        return np.concatenate([(d.site_xpos[s.tip_site] - target) * 1000, (R @ s.pin_axis - tool) * 50])

    def collides(q):
        d.qpos[s.qadr] = q
        mujoco.mj_forward(s.model, d)
        return any(d.contact[i].dist < 0 and (d.contact[i].geom1 in s.robot_geoms or d.contact[i].geom2 in s.robot_geoms)
                   for i in range(d.ncon))

    rng = np.random.default_rng(seed)
    sols = sorted((least_squares(resid, rng.uniform(s.qlo, s.qhi), bounds=(s.qlo, s.qhi)) for _ in range(n)),
                  key=lambda r: r.cost)
    best = next((r for r in sols if r.cost < 1e-3 and not collides(r.x)), sols[0])
    scene.home = best.x
    return best.x


def _solve_full_impl(self, q0, target, tool_dir, iters, lam, w_dir=0.05):
    """side 배치: 위치와 핀 방향을 함께 맞춘다(감쇠 최소제곱). 5축으로 수평 자세를 만들면 남는 자유도가 없어서,
    위치를 먼저 맞추고 방향을 남는 자유도로 맞추는 방식(top 배치)은 방향이 크게 틀어진다."""
    s, d = self.s, self.data
    q = q0.copy()
    for _ in range(iters):
        d.qpos[:] = 0
        d.qpos[s.qadr] = q
        mujoco.mj_kinematics(s.model, d)
        mujoco.mj_comPos(s.model, d)
        e_p = target - d.site_xpos[s.tip_site]
        axis = d.xmat[s.gripper_body].reshape(3, 3) @ s.pin_axis
        e_o = np.cross(axis, tool_dir)
        mujoco.mj_jacSite(s.model, d, self.jacp, self.jacr, s.tip_site)
        J = np.vstack([self.jacp[:, s.dadr], w_dir * self.jacr[:, s.dadr]])
        e = np.concatenate([e_p, w_dir * e_o])
        dq = _dpinv(J, lam) @ e
        q = np.clip(q + np.clip(dq, -0.2, 0.2), s.qlo, s.qhi)
        if np.linalg.norm(e_p) < 2e-4 and np.linalg.norm(dq) < 1e-4:
            break
    return q


IK._solve_full = _solve_full_impl


# ---------- 소켓 추정 ----------

def ar_predict(series, order, steps):
    """1차원 시계열을 AR(order)로 맞추고 steps 만큼 앞을 예측한 값들(길이 steps)."""
    x = np.asarray(series)
    mu = x.mean()
    y = x - mu
    if len(y) < order * 3:
        return np.full(steps, x[-1])
    X = np.stack([y[i:len(y) - order + i] for i in range(order)], axis=1)
    coef, *_ = np.linalg.lstsq(X, y[order:], rcond=None)
    hist = list(y[-order:])
    out = []
    for _ in range(steps):
        nxt = float(np.dot(coef, hist[-order:]))
        out.append(nxt)
        hist.append(nxt)
    return np.array(out) + mu


class SocketEstimator:
    def __init__(self, strategy, p: Params, rng):
        self.strategy, self.p, self.rng = strategy, p, rng
        self.obs = []  # (측정 시각, 입구 위치, 축)
        self.next_cam = 0.0

    def capture(self, t, entrance, axis):
        if t + 1e-9 >= self.next_cam:
            self.obs.append((t, entrance + self.rng.normal(0, self.p.noise, 3), axis.copy()))
            self.next_cam += 1.0 / self.p.cam_hz

    def available(self, t):
        return [o for o in self.obs if o[0] <= t - self.p.latency]

    def estimate(self, t):
        """(예측 입구 위치, 축, 예측 상하, 좌우 속도 크기). 관측이 아직 없으면 None."""
        av = self.available(t)
        if not av:
            return None
        if self.strategy == "static":
            return av[0][1], av[0][2], 0.0
        if self.strategy == "reactive":
            return av[-1][1], av[-1][2], 0.0
        win = [o for o in av if o[0] >= av[-1][0] - self.p.ar_window]
        P = np.array([o[1] for o in win])
        dt = 1.0 / self.p.cam_hz
        ahead = t - av[-1][0] + self.p.lead
        steps = max(1, int(round(ahead / dt)))
        pred = np.stack([ar_predict(P[:, k], self.p.ar_order, steps + 1) for k in range(3)], axis=1)
        speed = np.linalg.norm(pred[-1] - pred[-2]) / dt
        return pred[-2] if steps > 1 else pred[0], av[-1][2], speed


# ---------- 시행 ----------

@dataclass
class Trial:
    inserted: bool = False  # 보어 안 목표 깊이까지 들어감
    held: bool = False  # 이후 유지 시간 동안 빠지지 않고 접촉력 한계 안
    reason: str = ""
    t_insert: float = float("nan")
    f_insert_p95: float = 0.0  # 삽입 중 플러그-소켓 접촉력 95백분위(N)
    f_hold_p95: float = 0.0  # 유지 중 접촉력 95백분위(N)
    forces: list = field(default_factory=list)

    @property
    def success(self):
        return self.inserted and self.held


def contacts(scene: Scene):
    """(로봇 몸체 충돌: None | "socket"(그리퍼가 소켓에 닿음) | "env"(선체, 부두 등), 플러그-선체 접촉 여부, 플러그-소켓 접촉력 합)"""
    m, d = scene.model, scene.data
    body_hit, plug_hull, force = None, False, 0.0
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = c.geom1, c.geom2
        if g1 in scene.env_geoms:
            g1, g2 = g2, g1
        if g1 not in scene.robot_geoms or g2 not in scene.env_geoms or c.dist > 0:
            continue
        if g1 in scene.plug_geoms:
            if g2 in scene.socket_geoms:
                mujoco.mj_contactForce(m, d, i, f6)
                force += abs(f6[0])
            else:
                plug_hull = True
        elif body_hit != "env":
            body_hit = "socket" if g2 in scene.socket_geoms else "env"
    return body_hit, plug_hull, force


def run_trial(scene: Scene, strategy, motion=None, p: Params = Params(), seed=0, record=None, on_step=None):
    m, d = scene.model, scene.data
    rng = np.random.default_rng(seed)
    mujoco.mj_resetData(m, d)
    home = HOME if scene.layout == "top" else home_pose(scene)
    d.qpos[scene.qadr] = home
    d.ctrl[scene.act] = home
    d.ctrl[scene.grip_act] = 0.0
    if scene.mode == "ship":
        scene.set_ship(*motion(0.0))
    mujoco.mj_forward(m, d)

    ik = IK(scene)
    est = SocketEstimator(strategy, p, rng)
    tr = Trial()
    phase, depth_cmd, gate_t, hold_t = "approach", -p.hover, None, 0.0
    n_sub = int(round(CONTROL_DT / m.opt.timestep))
    t, t_end = 0.0, p.timeout
    f_ins, f_keep, min_keep_depth = [], [], np.inf
    while t < t_end:
        entrance, axis = scene.socket_frame()
        est.capture(t, entrance, axis)
        e = est.estimate(t)
        if e is not None:
            ent_hat, ax_hat, speed = e
            tip = d.site_xpos[scene.tip_site].copy()
            rel = tip - ent_hat
            lateral = np.linalg.norm(rel - np.dot(rel, ax_hat) * ax_hat)
            if phase == "approach":
                if lateral < p.align_tol and abs(np.dot(rel, ax_hat) - p.hover) < p.align_tol:
                    phase, gate_t = "gate", t
            if phase == "gate":
                ready = strategy != "predictive" or speed < p.gate_speed or t - gate_t > p.gate_timeout
                if ready:
                    phase = "insert"
            if phase in ("insert", "keep"):
                depth_cmd = min(depth_cmd + p.descend_speed * CONTROL_DT, p.depth_goal + 0.004)
            target = ent_hat - ax_hat * depth_cmd
            q_cmd = ik.solve(d.qpos[scene.qadr].copy(), target, -ax_hat)
            d.ctrl[scene.act] = q_cmd

        for _ in range(n_sub):
            if scene.mode == "ship":
                scene.set_ship(*motion(t))
            mujoco.mj_step(m, d)
            t += m.opt.timestep
        if on_step is not None:  # 학습 데이터 녹화용(제어 주기마다 한 번, 물리에는 영향 없음)
            on_step(t, phase)

        body_hit, plug_hull, force = contacts(scene)
        tr.forces.append(force)
        (f_keep if phase == "keep" else f_ins).append(force)
        entrance, axis = scene.socket_frame()
        rel = d.site_xpos[scene.tip_site] - entrance
        depth = -np.dot(rel, axis)
        lateral = np.linalg.norm(rel + depth * axis)
        if record is not None:
            record.append((t, d.qpos.copy(), d.mocap_pos.copy(), d.mocap_quat.copy(), phase, depth, lateral, force))
        if body_hit:
            tr.reason = "그리퍼가 소켓 테두리에 닿음" if body_hit == "socket" else "로봇 몸체 충돌"
            break
        if plug_hull:
            tr.reason = "플러그가 소켓 밖 선체에 부딪힘"
            break
        seated = depth >= p.depth_goal and lateral < BORE_R - PIN_R + 0.001
        if phase == "keep":
            min_keep_depth = min(min_keep_depth, depth)
            if depth < SOCKET_H - BORE_TOP:  # 핀 끝이 보어 위(원뿔 구간)로 올라옴
                tr.reason = "유지 중 빠짐"
                break
            continue
        if seated:
            hold_t += CONTROL_DT
            if hold_t >= p.hold:
                tr.inserted, tr.t_insert = True, t
                phase, t_end = "keep", t + p.keep
        else:
            hold_t = 0.0
    else:
        if tr.inserted:
            tr.f_hold_p95 = float(np.percentile(f_keep, 95)) if f_keep else 0.0
            tr.held = tr.f_hold_p95 <= p.force_limit
            tr.reason = "" if tr.held else "유지 중 과부하"
        else:
            tr.reason = "시간 초과(걸림, 정렬 실패)"
    tr.f_insert_p95 = float(np.percentile(f_ins, 95)) if f_ins else 0.0
    if tr.inserted and not tr.f_hold_p95 and f_keep:
        tr.f_hold_p95 = float(np.percentile(f_keep, 95))
    return tr
