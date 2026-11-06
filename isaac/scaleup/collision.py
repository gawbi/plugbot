"""UR10e 실시간 원격조종의 안전 장치 세 가지. 가상 울타리(fence), 연속 추종(dls_step), 충돌 방지(Guard).

충돌 방지: 구(sphere)로 근사한 팔과 플러그가 장면, 자기 자신과 얼마나 떨어져 있는지 잰다.

팔의 구는 Isaac이 주는 UR10e 설명 파일(collision_spheres)을 그대로 쓴다. 물리 접촉 계산이 아니라 기하 근사다.
핀은 소켓 입구의 포착 반경 안이면 소켓과 현측 면을 검사하지 않고 보어 바닥만 본다(깔때기 포착 가정, README).
자체 시험(Isaac 없이): python isaac/scaleup/collision.py
"""

import numpy as np
from scipy.spatial.transform import Rotation

CHAIN = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
TOOL = len(CHAIN)  # 플러그와 카메라: tool0 프레임의 구
SELF_GAP = 3  # 사슬에서 이만큼 이상 떨어진 부위끼리만 자기 충돌을 본다(이웃 링크의 구는 원래 겹친다)


def fence(g, pin, lo, hi, cone):
    """소켓 좌표계의 핀 끝 목표 g와 핀 방향 pin을 안전 구역으로 제한한다(리더 팔을 아무렇게나 움직여도 UR10e는 이 안에 머문다).
    lo, hi: g의 범위(m), cone: 소켓 안쪽(-z)에서 벗어날 수 있는 핀 각도(rad)."""
    g = np.clip(g, lo, hi)
    pin = np.asarray(pin) / np.linalg.norm(pin)
    if np.arccos(np.clip(-pin[2], -1, 1)) > cone:
        side = np.array([pin[0], pin[1], 0.0])
        side /= np.linalg.norm(side) + 1e-9
        pin = np.cos(cone) * np.array([0, 0, -1.0]) + np.sin(cone) * side
    return g, pin


def dls_step(fk, q, pos_t, R_t, lam=0.05, iters=3, h=1e-4, step_max=0.3):
    """현재 관절에서 목표로 조금씩 다가가는 감쇠 최소제곱 IK. 전역 IK와 달리 해가 튀지 않고(자세가 뒤집히지 않고),
    못 닿는 목표에서는 가장 가까운 자세에 머문다. (관절, 목표에 닿았는지)"""
    q = q.copy()
    for _ in range(iters):
        p, R = map(np.asarray, fk("tool0", q))
        e = np.r_[pos_t - p, Rotation.from_matrix(R_t @ R.T).as_rotvec()]
        J = np.zeros((6, 6))
        for j in range(6):
            qq = q.copy()
            qq[j] += h
            p2, R2 = map(np.asarray, fk("tool0", qq))
            J[:3, j] = (p2 - p) / h
            J[3:, j] = Rotation.from_matrix(R2 @ R.T).as_rotvec() / h
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        m = np.max(np.abs(dq))
        q += dq * (min(1.0, step_max / m) if m > 0 else 1.0)
    p, R = map(np.asarray, fk("tool0", q))
    return q, bool(np.linalg.norm(pos_t - p) < 0.01 and np.linalg.norm(Rotation.from_matrix(R_t @ R.T).as_rotvec()) < 0.05)


def load_spheres(description_yaml):
    import yaml
    data = yaml.safe_load(open(description_yaml))["collision_spheres"]
    links = {name: spheres for item in data for name, spheres in item.items()}
    return {k: ([s["center"] for s in links[k]], [s["radius"] for s in links[k]]) for k in CHAIN}


class Guard:
    def __init__(self, fk, link_spheres, tool_body, tool_pin, cylinders, boxes, ship):
        """fk(frame, q) → (위치, 회전행렬). tool_body, tool_pin: tool0 프레임 [(x, y, z, r), ...].
        cylinders: {이름: (x, y, 반지름, 윗면 z)}, boxes: {이름: (중심, 반치수)}.
        ship: 선박 기준 좌표(쉴 때 world 값) dict(o0, hull_x, ent_x, ent_z, sock_r, capture_r, bottom_x)."""
        self.fk, self.cyl, self.box, self.ship = fk, cylinders, boxes, ship
        self.c = [np.array(link_spheres[k][0], float) for k in CHAIN]
        self.r = np.concatenate([np.array(link_spheres[k][1], float) for k in CHAIN] + [[s[3] for s in tool_body + tool_pin]])
        self.tool = np.array([s[:3] for s in tool_body + tool_pin], float)
        n = [len(c) for c in self.c] + [len(self.tool)]
        self.group = np.repeat(np.arange(TOOL + 1), n)
        self.is_pin = np.zeros(len(self.r), bool)
        self.is_pin[-len(tool_pin):] = True
        self.names = CHAIN + ["plug"]
        g = self.group
        self.pairs = np.argwhere(np.triu(np.abs(g[:, None] - g[None, :]) >= SELF_GAP))

    def spheres(self, q):
        out = []
        for k, c in zip(CHAIN, self.c):
            p, R = self.fk(k, q)
            out.append(np.asarray(p) + c @ np.asarray(R).T)
        p, R = self.fk("tool0", q)
        out.append(np.asarray(p) + self.tool @ np.asarray(R).T)
        return np.vstack(out)

    def clearance(self, q, ship_o, ship_R, captured=False):
        """(최소 여유 m, 어디와 어디). 음수면 겹침. captured: 핀이 포착된 동안은 깔때기가 옆 오차를 흡수한다고 보고 보어 바닥만 검사한다."""
        p = self.p = self.spheres(q)
        best = (np.inf, "")

        def take(d, what):
            nonlocal best
            i = int(np.argmin(d))
            if d[i] < best[0]:
                best = (float(d[i]), f"{self.names[self.group[i]]}-{what}")

        take(p[:, 2] - self.r, "floor")
        not_shoulder = self.group > 0  # 어깨는 받침 위에 얹혀 있다
        for name, (x, y, rad, top) in self.cyl.items():
            dr = np.hypot(p[:, 0] - x, p[:, 1] - y) - rad
            dz = p[:, 2] - top
            d = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)) - self.r
            take(np.where(not_shoulder | (name != "pedestal"), d, np.inf), name)
        for name, (center, half) in self.box.items():
            e = np.maximum(np.abs(p - center) - half, 0)
            take(np.linalg.norm(e, axis=1) - self.r, name)

        s = self.ship
        ps = (p - ship_o) @ ship_R + s["o0"]  # 선박 좌표로(ship_R의 역행렬은 전치)
        rax = np.hypot(ps[:, 1], ps[:, 2] - s["ent_z"])
        in_mouth = self.is_pin & (captured | (rax < s["capture_r"]))
        take(np.where(in_mouth, np.inf, s["hull_x"] - ps[:, 0] - self.r), "hull")
        take(np.where(in_mouth, s["bottom_x"] - ps[:, 0] - self.r, np.inf), "bore_bottom")
        dx = np.maximum(s["ent_x"] - ps[:, 0], 0)
        d = np.hypot(dx, np.maximum(rax - s["sock_r"], 0)) - self.r
        take(np.where(in_mouth, np.inf, d), "socket")

        i, j = self.pairs[:, 0], self.pairs[:, 1]
        d = np.linalg.norm(p[i] - p[j], axis=1) - self.r[i] - self.r[j]
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), f"{self.names[self.group[i[k]]]}-{self.names[self.group[j[k]]]}")
        return best

    def step(self, q, q_next, ship_o, ship_R, captured, margin):
        """q_next로 가도 되는지 본다. 여유가 margin보다 작으면 막되, 가장 가까운 대상이 그대로이고 좋아지는 움직임은 허용하고
        (배가 다가와 여유가 줄어도 갇히지 않게), 나머지는 관절별로 되는 만큼만 움직인다(장애물을 따라 미끄러지게).
        (실제로 갈 관절, 여유, 어디와 어디, 막혔는지)"""
        c, w = self.clearance(q_next, ship_o, ship_R, captured)
        if c >= margin:
            return q_next, c, w, False
        cq, wq = self.clearance(q, ship_o, ship_R, captured)
        if c >= cq and w == wq:
            return q_next, c, w, False
        qs = q.copy()
        for j in np.argsort(-np.abs(q_next - q)):
            tr = qs.copy()
            tr[j] = q_next[j]
            ct, wt = self.clearance(tr, ship_o, ship_R, captured)
            if ct >= margin or (ct >= cq and wt == wq):
                qs, cq, wq = tr, ct, wt
        return qs, cq, wq, True

    def cable_clearance(self, pts, cable_r, pedestal):
        """마지막 clearance()의 자세에서 케이블 점들과 팔(손목 끝과 플러그 제외, 케이블이 거기 붙는다), 받침의 최소 여유."""
        keep = self.group < TOOL - 1
        d = np.linalg.norm(pts[:, None] - self.p[keep], axis=2) - self.r[keep] - cable_r
        x, y, rad, top = pedestal
        low = pts[:, 2] < top
        d_ped = np.hypot(pts[low, 0] - x, pts[low, 1] - y) - rad - cable_r
        return float(min(d.min(), d_ped.min() if low.any() else np.inf))


if __name__ == "__main__":  # 곧게 위로 선 가짜 팔로 판정을 확인한다
    links = {k: ([[0.0, 0.0, 0.0]], [0.05]) for k in CHAIN}
    height = {k: 1.0 + 0.1 * i for i, k in enumerate(CHAIN)} | {"tool0": 2.3}
    x = [0.0]

    def fk(frame, q):
        return np.array([x[0] if frame == "tool0" else 0.0, 0.0, height[frame]]), np.eye(3)

    ship = dict(o0=np.zeros(3), hull_x=1.0, ent_x=0.8, ent_z=2.3, sock_r=0.15, capture_r=0.09, bottom_x=0.98)
    g = Guard(fk, links, [(0, 0, 0, 0.05)], [(0.3, 0, 0, 0.03)], {"pole": (-1.0, 0.0, 0.1, 5.0)},
              {"cab": (np.array([0.0, 2.0, 0.5]), np.array([0.2, 0.2, 0.5]))}, ship)
    same = (np.zeros(3), np.eye(3))
    x[0] = 0.6  # 핀(tool0 + 0.3 m)이 소켓 축 위로 입구를 지났다: 소켓과 부딪힌 것이 아니다
    c, w = g.clearance(None, *same)
    assert c > 0 and "socket" not in w, (c, w)
    x[0] = 0.7  # 핀 끝이 보어 바닥을 지난다
    c, w = g.clearance(None, *same)
    assert c < 0 and w == "plug-bore_bottom", (c, w)
    height["tool0"] = 2.6  # 입구 밖(포착 반경 밖)에서 현측에 닿는다: 핀도 검사한다
    x[0] = 0.75
    c, w = g.clearance(None, *same)
    assert c < 0 and w == "plug-hull", (c, w)
    assert g.clearance(None, *same, captured=True)[1] != "plug-hull"  # 포착 중이면 핀의 옆 오차로 멈추지 않는다
    height["tool0"], x[0] = 2.3, 0.85  # 몸통이 소켓 안으로 들어간 자세
    assert g.clearance(None, *same)[0] < 0
    x[0] = 0.0
    height["shoulder_link"] = 0.02
    assert g.clearance(None, *same)[1] == "shoulder_link-floor"
    assert g.clearance(None, np.array([0.5, 0, 0.0]), np.eye(3))[0] < 0.02  # 배 위치도 반영
    assert np.array_equal(fence(np.array([9.0, 0, -9.0]), (0, 0, -1.0), -np.ones(3), np.ones(3), 1.0)[0], [1.0, 0, -1.0])
    p = fence(np.zeros(3), (1.0, 0, 0), -np.ones(3), np.ones(3), np.deg2rad(60))[1]  # 옆을 보는 핀은 60°까지만
    assert abs(np.arccos(-p[2]) - np.deg2rad(60)) < 1e-9 and p[0] > 0
    height["shoulder_link"] = 1.0

    def fk2(frame, q):  # q[0]: 내려가면 바닥, q[1]: 옆으로만
        return np.array([q[1], 0.0, q[0] + height[frame]]), np.eye(3)

    g2 = Guard(fk2, links, [(0, 0, 0, 0.05)], [(0, 0, 0.1, 0.03)], {}, {}, dict(ship, hull_x=9.0, ent_x=8.0, ent_z=99.0))
    q0, qn, _, held = np.array([0.0, 0.0]), np.array([-1.0, 0.5]), 0, 0
    qr, c, _, held = g2.step(q0, qn, *same, False, 0.03)
    assert held and qr[0] == 0.0 and qr[1] == 0.5 and c >= 0.03, (qr, c)  # 바닥 쪽은 막고 옆으로는 미끄러진다
    x0, r0 = np.array([0.3, -0.2, 0.1, 0.2, 0.1, 0.0]), np.eye(3)

    def fk3(frame, q):  # 관절 3개는 위치, 3개는 회전으로 바로 가는 팔
        return q[:3], Rotation.from_rotvec(q[3:]).as_matrix()

    q3, ok = dls_step(fk3, x0, np.array([0.5, 0.0, 0.2]), Rotation.from_rotvec([0.0, 0.3, 0.0]).as_matrix(), iters=8)
    assert ok and np.linalg.norm(q3[:3] - [0.5, 0.0, 0.2]) < 0.01, q3
    print("collision.py 자체 시험 통과")
