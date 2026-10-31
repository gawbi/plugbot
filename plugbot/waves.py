"""축소 모형용 불규칙파 선체 운동.

JONSWAP 스펙트럼(γ = 3.3)을 성분 파 여러 개로 나눠 상하(heave) 변위를 만들고, 같은 성분에 위상을 달리해
좌우(sway), 앞뒤(surge), 롤, 피치를 만든다. 크기는 유의파고 Hs(상하 변위 표준편차의 4배)로 맞춘다.
실제 선박 운동 응답(RAO)이 아니라 축소 실험용 근사다.

계류 방향(berth):
- "bow": 선수를 부두(로봇) 쪽으로 댄 배(예선 실험). 롤 축 = 장면 x축(로봇→선박), 피치 축 = y축.
- "alongside": 부두에 옆으로 댄 배(여객선, 페리의 보통 계류). 롤 축 = 부두와 나란한 y축이라
  좌우 흔들림(롤)이 부두 쪽 현측 소켓을 위아래로 움직이고 소켓 축을 기울인다. 실물 힌지 갑판(real/shaker)도 y축 회전이다.
"""

from dataclasses import dataclass, field

import numpy as np


def jonswap(omega, wp, gamma=3.3):
    sigma = np.where(omega <= wp, 0.07, 0.09)
    r = np.exp(-((omega - wp) ** 2) / (2 * sigma**2 * wp**2))
    return omega**-5 * np.exp(-1.25 * (wp / omega) ** 4) * gamma**r


@dataclass
class ShipMotion:
    hs: float  # 유의파고(m), 상하 변위 표준편차의 4배
    tp: float = 2.0  # 첨두 주기(s), 축소 비율에 맞춘 값
    seed: int = 0
    n: int = 16
    roll_per_m: float = np.deg2rad(60.0)  # 상하 표준편차 1 cm당 롤 표준편차 0.6°
    pitch_per_m: float = np.deg2rad(30.0)
    sway_ratio: float = 0.8  # 계류 여유로 좌우, 앞뒤로도 움직인다
    surge_ratio: float = 0.6
    berth: str = "bow"

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        wp = 2 * np.pi / self.tp
        self.w = np.linspace(0.6 * wp, 2.5 * wp, self.n)
        s = jonswap(self.w, wp)
        a = np.sqrt(2 * s * (self.w[1] - self.w[0]))
        std = np.sqrt(np.sum(a**2) / 2)
        self.a = a * (self.hs / 4) / std if self.hs > 0 else a * 0
        self.ph = rng.uniform(0, 2 * np.pi, (5, self.n))

    def _sum(self, t, k):
        return float(np.sum(self.a * np.sin(self.w * t + self.ph[k])))

    def __call__(self, t):
        """Scene.set_ship에 넣는 (변위 [dx, dy, dz], x축 회전, y축 회전). 장면 x는 로봇→선박, y는 부두와 나란한 방향."""
        heave = self._sum(t, 0)
        sway = self.sway_ratio * self._sum(t, 1)
        roll = self.roll_per_m * self._sum(t, 2)
        pitch = self.pitch_per_m * self._sum(t, 3)
        surge = self.surge_ratio * self._sum(t, 4)
        if self.berth == "alongside":  # 선체 길이 방향 = y: 좌우(sway)는 x, 롤은 y축 회전
            return np.array([sway, surge, heave]), pitch, roll
        return np.array([surge, sway, heave]), roll, pitch


# ---------- 실물 흔들림 갑판(힌지 + 서보, real/shaker)과 같은 운동 ----------
# 갑판판은 로봇 쪽 힌지 축을 중심으로 기울고, 서보가 반대쪽 레버를 올렸다 내린다(cad/make_parts.py 치수).
HINGE_TO_SOCKET = 0.065  # 힌지 축에서 소켓 중심까지 수평 거리(m)
HINGE_BELOW_SOCKET = 0.019  # 힌지 축이 소켓 바닥보다 낮은 높이(m)
HINGE_BELOW_SIDE_SOCKET = 0.104  # side 배치: 세운 판 소켓 중심(갑판판 위 85 mm)이 힌지 축보다 높은 거리(m)


@dataclass
class HeaveProfile:
    """소켓의 상하 변위 h(t)(m). 아두이노 코드(real/shaker)가 같은 식을 계산한다.

    mode "sine": 진폭 amp, 주기 period의 정현파(ACT 시범처럼 규칙적인 흔들림)
    mode "wave": JONSWAP 불규칙파(ShipMotion의 상하 성분과 같음). 표준편차 = hs / 4
    """
    mode: str = "sine"
    amp: float = 0.008
    period: float = 2.5
    hs: float = 0.02
    seed: int = 0

    def __post_init__(self):
        sm = ShipMotion(self.hs, tp=self.period, seed=self.seed)
        self.w, self.a, self.ph = sm.w, sm.a, sm.ph[0]

    def __call__(self, t):
        if self.mode == "sine":
            return self.amp * np.sin(2 * np.pi * t / self.period)
        if self.mode == "wave":
            return float(np.sum(self.a * np.sin(self.w * t + self.ph)))
        return 0.0


@dataclass
class HingeDeckMotion:
    """Scene.set_ship에 넣는 (변위, x축 회전, y축 회전). 힌지(로봇 쪽, -x)를 축으로 갑판이 기운다.
    side 배치는 socket_on_ship=scene.SOCKET_SIDE_ON_SHIP, hinge_below=HINGE_BELOW_SIDE_SOCKET."""
    profile: HeaveProfile = field(default_factory=HeaveProfile)
    socket_on_ship: tuple = (-0.10, 0.0, 0.0)  # plugbot.scene.SOCKET_ON_SHIP
    hinge_below: float = HINGE_BELOW_SOCKET

    def __call__(self, t):
        theta = np.arcsin(np.clip(self.profile(t) / HINGE_TO_SOCKET, -0.9, 0.9))  # 소켓이 올라가면 +
        pitch = -theta  # y축 회전: +x(소켓 쪽)가 올라가려면 음의 피치
        h = np.array(self.socket_on_ship) + [-HINGE_TO_SOCKET, 0.0, -self.hinge_below]  # 힌지(선박 좌표)
        c, s = np.cos(pitch), np.sin(pitch)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return h - R @ h, 0.0, pitch  # 선박 원점이 힌지를 중심으로 돈 만큼의 변위
