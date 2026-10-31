"""축소 모형용 불규칙파 선체 운동.

JONSWAP 스펙트럼(γ = 3.3)을 성분 파 여러 개로 나눠 상하(heave) 변위를 만들고, 같은 성분에 위상을 달리해
좌우(sway)·롤·피치를 만든다. 크기는 유의파고 Hs(상하 변위 표준편차의 4배)로 맞춘다.
실제 선박 운동 응답(RAO)이 아니라 축소 실험용 근사다.
"""

from dataclasses import dataclass

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
    sway_ratio: float = 0.8  # 계류 여유로 좌우·앞뒤로도 움직인다
    surge_ratio: float = 0.6

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
        """(변위 [dx, dy, dz], 롤, 피치)"""
        heave = self._sum(t, 0)
        sway = self.sway_ratio * self._sum(t, 1)
        roll = self.roll_per_m * self._sum(t, 2)
        pitch = self.pitch_per_m * self._sum(t, 3)
        surge = self.surge_ratio * self._sum(t, 4)
        return np.array([surge, sway, heave]), roll, pitch
