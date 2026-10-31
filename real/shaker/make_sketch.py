"""흔들림 갑판용 아두이노 코드 만들기. 시뮬레이션(plugbot.waves.HeaveProfile)과 같은 운동을 서보로 재생한다.

실행: .venv/bin/python -m real.shaker.make_sketch [--hs 0.02 --period 2.5 --seed 0]
결과: real/shaker/shaker/shaker.ino  (아두이노 IDE로 열어 나노/우노에 올린다)

배선: 서보 신호선 → D9, 서보 전원(+, -) → 5~6 V 2 A 어댑터, 어댑터 -(GND) ↔ 아두이노 GND (공통 접지 필수).
기구: 서보 혼 구멍(반지름 HORN_MM) ─ 연결 막대(세로) ─ 갑판 레버 구멍(힌지에서 120/130/140/150 mm).
"""

import argparse
from pathlib import Path

import numpy as np

from plugbot.waves import HINGE_TO_SOCKET, HeaveProfile

OUT = Path(__file__).resolve().parent / "shaker" / "shaker.ino"

TEMPLATE = r"""// PlugBot 흔들림 갑판 (자동 생성: real/shaker/make_sketch.py, 직접 고치지 말고 다시 생성)
// 시뮬레이션 plugbot.waves.HeaveProfile과 같은 식으로 소켓 상하 변위 h(t)를 만들고, 서보 각도로 바꾼다.
// 시리얼(115200): s=정현파  w=불규칙파(시뮬레이션과 같음)  h=멈춤(가운데)  +/-=진폭 1 mm  p2.5=주기(s)  ?=상태
#include <Servo.h>

const int SERVO_PIN = 9;
const int SERVO_DIR = 1;          // 소켓이 반대로 움직이면 -1
const float CENTER_US = 1500;     // 서보 가운데(레버가 수평일 때). 조립 후 h 모드에서 맞춘다
const float US_PER_DEG = 1000.0 / 90.0;
const float MAX_DEG = 60;         // 서보 최대 편차(기구 보호)

// 기구 치수(mm): cad/make_parts.py
const float SOCKET_ARM_MM = __SOCKET_ARM__;  // 힌지 → 소켓 중심
float LEVER_ARM_MM = 120.0;       // 힌지 → 연결 막대를 꽂은 레버 구멍(120/130/140/150)
float HORN_MM = 20.0;             // 서보 축 → 혼 구멍

// 흔들림
char mode = 'h';
float ampMM = __AMP__;            // 정현파 진폭(mm)
float periodS = __PERIOD__;       // 정현파 주기(s)
const int N = __N__;              // 불규칙파 성분 수(JONSWAP, 유의파고 __HS_CM__ cm, 시드 __SEED__)
const float W[N] = {__W__};       // rad/s
const float A[N] = {__A__};       // mm
const float PH[N] = {__PH__};     // rad

Servo servo;
unsigned long t0;
float ramp = 0;                   // 시작, 모드 변경 때 3초에 걸쳐 진폭을 키운다

float heaveMM(float t) {
  if (mode == 's') return ampMM * sin(2 * PI * t / periodS);
  if (mode == 'w') { float h = 0; for (int i = 0; i < N; i++) h += A[i] * sin(W[i] * t + PH[i]); return h; }
  return 0;
}

void setMode(char m) { mode = m; ramp = 0; t0 = millis(); }

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(CENTER_US);
  t0 = millis();
  Serial.println(F("PlugBot shaker: s=sine w=wave h=hold +/- amp p<sec> ?"));
}

void loop() {
  static unsigned long next = 0;
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'w' || c == 'h') setMode(c);
    else if (c == '+') ampMM += 1;
    else if (c == '-') ampMM = max(0.0, ampMM - 1);
    else if (c == 'p') { float p = Serial.parseFloat(); if (p > 0.5) periodS = p; }
    else if (c == '?') { Serial.print(F("mode ")); Serial.print(mode); Serial.print(F(" amp ")); Serial.print(ampMM);
                         Serial.print(F(" period ")); Serial.println(periodS); }
  }
  unsigned long now = millis();
  if (now < next) return;
  next = now + 20;                // 50 Hz
  float t = (now - t0) / 1000.0;
  ramp = min(1.0, ramp + 0.02 / 3.0);
  float h = ramp * heaveMM(t);                                   // 소켓 상하(mm)
  float lever = h * LEVER_ARM_MM / SOCKET_ARM_MM;                // 레버 구멍 상하(mm)
  float s = constrain(lever / HORN_MM, -0.95, 0.95);
  float deg = constrain(degrees(asin(s)), -MAX_DEG, MAX_DEG);
  servo.writeMicroseconds(CENTER_US + SERVO_DIR * deg * US_PER_DEG);
  static int k = 0;
  if (++k % 5 == 0) { Serial.print(t, 2); Serial.print(','); Serial.println(h, 2); }  // 시각(s), 소켓 상하(mm)
}
"""


def fmt(xs, digits=5):
    return ", ".join(f"{x:.{digits}f}" for x in xs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hs", type=float, default=0.02, help="불규칙파 유의파고(m). 소켓 상하 표준편차 = hs/4")
    ap.add_argument("--amp", type=float, default=0.008, help="정현파 진폭(m)")
    ap.add_argument("--period", type=float, default=2.5)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    prof = HeaveProfile(mode="wave", hs=a.hs, period=a.period, seed=a.seed)
    rep = {"__SOCKET_ARM__": f"{HINGE_TO_SOCKET * 1000:.1f}", "__AMP__": f"{a.amp * 1000:.1f}",
           "__PERIOD__": f"{a.period:.2f}", "__N__": str(len(prof.w)), "__HS_CM__": f"{a.hs * 100:.1f}",
           "__SEED__": str(a.seed), "__W__": fmt(prof.w), "__A__": fmt(prof.a * 1000), "__PH__": fmt(prof.ph)}
    text = TEMPLATE
    for k, v in rep.items():
        text = text.replace(k, v)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    # 검증: 아두이노와 같은 식을 파이썬으로 계산해 시뮬레이션 프로파일과 비교
    t = np.linspace(0, 30, 1501)
    ino = np.array([np.sum(prof.a * np.sin(prof.w * tt + prof.ph)) for tt in t])
    sim = np.array([prof(tt) for tt in t])
    lever = ino * 1000 * 120 / (HINGE_TO_SOCKET * 1000)
    print(OUT)
    print(f"불규칙파: 소켓 상하 {ino.min() * 1000:.1f}~{ino.max() * 1000:.1f} mm(표준편차 {ino.std() * 1000:.1f}), "
          f"시뮬레이션과 차 {np.abs(ino - sim).max() * 1000:.4f} mm, 레버 120 mm 구멍 최대 {np.abs(lever).max():.1f} mm")
    print(f"정현파: 진폭 {a.amp * 1000:.1f} mm, 주기 {a.period} s → 혼 20 mm일 때 서보 최대 "
          f"{np.degrees(np.arcsin(min(0.95, a.amp * 1000 * 120 / (HINGE_TO_SOCKET * 1000) / 20))):.0f}°")


if __name__ == "__main__":
    main()
