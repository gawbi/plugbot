// PlugBot 흔들림 갑판 (자동 생성: real/shaker/make_sketch.py, 직접 고치지 말고 다시 생성)
// 시뮬레이션 plugbot.waves.HeaveProfile과 같은 식으로 소켓 상하 변위 h(t)를 만들고, 서보 각도로 바꾼다.
// 시리얼(115200): s=정현파  w=불규칙파(시뮬레이션과 같음)  h=멈춤(가운데)  +/-=진폭 1 mm  p2.5=주기(s)  ?=상태
#include <Servo.h>

const int SERVO_PIN = 9;
const int SERVO_DIR = 1;          // 소켓이 반대로 움직이면 -1
const float CENTER_US = 1500;     // 서보 가운데(레버가 수평일 때). 조립 후 h 모드에서 맞춘다
const float US_PER_DEG = 1000.0 / 90.0;
const float MAX_DEG = 60;         // 서보 최대 편차(기구 보호)

// 기구 치수(mm): cad/make_parts.py
const float SOCKET_ARM_MM = 65.0;  // 힌지 → 소켓 중심
float LEVER_ARM_MM = 120.0;       // 힌지 → 연결 막대를 꽂은 레버 구멍(120/130/140/150)
float HORN_MM = 20.0;             // 서보 축 → 혼 구멍

// 흔들림
char mode = 'h';
float ampMM = 8.0;            // 정현파 진폭(mm)
float periodS = 2.50;       // 정현파 주기(s)
const int N = 16;              // 불규칙파 성분 수(JONSWAP, 유의파고 2.0 cm, 시드 0)
const float W[N] = {1.50796, 1.82631, 2.14466, 2.46301, 2.78136, 3.09970, 3.41805, 3.73640, 4.05475, 4.37310, 4.69145, 5.00979, 5.32814, 5.64649, 5.96484, 6.28319};       // rad/s
const float A[N] = {0.13181, 1.07908, 2.23309, 4.32793, 3.14175, 2.10728, 1.76455, 1.49155, 1.25997, 1.06844, 0.91141, 0.78270, 0.67680, 0.58916, 0.51616, 0.45492};       // mm
const float PH[N] = {4.00215, 1.69512, 0.25744, 0.10385, 5.10993, 5.73501, 3.81160, 4.58356, 3.41570, 5.87523, 5.12616, 0.01721, 5.38723, 0.21102, 4.58456, 1.10368};     // rad

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
