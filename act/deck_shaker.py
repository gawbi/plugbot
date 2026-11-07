"""흔들림 갑판: STS3215(ID 1) 한 개로 갑판을 힌지 둘레로 흔든다. 윈도우용 코드를 우분투로 옮김(2026-11-06).

실행(키 입력을 받으므로 사용자가 직접):
    source ~/shaker_env/bin/activate
    python ~/plugbot/act/deck_shaker.py
키(입력 후 Enter): s 정현파 / w 불규칙파 / h 가운데 멈춤 / + - 진폭 1 mm(최대 12) / p2.5 주기(s) / q 끝
주의: 실행하자마자 토크를 켜고 정현파 진폭 12 mm로 움직인다.

윈도우판과 달라진 것:
- PORT 기본값: 흔들림 보드(시리얼 5B91044141)의 /dev/serial/by-id 경로
- 자동 탐색은 /dev/ttyUSB*, /dev/ttyACM*만, 그리고 로봇 팔 보드 두 개는 건너뛴다.
  팔로워와 리더의 어깨 모터도 ID 1이라, 그대로 두면 팔 보드에서 ping이 먼저 성공해 팔 모터를 흔들 수 있다.
- 포트마다 열기, ping 오류를 출력한다(권한 오류가 숨지 않게).
- Ctrl+C나 오류로 끝나도 CENTER로 되돌리고 토크를 끈 뒤 포트를 닫는다(try/finally).
CENTER, LEVER, SOCKET, HORN, 파형(W, A, P), 진폭과 각도 제한, 키 동작은 윈도우판과 같다.
"""

import glob
import math
import os
import sys
import threading
import time

from scservo_sdk import PacketHandler, PortHandler

PORT = os.environ.get("SHAKER_PORT", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B91044141-if00")
ROBOT_SERIALS = ("5AE6083026", "5B79015054")  # 팔로워, 리더 보드(act/env.sh). 자동 탐색에서 뺀다
CENTER = 3285          # servo center
LEVER = 200.0          # hinge -> deck tab (mm)
SOCKET = 190.0         # hinge -> socket (mm)
HORN = 20.0            # horn arm (mm)
TICK = 4096 / 360.0

W = [1.508, 1.826, 2.145, 2.463, 2.781, 3.100, 3.418, 3.736,
     4.055, 4.373, 4.691, 5.010, 5.328, 5.646, 5.965, 6.283]
A = [0.132, 1.079, 2.233, 4.328, 3.142, 2.107, 1.765, 1.492,
     1.260, 1.068, 0.911, 0.783, 0.677, 0.589, 0.516, 0.455]
P = [4.002, 1.695, 0.257, 0.104, 5.110, 5.735, 3.812, 4.584,
     3.416, 5.875, 5.126, 0.017, 5.387, 0.211, 4.585, 1.104]

k = PacketHandler(0)


def candidates():
    """PORT를 먼저, 그다음 /dev/ttyUSB*, /dev/ttyACM* 중 로봇 팔 보드가 아닌 것."""
    from serial.tools import list_ports
    serial_of = {x.device: (x.serial_number or "") for x in list_ports.comports()}
    names = [PORT]
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if os.path.realpath(dev) == os.path.realpath(PORT):
            continue
        if serial_of.get(dev, "") in ROBOT_SERIALS:
            print(f"skip {dev}: robot arm board ({serial_of[dev]})")
            continue
        names.append(dev)
    return names


def find_port():
    for name in candidates():
        if not os.path.exists(name):
            print(f"{name}: not present")
            continue
        q = None
        try:
            q = PortHandler(name)
            if not q.openPort():
                print(f"{name}: openPort failed")
                continue
            q.setBaudRate(1000000)
            model, res, err = k.ping(q, 1)
            if res == 0:
                print("servo found on", name, "model", model)
                return q
            print(f"{name}: ping id 1 failed ({k.getTxRxResult(res)})")
            q.closePort()
        except Exception as e:  # 권한 오류 등을 숨기지 않고 보여 준다
            print(f"{name}: {type(e).__name__}: {e}")
            if q is not None:
                try:
                    q.closePort()
                except Exception:
                    pass
    sys.exit("servo not found: check USB cable, 5V adapter, jumper on B")


# 시작 진폭과 주기: 기본은 윈도우판과 같은 12 mm, 2.5 s. 녹화 화면의 [세게 흔들기]는 SHAKER_AMP=16 으로 켠다(2026-11-06).
# 16 mm면 서보 약 57°(제한 60° 바로 안쪽. 17 mm부터 60°에 걸려 잘림). + 키 상한은 시작 진폭과 12 중 큰 값
AMP0 = float(os.environ.get("SHAKER_AMP", "12"))
PER0 = float(os.environ.get("SHAKER_PERIOD", "2.5"))
AMP_MAX = max(12.0, AMP0)
state = {"mode": "s", "amp": AMP0, "per": PER0, "run": True}


def keys():
    for line in sys.stdin:
        c = line.strip()
        if c in ("s", "w", "h"):
            state["mode"] = c
        elif c == "+":
            state["amp"] = min(AMP_MAX, state["amp"] + 1)
        elif c == "-":
            state["amp"] = max(0, state["amp"] - 1)
        elif c.startswith("p") and len(c) > 1:
            state["per"] = float(c[1:])
        elif c == "q":
            state["run"] = False
            return
        print("mode", state["mode"], " amp", state["amp"], "mm  period", state["per"], "s")


LIVE = "/dev/shm/plugbot_shaker.json"  # 부스 화면(act/booth.py)이 읽는다: 명령 높이와 서보가 실제로 간 위치로 계산한 높이


def live(p, t, h_cmd):
    """서보 현재 위치(주소 56)를 읽어 소켓 높이(mm)로 바꿔 적는다. 실패해도 흔들기는 계속한다."""
    try:
        raw, res, _ = k.read2ByteTxRx(p, 1, 56)
        meas = None
        if res == 0:
            deg = (raw - CENTER) / TICK
            meas = HORN * math.sin(math.radians(deg)) * SOCKET / LEVER
        tmp = LIVE + ".tmp"
        with open(tmp, "w") as f:
            f.write('{"wall": %.3f, "t": %.3f, "cmd_mm": %.2f, "meas_mm": %s, "mode": "%s", "amp": %.1f, "per": %.2f}'
                    % (time.time(), t, h_cmd, "null" if meas is None else "%.2f" % meas, state["mode"], state["amp"], state["per"]))
        os.replace(tmp, LIVE)
    except Exception:
        pass


def main():
    p = find_port()
    try:
        k.write1ByteTxRx(p, 1, 40, 1)
        threading.Thread(target=keys, daemon=True).start()
        print("keys: s sine / w waves / h hold / + - amp / p2.5 period / q quit")
        print("start amp", state["amp"], "mm  period", state["per"], "s")
        t0 = time.time()
        while state["run"]:
            t = time.time() - t0
            if state["mode"] == "s":
                h = state["amp"] * math.sin(2 * math.pi * t / state["per"])
            elif state["mode"] == "w":
                h = sum(A[i] * math.sin(W[i] * t + P[i]) for i in range(16)) * state["amp"] / 8
            else:
                h = 0.0
            lift = LEVER * h / SOCKET
            deg = math.degrees(math.asin(max(-1, min(1, lift / HORN))))
            deg = max(-60, min(60, deg))
            pos = int(CENTER + deg * TICK)
            p.clearPort()
            k.write2ByteTxRx(p, 1, 42, pos)
            live(p, t, h)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        # 어떻게 끝나든 가운데로 되돌리고 토크를 끈 뒤 포트를 닫는다
        try:
            p.clearPort()
            k.write2ByteTxRx(p, 1, 42, CENTER)
            time.sleep(0.5)
            k.write1ByteTxRx(p, 1, 40, 0)
        except Exception as e:
            print(f"cleanup error: {type(e).__name__}: {e}")
        p.closePort()
        print("done")


if __name__ == "__main__":
    main()
