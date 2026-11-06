"""흔들림 보드 통신 시험: ping과 현재 위치, 전압, 온도만 읽는다. 토크를 켜지 않고 모터를 움직이지 않는다.

실행: ~/shaker_env/bin/python ~/plugbot/act/shaker_ping.py [반복 초]
반복 초를 주면 그동안 0.2 s마다 위치를 출력한다(손으로 갑판을 움직여 범위를 잴 때).
"""

import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__file__))
from deck_shaker import CENTER, TICK, find_port, k  # noqa: E402

p = find_port()
try:
    pos, res, err = k.read2ByteTxRx(p, 1, 56)
    volt, _, _ = k.read1ByteTxRx(p, 1, 62)
    temp, _, _ = k.read1ByteTxRx(p, 1, 63)
    torque, _, _ = k.read1ByteTxRx(p, 1, 40)
    print(f"position {pos} (CENTER {CENTER}, {(pos - CENTER) / TICK:+.1f} deg)  voltage {volt / 10:.1f} V  temp {temp} C  torque {torque}")
    end = time.time() + (float(sys.argv[1]) if len(sys.argv) > 1 else 0)
    lo = hi = pos
    while time.time() < end:
        pos, res, _ = k.read2ByteTxRx(p, 1, 56)
        if res == 0:
            lo, hi = min(lo, pos), max(hi, pos)
            print(f"{pos}  {(pos - CENTER) / TICK:+6.1f} deg   min {lo} max {hi}", flush=True)
        time.sleep(0.2)
finally:
    p.closePort()
