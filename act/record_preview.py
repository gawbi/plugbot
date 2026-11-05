"""lerobot-record를 그대로 돌리되 두 가지를 덧붙인다(act/record.sh가 PREVIEW=1일 때 이것을 부른다).

1. 카메라 미리보기: 팔로워가 읽은 카메라 영상을 약 15 fps로 /dev/shm/plugbot_cam/<이름>.jpg에 쓴다.
   녹화 화면(act/record_gui.py)이 이것을 보여 준다(카메라는 녹화 프로그램이 쓰고 있어 다른 프로그램이 직접 열 수 없다).
2. 그리퍼 하한(GRIP_MIN, 기본 6): 그리퍼 덮개를 문 턱은 약 8(0~100 눈금, 0 = 완전히 닫힘)에서 칸막이에 닿는다.
   리더 손잡이를 닫으면 팔로워가 0까지 닫으려고 계속 눌러 6번 모터가 과부하로 멈췄다(2026-11-05).
   리더 값을 GRIP_MIN 아래로 내려가지 않게 잘라, 기록되는 행동과 보내는 명령이 같고 칸막이는 살짝만 누른다.
"""

import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from lerobot.robots.so_follower import SOFollower
from lerobot.scripts import lerobot_record
from lerobot.teleoperators.so_leader import SOLeader

SHM = Path("/dev/shm/plugbot_cam")
SHM.mkdir(parents=True, exist_ok=True)
GRIP_MIN = float(os.environ.get("GRIP_MIN", "6"))
_last = [0.0]

_get_obs = SOFollower.get_observation
_get_action = SOLeader.get_action
_send = SOFollower.send_action


def get_observation(self):
    obs = _get_obs(self)
    now = time.time()
    if now - _last[0] > 1 / 15:
        _last[0] = now
        for k, v in obs.items():
            if isinstance(v, np.ndarray) and v.ndim == 3:
                ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(v, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    tmp = SHM / f"{k}.tmp"
                    tmp.write_bytes(jpg.tobytes())
                    tmp.replace(SHM / f"{k}.jpg")
    return obs


def _clamp(action):
    if "gripper.pos" in action and action["gripper.pos"] < GRIP_MIN:
        action = dict(action)
        action["gripper.pos"] = GRIP_MIN
    return action


def get_action(self):
    return _clamp(_get_action(self))


def send_action(self, action):
    return _send(self, _clamp(action))


SOFollower.get_observation = get_observation
SOLeader.get_action = get_action
SOFollower.send_action = send_action

if __name__ == "__main__":
    sys.argv = ["lerobot-record", *sys.argv[1:]]
    print(f"[plugbot] 카메라 미리보기 {SHM}, 그리퍼 하한 {GRIP_MIN}", flush=True)
    lerobot_record.main()
