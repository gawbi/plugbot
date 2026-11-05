"""lerobot-record(또는 --plugbot-entry=rollout이면 lerobot-rollout)를 그대로 돌리되 덧붙인다(act/record.sh PREVIEW=1, 녹화 화면의 로봇 실행).

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
from lerobot.motors.feetech.feetech import FeetechMotorsBus
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


# 3. 토크 켜기, 끄기 쓰기를 한 번만 시도해서, 패킷 하나가 빠지면 연결 단계에서 죽었다(4번 모터 'Lock', 2026-11-05). 3번까지 다시 시도
_en, _dis = FeetechMotorsBus.enable_torque, FeetechMotorsBus.disable_torque
FeetechMotorsBus.enable_torque = lambda self, motors=None, num_retry=0: _en(self, motors, max(num_retry, 3))
FeetechMotorsBus.disable_torque = lambda self, motors=None, num_retry=0: _dis(self, motors, max(num_retry, 3))

SOFollower.get_observation = get_observation
SOLeader.get_action = get_action
SOFollower.send_action = send_action

if __name__ == "__main__":
    # --plugbot-entry=rollout 이면 학습한 정책 실행(lerobot-rollout)에 같은 덧붙임(미리보기, 그리퍼 하한, 토크 재시도)을 쓴다
    entry = "record"
    args = [a for a in sys.argv[1:] if not a.startswith("--plugbot-entry=")]
    for a in sys.argv[1:]:
        if a.startswith("--plugbot-entry="):
            entry = a.split("=", 1)[1]
    print(f"[plugbot] {entry}: 카메라 미리보기 {SHM}, 그리퍼 하한 {GRIP_MIN}", flush=True)
    if entry == "rollout":
        from lerobot.scripts import lerobot_rollout

        sys.argv = ["lerobot-rollout", *args]
        lerobot_rollout.main()
    else:
        sys.argv = ["lerobot-record", *args]
        lerobot_record.main()
