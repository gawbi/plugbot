"""이종 원격조종(실시간): SO-101 리더 팔 관절 → 그리퍼 끝의 소켓 기준 상대 위치와 핀 방향. run_scaleup.py --live-leader 가 쓴다.

변환(isaac/scaleup/real_episode_traj.py와 같은 식):
  리더 모터 원값(호밍 오프셋이 모터에 저장돼 가운데 = 2048) → 리더 보정 범위로 -100..100 눈금
  → 팔로워 보정 범위로 팔로워 원값(팔로워가 실제로 따라갈 자세) → 각도 → MuJoCo 기구학(plugbot.scene) → 소켓 기준.
소켓 기준: teleop_socket_ref.json(학습 데이터의 가운데 자리 꽂힌 자세 평균, make_socket_ref.py). 리더 팔로 그 자세를 만들면 UR10e가 꽂힌다.

필요: scservo_sdk(pip install feetech-servo-sdk), mujoco, numpy. LeRobot은 필요 없다(Isaac 파이썬 3.11에서 돈다).
시험(Isaac 없이): python isaac/scaleup/leader_source.py /dev/ttyACM0        0.1 s마다 소켓까지 거리와 깊이 출력
         가짜 입력: python isaac/scaleup/leader_source.py fake:isaac/scaleup/sample_traj_ep54.npz (저장소에 들어 있는 실물 시범 54번)
"""

import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAL_L = json.loads((HERE / "calib" / "plug_leader.json").read_text())
CAL_F = json.loads((HERE / "calib" / "plug_follower.json").read_text())
REF = json.loads((HERE / "teleop_socket_ref.json").read_text())


def leader_raw_to_deg(raw):
    out = []
    for r, j in zip(raw, J):
        lo, hi = CAL_L[j]["range_min"], CAL_L[j]["range_max"]
        n = (np.clip(r, lo, hi) - lo) / (hi - lo) * 200 - 100
        flo, fhi = CAL_F[j]["range_min"], CAL_F[j]["range_max"]
        out.append((flo + (n + 100) / 200 * (fhi - flo) - 2048) * 360 / 4096)
    return np.array(out)


class Kin:
    def __init__(self):
        import mujoco
        sys.path.insert(0, str(ROOT))
        from plugbot.scene import Scene
        self.mj = mujoco
        self.kin = Scene("car", "side")
        self.m = self.kin.model
        self.d = mujoco.MjData(self.m)
        self.site = self.m.site("gripperframe").id
        self.ent = np.array(REF["entrance_m"])
        self.B = np.array(REF["basis"])  # 열: x, y, z(축 바깥)

    def rel_pin(self, deg):
        self.d.qpos[:] = 0
        self.d.qpos[self.kin.qadr] = np.deg2rad(deg)
        self.mj.mj_kinematics(self.m, self.d)
        tip = self.d.site_xpos[self.site].copy()
        jaw = tip - self.d.xpos[self.kin.gripper_body]
        jaw /= np.linalg.norm(jaw)
        return (tip - self.ent) @ self.B, jaw @ self.B


class LeaderSource:
    """read() → (rel, pin) 소켓 좌표계(모형 크기, m). 읽기 실패하면 마지막 값을 돌려준다."""

    def __init__(self, port):
        self.kin = Kin()
        self.last = None
        if port.startswith("fake:"):
            z = np.load(port[5:])
            self.fake = list(zip(z["t"], z["rel"], z["pin"]))
            self.t0 = time.time()
            return
        self.fake = None
        from scservo_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
        self.ok = COMM_SUCCESS
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise RuntimeError(f"{port} 열기 실패(권한: dialout 그룹, 다른 프로그램이 쓰는지)")
        self.port.setBaudRate(1000000)
        self.pk = PacketHandler(0)
        self.sync = GroupSyncRead(self.port, self.pk, 56, 2)  # Present_Position
        for i in range(1, 6):
            self.sync.addParam(i)

    def read(self):
        if self.fake is not None:
            t = time.time() - self.t0
            k = 0
            while k + 1 < len(self.fake) and self.fake[k + 1][0] <= t:
                k += 1
            return self.fake[k][1], self.fake[k][2]
        if self.sync.txRxPacket() == self.ok:
            raw = [self.sync.getData(i, 56, 2) for i in range(1, 6)]
            self.last = self.kin.rel_pin(leader_raw_to_deg(raw))
        if self.last is None:
            raise RuntimeError("리더 팔 읽기 실패(전원 5 V, 케이블, 모터 ID 1~5 확인)")
        return self.last

    def close(self):
        if self.fake is None:
            self.port.closePort()


if __name__ == "__main__":
    src = LeaderSource(sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0")
    try:
        while True:
            rel, pin = src.read()
            print(f"소켓까지 {np.linalg.norm(rel) * 1000:6.1f} mm   깊이 {-rel[2] * 1000:+6.1f} mm   옆 {np.linalg.norm(rel[:2]) * 1000:5.1f} mm", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        src.close()
