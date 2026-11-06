"""현장에서 부스를 다시 세운 뒤 소켓 자리 확인: 지금 팔로워 자세(리더로 소켓에 꽂아 둔 상태)의 그리퍼 끝 위치를
학습 데이터의 꽂힌 위치(자리별 평균)와 비교해 몇 mm 어긋났는지 알려 준다. 모터에 명령을 보내지 않고 읽기만 한다.

기준: plug_static 54번 이후(2026-11-06 흔들림 갑판을 단 배치)의 마지막 프레임, 기구학은 isaac/scaleup/real_episode_traj.py와 같다.
실행(녹화와 로봇 실행이 꺼져 있을 때): ~/lerobot_env/bin/python act/check_placement.py [자리 이름, 기본 가운데]
좌표: x = 로봇 앞쪽, y = 로봇 왼쪽, z = 위 (로봇 베이스 기준, mm)
"""

import glob
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from plugbot.scene import Scene  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
FIRST_NEW = 54  # 이 번호부터 지금 배치
CAL = json.loads(Path(glob.glob(str(Path.home() / ".cache/huggingface/lerobot/calibration/robots/*/plug_follower.json"))[0]).read_text())
kin = Scene("car", "side")
m, d = kin.model, mujoco.MjData(kin.model)
site = m.site("gripperframe").id


def tip(n):
    deg = [(CAL[j]["range_min"] + (v + 100) / 200 * (CAL[j]["range_max"] - CAL[j]["range_min"]) - 2048) * 360 / 4096 for v, j in zip(n, J)]
    d.qpos[:] = 0
    d.qpos[kin.qadr] = np.deg2rad(deg)
    mujoco.mj_kinematics(m, d)
    return d.site_xpos[site] * 1000


def reference():
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    r = Path.home() / ".cache/huggingface/lerobot/local/plug_static"
    ds = LeRobotDataset("local/plug_static", root=r)
    e = ds.meta.episodes
    pos = json.loads((r / "plugbot_positions.json").read_text())
    s = np.stack(ds.hf_dataset["observation.state"])
    ref = {}
    for i in range(FIRST_NEW, ds.num_episodes):
        ref.setdefault(pos.get(str(i), "?"), []).append(tip(s[int(e["dataset_to_index"][i]) - 1][:5]))
    return {k: np.array(v) for k, v in ref.items()}


def read_now():
    from lerobot.robots.so_follower import SO101FollowerConfig, SOFollower
    rob = SOFollower(SO101FollowerConfig(port="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6083026-if00", id="plug_follower", cameras={}))
    rob.bus.connect()
    rob.bus.calibration = rob.calibration
    st = rob.bus.sync_read("Present_Position")
    rob.bus.port_handler.closePort()
    return np.array([st[j] for j in J])


if __name__ == "__main__":
    want = sys.argv[1] if len(sys.argv) > 1 else "가운데"
    ref = reference()
    for k, v in ref.items():
        print(f"기준 {k:12s} {len(v)}개  평균 x {v[:, 0].mean():6.1f}  y {v[:, 1].mean():6.1f}  z {v[:, 2].mean():6.1f}  (흩어짐 {v.std(0).mean():.1f})")
    now = tip(read_now())
    c = ref[want].mean(0)
    dx, dy, dz = now - c
    print(f"\n지금 그리퍼 끝: x {now[0]:.1f}  y {now[1]:.1f}  z {now[2]:.1f}")
    print(f"'{want}' 기준과 차이: 앞뒤 {dx:+.1f} mm, 좌우 {dy:+.1f} mm, 높이 {dz:+.1f} mm  (합 {np.linalg.norm([dx, dy, dz]):.1f} mm)")
    print("앞뒤 +: 소켓이 로봇에서 더 멀다 / 좌우 +: 로봇 쪽에서 볼 때 왼쪽 / 높이 +: 더 높다. 10 mm 안이면 학습 때와 비슷한 자리")
