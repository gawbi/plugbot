"""UR10e(Isaac Sim 5.1 기본 자산)와 하늘 HDR을 로컬에 받아 둔다. 인터넷 없이(선상) run_scaleup.py를 다시 돌리기 위해서다.

UR10e USD가 참조하는 레이어(구성, 물리, 그리퍼 옵션 등)를 모두 같은 폴더 구조로 복사하므로 상대 경로 참조가 그대로 살아 있다.
받는 곳: ~/isaacsim_assets/Assets/Isaac/5.1/... (저장소 밖. USD는 커밋하지 않는다)
실행: OMNI_KIT_ACCEPT_EULA=YES LD_LIBRARY_PATH=~/isaacsim_compat/lib ~/isaacsim_env/bin/python isaac/scaleup/fetch_assets.py
"""

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.client  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import UsdUtils  # noqa: E402

LOCAL = Path.home() / "isaacsim_assets" / "Assets" / "Isaac" / "5.1"
ROBOT = "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
SKY = "/NVIDIA/Assets/Skies/Cloudy/champagne_castle_1_4k.hdr"

root = get_assets_root_path()
layers, assets, unresolved = UsdUtils.ComputeAllDependencies(root + ROBOT)
urls = [l.identifier for l in layers] + [a for a in assets if a.startswith(root)] + [root + SKY]
n = 0
for url in urls:
    rel = url[len(root):]
    dst = LOCAL / rel.lstrip("/")
    dst.parent.mkdir(parents=True, exist_ok=True)
    result, _, content = omni.client.read_file(url)
    if result != omni.client.Result.OK:
        print("실패", url, result)
        continue
    dst.write_bytes(memoryview(content).tobytes())
    n += 1
print(f"{n}/{len(urls)}개 받음 → {LOCAL}" + (f", 못 찾은 참조 {unresolved}" if unresolved else ""))
sys.stdout.flush()
os._exit(0)
