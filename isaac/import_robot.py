"""[Isaac Sim 5.1, GPU 노트북] so101_plugbot{_side}.urdf → USD 변환(배치마다 한 번).

실행: <Isaac 파이썬> isaac/import_robot.py [--layout side|top]   (기본 side: L자 플러그)
결과: isaac/so101/so101_plugbot_side.usd (top은 so101_plugbot.usd)

※ 이 맥에서는 실행해 보지 못했다(GPU 없음). 명령 이름이 다르다는 오류가 나면 GUI로 같은 일을 한다:
   File > Import > so101_plugbot_side.urdf, Fix Base Link 켬, Merge Fixed Joints 끔, Joint Drive Type = Position,
   Collision = Convex Decomposition, 저장 위치 isaac/so101/so101_plugbot_side.usd
"""

import argparse
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--layout", default="side", choices=["side", "top"])
SUFFIX = "_side" if ap.parse_args().layout == "side" else ""

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402

HERE = Path(__file__).resolve().parent
URDF = HERE / f"so101/so101_plugbot{SUFFIX}.urdf"
USD = HERE / f"so101/so101_plugbot{SUFFIX}.usd"

ok, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False  # plug_tip, wrist_cam_link 링크를 남긴다
cfg.fix_base = True  # 로봇은 부두에 고정
cfg.make_default_prim = True
cfg.self_collision = False
cfg.distance_scale = 1.0
cfg.create_physics_scene = False
omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(URDF), import_config=cfg, dest_path=str(USD))
print("저장:", USD, "있음" if USD.exists() else "없음(GUI로 변환할 것)")
app.close()
