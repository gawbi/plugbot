"""[Isaac Sim 5.1, GPU 노트북] so101_plugbot.urdf → USD 변환(한 번).

실행(Isaac Sim 설치 폴더에서): ./python.sh <저장소>/plugbot_demo/isaac/import_robot.py
결과: isaac/so101/so101_plugbot.usd

※ 이 맥에서는 실행해 보지 못했다(GPU 없음). 명령 이름이 다르다는 오류가 나면 GUI로 같은 일을 한다:
   File > Import > so101_plugbot.urdf, Fix Base Link 켬, Merge Fixed Joints 끔, Joint Drive Type = Position,
   Collision = Convex Decomposition, 저장 위치 isaac/so101/so101_plugbot.usd
"""

from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402

HERE = Path(__file__).resolve().parent
URDF = HERE / "so101/so101_plugbot.urdf"
USD = HERE / "so101/so101_plugbot.usd"

ok, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False  # plug_tip 링크를 남긴다
cfg.fix_base = True  # 로봇은 부두에 고정
cfg.make_default_prim = True
cfg.self_collision = False
cfg.distance_scale = 1.0
cfg.create_physics_scene = False
omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(URDF), import_config=cfg, dest_path=str(USD))
print("저장:", USD, "있음" if USD.exists() else "없음(GUI로 변환할 것)")
app.close()
