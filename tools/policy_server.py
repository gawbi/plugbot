"""학습한 LeRobot 정책(ACT, SmolVLA 등)을 따로 띄워 두고, 다른 파이썬(Isaac Sim 등)이 관측을 보내면 행동을 돌려준다.

Isaac Sim 파이썬(3.11)과 LeRobot 환경(3.12)은 라이브러리가 달라 한 프로세스에 넣지 않는다. 표준 라이브러리 소켓
(multiprocessing.connection)으로 주고받고, 배열은 바이트로 보낸다(numpy 버전이 달라도 된다).

보내는 것(dict): {"reset": True} 또는 {"images": {"front": bytes, ...}, "shape": (H, W, 3), "state": [6개, 도 + 그리퍼]}
받는 것: 행동 6개(list, 도 + 그리퍼). 데이터셋(plugbot.dataset.to_lerobot_vector)과 같은 단위다.

실행: ~/lerobot_env/bin/python tools/policy_server.py ~/act_runs/act_isaac_v1/checkpoints/last/pretrained_model [--port 5555]
"""

import argparse
from multiprocessing.connection import Listener

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors

ap = argparse.ArgumentParser()
ap.add_argument("ckpt", help="pretrained_model 폴더")
ap.add_argument("--port", type=int, default=5555)
ap.add_argument("--task", default="Insert the plug into the socket")
ap.add_argument("--device", default="cuda")
args = ap.parse_args()

cfg = PreTrainedConfig.from_pretrained(args.ckpt)
cfg.device = args.device
policy = get_policy_class(cfg.type).from_pretrained(args.ckpt, config=cfg).to(args.device).eval()
pre, post = make_pre_post_processors(policy.config, args.ckpt,  # 저장된 장치 대신 --device를 쓴다
                                     preprocessor_overrides={"device_processor": {"device": args.device}})
print(f"정책 {cfg.type} 불러옴: {args.ckpt}", flush=True)

with Listener(("127.0.0.1", args.port), authkey=b"plugbot") as listener:
    print(f"대기: 127.0.0.1:{args.port}", flush=True)
    while True:
        with listener.accept() as conn:
            print("연결됨", flush=True)
            while True:
                try:
                    msg = conn.recv()
                except EOFError:
                    print("연결 끊김", flush=True)
                    break
                if msg.get("reset"):
                    policy.reset()
                    conn.send("ok")
                    continue
                h, w, c = msg["shape"]
                obs = {f"observation.images.{name}": torch.from_numpy(
                    np.frombuffer(buf, np.uint8).reshape(h, w, c).copy()).permute(2, 0, 1).float() / 255.0
                    for name, buf in msg["images"].items()}  # 데이터셋에서 읽을 때와 같은 (3, H, W), 0~1
                obs["observation.state"] = torch.tensor(msg["state"], dtype=torch.float32)
                obs["task"] = args.task
                with torch.inference_mode():
                    action = post(policy.select_action(pre(obs)))
                conn.send([float(v) for v in action.reshape(-1).cpu()])
