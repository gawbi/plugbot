"""로봇 혼자 꽂기(ACT) 판정 집계: data/rollouts/<시각>/result.json(녹화 화면의 [꽂힘]/[못 꽂음] 버튼)을 체크포인트와 조건별로 센다.

실행: ~/lerobot_env/bin/python tools/tally_rollouts.py [--ckpt act_real_v4/080000] [--out <덱 폴더>/real_results.json]
날짜로 거르기(본선 기간만): ... --from 20261108 [--to 20261110]   (실행 폴더 이름의 날짜 부분)
지난 실행에 판정 붙이기: ... tools/tally_rollouts.py --mark 20261106_191509 ok static
결과 json은 발표 덱(build_deck_photo.py 9장)이 읽어 '정지 a/b, 흔들림 c/d'와 Wilson 95% 신뢰구간을 채운다.
  승선 모델 → real_results_before.json, 선상 데이터로 추가 학습한 모델 → real_results.json (둘 다 있으면 9장이 전후로 보여 준다)
판정이 없는 실행은 세지 않는다(숫자를 만들지 않는다).
"""

import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "data" / "rollouts"


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (round(max(0, c - h) * 100), round(min(1, c + h) * 100))


ap = argparse.ArgumentParser()
ap.add_argument("--ckpt", default="", help="이 체크포인트만(예: act_real_v4/080000). 비우면 가장 많이 판정한 것")
ap.add_argument("--out", default="")
ap.add_argument("--mark", nargs=3, metavar=("RUN", "ok|fail", "static|moving"))
ap.add_argument("--from", dest="from_", default="", metavar="YYYYMMDD", help="이 날짜부터(실행 폴더 이름 기준)")
ap.add_argument("--to", default="", metavar="YYYYMMDD", help="이 날짜까지")
a = ap.parse_args()

if a.mark:
    run, ok, cond = a.mark
    d = RUNS / run
    meta = json.loads((d / "meta.json").read_text())
    (d / "result.json").write_text(json.dumps({"ok": ok == "ok", "cond": cond, "policy": meta.get("policy", ""),
                                               "marked": time.strftime("%Y-%m-%d %H:%M:%S"), "retro": True}, ensure_ascii=False, indent=1))
    print("판정 붙임:", run, ok, cond)

tab = defaultdict(lambda: {"static": [0, 0], "moving": [0, 0]})
for d in sorted(RUNS.glob("*/result.json")):
    day = d.parent.name[:8]
    if (a.from_ and day < a.from_) or (a.to and day > a.to):
        continue
    r = json.loads(d.read_text())
    ck = "/".join(Path(r["policy"]).parts[-4::2]) if r.get("policy") else "?"  # act_real_v4/080000
    c = tab[ck][r["cond"]]
    c[1] += 1
    c[0] += int(r["ok"])
for ck, v in sorted(tab.items()):
    print(f"{ck:28s} 정지 {v['static'][0]}/{v['static'][1]}   흔들림 {v['moving'][0]}/{v['moving'][1]}")
if not tab:
    print("판정된 실행이 없다")
elif a.out:
    ck = a.ckpt or max(tab, key=lambda k: sum(x[1] for x in tab[k].values()))
    v = tab[ck]
    out = {"ckpt": ck, "date": time.strftime("%Y-%m-%d"), "runs_from": a.from_ or None, "runs_to": a.to or None,
           "static": v["static"], "moving": v["moving"],
           "static_ci": wilson(*v["static"]), "moving_ci": wilson(*v["moving"]),
           "note": "녹화 화면 판정 버튼(사람이 보고 판정), data/rollouts에 실행마다 영상과 관절 기록"}
    Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print("저장:", a.out, out)
