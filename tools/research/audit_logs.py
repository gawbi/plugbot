"""실물 실행 기록 감사: data/rollouts/*/{meta,result}.json을 모델, 체크포인트, 조건, 판정별로 센다.

이 노트북에 남은 기록만 본다. 발표 표(outputs/real_results.json)는 다른 노트북 기록까지 합친 숫자다.
실행: python3 tools/research/audit_logs.py
결과: outputs/research/rollout_audit.json
"""

import json
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
runs = []
for meta in sorted((ROOT / "data/rollouts").glob("*/meta.json")):
    d = json.loads(meta.read_text())
    res_f = meta.with_name("result.json")
    res = json.loads(res_f.read_text()) if res_f.exists() else {}
    pol = d.get("policy", "").split("act_runs/")[-1].replace("/pretrained_model", "").replace("checkpoints/", "")
    model, _, ckpt = pol.partition("/")
    extra = [a for a in d.get("args", []) if "n_action_steps" in a or "temporal_ensemble" in a]
    runs.append({"run": meta.parent.name, "model": model, "ckpt": ckpt, "ok": res.get("ok"), "cond": res.get("cond"),
                 "retro": bool(res.get("retro")), "replan_args": extra, "frames": len(list((meta.parent / "front").glob("*.jpg")))})

table = defaultdict(lambda: {"ok": 0, "fail": 0, "unjudged": 0, "retro": 0, "runs": []})
for r in runs:
    key = f"{r['model']}/{r['ckpt']}|{r['cond'] or '-'}"
    t = table[key]
    t["ok" if r["ok"] is True else "fail" if r["ok"] is False else "unjudged"] += 1
    t["retro"] += r["retro"]
    t["runs"].append(r["run"])

out = {"note": "이 노트북(data/rollouts)에 남은 실행만. 사람 판정(result.json), retro = 실행 뒤에 --mark로 붙인 판정. "
               "판정자는 작성자 본인이고 눈가림은 없었다.",
       "total_runs": len(runs), "by_model_ckpt_cond": dict(sorted(table.items())), "runs": runs}
dst = ROOT / "outputs/research/rollout_audit.json"
dst.parent.mkdir(parents=True, exist_ok=True)
dst.write_text(json.dumps(out, ensure_ascii=False, indent=1))
for k, v in sorted(table.items()):
    print(f"{k:42s} ok {v['ok']:2d}  fail {v['fail']:2d}  unjudged {v['unjudged']:2d}  retro {v['retro']:2d}")
print("total", len(runs), "->", dst.relative_to(ROOT))
