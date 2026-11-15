"""원인 분리 대조 모델 정리: 같은 v2(2만 스텝)에서 같은 증강 시드로 1천 스텝 이어 학습, 다른 것은 카메라 보정뿐.
학습 시드 3개(1000, 2000, 3000)를 같은 시드끼리 짝지어 보정 후(c) 대 보정 전(nc)을 비교한다(오프라인 지표만, 실물 아님).

입력: outputs/research/offline_eval.json(접근과 꽂기 30~97%), offline_eval_early.json(접근 초반 2~30%)
결과: outputs/research/control_report.json
"""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
W = {"early": json.loads((ROOT / "outputs/research/offline_eval_early.json").read_text()),
     "late": json.loads((ROOT / "outputs/research/offline_eval.json").read_text())}
SEEDS = (1000, 2000, 3000)
KEYS = {"gt_mm_real": "사람 시범과 차이, 실물 화면", "gt_mm_gray": "사람 시범과 차이, 회색 화면",
        "gap_mm_real_tc": "실물 대 보정 후 트윈 짝 차이", "gap_mm_real_tn": "실물 대 보정 전 트윈 짝 차이"}
out = {"note": "오프라인 지표(mm, 1초 뒤 핀 끝). 실물 성패는 이 지표로 가릴 수 없었다(회색 화면 대조). 실물 시험 전 참고용."}
for w, R in W.items():
    rows = {}
    for k, name in KEYS.items():
        c = np.array([R[f"ctl_c1000_s{s}/001000"][k][0] for s in SEEDS])
        n = np.array([R[f"ctl_nc1000_s{s}/001000"][k][0] for s in SEEDS])
        rows[name] = {"보정 후(c) 시드별": c.round(1).tolist(), "보정 전(nc) 시드별": n.round(1).tolist(),
                      "c 평균": round(float(c.mean()), 1), "nc 평균": round(float(n.mean()), 1),
                      "같은 시드 차이 c-nc": (c - n).round(1).tolist()}
    rg = {}
    for tag in ("c1000", "nc1000"):
        v = np.array([R[f"ctl_{tag}_s{s}/001000"]["gt_mm_real"][0] - R[f"ctl_{tag}_s{s}/001000"]["gt_mm_gray"][0] for s in SEEDS])
        rg[tag] = {"시드별": v.round(1).tolist(), "평균": round(float(v.mean()), 1)}
    rows["실물 화면의 손해(실물 - 회색)"] = {"보정 후(c) 시드별": rg["c1000"]["시드별"], "보정 전(nc) 시드별": rg["nc1000"]["시드별"],
                                     "c 평균": rg["c1000"]["평균"], "nc 평균": rg["nc1000"]["평균"],
                                     "같은 시드 차이 c-nc": (np.array(rg["c1000"]["시드별"]) - np.array(rg["nc1000"]["시드별"])).round(1).tolist()}
    out[w] = rows
dst = ROOT / "outputs/research/control_report.json"
dst.write_text(json.dumps(out, ensure_ascii=False, indent=1))
for w in W:
    print("==", w)
    for name, r in out[w].items():
        print(f"  {name:22s} c {r['보정 후(c) 시드별']} 평균 {r['c 평균']:5.1f} | nc {r['보정 전(nc) 시드별']} 평균 {r['nc 평균']:5.1f} | 차 {r['같은 시드 차이 c-nc']}")
