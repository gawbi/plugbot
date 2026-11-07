"""본선(선상) 개발 증거: 승선 시점 기준선을 저장하고, 그 뒤로 늘어난 것을 자동으로 센다.

승선 직전 한 번:  python3 tools/onboard_report.py --snapshot      → docs/team/baseline_20261108.json
본선 중 아무 때나: python3 tools/onboard_report.py                 기준선 대비 차이를 markdown으로 출력(멘토에게 보여 준다)
                  python3 tools/onboard_report.py --json          + docs/team/onboard_delta.json (발표 덱 부록 F가 읽음)
                  python3 tools/onboard_report.py --append        + docs/team/본선_개발일지.md 끝에 시각과 함께 붙임
세는 것: 기준선 이후 커밋(시각, 메시지), 데이터셋 에피소드 수 변화, 새 체크포인트(생성 시각), 판정된 로봇 혼자 꽂기 실행(날짜, 체크포인트,
조건, 꽂힘 수), 새 outputs 파일, docs/troubleshooting.md 새 항목. 모두 파일과 git에서 읽는다(손으로 적는 숫자 없음).
LeRobot을 불러오지 않는다(meta/info.json만 읽음). 어느 파이썬으로든 돈다.
"""

import argparse
import json
import math
import re
import subprocess
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEAM = ROOT / "docs" / "team"
HF = Path.home() / ".cache" / "huggingface" / "lerobot" / "local"
RUNS = Path.home() / "act_runs"
ROLL = ROOT / "data" / "rollouts"
OUT = ROOT / "outputs"
SKIP_DS = ("backup", "bad", "test", "smoke")


def git(*a):
    return subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True, text=True).stdout.strip()


def wilson(k, n, z=1.96):
    if n == 0:
        return (0, 0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (round(max(0, c - h) * 100), round(min(1, c + h) * 100))


def ts(path):
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(path.stat().st_mtime))


def datasets():
    out = {}
    for f in sorted(HF.glob("*/meta/info.json")):
        name = f.parents[1].name
        if any(s in name for s in SKIP_DS):
            continue
        try:
            d = json.loads(f.read_text())
            out[name] = {"episodes": d.get("total_episodes"), "frames": d.get("total_frames")}
        except ValueError:
            pass
    return out


def checkpoints():
    out = {}
    for c in sorted(RUNS.glob("*/checkpoints/*")):
        if c.name == "last" or "smoke" in c.parents[1].name:
            continue
        out[f"{c.parents[1].name}/{c.name}"] = ts(c)
    return out


def rollouts():
    out = {}
    for r in sorted(ROLL.glob("*/result.json")):
        try:
            j = json.loads(r.read_text())
        except ValueError:
            continue
        ck = "/".join(Path(j["policy"]).parts[-4::2]) if j.get("policy") else "?"
        out[r.parent.name] = {"ok": bool(j.get("ok")), "cond": j.get("cond"), "ckpt": ck}
    return out


def outputs():
    return {p.name: ts(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.suffix in (".mp4", ".png", ".jpg", ".json", ".md", ".gif")}


def trouble():
    f = ROOT / "docs" / "troubleshooting.md"
    return re.findall(r"^### (.+)$", f.read_text(), re.M) if f.exists() else []


def snapshot():
    return {"taken": time.strftime("%Y-%m-%d %H:%M"), "git_head": git("rev-parse", "--short", "HEAD"), "git_tag": git("describe", "--tags", "--always"),
            "datasets": datasets(), "checkpoints": checkpoints(), "rollouts": rollouts(), "outputs": outputs(), "troubleshooting": trouble()}


TAG = "boarding-20261108"  # 기준선 문서를 커밋한 뒤 붙이는 태그. 있으면 커밋 목록은 태그 이후만(기준선 문서 커밋 자체는 뺌)


def delta(base):
    now = snapshot()
    ref = TAG if git("rev-parse", "-q", "--verify", f"refs/tags/{TAG}") else base["git_head"]
    commits = [c.split("|", 1) for c in git("log", f"{ref}..HEAD", "--format=%ad|%s", "--date=format:%m/%d %H:%M").splitlines() if c]
    ds = []
    for name, v in now["datasets"].items():
        b = base["datasets"].get(name)
        if b is None:
            ds.append({"dataset": name, "before": 0, "after": v["episodes"], "new": True})
        elif b["episodes"] != v["episodes"]:
            ds.append({"dataset": name, "before": b["episodes"], "after": v["episodes"], "new": False})
    ck = [{"ckpt": k, "time": v} for k, v in now["checkpoints"].items() if k not in base["checkpoints"]]
    new_runs = {k: v for k, v in now["rollouts"].items() if k not in base["rollouts"]}
    tally = defaultdict(lambda: {"static": [0, 0], "moving": [0, 0]})
    for run, v in new_runs.items():
        day = f"{run[4:6]}/{run[6:8]}"
        c = tally[(day, v["ckpt"])][v["cond"]]
        c[1] += 1
        c[0] += int(v["ok"])
    rows = [{"day": d, "ckpt": k, "static": v["static"], "moving": v["moving"],
             "static_ci": wilson(*v["static"]), "moving_ci": wilson(*v["moving"])} for (d, k), v in sorted(tally.items())]
    outs = [{"file": k, "time": v} for k, v in now["outputs"].items() if k not in base["outputs"]]
    tr = [t for t in now["troubleshooting"] if t not in base["troubleshooting"]]
    return {"generated": now["taken"], "baseline": {"taken": base["taken"], "git_head": base["git_head"], "git_tag": base["git_tag"]},
            "git_head": now["git_head"], "commits": [{"time": t, "msg": m} for t, m in commits], "datasets": ds, "checkpoints": ck,
            "rollouts": rows, "rollouts_n": len(new_runs), "outputs": outs, "troubleshooting": tr}


def markdown(d):
    L = [f"## 본선 개발 현황 ({d['generated']})", "",
         f"기준선: {d['baseline']['taken']} 승선, 커밋 {d['baseline']['git_head']} (태그 {d['baseline']['git_tag']}). 지금 커밋 {d['git_head']}.", ""]
    L.append(f"### 커밋 {len(d['commits'])}개")
    L += [f"- {c['time']}  {c['msg']}" for c in d["commits"]] or ["- 없음"]
    L += ["", "### 데이터셋(에피소드 수)"]
    L += [f"- {x['dataset']}: {x['before']} → {x['after']}" + (" (새로 만듦)" if x["new"] else "") for x in d["datasets"]] or ["- 변화 없음"]
    L += ["", "### 새 체크포인트"]
    L += [f"- {x['ckpt']} ({x['time']})" for x in d["checkpoints"]] or ["- 없음"]
    L += ["", f"### 로봇 혼자 꽂기 판정 {d['rollouts_n']}회 (사람이 보고 버튼으로, 실패도 셈)"]
    for x in d["rollouts"]:
        s, m = x["static"], x["moving"]
        parts = []
        if s[1]:
            parts.append(f"정지 {s[0]}/{s[1]} (95% {x['static_ci'][0]}~{x['static_ci'][1]}%)")
        if m[1]:
            parts.append(f"흔들림 {m[0]}/{m[1]} (95% {x['moving_ci'][0]}~{x['moving_ci'][1]}%)")
        L.append(f"- {x['day']}  {x['ckpt']}: " + ", ".join(parts))
    if not d["rollouts"]:
        L.append("- 없음")
    L += ["", "### 새 결과 파일(outputs/)"]
    L += [f"- {x['file']} ({x['time']})" for x in d["outputs"]] or ["- 없음"]
    L += ["", "### 현장 문제와 해결(docs/troubleshooting.md 새 항목)"]
    L += [f"- {t}" for t in d["troubleshooting"]] or ["- 없음"]
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", action="store_true", help="지금 상태를 기준선으로 저장")
    ap.add_argument("--baseline", default=str(TEAM / "baseline_20261108.json"))
    ap.add_argument("--json", action="store_true", help="docs/team/onboard_delta.json도 쓴다(덱 부록 F)")
    ap.add_argument("--append", action="store_true", help="docs/team/본선_개발일지.md 끝에 붙인다")
    a = ap.parse_args()
    bp = Path(a.baseline)
    if a.snapshot:
        if bp.exists():
            raise SystemExit(f"{bp} 가 이미 있다. 기준선은 한 번만 만든다(바꾸려면 파일을 지우고 다시)")
        s = snapshot()
        bp.write_text(json.dumps(s, ensure_ascii=False, indent=1))
        print(f"기준선 저장: {bp}  커밋 {s['git_head']}, 데이터셋 {len(s['datasets'])}개, 체크포인트 {len(s['checkpoints'])}개, "
              f"판정 {len(s['rollouts'])}회, outputs {len(s['outputs'])}개")
        raise SystemExit
    if not bp.exists():
        raise SystemExit(f"{bp} 가 없다. 먼저 --snapshot")
    d = delta(json.loads(bp.read_text()))
    md = markdown(d)
    print(md)
    if a.json:
        (TEAM / "onboard_delta.json").write_text(json.dumps(d, ensure_ascii=False, indent=1))
        print("저장:", TEAM / "onboard_delta.json")
    if a.append:
        log = TEAM / "본선_개발일지.md"
        with log.open("a") as f:
            f.write("\n" + md)
        print("붙임:", log)
