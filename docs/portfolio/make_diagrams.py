"""포트폴리오 아키텍처 그림 3장(SVG, 밝은 판과 어두운 판) + PNG(Chrome 헤드리스).

1 overview  : 하드웨어, 노트북 소프트웨어, 시뮬레이션이 어떻게 이어지나
2 pipeline  : 세 갈래(Real-to-Real, Sim-to-Real, 이종 원격조종)가 정책과 실물 판정으로 모이는 길
3 runtime   : 실행 중 프로세스, 포트, 장치, 공유 파일, 안전 장치
실행: python3 docs/portfolio/make_diagrams.py   결과: docs/portfolio/img/arch_{overview,pipeline,runtime}{,_dark}.svg, .png
숫자는 docs/portfolio/facts.md와 같아야 한다.
"""

import subprocess
from html import escape
from pathlib import Path

OUT = Path(__file__).resolve().parent / "img"
THEMES = {
    "": dict(bg="#ffffff", ink="#1d1d24", muted="#5c5c6b", line="#8b8b9a", panel="#f6f5f9", box="#ffffff",
             acc="#6b3fd6", accfill="#f1ecff", ok="#1f7a45"),
    "_dark": dict(bg="#101016", ink="#ececf2", muted="#a4a4b4", line="#7c7c8e", panel="#18181f", box="#1d1d26",
                  acc="#a98bff", accfill="#241c3a", ok="#5fcf8e"),
}
FONT = "Inter, 'Segoe UI', 'Noto Sans', 'DejaVu Sans', Arial, sans-serif"
MONO = "'JetBrains Mono', 'DejaVu Sans Mono', Menlo, monospace"


class D:
    def __init__(self, w, h, t, label):
        self.w, self.h, self.t, self.label, self.el = w, h, t, label, []

    def group(self, x, y, w, h, title):
        t = self.t
        self.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="{t["panel"]}"/>')
        self.el.append(f'<text x="{x + 16}" y="{y + 26}" font-size="13" font-weight="700" letter-spacing="0.08em" fill="{t["muted"]}">{escape(title.upper())}</text>')

    def box(self, x, y, w, h, title, sub=None, sim=False, acc=False, mono_sub=False):
        t = self.t
        stroke, fill = (t["acc"], t["accfill"]) if acc else (t["line"], t["box"])
        dash = ' stroke-dasharray="6 4"' if sim else ""
        self.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="9" fill="{fill}" stroke="{stroke}" stroke-width="{1.6 if acc else 1.2}"{dash}/>')
        cy = y + h / 2 + (-6 if sub else 5)
        self.el.append(f'<text x="{x + w / 2}" y="{cy}" text-anchor="middle" font-size="14" font-weight="650" fill="{t["ink"]}">{escape(title)}</text>')
        if sub:
            fam = f' font-family="{MONO}"' if mono_sub else ""
            self.el.append(f'<text x="{x + w / 2}" y="{cy + 18}" text-anchor="middle" font-size="11.5"{fam} fill="{t["muted"]}">{escape(sub)}</text>')

    def text(self, x, y, s, size=12, color="muted", anchor="start", weight=400, mono=False):
        fam = f' font-family="{MONO}"' if mono else ""
        self.el.append(f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-size="{size}" font-weight="{weight}"{fam} fill="{self.t[color]}">{escape(s)}</text>')

    def arrow(self, pts, label=None, lx=None, ly=None, acc=False, both=False, anchor="middle", dashed=False):
        t = self.t
        c = t["acc"] if acc else t["line"]
        mk = "a" if acc else "n"
        d = "M" + " L".join(f"{x},{y}" for x, y in pts)
        start = f' marker-start="url(#{mk}s)"' if both else ""
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        self.el.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="1.5"{dash} marker-end="url(#{mk})"{start}/>')
        if label:
            if lx is None:
                (x1, y1), (x2, y2) = pts[0], pts[1]
                lx, ly = (x1 + x2) / 2, (y1 + y2) / 2 - 7
            self.el.append(f'<text x="{lx}" y="{ly}" text-anchor="{anchor}" font-size="11.5" fill="{t["acc"] if acc else t["muted"]}">{escape(label)}</text>')

    def svg(self):
        t = self.t
        defs = "".join(
            f'<marker id="{i}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
            f'<marker id="{i}s" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
            for i, c in (("n", t["line"]), ("a", t["acc"])))
        return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" width="{self.w}" height="{self.h}" '
                f'role="img" aria-label="{escape(self.label)}" font-family="{FONT}">'
                f'<defs>{defs}</defs><rect width="100%" height="100%" fill="{t["bg"]}"/>' + "".join(self.el) + "</svg>")


def overview(t):
    d = D(1240, 600, t, "PlugBot system overview: real hardware, laptop software and simulation, and the data that moves between them")
    o = 36  # 맨 위에 실시간 원격조종 선이 지나갈 자리
    d.group(20, 20 + o, 330, 510, "Real hardware")
    d.group(430, 20 + o, 360, 510, "Laptop (RTX 5070, Ubuntu)")
    d.group(840, 20 + o, 370, 510, "Simulation")
    X1, X2, X3, W = 40, 450, 860, 320
    W1 = 290  # 하드웨어 상자는 좁게: 가운데 통로에 화살표 라벨이 들어가게
    d.box(X1, 64 + o, W1, 56, "SO-101 leader arm", "a person demonstrates")
    d.box(X1, 150 + o, W1, 56, "SO-101 follower + IEC plug cover", "6 x STS3215 servos, 3D-printed")
    d.box(X1, 236 + o, W1, 56, "Side camera + wrist camera", "640 x 480, 30 fps")
    d.box(X1, 322 + o, W1, 56, "Shaking deck", "1 x STS3215, sine or irregular waves")
    d.box(X1, 408 + o, W1, 56, "Photobox socket", "black box, white IEC-style socket")
    d.box(X2, 64 + o, W, 100, "LeRobot record / rollout", "child process with safety patches")
    d.box(X2, 194 + o, W, 56, "Datasets + ACT training", "LeRobot 0.6.1, 134 real demos")
    d.box(X2, 280 + o, W, 56, "Run logs + human judgement", "result.json per run")
    d.box(X2, 366 + o, W, 56, "Tally", "success / trials, Wilson 95% CI")
    d.box(X2, 452 + o, W, 56, "Recording GUI  (:8200)", "starts record / run / shaker, one at a time")
    d.box(X3, 64 + o, W, 56, "MuJoCo strategy study", "static / follow / predict", sim=True)
    d.box(X3, 150 + o, W, 56, "Isaac Sim photobox twin", "replays real joints, renders cameras", sim=True, acc=True)
    d.box(X3, 236 + o, W, 56, "Isaac Sim UR10e scale-up", "small leader drives industrial arm", sim=True)
    # 하드웨어 → LeRobot 프로세스
    d.arrow([(330, 92 + o), (450, 92 + o)], "joint positions", lx=390, ly=85 + o)
    d.arrow([(330, 178 + o), (400, 178 + o), (400, 124 + o), (450, 124 + o)], "goal / state", lx=334, ly=171 + o, anchor="start", both=True)
    d.arrow([(330, 264 + o), (420, 264 + o), (420, 152 + o), (450, 152 + o)], "frames", lx=334, ly=257 + o, anchor="start")
    # 녹화 화면 → 갑판, 판정
    d.arrow([(450, 480 + o), (390, 480 + o), (390, 350 + o), (330, 350 + o)], "keys via pipe", lx=334, ly=343 + o, anchor="start")
    d.arrow([(770, 480 + o), (782, 480 + o), (782, 308 + o), (770, 308 + o)], "judge", lx=786, ly=398 + o, anchor="start")
    # 소프트웨어 안
    d.arrow([(580, 164 + o), (580, 194 + o)], "episodes", lx=572, ly=184 + o, anchor="end")
    d.arrow([(640, 194 + o), (640, 164 + o)], "policy", lx=648, ly=184 + o, anchor="start")
    d.arrow([(460, 164 + o), (460, 172 + o), (440, 172 + o), (440, 308 + o), (450, 308 + o)], "logs", lx=444, ly=262 + o, anchor="start")
    d.arrow([(610, 336 + o), (610, 366 + o)])
    # 데이터셋 ↔ 트윈
    d.arrow([(770, 212 + o), (815, 212 + o), (815, 172 + o), (860, 172 + o)], "real joints", lx=774, ly=205 + o, anchor="start", acc=True)
    d.arrow([(860, 192 + o), (830, 192 + o), (830, 236 + o), (770, 236 + o)], "renders", lx=776, ly=252 + o, anchor="start", acc=True)
    # 리더 → UR10e (실시간 이종 원격조종): 그룹 위로 돌아간다
    d.arrow([(185, 64 + o), (185, 26), (1222, 26), (1222, 264 + o), (1180, 264 + o)],
            "live teleop: leader joints retargeted to a full-size UR10e (simulation)", lx=700, ly=19)
    d.text(20, 572 - 2, "Dashed = simulation. Purple = Sim-to-Real path: robot motion comes from real demonstrations, camera images are rendered.", 12)
    d.text(20, 592 - 2, "Both policies run on the same real follower arm and are judged the same way.", 12)
    return d


def pipeline(t):
    d = D(1260, 640, t, "Three learning paths converge on one real-robot evaluation: Real-to-Real 40 of 40, Sim-to-Real 39 of 40")
    C = [30, 235, 440, 645, 850]
    W, H = 180, 66
    # A
    d.text(C[0], 62, "A  REAL-TO-REAL", 12.5, "muted", weight=700)
    d.box(C[0], 74, W, H, "Human teleop", "leader -> follower")
    d.box(C[1], 74, W, H, "Real demos", "134 eps (94 static, 40 moving)")
    d.box(C[3], 74, W, H, "ACT, 80k steps", "act_real_v4")
    d.arrow([(C[0] + W, 107), (C[1], 107)])
    d.arrow([(C[1] + W, 107), (C[3], 107)], "behaviour cloning", lx=(C[1] + W + C[3]) / 2, ly=100)
    # B
    d.text(C[0], 252, "B  SIM-TO-REAL", 12.5, "acc", weight=700)
    d.text(C[0], 270, "same human demos,", 11.5, "muted")
    d.text(C[0], 286, "new camera images", 11.5, "muted")
    d.box(C[1], 226, W, H, "Static demos", "69 eps, joints only", acc=True)
    d.box(C[2], 226, W, H, "Isaac twin replay", "renders front + wrist", sim=True, acc=True)
    d.box(C[3], 226, W, H, "Augment", "harsh lights, blur, shake", sim=True, acc=True)
    d.box(C[4], 226, W, H, "ACT, +1k steps", "act_twin_v4", acc=True)
    d.arrow([(C[1] + W / 2, 140), (C[1] + W / 2, 226)], "subset", lx=C[1] + W / 2 + 8, ly=188, anchor="start", acc=True)
    d.arrow([(C[1] + W, 259), (C[2], 259)], acc=True)
    d.arrow([(C[2] + W, 259), (C[3], 259)], acc=True)
    d.arrow([(C[3] + W, 259), (C[4], 259)], acc=True)
    d.text(C[2], 316, "state + action = real   |   images = rendered (0 real photos)", 12, "acc")
    d.text(C[2], 334, "cameras calibrated against real inserted frames", 12, "muted")
    # 평가
    EX = 1065
    d.box(EX, 74, 170, 218, "Real SO-101 rollout", None)
    d.text(EX + 85, 214, "static socket", 11.5, "muted", "middle")
    d.text(EX + 85, 230, "+ shaking deck", 11.5, "muted", "middle")
    d.arrow([(C[3] + W, 107), (EX, 107)], "policy", lx=(C[3] + W + EX) / 2, ly=100)
    d.arrow([(C[4] + W, 259), (EX, 259)], acc=True)
    d.box(EX, 330, 170, 66, "Human judge", "button -> result.json")
    d.arrow([(EX + 85, 292), (EX + 85, 330)])
    # 결과
    RX, RY = 645, 420
    d.el.append(f'<rect x="{RX}" y="{RY}" width="590" height="132" rx="10" fill="{t["box"]}" stroke="{t["ink"]}" stroke-width="1.4"/>')
    d.arrow([(EX + 85, 396), (EX + 85, RY)])
    d.text(RX + 18, RY + 28, "REAL ROBOT, 20 TRIALS EACH, WILSON 95% CI", 12, "muted", weight=700)
    for i, (name, a, b, col) in enumerate([("Real-to-Real", "20/20  (84-100%)", "20/20  (84-100%)", "ink"),
                                           ("Sim-to-Real", "20/20  (84-100%)", "19/20  (76-99%)", "acc")]):
        y = RY + 64 + i * 34
        d.text(RX + 18, y, name, 15, col, weight=700)
        d.text(RX + 170, y, "static " + a, 14, "ink", mono=True)
        d.text(RX + 390, y, "moving " + b, 14, "ink", mono=True)
    # C
    d.text(C[0], 434, "C  HETEROGENEOUS TELEOP (SIMULATION)", 12.5, "muted", weight=700)
    d.box(C[0], 446, W, H, "SO-101 leader", "live joints over serial")
    d.box(C[1], 446, W, H, "Retarget", "socket-relative pose, x5", sim=True)
    d.box(C[2], 446, W, H, "UR10e in Isaac", "Lula IK, ferry socket", sim=True)
    d.arrow([(C[0] + W, 479), (C[1], 479)])
    d.arrow([(C[1] + W, 479), (C[2], 479)])
    d.text(C[0], 540, "Not trained, not real hardware: the same demonstrations", 12)
    d.text(C[0], 558, "can drive a full-size industrial arm.", 12)
    d.text(C[0], 610, "Dashed = simulation step. Purple = Sim-to-Real path. Numbers: two laptops combined, human-judged, aboard ship, 2026-11-10.", 12)
    return d


def runtime(t):
    d = D(1240, 590, t, "Runtime: the recording GUI starts one child process at a time; devices are addressed by stable IDs; previews and logs go through files")
    d.group(20, 20, 300, 520, "Devices (by-id)")
    d.group(360, 20, 500, 520, "Processes")
    d.group(900, 20, 330, 520, "Files and readers")
    DX, DW = 36, 268
    d.box(DX, 64, DW, 54, "Follower serial", "...5AE6083026", mono_sub=True)
    d.box(DX, 134, DW, 54, "Leader serial", "...5B79015054", mono_sub=True)
    d.box(DX, 234, DW, 54, "Side cam C270", "/dev/v4l/by-id/...C270", mono_sub=True)
    d.box(DX, 304, DW, 54, "Wrist cam", "/dev/v4l/by-id/...Generic", mono_sub=True)
    d.box(DX, 384, DW, 54, "Shaker serial", "...5B91044141", mono_sub=True)
    PX, PW = 380, 460
    d.box(PX, 64, PW, 54, "record_gui.py  (uvicorn 127.0.0.1:8200)", "browser buttons: record, run, judge, shake")
    d.el.append(f'<rect x="{PX + 10}" y="140" width="{PW - 20}" height="320" rx="10" fill="none" stroke="{t["line"]}" stroke-width="1" stroke-dasharray="3 4"/>')
    d.text(PX + 24, 160, "child processes, started one at a time (lock)", 11.5, "muted")
    d.arrow([(PX + PW / 2, 118), (PX + PW / 2, 140)], "spawns", lx=PX + PW / 2 + 8, ly=134, anchor="start")
    CX, CW = PX + 30, PW - 60
    d.box(CX, 172, CW, 68, "record_preview.py -> lerobot-record / rollout", "gripper floor 6/100, torque retry x3")
    d.box(CX, 280, CW, 54, "cam_preview.py", "idle only; writes the same preview files")
    d.box(CX, 384, CW, 54, "deck_shaker.py", "keys on stdin; stops when the pipe closes")
    # 장치 → record_preview (겹치지 않게 세로선 위치를 나눔)
    d.arrow([(DX + DW, 91), (352, 91), (352, 186), (CX, 186)], both=True)
    d.arrow([(DX + DW, 161), (344, 161), (344, 198), (CX, 198)])
    d.arrow([(DX + DW, 261), (336, 261), (336, 212), (CX, 212)])
    d.arrow([(DX + DW, 331), (348, 331), (348, 226), (CX, 226)])
    d.arrow([(DX + DW, 411), (CX, 411)], both=True)
    FX, FW = 916, 288
    d.box(FX, 64, FW, 54, "/dev/shm/plugbot_cam/*.jpg", "latest frames, about 15 fps")
    d.box(FX, 172, FW, 68, "data/rollouts/<time>/", "meta, joints, frames, result.json")
    d.box(FX, 280, FW, 54, "tally_rollouts.py", "per-checkpoint success, Wilson CI")
    d.box(FX, 384, FW, 54, "/dev/shm/plugbot_shaker.json", "deck height, commanded + measured")
    d.box(FX, 464, FW, 54, "booth.py  (:8300)", "audience screen, read-only")
    # 프로세스 → 파일
    d.arrow([(CX + CW, 182), (880, 182), (880, 91), (FX, 91)], "preview", lx=884, ly=136, anchor="start")
    d.arrow([(CX + CW, 222), (FX, 222)], "run log", lx=(CX + CW + FX) / 2 + 20, ly=215)
    d.arrow([(CX + CW, 411), (FX, 411)], "writes", lx=(CX + CW + FX) / 2 + 20, ly=404)
    d.arrow([(FX + FW / 2, 240), (FX + FW / 2, 280)], "reads", lx=FX + FW / 2 + 8, ly=264, anchor="start")
    d.arrow([(FX + FW, 91), (1218, 91), (1218, 491), (FX + FW, 491)], "reads", lx=1214, ly=456, anchor="end")
    d.el.append(f'<path d="M{FX + FW},411 L1218,411" fill="none" stroke="{t["line"]}" stroke-width="1.5"/>')
    d.text(20, 566, "Safety in code: one start at a time (lock), gripper command floor, torque-write retries, deck stops if the GUI dies, devices by serial number, not by /dev order.", 12)
    return d


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name, fn in (("arch_overview", overview), ("arch_pipeline", pipeline), ("arch_runtime", runtime)):
        for suf, t in THEMES.items():
            d = fn(t)
            p = OUT / f"{name}{suf}.svg"
            p.write_text(d.svg())
            png = p.with_suffix(".png")
            subprocess.run(["google-chrome", "--headless=new", "--disable-gpu", "--hide-scrollbars", "--force-device-scale-factor=2",
                            f"--window-size={d.w},{d.h}", f"--screenshot={png}", p.as_uri()],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
            print(p.name, png.name if png.exists() else "(png 실패)")


if __name__ == "__main__":
    main()
