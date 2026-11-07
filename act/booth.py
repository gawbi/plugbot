"""부스 전시 화면(보기 전용). 녹화 화면(record_gui, 8200)과 따로 돈다. 조작 버튼이 없어 측정에 영향을 주지 않는다.

실행: cd ~/plugbot && ~/lerobot_env/bin/python -m uvicorn act.booth:app --host 0.0.0.0 --port 8300
보기: http://127.0.0.1:8300  (전시 모니터에서 F11 전체 화면)
읽는 것(모두 읽기만):
  /dev/shm/plugbot_cam/{front,wrist}.jpg   카메라(녹화 화면의 미리보기나 로봇 실행이 쓰는 것)
  /dev/shm/plugbot_shaker.json             흔들림 갑판: 서보가 실제로 간 위치로 계산한 소켓 높이(deck_shaker.py)
  http://127.0.0.1:8200/state              녹화 화면의 현재 단계(대기, 로봇 실행 중 등)
  data/rollouts/*/result.json              [꽂힘]/[못 꽂음] 판정(사람이 본 결과). 실패도 모두 센다
  outputs/rollout_pair_*.mp4               실물과 UR10e 나란히 영상(act/rollout_pair.sh)
"""

import json
import time
import urllib.request
from pathlib import Path

from fastapi import FastAPI, Response
from fastapi.responses import FileResponse, HTMLResponse

ROOT = Path(__file__).resolve().parents[1]
CAM = Path("/dev/shm/plugbot_cam")
SHAKER = Path("/dev/shm/plugbot_shaker.json")
app = FastAPI()


@app.get("/frame/{name}")
def frame(name: str):
    if name not in ("front", "wrist"):
        return Response(status_code=404)
    try:
        return Response((CAM / f"{name}.jpg").read_bytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})
    except OSError:
        return Response(status_code=404)


@app.get("/deck")
def deck():
    try:
        d = json.loads(SHAKER.read_text())
        d["age"] = time.time() - d["wall"]
        return d
    except (OSError, ValueError, KeyError):
        return {"age": None}


def tally():
    today = time.strftime("%Y%m%d")
    out = {"static": [0, 0], "moving": [0, 0], "ckpt": None, "last": None}
    for f in sorted((ROOT / "data" / "rollouts").glob("*/result.json")):
        if not f.parent.name.startswith(today):
            continue
        try:
            r = json.loads(f.read_text())
        except ValueError:
            continue
        c = out.get(r.get("cond"))
        if c is None:
            continue
        c[1] += 1
        c[0] += int(bool(r.get("ok")))
        out["ckpt"] = "/".join(Path(r["policy"]).parts[-4::2]) if r.get("policy") else None
        out["last"] = {"run": f.parent.name, "ok": bool(r.get("ok")), "cond": r.get("cond")}
    return out


@app.get("/info")
def info():
    try:
        with urllib.request.urlopen("http://127.0.0.1:8200/state", timeout=0.4) as r:
            st = json.loads(r.read())
        phase = st.get("phase")
    except Exception:
        phase = None
    vids = sorted((ROOT / "outputs").glob("rollout_pair_*.mp4"), key=lambda p: p.stat().st_mtime)
    return {"phase": phase, "tally": tally(), "pair": vids[-1].name if vids else None}


@app.get("/video/{name}")
def video(name: str):
    p = ROOT / "outputs" / name
    if not (name.startswith("rollout_pair_") and name.endswith(".mp4") and p.exists()):
        return Response(status_code=404)
    return FileResponse(p, media_type="video/mp4")


@app.get("/", response_class=HTMLResponse)
def index():
    return PAGE


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PlugBot 부스</title>
<style>
:root{--bg:#07090b;--panel:#0b0e11;--line:#1d242c;--grid:#141a20;--fg:#e6e9ec;--mut:#6f7a86;--dim:#3a444e;
--amb:#ffb020;--grn:#4ade80;--red:#f05252;--mono:"DejaVu Sans Mono",ui-monospace,monospace;--sans:"Noto Sans CJK KR","Noto Sans KR",sans-serif}
*{box-sizing:border-box}html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font-family:var(--sans)}
body{display:grid;grid-template-rows:56px 1fr 300px;height:100vh;overflow:hidden}
header{display:flex;align-items:center;gap:28px;padding:0 22px;border-bottom:1px solid var(--line)}
.brand{font-family:var(--mono);font-size:15px;letter-spacing:.18em;color:var(--mut)}.brand b{color:var(--fg);font-weight:600}
.line{font-size:14px;color:var(--mut)}.tag{font-family:var(--mono);font-size:11px;letter-spacing:.14em;color:var(--mut);text-transform:uppercase}
.phase{margin-left:auto;display:flex;align-items:center;gap:10px;font-size:17px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--dim)}.dot.on{background:var(--amb);animation:bl 1s steps(2) infinite}
@keyframes bl{50%{opacity:.25}}
.clock{font-family:var(--mono);font-size:15px;color:var(--mut);min-width:86px;text-align:right}
main{display:grid;grid-template-columns:1fr 1fr 440px;min-height:0;border-bottom:1px solid var(--line)}
.cell{position:relative;border-right:1px solid var(--line);padding:14px 16px;min-height:0;display:flex;flex-direction:column}
.cell:last-child{border-right:0}
.hd{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:10px}
.hd .t{font-size:15px}.camw{flex:1;min-height:0;display:flex;align-items:center;justify-content:center}.cam{width:100%;max-height:100%;aspect-ratio:4/3;background:#000;position:relative;overflow:hidden}
.cam img{width:100%;height:100%;object-fit:cover;display:block}
.cam .nosig{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-family:var(--mono);font-size:13px;color:var(--dim);letter-spacing:.2em}
.tick{position:absolute;width:18px;height:18px;border-color:var(--amb);border-style:solid;opacity:.7}
.tl{left:8px;top:8px;border-width:2px 0 0 2px}.tr{right:8px;top:8px;border-width:2px 2px 0 0}
.bl{left:8px;bottom:8px;border-width:0 0 2px 2px}.br{right:8px;bottom:8px;border-width:0 2px 2px 0}
.score{display:grid;grid-template-columns:1fr 1fr;border:1px solid var(--line)}
.score>div{padding:14px 14px 12px}.score>div+div{border-left:1px solid var(--line)}
.big{font-family:var(--mono);font-size:54px;line-height:1;letter-spacing:-.02em}.big small{font-size:24px;color:var(--mut)}
.sub{font-size:13px;color:var(--mut);margin-top:6px}
.last{margin:12px 0 0;font-size:14px;color:var(--mut);min-height:20px}.last b{font-weight:600}
.pair{flex:1;min-height:0;margin-top:14px;display:flex;flex-direction:column}
.vw{flex:1;min-height:0;position:relative;background:#000}.vw video{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}.vw .nosig{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:14px;color:var(--dim)}
.note{font-size:12px;color:var(--dim);margin-top:6px;line-height:1.4}
footer{display:grid;grid-template-columns:1fr 440px;min-height:0}
.strip{position:relative;padding:12px 16px 10px;border-right:1px solid var(--line);display:flex;flex-direction:column}
.plot{flex:1;position:relative;min-height:0}canvas{position:absolute;inset:0;width:100%;height:100%;display:block}
.read{padding:14px 16px;display:grid;grid-template-rows:auto auto auto 1fr;gap:12px}
.kv{display:flex;justify-content:space-between;align-items:baseline;border-bottom:1px solid var(--grid);padding-bottom:8px}
.kv .v{font-family:var(--mono);font-size:30px}.kv .v small{font-size:15px;color:var(--mut);margin-left:4px}
.ok{color:var(--grn)}.ng{color:var(--red)}.amb{color:var(--amb)}
</style></head><body>
<header>
  <div class="brand"><b>PLUGBOT</b> &nbsp;/&nbsp; LAND2SEA</div>
  <div class="line">땅에서 배운 로봇이 바다에서 꽂는다</div>
  <div class="phase"><span class="dot" id="dot"></span><span id="phase">연결 대기</span></div>
  <div class="clock" id="clock">--:--:--</div>
</header>
<main>
  <section class="cell"><div class="hd"><span class="t">옆면 카메라</span><span class="tag">CAM 1 / FRONT</span></div>
    <div class="camw"><div class="cam"><img id="front" alt=""><div class="nosig" id="nf">NO SIGNAL</div><i class="tick tl"></i><i class="tick tr"></i><i class="tick bl"></i><i class="tick br"></i></div></div></section>
  <section class="cell"><div class="hd"><span class="t">손목 카메라</span><span class="tag">CAM 2 / WRIST</span></div>
    <div class="camw"><div class="cam"><img id="wrist" alt=""><div class="nosig" id="nw">NO SIGNAL</div><i class="tick tl"></i><i class="tick tr"></i><i class="tick bl"></i><i class="tick br"></i></div></div></section>
  <section class="cell">
    <div class="hd"><span class="t">오늘 로봇 혼자 꽂기</span><span class="tag" id="ckpt">ACT</span></div>
    <div class="score">
      <div><div class="big" id="s_st">0<small>/0</small></div><div class="sub">정지 소켓</div></div>
      <div><div class="big" id="s_mv">0<small>/0</small></div><div class="sub">흔들림 갑판</div></div>
    </div>
    <div class="last" id="last"></div>
    <div class="pair"><div class="hd"><span class="t">같은 동작, 실제 크기</span><span class="tag">UR10e / ISAAC SIM</span></div>
      <div class="vw"><video id="pair" muted loop playsinline autoplay></video><div class="nosig" id="np">성공한 실행이 생기면 여기서 재생</div></div>
      <div class="note">오른쪽 영상: 성공한 실행의 관절 기록을 실제 크기 UR10e로 옮긴 재생(시각화, 실시간 연동 아님). 판정은 사람이 눈으로 보고 버튼으로 기록, 실패도 모두 센다.</div></div>
  </section>
</main>
<footer>
  <section class="strip"><div class="hd"><span class="t">갑판 높이</span><span class="tag">DECK MOTION / SERVO FEEDBACK / 30 S</span></div><div class="plot"><canvas id="cv"></canvas></div></section>
  <section class="read">
    <div class="kv"><span>현재</span><span class="v amb" id="r_now">--<small>mm</small></span></div>
    <div class="kv"><span>진폭 (최근 10초)</span><span class="v" id="r_amp">--<small>mm</small></span></div>
    <div class="kv"><span>주기</span><span class="v" id="r_per">--<small>s</small></span></div>
    <div class="note" id="r_src">흔들기 모터의 실제 위치(서보 피드백)에서 계산한 소켓 높이. 명령값이 아니다.</div>
  </section>
</footer>
<script>
const $=id=>document.getElementById(id);
function tickClock(){const d=new Date();$('clock').textContent=d.toTimeString().slice(0,8)}setInterval(tickClock,1000);tickClock();
function cam(name,img,ns){const im=new Image();im.onload=()=>{$(img).src=im.src;$(ns).style.display='none';setTimeout(()=>cam(name,img,ns),100)};
 im.onerror=()=>{$(ns).style.display='flex';setTimeout(()=>cam(name,img,ns),1000)};im.src='/frame/'+name+'?'+Date.now()}
cam('front','front','nf');cam('wrist','wrist','nw');
const WIN=30,buf=[];let live=false;
const cv=$('cv'),cx=cv.getContext('2d');
function draw(){const r=devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;if(cv.width!==w*r){cv.width=w*r;cv.height=h*r}
 cx.setTransform(r,0,0,r,0,0);cx.clearRect(0,0,w,h);const L=44,R=w-6,T=6,B=h-22,mm=16,y=v=>T+(B-T)*(0.5-v/(2*mm));
 cx.font='11px DejaVu Sans Mono';cx.textAlign='right';cx.textBaseline='middle';
 for(let v=-16;v<=16;v+=4){cx.strokeStyle=v===0?'#2a333c':'#141a20';cx.lineWidth=1;cx.beginPath();cx.moveTo(L,y(v)+.5);cx.lineTo(R,y(v)+.5);cx.stroke();
  if(v%8===0){cx.fillStyle='#6f7a86';cx.fillText((v>0?'+':'')+v,L-8,y(v))}}
 const now=performance.now()/1000,x=s=>R-(now-s)/WIN*(R-L);cx.textAlign='center';cx.textBaseline='top';
 for(let s=0;s<=WIN;s+=5){const xx=R-s/WIN*(R-L);cx.strokeStyle='#141a20';cx.beginPath();cx.moveTo(xx+.5,T);cx.lineTo(xx+.5,B);cx.stroke();cx.fillStyle='#3a444e';cx.textAlign=s?'center':'right';cx.fillText(s?('-'+s+'s'):'now',s?xx:R,B+6)}
 while(buf.length&&now-buf[0][0]>WIN+1)buf.shift();
 cx.strokeStyle='#ffb020';cx.lineWidth=2;cx.beginPath();let first=true;
 for(const [s,v] of buf){if(v===null){first=true;continue}const X=x(s);if(X<L)continue;first?cx.moveTo(X,y(v)):cx.lineTo(X,y(v));first=false}cx.stroke();
 if(!live){cx.fillStyle='#3a444e';cx.font='13px DejaVu Sans Mono';cx.textBaseline='middle';cx.fillText('갑판 정지',(L+R)/2,(T+B)/2-14)}
 requestAnimationFrame(draw)}
requestAnimationFrame(draw);
async function pollDeck(){try{const d=await (await fetch('/deck')).json();const s=performance.now()/1000;
 live=d.age!==null&&d.age<0.5;const v=live?(d.meas_mm??null):null;buf.push([s,live?v:0]);
 $('r_now').innerHTML=(live&&v!==null?(v>=0?'+':'')+v.toFixed(1):'0.0')+'<small>mm</small>';
 const rec=buf.filter(b=>s-b[0]<10&&b[1]!==null).map(b=>b[1]);
 if(rec.length>20){$('r_amp').innerHTML=((Math.max(...rec)-Math.min(...rec))/2).toFixed(1)+'<small>mm</small>'}
 $('r_per').innerHTML=(live?Number(d.per).toFixed(1):'--')+'<small>s</small>';
 $('r_src').textContent=live?(d.meas_mm===null?'서보 위치를 읽지 못해 표시하지 않음':'흔들기 모터의 실제 위치(서보 피드백)에서 계산한 소켓 높이. 명령값이 아니다.'):'흔들기 모터가 꺼져 있다(갑판 정지).'
}catch(e){}setTimeout(pollDeck,40)}
pollDeck();
let pairName=null;
async function pollInfo(){try{const d=await (await fetch('/info')).json();
 const ph=d.phase||'연결 대기';$('phase').textContent=ph;$('dot').classList.toggle('on',ph==='로봇 실행 중'||ph==='녹화 중');
 const t=d.tally,f=(e,a)=>{$(e).innerHTML=a[1]?a[0]+'<small>/'+a[1]+'</small>':'0<small>/0</small>'};f('s_st',t.static);f('s_mv',t.moving);
 $('ckpt').textContent=t.ckpt?('ACT '+t.ckpt.replace('act_real_','')):'ACT';
 $('last').innerHTML=t.last?('최근 실행: <b class="'+(t.last.ok?'ok':'ng')+'">'+(t.last.ok?'꽂힘':'못 꽂음')+'</b> ('+(t.last.cond==='moving'?'흔들림':'정지')+')'):'오늘 판정한 실행 없음';
 if(d.pair&&d.pair!==pairName){pairName=d.pair;$('pair').src='/video/'+d.pair;$('np').style.display='none'}
}catch(e){}setTimeout(pollInfo,1000)}
pollInfo();
</script></body></html>"""
