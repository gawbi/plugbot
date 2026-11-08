"""markdown → A4 PDF(인쇄용 문서). Chrome으로 출력한다(인터넷 불필요).

사용: python3 tools/md_to_pdf.py <입력.md> <출력.pdf> [--docno "PB-01 ..."] [--author "김광빈"] [--basis "2026-11-08"]
모양: 글꼴 Noto Sans CJK KR 한 가족, 색은 검정과 남색 하나, 표는 옅은 띠. 맨 위에 문서 정보 칸, 아래에 쪽 번호.
'## ' 제목, '### ' 소제목, 표, 글머리(- , 1. ), **굵게**, `코드`만 다룬다. 첫 '# ' 줄이 문서 제목.
표에 '상태' 열이 있으면 됨(칠함), 코드만/명령만/일부(반), 정식 측정 전/없음/미검증(빈칸) 네모를 붙인다.
"""

import argparse
import html
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

CSS = """
@page{size:A4;margin:20mm 19mm 20mm 19mm;
 @bottom-left{content:"PlugBot  팀 Land2Sea";font:400 7.5pt 'Noto Sans CJK KR';color:#6b7280}
 @bottom-right{content:counter(page) "쪽 / " counter(pages) "쪽";font:400 7.5pt 'Noto Sans CJK KR';color:#6b7280}}
:root{--ink:#000;--text:#2b2b2b;--mute:#6b7280;--navy:#23395b;--band:#f1f4f8}
*{box-sizing:border-box}
body{font-family:'Noto Sans CJK KR',sans-serif;font-size:9.4pt;line-height:1.62;color:var(--text);margin:0;word-break:keep-all}
.card{display:grid;grid-template-columns:1fr 62mm;gap:8mm;align-items:end;padding-bottom:7mm;margin-bottom:6mm;border-bottom:2.2pt solid var(--navy)}
.card h1{font-weight:700;font-size:19pt;line-height:1.28;color:var(--ink);margin:0;letter-spacing:-.015em}
.card .ev{font-size:8.6pt;color:var(--mute);margin:0 0 3mm}
.meta{display:grid;grid-template-columns:auto 1fr;gap:1.2mm 4mm;font-size:8.2pt;margin:0}
.meta dt{color:var(--mute)}.meta dd{margin:0;color:var(--ink);font-weight:500}
h2{font-weight:700;font-size:11.5pt;color:var(--navy);margin:8mm 0 2.5mm;break-after:avoid}
h3{font-weight:700;font-size:10pt;color:var(--ink);margin:5mm 0 1.5mm;break-after:avoid}
p{margin:0 0 2.2mm}
ul,ol{margin:0 0 3mm;padding-left:5mm}li{margin:0 0 1.2mm}
b{font-weight:700;color:var(--ink)}
code{font-family:inherit;font-size:8pt;color:var(--mute);word-break:break-all}
table{border-collapse:collapse;width:100%;margin:1mm 0 4mm;font-size:8.3pt;line-height:1.45}
thead th{font-weight:700;font-size:7.8pt;color:var(--navy);text-align:left;padding:0 2.4mm 1.6mm;border-bottom:1.2pt solid var(--navy)}
td{padding:2mm 2.4mm;vertical-align:top;min-width:16mm}
td:first-child{width:28%}.st{width:30%}
tbody tr:nth-child(even) td{background:var(--band)}
tr{break-inside:avoid}
.m{display:inline-block;width:2.6mm;height:2.6mm;border:.8pt solid var(--navy);margin-right:1.6mm;vertical-align:-.2mm}
.m.full{background:var(--navy)}.m.half{background:linear-gradient(90deg,var(--navy) 50%,transparent 50%)}
.legend{font-size:7.8pt;color:var(--mute);margin:-1mm 0 2mm}
"""


def inline(s):
    s = html.escape(s)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", s)
    return s.replace("\\|", "|")


def status(c):
    raw = re.sub(r"<[^>]+>", "", c)
    if raw.startswith(("됨", "1개 됨")):
        k = "full"
    elif raw.startswith(("코드만", "명령만", "일부")):
        k = "half"
    elif raw.startswith(("정식 측정 전", "없음", "미검증")):
        k = ""
    else:
        return c
    return f'<span class="m {k}"></span>{c}'


def to_html(md, docno, author, basis):
    lines, body, i, title = md.splitlines(), [], 0, "PlugBot"
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                if not re.match(r"^\|[\s\-|:]+\|$", lines[i]):
                    rows.append([inline(c.strip()) for c in re.split(r"(?<!\\)\|", lines[i].strip())[1:-1]])
                i += 1
            h, *rest = rows
            si = next((k for k, x in enumerate(h) if x == "상태"), None)
            if si is not None:
                body.append('<p class="legend"><span class="m full"></span>됨 &nbsp; <span class="m half"></span>코드나 명령만 &nbsp; <span class="m"></span>아직</p>')
            body.append("<table><thead><tr>" + "".join(f"<th>{c}</th>" for c in h) + "</tr></thead><tbody>"
                        + "".join("<tr>" + "".join(f'<td class="{"st" if k == si else ""}">{status(c) if k == si else c}</td>'
                                                   for k, c in enumerate(r)) + "</tr>" for r in rest) + "</tbody></table>")
            continue
        if re.match(r"^\s*(- |\d+\. )", ln):
            tag = "ul" if ln.lstrip().startswith("- ") else "ol"
            items = []
            while i < len(lines) and re.match(r"^\s*(- |\d+\. )", lines[i]):
                items.append(re.sub(r"^\s*(- |\d+\. )", "", lines[i]))
                i += 1
            body.append(f"<{tag}>" + "".join(f"<li>{inline(x)}</li>" for x in items) + f"</{tag}>")
            continue
        if ln.startswith("# "):
            title = ln[2:]
        elif ln.startswith("### "):
            body.append(f"<h3>{inline(ln[4:])}</h3>")
        elif ln.startswith("## "):
            body.append(f"<h2>{inline(ln[3:])}</h2>")
        elif ln.strip() and not ln.startswith("```"):
            body.append(f"<p>{inline(ln)}</p>")
        i += 1
    meta = "".join(f"<dt>{k}</dt><dd>{html.escape(v)}</dd>" for k, v in (("문서", docno), ("작성", author), ("기준", basis), ("저장소", "github.com/gawbi/plugbot")) if v)
    return (f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><title>{html.escape(title)}</title><style>{CSS}</style></head><body>'
            f'<header class="card"><div><p class="ev">2026 동남권 멀티도메인 모빌리티 비즈니스 해커톤 본선</p><h1>{html.escape(title)}</h1></div>'
            f'<dl class="meta">{meta}</dl></header>{"".join(body)}</body></html>')


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--docno", default="")
    ap.add_argument("--author", default="김광빈")
    ap.add_argument("--basis", default="")
    a = ap.parse_args()
    chrome = shutil.which("google-chrome") or shutil.which("chromium")
    with tempfile.TemporaryDirectory() as td:
        h = Path(td) / "doc.html"
        h.write_text(to_html(Path(a.src).read_text(), a.docno, a.author, a.basis))
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", f"--print-to-pdf={Path(a.out).resolve()}", h.as_uri()],
                       capture_output=True, timeout=90)
    print(a.out)
