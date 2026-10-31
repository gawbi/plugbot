"""기획안 그림 조립.

실행: .venv/bin/python make_figures.py  (render_demo.py, run_experiments.py 결과가 먼저 있어야 한다)
결과(outputs/): fig_prediction.png, proposal_fig1.png, proposal_fig2.png
      (proposal_fig1, 2는 ../SimCell_심사패키지/07_그림1_개념도.png, 07_그림2_결과.png로도 복사)
사진(photos/): 팀이 보유한 SO-101 리더암, 팔로워암 원본
"""

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from PIL import Image, ImageDraw, ImageFont, ImageOps

from plugbot.control import Params, ar_predict
from plugbot.waves import ShipMotion

OUT = Path(__file__).resolve().parent / "outputs"
PHOTOS = Path(__file__).resolve().parent / "photos"
PKG = Path(__file__).resolve().parents[1] / "SimCell_심사패키지"
FONT = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
plt.rcParams["axes.unicode_minus"] = False


def font(size, bold=False):
    return ImageFont.truetype(FONT, size, index=6 if bold else 0)


def prediction_plot(hs=0.06, seed=3):
    """소켓 상하 변위, 0.1 s 늦은 관측, AR로 0.3 s 앞을 내다본 예측, 정온 구간."""
    p = Params()
    m = ShipMotion(hs, seed=seed)
    dt = 1 / p.cam_hz
    t = np.arange(0, 12, dt)
    z = np.array([m(tt)[0][2] for tt in t]) * 1000
    rng = np.random.default_rng(0)
    obs = z + rng.normal(0, p.noise * 1000, len(z))
    lag = int(round(p.latency / dt))
    ahead = int(round((p.latency + p.lead) / dt))
    pred = np.full(len(t), np.nan)
    speed = np.full(len(t), np.nan)
    w = int(p.ar_window / dt)
    for i in range(w + lag, len(t)):
        hist = obs[i - lag - w:i - lag]
        f = ar_predict(hist, p.ar_order, ahead + 1)
        pred[i] = f[ahead - 1]
        speed[i] = abs(f[ahead] - f[ahead - 1]) / dt
    quiet = speed < p.gate_speed * 1000
    fig, ax = plt.subplots(figsize=(6.0, 3.0), dpi=220)
    t0 = 4.0
    sel = t >= t0
    ax.plot(t[sel], z[sel], color="#222", lw=2, label="실제 소켓 상하 변위")
    ax.plot(t[sel] + 0 * t[sel], np.roll(obs, lag)[sel], color="#9a9a9a", lw=1.2, ls=":", label="카메라 관측(0.1 s 늦음)")
    ax.plot(t[sel], pred[sel], color="#0e5c3e", lw=1.6, ls="--", label="AR 예측(0.3 s 앞)")
    ylim = (np.nanmin(z[sel]) - 6, np.nanmax(z[sel]) + 6)
    ax.fill_between(t[sel], ylim[0], ylim[1], where=quiet[sel], color="#0e5c3e", alpha=0.10, lw=0, label="정온 구간(삽입 시작 가능)")
    ax.set_ylim(*ylim)
    ax.set_xlim(t0, t[-1])
    ax.set_xlabel("시간 (s)")
    ax.set_ylabel("상하 변위 (mm)")
    ax.legend(fontsize=7, loc="upper right", ncol=2, framealpha=0.9)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT / "fig_prediction.png")
    plt.close(fig)


def success_plot(path=OUT / "fig_success.png", figsize=(6.0, 3.6), dpi=220, fs=None, lw=2, ms=7, short=False):
    """파고별 삽입 성공률. 기획안용이라 그래프 안 제목은 두지 않는다(본문 캡션이 설명).

    fs를 주면 작은 칸(기획안 그림 2)에 들어가는 크기로 글자와 선을 맞춘다.
    """
    res = json.loads((OUT / "experiment.json").read_text())
    keys = list(res["결과"])
    hs = [0, 2, 4, 6, 8]
    labels = {"static": "처음 위치로(정적)", "reactive": "최신 위치 추종", "predictive": "AR 예측 + 정온 구간"}
    if short:
        labels = {"static": "정적", "reactive": "추종", "predictive": "예측"}
    style = {"static": ("#9a9a9a", "o", ":"), "reactive": ("#c8781e", "s", "--"), "predictive": ("#0e5c3e", "^", "-")}
    rc = {} if fs is None else {"font.size": fs, "axes.labelsize": fs, "xtick.labelsize": fs, "ytick.labelsize": fs}
    with plt.rc_context(rc):
        fig, ax = plt.subplots(figsize=figsize, dpi=dpi)
        for s, label in labels.items():
            c, mk, ls = style[s]
            ax.plot(hs, [res["결과"][k][s]["성공률"] * 100 for k in keys], color=c, marker=mk, ls=ls, lw=lw, ms=ms,
                    label=label)
        ax.set_xlabel("모형 유의파고 Hs (cm)")
        ax.set_ylabel("삽입 성공률 (%)")
        ax.set_ylim(-3, 103)
        ax.set_xticks(hs)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8 if fs is None else fs * 0.95, loc="lower left", handlelength=1.6, borderpad=0.3,
                  labelspacing=0.25)
        fig.tight_layout(pad=0.3)
        fig.savefig(path)
        plt.close(fig)
    return Image.open(path)


def photo_pair(gap=24):
    """팀이 보유한 SO-101 리더암(왼쪽)과 팔로워암(오른쪽) 사진을 잘라 같은 높이로 붙인다."""
    crops = [(PHOTOS / "leader_arm.jpeg", (0.10, 0.16, 0.84, 0.88)),
             (PHOTOS / "follower_arm.jpeg", (0.21, 0.13, 0.79, 0.86))]
    ims = []
    for path, (x0, y0, x1, y1) in crops:
        im = ImageOps.exif_transpose(Image.open(path)).convert("RGB")
        ims.append(im.crop((int(x0 * im.width), int(y0 * im.height), int(x1 * im.width), int(y1 * im.height))))
    h = min(im.height for im in ims)
    ims = [_fit_h(im, h) for im in ims]
    out = Image.new("RGB", (sum(im.width for im in ims) + gap * (len(ims) - 1), h), "white")
    x = 0
    for im in ims:
        out.paste(im, (x, 0))
        x += im.width + gap
    return out


def _crop_label(path):
    """render_demo가 붙인 제목 띠를 잘라 낸 장면만."""
    im = Image.open(path)
    return im.crop((0, 92, im.width, im.height))


def _fit_h(img, h):
    return img.resize((int(img.width * h / img.height), h), Image.LANCZOS)


def row_figure(images, captions, path, h=620, gap=60, arrows=False):
    """같은 높이로 맞춘 이미지를 가로로 놓고 각 이미지 아래에 '(a) …' 캡션을 단다."""
    imgs = [_fit_h(im.convert("RGB"), h) for im in images]
    width = sum(im.width for im in imgs) + gap * (len(imgs) - 1)
    size = max(40, round(width / 60))  # 본문 폭에 맞춰 줄여도 캡션이 약 7 pt 이상 되게
    cap_h = int(size * 1.9)
    fig = Image.new("RGB", (width, h + cap_h), "white")
    d = ImageDraw.Draw(fig)
    f = font(size)
    x = 0
    for k, (im, cap) in enumerate(zip(imgs, captions)):
        fig.paste(im, (x, 0))
        tw = d.textlength(cap, font=f)
        d.text((x + (im.width - tw) / 2, h + 14), cap, font=f, fill=(0, 0, 0))
        if arrows and k < len(imgs) - 1:
            y, x0, x1 = h // 2, x + im.width + 10, x + im.width + gap - 10
            d.line([(x0, y), (x1 - 14, y)], fill=(60, 60, 60), width=4)
            d.polygon([(x1, y), (x1 - 18, y - 10), (x1 - 18, y + 10)], fill=(60, 60, 60))
        x += im.width + gap
    fig.save(path)


def figure1():
    row_figure(
        [_crop_label(OUT / "sim_car.png"), Image.open(OUT / "fig_prediction.png"), _crop_label(OUT / "sim_ship.png")],
        ["(a) 육상: 전기차 충전구 삽입", "(b) 시뮬레이션: 선체 운동과 AR 예측", "(c) 해상: 갑판 소켓 삽입"],
        OUT / "proposal_fig1.png", arrows=True)


def figure2():
    """성공률 그래프는 기획안 칸 크기(폭 약 4 cm)에 맞춰 따로 그린다."""
    chart = success_plot(OUT / "fig_success_small.png", figsize=(1.6, 1.22), dpi=600, fs=7.5, lw=1.2, ms=3.5, short=True)
    row_figure(
        [chart, photo_pair(), _crop_label(OUT / "cad_kit.png")],
        ["(a) 파고별 삽입 성공률", "(b) 보유한 SO-101 리더암과 팔로워암", "(c) 본선 시연 키트 조립도"],
        OUT / "proposal_fig2.png")


if __name__ == "__main__":
    prediction_plot()
    success_plot()
    figure1()
    figure2()
    shutil.copy2(OUT / "proposal_fig1.png", PKG / "07_그림1_개념도.png")
    shutil.copy2(OUT / "proposal_fig2.png", PKG / "07_그림2_결과.png")
    print("ok")
