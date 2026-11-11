#!/usr/bin/env python3
"""층별 지도를 A3 PDF로 — 사람이 펜으로 호실 자리를 표시하는 종이(또는 아이패드)용.

한 쪽 = 한 층: 왼쪽 큰 지도(격자 5 m · 1 m), 오른쪽 위 엘리베이터 주변 확대(격자 1 m), 오른쪽 아래 범례.
엘리베이터 4점(탑승지점·문앞·하차지점·캐빈)과, 지금 location.yaml 에 있는 그 층 호실(파랑)을 그린다.

PDF 옆에 '<PDF>.geo.json'을 같이 쓴다 — 쪽마다 '지도 칸이 종이 어디에 있고(비율), 그 칸이
몇 m부터 몇 m까지인지'. make_point_tool.py 가 이걸로 '그림 픽셀 ↔ 지도 좌표'를 바꾼다.
그래서 PDF에 표시를 해도(그림 위에 덧그림) 좌표 변환은 그대로 맞는다. 쪽 크기만 바꾸지 말 것.

사용 예
  python3 make_map_pdf.py --out ~/Downloads/층별지도_좌표표시.pdf
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import LOCATION_YAML, OCC_MAX, PKG, UNKNOWN_MAX, load_map  # noqa: E402

PAGES = ["floor5", "floor1", "floor2", "floor3", "floor4"]   # 2026-11-11 PDF 쪽 순서
FONTS = ["/System/Library/Fonts/AppleSDGothicNeo.ttc",                 # mac
         "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"]            # ubuntu (fonts-nanum)


def set_font():
    for f in FONTS:
        if os.path.exists(f):
            font_manager.fontManager.addfont(f)
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name()
            break
    plt.rcParams["axes.unicode_minus"] = False


def yaw_of(v):
    return math.degrees(2 * math.atan2(v.get("z", 0.0), v.get("w", 1.0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    set_font()

    loc = yaml.safe_load(open(LOCATION_YAML, encoding="utf8"))["locations"]
    st = yaml.safe_load(open(os.path.join(PKG, "blind_nav_system/elevator_button_press/scene_targets.yaml"),
                             encoding="utf8"))["targets"]
    elev = [("엘리베이터 탑승지점", loc["엘리베이터 탑승지점"]["x"], loc["엘리베이터 탑승지점"]["y"], yaw_of(loc["엘리베이터 탑승지점"]), "#d62728"),
            ("문앞 (②정렬·⑥하차)", st["1"]["x"], st["1"]["y"], st["1"]["yaw_deg"], "#2ca02c"),
            ("엘리베이터 하차지점", loc["엘리베이터 하차지점"]["x"], loc["엘리베이터 하차지점"]["y"], yaw_of(loc["엘리베이터 하차지점"]), "#9467bd"),
            ("캐빈 안 (④)", st["3"]["x"], st["3"]["y"], st["3"]["yaw_deg"], "#ff7f0e")]
    ex, ey = elev[1][1], elev[1][2]

    def draw_map(ax, a, o, res, xl, yl):
        H, W = a.shape
        img = np.where(a <= OCC_MAX, 0, np.where(a > UNKNOWN_MAX, 255, 215)).astype(np.uint8)
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=(o[0], o[0] + W * res, o[1], o[1] + H * res),
                  interpolation="nearest", origin="upper")
        ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect("equal")

    def grid(ax, major, minor):
        for axis in (ax.xaxis, ax.yaxis):
            axis.set_major_locator(plt.MultipleLocator(major)); axis.set_minor_locator(plt.MultipleLocator(minor))
        ax.grid(which="major", color="#4a7fd6", alpha=0.45, lw=0.6)
        ax.grid(which="minor", color="#9db8ea", alpha=0.3, lw=0.3)
        ax.tick_params(labelsize=7)

    def plot_elev(ax, big=False):
        for _, x, y, yd, c in elev:
            ax.plot(x, y, "o", ms=7 if big else 4, mfc=c, mec="black", mew=0.6, zorder=5)
            L = 0.6 if big else 1.2
            ax.annotate("", xy=(x + L * math.cos(math.radians(yd)), y + L * math.sin(math.radians(yd))), xytext=(x, y),
                        arrowprops=dict(arrowstyle="-|>", color=c, lw=1.4), zorder=5)

    geo = {"pages": PAGES}
    with PdfPages(args.out) as pdf:
        for f in PAGES:
            a, o, res = load_map(f)
            H = a.shape[0]
            ys, xs = np.where(a != 205)
            xl = (o[0] + xs.min() * res - 3, o[0] + xs.max() * res + 3)
            yl = (o[1] + (H - 1 - ys.max()) * res - 3, o[1] + (H - 1 - ys.min()) * res + 3)
            rooms = [(k, v["x"], v["y"]) for k, v in loc.items() if v.get("floor") == int(f[-1])]

            fig = plt.figure(figsize=(16.5, 11.7))    # A3 가로
            ax = fig.add_axes([0.04, 0.06, 0.70, 0.86]); draw_map(ax, a, o, res, xl, yl); grid(ax, 5, 1)
            ax.set_xlabel("x [m]  (굵은 격자 5 m · 가는 격자 1 m)", fontsize=9); ax.set_ylabel("y [m]", fontsize=9)
            plot_elev(ax)
            for k, x, y in rooms:
                ax.plot(x, y, "o", ms=4, mfc="#1f77b4", mec="black", mew=0.5, zorder=4)
                ax.annotate(k, (x, y), xytext=(0, -9), textcoords="offset points", ha="center", va="top", fontsize=6.5,
                            color="#0b3d91", bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75), zorder=6)
            ax.set_title(f"{f[-1]}층 ({f}) — 호실 {len(rooms)}곳(파랑, location.yaml) + 엘리베이터 4곳(다섯 층 공통)",
                         fontsize=13, loc="left")
            ax2 = fig.add_axes([0.765, 0.50, 0.22, 0.42])
            draw_map(ax2, a, o, res, (ex - 4, ex + 4), (ey - 3.5, ey + 4.5)); grid(ax2, 1, 0.5); plot_elev(ax2, big=True)
            ax2.set_title("엘리베이터 주변 확대 (격자 1 m)", fontsize=10)
            ax.add_patch(plt.Rectangle((ex - 4, ey - 3.5), 8, 8, fill=False, ec="#ff7f0e", lw=1.2, ls="--", zorder=6))
            ax3 = fig.add_axes([0.765, 0.08, 0.22, 0.36]); ax3.axis("off"); T = ax3.transAxes
            ax3.text(0, 0.98, "엘리베이터 관련 좌표 (다섯 층 공통)", fontsize=10, fontweight="bold", va="top", transform=T)
            yy = 0.86
            for n, x, y, yd, c in elev:
                ax3.plot([0.03], [yy - 0.035], "o", ms=8, mfc=c, mec="black", transform=T, clip_on=False)
                ax3.text(0.08, yy, f"{n}\n({x:.3f}, {y:.3f}), 방향 {yd:.1f}°", fontsize=8.5, va="top", transform=T)
                yy -= 0.15
            ax3.text(0, yy, "화살표 = 로봇이 바라보는 방향\n지도 색: 흰색 길 · 검정 벽 · 회색 모름",
                     fontsize=8, va="top", color="#444", transform=T)

            fig.canvas.draw()   # set_aspect('equal') 가 칸 위치를 줄이므로 그린 뒤에 잰다
            geo[f] = {k: {"pos": list(map(float, axx.get_position().bounds)),
                          "xl": list(map(float, axx.get_xlim())), "yl": list(map(float, axx.get_ylim()))}
                      for k, axx in (("main", ax), ("inset", ax2))}
            pdf.savefig(fig)
            plt.close(fig)
    json.dump(geo, open(args.out + ".geo.json", "w"), indent=1)
    print(args.out, "\n", args.out + ".geo.json")


if __name__ == "__main__":
    main()
