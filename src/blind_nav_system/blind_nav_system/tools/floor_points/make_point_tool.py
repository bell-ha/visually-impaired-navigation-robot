#!/usr/bin/env python3
"""(표시한) 층별 지도 PDF → 클릭해서 좌표를 찍는 HTML 한 장.

브라우저로 열기만 하면 된다(서버·설치 없음). 그림과 벽 거리 지도를 파일 안에 넣어 3 MB 정도.
찍은 점은 그 브라우저에 자동 저장되고, 'JSON 저장'으로 내보내 import_points.py 로 넘긴다.

원리
  · 그림 → 좌표 : PDF 쪽마다 지도 칸의 위치·범위(.geo.json, make_map_pdf.py 가 씀)로 선형 변환.
                 큰 지도와 엘리베이터 확대 그림 둘 다 클릭 가능.
  · 방향        : 클릭 = 0°(지도 +x, 오른쪽), 드래그 = 끈 방향 (8 px 이상 끌어야 방향으로 봄).
  · 벽 검사     : 층 지도에서 '벽까지 거리'(cm, EDT)를 PNG 빨강 채널, '모름' 칸을 초록 채널에 담아
                 넣고, 브라우저에서 로봇+사용자 모양(common.py FP_*) 안을 5 cm 간격으로 훑는다.
                 초록 = 여유 충분, 주황 = 모양-벽 10 cm 미만 또는 중심-벽 50 cm 미만, 빨강 = 벽/지도 밖에 닿음.
  · 같은 규칙을 import_points.py 가 파이썬으로 다시 검사한다(브라우저 값을 믿지 않음).

사용 예
  python3 make_point_tool.py --pdf ~/Downloads/층별지도_좌표표시.pdf --out ~/Downloads/층별좌표찍기_도구.html
  # 2026-11-11 판 다시 만들기 (사람이 표시한 PDF 동봉)
  python3 make_point_tool.py --pdf data/floor_maps_annotated_20261111.pdf --out /tmp/tool.html
"""

import argparse
import base64
import io
import json
import os
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (ELEV, FLOORS, FP_FRONT, FP_LEFT, FP_REAR, FP_RIGHT, OCC_MAX,  # noqa: E402
                    UNKNOWN_MAX, WARN_CENTER, WARN_MIN, load_map)

HERE = os.path.dirname(os.path.abspath(__file__))
DPI = 110          # A3 가로 16.5 in → 약 1815 px


def b64(img, fmt, **kw):
    buf = io.BytesIO()
    img.save(buf, fmt, **kw)
    return f"data:image/{fmt.lower()};base64," + base64.b64encode(buf.getvalue()).decode()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pdf", required=True, help="make_map_pdf.py 로 만든 PDF (펜으로 표시해도 됨, 쪽 크기만 그대로)")
    ap.add_argument("--geo", help="기본: <pdf>.geo.json, 없으면 data/pdf_geo_20261111.json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--store-key", default="floorpts_v1", help="브라우저 자동 저장 이름 (새 작업이면 바꿀 것)")
    args = ap.parse_args()

    geo_path = args.geo or (args.pdf + ".geo.json" if os.path.exists(args.pdf + ".geo.json")
                            else os.path.join(HERE, "data/pdf_geo_20261111.json"))
    geo = json.load(open(geo_path))
    pages = geo.get("pages", ["floor5", "floor1", "floor2", "floor3", "floor4"])

    from scipy.ndimage import distance_transform_edt
    imgs, dmaps = {}, {}
    with tempfile.TemporaryDirectory() as td:
        subprocess.run(["pdftoppm", "-png", "-r", str(DPI), args.pdf, os.path.join(td, "p")], check=True)
        files = sorted(os.listdir(td))
        if len(files) != len(pages):
            sys.exit(f"PDF 쪽 수 {len(files)} ≠ geo 쪽 수 {len(pages)}")
        for f, fn in zip(pages, files):
            imgs[f] = b64(Image.open(os.path.join(td, fn)).convert("RGB"), "JPEG", quality=88)
    for f in FLOORS:
        a, o, res = load_map(f)
        d = np.clip(np.rint(distance_transform_edt(a > OCC_MAX) * res * 100), 0, 255).astype(np.uint8)   # cm, 255 = 2.55 m 이상
        unk = ((a > OCC_MAX) & (a <= UNKNOWN_MAX)).astype(np.uint8) * 255
        rgb = np.stack([d, unk, np.zeros_like(d)], -1)
        dmaps[f] = {"png": b64(Image.fromarray(rgb), "PNG", optimize=True), "w": a.shape[1], "h": a.shape[0],
                    "ox": o[0], "oy": o[1], "res": res}

    data = {"geo": geo, "imgs": imgs, "dmaps": dmaps, "order": FLOORS,
            "label": {f: f"{f[-1]}층" for f in FLOORS},
            "elev": [list(e) for e in ELEV],
            "fp": [[FP_FRONT, FP_RIGHT], [FP_FRONT, FP_LEFT], [FP_REAR, FP_LEFT], [FP_REAR, FP_RIGHT]],
            "fpb": {"front": FP_FRONT, "rear": FP_REAR, "left": FP_LEFT, "right": FP_RIGHT},
            "warn": {"min": WARN_MIN, "center": WARN_CENTER},
            "storeKey": args.store_key}
    html = open(os.path.join(HERE, "point_tool_template.html"), encoding="utf8").read()
    open(args.out, "w", encoding="utf8").write(html.replace("__DATA__", json.dumps(data)))
    print(args.out, os.path.getsize(args.out) // 1024, "KB")


if __name__ == "__main__":
    main()
