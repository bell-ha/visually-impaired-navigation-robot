#!/usr/bin/env python3
"""cabin_capture.py의 기록을 기존 맵 위에 겹쳐 새 PGM/YAML을 만든다 (오프라인).

원본(all.pgm / all.yaml)은 읽기만 한다. 절대 덮어쓰지 않는다 — 출력이 입력과
같은 경로면 시작하지 않는다.

필터 3개를 독립으로 겹친다. 각각이 나머지가 놓치는 것을 잡는다.
  ① 전방 섹터 ±45°  — 로봇 뒤쪽(운영자·열린 문)을 뺀다.
     한 바퀴에 임의 방향이 섹터에 머무는 스캔 수 N = 2α·f/ω 이고
     α=45°, ω=0.25rad/s, f≈7.7Hz면 48장이다. 점유 판정엔 5~10장이면 충분해
     과충족이고, 남는 여유는 전부 운영자 배제 마진으로 쓴다.
     ±90°는 커버 이득이 없는데 안전 마진만 절반으로 깎는다.
     ※ '정면'은 base_link 기준이다. 라이다는 장착 회전이 있으므로 빔 각도를
       laser 기준으로 그대로 자르면 정확히 반대쪽(뒤)을 고르게 된다. 그래서
       capture가 pose_laser와 pose_base를 둘 다 남기고, 여기서 두 yaw의 차이로
       계산한다 — 오프셋 상수를 손으로 넣지 않는다.
  ② 공간 박스 — 앵커 근처 ±box(m) 밖의 끝점은 전부 버린다. 각도 마스크는 사람
     행동(운영자가 어디 서 있나)에 의존하지만 박스는 의존하지 않는다. 홀·복도·
     운영자·문 밖 통행인을 각도와 무관하게 통째로 걷어낸다.
  ③ 로그오즈 누적 — 한두 장의 튄 관측으로 셀이 뒤집히지 않게 한다.

끝점만 찍지 않고 레이를 따라 free를 칠한다(Bresenham). 끝점만 찍으면 점선
윤곽만 생기고 내부가 unknown으로 남아 맵으로 쓸모가 없다.

PGM 행 순서: 이미지 0행 = y 최대다. row = height-1-int((y-origin_y)/res).
여기서 뒤집는 것이 이 작업의 고전적 버그다.

결과를 '완성'으로 취급하지 마라. 이 산출물은 "캐빈이 이렇게 생겼다"를 눈으로
확인하는 것까지다. nav2에 물리는 것은 별개 판단이다.

사용 예:
  python3 cabin_render.py --input cabin_20260902_161500.jsonl \
      --map ../../maps/all.yaml --out /tmp/all_with_cabin
"""

import argparse
import json
import math
import os
import sys

import numpy as np

FREE_PIX = 254
OCC_PIX = 0
UNKNOWN_PIX = 205


def read_pgm(path):
    """P5 PGM → (numpy uint8 [h, w], maxval). 주석(#) 허용."""
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(b"P5"):
        raise ValueError(f"P5 PGM이 아니다: {path}")
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i] != 0x0A:
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    i += 1                                  # 헤더 뒤 공백 1바이트
    w, h, maxval = fields
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img.copy(), maxval


def write_pgm(path, img, maxval=255):
    with open(path, "wb") as f:
        f.write(b"P5\n# CREATOR: cabin_render.py %.3f m/pix\n" % 0.05)
        f.write(b"%d %d\n%d\n" % (img.shape[1], img.shape[0], maxval))
        f.write(img.tobytes())


def read_yaml(path):
    """map_server yaml의 필요한 필드만 읽는다(PyYAML 의존을 만들지 않는다)."""
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            out[k.strip()] = v.strip()
    return out


def bresenham(c0, r0, c1, r1):
    """격자 직선. 끝점은 포함하지 않는다(끝점은 점유로 따로 칠한다)."""
    cells = []
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    sc = 1 if c1 > c0 else -1
    sr = 1 if r1 > r0 else -1
    err = dc - dr
    c, r = c0, r0
    while True:
        if c == c1 and r == r1:
            break
        cells.append((c, r))
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser(description="캐빈 기록을 기존 맵 위에 겹쳐 새 맵을 만든다.")
    ap.add_argument("--input", required=True, help="cabin_capture.py가 만든 JSONL")
    ap.add_argument("--map", required=True, help="기존 맵 yaml (읽기 전용)")
    ap.add_argument("--out", required=True, help="출력 경로 접두사 (.pgm/.yaml이 붙는다)")
    ap.add_argument("--sector-deg", type=float, default=45.0, help="전방 섹터 반각(도)")
    ap.add_argument("--box", type=float, default=1.5, help="앵커 기준 박스 반변(m)")
    ap.add_argument("--box-center", default="", help="'x,y'로 박스 중심 지정(기본: 첫 포즈)")
    ap.add_argument("--l-occ", type=float, default=0.85)
    ap.add_argument("--l-free", type=float, default=-0.40)
    ap.add_argument("--l-clamp", type=float, default=5.0)
    ap.add_argument("--occ-thresh", type=float, default=0.65)
    ap.add_argument("--free-thresh", type=float, default=0.196)
    ap.add_argument("--use-amcl", action="store_true",
                    help="앵커 포즈 대신 실시간 AMCL 포즈로 렌더(비교용)")
    ap.add_argument("--overwrite-known", action="store_true",
                    help="원본이 이미 free/occupied인 셀도 덮어쓴다(기본: unknown만)")
    args = ap.parse_args()

    yml = read_yaml(args.map)
    map_dir = os.path.dirname(os.path.abspath(args.map))
    pgm_in = os.path.join(map_dir, yml["image"])
    res = float(yml["resolution"])
    ox, oy = [float(x) for x in yml["origin"].strip("[]").split(",")[:2]]

    pgm_out = os.path.abspath(args.out + ".pgm")
    yaml_out = os.path.abspath(args.out + ".yaml")
    for src, dst in ((pgm_in, pgm_out), (os.path.abspath(args.map), yaml_out)):
        if os.path.abspath(src) == dst:
            print(f"⛔ 출력이 입력과 같다: {dst} — 원본을 덮어쓰지 않는다.")
            return 1

    img, maxval = read_pgm(pgm_in)
    h, w = img.shape
    print(f"맵 {pgm_in}  {w}x{h}  res={res}  origin=({ox}, {oy})")

    recs = []
    header = None
    with open(args.input, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r.get("_header"):
                header = r
                continue
            recs.append(r)
    if not recs:
        print("⛔ 기록이 비어 있다.")
        return 1
    print(f"기록 {len(recs)}장 (헤더 {'있음' if header else '없음'})")

    if args.box_center:
        bx, by = [float(v) for v in args.box_center.split(",")]
    else:
        bx, by = recs[0]["pose_base"][0], recs[0]["pose_base"][1]
    print(f"박스 중심 ({bx:+.3f}, {by:+.3f}) 반변 {args.box}m · "
          f"섹터 ±{args.sector_deg}° · 포즈 {'AMCL' if args.use_amcl else '앵커'}")

    logodds = np.zeros((h, w), dtype=np.float32)
    sector = math.radians(args.sector_deg)
    n_beam = n_sector = n_box = n_short = 0

    for rec in recs:
        pl = rec.get("pose_amcl") if args.use_amcl else rec["pose_laser"]
        if pl is None:
            continue
        lx, ly, lyaw = pl
        byaw = rec["pose_base"][2]
        # 라이다 장착 회전은 (lyaw - byaw)에 이미 들어 있다. 빔의 '로봇 정면 기준'
        # 각도 = wrap(lyaw + a - byaw) — 오프셋 상수를 손으로 넣지 않는 이유다.
        a0 = rec["angle_min"]
        da = rec["angle_increment"]
        rmax = rec["range_max"]
        c0 = int((lx - ox) / res)
        r0 = h - 1 - int((ly - oy) / res)
        for i, rng in enumerate(rec["ranges"]):
            n_beam += 1
            if rng is None or rng <= rec["range_min"]:
                continue
            a = a0 + i * da
            if abs(wrap(lyaw + a - byaw)) > sector:      # ① 섹터
                n_sector += 1
                continue
            hit = rng < rmax
            px = lx + rng * math.cos(lyaw + a)
            py = ly + rng * math.sin(lyaw + a)
            if abs(px - bx) > args.box or abs(py - by) > args.box:   # ② 박스
                n_box += 1
                continue
            c1 = int((px - ox) / res)
            r1 = h - 1 - int((py - oy) / res)
            if not (0 <= c1 < w and 0 <= r1 < h and 0 <= c0 < w and 0 <= r0 < h):
                continue
            for (cc, rr) in bresenham(c0, r0, c1, r1):               # ③ 로그오즈
                logodds[rr, cc] += args.l_free
            if hit:
                logodds[r1, c1] += args.l_occ
            else:
                logodds[r1, c1] += args.l_free
                n_short += 1
    np.clip(logodds, -args.l_clamp, args.l_clamp, out=logodds)

    prob = 1.0 / (1.0 + np.exp(-logodds))
    touched = logodds != 0.0
    occ = touched & (prob > args.occ_thresh)
    free = touched & (prob < args.free_thresh)
    if not args.overwrite_known:
        # 원본이 이미 아는 셀은 건드리지 않는다 — 캐빈은 unknown이라 이것만으로
        # 충분하고, 좋은 맵 데이터를 이번 기록으로 훼손할 위험을 없앤다.
        known = img != UNKNOWN_PIX
        occ &= ~known
        free &= ~known

    out = img.copy()
    out[free] = FREE_PIX
    out[occ] = OCC_PIX
    write_pgm(pgm_out, out, maxval)
    with open(yaml_out, "w", encoding="utf-8") as f:
        f.write(f"image: {os.path.basename(pgm_out)}\n"
                f"resolution: {res:.6f}\n"
                f"origin: [{ox:.6f}, {oy:.6f}, 0.000000]\n"
                f"negate: {yml.get('negate', '0')}\n"
                f"occupied_thresh: {yml.get('occupied_thresh', '0.65')}\n"
                f"free_thresh: {yml.get('free_thresh', '0.196')}\n")

    print(f"빔 {n_beam} — 섹터에서 버림 {n_sector} · 박스에서 버림 {n_box} · "
          f"최대사거리(미탐) {n_short}")
    print(f"셀 변경: free {int(free.sum())} · occupied {int(occ.sum())}"
          + ("" if args.overwrite_known else "  (원본 unknown 셀만)"))
    print(f"출력: {pgm_out}\n      {yaml_out}")
    if int(occ.sum()) == 0:
        print("⚠ 점유 셀이 하나도 안 생겼다 — 박스 중심/크기나 섹터를 의심하라.")
    print("이 결과는 '캐빈이 이렇게 생겼다'를 눈으로 확인하는 용도다. 완성이 아니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
