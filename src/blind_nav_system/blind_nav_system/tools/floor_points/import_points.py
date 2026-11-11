#!/usr/bin/env python3
"""웹 도구에서 내보낸 JSON → 검사 → location.yaml 호실 블록.

하는 일
  1. 이름 정리   — '101' → '101호', '316-1' → '316다시1호', '107(GS25)' → '107호(GS25)',
                  LAB 에 적힌 방은 '(연구실)'을 붙인다. 숫자가 아닌 이름('공과대학 라운지')은 그대로.
                  '-' 대신 '다시'를 쓰는 이유: 음성 인식이 '316-1'을 '삼백십육 다시 일'로 읽는다
                  (interface.py 가 '다시'를 숫자 사이에서는 재개가 아니라 방 이름으로 본다).
  2. 검사        — 이름 겹침, 층 번호와 방 번호 앞자리 불일치(경고), 로봇 모양 벽 검사(common.WallCheck).
                  벽에 닿는(bad) 점이 있으면 --apply 를 거부한다.
  3. 출력        — 기본은 화면에 YAML 블록만 찍는다(파일 안 건드림).
                  --apply 면 location.yaml 의 '# ── 호실 좌표' 줄부터 끝까지를 새 블록으로 바꾼다.
                  그 위(엘리베이터 탑승지점·문앞·하차지점)는 그대로 둔다. 바꾸기 전 .bak 사본을 남긴다.

사용 예
  python3 import_points.py data/points_20261111.json            # 검사 + 미리보기
  python3 import_points.py data/points_20261111.json --apply    # location.yaml 반영
"""

import argparse
import datetime
import json
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import FLOORS, LOCATION_YAML, WallCheck, yaw_to_zw  # noqa: E402

MARKER = " # ── 호실 좌표"
LAB = {"504", "520"}      # 이름에 '(연구실)'을 붙일 방


def normalize(raw):
    s = raw.strip().replace(" ", "")
    m = re.fullmatch(r"(\d+)(?:-|다시)(\d+)호?", s)
    if m:
        return f"{m.group(1)}다시{m.group(2)}호"
    m = re.fullmatch(r"(\d+)호?(\(.+\))?", s)
    if m:
        num, tail = m.group(1), m.group(2) or ""
        if not tail and num in LAB:
            tail = "(연구실)"
        return f"{num}호{tail}"
    return raw.strip()


def build(points, floors_desc=("floor5", "floor4", "floor3", "floor2", "floor1")):
    today = datetime.date.today().isoformat()
    out = [f"{MARKER}: {today} 층별 지도(floor5 좌표계)에서 새로 찍음 (tools 좌표찍기 도구, 로봇 모양 벽 여유 검사 통과)",
           " #    방향 = 도착 후 로봇이 바라보는 방향. 이전 값(1~4층은 5층 복사본)은 git 기록에 있다.", ""]
    first = True
    for f in floors_desc:
        rows = sorted(points.get(f, []), key=lambda p: p["name"])
        if not rows:
            continue
        if not first:
            out.append("")
        first = False
        out.append(f" # ── {f[-1]}층")
        for p in rows:
            z, w = yaw_to_zw(p["yaw"])
            out += [f" {p['name']}:", f"   x: {p['x']}", f"   y: {p['y']}",
                    f"   z: {z:.6f}", f"   w: {w:.6f}", f"   floor: {f[-1]}", ""]
    return "\n".join(out).rstrip("\n") + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", help="웹 도구 'JSON 저장' 파일")
    ap.add_argument("--apply", action="store_true", help="location.yaml 에 반영")
    ap.add_argument("--yaml", default=LOCATION_YAML)
    args = ap.parse_args()

    raw = json.load(open(args.json, encoding="utf8"))
    points, problems, warns = {}, [], []
    seen = {}
    for f in FLOORS:
        if not raw.get(f):
            continue
        wc = WallCheck(f)
        for p in raw[f]:
            name = normalize(p["name"])
            q = {"name": name, "x": p["x"], "y": p["y"], "yaw": float(p["yaw"])}   # x·y 는 받은 모양 그대로 쓴다
            if name in seen:
                problems.append(f"이름 겹침: {name} ({seen[name]} · {f})")
            seen[name] = f
            m = re.match(r"\d+", name)
            if m and (len(m.group(0)) != 3 or m.group(0)[0] != f[-1]):
                warns.append(f"{f} 에 {name} — 세 자리가 아니거나 층 번호와 앞자리가 다름 (오타?)")
            st, min_d, center = wc.check(q["x"], q["y"], q["yaw"])
            if st == "bad":
                problems.append(f"벽 닿음: {f} {name} ({q['x']:.2f}, {q['y']:.2f}, {q['yaw']:.0f}°)")
            elif st == "warn":
                warns.append(f"벽 가까움: {f} {name} 모양 {min_d:.2f} m · 중심 {center:.2f} m")
            points.setdefault(f, []).append(q)

    total = sum(len(v) for v in points.values())
    print("층별 개수:", {f: len(v) for f, v in points.items()}, "· 합계", total)
    for w in warns:
        print("  ⚠", w)
    for p in problems:
        print("  ✖", p)

    block = build(points)
    if not args.apply:
        print("\n----- 미리보기 (--apply 로 반영) -----\n" + block)
        return 1 if problems else 0
    if problems:
        print("✖ 문제가 있어 반영하지 않았다.")
        return 1

    text = open(args.yaml, encoding="utf8").read()
    i = text.find("\n" + MARKER)
    if i < 0:
        print(f"✖ {args.yaml} 에서 '{MARKER.strip()}' 줄을 못 찾았다 — 손으로 붙여 넣을 것.")
        return 1
    bak = args.yaml + ".bak-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    shutil.copy2(args.yaml, bak)
    open(args.yaml, "w", encoding="utf8").write(text[: i + 1] + block)
    print(f"반영: {args.yaml}  (이전 파일: {bak})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
