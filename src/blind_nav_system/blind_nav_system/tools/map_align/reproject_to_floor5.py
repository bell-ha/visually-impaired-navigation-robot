#!/usr/bin/env python3
"""다른 층 지도를 floor5 좌표계로 옮긴다 (그림을 돌리고 옮겨 새 PGM + origin 을 만든다).

align_to_floor5.py 로 찾은 정렬(floor5 → 대상 층)의 역변환이다. 결과 지도에서는 엘리베이터
문앞·캐빈이 floor5 와 같은 좌표가 되어 scene_targets.yaml 한 세트를 다섯 층이 같이 쓴다.
(호실 좌표는 층마다 다르니 location.yaml 은 층별로 따로 따야 한다.)

입력 지도는 읽기만 한다. 출력 PGM 은 0/205/254 만 쓰며, 한 칸을 3×3 으로 나눠 읽고
'벽이 하나라도 걸리면 벽'으로 남긴다 — 45° 가까이 돌리면 1칸짜리 벽이 끊기기 때문이다
(2026-11-11 3층: 가까운 칸 하나만 읽으면 원본 칸의 17%가 빠졌다).

사용 예 (A 는 transforms_20261111.json 또는 align_to_floor5.py 결과에서):
  python3 reproject_to_floor5.py --src floor3_limo.yaml --affine transforms_20261111.json \
      --floor floor3 --out-dir /tmp/out
"""
import argparse
import json
import os

import numpy as np
from PIL import Image

from align_to_floor5 import DEFAULT_REF, read_map


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, help="옮길 지도 yaml (자기 좌표계)")
    ap.add_argument("--affine", required=True, help="transforms JSON (floors.<floor>.A)")
    ap.add_argument("--floor", required=True, help="JSON 안의 층 이름 (예: floor3)")
    ap.add_argument("--ref", default=DEFAULT_REF)
    ap.add_argument("--out-dir", required=True)
    a = ap.parse_args()
    E, _, res = read_map(a.src)
    F, oF, _ = read_map(a.ref)
    A = np.array(json.load(open(a.affine))["floors"][a.floor]["A"])
    L, t = A[:, :2], A[:, 2]
    ys, xs = np.where(E != 205)
    f5p = np.linalg.inv(L) @ (np.stack([ys, xs]) - t[:, None])
    r_min, r_max = int(np.floor(f5p[0].min())) - 40, int(np.ceil(f5p[0].max())) + 40
    c_min, c_max = int(np.floor(f5p[1].min())) - 40, int(np.ceil(f5p[1].max())) + 40
    R, C = np.mgrid[r_min:r_max + 1, c_min:c_max + 1].astype(np.float64)
    n_occ = np.zeros(R.shape, np.int16); n_free = np.zeros(R.shape, np.int16)
    for dr in (-1 / 3, 0, 1 / 3):
        for dc in (-1 / 3, 0, 1 / 3):
            rr, cc = R + dr, C + dc
            sr = np.rint(A[0, 0] * rr + A[0, 1] * cc + A[0, 2]).astype(int)
            sc = np.rint(A[1, 0] * rr + A[1, 1] * cc + A[1, 2]).astype(int)
            ok = (sr >= 0) & (sr < E.shape[0]) & (sc >= 0) & (sc < E.shape[1])
            v = np.full(R.shape, 205, np.uint8); v[ok] = E[sr[ok], sc[ok]]
            n_occ += (v < 50); n_free += (v >= 250)
    out = np.full(R.shape, 205, np.uint8)
    out[n_free * 2 >= 9] = 254
    out[n_occ > 0] = 0
    ox = oF[0] + c_min * res
    oy = oF[1] + (F.shape[0] - 1 - r_max) * res
    os.makedirs(a.out_dir, exist_ok=True)
    pgm = os.path.join(a.out_dir, f"{a.floor}.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n# {a.floor} in floor5 frame\n{out.shape[1]} {out.shape[0]}\n255\n".encode())
        f.write(out.tobytes())
    with open(os.path.join(a.out_dir, f"{a.floor}.yaml"), "w") as f:
        f.write(f"image: {a.floor}.pgm\nresolution: {res:.6f}\norigin: [{ox:.6f}, {oy:.6f}, 0.000000]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"{pgm}  크기 {out.shape[1]}x{out.shape[0]}  origin ({ox:.3f}, {oy:.3f})")


if __name__ == "__main__":
    main()
