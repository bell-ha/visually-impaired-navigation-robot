#!/usr/bin/env python3
"""floor5(기준 지도)를 다른 층 지도 위에 정렬해, GIMP에서 '제자리에 붙여넣기'로 쓸 레이어를 만든다.

2026-11-11 층별 지도 보정에 쓴 도구다. floor1~4 는 LIMO 로 따로 찍어 원점·방향이 제각각이고,
floor5(옛 all.pgm, Stretch 로 찍고 캐빈까지 붙인 완성본)의 일부를 가져와 빈 곳을 채웠다.

하는 일 (원본 지도는 읽기만 한다)
  1. 전체 정렬   — floor5 를 2° 간격으로 한 바퀴 돌려 보며 '길(254)' 겹침이 최대인 각도·위치를
                  FFT 상관으로 찾고, 0.1° 간격으로 다듬는다.
  2. 엘리베이터 정렬 — 지도 전체 기준은 엘리베이터 근처가 수십 cm 어긋날 수 있다(3층 60cm 실측).
                  floor5 의 엘리베이터 문 앞 주변 ±5 m 조각만으로 벽(0) 겹침을 다시 맞추고,
                  위치가 다른 후보를 여러 개 낸다.
                  ⚠ 복도에 비슷한 오목한 공간이 반복되면 한 칸 어긋난 자리가 1등이 될 수 있다
                    (2026-11-11 1층: 화장실 자리에 맞춰짐 → 사람이 후보 그림을 보고 바로잡음).
                    그래서 자동으로 고르지 않고 후보 그림을 보고 --pick 으로 고른다.
  3. 출력       — 대상 지도와 같은 크기의 정렬본 PGM(0/205/254), 겹침 그림, 결과 JSON
                  (각도·이동, 대상 지도 좌표계에서의 엘리베이터 문 앞·캐빈 추정 좌표).

사용 예
  # 1) 후보 보기 (엘리베이터 후보 그림만 만든다)
  python3 align_to_floor5.py --target ../../maps/floor1.yaml --out /tmp/f1
  # 2) 후보 그림(/tmp/f1/elevator_candidates.png)을 보고 번호를 골라 정렬본 만들기
  python3 align_to_floor5.py --target ../../maps/floor1.yaml --out /tmp/f1 --pick 1

색 규약(map_server, negate 0): 0 = 벽, 205 = 모름, 254 = 길. GIMP 에서는 연필 도구만,
흰색은 #FEFEFE. 이미지 크기를 바꾸거나 자르면 yaml origin 이 틀어진다.
"""

import argparse
import json
import os

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_dilation
from scipy.signal import fftconvolve

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_REF = os.path.join(HERE, "../../../maps/floor5.yaml")
# scene_targets.yaml 의 5층 엘리베이터 (map 프레임, TF 값)
DOOR5 = (-48.228, 5.224)    # ② 문앞 정렬 = ⑥ 하차
CABIN5 = (-47.819, 7.057)   # ④ 캐빈 안


def read_map(yaml_path):
    meta = {}
    for line in open(yaml_path, encoding="utf8"):
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    origin = [float(x) for x in meta["origin"].strip("[]").split(",")[:2]]
    arr = np.array(Image.open(img_path).convert("L"))
    return arr, origin, float(meta["resolution"])


def w2p(x, y, origin, res, h):          # 지도 좌표 → (행, 열). PGM 0행 = y 최대
    return h - 1 - (y - origin[1]) / res, (x - origin[0]) / res


def p2w(r, c, origin, res, h):
    return origin[0] + c * res, origin[1] + (h - 1 - r) * res


def rotate(a, ang, fill):
    return np.array(Image.fromarray(a).rotate(ang, resample=Image.NEAREST, expand=True, fillcolor=fill))


def track(shape, r, c, ang):
    """원본 (r,c) 점이 rotate(…, expand=True) 뒤 어디로 가는지 (점 마스크를 같이 돌린다)."""
    m = np.zeros(shape, np.uint8)
    m[int(round(r)) - 1:int(round(r)) + 2, int(round(c)) - 1:int(round(c)) + 2] = 255
    yy, xx = np.where(rotate(m, ang, 0) > 0)
    return yy.mean(), xx.mean()


def global_align(T, F):
    A = (T >= 250).astype(np.float32)
    H = max(T.shape[0], F.shape[0]) * 3
    W = max(T.shape[1], F.shape[1]) * 3
    FA = np.fft.rfft2(A, (H, W))

    def search(angles):
        best = (-1.0,)
        for ang in angles:
            B = (rotate(F, ang, 205) >= 250).astype(np.float32)
            c = np.fft.irfft2(FA * np.conj(np.fft.rfft2(B, (H, W))), (H, W))
            k = c.argmax()
            if c.flat[k] > best[0]:
                best = (float(c.flat[k]), float(ang), np.unravel_index(k, c.shape))
        return best

    b = search(np.arange(-180, 180, 2.0))
    s, ang, (py, px) = search(np.arange(b[1] - 2.5, b[1] + 2.51, 0.1))
    dy = py - H if py > H // 2 else py
    dx = px - W if px > W // 2 else px
    return round(ang, 1), int(dy), int(dx), float(s / A.sum())


def elevator_candidates(T, F, ang_g, near_rc, door5_rc, n=5, radius_px=260):
    """floor5 엘리베이터 주변 조각을 near_rc 근처에서 다시 찾는다. 위치가 다른 후보 n개."""
    R = 100
    r5, c5 = int(door5_rc[0]), int(door5_rc[1])
    patch = F[r5 - R:r5 + R, c5 - R:c5 + R]
    occT = binary_dilation(T < 50, iterations=2).astype(np.float32)
    freeT = (T >= 250).astype(np.float32)
    sr0 = max(0, int(near_rc[0] - radius_px)); sr1 = min(T.shape[0], int(near_rc[0] + radius_px))
    sc0 = max(0, int(near_rc[1] - radius_px)); sc1 = min(T.shape[1], int(near_rc[1] + radius_px))
    occW, freeW = occT[sr0:sr1, sc0:sc1], freeT[sr0:sr1, sc0:sc1]
    cands = []
    for ang in np.arange(ang_g - 6, ang_g + 6.01, 0.5):
        P = rotate(patch, ang, 205)
        dr, dc = track(patch.shape, R, R, ang)
        occP = (P < 50).astype(np.float32)
        freeP = (P >= 250).astype(np.float32)
        s = (2 * fftconvolve(occW, occP[::-1, ::-1], mode="valid")
             + 0.3 * fftconvolve(freeW, freeP[::-1, ::-1], mode="valid")
             - 1.0 * fftconvolve(occW, freeP[::-1, ::-1], mode="valid"))
        for _ in range(6):
            k = s.argmax(); y, x = np.unravel_index(k, s.shape)
            cands.append((float(s[y, x]), float(ang), sr0 + y + dr, sc0 + x + dc))
            s[max(0, y - 15):y + 15, max(0, x - 15):x + 15] = -1e18
    cands.sort(reverse=True)
    picked = []
    for c in cands:   # 2 m 안의 후보는 같은 자리로 본다
        if all(np.hypot(c[2] - p[2], c[3] - p[3]) > 40 for p in picked):
            picked.append(c)
        if len(picked) == n:
            break
    return picked


def place(T, F, ang, door5_rc, door_target_rc):
    """floor5 를 ang 만큼 돌려, floor5 문 앞이 door_target_rc 에 오도록 대상 크기 캔버스에 놓는다."""
    Rimg = rotate(F, ang, 205)
    mr, mc = track(F.shape, door5_rc[0], door5_rc[1], ang)
    oy, ox = int(round(door_target_rc[0] - mr)), int(round(door_target_rc[1] - mc))
    P = np.full(T.shape, 205, np.uint8)
    h, w = Rimg.shape
    ys, xs = max(0, oy), max(0, ox)
    ye, xe = min(T.shape[0], oy + h), min(T.shape[1], ox + w)
    P[ys:ye, xs:xe] = Rimg[ys - oy:ye - oy, xs - ox:xe - ox]
    P = np.where(P <= 89, 0, np.where(P > 205, 254, 205)).astype(np.uint8)
    fy, fx = np.where(Rimg >= 250)
    lost = int(((fy + oy < 0) | (fy + oy >= T.shape[0]) | (fx + ox < 0) | (fx + ox >= T.shape[1])).sum())
    return P, (oy, ox), lost


def overlay(T, P):
    img = np.full(T.shape + (3,), 235, np.uint8)
    tf, ff = T >= 250, P >= 250
    img[ff & ~tf] = (255, 200, 200)      # floor5 에만 있는 길 (가져올 후보)
    img[tf & ~ff] = (190, 225, 255)      # 대상에만 있는 길
    img[tf & ff] = (255, 255, 255)
    img[P < 50] = (220, 40, 40)          # floor5 벽
    img[T < 50] = (30, 60, 200)          # 대상 벽
    return img


def write_pgm(path, a, comment):
    with open(path, "wb") as f:
        f.write(f"P5\n# {comment}\n{a.shape[1]} {a.shape[0]}\n255\n".encode())
        f.write(a.astype(np.uint8).tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True, help="고칠 층 yaml (예: maps/floor1.yaml)")
    ap.add_argument("--ref", default=DEFAULT_REF, help="기준 지도 yaml (기본 floor5.yaml)")
    ap.add_argument("--out", required=True, help="출력 폴더")
    ap.add_argument("--pick", type=int, default=0, help="엘리베이터 후보 번호 (0 = 후보 그림만)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    T, oT, resT = read_map(a.target)
    F, oF, resF = read_map(a.ref)
    if abs(resT - resF) > 1e-9:
        raise SystemExit(f"해상도가 다르다 ({resT} vs {resF}) — 이 도구는 같은 해상도만 다룬다")
    door5 = w2p(*DOOR5, oF, resF, F.shape[0])
    cabin5 = w2p(*CABIN5, oF, resF, F.shape[0])

    ang, dy, dx, ov = global_align(T, F)
    mr, mc = track(F.shape, door5[0], door5[1], ang)
    near = (mr + dy, mc + dx)
    print(f"전체 정렬: 회전 {ang}°, 이동 ({dy},{dx})칸, 대상 길의 {ov:.1%} 겹침")

    cands = elevator_candidates(T, F, ang, near, door5)
    S = 2
    r0, r1 = max(0, int(near[0] - 200)), min(T.shape[0], int(near[0] + 200))
    c0, c1 = max(0, int(near[1] - 200)), min(T.shape[1], int(near[1] + 200))
    z = Image.fromarray(np.stack([T[r0:r1, c0:c1]] * 3, -1)).resize(((c1 - c0) * S, (r1 - r0) * S), Image.NEAREST)
    d = ImageDraw.Draw(z)
    cols = [(220, 0, 0), (0, 150, 0), (0, 0, 220), (200, 0, 200), (230, 130, 0)]
    for i, (sc, an, rr, cc) in enumerate(cands):
        X, Y = (cc - c0) * S, (rr - r0) * S
        d.ellipse((X - 14, Y - 14, X + 14, Y + 14), outline=cols[i % 5], width=5)
        d.text((X + 18, Y - 10), str(i + 1), fill=cols[i % 5])
        print(f"  엘리베이터 후보 {i + 1}: 회전 {an}°, 문앞 칸 ({rr:.0f},{cc:.0f}), 점수 {sc:.0f}")
    z.save(os.path.join(a.out, "elevator_candidates.png"))
    print(f"후보 그림: {os.path.join(a.out, 'elevator_candidates.png')} — 엘리베이터 문 앞 번호를 골라 --pick N")
    if not a.pick:
        return

    sc, ang2, rr, cc = cands[a.pick - 1]
    P, (oy, ox), lost = place(T, F, ang2, door5, (rr, cc))
    cr, ccab = track(F.shape, cabin5[0], cabin5[1], ang2)
    cr, ccab = cr + oy, ccab + ox
    name = os.path.splitext(os.path.basename(a.target))[0]
    write_pgm(os.path.join(a.out, f"floor5_aligned_to_{name}.pgm"), P, f"floor5 aligned to {name}, rot {ang2} deg")
    img = overlay(T, P)
    im = Image.fromarray(img); d = ImageDraw.Draw(im)
    for (r_, c_), col in (((rr, cc), (0, 170, 0)), ((cr, ccab), (255, 140, 0))):
        d.ellipse((c_ - 14, r_ - 14, c_ + 14, r_ + 14), outline=col, width=6)
    im.save(os.path.join(a.out, f"overlay_{name}_blue-target_red-floor5.png"))
    result = {
        "target": a.target, "global": {"angle_deg": ang, "shift_px": [dy, dx], "overlap": round(ov, 3)},
        "elevator": {"angle_deg": float(ang2), "pick": a.pick,
                     "door_world_est": [round(float(v), 3) for v in p2w(rr, cc, oT, resT, T.shape[0])],
                     "cabin_world_est": [round(float(v), 3) for v in p2w(cr, ccab, oT, resT, T.shape[0])]},
        "floor5_free_px_outside_canvas": lost,
        "note": "엘리베이터 좌표는 지도 정렬로 계산한 추정값이다. 로봇에서 TF(map→base_link)로 다시 잴 것.",
    }
    json.dump(result, open(os.path.join(a.out, "result.json"), "w"), ensure_ascii=False, indent=1)
    print(json.dumps(result, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
