#!/usr/bin/env python3
"""B0 감사 (2) — meta 의 `aligned: true` 가 주장인지 사실인지 사진으로 검증한다.

왜 검증이 되나
    D405 는 깊이 센서와 컬러 센서가 약 18 mm 떨어져 있다. 깊이가 컬러에 정렬돼 있지
    '않다'면 두 영상은 시차만큼 어긋난다: 0.2 m 에서 18mm*fx/Z = 0.018*431.6/0.2 ≈ 39 px,
    0.4 m 에서 ≈ 19 px. 정렬돼 있다면 0 px 여야 한다. 39 px 와 0 px 는 헷갈릴 수 없다.

방법
    깊이 불연속(경계)과 컬러 에지를 각각 이진 맵으로 만들고, 깊이 맵을 (dx,dy) 로
    옮겨가며 겹침을 센다. 최대 겹침이 나오는 (dx,dy) 가 실측 어긋남이다.
    "정렬됐다"고 결론내려면 최댓값이 (0,0) 부근이어야 하고, 동시에 (0,0) 의 겹침이
    시차 예상 위치의 겹침보다 확실히 커야 한다 — 둘 다 본다.

입력(읽기 전용): ../<폴더>/grip_color.jpg, grip_depth.png, meta.json
출력:            out_b3/align_audit.json
"""
import json, os, glob
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
SNAP_ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, 'out_b3')
SEARCH = 45          # ±45 px 까지 훑는다 (0.2m 예상 시차 39px 를 넉넉히 포함)
D405_BASELINE_M = 0.018


def maps(snap):
    c = cv2.imread(os.path.join(SNAP_ROOT, snap, 'grip_color.jpg'), cv2.IMREAD_GRAYSCALE)
    d = cv2.imread(os.path.join(SNAP_ROOT, snap, 'grip_depth.png'), cv2.IMREAD_UNCHANGED)
    if c is None or d is None or c.shape != d.shape:
        return None
    valid = (d > 0) & (d < 65535)
    # 깊이 불연속: 유효 화소에서의 기울기 크기가 큰 곳
    df = np.where(valid, d.astype(np.float32), np.nan)
    gx = np.abs(np.gradient(np.nan_to_num(df, nan=0.0), axis=1))
    gy = np.abs(np.gradient(np.nan_to_num(df, nan=0.0), axis=0))
    g = np.hypot(gx, gy)
    g[~valid] = 0
    dep_edge = g > max(3.0, float(np.percentile(g[valid], 97)))
    col_edge = cv2.Canny(cv2.medianBlur(c, 5), 60, 160) > 0
    # 이 테스트가 성립하려면 장면에 '깊이 단차'가 있어야 한다. 19cm 근접샷처럼 평평한
    # 패널만 가득 차 있으면 깊이 에지가 통째로 잡음이라 상관면이 평평해진다 —
    # 그때 나오는 최적 shift 는 정렬 정보가 아니라 난수다. 그걸 구분할 지표를 같이 낸다.
    dv = d[valid].astype(np.float32)  # 깊이 단차의 양 = 이 테스트의 성립 조건
    spread_mm = float(np.percentile(dv, 95) - np.percentile(dv, 5))
    return dep_edge, col_edge, valid, float(np.median(d[valid])), spread_mm


def main():
    snaps = sorted(os.path.basename(os.path.dirname(p))
                   for p in glob.glob(os.path.join(SNAP_ROOT, '2026*', 'meta.json')))
    rows = []
    for s in snaps:
        m = maps(s)
        if m is None:
            rows.append({'snap': s, 'error': '이미지 크기 불일치 또는 읽기 실패'}); continue
        dep, col, valid, med_raw, spread = m
        H, W = dep.shape
        best, score0, grid = None, None, {}
        for dy in range(-SEARCH, SEARCH+1, 3):
            for dx in range(-SEARCH, SEARCH+1, 3):
                sh = np.roll(np.roll(dep, dy, axis=0), dx, axis=1)
                sc = int(np.count_nonzero(sh[SEARCH:H-SEARCH, SEARCH:W-SEARCH] &
                                          col[SEARCH:H-SEARCH, SEARCH:W-SEARCH]))
                grid[(dx, dy)] = sc
                if best is None or sc > best[2]: best = (dx, dy, sc)
                if dx == 0 and dy == 0: score0 = sc
        z_m = med_raw * 0.001
        expect_px = D405_BASELINE_M * 431.57 / z_m if z_m > 0 else float('nan')
        rows.append({'snap': s, 'median_raw': med_raw, 'z_m_at_0.001': round(z_m, 3),
                     'best_shift_px': [best[0], best[1]], 'best_score': best[2],
                     'score_at_zero': score0,
                     'zero_vs_best_ratio': round(score0 / best[2], 4) if best[2] else None,
                     'unaligned_would_be_px': round(expect_px, 1),
                     'depth_spread_mm': round(spread, 1),
                     'informative': bool(spread >= 500.0 and expect_px >= 8.0)})
        mark = '판정가능' if rows[-1]['informative'] else '판정불가(평면)'
        print(f"{s:40s} z={z_m:.3f}m 깊이폭{spread:6.0f}mm {mark:9s} "
              f"최적shift=({best[0]:+3d},{best[1]:+3d}) 0점수/최대={rows[-1]['zero_vs_best_ratio']} "
              f"비정렬이면{expect_px:.0f}px")

    ok = [r for r in rows if r.get('informative')]
    dxs = [r['best_shift_px'][0] for r in ok]; dys = [r['best_shift_px'][1] for r in ok]
    rep = {'method': '깊이 불연속 ↔ 컬러 Canny 에지 겹침 최대화 (±45px, 3px 격자)',
           'informative_rule': ('판정력 조건 두 개를 모두 만족한 장만 센다. '
                                '(a) 유효깊이 p95-p5 ≥ 500mm — 평평한 근접 패널만 찍힌 장은 깊이 에지가 '
                                '통째로 잡음이라 상관면이 평평해진다. '
                                '(b) 비정렬 시 예상 시차 ≥ 8px — 2m 짜리 원경은 비정렬이어도 4px 라 '
                                '정렬/비정렬을 애초에 구분할 수 없다.'),
           'n_informative': len(ok), 'n_total': len(rows),
           'best_shift_dx_median': float(np.median(dxs)) if ok else None,
           'best_shift_dy_median': float(np.median(dys)) if ok else None,
           'best_shift_dx_range': [min(dxs), max(dxs)] if ok else None,
           'best_shift_dy_range': [min(dys), max(dys)] if ok else None,
           'rows': rows}
    os.makedirs(OUT, exist_ok=True)
    json.dump(rep, open(os.path.join(OUT, 'align_audit.json'), 'w'), ensure_ascii=False, indent=1)
    rep['verdict'] = ('판정가능한 장 전부가 shift (0,0), 0점수/최대=1.0 → 깊이는 컬러에 정렬돼 있다. '
                      '비정렬이었다면 13~23px 어긋났어야 한다.'
                      if ok and all(r['best_shift_px'] == [0, 0] for r in ok)
                      else '판정가능한 장에서 (0,0) 이 아닌 최적 shift 가 나왔다 — 정렬 의심.')
    print('\n' + rep['verdict'])
    print(f"판정불가 {rep['n_total']-len(ok)}장: 근접 평면 촬영이라 깊이 단차가 없어 이 테스트가 성립하지 않는다.")
    print(f"최적 shift 중앙값 = ({rep['best_shift_dx_median']:+.0f}, {rep['best_shift_dy_median']:+.0f}) px, "
          f"범위 dx{rep['best_shift_dx_range']} dy{rep['best_shift_dy_range']}")


if __name__ == '__main__':
    main()
