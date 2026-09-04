#!/usr/bin/env python3
"""B3 — 8/31 결과(0.55 m 촬영)와 9/2 결과(0.19~0.44 m 촬영)를 정량 대조한다.

왜 이게 검증인가
    두 산출물은 촬영일·로봇 자세·촬영거리(2.9배)·검출 파라미터·작성자가 전부 다르다.
    공유하는 건 '같은 물리 패널' 하나뿐이다. 그래서 일치하면 우연이 아니고, 어긋나면
    어디서 갈리는지가 곧 오차의 출처다.

두 가지를 따로 잰다 — 섞으면 원인을 못 가린다
  (A) 패널 내부 기하 (자세 무관): 행/열 간격, 버튼 지름, 평면 잔차, 법선.
      AMCL·TF·지도와 무관하다. 순수하게 '카메라+깊이+K'만의 성능이다.
  (B) 지도 절대좌표: map 프레임에서 같은 셀끼리 몇 mm 떨어져 있나.
      여기엔 AMCL 위치추정 오차와 지도 교체(9/2 17:59, 좌표계는 동일)가 함께 들어온다.
      (A)는 맞는데 (B)만 어긋나면 범인은 비전이 아니라 위치추정이다.

8/28 원본 사진은 커밋 2f38eb9 에서 지워졌다. meta.json 은 git 이력에서 읽어온다
(작업트리에 아무것도 복원하지 않는다 — git show 로 표준출력만 받는다).

입력(읽기 전용): out/final_report.json, out3d/*.json, git show 2f38eb9^:<8/28 meta>
출력:            out_b3/compare_0831_vs_0902.json
"""
import json, os, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out_b3')
REPO_META = ('src/blind_nav_system/blind_nav_system/snapshots/{}/meta.json')
DEAD_COMMIT = '2f38eb9^'          # 8/28 스냅샷이 아직 살아 있던 마지막 커밋

EPOCH_A = ['20260828T180736_snap', '20260828T180746_snap']    # 8/31 내 결과
EPOCH_B = '20260902T183106_엘리베이터안탑승장소(좌표)'          # 9/2, 패널 7개가 한 장에


def quat_to_R(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def git_meta(snap):
    out = subprocess.run(['git', 'show', f'{DEAD_COMMIT}:{REPO_META.format(snap)}'],
                         cwd=HERE, capture_output=True, text=True)
    if out.returncode != 0:
        raise SystemExit(f'git 이력에서 {snap}/meta.json 을 못 읽었다: {out.stderr[:200]}')
    return json.loads(out.stdout)


def base_to_map(meta):
    """map__base_link 이 없으면 None 을 준다. 조용히 amcl_pose 로 대체하지 않는다 —
    8/28 은 localization_available=false 라 amcl_pose 가 낡은 값이다(실제로 캐빈 안에서
    찍은 사진인데 기록된 위치는 승강장이다). 대체했다면 그럴듯한 숫자가 나오면서
    틀렸을 것이다."""
    tf = meta.get('tf', {}).get('map__base_link')
    if not tf:
        return None
    R = quat_to_R(tf['rotation'])
    t = np.array([tf['translation'][k] for k in 'xyz'])
    return lambda p: R @ np.asarray(p) + t


def panel_frame(P, cam_center):
    """평면 적합 + 중력정렬 면내축. 법선은 '카메라 쪽'으로 고정한다.

    왜 카메라 쪽인가: 옛 final.py 는 base_link 의 y 부호로 법선 방향을 정했는데,
    그건 로봇이 어느 쪽을 보고 있느냐에 따라 뒤집힌다. 8/28(yaw 171°)과
    9/2(yaw 75°)는 96° 차이라 그 규칙으로는 두 시기의 좌우축이 반대로 잡힌다.
    '법선은 관측한 카메라를 향한다'는 촬영자세와 무관하게 늘 같은 답을 준다.
    """
    c = P.mean(0)
    n = np.linalg.svd(P - c)[2][2]
    if n @ (np.asarray(cam_center) - c) < 0:
        n = -n
    zu = np.array([0., 0., 1.])
    e2 = zu - (zu @ n) * n; e2 /= np.linalg.norm(e2)     # 면내 '위' (중력 기준)
    e1 = np.cross(e2, n); e1 /= np.linalg.norm(e1)       # 면내 '가로'
    uv = np.stack([(P - c) @ e1, (P - c) @ e2], 1)
    resid = (P - c) @ n
    return c, n, e1, e2, uv, resid


def cluster1d(vals, tol=0.020):
    g = []
    for i in np.argsort(vals):
        for grp in g:
            if abs(vals[i] - np.mean([vals[j] for j in grp])) < tol:
                grp.append(i); break
        else:
            g.append([i])
    return g


def gridify(uv):
    rows = cluster1d(-uv[:, 1])          # 위에서 아래로
    cols = cluster1d(uv[:, 0])
    G = [[None]*len(cols) for _ in rows]
    collide = 0
    for r, grp in enumerate(rows):
        for i in grp:
            k = next(k for k, gg in enumerate(cols) if i in gg)
            if G[r][k] is not None: collide += 1
            G[r][k] = i
    row_c = [float(np.mean([-uv[i, 1] for i in g])) for g in rows]
    col_c = [float(np.mean([uv[i, 0] for i in g])) for g in cols]
    return G, row_c, col_c, collide


def pitches(centers):
    return [round((b - a)*1000, 1) for a, b in zip(centers, centers[1:])]


def load_epochA(snap):
    fr = json.load(open(os.path.join(HERE, 'out', 'final_report.json')))[snap]
    meta = git_meta(snap)
    labels = list(fr['labels'].keys())
    P = np.array([fr['labels'][k] for k in labels])          # base_link
    tf = meta['tf']['base_link__grip_color']
    cam = np.array([tf['translation'][k] for k in 'xyz'])
    return labels, P, cam, base_to_map(meta), meta


def load_epochB(snap):
    d = json.load(open(os.path.join(HERE, 'out3d', snap + '.json')))
    P, dia = [], []
    for c in d['circles']:
        zc = c['depth']['z_med_r3']
        if zc['n_valid'] == 0: continue
        P.append(zc['base_xyz']); dia.append(zc['diameter_mm'])
    tf = d['tf']['base_link__grip_color']
    cam = np.array([tf['translation'][k] for k in 'xyz'])
    return np.array(P), np.array(dia), cam, base_to_map(d), d


def describe(name, P, cam, dia=None):
    c, n, e1, e2, uv, resid = panel_frame(P, cam)
    G, rowc, colc, collide = gridify(uv)
    occ = [[x is not None for x in r] for r in G]
    rng = float(np.linalg.norm(P.mean(0) - cam))
    d = {'name': name, 'n_points': len(P),
         'cam_to_panel_m': round(rng, 3),
         'plane_rms_mm': round(float(np.sqrt((resid**2).mean())*1000), 2),
         'plane_max_mm': round(float(np.abs(resid).max()*1000), 2),
         'grid': f'{len(G)}×{len(G[0])}',
         'fill': [['●' if x else '·' for x in r] for r in occ],
         'grid_collisions': collide,
         'row_pitch_mm': pitches(rowc),
         'col_pitch_mm': pitches(colc),
         'normal_base': [round(float(x), 4) for x in n]}
    if dia is not None and len(dia):
        d['button_diam_mm'] = {'median': round(float(np.median(dia)), 1),
                               'min': round(float(dia.min()), 1),
                               'max': round(float(dia.max()), 1)}
    return d, G, uv, occ


def main():
    rep = {'purpose': '8/31(0.55m) vs 9/2(0.19~0.44m) 독립 측정 대조',
           'note_source_images': f'8/28 원본 사진은 커밋 2f38eb9 에서 삭제됨. meta 는 {DEAD_COMMIT} 에서 읽음.',
           'A_panel_intrinsic': {}, 'B_map_absolute': {}}

    # ── A: 패널 내부 기하 (자세·지도 무관)
    outs = {}
    for s in EPOCH_A:
        labels, P, cam, tomap, meta = load_epochA(s)
        d, G, uv, occ = describe(f'8/31 · {s}', P, cam)
        d['labels_order'] = labels
        rep['A_panel_intrinsic'][s] = d
        outs[s] = (labels, P, tomap, G, occ, meta)

    Pb, dia, camb, tomapb, db = load_epochB(EPOCH_B)
    d, Gb, uvb, occb = describe(f'9/2 · {EPOCH_B}', Pb, camb, dia)
    rep['A_panel_intrinsic'][EPOCH_B] = d

    # 9/2 근접 촬영분 전체에서 '자세와 무관한 불변량'만 뽑아 산포를 본다.
    # 0.19 m 에선 패널이 화면을 넘쳐 버튼이 일부만 잡히므로 격자 전체는 못 만든다.
    # 그래도 잡힌 점들의 평면 잔차와 최근접 버튼 간격은 잴 수 있다.
    import glob
    spread = []
    for f in sorted(glob.glob(os.path.join(HERE, 'out3d', '*.json'))):
        dd = json.load(open(f))
        pts = [c['depth']['z_med_r3'] for c in dd['circles'] if c['depth']['z_med_r3']['n_valid']]
        if len(pts) < 4: continue
        Q = np.array([q['base_xyz'] for q in pts])
        tfq = dd['tf']['base_link__grip_color']
        camq = np.array([tfq['translation'][k] for k in 'xyz'])
        _, _, _, _, _, rq = panel_frame(Q, camq)
        nn = [min(np.linalg.norm(Q[i]-Q[j]) for j in range(len(Q)) if j != i) for i in range(len(Q))]
        spread.append({'snap': dd['snapshot'], 'n': len(Q),
                       'range_m': round(float(np.median([q['z_m'] for q in pts])), 3),
                       'plane_rms_mm': round(float(np.sqrt((rq**2).mean())*1000), 2),
                       'nn_gap_mm': round(float(np.median(nn))*1000, 1),
                       'diam_median_mm': round(float(np.median(
                           [q['diameter_mm'] for q in pts if q['diameter_mm']])), 1)})
    rep['A_multiframe_0902'] = spread

    # ── B: 지도 절대좌표. 셀 구조가 같아야 대응이 성립한다.
    la, Pa, tomapa, Ga, occa, metaa = outs[EPOCH_A[0]]
    if tomapa is None:
        rep['B_map_absolute'] = {
            'possible': False,
            'reason': ('8/28 meta 의 tf.map__base_link 가 null 이다. '
                       'localization_available=false, missing 에 "tf:map__base_link" 가 찍혀 있다 '
                       '— 촬영 당시 map 프레임 자체가 없었다(AMCL/지도 미구동).'),
            'stale_amcl_evidence': {
                'recorded_amcl_8_28': metaa['amcl_pose'],
                'today_hall_panel_pose': [-47.454, 4.993, 168.4],
                'today_cabin_pose': [-47.824, 7.102, 68.9],
                'note': ('기록된 8/28 amcl(-47.62, 4.917, 171°)은 오늘의 승강장 자세와 17cm·2.5° 안에서 '
                         '일치한다. 그런데 그 사진은 캐빈 *안*에서 찍은 조작반이다(오늘 캐빈 자세는 '
                         '-47.82, 7.10). 즉 기록된 값은 이전 위치에서 멈춰 있던 낡은 값이다. '
                         '이걸 map 변환에 썼다면 2m 넘게 틀린 좌표가 그럴듯하게 나왔을 것이다.')},
            'consequence': ('8/31 산출물은 base_link 좌표만 의미가 있다. 지도 절대좌표로 승격할 수 없고, '
                            'DECISIONS #14 의 B3 "실제 press 좌표 대조"는 이 데이터로는 불가능하다. '
                            '9/2 수집분은 24세트 전부 map__base_link 가 살아 있어 이 문제가 없다.')}
        finish(rep)
        return
    same_shape = (len(Ga), len(Ga[0])) == (len(Gb), len(Gb[0]))
    same_fill = same_shape and occa == occb
    rep['B_map_absolute']['grid_shape_match'] = bool(same_shape)
    rep['B_map_absolute']['fill_pattern_match'] = bool(same_fill)
    rep['B_map_absolute']['A_fill'] = [['●' if x else '·' for x in r] for r in occa]
    rep['B_map_absolute']['B_fill'] = [['●' if x else '·' for x in r] for r in occb]

    if same_fill:
        rows = []
        for r in range(len(Ga)):
            for k in range(len(Ga[0])):
                ia, ib = Ga[r][k], Gb[r][k]
                if ia is None or ib is None: continue
                ma = tomapa(Pa[ia]); mb = tomapb(Pb[ib])
                rows.append({'cell': [r, k], 'label_8_31': la[ia],
                             'map_8_31': [round(float(x), 4) for x in ma],
                             'map_9_02': [round(float(x), 4) for x in mb],
                             'delta_mm': [round(float(x)*1000, 1) for x in (mb - ma)],
                             'dist_mm': round(float(np.linalg.norm(mb - ma))*1000, 1)})
        dist = np.array([r['dist_mm'] for r in rows])
        dxyz = np.array([r['delta_mm'] for r in rows])
        rep['B_map_absolute']['cells'] = rows
        rep['B_map_absolute']['summary'] = {
            'n': len(rows),
            'dist_mean_mm': round(float(dist.mean()), 1),
            'dist_max_mm': round(float(dist.max()), 1),
            'common_offset_mm': [round(float(x), 1) for x in dxyz.mean(0)],
            'after_removing_common_offset_mean_mm':
                round(float(np.linalg.norm(dxyz - dxyz.mean(0), axis=1).mean()), 1),
            'after_removing_common_offset_max_mm':
                round(float(np.linalg.norm(dxyz - dxyz.mean(0), axis=1).max()), 1),
            'interpretation': ('공통 오프셋은 AMCL 위치추정 차이(로봇 자세가 8/28 yaw 171° vs '
                               '9/2 yaw 75°, 그 사이 지도 파일도 교체됨)에서 온다. 오프셋을 뺀 '
                               '나머지가 비전·기하 자체의 불일치다.')}
        rep['B_map_absolute']['amcl'] = {
            '8_31': metaa['amcl_pose'], '9_02': db['amcl_pose']}

    finish(rep)


def finish(rep):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, 'compare_0831_vs_0902.json')
    json.dump(rep, open(p, 'w'), ensure_ascii=False, indent=1)

    print('══ (A) 패널 내부 기하 — 자세·지도 무관')
    for k, v in rep['A_panel_intrinsic'].items():
        print(f"  {v['name']}")
        print(f"     점 {v['n_points']}개, 촬영거리 {v['cam_to_panel_m']} m, 격자 {v['grid']} "
              f"{v['fill']} 충돌 {v['grid_collisions']}")
        print(f"     평면잔차 rms {v['plane_rms_mm']} / max {v['plane_max_mm']} mm")
        print(f"     행간격 {v['row_pitch_mm']} mm   열간격 {v['col_pitch_mm']} mm"
              + (f"   버튼지름 {v['button_diam_mm']}" if 'button_diam_mm' in v else ''))
    if rep.get('A_multiframe_0902'):
        print('\n══ (A2) 9/2 여러 장의 자세-무관 불변량')
        print(f"     {'폴더':40s} {'n':>2s} {'거리m':>6s} {'평면rms':>8s} {'최근접간격':>9s} {'지름':>6s}")
        for r in rep['A_multiframe_0902']:
            print(f"     {r['snap']:40s} {r['n']:2d} {r['range_m']:6.3f} "
                  f"{r['plane_rms_mm']:8.2f} {r['nn_gap_mm']:9.1f} {r['diam_median_mm']:6.1f}")
    b = rep['B_map_absolute']
    if not b.get('possible', True):
        print('\n══ (B) 지도 절대좌표 — 불가능')
        print(f"     {b['reason']}")
        print(f"     {b['stale_amcl_evidence']['note']}")
        print(f"     → {b['consequence']}")
        print(f"\n→ {p}")
        return
    print(f"\n══ (B) 지도 절대좌표 — 격자모양일치={b['grid_shape_match']} 채움일치={b['fill_pattern_match']}")
    print(f"     8/31 {b['A_fill']}\n     9/02 {b['B_fill']}")
    if 'summary' in b:
        s = b['summary']
        print(f"     같은 셀 {s['n']}개: 평균 {s['dist_mean_mm']} mm / 최대 {s['dist_max_mm']} mm")
        print(f"     공통 오프셋 {s['common_offset_mm']} mm  → 그걸 뺀 나머지 "
              f"평균 {s['after_removing_common_offset_mean_mm']} / 최대 "
              f"{s['after_removing_common_offset_max_mm']} mm")
        for r in b['cells']:
            print(f"       셀{r['cell']} {r['label_8_31']:>4}  Δ{r['delta_mm']} = {r['dist_mm']} mm")
    print(f"\n→ {p}")


if __name__ == '__main__':
    main()
