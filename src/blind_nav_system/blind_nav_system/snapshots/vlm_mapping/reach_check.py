#!/usr/bin/env python3
"""실기 테스트 전 점검 — 씬1/씬3/탑승지점 목표좌표가 9/2 실측 3D 와 맞는가.

세 가지를 각각 따로 확인한다.
  (1) 목표좌표가 정말 그 스냅샷의 amcl_pose 인가 (전사 오류 확인)
  (2) 그 스냅샷의 amcl_pose 가 같은 순간의 map__base_link TF 와 일치하는가
      — 8/28 데이터에서 amcl 이 낡은 값이었던 전례가 있다. 여기서 어긋나면
        목표좌표 자체가 틀린 자리를 가리킨다.
  (3) 그 자세에서 버튼이 팔 도달범위 안인가 — 팔 기하는 문서값이 아니라
      스냅샷 TF 에서 실측으로 뽑는다(신장량이 다른 장들을 회귀).

입력(읽기 전용): out3d/*.json, ../<폴더>/meta.json, scene_targets.yaml, location.yaml
출력:            out_b3/reach_check.json
"""
import json, os, glob, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out_b3')
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', '..'))

HALL_SNAPS = ['20260902T182307_5층버튼누르는지점', '20260902T182406_5층버튼수집중_1',
              '20260902T182417_5층버튼수집중_2', '20260902T182428_5층버튼수집중_3']
CAB_SNAPS = [os.path.basename(p)[:-5] for p in sorted(glob.glob(os.path.join(HERE, 'out3d', '*.json')))
             if '버튼수집' in p or '183106' in p]


def quat_to_R(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def yaw_of(q):
    return math.degrees(math.atan2(2*(q['w']*q['z'] + q['x']*q['y']),
                                   1 - 2*(q['y']**2 + q['z']**2)))


def load(snap):
    return json.load(open(os.path.join(HERE, 'out3d', snap + '.json')))


# ── (2) amcl_pose ↔ map__base_link TF 정합
def amcl_vs_tf():
    rows = []
    for p in sorted(glob.glob(os.path.join(HERE, 'out3d', '*.json'))):
        d = json.load(open(p))
        tf = d['tf'].get('map__base_link')
        a = d.get('amcl_pose')
        if not tf or not a:
            rows.append({'snap': d['snapshot'], 'tf_present': bool(tf), 'skipped': True}); continue
        t = tf['translation']
        dx, dy = (t['x'] - a['x'])*1000, (t['y'] - a['y'])*1000
        dyaw = yaw_of(tf['rotation']) - a['yaw_deg']
        dyaw = (dyaw + 180) % 360 - 180
        rows.append({'snap': d['snapshot'],
                     'amcl': [a['x'], a['y'], a['yaw_deg']],
                     'tf': [round(t['x'], 4), round(t['y'], 4), round(yaw_of(tf['rotation']), 2)],
                     'd_mm': [round(dx, 1), round(dy, 1)],
                     'd_yaw_deg': round(dyaw, 3),
                     'dist_mm': round(math.hypot(dx, dy), 1)})
    return rows


# ── (3) 팔 기하를 스냅샷 TF 에서 실측한다
def arm_axis():
    """wrist_extension 이 다른 장들로 회귀: p_grip(base) = p0 + ext * u.

    문서/URDF 를 안 읽고 실측만 쓴다. 신장 결측(None)인 장은 뺀다.
    """
    P, E = [], []
    for p in sorted(glob.glob(os.path.join(HERE, 'out3d', '*.json'))):
        d = json.load(open(p))
        e = d['joint_states'].get('wrist_extension')
        if e is None: continue
        t = d['tf']['base_link__grip_color']['translation']
        P.append([t['x'], t['y'], t['z']]); E.append(e)
    P = np.array(P); E = np.array(E)
    A = np.stack([E, np.ones_like(E)], 1)
    coef, *_ = np.linalg.lstsq(A, P, rcond=None)      # 행0 = 방향*1, 행1 = 절편
    u = coef[0]; p0 = coef[1]
    resid = P - (A @ coef)
    return {'n': len(E), 'ext_range': [round(float(E.min()), 4), round(float(E.max()), 4)],
            'dir_unit_base': [round(float(x), 4) for x in u/np.linalg.norm(u)],
            'mm_per_ext_unit': round(float(np.linalg.norm(u)), 4),
            'grip_at_ext0_base': [round(float(x), 4) for x in p0],
            'fit_resid_rms_mm': round(float(np.sqrt((resid**2).sum(1).mean())*1000), 2)}


# ── 버튼의 map 좌표 (평면 중심)
def panel_map_center(snaps, min_circles=1):
    pts = []
    for s in snaps:
        d = load(s)
        for c in d['circles']:
            zc = c['depth']['z_med_r3']
            if zc['n_valid']: pts.append(zc['map_xyz'])
    return np.array(pts)


def reach_from(pose_xy_yaw, panel_pts, arm, label):
    """목표 자세에서 각 버튼까지 필요한 신장량을 구한다.

    팔은 base_link 안에서 고정된 축 u 로만 뻗는다(위 실측). 그러니
    목표점을 base_link 로 옮긴 뒤 u 방향 성분이 '필요 신장', u 에 수직인
    수평 성분이 '옆으로 얼마나 빗나가는가'다. 세로(z)는 리프트가 담당한다.
    """
    x, y, yawd = pose_xy_yaw
    th = math.radians(yawd)
    R = np.array([[math.cos(th), -math.sin(th), 0],
                  [math.sin(th),  math.cos(th), 0],
                  [0, 0, 1]])
    t = np.array([x, y, 0.0])
    u = np.array(arm['dir_unit_base'])
    uh = np.array([u[0], u[1], 0.0]); uh /= np.linalg.norm(uh)   # 수평 성분만
    rows = []
    for p in panel_pts:
        pb = R.T @ (np.asarray(p) - t)          # base_link 로
        along = float(pb @ uh)                  # 팔 방향 성분
        lateral = float(pb[:2] @ np.array([-uh[1], uh[0]]))   # 옆으로 빗나감
        rows.append({'map': [round(float(v), 4) for v in p],
                     'along_m': round(along, 4), 'lateral_m': round(lateral, 4),
                     'height_m': round(float(pb[2]), 4),
                     'horiz_dist_m': round(float(np.linalg.norm(pb[:2])), 4)})
    a = np.array([r['along_m'] for r in rows])
    l = np.array([r['lateral_m'] for r in rows])
    h = np.array([r['height_m'] for r in rows])
    return {'label': label, 'pose': [x, y, yawd], 'n_buttons': len(rows),
            'along_m': {'min': round(float(a.min()), 4), 'max': round(float(a.max()), 4),
                        'median': round(float(np.median(a)), 4)},
            'lateral_m': {'min': round(float(l.min()), 4), 'max': round(float(l.max()), 4)},
            'height_m': {'min': round(float(h.min()), 4), 'max': round(float(h.max()), 4)},
            'buttons': rows}


def main():
    rep = {}
    rep['arm_axis_measured'] = arm = arm_axis()
    rep['amcl_vs_map_tf'] = rows = amcl_vs_tf()
    bad = [r for r in rows if not r.get('skipped') and r['dist_mm'] > 5]
    rep['amcl_vs_map_tf_summary'] = {
        'n': len([r for r in rows if not r.get('skipped')]),
        'max_dist_mm': max((r['dist_mm'] for r in rows if not r.get('skipped')), default=None),
        'max_dyaw_deg': max((abs(r['d_yaw_deg']) for r in rows if not r.get('skipped')), default=None),
        'n_over_5mm': len(bad)}

    hall = panel_map_center(HALL_SNAPS)
    cab = panel_map_center([s for s in CAB_SNAPS if '183106' in s or '수집_3' in s
                            or '수집_4' in s or '수집_5' in s])
    rep['panel_points'] = {'hall_n': len(hall), 'cabin_n': len(cab),
                           'hall_map_mean': [round(float(v), 4) for v in hall.mean(0)] if len(hall) else None,
                           'cabin_map_mean': [round(float(v), 4) for v in cab.mean(0)] if len(cab) else None}

    def tf_pose(snap):
        d = load(snap); t = d['tf']['map__base_link']
        return (round(t['translation']['x'], 4), round(t['translation']['y'], 4),
                round(yaw_of(t['rotation']), 2))

    rep['reach'] = [
        reach_from(tf_pose('20260902T182307_5층버튼누르는지점'), hall, arm,
                   '실제 촬영 자세(map TF) → 승강장 호출패널'),
        reach_from(tf_pose('20260902T182811_엘리베이터버튼수집_3'), cab, arm,
                   '실제 촬영 자세(map TF) → 캐빈 조작반'),
        reach_from((-47.454, 4.993, 168.4), hall, arm, 'location.yaml 탑승지점(신) → 승강장 호출패널'),
        reach_from((-47.668, 4.9445, 172.35), hall, arm, 'location.yaml 탑승지점(구) → 승강장 호출패널'),
        reach_from((-47.819, 7.057, 69.9), cab, arm, 'scene_targets "3"(신) → 캐빈 조작반'),
        reach_from((-47.825, 7.254, 74.7), cab, arm, 'scene_targets "3"(구) → 캐빈 조작반'),
    ]

    os.makedirs(OUT, exist_ok=True)
    json.dump(rep, open(os.path.join(OUT, 'reach_check.json'), 'w'), ensure_ascii=False, indent=1)

    print('══ 팔 기하 (스냅샷 TF 실측 회귀)')
    print(f"   {arm['n']}장, 신장 {arm['ext_range']}, 방향(base) {arm['dir_unit_base']}, "
          f"적합잔차 {arm['fit_resid_rms_mm']} mm")
    print('\n══ amcl_pose ↔ map__base_link TF 정합 (24장)')
    s = rep['amcl_vs_map_tf_summary']
    print(f"   최대 어긋남 {s['max_dist_mm']} mm / {s['max_dyaw_deg']}° · 5mm 초과 {s['n_over_5mm']}장 / {s['n']}장")
    for r in rows:
        if not r.get('skipped') and r['dist_mm'] > 5:
            print(f"     ⚠ {r['snap']}: amcl{r['amcl']} vs tf{r['tf']} → {r['dist_mm']}mm, {r['d_yaw_deg']}°")
    print('\n══ 도달 검사 (버튼까지 필요한 팔 신장량)')
    print(f"   승강장 패널 점 {rep['panel_points']['hall_n']}개, 캐빈 패널 점 {rep['panel_points']['cabin_n']}개")
    for r in rep['reach']:
        print(f"\n   {r['label']}")
        print(f"     자세 {r['pose']}  버튼 {r['n_buttons']}개")
        print(f"     팔방향 거리 {r['along_m']['min']}~{r['along_m']['max']} m (중앙 {r['along_m']['median']})")
        print(f"     옆으로 빗나감 {r['lateral_m']['min']}~{r['lateral_m']['max']} m")
        print(f"     높이(base) {r['height_m']['min']}~{r['height_m']['max']} m")
    print(f"\n→ {os.path.join(OUT, 'reach_check.json')}")


if __name__ == '__main__':
    main()
