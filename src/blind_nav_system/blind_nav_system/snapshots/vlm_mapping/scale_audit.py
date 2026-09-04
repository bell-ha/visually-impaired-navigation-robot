#!/usr/bin/env python3
"""B0 감사 — grip_depth.png 의 단위(m/unit)를 '가정'이 아니라 '측정'으로 확정한다.

왜 필요한가
    meta.json 24개 전부 depth_scale_m_per_unit 필드가 없다. 우리는 "깊이 중앙값 192가
    0.19 m 와 맞으니 mm"라고 추론했지만, 그 0.19 m 자체가 눈대중이라 순환논증이다.
    RealSense D405 는 SDK 기본 depth unit 이 0.1 mm 인 모드도 있어서 10배 오차가
    실제로 가능한 실수다. 스케일이 k 배 틀리면 복원된 3D 전체가 k 배로 늘어나므로
    "버튼 지름 30mm"같은 자체 검산으로는 절대 못 잡는다(전부 같이 늘어난다).

측정 원리 — 깊이를 전혀 쓰지 않는 자(尺)를 하나 가져온다
    리프트 관절은 엔코더로 mm 단위를 아는 '자'다. 로봇이 멈춘 채 팔만 위아래로 움직인
    묶음에서, 카메라 중심 C_i 는 TF(=엔코더+URDF)로 정확히 안다. 같은 버튼을 여러 각도
    에서 본 광선들을 삼각측량하면 깊이 없이 3D 점 P 가 나온다. 그러면
        s = |P - C_i| / raw_i        (raw_i = 그 화소의 uint16 원값)
    이 스케일이다. 분자는 엔코더 유래(미터), 분모는 깊이 원값(단위 미상) → s 의 단위가
    바로 m/unit 이다. 순환하지 않는다.

전제와 그 검증
    · 로봇 베이스가 안 움직여야 base_link 가 정지 프레임이다.
      → amcl_pose 가 완전히 동일한 폴더끼리만 묶는다(정지 시 AMCL 은 갱신 자체를 안 한다).
    · 대응(어느 원이 어느 원인지)이 필요하다.
      → 임시로 s0=0.001 을 써서 근처끼리 묶는다. 대응에만 쓰고 s 계산엔 안 쓴다.
        s 가 10배 틀렸다면 이 대응이 통째로 실패하므로 그것도 신호다(자진 기록).

입력(읽기 전용): out3d/*.json (code-editor 산출물), ../<폴더>/meta.json
출력:            out_b3/scale_audit.json
"""
import json, os, glob, itertools
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT3D = os.path.join(HERE, 'out3d')
OUT = os.path.join(HERE, 'out_b3')

S0_ASSOC = 0.001      # 대응(짝짓기) 전용 임시 스케일. s 계산에는 안 쓴다.
ASSOC_TOL_M = 0.025   # 짝짓기 허용 반경 25mm (버튼 간격 ~60mm 의 절반 미만)
MIN_VIEWS = 2         # 삼각측량 최소 뷰 수. 2뷰도 베이스라인이 크면 유효하다
MIN_PERP_BASE_M = 0.030  # 시선에 수직인 베이스라인 하한 30mm. 이하는 퇴화로 보고 버린다


def quat_to_R(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def load_view(path):
    d = json.load(open(path))
    tf = d['tf']['base_link__grip_color']
    R = quat_to_R(tf['rotation'])
    C = np.array([tf['translation'][k] for k in 'xyz'])   # 카메라 중심 (base_link)
    obs = []
    for c in d['circles']:
        zc = c['depth']['z_med_r3']
        if zc['n_valid'] == 0:
            continue
        nx, ny = c['norm_undistorted']
        m_cam = np.array([nx, ny, 1.0])          # 정규화 광선 (스케일 미정)
        dir_base = R @ m_cam
        dir_base = dir_base / np.linalg.norm(dir_base)
        obs.append({
            'idx': c['idx'], 'u': c['u'], 'v': c['v'],
            'raw': float(zc['median_mm']),        # uint16 원값. 단위는 아직 모른다
            'm_cam': m_cam,                       # |m_cam| != 1 (z=1 정규화)
            'dir': dir_base,                      # 단위 광선
            'p_s0': C + R @ (S0_ASSOC * zc['median_mm'] * m_cam),   # 짝짓기 전용
        })
    return {'snap': d['snapshot'], 'R': R, 'C': C, 'obs': obs,
            'amcl': (round(d['amcl_pose']['x'], 3), round(d['amcl_pose']['y'], 3),
                     round(d['amcl_pose']['yaw_deg'], 1)),
            'lift': d['joint_states']['joint_lift'],
            'ext': d['joint_states'].get('wrist_extension') or 0.0}


def triangulate(rays):
    """rays = [(C, d_unit), ...] → 광선들에 대한 최소제곱 최근접점. 깊이 미사용."""
    A = np.zeros((3, 3)); b = np.zeros(3)
    for C, d in rays:
        P = np.eye(3) - np.outer(d, d)     # 광선에 수직인 성분만 벌준다
        A += P; b += P @ C
    return np.linalg.solve(A, b)


def main():
    views = [load_view(p) for p in sorted(glob.glob(os.path.join(OUT3D, '*.json')))]
    # amcl 이 완전히 같은 것끼리 = 로봇이 확실히 정지해 있던 묶음
    groups = {}
    for v in views:
        groups.setdefault(v['amcl'], []).append(v)

    report = {'method': '깊이 미사용 광선 삼각측량 vs 깊이 원값 → s = |P-C| / raw',
              'assoc_scale_used_for_matching_only': S0_ASSOC,
              'groups': []}
    all_s = []
    by_axis = {}

    for amcl, gv in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        lifts = [v['lift'] for v in gv]
        Cs = np.array([v['C'] for v in gv])
        baseline = max(float(np.linalg.norm(a-b)) for a, b in itertools.combinations(Cs, 2)) \
                   if len(gv) > 1 else 0.0
        # 어느 축으로 움직였나 = 어느 관절이 베이스라인을 만들었나
        span = (Cs.max(0) - Cs.min(0))
        axis = ['x', 'y', 'z'][int(np.argmax(span))]
        g = {'amcl_xyyaw': list(amcl), 'n_views': len(gv),
             'baseline_m': round(baseline, 4),
             'baseline_axis': axis,
             'cam_span_xyz_mm': [round(float(x)*1000, 1) for x in span],
             'lift_range_m': round(max(lifts) - min(lifts), 4),
             'ext_range_m': round(max(v['ext'] for v in gv) - min(v['ext'] for v in gv), 4),
             'snaps': [v['snap'] for v in gv]}
        if len(gv) < MIN_VIEWS or baseline < 0.02:
            g['skipped'] = f'뷰 {len(gv)}개 / 카메라 베이스라인 {baseline*1000:.0f}mm — 삼각측량 기하 부족'
            report['groups'].append(g); continue

        ref = max(gv, key=lambda v: len(v['obs']))     # 원이 가장 많은 뷰를 기준
        tracks = []
        for o in ref['obs']:
            tr = [(ref, o)]
            for v in gv:
                if v is ref: continue
                best, bd = None, 1e9
                for o2 in v['obs']:
                    dd = np.linalg.norm(o2['p_s0'] - o['p_s0'])
                    if dd < bd: best, bd = o2, dd
                if best is not None and bd < ASSOC_TOL_M:
                    tr.append((v, best))
            if len(tr) >= MIN_VIEWS:
                tracks.append(tr)

        g['ref_snap'] = ref['snap']
        g['n_ref_circles'] = len(ref['obs'])
        g['n_tracks'] = len(tracks)
        g['tracks'] = []
        for tr in tracks:
            Ct = np.array([v['C'] for v, _ in tr])
            base_full = max(float(np.linalg.norm(a-b))
                            for a, b in itertools.combinations(Ct, 2)) if len(tr) > 1 else 0.0
            mean_dir = np.mean([o['dir'] for _, o in tr], axis=0)
            mean_dir /= np.linalg.norm(mean_dir)
            # 시선에 수직인 베이스라인 = 실제로 삼각측량을 가능하게 하는 성분
            perp = [np.linalg.norm((a-b) - ((a-b) @ mean_dir) * mean_dir)
                    for a, b in itertools.combinations(Ct, 2)]
            base_perp = max(perp) if perp else 0.0
            P = triangulate([(v['C'], o['dir']) for v, o in tr])
            # 삼각측량 잔차: 각 광선과 P 의 거리
            resid = [float(np.linalg.norm(np.cross(o['dir'], P - v['C']))) for v, o in tr]
            ss = [float(np.linalg.norm(P - v['C']) / o['raw']) for v, o in tr]
            # 광선-깊이 정합: 깊이가 재는 건 z(광축 성분)지 사거리가 아니다 → z 성분으로 다시
            ss_z = []
            for v, o in tr:
                p_cam = v['R'].T @ (P - v['C'])
                ss_z.append(float(p_cam[2] / o['raw']))
            t = {'n_views': len(tr),
                 'baseline_mm': round(base_full*1000, 1),
                 'baseline_perp_mm': round(base_perp*1000, 1),
                 'ref_px': [tr[0][1]['u'], tr[0][1]['v']],
                 'P_base': [round(float(x), 5) for x in P],
                 'tri_resid_mm': round(float(np.mean(resid))*1000, 2),
                 'raw_range': [min(o['raw'] for _, o in tr), max(o['raw'] for _, o in tr)],
                 's_from_slantrange': round(float(np.median(ss)), 7),
                 's_from_opticalz': round(float(np.median(ss_z)), 7)}
            g['tracks'].append(t)
            # 채택 조건: 시선수직 베이스라인이 충분하고(퇴화 배제) 잔차가 작을 것
            t['accepted'] = bool(base_perp >= MIN_PERP_BASE_M and t['tri_resid_mm'] < 4.0)
            if t['accepted']:
                all_s.append(t['s_from_opticalz'])
                by_axis.setdefault(g['baseline_axis'], []).append(t['s_from_opticalz'])
        report['groups'].append(g)

    if all_s:
        a = np.array(all_s)
        report['verdict'] = {
            'n_tracks_used': len(all_s),
            's_median_m_per_unit': round(float(np.median(a)), 7),
            's_mean': round(float(a.mean()), 7),
            's_std': round(float(a.std()), 7),
            's_min': round(float(a.min()), 7),
            's_max': round(float(a.max()), 7),
            'ratio_to_0.001': round(float(np.median(a)) / 0.001, 4),
            'accept_rule': f'시선수직 베이스라인 ≥ {MIN_PERP_BASE_M*1000:.0f}mm 이고 삼각측량 잔차 < 4mm',
            'by_baseline_axis': {k: {'n': len(v), 's_median': round(float(np.median(v)), 7)}
                                 for k, v in sorted(by_axis.items())},
            'degeneracy_note': ('s 와 베이스라인 길이는 이 실험에서 정확히 축퇴한다. '
                                's=0.00106 은 "깊이 단위가 1.06mm" 와 "카메라 실제 이동이 '
                                'TF 보다 6% 작다" 를 구분하지 못한다. 서로 다른 관절(리프트=z축, '
                                '팔신장=x/y축)이 같은 s 를 주면 원인은 깊이 쪽, 다르면 그 관절 TF 쪽이다.'),
        }
    else:
        report['verdict'] = {'n_tracks_used': 0, 'note': '유효 트랙 없음 — 판정 불가'}

    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, 'scale_audit.json')
    json.dump(report, open(p, 'w'), ensure_ascii=False, indent=1)
    print(json.dumps(report['verdict'], ensure_ascii=False, indent=1))
    for g in report['groups']:
        if 'skipped' in g:
            print(f"  건너뜀 amcl={g['amcl_xyyaw']}: {g['skipped']}")
        else:
            print(f"  amcl={g['amcl_xyyaw']} 뷰{g['n_views']}개 "
                  f"베이스라인{g['baseline_m']*1000:.0f}mm({g['baseline_axis']}축, "
                  f"리프트{g['lift_range_m']*1000:.0f}/신장{g['ext_range_m']*1000:.0f}) "
                  f"→ 트랙 {g['n_tracks']}개")
            for t in g['tracks']:
                mark = ' ' if t['accepted'] else '✗'
                print(f"   {mark} px{t['ref_px']} {t['n_views']}뷰 raw{t['raw_range']} "
                      f"베이스{t['baseline_mm']:.0f}/수직{t['baseline_perp_mm']:.0f}mm "
                      f"잔차{t['tri_resid_mm']}mm  s={t['s_from_opticalz']}")
    print(f"\n→ {p}")


if __name__ == '__main__':
    main()
