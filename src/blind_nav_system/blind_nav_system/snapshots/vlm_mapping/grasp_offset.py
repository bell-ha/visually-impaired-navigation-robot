#!/usr/bin/env python3
"""손끝(link_grasp_center)과 그리퍼 카메라의 고정 오프셋을 URDF 에서 계산한다.

왜 관절값이 필요 없나: 둘 다 손목 아래에 '고정 조인트'로만 매달려 있다. 그래서 둘 사이의
변환은 리프트·신장·요 값과 무관한 상수다. 공통 조상까지 올라가 고정 변환만 곱하면 된다.
로봇을 움직이지 않고, TF 구독도 없이, 파일만 읽어서 구한다.

쓰는 URDF: 이 로봇의 캘리브레이션 산출물(head_calibrated_*.urdf). 일반 모델이 아니다.
출력: out_b3/grasp_offset.json
"""
import json, os, glob, math
import xml.etree.ElementTree as ET
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out_b3')
URDF_GLOB = '/home/hello-robot/stretch_user/*/calibration_ros/head_calibrated_*.urdf'
CAM = 'gripper_camera_color_optical_frame'
TIP = 'link_grasp_center'


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp,   cp*sr,            cp*cr]])


def load(urdf):
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall('joint'):
        ch = j.find('child').get('link'); pa = j.find('parent').get('link')
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()] if o is not None else [0, 0, 0]
        joints[ch] = {'parent': pa, 'type': j.get('type'), 'name': j.get('name'),
                      't': np.array(xyz), 'R': rpy_to_R(*rpy)}
    return joints


def chain_to_root(joints, link):
    out = []
    while link in joints:
        out.append(joints[link]); link = joints[link]['parent']
    return out, link


def pose_from(joints, link, stop):
    """link 좌표를 stop 링크 좌표로 옮기는 (R, t). 경로에 비고정 조인트가 있으면 알린다."""
    R = np.eye(3); t = np.zeros(3); moving = []
    cur = link
    while cur != stop:
        j = joints[cur]
        if j['type'] != 'fixed':
            moving.append(j['name'])
        R = j['R'] @ R
        t = j['R'] @ t + j['t']
        cur = j['parent']
    return R, t, moving


def main():
    urdfs = sorted(glob.glob(URDF_GLOB))
    if not urdfs:
        raise SystemExit('캘리브레이션 URDF 를 못 찾았다')
    urdf = urdfs[-1]
    joints = load(urdf)

    ca, _ = chain_to_root(joints, CAM)
    ta, _ = chain_to_root(joints, TIP)
    setc = []
    cur = CAM
    while cur in joints: setc.append(cur); cur = joints[cur]['parent']
    setc.append(cur)
    cur = TIP; anc = None
    while True:
        if cur in setc: anc = cur; break
        if cur not in joints: break
        cur = joints[cur]['parent']

    Rc, tc, mc = pose_from(joints, CAM, anc)     # 카메라 → 공통조상
    Rt, tt, mt = pose_from(joints, TIP, anc)     # 손끝  → 공통조상
    # 카메라 좌표계에서 본 손끝 위치
    t_cam_to_tip = Rc.T @ (tt - tc)
    R_cam_to_tip = Rc.T @ Rt

    rep = {'urdf': urdf, 'common_ancestor': anc,
           'moving_joints_on_path': sorted(set(mc + mt)),
           'tip_in_camera_frame_m': [round(float(x), 5) for x in t_cam_to_tip],
           'distance_m': round(float(np.linalg.norm(t_cam_to_tip)), 5)}

    # 스냅샷 자세에 실제로 적용해서 base_link 에서의 손끝 위치를 낸다
    rows = []
    for p in sorted(glob.glob(os.path.join(HERE, 'out3d', '*.json'))):
        d = json.load(open(p))
        tf = d['tf']['base_link__grip_color']
        q = tf['rotation']
        x, y, z, w = q['x'], q['y'], q['z'], q['w']
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                      [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
        c = np.array([tf['translation'][k] for k in 'xyz'])
        tip = R @ t_cam_to_tip + c
        e = d['joint_states'].get('wrist_extension')
        rows.append({'snap': d['snapshot'], 'ext': e,
                     'cam_base': [round(float(v), 4) for v in c],
                     'tip_base': [round(float(v), 4) for v in tip],
                     'tip_minus_cam_y_m': round(float(tip[1] - c[1]), 4)})
    rep['per_snapshot'] = rows

    ext0 = [r for r in rows if r['ext'] is not None and r['ext'] < 0.05]
    if ext0:
        tipy = np.mean([r['tip_base'][1] for r in ext0])
        camy = np.mean([r['cam_base'][1] for r in ext0])
        rep['reach_budget'] = {
            'note': ('팔은 base_link −y 로만 뻗는다(스냅샷 TF 회귀 실측: 방향 [0, -0.9996, -0.029]). '
                     '신장 0 일 때의 위치에 ARM_EXT_MAX 를 더한 값이 도달 상한이다.'),
            'ARM_EXT_MAX_m': 0.50,
            'cam_y_at_ext0_m': round(float(camy), 4),
            'tip_y_at_ext0_m': round(float(tipy), 4),
            'tip_beyond_cam_m': round(float(camy - tipy), 4),
            'reach_limit_camera_ref_m': round(float(-camy + 0.50), 4),
            'reach_limit_fingertip_m': round(float(-tipy + 0.50), 4)}

    os.makedirs(OUT, exist_ok=True)
    json.dump(rep, open(os.path.join(OUT, 'grasp_offset.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"URDF: {urdf}")
    print(f"공통 조상 링크: {anc}   경로상 비고정 조인트: {rep['moving_joints_on_path'] or '없음(순수 고정)'}")
    print(f"카메라 좌표계에서 본 손끝: {rep['tip_in_camera_frame_m']} m  (거리 {rep['distance_m']} m)")
    if 'reach_budget' in rep:
        b = rep['reach_budget']
        print(f"\n신장=0 일 때  카메라 y={b['cam_y_at_ext0_m']}  손끝 y={b['tip_y_at_ext0_m']}"
              f"  → 손끝이 카메라보다 {b['tip_beyond_cam_m']*1000:.0f} mm 더 나가 있다")
        print(f"도달 상한(손끝 기준) = {b['reach_limit_fingertip_m']} m")
        print(f"도달 상한(카메라 기준) = {b['reach_limit_camera_ref_m']} m")
    print(f"\n→ {os.path.join(OUT, 'grasp_offset.json')}")


if __name__ == '__main__':
    main()
