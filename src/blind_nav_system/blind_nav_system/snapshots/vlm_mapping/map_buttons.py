"""엘베 버튼 3D 매핑 프로토타입 (레인B / B2 예행)

파이프라인:
  1) VLM(=Claude)이 이미지를 보고 패널 ROI + 각 버튼의 '의미 라벨'을 준다  (labels_*.json)
  2) HoughCircles가 ROI 안에서 원 중심을 '정밀하게' 잡는다               (검출)
  3) depth(16bit mm) + K로 픽셀→카메라광학계 3D 역투영
  4) meta.tf[base_link__grip_color] 쿼터니언으로 base_link 3D 변환
  5) 격자 구조 복원 → button_layout.json(수기 정답지)과 대조

전제(meta에 없어서 가정한 값 — B0 항목):
  depth_scale = 0.001 m/unit,  depth는 color에 aligned(같은 K)
"""
import cv2, numpy as np, json, os, sys

DEPTH_SCALE = 0.001          # 가정 (meta에 키 없음)
SNAPDIR = os.path.join('..', sys.argv[1] if len(sys.argv) > 1 else '20260828T180736_snap')
ROI = (555, 140, 810, 300)   # x0,y0,x1,y1 — VLM이 지정한 버튼 패널 영역


def quat_to_R(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def detect_circles(img, roi):
    x0, y0, x1, y1 = roi
    g = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 3)
    c = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=18,
                         minRadius=8, maxRadius=22)
    out = []
    if c is not None:
        for x, y, r in np.round(c[0]).astype(int):
            if x0 < x < x1 and y0 < y < y1:
                out.append({"u": int(x), "v": int(y), "r": int(r)})
    return out


def sample_depth(depth, u, v, r):
    """중심 주변 원판의 '유효 픽셀 중앙값' — 0(무효)은 제외, 표본수도 함께 반환."""
    rad = max(2, int(r * 0.6))
    ys, xs = np.ogrid[-rad:rad+1, -rad:rad+1]
    mask = xs*xs + ys*ys <= rad*rad
    patch = depth[v-rad:v+rad+1, u-rad:u+rad+1]
    vals = patch[mask & (patch > 0)]
    if vals.size == 0:
        return None, 0, None
    return float(np.median(vals)) * DEPTH_SCALE, int(vals.size), float(np.std(vals) * DEPTH_SCALE)


def main():
    img = cv2.imread(os.path.join(SNAPDIR, 'grip_color.jpg'))
    depth = cv2.imread(os.path.join(SNAPDIR, 'grip_depth.png'), cv2.IMREAD_UNCHANGED)
    meta = json.load(open(os.path.join(SNAPDIR, 'meta.json')))
    K = meta['cameras']['gripper']['color_info']['K']
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    tf = meta['tf']['base_link__grip_color']
    R = quat_to_R(tf['rotation'])
    t = np.array([tf['translation'][k] for k in ('x', 'y', 'z')])

    circles = detect_circles(img, ROI)
    circles.sort(key=lambda c: (c['v'], c['u']))
    print(f"[검출] ROI 내 원 {len(circles)}개")

    pts = []
    for i, c in enumerate(circles):
        z, n, sd = sample_depth(depth, c['u'], c['v'], c['r'])
        rec = dict(c, idx=i, depth_m=z, depth_n=n, depth_sd=sd)
        if z is not None:
            Xc = (c['u'] - cx) / fx * z
            Yc = (c['v'] - cy) / fy * z
            p_cam = np.array([Xc, Yc, z])
            p_base = R @ p_cam + t
            rec['cam_xyz'] = [round(v, 4) for v in p_cam]
            rec['base_xyz'] = [round(v, 4) for v in p_base]
        pts.append(rec)

    json.dump({"snapshot": os.path.basename(SNAPDIR), "roi": ROI,
               "depth_scale_assumed": DEPTH_SCALE, "buttons": pts},
              open('out/mapping_raw.json', 'w'), ensure_ascii=False, indent=1)

    print(f"{'i':>2} {'u':>4} {'v':>4} {'r':>3} {'depth(m)':>8} {'n':>4} {'sd':>6}   base_link XYZ(m)")
    for p in pts:
        b = p.get('base_xyz')
        print(f"{p['idx']:>2} {p['u']:>4} {p['v']:>4} {p['r']:>3} "
              f"{p['depth_m'] if p['depth_m'] else -1:>8.3f} {p['depth_n']:>4} "
              f"{p['depth_sd'] if p['depth_sd'] is not None else -1:>6.4f}   "
              f"{b if b else 'DEPTH 없음'}")

    vis = img.copy()
    cv2.rectangle(vis, ROI[:2], ROI[2:], (255, 0, 0), 1)
    for p in pts:
        cv2.circle(vis, (p['u'], p['v']), p['r'], (0, 255, 0), 2)
        cv2.putText(vis, str(p['idx']), (p['u']+p['r']+2, p['v']-2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.imwrite('out/detect_indexed.png', vis)
    cv2.imwrite('out/detect_zoom.png', cv2.resize(vis[130:300, 540:770], None, fx=4, fy=4,
                                                  interpolation=cv2.INTER_CUBIC))


main()
