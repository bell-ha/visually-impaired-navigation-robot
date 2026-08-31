"""ROI 수기지정 없는 자동 파이프라인 — 원검출(전체) → 3D → 평면·근접 군집 → 격자 → 라벨"""
import cv2, numpy as np, json, os, sys
DEPTH_SCALE = 0.001

def quat_to_R(q):
    x,y,z,w = q['x'],q['y'],q['z'],q['w']
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def load(snap):
    d = os.path.join('..', snap)
    return (cv2.imread(os.path.join(d,'grip_color.jpg')),
            cv2.imread(os.path.join(d,'grip_depth.png'), cv2.IMREAD_UNCHANGED),
            json.load(open(os.path.join(d,'meta.json'))))

def backproject(circles, depth, K, R, t):
    fx,fy,cx,cy = K[0],K[4],K[2],K[5]; out=[]
    for c in circles:
        u,v,r = c['u'],c['v'],c['r']; rad=max(2,int(r*0.6))
        ys,xs = np.ogrid[-rad:rad+1,-rad:rad+1]; m = xs*xs+ys*ys<=rad*rad
        patch = depth[v-rad:v+rad+1, u-rad:u+rad+1]
        vals = patch[m & (patch>0)]
        if vals.size < 10: continue
        z = float(np.median(vals))*DEPTH_SCALE
        p_cam = np.array([(u-cx)/fx*z, (v-cy)/fy*z, z])
        out.append(dict(c, depth_m=z, depth_n=int(vals.size), base=R@p_cam+t))
    return out

def largest_planar_cluster(pts, box=0.30, plane_tol=0.008, min_n=5):
    """3D 근접 군집 → 평면 적합 → 인라이어 최대 집합 (RANSAC-lite, 전수조사)"""
    best = []
    P = np.array([p['base'] for p in pts])
    for i in range(len(pts)):
        near = [j for j in range(len(pts)) if np.linalg.norm(P[j]-P[i]) < box]
        if len(near) < min_n: continue
        Q = P[near]; c = Q.mean(0); n = np.linalg.svd(Q-c)[2][2]
        inl = [j for j in near if abs((P[j]-c)@n) < plane_tol]
        if len(inl) > len(best): best = inl
    return [pts[j] for j in best]

def grid_and_labels(cluster, layout_rows):
    P = np.array([p['base'] for p in cluster]); c = P.mean(0)
    n = np.linalg.svd(P-c)[2][2]
    z_up = np.array([0.,0.,1.]); e2 = z_up-(z_up@n)*n; e2/=np.linalg.norm(e2)
    e1 = np.cross(e2,n); e1/=np.linalg.norm(e1)
    if e1[0] > 0 and abs(e1[0]) > abs(e1[1]): e1 = -e1     # 부호 고정용(정보성)
    uv = np.stack([(P-c)@e1, (P-c)@e2], 1)
    def cluster1d(vals, tol=0.020):
        g=[]
        for i in np.argsort(vals):
            for grp in g:
                if abs(vals[i]-np.mean([vals[j] for j in grp]))<tol: grp.append(i); break
            else: g.append([i])
        return g
    rows = cluster1d(-uv[:,1]); cols = cluster1d(uv[:,0])
    grid = [[None]*len(cols) for _ in rows]
    for r,g in enumerate(rows):
        for i in g:
            cc = next(k for k,gg in enumerate(cols) if i in gg)
            grid[r][cc] = i
    occ = [[v is not None for v in r] for r in grid]
    gt  = [[bool(x) for x in r] for r in layout_rows]
    labels = {}
    if occ == gt:
        for r in range(len(grid)):
            for cc in range(len(grid[0])):
                if grid[r][cc] is not None: labels[layout_rows[r][cc]] = cluster[grid[r][cc]]
    return grid, occ, occ==gt, labels, uv

def run(snap, layout):
    img, depth, meta = load(snap)
    if img is None or depth is None: return None
    tfk = (meta.get('tf') or {}).get('base_link__grip_color')
    if tfk is None: print(f"  {snap}: TF 없음 → 건너뜀"); return None
    K = meta['cameras']['gripper']['color_info']['K']
    R = quat_to_R(tfk['rotation']); t = np.array([tfk['translation'][k] for k in 'xyz'])
    g = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),3)
    cc = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT,1,20,param1=100,param2=18,minRadius=8,maxRadius=22)
    circles = [{"u":int(x),"v":int(y),"r":int(r)} for x,y,r in np.round(cc[0]).astype(int)] if cc is not None else []
    pts = backproject(circles, depth, K, R, t)
    cl = largest_planar_cluster(pts)
    grid, occ, match, labels, uv = grid_and_labels(cl, layout) if len(cl)>=5 else (None,None,False,{},None)
    print(f"\n── {snap}")
    print(f"   원검출 {len(circles)} → depth유효 {len(pts)} → 평면군집 {len(cl)}")
    if grid: 
        print("   격자:", [[('·' if v is None else '●') for v in r] for r in grid], "정답패턴 일치:", "✅" if match else "❌")
    return {"snap": snap, "labels": {k:[round(float(x),4) for x in v['base']] for k,v in labels.items()},
            "px": {k:[v['u'],v['v']] for k,v in labels.items()}, "n_cluster": len(cl), "match": match}

layout = json.load(open('../../elevator_button_press/button_layout.json'))['rows']
snaps = [s for s in sorted(os.listdir('..')) if s.startswith('2026') and os.path.isdir(os.path.join('..',s))]
res = [r for r in (run(s, layout) for s in snaps) if r]
json.dump(res, open('out/cross_snapshot.json','w'), ensure_ascii=False, indent=1)

print("\n=== 스냅샷 간 재현성 (base_link, mm) — 8/28 세션은 로봇 미이동 가정 ===")
ok = [r for r in res if r['match']]
if len(ok) >= 2:
    keys = set.intersection(*[set(r['labels']) for r in ok])
    print(f"{'라벨':>6} " + " ".join(f"{r['snap'][9:15]:>22}" for r in ok) + "   최대편차")
    for k in sorted(keys):
        vs = np.array([ok_r['labels'][k] for ok_r in ok])
        spread = (vs.max(0)-vs.min(0))*1000
        print(f"{k:>6} " + " ".join("[%+.3f %+.3f %+.3f]" % tuple(v) for v in vs) +
              f"   {spread.max():5.1f} mm")
