"""VLM 판정 + 중복제거 + 3D + 격자 + 정답지 대조 + 스냅샷간 재현성"""
import cv2, numpy as np, json, os
DEPTH_SCALE = 0.001
V = json.load(open('vlm_verdicts.json'))
LAYOUT = json.load(open('../../elevator_button_press/button_layout.json'))['rows']

def quat_to_R(q):
    x,y,z,w=q['x'],q['y'],q['z'],q['w']
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def process(snap):
    v = V[snap]
    cand = json.load(open(f'out/cand_{snap}.json'))
    meta = json.load(open(os.path.join('..',snap,'meta.json')))
    depth = cv2.imread(os.path.join('..',snap,'grip_depth.png'), cv2.IMREAD_UNCHANGED)
    K = meta['cameras']['gripper']['color_info']['K']; fx,fy,cx,cy = K[0],K[4],K[2],K[5]
    tf = meta['tf']['base_link__grip_color']
    R = quat_to_R(tf['rotation']); t = np.array([tf['translation'][k] for k in 'xyz'])
    pts=[]
    for i in v['buttons']:
        c = cand[i]; u,vv,r = c['u'],c['v'],c['r']; rad=max(2,int(r*0.6))
        ys,xs=np.ogrid[-rad:rad+1,-rad:rad+1]; m=xs*xs+ys*ys<=rad*rad
        vals = depth[vv-rad:vv+rad+1,u-rad:u+rad+1][m & (depth[vv-rad:vv+rad+1,u-rad:u+rad+1]>0)]
        z=float(np.median(vals))*DEPTH_SCALE
        p=np.array([(u-cx)/fx*z,(vv-cy)/fy*z,z])
        pts.append({"cand":i,"u":u,"v":vv,"r":r,"depth_m":round(z,4),"n":int(vals.size),
                    "base":R@p+t})
    # 중복제거: 3D 20mm 이내 → 유효 depth 표본 많은 쪽
    keep=[]
    for p in sorted(pts, key=lambda p:-p['n']):
        if all(np.linalg.norm(p['base']-q['base'])>0.030 for q in keep): keep.append(p)   # 30mm: 동심원 중복 제거(행/열 간격 ~60mm보다 작음)
    keep.sort(key=lambda p:(p['v'],p['u']))
    return keep, v

def grid(keep):
    P=np.array([p['base'] for p in keep]); c=P.mean(0); n=np.linalg.svd(P-c)[2][2]
    if n[1]<0: n=-n
    zu=np.array([0.,0.,1.]); e2=zu-(zu@n)*n; e2/=np.linalg.norm(e2)
    e1=np.cross(e2,n); e1/=np.linalg.norm(e1)
    uv=np.stack([(P-c)@e1,(P-c)@e2],1)
    resid=(P-c)@n
    def cl(vals,tol=0.020):
        g=[]
        for i in np.argsort(vals):
            for grp in g:
                if abs(vals[i]-np.mean([vals[j] for j in grp]))<tol: grp.append(i); break
            else: g.append([i])
        return g
    rows=cl(-uv[:,1]); cols=cl(uv[:,0])
    G=[[None]*len(cols) for _ in rows]
    for r,g in enumerate(rows):
        for i in g:
            k=next(k for k,gg in enumerate(cols) if i in gg)
            if G[r][k] is not None: print(f"   ⚠ 격자 충돌: ({r},{k})에 2개 — 중복제거 실패 의심")
            G[r][k]=i
    return G,uv,resid,n

report={}
for snap in ["20260828T180736_snap","20260828T180746_snap","20260828T180326_snap","20260828T161806_test_first"]:
    keep,v = process(snap)
    print(f"\n══ {snap}  ({v['scene']})")
    print(f"   후보 {len(json.load(open(f'out/cand_{snap}.json')))} → VLM 버튼판정 {len(v['buttons'])} → 중복제거 후 {len(keep)}")
    if len(keep)<3:
        for p in keep: print(f"   단독버튼 px=({p['u']},{p['v']}) depth={p['depth_m']} base={np.round(p['base'],4)}")
        report[snap]={"n":len(keep),"buttons":[{"px":[p['u'],p['v']],"base":[round(float(x),4) for x in p['base']]} for p in keep]}
        continue
    G,uv,resid,n = grid(keep)
    print(f"   평면 잔차 rms {np.sqrt((resid**2).mean())*1000:.1f} mm / max {np.abs(resid).max()*1000:.1f} mm, 법선 {np.round(n,3)}")
    print(f"   격자 {len(G)}×{len(G[0])}:", [[('·' if x is None else '●') for x in r] for r in G])
    occ=[[x is not None for x in r] for r in G]; gt=[[bool(x) for x in r] for r in LAYOUT]
    match = occ==gt
    print(f"   정답지 채움패턴 대조: {'✅ 완전일치' if match else '❌ 불일치'}")
    labels={}
    if match:
        for r in range(len(G)):
            for cc in range(len(G[0])):
                if G[r][cc] is not None: labels[LAYOUT[r][cc]]=keep[G[r][cc]]
        for k,p in labels.items():
            print(f"     {k:>4}: base=[{p['base'][0]:+.4f} {p['base'][1]:+.4f} {p['base'][2]:+.4f}] px=({p['u']},{p['v']}) d={p['depth_m']}")
    report[snap]={"n":len(keep),"match":match,
                  "plane_rms_mm":round(float(np.sqrt((resid**2).mean())*1000),2),
                  "labels":{k:[round(float(x),4) for x in p['base']] for k,p in labels.items()},
                  "px":{k:[p['u'],p['v']] for k,p in labels.items()}}

a,b = report["20260828T180736_snap"], report["20260828T180746_snap"]
if a.get('labels') and b.get('labels'):
    print("\n══ 스냅샷 간 재현성 (180736 vs 180746, 10초 간격·로봇 정지)")
    print(f"   {'라벨':>5} {'Δx':>7} {'Δy':>7} {'Δz':>7} {'Δ거리(mm)':>10}")
    ds=[]
    for k in a['labels']:
        d=(np.array(b['labels'][k])-np.array(a['labels'][k]))*1000; ds.append(np.linalg.norm(d))
        print(f"   {k:>5} {d[0]:+7.1f} {d[1]:+7.1f} {d[2]:+7.1f} {np.linalg.norm(d):10.1f}")
    print(f"   → 평균 {np.mean(ds):.1f} mm, 최대 {np.max(ds):.1f} mm")
    report['repeatability_mm']={"mean":round(float(np.mean(ds)),2),"max":round(float(np.max(ds)),2)}
json.dump(report, open('out/final_report.json','w'), ensure_ascii=False, indent=1)
