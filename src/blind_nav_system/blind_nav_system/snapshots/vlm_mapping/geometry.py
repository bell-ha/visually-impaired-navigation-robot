"""기하 타당성 검사 + 격자 복원 + button_layout.json 대조"""
import json, numpy as np, itertools

d = json.load(open('out/mapping_raw.json'))
pts = [p for p in d['buttons'] if p.get('base_xyz')]
P = np.array([p['base_xyz'] for p in pts])
idx = [p['idx'] for p in pts]

# ── 1. 평면성 (SVD 최소 특이벡터 = 법선) ──
c = P.mean(0); U, S, Vt = np.linalg.svd(P - c)
n = Vt[2]; resid = (P - c) @ n
print("=== 평면성 ===")
print(f"법선(base_link) = [{n[0]:+.3f} {n[1]:+.3f} {n[2]:+.3f}]  (|y|≈1이면 로봇 좌우 벽면)")
print(f"평면 잔차: max {np.abs(resid).max()*1000:.1f} mm, rms {np.sqrt((resid**2).mean())*1000:.1f} mm")
print("특이값(정규화):", np.round(S/np.sqrt(len(P)),4), "→ 3번째가 훨씬 작아야 평면")

# ── 2. 패널 평면상 2D 좌표계 (u축=수평, v축=수직↑) ──
# 패널 평면 내 축을 '중력 기준'으로 고정: v축 = base_link z(위)를 평면에 투영, u축 = v × n
z_up = np.array([0.0, 0.0, 1.0])
e2 = z_up - (z_up @ n) * n; e2 /= np.linalg.norm(e2)      # 평면 내 '위'
e1 = np.cross(e2, n); e1 /= np.linalg.norm(e1)            # 평면 내 '가로'
uv = np.stack([(P - c) @ e1, (P - c) @ e2], 1)
print("\n=== 패널 평면 2D (mm) ===")
for i, (a, b) in zip(idx, uv * 1000): print(f"  #{i}: u={a:+7.1f}  v={b:+7.1f}")

# ── 3. 격자 복원: 행=v 클러스터, 열=u 클러스터 (버튼 지름의 절반 이내를 동일군) ──
def cluster(vals, tol):
    order = np.argsort(vals); groups = []
    for i in order:
        for g in groups:
            if abs(vals[i] - np.mean([vals[j] for j in g])) < tol: g.append(i); break
        else: groups.append([i])
    return groups

rows = cluster(-uv[:, 1], 0.020)          # 위→아래
cols = cluster(uv[:, 0], 0.020)           # 좌→우
row_of = {i: r for r, g in enumerate(rows) for i in g}
col_of = {i: c_ for c_, g in enumerate(cols) for i in g}
grid = [[None]*len(cols) for _ in rows]
for k in range(len(pts)): grid[row_of[k]][col_of[k]] = idx[k]
print(f"\n=== 격자 {len(rows)}행 × {len(cols)}열 (검출 인덱스) ===")
for r in grid: print("  ", [("·" if v is None else f"#{v}") for v in r])

# 간격
rv = sorted([np.mean([-uv[i,1] for i in g]) for g in rows])
cu = sorted([np.mean([uv[i,0] for i in g]) for g in cols])
print(f"행 간격(mm): {[round((b-a)*1000,1) for a,b in zip(rv,rv[1:])]}")
print(f"열 간격(mm): {[round((b-a)*1000,1) for a,b in zip(cu,cu[1:])]}")
diam = [2*p['r']/431.57*p['depth_m']*1000 for p in pts]
print(f"버튼 지름(mm, 검출 r 기준): {[round(x) for x in diam]}")

# ── 4. 정답지 대조 ──
gt = json.load(open('../../elevator_button_press/button_layout.json'))['rows']
print(f"\n=== 정답지(button_layout.json) 대조 ===")
print("  정답 격자:", gt)
occ_gt = [[bool(x) for x in r] for r in gt]
occ_det = [[v is not None for v in r] for r in grid]
print("  정답 채움패턴:", occ_gt)
print("  검출 채움패턴:", occ_det)
print("  패턴 일치:", "✅ 완전일치" if occ_gt == occ_det else "❌ 불일치")
if occ_gt == occ_det:
    label_map = {}
    for r in range(len(grid)):
        for c_ in range(len(grid[0])):
            if grid[r][c_] is not None: label_map[gt[r][c_]] = grid[r][c_]
    print("\n=== 라벨 → 3D 좌표 (base_link, m) ===")
    out = {}
    for lab, i in label_map.items():
        p = next(x for x in pts if x['idx'] == i)
        out[lab] = {"base_xyz": p['base_xyz'], "pixel": [p['u'], p['v']],
                    "depth_m": round(p['depth_m'], 4), "det_idx": i}
        print(f"  {lab:>4} : #{i}  base=[{p['base_xyz'][0]:+.4f} {p['base_xyz'][1]:+.4f} {p['base_xyz'][2]:+.4f}]  px=({p['u']},{p['v']})")
    json.dump({"snapshot": d['snapshot'], "source": "grid-structure match vs button_layout.json",
               "buttons": out}, open('out/button_map_3d.json', 'w'), ensure_ascii=False, indent=1)
