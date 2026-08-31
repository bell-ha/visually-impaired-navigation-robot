"""고감도 원검출 → 후보 타일 몽타주 (VLM이 '버튼/아님' 판정할 입력)"""
import cv2, numpy as np, json, os, sys
snap = sys.argv[1]; p2 = int(sys.argv[2]) if len(sys.argv)>2 else 14
img = cv2.imread(os.path.join('..',snap,'grip_color.jpg'))
depth = cv2.imread(os.path.join('..',snap,'grip_depth.png'), cv2.IMREAD_UNCHANGED)
g = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),3)
cc = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT,1,18,param1=100,param2=p2,minRadius=7,maxRadius=24)
cand=[]
if cc is not None:
    for x,y,r in np.round(cc[0]).astype(int):
        if not (r+9 <= x < img.shape[1]-r-9 and r+9 <= y < img.shape[0]-r-9): continue
        rad=max(2,int(r*0.6)); ys,xs=np.ogrid[-rad:rad+1,-rad:rad+1]; m=xs*xs+ys*ys<=rad*rad
        patch=depth[y-rad:y+rad+1, x-rad:x+rad+1]; vals=patch[m&(patch>0)]
        if vals.size<10: continue
        cand.append({"u":int(x),"v":int(y),"r":int(r),"depth_m":round(float(np.median(vals))*0.001,4)})
cand.sort(key=lambda c:(c['v'],c['u']))
tiles=[]
for i,c in enumerate(cand):
    pad=c['r']+9; t=img[c['v']-pad:c['v']+pad, c['u']-pad:c['u']+pad]
    z=cv2.resize(t,(150,150),interpolation=cv2.INTER_CUBIC)
    cv2.putText(z,str(i),(4,22),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)
    cv2.putText(z,f"{c['depth_m']:.2f}",(4,145),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,0),1)
    tiles.append(z)
W=8
rows=[np.hstack(tiles[i:i+W]) for i in range(0,len(tiles),W)]
w=max(r.shape[1] for r in rows)
rows=[np.pad(r,((0,0),(0,w-r.shape[1]),(0,0))) for r in rows]
cv2.imwrite(f'out/cand_{snap}.png', np.vstack(rows))
json.dump(cand, open(f'out/cand_{snap}.json','w'), indent=1)
print(snap, "후보", len(cand), "개")
