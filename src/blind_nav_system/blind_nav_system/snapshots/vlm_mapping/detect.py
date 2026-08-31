"""버튼 원 검출 — VLM(의미) 보조용 정밀 중심 산출. 결과: out/circles_<snap>.json + 시각화"""
import cv2, numpy as np, json, sys, os
snap = sys.argv[1]
img = cv2.imread(os.path.join('..', snap, 'grip_color.jpg'))
g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
g = cv2.medianBlur(g, 3)
circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                           param1=100, param2=22, minRadius=8, maxRadius=22)
out = []
if circles is not None:
    for x, y, r in np.round(circles[0]).astype(int):
        out.append({"u": int(x), "v": int(y), "r": int(r)})
out.sort(key=lambda c: (c["v"], c["u"]))
vis = img.copy()
for i, c in enumerate(out):
    cv2.circle(vis, (c["u"], c["v"]), c["r"], (0, 255, 0), 2)
    cv2.circle(vis, (c["u"], c["v"]), 2, (0, 0, 255), 3)
    cv2.putText(vis, str(i), (c["u"] + c["r"] + 2, c["v"]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
os.makedirs('out', exist_ok=True)
json.dump(out, open(f'out/circles_{snap}.json', 'w'), indent=1)
cv2.imwrite(f'out/circles_{snap}.png', vis)
print(f"검출 {len(out)}개")
for i, c in enumerate(out): print(i, c)
