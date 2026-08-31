import cv2, numpy as np, sys, os
SNAP = sys.argv[1]; x0,y0,x1,y1 = map(int, sys.argv[2:6]); scale = float(sys.argv[6]); out = sys.argv[7]
img = cv2.imread(os.path.join('..', SNAP, 'grip_color.jpg'))
crop = img[y0:y1, x0:x1]
z = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
# 원본 픽셀 좌표 눈금 (25px 간격 = 원본 기준)
step = 25
for x in range(int(np.ceil(x0/step))*step, x1, step):
    X = int((x-x0)*scale)
    cv2.line(z,(X,0),(X,z.shape[0]),(0,255,0),1)
    cv2.putText(z,str(x),(X+2,14),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)
for y in range(int(np.ceil(y0/step))*step, y1, step):
    Y = int((y-y0)*scale)
    cv2.line(z,(0,Y),(z.shape[1],Y),(0,255,255),1)
    cv2.putText(z,str(y),(2,Y-3),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),1)
cv2.imwrite(out, z); print(out, z.shape)
