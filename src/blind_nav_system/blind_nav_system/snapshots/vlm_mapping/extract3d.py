#!/usr/bin/env python3
"""엘베 버튼 원 검출 → 4가지 깊이 추정 → 역투영 → base_link / map 3D 좌표.

목적: "그리퍼가 1~5층 버튼을 누르려면 좌표와 높이가 얼마인가"에 답할 실측치를 뽑는다.
      어느 깊이 추정이 맞는지는 이 스크립트가 정하지 않는다. 네 가지를 전부 내보내고,
      각각의 '유효율'과 '표준편차'를 같이 낸다. 못 믿을 값은 조용히 채우지 않는다.

입력(읽기 전용):  ../<스냅샷폴더>/grip_color.jpg, grip_depth.png, meta.json
출력(여기에만):   out3d/<폴더>.json, out3d/<폴더>_annot.png, out3d/<폴더>_tiles/cNN.png
사용법:           python3 extract3d.py [폴더명 ...]      (인자 없으면 스냅샷 폴더 전부)

※ 같은 폴더의 detect.py / geometry.py / map_buttons.py 는 삭제된 옛 스냅샷(8/28~8/31) 기준이라
   지금 입력과 맞지 않는다. 필요한 로직만 여기로 복사했고 그 파일들은 건드리지 않았다.

────────────────────────────────────────────────────────────────────────────
[사전지식 고지]  "오로지 사진만으로 판단 가능한가"를 나중에 판정할 수 있도록,
                이 스크립트에 들어간 '사진 밖에서 온 정보'를 전부 여기 적는다.

■ 읽지 않는 것 (의도적)
  · button_layout.json(수기 정답지)을 읽지 않는다. 층 배치·버튼 개수·행/열 수를
    코드에 박지 않았다. 이 스크립트는 "원이 몇 개 있고 각각 어디에 있다"까지만 말한다.
    어느 원이 몇 층인지는 사람이 타일 이미지를 보고 붙인다.

■ 사진 밖에서 온 정보 = 3가지뿐, 전부 아래에 나열
  (1) 카메라 내부/외부 파라미터(K, D, TF): meta.json 에서 읽는다. 캘리브레이션 값이지
      "이 엘리베이터에 대한 지식"은 아니다.
  (2) 깊이 규약: uint16, 단위 mm, 0과 65535는 무효. 센서 규약.
  (3) 버튼의 '물리 크기 범위'와 '최소 간격' ↓ — 통념이다. 이게 가장 논쟁적이다.
      (픽셀 상수가 아니라 mm 상수라는 점이 중요하다. 아래 참조)

■ Hough 파라미터를 어떻게 잡았나 (숨기지 않고 전부 기록)

  [1차 시도 — 픽셀 반경을 상수로 박았다. 폐기함]
  옛 detect.py 값: dp=1 minDist=20 param1=100 param2=22 minRadius=8 maxRadius=22
  그대로 쓰면 이번 사진에서 버튼 대신 나사·반사점을 잡는다(실측: r=11~21px 만 검출).
  원인은 촬영 거리다. 8월 스냅샷은 멀리서, 9/2 수집분은 패널까지 19~20 cm 로 붙어 찍었다.
  그래서 r=20~60px, minDist=40px, param2=30 으로 바꿔 잡았고, 시도한 5가지는 이랬다
  (3개 폴더에서 확인):
      old(detect.py) r=8~22,   param2=22  → 4~6개, 전부 나사·반사점 크기. 버튼 놓침.
      wide-r         r=20~60,  param2=30  → 5~6개. (1차 채택)
      wide-r-loose   r=20~60,  param2=22  → 8~9개. 오검출 증가(패널 얼룩)
      wide-r-tight   r=20~60,  param1=120 param2=40 → 5개. 가려진 버튼을 더 놓침
      dp2-wide       dp=2,     param2=60  → 6~12개. 같은 버튼을 반경만 다르게 중복 검출
  1차 채택값으로 24폴더를 돌린 결과 원 100개를 얻었지만, **0.32~0.44 m 에서 찍은 7개
  폴더가 통째로 비거나 1개만 나왔다.** 거리가 멀어 버튼 반경이 18px 로 내려가
  minRadius=20 바로 밑에 깔린 것이다. 파라미터가 특정 촬영 거리에 붙어 있었다.

  [2차 = 현재 — 픽셀 상수를 버리고 깊이에서 매 프레임 역산한다]
  RGB-D 로 Z 를 아는데 픽셀 반경을 상수로 박을 이유가 없다. 그래서:
      radius_px = D_mm * fx / (2000 * Z_m)      (D_mm = 버튼 바깥 링 지름)
      minDist_px = PITCH_mm * fx / (1000 * Z_m) (PITCH_mm = 버튼 중심 간 최소 간격)
  이렇게 하면 튜닝 상수가 '픽셀 20~60'(이 카메라·이 거리 전용 지식)에서
  '버튼 지름 15~55 mm'(일반 승강기에 대한 진술)로 옮겨간다. 사전지식이 사라지는 게
  아니라 **일반화 가능한 형태로 바뀐다.** 이 시스템이 RGB-D 를 쓰는 이점이 여기다.

  ⚠ 자진신고 — 여전히 통념이고 사진에서 잰 값이 아니다:
      · BUTTON_DIAM_MIN/MAX = 15~55 mm.  advisor 는 독립적으로 15~40 mm 를 제안했다.
        상한이 다르다. 내가 55 를 쓰는 근거는 1차 실행에서 실제로 검출된 원의 물리
        지름 중앙값이 30.4 mm, 최대 38.2 mm 였다는 실측이다. 40 은 그 최대값에
        너무 붙어 있어 조금만 크거나 원이 바깥 링을 물면 잘려 나간다. 55 는 여유다.
        하한 15 는 두 안이 같다.
      · BUTTON_PITCH_MIN = 40 mm. "버튼 중심이 4cm 보다 가깝게 붙어 있진 않다"는 통념.
        1차의 minDist=40px 를 그대로 mm 로 옮긴 게 아니라, 같은 조건(Z=0.19m, fx=431.6)
        에서 40px ≈ 18mm 였으므로 실제로는 **더 넉넉하게(=중복검출에 더 엄격하게)** 잡았다.
      · param1=100 / param2=30 / medianBlur=5 는 여전히 픽셀·밝기 영역 상수다.
        이것들은 거리에 따라 변하는 양이 아니라서 역산 대상이 아니지만, 손으로 고른
        값인 것은 맞다. 숨기지 않는다.

  Z 는 그 사진의 '전체 유효 깊이 중앙값'을 쓴다. 유효율이 낮으면 이 값이 망가지므로
  Z_REF_MIN_VALID 미만이면 결과를 버리지 않고 **저신뢰 플래그를 달아 그대로 내보낸다**
  (조용한 기본값 대체를 하지 않는다). 실측 24폴더의 유효율은 0.648~0.869 로 전부
  문턱 위였다 — 이 분기는 이번 데이터에서 한 번도 발동하지 않았다.

■ 신뢰도 원자료에 대하여
  cv2.HoughCircles 는 이 빌드(OpenCV 4.12)에서 (x, y, r) 3열만 돌려준다. 누산기 표값을
  주지 않는다. HOUGH_GRADIENT_ALT 도 3열이었다. 그래서 votes 는 null 로 나간다.
  대신 사진에서 직접 잰 `edge_support`(원 둘레 중 Canny 에지에 걸친 비율)를 넣었다.
  이건 cv2가 준 값이 아니라 이 스크립트가 계산한 값이다 — JSON에도 그렇게 표기한다.
────────────────────────────────────────────────────────────────────────────
"""
import json
import os
import sys

import cv2
import numpy as np

# ── 경로 ────────────────────────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
SNAP_ROOT = os.path.dirname(HERE)          # .../snapshots  (읽기 전용)
OUT_DIR = os.path.join(HERE, 'out3d')      # 쓰기는 오직 여기

# ── 깊이 규약 ───────────────────────────────────────────────────────────────
DEPTH_SCALE = 0.001        # grip_depth.png 는 uint16, 단위 mm → m
DEPTH_ZERO = 0             # 무효(측정 실패)
DEPTH_SAT = 65535          # 무효(포화/오버플로) — 실제 데이터에 존재함(≈0.03%)

# ── HoughCircles 파라미터 (근거는 상단 [사전지식 고지] 참조) ────────────────
# 거리 의존 파라미터(반경·최소간격)는 상수로 박지 않고 mm → px 로 매 프레임 역산한다.
BUTTON_DIAM_MIN_MM = 15.0  # 버튼 바깥 링 지름 하한 (통념)
BUTTON_DIAM_MAX_MM = 55.0  # 상한. 실측 최대 38.2 mm 에 여유를 준 값
BUTTON_PITCH_MIN_MM = 40.0 # 버튼 중심 간 최소 간격 (통념) → minDist
Z_REF_MIN_VALID = 0.50     # 이 미만이면 Z 기준값을 저신뢰로 표시(버리지는 않는다)

# 거리와 무관한 파라미터(밝기·누산기 영역) — 손으로 고른 값
BLUR_KSIZE = 5             # medianBlur — 스테인리스 패널의 반사 얼룩 억제
HOUGH_DP = 1               # 누산기 해상도 = 입력과 동일
HOUGH_PARAM1 = 100         # 내부 Canny 상단 임계값(하단은 절반)
HOUGH_PARAM2 = 30          # 누산기 임계값. 낮추면 오검출↑, 높이면 미검출↑

# ── 깊이 표본 영역 ──────────────────────────────────────────────────────────
R3_RADIUS = 3              # z_med_r3: 중심 반경 3 px 원판
HALF_SCALE = 0.5           # z_med_half: 반경 r/2 원판
RING_IN_SCALE = 1.0        # z_med_ring: 반경 r ~ 1.5r 고리 = 버튼 바깥 패널면
RING_OUT_SCALE = 1.5

DEPTH_VARIANTS = ('z_center', 'z_med_r3', 'z_med_half', 'z_med_ring')

# ── 가림(occlusion) 판정 ────────────────────────────────────────────────────
# 그리퍼 손가락(검은 패드 + 은색 지지대)이 하단 버튼을 가리면 그 영역의 깊이가
# '손가락 거리'와 '버튼 거리'로 갈려 표준편차가 튄다. 가려지지 않은 버튼은
# σ 0.5~1.4 mm 수준이므로 10 mm 는 넉넉한 문턱이다.
SIGMA_HIGH_MM = 10.0
# 원 단위 플래그는 '4개 변형 중 하나라도' σ 초과면 켠다(놓치기보다 과탐지 쪽으로).
# z_center 는 1픽셀이라 σ=0 이므로 실질적으로 r3/half/ring 이 판정한다.

# ── 에지 지지도 (검출 신뢰도 대용 — cv2가 아니라 이 스크립트가 계산) ────────
EDGE_SAMPLES = 180         # 원 둘레를 2도 간격으로 샘플
EDGE_TOL_PX = 2            # 에지맵을 이만큼 팽창시켜 ±2px 오차 허용

# ── 주석 이미지 / 타일 ──────────────────────────────────────────────────────
FONT = cv2.FONT_HERSHEY_SIMPLEX
LABEL_SCALE = 1.1          # 눈으로 보고 라벨을 붙일 거라 크게
LABEL_OUTLINE = 6          # 검은 외곽선 두께(대비 확보)
LABEL_THICK = 2
TILE_PAD_SCALE = 2.0       # 타일 반폭 = 원 반경의 2배 (여유 있게)
TILE_ZOOM = 4              # 4배 확대


def quat_to_R(q):
    """쿼터니언 → 3x3 회전행렬.

    meta.json 의 rotation 은 (x, y, z, w) 순서다.  ← 순서 명시(요구사항 4)
    dict 키로 꺼내므로 순서 착오가 생길 여지는 없지만, 값 배열로 바꿀 일이 있으면
    반드시 x, y, z, w 임을 기억할 것. (ROS geometry_msgs/Quaternion 과 동일)
    """
    x, y, z, w = float(q['x']), float(q['y']), float(q['z']), float(q['w'])
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        raise ValueError('쿼터니언 노름이 0이다')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def tf_of(meta, key):
    """meta['tf'][key] → (R, t). 없으면 (None, None)."""
    tf = meta.get('tf', {}).get(key)
    if not tf:
        return None, None
    t = tf['translation']
    return quat_to_R(tf['rotation']), np.array([float(t['x']), float(t['y']), float(t['z'])])


def apply_tf(R, t, p):
    """p(3,) 를 R @ p + t 로 변환. R 이 없으면 None."""
    if R is None or p is None:
        return None
    return R @ np.asarray(p, dtype=float) + t


def region_stats(depth, mask):
    """마스크 영역의 깊이 통계.

    유효 = 0도 65535도 아닌 픽셀. n_total 은 '이미지 안에 실제로 있는' 픽셀 수라
    원이 화면 밖으로 걸치면 이상적 면적보다 작아진다(그대로 노출한다).
    유효 픽셀이 하나도 없으면 z 는 None — 임의로 채우지 않는다.
    """
    patch = depth[mask]
    n_total = int(patch.size)
    valid = patch[(patch != DEPTH_ZERO) & (patch != DEPTH_SAT)]
    n_valid = int(valid.size)
    out = {
        'n_total': n_total,
        'n_valid': n_valid,
        'valid_ratio': round(n_valid / n_total, 4) if n_total else 0.0,
        'median_mm': None,
        'std_mm': None,
        'z_m': None,
    }
    if n_valid:
        v = valid.astype(np.float64)
        out['median_mm'] = round(float(np.median(v)), 1)
        out['std_mm'] = round(float(np.std(v)), 1)
        out['z_m'] = round(out['median_mm'] * DEPTH_SCALE, 5)
    return out


def depth_regions(depth, u, v, r, yy, xx):
    """요구된 네 가지 깊이 추정을 전부 계산해 dict 로 돌려준다."""
    d2 = (xx - u) ** 2 + (yy - v) ** 2       # 중심까지 거리의 제곱 (px^2)
    res = {}

    # z_center: 중심 1픽셀 — 마스크 대신 직접 읽어 n_total=1 을 보장
    h, w = depth.shape
    if 0 <= v < h and 0 <= u < w:
        px = int(depth[v, u])
        ok = px not in (DEPTH_ZERO, DEPTH_SAT)
        res['z_center'] = {
            'n_total': 1, 'n_valid': int(ok), 'valid_ratio': 1.0 if ok else 0.0,
            'median_mm': float(px) if ok else None,
            'std_mm': 0.0 if ok else None,
            'z_m': round(px * DEPTH_SCALE, 5) if ok else None,
            'raw_mm': px,   # 무효여도 원값을 남긴다(0인지 65535인지 구분용)
        }
    else:
        res['z_center'] = {'n_total': 0, 'n_valid': 0, 'valid_ratio': 0.0,
                           'median_mm': None, 'std_mm': None, 'z_m': None, 'raw_mm': None}

    res['z_med_r3'] = region_stats(depth, d2 <= R3_RADIUS ** 2)
    r_half = max(1.0, r * HALF_SCALE)
    res['z_med_half'] = region_stats(depth, d2 <= r_half ** 2)
    r_in, r_out = r * RING_IN_SCALE, r * RING_OUT_SCALE
    res['z_med_ring'] = region_stats(depth, (d2 > r_in ** 2) & (d2 <= r_out ** 2))

    for k in DEPTH_VARIANTS:
        res[k]['radius_px'] = {'z_center': 0.0, 'z_med_r3': float(R3_RADIUS),
                               'z_med_half': round(r_half, 1),
                               'z_med_ring': [round(r_in, 1), round(r_out, 1)]}[k]
        sd = res[k]['std_mm']
        # σ 문턱 초과 = 한 영역 안에 서로 다른 거리의 물체가 섞였다 = 가림 의심
        res[k]['sigma_high'] = (sd is not None and sd > SIGMA_HIGH_MM)
    return res


def edge_support(edge_dil, u, v, r):
    """원 둘레 중 Canny 에지에 걸친 표본 비율(0~1). 이 스크립트가 계산한 값이다.

    cv2.HoughCircles 가 누산기 표값을 돌려주지 않아서(3열 출력) 넣은 대체 지표다.
    낮으면 '둘레가 실제로 보이지 않는 원' = 오검출이거나 가려진 원이다.
    """
    h, w = edge_dil.shape
    th = np.linspace(0.0, 2.0 * np.pi, EDGE_SAMPLES, endpoint=False)
    xs = np.round(u + r * np.cos(th)).astype(int)
    ys = np.round(v + r * np.sin(th)).astype(int)
    inside = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    if not inside.any():
        return 0.0, 0
    hit = edge_dil[ys[inside], xs[inside]] > 0
    return round(float(hit.mean()), 4), int(inside.sum())


def z_reference(depth):
    """이 사진의 기준 거리 Z(m)와 그 근거가 된 유효 픽셀 비율.

    화면 전체의 유효 깊이 중앙값을 쓴다. 근접 촬영에서는 패널이 화면 대부분을
    차지하므로 이게 곧 패널까지의 거리다. 유효 픽셀이 하나도 없으면 (None, 0.0).
    """
    ok = (depth != DEPTH_ZERO) & (depth != DEPTH_SAT)
    ratio = float(ok.mean()) if depth.size else 0.0
    if not ok.any():
        return None, ratio
    return float(np.median(depth[ok])) * DEPTH_SCALE, ratio


def hough_geom(fx, z_ref):
    """물리 치수(mm) → 이 거리에서의 픽셀 파라미터.

        radius_px  = D_mm * fx / (2000 * Z_m)
        minDist_px = PITCH_mm * fx / (1000 * Z_m)

    Z 를 모르면(유효 깊이 0) None 을 돌려준다 — 임의의 픽셀 기본값으로 때우지 않는다.
    """
    if not z_ref or z_ref <= 0.0:
        return None
    r_min = BUTTON_DIAM_MIN_MM * fx / (2000.0 * z_ref)
    r_max = BUTTON_DIAM_MAX_MM * fx / (2000.0 * z_ref)
    md = BUTTON_PITCH_MIN_MM * fx / (1000.0 * z_ref)
    return {
        'minRadius': max(1, int(round(r_min))),
        'maxRadius': max(2, int(round(r_max))),
        'minDist': max(1.0, round(md, 1)),
        'px_per_mm': round(fx / (1000.0 * z_ref), 4),
    }


def detect_circles(gray, geom):
    """HoughCircles 로 원 목록. 위→아래, 같은 높이면 좌→우 순으로 정렬.

    반경·최소간격은 geom(깊이에서 역산한 값)을 쓴다. geom 이 None 이면 검출을 하지 않는다
    — 픽셀 기본값으로 대신 돌리면 그게 곧 숨은 사전지식이 되기 때문이다.

    반환: (circles, ncols)  — ncols 는 cv2 가 돌려준 열 수.
    OpenCV 가 4열(x, y, r, votes)을 주는 빌드면 votes 를 그대로 싣는다. 이 빌드는 3열이라
    votes=None 이 나간다(상단 [사전지식 고지] 참조). 지어내지 않는다.
    """
    if geom is None:
        return [], 0
    found = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=HOUGH_DP, minDist=geom['minDist'],
        param1=HOUGH_PARAM1, param2=HOUGH_PARAM2,
        minRadius=geom['minRadius'], maxRadius=geom['maxRadius'])
    if found is None:
        return [], 0
    arr = found[0]
    ncols = int(arr.shape[1])
    circles = []
    for row in arr:
        circles.append({'u': int(round(float(row[0]))), 'v': int(round(float(row[1]))),
                        'r': int(round(float(row[2]))),
                        'hough_votes': (float(row[3]) if ncols >= 4 else None)})
    circles.sort(key=lambda c: (c['v'], c['u']))
    return circles, ncols


def save_tile(vis_free_img, snap, idx, u, v, r):
    """원 주변을 잘라 4배 확대한 개별 타일 PNG. 사람이 층 숫자를 읽기 위한 것."""
    h, w = vis_free_img.shape[:2]
    pad = max(8, int(round(r * TILE_PAD_SCALE)))
    x0, x1 = max(0, u - pad), min(w, u + pad + 1)
    y0, y1 = max(0, v - pad), min(h, v + pad + 1)
    crop = vis_free_img[y0:y1, x0:x1]
    if crop.size == 0:
        return None
    big = cv2.resize(crop, None, fx=TILE_ZOOM, fy=TILE_ZOOM, interpolation=cv2.INTER_CUBIC)
    tdir = os.path.join(OUT_DIR, snap + '_tiles')
    assert os.path.abspath(tdir).startswith(os.path.abspath(OUT_DIR) + os.sep), tdir
    os.makedirs(tdir, exist_ok=True)
    path = os.path.join(tdir, f'c{idx:02d}.png')
    cv2.imwrite(path, big)
    return os.path.relpath(path, OUT_DIR)


def process(snap):
    """스냅샷 폴더 하나 처리. 결과 dict 를 돌려주고 출력 파일을 쓴다."""
    sdir = os.path.join(SNAP_ROOT, snap)
    img = cv2.imread(os.path.join(sdir, 'grip_color.jpg'))
    depth = cv2.imread(os.path.join(sdir, 'grip_depth.png'), cv2.IMREAD_UNCHANGED)
    with open(os.path.join(sdir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    if img is None or depth is None:
        raise IOError('grip_color.jpg 또는 grip_depth.png 를 읽지 못했다')
    if depth.shape[:2] != img.shape[:2]:
        raise ValueError(f'color {img.shape[:2]} 와 depth {depth.shape[:2]} 크기가 다르다')

    cam = meta['cameras']['gripper']['color_info']
    K = cam['K']
    fx, cx, fy, cy = float(K[0]), float(K[2]), float(K[4]), float(K[5])
    D = np.array(cam.get('D') or [0.0] * 5, dtype=np.float64)
    Kmat = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    R_bc, t_bc = tf_of(meta, 'base_link__grip_color')   # grip_color → base_link
    R_mb, t_mb = tf_of(meta, 'map__base_link')          # base_link  → map

    h, w = depth.shape
    yy, xx = np.ogrid[0:h, 0:w]

    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), BLUR_KSIZE)
    # Hough 내부와 같은 임계값으로 에지맵을 따로 만든다(에지 지지도 계산용)
    edges = cv2.Canny(gray, HOUGH_PARAM1 // 2, HOUGH_PARAM1)
    k = 2 * EDGE_TOL_PX + 1
    edge_dil = cv2.dilate(edges, np.ones((k, k), np.uint8))

    z_ref, z_valid = z_reference(depth)
    geom = hough_geom(fx, z_ref)
    z_low_conf = (z_valid < Z_REF_MIN_VALID)   # 낮아도 버리지 않는다. 표시만 한다.

    circles, ncols = detect_circles(gray, geom)
    recs = []
    for i, c in enumerate(circles):
        u, v, r = c['u'], c['v'], c['r']

        # 왜곡 보정: 픽셀 → 정규화 좌표. P 를 주지 않으면 정규화 좌표(x', y')가 나온다.
        und = cv2.undistortPoints(np.array([[[float(u), float(v)]]], dtype=np.float64), Kmat, D)
        xn_u, yn_u = float(und[0, 0, 0]), float(und[0, 0, 1])
        # 보정 안 한 정규화 좌표(핀홀 그대로)
        xn_r, yn_r = (u - cx) / fx, (v - cy) / fy

        es, es_n = edge_support(edge_dil, u, v, r)
        dep = depth_regions(depth, u, v, r, yy, xx)
        rec = {'idx': i, 'u': u, 'v': v, 'r': r,
               'detection': {
                   'hough_votes': c['hough_votes'],
                   'hough_output_cols': ncols,
                   'hough_votes_note': ('OpenCV가 votes 열을 주지 않는다(3열 출력) — null'
                                        if ncols < 4 else 'cv2 제공 누산기 값'),
                   'edge_support': es,              # 0~1, 이 스크립트가 계산
                   'edge_support_samples': es_n,
                   'edge_support_note': 'cv2 제공값 아님. 원 둘레 중 Canny 에지에 걸친 비율',
               },
               'norm_raw': [round(xn_r, 6), round(yn_r, 6)],
               'norm_undistorted': [round(xn_u, 6), round(yn_u, 6)],
               'depth': dep}

        for name in DEPTH_VARIANTS:
            z = dep[name]['z_m']
            if z is None:
                dep[name].update(cam_xyz=None, cam_xyz_undist=None, undist_diff_mm=None,
                                 base_xyz=None, base_xyz_undist=None,
                                 map_xyz=None, map_xyz_undist=None, diameter_mm=None)
                continue
            p_raw = np.array([xn_r * z, yn_r * z, z])     # X=(u-cx)Z/fx, Y=(v-cy)Z/fy, Z
            p_und = np.array([xn_u * z, yn_u * z, z])
            b_raw = apply_tf(R_bc, t_bc, p_raw)
            b_und = apply_tf(R_bc, t_bc, p_und)
            m_raw = apply_tf(R_mb, t_mb, b_raw)
            m_und = apply_tf(R_mb, t_mb, b_und)
            rnd = lambda p: None if p is None else [round(float(x), 5) for x in p]
            dep[name].update(
                cam_xyz=rnd(p_raw), cam_xyz_undist=rnd(p_und),
                undist_diff_mm=round(float(np.linalg.norm(p_und - p_raw)) * 1000.0, 2),
                base_xyz=rnd(b_raw), base_xyz_undist=rnd(b_und),
                map_xyz=rnd(m_raw), map_xyz_undist=rnd(m_und),
                # 검출 반경 r 이 이 깊이에서 갖는 물리 지름 = 2*r*Z/fx
                diameter_mm=round(2.0 * r * z / fx * 1000.0, 1))
        # 원 단위 가림 의심: 4개 변형 중 하나라도 σ > 문턱
        rec['sigma_high_variants'] = [k2 for k2 in DEPTH_VARIANTS if dep[k2]['sigma_high']]
        rec['occlusion_suspect'] = bool(rec['sigma_high_variants'])
        rec['tile'] = save_tile(img, snap, i, u, v, r)
        recs.append(rec)

    out = {
        'snapshot': snap,
        'generated_by': 'extract3d.py',
        'dash_time': meta.get('dash_time'),
        'wall_time': meta.get('wall_time'),
        'floor': meta.get('floor'),
        'image': {'w': int(w), 'h': int(h)},
        'depth_scale_m_per_unit': DEPTH_SCALE,
        'depth_invalid_values': [DEPTH_ZERO, DEPTH_SAT],
        'prior_knowledge_note': ('button_layout.json 을 읽지 않았다. 층 배치·버튼 개수·'
                                 '행열 수를 코드에 박지 않았다. 사진 밖에서 온 정보는 '
                                 'K/D/TF, 깊이 규약, 그리고 버튼 물리 치수 통념'
                                 '(지름 15~55mm, 최소 간격 40mm)뿐이다. 픽셀 반경 상수는 '
                                 '쓰지 않는다 — 깊이에서 매 프레임 역산한다.'),
        'z_reference': {
            'z_m': None if z_ref is None else round(z_ref, 4),
            'source': '화면 전체 유효 깊이의 중앙값',
            'valid_ratio': round(z_valid, 4),
            'min_valid_threshold': Z_REF_MIN_VALID,
            'low_confidence': bool(z_low_conf),
            'note': ('유효율이 문턱 미만이다. Z 역산이 흔들릴 수 있어 표시만 하고 '
                     '결과는 그대로 낸다(기본값 대체 없음).') if z_low_conf else None,
        },
        'hough': {'dp': HOUGH_DP, 'param1': HOUGH_PARAM1, 'param2': HOUGH_PARAM2,
                  'medianBlur': BLUR_KSIZE, 'output_cols': ncols,
                  # 거리에서 역산한 값 (px 상수 아님)
                  'derived_from_depth': geom,
                  'button_diam_mm': [BUTTON_DIAM_MIN_MM, BUTTON_DIAM_MAX_MM],
                  'button_pitch_min_mm': BUTTON_PITCH_MIN_MM},
        'sigma_high_mm': SIGMA_HIGH_MM,
        'intrinsics': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'D': list(D),
                       'distortion_model': cam.get('distortion_model')},
        'tf': {'base_link__grip_color': meta.get('tf', {}).get('base_link__grip_color'),
               'map__base_link': meta.get('tf', {}).get('map__base_link')},
        'quaternion_order': 'xyzw',
        'joint_states': meta.get('joint_states'),
        'amcl_pose': meta.get('amcl_pose'),
        'localization_available': meta.get('localization_available'),
        'n_circles': len(recs),
        'circles': recs,
    }

    jpath = os.path.join(OUT_DIR, snap + '.json')
    ppath = os.path.join(OUT_DIR, snap + '_annot.png')
    for p in (jpath, ppath):     # 출력이 OUT_DIR 밖으로 나가는 일이 없도록 잠근다
        assert os.path.abspath(p).startswith(os.path.abspath(OUT_DIR) + os.sep), p
    with open(jpath, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)

    vis = img.copy()
    for rec in recs:
        u, v, r = rec['u'], rec['v'], rec['r']
        col = (0, 128, 255) if rec['occlusion_suspect'] else (0, 255, 0)  # 가림 의심=주황
        cv2.circle(vis, (u, v), r, col, 2)
        cv2.circle(vis, (u, v), 3, (0, 0, 255), 4)
        txt, org = str(rec['idx']), (u + r + 4, v + 8)
        cv2.putText(vis, txt, org, FONT, LABEL_SCALE, (0, 0, 0), LABEL_OUTLINE)
        cv2.putText(vis, txt, org, FONT, LABEL_SCALE, (0, 255, 255), LABEL_THICK)
    # cv2.putText 는 한글을 못 그린다(물음표로 나온다). 폴더명 중 ASCII 부분만 얹는다.
    # 전체 폴더명은 JSON 의 snapshot 필드와 파일명에 그대로 남는다.
    head = (f"{snap.split('_')[0]}  circles={len(recs)}  "
            f'(orange=sigma>{SIGMA_HIGH_MM:.0f}mm)')
    cv2.putText(vis, head, (8, 24), FONT, 0.6, (0, 0, 0), 4)
    cv2.putText(vis, head, (8, 24), FONT, 0.6, (255, 255, 255), 1)
    cv2.imwrite(ppath, vis)
    return out


def print_summary(out):
    """폴더별 요약표 — 원 × 깊이추정 4종을 한 줄씩."""
    js = out.get('joint_states') or {}
    zr = out['z_reference']; g = out['hough']['derived_from_depth']
    print(f"\n=== {out['snapshot']}  ({out.get('dash_time')})  원 {out['n_circles']}개 ===")
    print(f"    lift={js.get('joint_lift')}  wrist_extension={js.get('wrist_extension')}  "
          f"floor={out.get('floor')}  localized={out.get('localization_available')}")
    if g is None:
        print(f"    Z기준=없음(유효깊이 0) → 역산 불가라 검출 안 함")
    else:
        print(f"    Z기준={zr['z_m']}m (유효 {zr['valid_ratio']*100:.1f}%"
              f"{' ⚠저신뢰' if zr['low_confidence'] else ''}) → "
              f"r={g['minRadius']}~{g['maxRadius']}px minDist={g['minDist']}px "
              f"({g['px_per_mm']:.2f} px/mm)")
    if not out['n_circles']:
        print('    ⚠ 검출 0개')
        return
    print(f"    {'i':>2} {'u':>4} {'v':>4} {'r':>3} {'ES':>5}  {'변형':<10} {'z(mm)':>8} "
          f"{'유효%':>6} {'sd':>6} {'지름mm':>7}  {'base X':>8} {'base Y':>8} {'base Z':>8} "
          f"{'왜곡mm':>7} 가림")
    for c in out['circles']:
        es = c['detection']['edge_support']
        for k in DEPTH_VARIANTS:
            d = c['depth'][k]
            flag = '⚠' if d['sigma_high'] else ' '
            if d['z_m'] is None:
                print(f"    {c['idx']:>2} {c['u']:>4} {c['v']:>4} {c['r']:>3} {es:>5.2f}  "
                      f"{k:<10} {'--':>8} {d['valid_ratio']*100:>5.1f}% "
                      f"{'--':>6} {'--':>7}  {'유효 깊이 없음':>26} {flag}")
                continue
            b = d['base_xyz'] or [float('nan')] * 3
            print(f"    {c['idx']:>2} {c['u']:>4} {c['v']:>4} {c['r']:>3} {es:>5.2f}  "
                  f"{k:<10} {d['median_mm']:>8.1f} {d['valid_ratio']*100:>5.1f}% "
                  f"{d['std_mm']:>6.1f} {d['diameter_mm']:>7.1f}  "
                  f"{b[0]:>8.4f} {b[1]:>8.4f} {b[2]:>8.4f} {d['undist_diff_mm']:>7.2f} {flag}")
    sus = [c['idx'] for c in out['circles'] if c['occlusion_suspect']]
    print(f"    가림 의심(σ>{SIGMA_HIGH_MM:.0f}mm) 원: {sus if sus else '없음'}")


def all_snapshots():
    return sorted(d for d in os.listdir(SNAP_ROOT)
                  if os.path.isfile(os.path.join(SNAP_ROOT, d, 'meta.json')))


def main():
    snaps = sys.argv[1:] or all_snapshots()
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"스냅샷 {len(snaps)}개  |  입력 {SNAP_ROOT} (읽기 전용)  |  출력 {OUT_DIR}")
    print(f"Hough: dp={HOUGH_DP} param1={HOUGH_PARAM1} param2={HOUGH_PARAM2} "
          f"blur={BLUR_KSIZE} | 반경·minDist는 깊이에서 역산")
    print(f"물리 상수: 버튼 지름 {BUTTON_DIAM_MIN_MM:.0f}~{BUTTON_DIAM_MAX_MM:.0f}mm, "
          f"최소 간격 {BUTTON_PITCH_MIN_MM:.0f}mm (픽셀 반경 상수 없음)")
    print('정답지(button_layout.json) 미사용 — 사진 + 카메라 파라미터만 사용')
    zero, failed, tot, occ, lowz = [], [], 0, 0, []
    for s in snaps:
        try:
            out = process(s)
        except Exception as e:                      # 한 폴더가 죽어도 나머지는 계속
            failed.append((s, f'{type(e).__name__}: {e}'))
            print(f"\n=== {s} ===\n    ❌ 처리 실패: {type(e).__name__}: {e}")
            continue
        print_summary(out)
        tot += out['n_circles']
        occ += sum(1 for c in out['circles'] if c['occlusion_suspect'])
        if out['z_reference']['low_confidence']:
            lowz.append((s, out['z_reference']['valid_ratio']))
        if out['n_circles'] == 0:
            zero.append(s)
    print('\n' + '=' * 70)
    print(f"완료: {len(snaps) - len(failed)}/{len(snaps)} 폴더 처리, 원 총 {tot}개 "
          f"(가림 의심 {occ}개)")
    print(f"Z 저신뢰(유효율<{Z_REF_MIN_VALID:.0%}) 폴더 {len(lowz)}개: "
          f"{lowz if lowz else '없음 — 이 분기는 발동하지 않았다'}")
    print(f"검출 0개 폴더 {len(zero)}개: {zero if zero else '없음'}")
    print(f"처리 실패 폴더 {len(failed)}개: {[s for s, _ in failed] if failed else '없음'}")
    for s, e in failed:
        print(f"  - {s}: {e}")


if __name__ == '__main__':
    main()
