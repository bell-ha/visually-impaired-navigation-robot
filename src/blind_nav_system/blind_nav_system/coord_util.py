"""좌표 변환 한 곳 — 쿼터니언 ↔ yaw(도). **이 파일이 그 규약의 유일한 구현이다.**

왜 생겼나 (2026-09-10)
    이 저장소에는 변환 **함수**가 없었다. 규약은 주석으로 있는데
    (`elevator_button_press/scene_targets.yaml` 헤더의 공식, `config/location.yaml`
    각 항목의 `z = sin(yaw/2), w = cos(yaw/2)` 수기 계산),
    **실제 변환은 사람이 계산기를 두드려 yaml 에 손으로 박아 넣는 것**이 유일한 경로였다.
    그러면 계산 실수가 아무 검증 없이 좌표가 된다 — `엘리베이터 하차지점` 이
    `엘리베이터 문앞` 과 29.22cm / 8.00° 어긋난 채 남아 있던 건(FINDINGS #182)이
    그 부류다. 함수로 만들면 최소한 **계산 실수**는 사라지고, 기존 값을 기계로
    재검증할 수 있다.

규약 (이 프로젝트)
    · 프레임: map. 각도 단위: 도(°), 범위 (-180, 180].
    · 로봇은 항상 수평이다 → 자세는 yaw 하나로 결정되고 x = y = 0 이다.
      그래서 `z = sin(yaw/2)`, `w = cos(yaw/2)` 가 성립한다.
    · 일반형(x, y 가 0 이 아닐 때도 맞는 2D yaw 추출):
          yaw = atan2(2(wz + xy), 1 - 2(yy + zz))
      두 식은 x = y = 0 에서 같은 값을 낸다. 이 파일은 일반형으로 읽고(안전),
      쓸 때는 단순형으로 만든다(로봇이 수평이라는 전제를 그대로 반영).

경계 (이 파일이 **안 하는** 것)
    · **값을 고치지 않는다.** 좌표 통합·`엘리베이터 하차지점` 처리는 별 건이고
      사용자 결정 대기 중이다(wip/20260910-coord-unification-design.md 3번).
      여기는 변환만 제공한다.
    · `/amcl_pose` 가 아니라 map→base_link **TF** 를 쓴다는 규약은 **다른 규약**이고
      이 파일 범위가 아니다(그쪽은 main.py:235 · scale_audit.py:21 · compare.py:51-52
      에 적혀 있다). 두 규약을 섞지 말 것 — 하나는 '각도 표기', 하나는 '측정 출처'다.
    · `snapshots/vlm_mapping/compare.py` 의 `quat_to_R` 은 회전행렬 전체가 필요한
      오프라인 분석용이라 그대로 둔다. 런타임 코드가 snapshots 아래를 import 하는
      경계가 이상해서 옮기지 않았다.

사람이 쓰는 법 (계산기 대신)
    python3 coord_util.py 83.3            → z/w 를 찍는다 (yaml 에 붙여넣기)
    python3 coord_util.py 0.664579 0.747218  → yaw(도) 를 찍는다 (기존 값 검산)
"""

import math

__all__ = ["yaw_deg_to_quat", "quat_to_yaw_deg", "quat_dict_to_yaw_deg",
           "yaw_deg_norm"]


def yaw_deg_norm(deg: float) -> float:
    """각도를 (-180, 180] 로 정규화. 180 은 180 으로 남긴다(-180 으로 넘기지 않는다)."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def yaw_deg_to_quat(yaw_deg: float) -> tuple:
    """yaw(도) → (z, w). x = y = 0 (로봇 수평 전제)."""
    h = math.radians(yaw_deg_norm(yaw_deg)) / 2.0
    return math.sin(h), math.cos(h)


def quat_to_yaw_deg(z: float, w: float, x: float = 0.0, y: float = 0.0) -> float:
    """쿼터니언 → yaw(도). 일반형으로 읽으므로 x/y 가 0 이 아니어도 맞다."""
    z, w, x, y = float(z), float(w), float(x), float(y)
    return yaw_deg_norm(math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))))


def quat_dict_to_yaw_deg(d: dict) -> float:
    """yaml 항목(dict) → yaw(도). `z`/`w` 는 필수, `x`/`y` 는 없으면 0 으로 본다.

    location.yaml 항목이 `x`/`y` 를 **위치** 좌표로 쓰고 쿼터니언 x/y 는 아예 안 적는
    형식이라, 위치 키와 헷갈리지 않게 쿼터니언 x/y 는 `qx`/`qy` 로만 읽는다.
    (그래서 location.yaml 항목을 그대로 넣어도 위치 x/y 가 회전에 섞이지 않는다.)
    """
    return quat_to_yaw_deg(d["z"], d["w"], d.get("qx", 0.0), d.get("qy", 0.0))


if __name__ == "__main__":
    import sys
    a = sys.argv[1:]
    if len(a) == 1:
        z, w = yaw_deg_to_quat(float(a[0]))
        print(f"yaw {yaw_deg_norm(float(a[0])):.4f}° → z: {z:.6f}  w: {w:.6f}")
    elif len(a) == 2:
        print(f"z {a[0]} / w {a[1]} → yaw {quat_to_yaw_deg(float(a[0]), float(a[1])):.4f}°")
    elif len(a) == 4:
        print(f"→ yaw {quat_to_yaw_deg(float(a[2]), float(a[3]), float(a[0]), float(a[1])):.4f}°")
    else:
        print(__doc__.rstrip().rsplit("사람이 쓰는 법", 1)[-1].strip())
        raise SystemExit(2)
