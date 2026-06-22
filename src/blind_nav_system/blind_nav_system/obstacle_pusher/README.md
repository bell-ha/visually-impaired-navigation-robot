# obstacle_pusher — 장애물 밀기 기반 경로 최적화

depth 카메라로 물체 장애물을 감지하고, 우회하는 대신 **밀고 지나가거나 가장자리를 스쳐 치우는** 방식으로 이동 시간을 줄이는 모듈.

> **통합 시스템 위치:**
> `people_tracker` (사람 장애물) + `obstacle_pusher` (물체 장애물)
> → 두 모듈이 함께 이동시간 최소화를 목표로 함

---

## 핵심 아이디어

기존 Nav2는 모든 장애물을 우회한다. 이 시스템은 물체가 밀릴 수 있는지 판단하고,
밀 수 있으면 **박스 옆 경유지**를 설정해 로봇이 가장자리를 스쳐 지나가며 자연스럽게 치우게 한다.

```
Nav2 경로 상에 물체 감지
        │
        ▼
   YOLO 분류
        │
 ┌──────┼────────────┐
 ▼      ▼            ▼
의자/가구  병/컵/공   박스 등 미인식
        │
  즉시 우회  바로 밀기    탐침 push
  (접촉 X)  (경유지)     (0.08m/s × 1.5s)
                         └→ 움직이면: 경유지 삽입
                         └→ 안 움직이면: 우회
```

**논문 Contribution:**
> 기존 시스템은 모든 장애물을 우회한다. 이 시스템은 물리적 탐침으로 실제 저항을 측정해
> 밀기/우회를 결정하고, 박스 옆 경유지를 통해 치우며 통과한다.

---

## 구현된 파이프라인

```
[aligned_depth_to_color]  [color/image_raw]
         │                       │
         ▼                       ▼
 [obstacle_detector.py]
   RANSAC 평면 제거 (바닥/벽)
   → DBSCAN 클러스터링
   → 크기 필터 (5~60cm)
   → YOLO로 분류:
       "pushable"     초록 큐브
       "not_pushable" 빨강 큐브
       "unknown"      노란 큐브
         │
         ▼ /obstacle_pusher/objects (JSON)
 [push_probe.py]
   pushable     → 즉시 push 신호
   not_pushable → 즉시 detour 신호
   unknown      → 탐침 (cmd_vel 0.08m/s × 1.5s)
                  → 이동량 측정 → push or detour 신호
         │
         ▼ /obstacle_pusher/decision (JSON)
 [navigation_client.py]
   push   → 박스에서 오른쪽 20cm 경유지 삽입 → Nav2 재계획
   detour → 기존 경로 유지 (Nav2 costmap 우회)
```

---

## 파일 구조

| 파일 | 역할 | 상태 |
|------|------|------|
| `obstacle_detector.py` | depth 클러스터링 + YOLO 분류 → 박스 후보 퍼블리시 | ✅ 구현 |
| `push_probe.py` | 탐침 push + push/detour 결정 퍼블리시 | ✅ 구현 |
| `main.py` | 두 노드 통합 진입점 | ✅ 구현 |

> `navigation_client.py` (상위 모듈)가 `/obstacle_pusher/decision`을 구독해 경로 재계획

---

## 주요 토픽

| 토픽 | 방향 | 내용 |
|------|------|------|
| `/obstacle_pusher/objects` | 발행 | 박스 후보 목록 (위치, 크기, pushable 분류) |
| `/obstacle_pusher/decision` | 발행 | `push` / `detour` / `probe_start` 결정 |
| `/obstacle_pusher/markers` | 발행 | RViz 시각화 (큐브 + 텍스트) |

---

## YOLO 분류 기준 (COCO class ID)

| 분류 | 물체 | 동작 |
|------|------|------|
| `not_pushable` | 의자(56), 소파(57), 침대(59), 테이블(60), TV(62) 등 | 즉시 우회 |
| `pushable` | 병(39), 컵(41), 공(32), 책(73), 리모컨(65) 등 | 바로 밀기 |
| `unknown` | **골판지 박스** 등 YOLO 미인식 | 탐침 push로 물리적 판단 |

> 실험에 쓰는 골판지 박스는 YOLO가 인식하지 못하므로 항상 `unknown → 탐침` 경로를 탄다.

---

## 탐침 push 파라미터

| 파라미터 | 값 | 의미 |
|---------|-----|------|
| `PROBE_SPEED` | 0.08 m/s | 탐침 속도 |
| `PROBE_DURATION` | 1.5 s | 탐침 시간 |
| `PROBE_MOVE_THR` | 0.04 m | 이 이상 이동 시 "가벼움" 판정 |
| `PROBE_TRIGGER_DIST` | 1.2 m | 이 거리 이내일 때만 탐침 시작 |
| `PUSH_SIDE_OFFSET` | 0.20 m | 박스 중심에서 옆으로 스쳐갈 오프셋 |

---

## ON/OFF 제어

웹 대시보드 (`../main.py`) 에서 **📦 장애물 밀기** 버튼으로 실시간 토글.

```
/tmp/obstacle_push_enabled  →  "1" = ON, "0" = OFF
```

사회적 회피(사람)와 독립적으로 켜고 끌 수 있어 실험 조건 분리 가능:

| 실험 조건 | 사회적 회피 | 장애물 밀기 |
|-----------|------------|------------|
| Baseline Nav2 | OFF | OFF |
| 사회적 회피만 | ON | OFF |
| 장애물 밀기만 | OFF | ON |
| 통합 시스템 | ON | ON |

---

## RViz 시각화

| 색상 | 의미 |
|------|------|
| 초록 큐브 | YOLO `pushable` — 가벼운 물체 |
| 빨강 큐브 | YOLO `not_pushable` — 가구/고정물 |
| 노란 큐브 | `unknown` — 탐침 대기 중 |
| 초록 텍스트 | 밀고 통과 결정 (push) |
| 주황 텍스트 | 우회 결정 (detour) |

---

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
cd .../blind_nav_system/obstacle_pusher
python3 main.py
```

---

## 논문 실험 시나리오

| 시나리오 | 설정 | 측정 지표 |
|----------|------|-----------|
| A — 빈 복도 | 장애물 없음 | 기본 이동 시간 |
| B — 가벼운 박스 | 골판지 박스 (빈 것) | 탐침 후 밀기 성공 → 이동 시간 |
| C — 무거운 박스 | 골판지 박스 (무거운 것) | 탐침 후 우회 → 이동 시간 |
| D — 사람 + 박스 | 동시 등장 | 두 모듈 동시 동작 확인 |

**비교 지표:** 이동 완료 시간(초), 우회 대비 절약 거리(m), 탐침 성공률
