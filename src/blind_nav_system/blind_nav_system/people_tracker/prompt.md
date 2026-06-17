# 사회적 규범 기반 실내 경로 계획 — 구현 설계

**플랫폼**: Hello Robot Stretch SE3 / ROS2 Humble / Intel RealSense D435i (RGB-D) / CPU only

---

## 왜 이게 필요한가

Nav2 기본 경로 계획은 "장애물을 피한다"만 안다.  
보행자가 반대 방향에서 걸어오면, 로봇이 멈추거나 진동하다가 재계획을 반복한다.  
사전에 흐름을 파악해 우측으로 비켜가면 이 재계획이 줄고, **이동 시간이 단축**된다.

---

## 단계별 논리

### Step 1. 사람 검출
RGB 이미지에서 사람의 위치를 찾아야 한다.

- **어떻게**: YOLOv8 (가장 가벼운 n 모델) + ByteTrack
- **왜 ByteTrack**: 프레임 간 같은 사람에 동일 ID를 유지해야 방향 추정이 가능하다
- **출력**: 사람별 bounding box + track_id

---

### Step 2. 사람 방향 추정
한 프레임만 봐서는 방향을 알 수 없다. 시간 이력이 필요하다.

- **어떻게**: track_id별로 위치(중심점) 이력을 쌓고, 선형회귀로 속도 벡터(vx, vy) 추정
- **문제**: 픽셀 좌표는 카메라 각도에 따라 달라진다 → 맵 좌표로 변환해야 신뢰성 있음
- **맵 좌표 변환**: depth 이미지 + 카메라 내부 파라미터 → 3D 카메라 좌표 → TF2로 map 프레임 변환
- **출력**: 사람별 맵 기준 속도 벡터 (vx, vy) + 방향 각도 (degree)

---

### Step 3. 군중 흐름 판단
개별 방향을 모아 "지금 복도에서 사람들이 전반적으로 어느 방향으로 가고 있나"를 판단한다.

- **어떻게**: 이동 중인 사람들의 vx, vy 평균 → 시간적 스무딩 (최근 N프레임 이동평균)
- **주의**: 정지한 사람은 제외한다 (속도 threshold 이하는 무시)
- **출력**: `dominant_angle` (군중 전체의 지배적 이동 방향, degree)
- **ROS 연동**: 이 결과를 `/people_tracker/crowd_flow` 토픽으로 퍼블리시해야 다른 노드가 사용 가능하다

---

### Step 4. 로봇 진행 방향 vs 군중 방향 비교
로봇이 어느 방향으로 가고 있는지 알아야 한다.

- **어떻게**: TF2에서 `map → base_link` 변환의 yaw 각도 추출
- **비교**:
  - `|dominant_angle - robot_yaw| > 90°` → 반대 방향 (마주오는 흐름)
  - `|dominant_angle - robot_yaw| ≤ 90°` → 같은 방향
  - 이동하는 사람 없음 → 판단 불가, 기본 경로 유지

---

### Step 5. 목표 위치 오프셋 계산
로봇 목적지는 그대로지만, 거기 도달하는 **경유 방향**을 오른쪽으로 틀어야 한다.

- **어떻게**: 현재 goal pose를 로봇 진행방향 기준 **우측으로 d미터 이동**한 위치로 수정
  ```
  right_dir = robot_yaw - 90°  (ROS: +y가 왼쪽이므로 -90° = 오른쪽)
  
  offset_goal.x = goal.x + d * cos(right_dir)
  offset_goal.y = goal.y + d * sin(right_dir)
  ```
- **d 값**:
  - 반대 방향 흐름: 0.4 m
  - 같은 방향 흐름: 0.2 m
  - 흐름 없음: 0 m
- **안전**: Nav2 costmap이 벽/장애물 충돌을 막으므로 offset이 지나쳐도 자동 보정된다

---

### Step 6. Nav2에 전달
수정된 goal pose를 `navigate_to_pose` 액션에 보내면 된다.  
최종 목적지가 아니라, 최종 목적지로 가는 중간 경로 편향을 주는 것이 목적이다.

---

## 구현 순서 (처음부터 짜는 순서)

```
1. PersonTracker  —  YOLOv8n + ByteTrack으로 사람 검출·ID 추적
2. DirectionEstimator  —  맵 좌표 기반 속도 벡터 추정 (depth + TF2 필요)
3. CrowdFlowEstimator  —  dominant_angle 계산 + ROS 토픽 퍼블리시
4. SocialPlannerNode  —  robot yaw vs dominant_angle 비교 → offset goal 서비스
5. NavigationClient 수정  —  goal 전송 전 offset 적용
```

---

## 논문에서 시간 단축을 어떻게 보이나

**실험 설정**: 복도 10 m, 보행자 3~5명이 로봇 반대 방향으로 걷는 상황

| 조건 | 예상 동작 |
|------|-----------|
| Baseline (현재) | 중앙 주행 → 보행자와 마주침 → 로컬 플래너 재계획 반복 → 느림 |
| Proposed | 사전에 우측 주행 → 마주침 없음 → 재계획 없음 → 빠름 |

**측정 지표**:
- `T_nav`: 구간 이동 시간 (초)
- `N_replan`: Nav2 경로 재계획 횟수
- `N_osc`: cmd_vel angular.z 방향이 짧은 시간 안에 반전된 횟수 (진동 지표)

10회 반복 실험 → 평균 비교 → "Proposed가 T_nav 기준 XX% 단축"
