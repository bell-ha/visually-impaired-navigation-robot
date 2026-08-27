# 검증 발견 사항 (FINDINGS)

verifier 세션이 찾은 버그·공백 누적 기록. **확증**=코드로 증명 / **정황**=로그·상관관계(단정 금지).
최종 갱신: 2026-08-27 · 상태: 배치1 ✅완료 / 나머지 대기

## 발견 목록

| # | 사용자가 겪는 증상 | 위치 | 심각도 | 배치 | 확신도 | 상태 |
|---|---|---|---|---|---|---|
| **L** | 다른 층 말하면 현재층 같은 자리로 데려가고 "도착했습니다" | interface.py / navigation_client.py (floor 참조 0건) | 🔴안전 | B | 확증 | 대기 |
| **1** | Nav2 재전송 실패 시 멈춘 채 침묵 | navigation_client `_obstacle_cb`/`_replan_check` | 🔴 | 1 | 확증 | ✅완료 |
| **2** | 손잡이 당겨 일시정지했는데 로봇 계속 감 | navigation_client `cleanup()` — goal_handle 아직 None | 🔴안전 | 3 | 확증 | 대기 |
| **3** | 여정 취소했는데 엘베 앞 전진 56.5+회전 90° | main `_auto_run` — `_elev_wait_press_done()` 반환 무시 | 🔴안전 | 2 | 확증 | 대기 |
| **4** | 주행 중 다른 장소 클릭 시 노드명 충돌(유령참가자) | navigation_client L31 `nav_client_{int(time.time())}` | 🟠 | 3 | 확증 | 대기 |
| **5** | 엘베앱 죽어도 "누르는 중" 말하고 문 앞 전진 | main `_auto_run` — scene/select/press 반환 전부 무시 | 🔴안전 | 2 | 확증 | 대기 |
| **6** | 지도전환 실패해도 진행 → 4층인데 5층 지도로 주행 | main `/switch_map` 실패 2경로가 HTTP 200 | 🔴 | 2 | 확증 | 대기 |
| **7** | 층 전환 후 AMCL이 이전 층 위치 믿은 채 주행 | main `switch_map` — initialpose 발행 후 검증 없음 | 🟠 | 3 | 확증(로그2) | 대기 |
| **8** | 목적지 말하고 최대 35초 무반응(버튼도 안 먹음) | interface `GPTPlanner.plan` timeout=35, 메인 루프 블로킹 | 🟠 | 4 | 확증 | 대기 |
| **9** | 취소 시 팔 뻗은 채 고착 → 그대로 주행 → 문틀 충돌 | 엘베 authority 회수·SIGTERM 경로에 팔 수납 없음 | 🔴안전 | 2 | 확증 | 대기 |
| **10** | 이동 중 배터리 고갈 시 말없이 멈춤 | main — 임계·경고·중단 로직 0건 (voltage 기준 필요) | 🟡공백 | 4 | 확증 | 대기 |
| **11** | 대시보드 두 번 실행 → 마이크 잡은 좀비 남음 | main `main()` — 포트 바인딩이 맨 마지막 | 🟠운영 | 3 | 확증 | 대기 |
| **12** | 대시보드 크래시/kill 시 자식 전부 방치 | main `start_subprocesses` — start_new_session 없음, atexit 0 | 🟠운영 | 3 | 확증 | 대기 |
| **13** | 버튼2 음성이 열린 마이크에 들어가 오인식/동시 발화 | vision_assistant.py (interface와 조율 0건) | 🔴 | 미배정 | 확증 | 대기 |
| **14** | probe 시작 0.5초 구간에 nav2와 cmd_vel 동시 발행 | push_probe `_run_probe` | 🟠 | 3 | 확증 | 대기(저) |
| **15** | 대시보드 로그가 파일로 전혀 안 남음(계측 공백) | main `_log()` 메모리 전용 ringbuffer maxlen=800 | 🔴계측 | A | 확증 | 대기 |
| **16** | 두 액션서버 전환 시 preempt 안 됨(동시 goal) | navigation_client `_send_to_nav2` | 🟠 | 미확증 | **정황** | 확인필요 |
| **17** | **주행 중 여러 번 정지 (← 사용자 원래 증상!)** | people_tracker 오분류 + `_replan_check` 1Hz 재계획 | 🔴 | A→B | 확증(발화 미관측) | 대기 |
| **18** | 1-B 이후 두 오케 불일치(가짜도착 or 200초 침묵) | main `_auto_run` vs interface | 🔴 | A | 확증 | 대기 |
| **19** | 사람 데이터 신선도 없음 → 스톨 시 approaching 래치 | navigation_modifier `_people_cb` | 🔴 | A | 확증 | 대기 |

## 뿌리 3개
- **A. 실패·취소 시 안전상태로 못 돌아감** (1·2·3·5·6·9) — 반환값 안 보고 이동으로 진행. 물리안전 4건.
- **B. 실패가 사용자에게 전달 안 됨** (1·L·6·10) — 틀린 곳/층에 "도착" 선언. 못 알아챔.
- **C. 프로세스 생명주기 관리 부재** (4·11·12) — 좀비·고아·유령 참가자.

## 배치 계획
- **배치1** = #1 (1-B 완결) — ✅ **완료** (아래 결정 로그)
- **배치A (주행 준비)** = #15(로그영속) + #18(orchestrator 정합) + #19(신선도 3줄) → 첫 주행이 유효+포착+구분가능
- **통제된 첫 주행** = people_tracker ON, social nav OFF, 같은 층·엘베 없음, 사람 있는 시간대 → #17 실증
- **배치B** = #17 #0(social gate)/#1(재계획 간격)/#2(분류)/#3(히스테리시스) + #2·#4·#7
- **배치C** = 엘베 물리안전 #3·#5·#9 (엘베 주행 전 필수)
- **배치D** = #8·#10·#13 + twist_mux(결정)

## 확증 아닌 것 (정황/참고 — 단정 금지)
- `wrist_extension contact detected` ×10 — #3·#9의 정황. 정상 press와 구분 불가.
- `cmd_vel_pubs=8` — twist_mux 판단 근거자료. 충돌 확증 아님.
- #16 — Nav2 Humble이 두 navigator 동시실행 막는지 미확인. `ros2 action list` 실측 필요.

## 결정 로그
- **배치1 (#1)**: `_replan_check`은 cancel 후라 `if not _send_to_nav2: _nav_failed` as-is. `_obstacle_cb`은 cancel 없는 의도된 preempt라 **(d) 채택** — `goal_handle is None`일 때만 `_nav_failed`, 살아있으면 로그만(정상 주행 유지). `_nav_failed`에 `get_logger().error()` 추가(→/rosout→robot_diag_nav 영속). server 대기는 `start_navigation`에서 1회, `_send_to_nav2`는 `server_is_ready()` 논블로킹. goal_gen으로 낡은 콜백 필터. 재진입 플래그 `_failed_announced` start_navigation에서 리셋. **verifier 최종 통과.**
- **twist_mux**: 맞는 도구지만 "런치-only-안전" 아님(6파일 재배선+엘베 캘리브레이션+cleanup 정지Twist 무력화→비상정지 재설계+엘베 최우선). 보류, 배치D.
- **14/(d) 긴급도 하향**: `_obstacle_push_enabled=False`라 obstacle_pusher 실질 미사용.

## 지금 검증 가능(주행 없이): L, 6, 8, 10, 11, 12, 13, 15, 17(코드)
