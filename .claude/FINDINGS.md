# 검증 발견 사항 (FINDINGS)

verifier 세션이 찾은 버그·공백 누적 기록. **확증**=코드로 증명 / **정황**=로그·상관관계(단정 금지).
최종 갱신: 2026-08-27 · 상태: 배치1 ✅완료 / #20 ✅완료 / 나머지 대기 (최신: #27 신규, #22·#24·#25·#26-A/B, #21 정정)

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
| | ⚠️ **수정 시 주의(advisor, 미래사고 방지)**: 수정 시 `_sys_procs['launch']`는 제외 — 로봇 수명 등급(M1). #12 자식정리(atexit+killpg)에 런치를 쓸어담으면 M1(파이프 분리)을 원상복귀시킨다. 세션 수명 자식만 정리. | | | | | |
| **13** | 버튼2 음성이 열린 마이크에 들어가 오인식/동시 발화 | vision_assistant.py (interface와 조율 0건) | 🔴 | 미배정 | 확증 | 대기 |
| **14** | probe 시작 0.5초 구간에 nav2와 cmd_vel 동시 발행 | push_probe `_run_probe` | 🟠 | 3 | 확증 | 대기(저) |
| **15** | 대시보드 로그가 파일로 전혀 안 남음(계측 공백) | main `_log()` 메모리 전용 ringbuffer maxlen=800 | 🔴계측 | A | 확증 | 대기 |
| **16** | 두 액션서버 전환 시 preempt 안 됨(동시 goal) | navigation_client `_send_to_nav2` | 🟠 | 미확증 | **정황** | 확인필요 |
| **17** | **주행 중 여러 번 정지 (← 사용자 원래 증상!)** | people_tracker 오분류 + `_replan_check` 1Hz 재계획 | 🔴 | A→B | 확증(발화 미관측) | 대기 |
| | **후보처방(배치B)**: #24의 구조수정(waypoint 비교로 불필요 1Hz 취소 제거)이 #17을 일부 직격 — social nav ON 경로에서의 원인 하나를 제거함. 단, 이 처방은 첫주행(사람시간대) 후 효과 확인 예정. | | | | | |
| **18** | 1-B 이후 두 오케 불일치(가짜도착 or 200초 침묵) | main `_auto_run` vs interface | 🔴 | A | 확증 | 대기 |
| | **하위 발견 "#18-침묵" 🟠**: interface `_watch` (L942-949) — `is_arrived`/`nav_failed`/`stop_ev` 셋 중 하나 안 뜨면 **타임아웃 없이 무한 대기**(**확증**, 직접 대조 — 루프에 시간제한 코드 없음). nav2가 무통보로 정지하면 시각장애인은 무한 침묵. 주행 후 수정 가능. | | | | | |
| **19** | 사람 데이터 신선도 없음 → 스톨 시 approaching 래치 | navigation_modifier `_people_cb` | 🔴 | A | 확증 | 대기 |
| | **역할 명확화**: #19는 안전수정이 아니라 **관측장비**(배치A) — 스톨과 오분류를 구분하기 위한 계측. 이 자체가 버그를 고치는 게 아니라 다른 항목(#17 등) 진단을 가능하게 하는 용도. | | | | | |
| **20** | 엘베 그리퍼 카메라(D405) 기동 실패 — 요청 fps가 D405 미지원 프로파일 → 근사 fallback → 혼합 fps → Frames Timeout | launch `stretch_robot_process.launch.xml` L85-93 (`depth_module.profile`/`color_profile`=`...x6` 요청. D405 실지원: Depth 480x270={60,30,15,5}, Color 1280x720={15,10,5} — 6 없음, 확증 직접 대조) | 🔴확증(분기)/미확정(기전) | 미배정 | 확증 (443런 중 fps6 개통 0회, 8/13 자연실험 35분 3성공1실패, 신설정 적용률 9%) | **✅완료 — 커밋 `3f25324`** |
| | ✅ **채택안(오케 확정)**: 후보A — 프로파일 파라미터 전부 삭제 → 848x480@10 드라이버 기본값 사용, 아무것도 추가 안 함. 근거: **"7/09 이후 1280 개통 0건"** (71런 실증). | | | | | |
| | **verifier 확정 한줄**: "런치가 D405 미지원 `x6`을 요구해 드라이버 협상이 런마다 갈림. 안전분기(기본값 848x480@10 균일) 71런 무사고 / 실패분기(부분적용, fps 혼합) 8/9 실패, 프레임 0장·자력회복 없음. 실패율 약 8%. 수정: 요구 자체 삭제→분기 제거(CAM-A). 판정 = `invalid` 에러 0 + Open profile 848@10 균일 1회 관측. 무사망 횟수로 판정 금지(기저 91%). 기전(혼합fps vs 센서 재시작반복) 미확정 — 격상금지." | | | | | |
| | **적용 확인(이 세션, 확증)**: `git diff`로 launch 파일 대조 — `depth_module.depth_profile`/`profile`/`color_profile`/`rgb_camera.profile`/`color_profile` 6줄 전부 삭제됨, 주석도 "프로파일 미지정 — 드라이버 기본협상" 으로 교체됨. CAM-A가 코드에 실제 반영된 상태(단, 미커밋). 게이트 통과 결과(invalid 2→0, timeout0, 16:37 재시작)는 이 세션이 로그로 직접 본 것은 아님 — verifier 보고 그대로, **정황**. | | | | | |
| **21** | 몸체 카메라(D435) 파라미터가 실제 적용 안 됨 → nav 과부하 → 주행 멈춤 (#17과 동일 뿌리) | launch 동일 파일 L42-64 (`depth_module.profile`="424x240x6"로 8/18 감축 결정 — 라인 존재 확증 대조. 단, 269런 실측 전부 848x480@30+1280x720@30로 관측됐다는 것은 정황, 이 세션은 런타임 로그 미확인) | 🔴 | A→B(#17과 동일 뿌리) | 확증(구성)/정황(영향) | 진단완료·수정설계중 |
| | ⚠️ **판정 기준 명확화(verifier 확정, 오기각 방지)**: "#21은 USB 대역폭 문제가 아니라 pointcloud 생성 + align_depth(720p 정렬) CPU/지연 부하다. 몸체가 USB3인 것은 #21의 반박이 아니다. 판정지표 = camera_depth 드롭 ÷ laser 드롭 비율(기준선 3.1~3.9배); 이 비율이 안 내려가면 #21 미해결. #21의 레버는 depth/color 프로파일이지 infra/motion(②)이 아니다." 참고: 몸체 infra 전수 269런 중 265런에 infra1/2+Motion Module 열림이나 구독 코드 0건(**확증**, grep) — 저가치라 ②(infra2/motion off)는 이번 커밋서 제외, #21 본체(프로파일) 수정은 첫주행 baseline 관측 뒤 별도 커밋으로 재개 예정. | | | | | |
| | ↩️ **정정(오늘)**: "카메라 3배"라는 표현은 발행률(30fps vs 10fps) 계산 아티팩트였음 — 철회. 다만 현상 자체(정지 시 local_costmap/odom 드롭)는 유효 — 카운트 이중경로가 일치해 신뢰 가능. 정량화(실제 부하 배율)는 소스레벨 계측이 별도로 필요. | | | | | |
| **22** | costmap_cleaner가 5초 간격으로 `ros2 service call` 프로세스를 새로 만들고 죽여 DDS churn 유발 (뿌리C) | launch 동일 파일 L144-147 (`sleep 5` 루프로 매 반복 새 프로세스 생성 — 확증 직접 대조. 실측 간격 22초=의도 대비 2.7배는 정황, 이 세션은 실측 로그 미확인) | 🟠 | C | 확증(구조)/정황(영향) | 진단완료·수정설계중 |
| **23** | 대시보드 재시작 시마다 런치·카메라·센서 노드가 딸려 죽어, 사용자가 런치를 재기동하다 #20 카메라 레이스를 반복 유발 | main `sys_proc_ctrl()`의 `_sys_procs` spawn 블록 — `stdout=subprocess.PIPE` 확증 직접 대조(대시보드 사망 시 파이프 read-end 닫힘 → 런치+상속자식 EPIPE/SIGPIPE 전파, 로직은 정황). `start_new_session=True`도 같은 위치에 이미 존재 확인 — 터미널 시그널 결합은 이미 차단됨, 결합 경로는 PIPE뿐. 줄번호 참고(커밋 `68c1adf` 기준, 재검증 완료): `stdout=subprocess.PIPE` L1356, `start_new_session=True` L1358, `_sys_procs[name]=proc` L1360 | 🔴 | C (#12와 상호작용, 위 참고) | **정황** — 확증 실험 대기(대시보드만 kill 후 `pgrep -f "ros2 launch"` 생존 확인, 1분) | 대기(1분실험) |
| **24** | social nav를 OFF해도 사람이 접근하면 로봇이 매초 goal 취소+재전송을 반복 — 회피는 안 하면서 조용히 멈칫거림(무기한, 통보 없음) | navigation_client `_replan_check` L202-204 — `approaching_present`가 `_modifier._people`를 직접 읽음. 게이트(`_social_nav_enabled`, L224-232)는 `_compute_waypoints_if_enabled()`에만 걸려있고 `approaching_present` 계산은 그 게이트를 거치지 않음(**확증**, 직접 대조 — 두 코드 경로가 물리적으로 분리돼 있음) | 🔴(실험유효)/🟠(운영) | B | 확증 | 진단완료·즉시수정후보 |
| | **즉시수정** = 게이트 1줄(`approaching_present` 계산에도 `_social_nav_enabled()` 체크 추가). **구조수정(배치B)** = `if waypoints == self._last_sent_waypoints: return`(불필요 1Hz 취소 제거 — #17에 후보처방으로 연결, 아래 참고). | | | | | |
| **25** | 로봇의 자기 위치를 people_tracker 발행값에 전적으로 의존 | navigation_modifier L47-49 `data.get('robot_x') or 0.0` — 값이 null이면 조용히 (0,0)으로 처리(배터리 NaN 계열과 동일 패턴, [[battery-nan-crash]]) | 🔴 | 별도배치 | 확증(코드경로)/미관측(실제 발생 빈도) | 대기 |
| | 수정 방향 = TF/AMCL에서 로봇 위치를 직접 읽도록 교체(별도 배치, 미착수). | | | | | |
| **26-A** | 초기자세(initial pose) 미설정 상태로 주행을 시작하면 nav2가 37분간 조용히 반쯤 죽는데도 대시보드는 "준비됨"으로 표시 — 오케까지 속을 정도로 아무 표시 안 남음. 시각장애인은 알아챌 방법이 없음 | map 프레임이 없는 상태로 `planner_server`가 activate 진행 중 global_costmap이 TF를 기다리며 블록됨(**확증**, 뿌리B) | 🔴안전 | 준비상태바(DECISIONS.md #10) | 확증 | 진단완료·설계확정 |
| | **하위 발견 "#26-B" 🟠확증(뿌리C)**: 이 블록 상태(#26-A)에서 종료 시도 시, transition 진행 중 SIGINT가 들어오면 `planner_server`가 SIGABRT(exit -6)로 죽음. #26-A가 선행조건. | | | | | |
| **27** | 수동주행 중 브라우저 연결이 끊기면(탭닫힘/크래시/와이파이 끊김/노트북 덮개 닫힘) 로봇이 마지막 속도로 **영원히 계속 움직임** | main `_manual_loop()` — `_manual_active`면 0.1초마다 `_manual_cmd` 마지막 값을 계속 발행. `/cmd`(`cmd()` 함수)는 값을 덮어쓰기만 하고 **수신 시각을 전혀 기록하지 않음**(**확증**, 타임스탬프 변수 자체가 없음). 줄번호 참고(커밋 `68c1adf` 기준, HTML 추출 후 재검증 완료): `_manual_loop` L403-412, `cmd()` L439-448 | 🔴안전 | A | 확증(코드경로) | 진단완료·수정설계중 |
| | 처방(code-editor 제안): `/cmd`에 수신시각 기록 + `_manual_loop`에서 `now - last_cmd_t > ~0.5s`면 `publish_cmd(0,0)` + `_manual_active=False`(서버측 데드맨). #2(손잡이 당겨도 계속 감, `navigation_client.cleanup()`)와 증상은 유사하나 경로가 다름(#2=자동주행 취소 경로, #27=수동주행 `_manual_loop` 신규 발견) — 별개 항목 유지. "2a 나이작업과 합칠 후보" = 대시보드 통합 스텝2a(`/readiness` 백엔드, DECISIONS.md #10) — 상태신호에 갱신 타임스탬프를 붙이는 작업이라 #27 처방(수신시각 기록)과 같은 부류. verifier 판정 대기. | | | | | |

## 뿌리 3개
- **A. 실패·취소 시 안전상태로 못 돌아감** (1·2·3·5·6·9·27) — 반환값 안 보고 이동으로 진행. 물리안전 4건.
- **B. 실패가 사용자에게 전달 안 됨** (1·L·6·10·24·26-A) — 틀린 곳/층에 "도착" 선언, 또는 아무 표시 없이 조용히 반쯤 죽음. 못 알아챔.
- **C. 프로세스 생명주기 관리 부재** (4·11·12·22·26-B) — 좀비·고아·유령 참가자 + DDS churn.

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
- #21 — "269런 848x480@30+1280x720@30 관측" 자체(런타임 실제 협상 해상도)는 이 세션에서 로그 미확인.
- #22 — "실측 간격 22초=의도 2.7배"는 이 세션에서 로그 미확인.
- #23 — **전제 조건 미확인 상태**: 런치가 대시보드 버튼으로 떠야 성립. 사용자가 터미널에서 직접 `ros2 launch`하면 파이프 결합 없음 → 기각. 사용자 확인 중.

## 주행 유효성 최소 계측세트
**#15(로그영속) + #24(social nav 게이트 우회 즉시수정) + #19(사람데이터 신선도 관측)** = 첫 주행 1회를 "유효하게 만드는" 최소 계측 조합. 하나라도 빠지면 사람 있는 시간대의 주행 기회 1회가 계측 공백으로 낭비될 위험.

**🟡 저비용 개선 후보(미배정 번호)**: `/tmp/navigation_active` — navigation_client가 이미 쓰고 있음(L101, L370, L387, L417, **확증** 직접 대조)인데 main.py가 읽지 않음(**확증**, 전체 grep 0건). 3~4줄만 추가하면 대시보드에 주행상태 실시간 표시 가능.

**🟡 계측 공백(미배정 번호)**: main.py의 attach 호출에 `cameras=` 인자가 없어 대시보드가 카메라 상태를 전혀 감시하지 못함(수정 4줄 규모). 손잡이(아두이노) 연결 끊김 시 3초 간격 무한 재시도가 로그를 스팸함.

## 보류 후보 (미래 검토, 폐기 아님)
- **D′ (D405 OCR 개선안)**: `rgb_camera.color_profile=1280x720x5` + `depth_module.depth_profile=848x480x5` + `enable_infra1=false`. D405 지원 프로파일로 실측 확인됨, 대역폭 15.1MB/s(<24.4 한도), ROI 디테일 1.51배(#20 채택안 848x480@10 대비). 조건: fps 균일 필수, 게이트 3개(미상세). advisor 설계·데이터 뒷받침.
- **채택 시점**: 지금은 미채택 — #20 후보A 실측 데이터(첫 주행 baseline)를 본 뒤, 층 표시기 OCR 판독력이 부족하다고 판단되면 그때 재검토.

## 결정 로그
- **배치1 (#1)**: `_replan_check`은 cancel 후라 `if not _send_to_nav2: _nav_failed` as-is. `_obstacle_cb`은 cancel 없는 의도된 preempt라 **(d) 채택** — `goal_handle is None`일 때만 `_nav_failed`, 살아있으면 로그만(정상 주행 유지). `_nav_failed`에 `get_logger().error()` 추가(→/rosout→robot_diag_nav 영속). server 대기는 `start_navigation`에서 1회, `_send_to_nav2`는 `server_is_ready()` 논블로킹. goal_gen으로 낡은 콜백 필터. 재진입 플래그 `_failed_announced` start_navigation에서 리셋. **verifier 최종 통과.**
- **twist_mux**: 맞는 도구지만 "런치-only-안전" 아님(6파일 재배선+엘베 캘리브레이션+cleanup 정지Twist 무력화→비상정지 재설계+엘베 최우선). 보류, 배치D.
- **14/(d) 긴급도 하향**: `_obstacle_push_enabled=False`라 obstacle_pusher 실질 미사용.

## 지금 검증 가능(주행 없이): L, 6, 8, 10, 11, 12, 13, 15, 17(코드)

## 주행 전 체크리스트 (유실 방지용, #26-A 이후 추가)
주행 시작 전 아래 4가지를 확인. 대시보드 준비상태 바(DECISIONS.md #10)가 구현되기 전까지는 수동 확인 필수.
- **손잡이 연결**: `ls /dev/serial/by-id/` — 기대 디바이스 존재 확인
- **측위**: `/amcl_pose` 발행 확인 (map 프레임 있어야 함 — #26-A 재발 방지)
- **planner_server 상태**: `ros2 lifecycle get /planner_server` → `active` 확인
- **nav2 bond**: 6개 전부 active 확인
