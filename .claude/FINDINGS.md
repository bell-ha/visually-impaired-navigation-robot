# 검증 발견 사항 (FINDINGS)

verifier 세션이 찾은 버그·공백 누적 기록. **확증**=코드로 증명 / **정황**=로그·상관관계(단정 금지).
최종 갱신: 2026-08-28 · **#35 UDP 버퍼 미스매치 — 수치확증 + 조치(sysctl 16MB) 적용 완료(이 세션 직접 재확인) · #36 신규(수동-grant deadman 사각지대) ✅수정완료(커밋 `7c382a3`)** · 상태: 배치1 ✅완료 / #20 ✅완료 / 2a·2b(#10 준비상태 바) 완료 / 리스1~3(#11 엘베 제어권 단일주인) 완료·물리검증 대기 / **#26-A 실기 근본메커니즘 확증(nav-activation 시 localization 미준비→lifecycle manager hung), 물리주행 검증은 아직** / 나머지 대기 (최신: #34 신규(런치 토글 레이스, 근본C) + #23 재검토 권고, #33 신규(RViz 2D Pose Estimate 미전달), #26-A 근본메커니즘 기록, #29~32 신규(리스 리팩터 중 이중비평이 잡은 물리안전 결함, 모두 수정됨), 2a 공백 2건, #28 신규, #20 세부메모 정정, #27·#22·#24·#25·#26-A/B)

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
| | **적용 확인(확증)**: launch 파일 대조 — `depth_module.depth_profile`/`profile`/`color_profile`/`rgb_camera.profile`/`color_profile` 6줄 전부 삭제됨, 주석도 "프로파일 미지정 — 드라이버 기본협상" 으로 교체됨. ↩️ **정정**: CAM-A는 **✅커밋 `3f25324`** 완료 상태(이전 기록의 "단, 미커밋"은 이 항목 상태칸과 모순되는 stale 기록이었음 — 삭제·정정). 게이트 통과 결과(invalid 2→0, timeout0, 16:37 재시작)는 이 세션이 로그로 직접 본 것은 아님 — verifier 보고 그대로, **정황**. | | | | | |
| **21** | 몸체 카메라(D435) 파라미터가 실제 적용 안 됨 → nav 과부하 → 주행 멈춤 (#17과 동일 뿌리) | launch 동일 파일 L42-64 (`depth_module.profile`="424x240x6"로 8/18 감축 결정 — 라인 존재 확증 대조. 단, 269런 실측 전부 848x480@30+1280x720@30로 관측됐다는 것은 정황, 이 세션은 런타임 로그 미확인) | 🔴 | A→B(#17과 동일 뿌리) | 확증(구성)/정황(영향) | 진단완료·수정설계중 |
| | ⚠️ **판정 기준 명확화(verifier 확정, 오기각 방지)**: "#21은 USB 대역폭 문제가 아니라 pointcloud 생성 + align_depth(720p 정렬) CPU/지연 부하다. 몸체가 USB3인 것은 #21의 반박이 아니다. 판정지표 = camera_depth 드롭 ÷ laser 드롭 비율(기준선 3.1~3.9배); 이 비율이 안 내려가면 #21 미해결. #21의 레버는 depth/color 프로파일이지 infra/motion(②)이 아니다." 참고: 몸체 infra 전수 269런 중 265런에 infra1/2+Motion Module 열림이나 구독 코드 0건(**확증**, grep) — 저가치라 ②(infra2/motion off)는 이번 커밋서 제외, #21 본체(프로파일) 수정은 첫주행 baseline 관측 뒤 별도 커밋으로 재개 예정. | | | | | |
| | ↩️ **정정(오늘)**: "카메라 3배"라는 표현은 발행률(30fps vs 10fps) 계산 아티팩트였음 — 철회. 다만 현상 자체(정지 시 local_costmap/odom 드롭)는 유효 — 카운트 이중경로가 일치해 신뢰 가능. 정량화(실제 부하 배율)는 소스레벨 계측이 별도로 필요. | | | | | |
| **22** | costmap_cleaner가 5초 간격으로 `ros2 service call` 프로세스를 새로 만들고 죽여 DDS churn 유발 (뿌리C) | launch 동일 파일 L144-147 (`sleep 5` 루프로 매 반복 새 프로세스 생성 — 확증 직접 대조. 실측 간격 22초=의도 대비 2.7배는 정황, 이 세션은 실측 로그 미확인) | 🟠 | C | 확증(구조)/정황(영향) | 진단완료·수정설계중 |
| **23** | 대시보드 재시작 시마다 런치·카메라·센서 노드가 딸려 죽어, 사용자가 런치를 재기동하다 #20 카메라 레이스를 반복 유발 | main `sys_proc_ctrl()`의 `_sys_procs` spawn 블록 — `stdout=subprocess.PIPE` 확증 직접 대조(대시보드 사망 시 파이프 read-end 닫힘 → 런치+상속자식 EPIPE/SIGPIPE 전파, 로직은 정황). `start_new_session=True`도 같은 위치에 이미 존재 확인 — 터미널 시그널 결합은 이미 차단됨, 결합 경로는 PIPE뿐. 줄번호 참고(커밋 `68c1adf` 기준, 재검증 완료): `stdout=subprocess.PIPE` L1356, `start_new_session=True` L1358, `_sys_procs[name]=proc` L1360 | 🔴 | C (#12와 상호작용, 위 참고) | **정황** — 확증 실험 대기(대시보드만 kill 후 `pgrep -f "ros2 launch"` 생존 확인, 1분) | 대기(1분실험) |
| **24** | social nav를 OFF해도 사람이 접근하면 로봇이 매초 goal 취소+재전송을 반복 — 회피는 안 하면서 조용히 멈칫거림(무기한, 통보 없음) | navigation_client `_replan_check` L202-204 — `approaching_present`가 `_modifier._people`를 직접 읽음. 게이트(`_social_nav_enabled`, L224-232)는 `_compute_waypoints_if_enabled()`에만 걸려있고 `approaching_present` 계산은 그 게이트를 거치지 않음(**확증**, 직접 대조 — 두 코드 경로가 물리적으로 분리돼 있음) | 🔴(실험유효)/🟠(운영) | B | 확증 | 진단완료·즉시수정후보 |
| | **즉시수정** = 게이트 1줄(`approaching_present` 계산에도 `_social_nav_enabled()` 체크 추가). **구조수정(배치B)** = `if waypoints == self._last_sent_waypoints: return`(불필요 1Hz 취소 제거 — #17에 후보처방으로 연결, 아래 참고). | | | | | |
| **25** | 로봇의 자기 위치를 people_tracker 발행값에 전적으로 의존 | navigation_modifier L47-49 `data.get('robot_x') or 0.0` — 값이 null이면 조용히 (0,0)으로 처리(배터리 NaN 계열과 동일 패턴, [[battery-nan-crash]]) | 🔴 | 별도배치 | 확증(코드경로)/미관측(실제 발생 빈도) | 대기 |
| | 수정 방향 = TF/AMCL에서 로봇 위치를 직접 읽도록 교체(별도 배치, 미착수). | | | | | |
| **26-A** | 초기자세(initial pose) 미설정 상태로 주행을 시작하면 nav2가 37분간 조용히 반쯤 죽는데도 대시보드는 "준비됨"으로 표시 — 오케까지 속을 정도로 아무 표시 안 남음. 시각장애인은 알아챌 방법이 없음 | map 프레임이 없는 상태로 `planner_server`가 activate 진행 중 global_costmap이 TF를 기다리며 블록됨(**확증**, 뿌리B) | 🔴안전 | 준비상태바(DECISIONS.md #10) | 확증 | **실기 근본메커니즘 확증(2026-08-28) — 아래 참고, 물리주행 검증은 별도** |
| | ✅ **실기 검증 기록(2026-08-28, 오케 직접 관측 보고 — 이 세션은 라이브 로봇 상태를 직접 재현·재확인하지 않음, 관측 자체는 **정황**이나 오케가 실기에서 직접 본 1차 보고)**: 런치 24노드 정상 기동, 대시보드 :8080 기동, 초기위치(2D Pose Estimate) **의도적으로 미설정**한 상태에서 `/readiness` 실측 — 측위(amcl)=`status:unknown` "측위 미수신"(age null), nav2=`status:unknown` "측위 활성 / 주행 응답없음(매니저 멈춤 의심)". 오케가 직접 `ros2 service call /lifecycle_manager_navigation/is_active`를 쳐서 타임아웃(무응답) 확인, `lifecycle_manager_localization`의 `is_active`만 `success=True`로 응답 — 즉 초기위치 미설정 → nav2 주행노드가 map/TF 대기로 startup에 갇힘 → 매니저 무응답 → 준비바가 ⚠️(모름)으로 정확히 반응. map TF는 "frame does not exist"로 직접 확인됨. battery=ok(전압판독 정상), elev_app=unknown(미기동), gripper=미구현(기존 계측공백 항목 참고), handle=미수신(아두이노 미연결). **결론**: 옛 대시보드가 이 상황에서 "준비됨"으로 오탐하던 문제(#26-A 핵심 증상)가 새 준비상태 바에서는 재현되지 않음 — amcl "미수신" + nav2 "응답없음"으로 정직하게 표시됨. **단, 이는 준비바가 미측위 상태를 정확히 잡는다는 확인이지, 로봇이 실제로 주행하는 물리 검증은 아님** — 이 시점 베이스 이동은 의도적으로 금지된 상태(안전상 정지 유지). |
| | ✅ **실기 근본메커니즘 확증(2026-08-28, 오케 직접 관측·CLI 조작 1차 보고 — 이 세션은 라이브 로봇을 직접 재현·재확인하지 않음, 아래는 정황이나 오케가 CLI로 직접 유발·관측한 상세 진단)**: 사용자 시나리오 중 costmap이 안 뜨는 문제를 라이브 디버깅. `map_server` active·`/map` 발행 정상, `amcl` active, `scan` 8Hz, `base→laser`·`odom→base_footprint` TF 정상, amcl 파라미터 정상(`tf_broadcast=True`, `base_frame_id=base_footprint`, `scan_topic=scan`) — 그런데 `amcl_pose`는 값이 있는데도 **`map→odom` TF가 없고 `particle_cloud` 미발행** = AMCL 필터가 업데이트를 안 하는 상태. 오케가 CLI로 `/initialpose`를 직접 발행하자 **즉시 `map→odom` TF 생성됨**([-22.834, 4.760]) — AMCL 자체는 정상이고, **RViz "2D Pose Estimate" 클릭이 AMCL에 안 닿는 것**이 사용자가 아무리 초기위치를 찍어도 측위가 안 잡히던 원인. 단, `map→odom`이 생긴 뒤에도 `global_costmap`/`controller`/`planner`가 전부 inactive 유지, `lifecycle_manager_navigation`의 `manage_nodes(STARTUP)` 호출이 **타임아웃(hung)** — 서비스 자체가 무응답. **결론(근본 메커니즘)**: nav 스택이 activation(startup)을 진행하는 시점에 localization이 아직 준비 안 된 채로 걸리면, costmap activation이 멈추고 `lifecycle_manager_navigation`이 그대로 hung 상태로 굳는다 — **#26-A "nav2가 조용히 반쯤 죽는다"의 실기 재현·근본 메커니즘.** 사용자가 말한 "이전엔 잘 됐다"는 그때는 측위 준비 타이밍이 우연히 맞아떨어졌던 것으로 설명됨. **회복 불가 확인**: hung 상태에서 명령으로 revive 시도했으나 안 됐고, 재시작이 필요했음. **별도 신규 이슈**: RViz "2D Pose Estimate"가 `/initialpose`를 AMCL에 전달 못 하는 문제 — CLI로 같은 토픽을 직접 발행하면 정상 동작하므로, AMCL 쪽이 아니라 **RViz 쪽(플러그인 설정/토픽 리매핑/QoS 등)** 경로가 의심됨(미진단, 원인 미확정). 이번 진단은 물리주행 없이 진행됨. |
| | **하위 발견 "#26-B" 🟠확증(뿌리C)**: 이 블록 상태(#26-A)에서 종료 시도 시, transition 진행 중 SIGINT가 들어오면 `planner_server`가 SIGABRT(exit -6)로 죽음. #26-A가 선행조건. | | | | | |
| **27** | 수동주행 중 브라우저 연결이 끊기면(탭닫힘/크래시/와이파이 끊김/노트북 덮개 닫힘) 로봇이 마지막 속도로 **영원히 계속 움직임** | main `_manual_loop()` — `_manual_active`면 0.1초마다 `_manual_cmd` 마지막 값을 계속 발행. `/cmd`(`cmd()` 함수)는 값을 덮어쓰기만 하고 **수신 시각을 전혀 기록하지 않음**(**확증**, 타임스탬프 변수 자체가 없음). 줄번호 참고(커밋 `68c1adf` 기준, HTML 추출 후 재검증 완료): `_manual_loop` L403-412, `cmd()` L439-448 | 🔴안전 | A | 확증(코드경로) | 진단완료·수정설계중 |
| | 처방(code-editor 제안): `/cmd`에 수신시각 기록 + `_manual_loop`에서 `now - last_cmd_t > ~0.5s`면 `publish_cmd(0,0)` + `_manual_active=False`(서버측 데드맨). #2(손잡이 당겨도 계속 감, `navigation_client.cleanup()`)와 증상은 유사하나 경로가 다름(#2=자동주행 취소 경로, #27=수동주행 `_manual_loop` 신규 발견) — 별개 항목 유지. "2a 나이작업과 합칠 후보" = 대시보드 통합 스텝2a(`/readiness` 백엔드, DECISIONS.md #10) — 상태신호에 갱신 타임스탬프를 붙이는 작업이라 #27 처방(수신시각 기록)과 같은 부류. verifier 판정 대기. | | | | | |
| **28** | nav 블랙박스(`robot_diag_nav.py`)가 생성 시점에 조용히 죽으면 nav2 로그가 통째로 안 남는데 아무 경고도 없음 | `robot_diag_nav.py` `main()`의 `logger = DiagLogger("nav")` 생성 호출이 try/except로 감싸이지 않음(**확증**, 직접 대조 — 바로 아래 `boot_snapshot()`만 try로 감싸여 있고 생성자 자체는 무방비). #15(`aeb455f`)에서 main.py `DiagLogger("system")`은 try/except+대체로그로 감쌌는데(main.py `main()`, `except Exception as _le: _log("SYS", "생성 실패...")`) nav 쪽만 그 명세 적용에서 누락됨. 쓰기실패 경고는 `DiagLogger` 내부 공유 로직이라 자동 적용됐고, **생성 예외만** 안 걸림 | 🟡 | A(배치A 후보) | 확증(코드경로) | 대기 |
| **29** | (P4) 라이다 충돌가드 복원 실패가 로그만 남고 강제되지 않던 초기 설계 — 가드 꺼진 채 주행 가능 상태로 방치될 위험 | `main.py` 리스2/3(`62f3b47`) 이전 `_wait_and_grant_authority`/revoke 경로 | 🔴안전 | 리스2/3 | 확증(오케 보고, advisor 설계비평 단계에서 지적) | **✅수정됨(커밋 `62f3b47`)** — revoke 실측검증 2회 강제, 실패 시 여정중단으로 전환 |
| **30** | phantom 하트비트로 인한 재부여 — 대시보드-엘베앱 접촉이 끊겼다 재개될 때 하트비트 흔적이 남아 있으면 리스가 재부여될 위험 | `main.py`/`elevator_button_press/main.py` 리스3/3(`ef2bc6e`) 이전 하트비트 경로 | 🔴안전 | 리스3/3 | 확증(오케 보고, verifier 코드검증 단계에서 지적) | **✅수정됨(커밋 `ef2bc6e`)** — phantom 하트비트 갭 차단(종료 라우트서도 하트비트 정지) |
| **31** | standalone(엘베앱 단독구동) 모드에서 자기 자신이 리스를 회수하는 자기회수 경로 — deadman 설계와 충돌 위험 | `elevator_button_press/main.py` 리스3/3(`ef2bc6e`) 이전 워치독 경로 | 🟠 | 리스3/3 | 확증(오케 보고) | **✅수정됨(커밋 `ef2bc6e`)** — standalone deadman 면제 처리 |
| **32** | 하트비트 자체가 armleft(팔 관련) 스팸을 유발 — #22(costmap_cleaner churn)와 동일 패턴(주기적 프로세스/메시지 churn) | `elevator_button_press/main.py`/`main.py` 리스3/3(`ef2bc6e`) 이전 하트비트 구현 | 🟠 | 리스3/3 | 확증(오케 보고, #22와 동일 패턴으로 식별) | **✅수정됨(커밋 `ef2bc6e`)** — 하트비트 armleft spam 제거 |
| **33** | RViz "2D Pose Estimate"로 초기위치를 찍어도 AMCL이 반응 안 함(측위 안 잡힘) — #26-A 재발의 실제 트리거 | RViz 쪽 경로(플러그인 설정/토픽 리매핑/QoS 등) 의심 — AMCL 자체는 정상(CLI로 `/initialpose` 직접 발행하면 즉시 `map→odom` 생성 확인됨, 2026-08-28 실기) | 🔴안전 | 미배정 | 확증(현상: CLI vs RViz 대조) / 미확정(RViz측 원인) | 대기(미진단) |
| **34** | 런치 재시작 시 driver·lidar만 죽는 "반쪽상태" — 사용자가 UI에서 "정지됨" 확인 후 바로 "시작"을 누르면 새로 뜬 런치의 rplidar/stretch_driver가 곧 죽음 | main.py `sys_proc_ctrl` 정지경로 — 토글 레이스(**확증**, 이 세션 grep 직접 대조: `_sys_procs.pop`은 즉시 실행되어 UI가 바로 "정지됨"으로 바뀌지만, 실제 정리는 `_do_kill`이라는 **백그라운드 스레드**(L1522, `threading.Thread(target=_do_kill, daemon=True).start()` L1574)에서 최대 20초+ 걸려 진행됨. 사용자가 UI만 보고 재시작하면, 새 런치가 뜨는 동안 옛 `_do_kill`이 (a) `pgrep -a rplidar`(L1529, **전역 검색** — 새/옛 프로세스 구분 안 함)로 새로 뜬 rplidar까지 SIGKILL, (b) `stretch_free_robot_process.py` 경로로 새 stretch_driver를 SIGTERM. 카메라는 이 두 정리 경로에 안 걸려있어 생존) | 🔴안전(근본C) | 미배정 | 확증(코드경로: `_do_kill` 비동기·전역 pgrep·auto_free_lock 게이팅 3가지 모두 grep 직접 대조) / 정황(실기 관측: -9/-15 신호·카메라 생존·~10초 지연 4가지가 이 경로로 설명됨 — 오케 1차보고, 이 세션은 라이브 재현 안 함) | 대기(수정설계 있음, 아래 참고) |
| | **수정방향(advisor 제안, 미착수)**: ①정리 진행 중엔 "시작" 버튼 게이트(재시작 금지) ②전역 `pgrep`을 프로세스그룹(pgid) 한정으로 교체 ③재입양(대시보드 재부팅 시 기존 살아있는 런치를 탐지·등록해 고아로 안 만듦) ④preflight에서 `PPID==1` 기준 고아 판정 + stale 락파일 삭제 + 기본 dry-run. **뿌리C**(#4·#11·#12·#22·#26-B와 동일 계열: 프로세스 생명주기 관리 부재) 신규 사례로 분류. | | | | | |
| | ⚠️ **stale 락 확증**: `auto_free_lock`(L1410, `"auto_free_lock": True # 만약 SIGKILL로 강제 종료됐을 때 Stretch filelock 안전망")이 `_do_kill`(정지경로)에만 걸려있어, 대시보드 프로세스 자체가 그냥 죽으면(정지 버튼을 안 거치면) 락이 안 풀림 → stale 락 발생(**확증**, 코드경로 grep 직접 대조). 실기에서 죽은 PID `144809`의 stale 락을 오케가 직접 확인(**정황**, 실기 로그는 오케 1차보고). | | | | | |
| | ↩️ **#23 재검토 권고(advisor, 오늘 관측 기반)**: #23의 원 가설("`stdout=subprocess.PIPE`로 대시보드가 죽으면 자식이 EPIPE/SIGPIPE로 함께 죽는다")이 오늘의 실기 관측과 충돌한다 — 자식 프로세스(interface/vision/diag/elevator)가 각각 54분·21분 생존한 것이 관측됨(**정황**, 오케 1차보고). #34의 토글 레이스가 실제 원인일 가능성이 높아 #23은 **가설 기각 쪽으로 재검토 필요** — 단, #23 자체의 상태 칸은 아직 "정황 — 확증 실험 대기"로 유지(이 세션이 직접 재검증하지 않았으므로 성급히 기각 확정하지 않음). | | | | | |
| | ✅ **clean-slate 확인**: 오늘 실기 진단 종료 시점에 관련 프로세스·락 상태 전부 0(깨끗)으로 확인됨(**정황**, 오케 1차보고). | | | | | |
| **35** | **[수치확증+조치완료]** "사용자가 말한 그 버그" — 프로세스는 전부 살아있는데(launch·stretch_driver·rplidar·realsense 존재) 갑자기 `ros2 node list`=0(daemon 재시작해도), `/scan`·카메라·`map→odom` 데이터 전무. 노드가 서로/CLI에 안 보임. 카메라는 alive+60%CPU인데 0프레임(프리즈) — 디스커버리·데이터 채널 통째 붕괴 | `~/.ros/fastdds_no_shm.xml` — sendBuffer/receiveBuffer=8MB 요구, 주석에 "커널 한도는 `/etc/sysctl.d/99-ros2-udp-buffers.conf`(rmem_max 16MB)와 짝"이라 명시(**확증**, 이 세션이 파일 직접 cat) | 🔴확증(근본C/DDS) | 미배정 | **확증(직접 재현·측정)** | 대기(재시작 후 근본진단 계속 예정) |
| | ✅ **스모킹건 — 이 세션이 직접 재현·측정(2026-08-28)**: `cat /etc/sysctl.d/99-ros2-udp-buffers.conf` → **"그런 파일이나 디렉터리가 없음"**(파일 자체가 없음, xml 주석이 참조하는 짝 파일이 존재 안 함). `sysctl net.core.rmem_max net.core.wmem_max net.core.netdev_max_backlog` 실측 → `rmem_max=212992`(≈208KB), `wmem_max=212992`(≈208KB), `netdev_max_backlog=1000` — xml이 요구하는 16MB의 **약 1/76**. 즉 fastdds 프로필이 8MB 송수신버퍼를 쓰겠다고 선언했는데 커널이 208KB로 막고 있는 미스매치가 **이 세션에서 직접 확증됨**(cat/sysctl 명령 결과 그대로). → 큰 메시지(지도 5MB·포인트클라우드)가 UDP 조각 유실로 손실 → "탐색은 되나 데이터 안 옴" 고장 패턴과 일치. **이것이 SHM을 껐던 원래 이유(2026-07-21 섬 분리·2026-07-24 데이터채널 잠김, `/dev/shm` 손상)와 동일한 실패모드가 UDP 경로에서 재발한 것** — 즉 SHM 회피책이 새 병목(커널 UDP 버퍼)으로 문제를 옮겼을 뿐 근본은 해결 안 됐을 가능성. | | | | | |
| | **맥락(오케 1차보고, 정황)**: 하루 동안 start/stop 반복(#34 토글레이스 사고 포함) 후 이 상태 발생. CPU load 5.35로 과부하 수준 아님. `stretch_driver`는 부모-자식 관계(161619→161877)라 중복 프로세스는 아닌 것으로 보임. fastdds 설정: SHM OFF, UDPv4 only, `data_sharing OFF`, RMW=`rmw_fastrtps_cpp`([[fastdds-no-shm]] 메모리 참고). | | | | | |
| | ✅ **수치확증 — /proc/net/snmp 직접 대조(2026-08-28)**: 6e 세션이 `/proc/net/snmp`의 UDP 통계를 직접 실행해 관측 — **`RcvbufErrors=4,168,844`(약 410만), `InErrors=4,168,844`(동일)** — 수신 소켓버퍼 오버플로로 커널이 버린 UDP 패킷이 410만 건. **이 세션(scribe)도 독립적으로 `/proc/net/snmp` 재실행해 재확인** — 이 시점 값은 `RcvbufErrors=4,189,981`(카운터가 계속 누적 중이라 6e 관측 이후 약 2만 더 증가, 자릿수·패턴 일치, **확증**). IP 계층 `ReasmFails=0` + `ipfrag_high_thresh=4194304`(4MB) → 손실 지점이 **IP 재조립이 아니라 UDP 소켓버퍼(rmem_max 208KB) 단계**임을 수치로 특정. `/dev/shm` 항목 0개도 이 세션이 `ls /dev/shm` 직접 실행해 재확인(**확증**) — SHM 문제 아님 재확인. | | | | | |
| | ↩️ **stretch_driver "중복 프로세스" 의혹 정정**: 관측된 2개 프로세스(161619→161877)는 부모-자식 관계이며 같은 params-file을 쓰는 **정상 구조** — advisor의 "중복 의심"은 정정됨(오케 1차보고, 정황). | | | | | |
| | ✅ **advisor 진단**: "디스커버리 기아(discovery starvation)" — 대용량 데이터(지도·포인트클라우드)가 작은 소켓버퍼(208KB)를 채워 디스커버리 패킷까지 함께 유실되면서 `ros2 node list`가 0으로 보이는 현상까지 설명됨. CPU load 5.35의 "한가함"은 과부하가 아니라 **패킷을 계속 버리는 중이라 조용한 것**으로 재해석. | | | | | |
| | 🔧 **조치안(advisor 근본책 1순위) → ✅ 적용 완료(2026-08-28)**: `/etc/sysctl.d/99-ros2-udp-buffers.conf` 신설 — `rmem_max`/`wmem_max`=16MB, `ipfrag_high_thresh`=128MB, `ipfrag_time`=3 → `sysctl --system` 적용. **이 세션이 직접 재확인**: `sysctl net.core.rmem_max net.core.wmem_max` → `16777216`(16MB) 그대로 반영, 파일도 `cat`으로 직접 대조(값 일치, **확증**). `rmem_default`는 건드리지 않음(전역 영향 회피). SHM 재활성화는 **반대**(2026-07-21/07-24 `/dev/shm` 손상 고장 이력 재발 우려, [[fastdds-no-shm]] 참고). **주의**: 소켓버퍼는 프로세스 생성 시점에 결정되므로 전체 재시작 이후 노드들부터 적용됨 — 재시작 전 뜬 노드에는 소급 미적용. | | | | | |
| | **상태 갱신**: #35 = **수치확증 + 조치 적용 완료.** 재발 여부는 이후 운영에서 관찰 필요(재발 시 별도 기록). | | | | | |
| **36** | 수동으로 엘베 제어권 토글(대시보드 "🔑 엘베 제어권" 스위치, DECISIONS.md #11 리스2/3)로 켰을 때, 실기 시나리오(탑승/층선택/문대기 버튼) 도중 6초마다 제어권이 갑자기 회수돼 로봇이 멈춤 | `main.py` `/elev_authority` POST 라우트 — 이전엔 `_set_elev_authority`를 직접 호출해 하트비트 renewer(`_lease_renewer`)를 안 켰음(renewer는 자동여정 경로 `_grant_elev_lease`에서만 시작되던 구조). 하트비트가 안 오니 엘베앱 deadman(리스3/3 `ef2bc6e`, TTL 6s)이 정상 작동하며 6초마다 제어권을 회수 — **리스3/3(deadman) 자체의 버그가 아니라, 수동-grant 경로가 deadman의 하트비트 계약을 안 지킨 사각지대였음** | 🔴안전 | 리스 계열(#11) | 확증(코드경로 diff 직접 대조) | **✅수정됨(커밋 `7c382a3`) — 실기 재현 후 verifier 통과** |
| | **수정**: `/elev_authority` POST가 `_set_elev_authority` 직접호출 대신 `_grant_elev_lease`를 경유하도록 변경 — 수동 grant도 자동여정과 동일하게 2초 하트비트로 유지됨(**확증**, 이 세션이 `git show 7c382a3` 직접 대조: `main.py` 6줄, +4/-2). | | | | | |
| | ↩️ **DECISIONS.md #11 연결**: 리스3/3(deadman, `ef2bc6e`)이 설계 의도대로 정확히 작동한 것이 오히려 이 버그를 드러냄 — deadman 자체는 정상, 계약(하트비트)을 맺는 경로 하나가 누락됐던 것. 실기 테스트가 정적 검증(코드리뷰)으로는 못 잡는 "경로 사각지대"를 찾아낸 사례. | | | | | |
| | ⚠️ **#29~32 상태 표기 참고**: 4건 모두 이 세션 기록 시점엔 이미 대응 커밋(`62f3b47`/`ef2bc6e`)에 반영 완료 — "설계 단계에서 이중비평이 잡아낸 뒤 곧바로 수정된" 항목이라 별도 "대기" 상태 없이 완료로 기록. 물리(주행) 검증은 **미실시**(사용자 예정) — 코드 수정 확증이지 실주행 확증 아님. | | | | | |

## 뿌리 3개
- **A. 실패·취소 시 안전상태로 못 돌아감** (1·2·3·5·6·9·27) — 반환값 안 보고 이동으로 진행. 물리안전 4건.
- **B. 실패가 사용자에게 전달 안 됨** (1·L·6·10·24·26-A) — 틀린 곳/층에 "도착" 선언, 또는 아무 표시 없이 조용히 반쯤 죽음. 못 알아챔.
- **C. 프로세스 생명주기 관리 부재** (4·11·12·22·26-B·34·35) — 좀비·고아·유령 참가자 + DDS churn + 토글 레이스 + UDP 버퍼 미스매치.

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

**🟡 계측 공백(미배정 번호)**: main.py의 attach 호출에 `cameras=` 인자가 없어 대시보드가 카메라 상태를 전혀 감시하지 못함(수정 4줄 규모). 손잡이(아두이노) 연결 끊김 시 3초 간격 무한 재시도가 로그를 스팸함. → 2a-2(커밋 `8b71674`) 시점까지도 미구현 확인됨(DECISIONS.md #10 진행메모 참고).

**🟡 2a 구현 중 재확인된 공백(2026-08-28, DECISIONS.md #10 2a-2 참고)**:
- `is_active`(nav2 준비신호)는 "활성화됨"만 확인 — 활성화 후 내부에서 멈춘 상태(#26-A류)는 못 잡음. #26-A 완전 대응 아님. 진짜 서명 후보는 map→odom TF 확인이나 UDP 비용 실측이 먼저 필요해 별건 대기.
- 측위 준비신호가 `/amcl_pose`만 구현됨 — DECISIONS.md #10 설계가 명시한 "측위(`/amcl_pose` + map→odom)" 중 map→odom 절반 미구현.

**🟡 리스 리팩터(DECISIONS.md #11) 중 열어둔 항목(2026-08-28, 미배정 번호, 오케 보고)**:
- twist_mux — 진짜 `cmd_vel` DDS 중재. 엘베앱은 여전히 `/stretch/cmd_vel` 직접 퍼블리시(리스 정책은 소프트웨어 정책 수준, 커널/DDS 수준 중재 아님). 항목 8·DECISIONS.md #11 분석 참고.
- UDP/SHM 문제 — 별개 트랙, [[fastdds-no-shm]] 참고.
- 자동기동 lazy start — 측정 후 재검토.
- 측위 map→odom TF — 별건(위 2a 공백과 동일 항목).
- 실패 경로에서 약 9~10초 구간 UI에 "처리중" 표시 없음 — 경미 UX, 안전 등급 아님.

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
- **nav2 bond**: 7개 전부 active 확인(`controller_server`·`planner_server`·`behavior_server`·`bt_navigator`·`waypoint_follower`·`map_server`·`amcl` — DECISIONS.md #10 정정 참고)
