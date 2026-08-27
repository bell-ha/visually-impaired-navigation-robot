# 시각장애인 안내 로봇 (visually-impaired-navigation-robot)

Stretch3 로봇으로 시각장애인을 안내하고, 엘리베이터를 자율 탑승해 층간 이동하는 시스템. 논문화 목표(사회적 마찰 novelty).

## 아키텍처 (현재)
- **대시보드** `src/blind_nav_system/blind_nav_system/main.py` (Flask :8080) — 운영자 관제 + 엘베 자동여정 오케스트레이터(`_auto_run`)
- **음성 인터페이스** `interface.py` (1937줄) — 시각장애인용 음성 STT/TTS + `GuidanceStateMachine` + 아두이노 손잡이
- **엘베 앱** `elevator_button_press/main.py` (Flask :5000) — 그리퍼 카메라 OCR로 버튼 인식·누르기
- **주행** `navigation_client.py`(Nav2 래퍼) + `navigation_modifier.py` + nav2 (fork: bell-ha/human-nav, `nav2_params_human.yaml`)
- 스크립트 프로젝트 (colcon 아님, python3 직접 실행)

## 하드웨어 핵심
- D435(몸체, USB3, `/camera/camera/`) + D405(그리퍼, USB2 딥허브, `image_rect_raw`만 발행) — 시리얼 고정 필수
- FastDDS SHM 비활성 → UDP 강제 (`~/.ros/fastdds_no_shm.xml`). 유령 참가자 주의.
- 배터리 pct=NaN 자주 → **voltage 기준 판단** (과거 `round(NaN)`이 spin 스레드 죽임)
- 뎁스 포인트클라우드가 UDP로 무거워 nav 멈춤 유발 (transform 캐시 드롭)

## 알려진 핵심 문제 = 뿌리 3개 (상세: `.claude/FINDINGS.md`)
- **A. 실패·취소 시 안전복귀 실패** — 반환값 안 보고 이동으로 진행 (물리안전)
- **B. 실패가 사용자에 전달 안 됨** — 틀린 곳/층에 "도착" 선언 (시각장애인이 못 알아챔)
- **C. 프로세스 생명주기** — 좀비·고아·유령 참가자

## 작업 순서 원칙
**버그 먼저 → 리팩터 나중.** 리팩터(God파일 분해·폴더구성·오케 통합)는 버그 잡은 뒤.

## 클로드 팀 (멀티 에이전트) — 5역할 고정
- 🧭 **오케스트레이터** (메인 세션) — 사용자와 구체 의논·계획, 모든 요청의 허브, 객관 판단. 코드 안 짬.
- 🤔 **advisor** (opus) — 오케의 명령·계획·방향을 전담으로 염려·조언 (실행 前 비평). +구조·뿌리 의심. 읽기전용.
- ✍️ **code-editor** (opus) — 명령대로 코드 정확히 수정.
- 🔍 **verifier** (opus) — 새 버그 발굴 + 수정 검증 (읽기전용). 안 고침.
- 📝 **scribe** (sonnet) — 우리 과정·고충·로그·결정·FINDINGS 기록·영속 (실시간 모니터링 대신 사후 검토).

**규칙:** 허브 앤 스포크 — 모두 오케스트레이터에게 보고, 사용자와는 오케만 소통. 도구 경계로 역할 강제(advisor·verifier는 Edit 없음). **이중 비평:** advisor=계획 비평(실행 前) / verifier=코드 비평(실행 後).

## 지켜야 할 규칙
- **커밋에 `Co-Authored-By: Claude` 트레일러 금지.**
- **물리 안전 최우선** — 사람 옆에서 움직이는 로봇.
- **수정 전 설명·승인** — 사용자는 관찰 우선, 큰 변경은 승인받고. 런치 재시작 최소화.
- 체크포인트 태그 `checkpoint-before-arch-changes` 존재 (되돌리기 지점).
