Hello robot 최종 기능 명세서
최종 기능 명세서 v3
1. 목적
이 시스템은 Stretch 로봇의 안내 인터페이스를 담당한다.
역할:
•	버튼, pull, 마이크 입력 처리
•	TTS, beep, 수음 게이트 제어
•	GPT 기반 의도 해석
•	ROS 2 상태머신 및 네비게이션 연동
원칙:
•	GPT는 의도 해석만 한다
•	최종 상태 전이는 코드가 한다
•	최종 확정은 항상 버튼이다
 
2. 상태 집합
상태는 아래 4개만 사용한다. 업로드된 상태머신도 동일한 4상태를 사용한다.
•	LOCKED
•	READY
•	NAV
•	PAUSED
2.1 LOCKED
기본 대기 상태.
규칙:
•	초기 상태는 반드시 LOCKED
•	마이크 비활성
•	버튼만 유효
•	pull 무시
•	text 무시
•	pending confirm 없음
2.2 READY
목적지 입력 및 출발 확인 상태.
규칙:
•	마이크 활성 가능
•	GPT 호출 가능
•	목적지 입력, 후보 재질문, 출발 확인 담당
READY 내부 하위 국면:
•	READY_DEST_INPUT
•	READY_DISAMBIGUATE
•	READY_CONFIRM
2.3 NAV
실제 이동 상태.
규칙:
•	마이크 비활성
•	GPT 호출 금지
•	button 또는 pull 입력 시 즉시 정지 절차 후 PAUSED
•	도착 시 LOCKED
•	start 실패 시 NAV로 가지 않고 READY 유지
현재 상태머신은 start 성공 시 NAV, 실패 시 READY 유지, 도착 시 LOCKED, pause 시 PAUSED로 처리한다.
2.4 PAUSED
이동 중단 상태.
규칙:
•	마이크 활성 가능
•	GPT 호출 가능
•	재개/종료/변경 의도 해석 담당
•	기존 목적지 정보 유지
PAUSED 내부 하위 국면:
•	PAUSED_INTENT_INPUT
•	PAUSED_CONFIRM
 
3. 하위 국면 정의
하위 국면은 같은 상태 안의 세부 단계다.
예:
•	state = READY, phase = READY_DEST_INPUT
•	state = READY, phase = READY_CONFIRM
•	state = PAUSED, phase = PAUSED_INTENT_INPUT
하위 국면은 필수다.
같은 READY 안에서도 “목적지 듣는 중”과 “출발 확인 대기 중”은 버튼과 음성 의미가 다르기 때문이다.
 
4. PendingConfirm
확인 절차는 상태가 아니라 PendingConfirm 객체로 관리한다. 현재 업로드된 interface.py도 PendingConfirm(action, destination, deadline) 구조를 사용한다.
필드:
•	action: str
•	destination: Optional[str]
•	deadline: float
허용 action:
•	start
•	resume
•	abort
•	change_ready
•	change_start
의미:
•	start: READY에서 출발 확인 대기
•	resume: PAUSED에서 원래 목적지 재개 확인 대기
•	abort: 종료 확인 대기
•	change_ready: 목적지 변경 입력 모드 진입 확인 대기
•	change_start: 새 목적지로 변경 출발 확인 대기
 
5. 하드웨어
입력 장치:
•	버튼 1개
•	pull 센서 1개
•	마이크 1개
출력 장치:
•	스피커 1개
•	시작 beep 1종
•	종료 beep 1종
입력 의미:
•	버튼: 상태 진입 또는 확인 확정
•	pull: NAV 중 정지 요청
•	마이크: 목적지/의도 발화 입력
 
6. 숫자 파라미터
아래 값으로 고정한다.
BUTTON_GUARD_SEC = 0.8
PULL_GUARD_SEC = 1.0

LISTEN_OPEN_DELAY_SEC = 0.35
LISTEN_TIMEOUT_SEC = 10.0
LISTEN_PHRASE_TIME_LIMIT_SEC = 10.0
LISTEN_PAUSE_THRESHOLD_SEC = 0.8
DEDUP_WINDOW_SEC = 2.2

NO_RESPONSE_SESSION_SEC = 10.0
NO_RESPONSE_MAX_SESSIONS = 3
NO_RESPONSE_TOTAL_SEC = 30.0
REMINDER_MAX_COUNT = 2

READY_RETRY_MAX = 3
PAUSED_RETRY_MAX = 3
CONFIRM_TIMEOUT_SEC = 30.0

START_LISTEN_BEEP_HZ = 1350
START_LISTEN_BEEP_MS = 110
END_LISTEN_BEEP_HZ = 900
END_LISTEN_BEEP_MS = 90

NAV_STOP_PUBLISH_SEC = 1.0
TURN_THRESHOLD_RAD_PER_SEC = 0.4
TURN_ANNOUNCE_MIN_INTERVAL_SEC = 5.0

TTS_MAX_CHARS = 120
현재 코드에는 시작 beep 1350Hz/110ms, 종료 beep 900Hz/90ms, 회전 임계값 0.4rad/s, 회전 안내 최소 간격 5초가 이미 구현돼 있다.
 
7. 목적지 후보 규칙
목적지 정답 후보 목록은 config/location.yaml의 locations 키 전체다. NavigationClient도 data['locations'].get(self.target_key)로 목적지를 읽는다.
규칙:
•	alias 체계는 사용하지 않는다
•	YAML key 자체를 공식 목적지 이름으로 사용한다
•	GPT에는 공식 목적지 목록 전체를 전달한다
•	GPT가 반환한 destination은 반드시 이 목록 안에 있어야 한다
•	목록 밖 목적지는 유효하지 않다
 
8. 오디오/수음 절대 순서
현재 MicTextBridge는 AudioGate가 열려야 듣고, 처음 열릴 때만 시작 beep를 내고, 수집이 끝나면 종료 beep를 낸다.
모든 수음 세션은 반드시 아래 순서를 따른다.
1.	TTS 재생
2.	TTS 완전 종료
3.	AudioGate 열림 확인
4.	0.35초 대기
5.	시작 beep 재생
6.	수음 시작
7.	수음 종료
8.	종료 beep 재생
9.	STT 처리
금지:
•	TTS 중 수음 금지
•	beep 중 수음 금지
•	리마인드 중 수음 금지
•	NAV 상태 수음 금지
8.1 리마인드 절차
리마인드가 필요한 경우 반드시 아래 순서만 허용한다.
1.	현재 수음 세션 종료
2.	마이크 닫힘 유지
3.	리마인드 TTS 재생
4.	리마인드 TTS 종료
5.	AudioGate 열림 확인
6.	0.35초 대기
7.	시작 beep 재생
8.	다음 수음 세션 시작
즉, 리마인드 직전과 리마인드 중에는 수음이 절대 되면 안 된다.
 
9. STT 처리 규칙
현재 interface.py에는 짧은 텍스트/잡음 필터가 있고 "응", "네", "예", "아니" 등을 버린다. 이 규칙은 제거해야 한다.
9.1 제거
제거할 것:
•	최소 글자 수 필터
•	잡음 단어 목록 필터
9.2 유지
유지할 것:
•	동일 텍스트 중복 차단만 유지
•	중복 차단 윈도우 2.2초
9.3 처리
•	STT 결과가 빈 문자열이면 인식 실패
•	UnknownValueError도 인식 실패
•	인식 실패는 GPT에 보내지 않는다
•	짧은 텍스트라도 비어 있지 않으면 GPT로 보낸다
허용 예:
•	"네"
•	"예"
•	"응"
•	"아니요"
•	"맞아요"
•	"1동"
 
10. 무응답과 재질문
10.1 무응답
무응답은 10초 수음 세션 동안 유효 발화가 없는 경우다.
총 3세션 사용:
•	1차 수음 10초
•	리마인드 1
•	2차 수음 10초
•	리마인드 2
•	3차 수음 10초
•	그래도 무응답이면 종료
즉:
•	총 수음 기회 3회
•	총 수음 시간 30초
•	리마인드 2회
10.2 재질문
재질문은 다음 경우다.
•	인식 실패
•	목적지 불명확
•	후보 복수
•	PAUSED 의도 불명확
retry count:
•	READY retry count와 PAUSED retry count를 분리한다
•	각 상태 진입 시 0으로 초기화한다
•	최대 3회
•	3회 초과 시 종료
10.3 무응답과 인식 실패의 차이
•	무응답: 수음 세션 1회를 소모한다
•	인식 실패: retry count를 1 증가시킨다
•	둘은 같은 것으로 취급하지 않는다
 
11. 입력 무시 규칙
LOCKED
•	button: 유효
•	pull: 무시
•	text: 무시
READY_DEST_INPUT
•	button: 무시
•	pull: 무시
•	text: 유효
READY_DISAMBIGUATE
•	button: 무시
•	pull: 무시
•	text: 유효
READY_CONFIRM
•	button: 유효
•	pull: 무시
•	text: 유효
NAV
•	button: 유효
•	pull: 유효
•	text: 무시
PAUSED_INTENT_INPUT
•	button: 유효
•	pull: 무시
•	text: 유효
PAUSED_CONFIRM
•	button: 유효
•	pull: 무시
•	text: 유효
 
12. GPT 역할 및 형식
현재 planner는 이미 strict JSON schema 형태를 사용하고 있다.
12.1 GPT 역할
GPT는 아래만 한다.
•	목적지 후보 추론
•	복수 후보 정리
•	재개/종료/변경 의도 분류
•	재질문 필요 여부 판단
12.2 GPT 금지
GPT는 아래를 하지 않는다.
•	상태 변경
•	출발 확정
•	종료 확정
•	재개 확정
12.3 GPT 출력 스키마
GPT는 반드시 아래 JSON만 반환한다.
{
  "action": "noop | propose | ask_again | disambiguate | resume_propose | abort_propose | change_dest_request | change_dest_propose",
  "destination": "string | null",
  "candidates": ["string"],
  "need_button": true,
  "button_action": "none | start | resume | abort | change_ready | change_start",
  "confidence": "high | medium | low",
  "reason": "none | out_of_list | ambiguous | low_confidence | stt_failed"
}
12.4 action 허용 범위
READY에서 허용:
•	propose
•	ask_again
•	disambiguate
PAUSED에서 허용:
•	resume_propose
•	abort_propose
•	change_dest_request
•	change_dest_propose
•	ask_again
cancel_confirm는 사용하지 않는다. 스키마에서 제거한다.
12.5 후보 수 규칙
•	후보 1개: propose
•	후보 2~3개: disambiguate
•	후보 4개 이상: ask_again
•	confidence low: ask_again
12.6 reason 규칙
•	out_of_list: 목록 밖 목적지
•	ambiguous: 의미는 있으나 후보 다수 또는 불명확
•	low_confidence: 확신 부족
•	stt_failed: STT 실패
•	none: 해당 없음
 
13. 고정 TTS 문구
GPT 자유문장을 직접 TTS로 사용하지 않는다.
아래 문구만 사용한다.
1.	LOCKED → READY
"버튼이 눌렸습니다. 어디로 가실 건가요?"
2.	READY 재질문
"목적지를 다시 짧게 말씀해 주세요."
3.	READY 목록 밖
"현재 갈 수 있는 장소가 아닙니다. 다시 말씀해 주세요."
4.	READY 복수 후보
"후보가 여러 개 있습니다. {후보1}, {후보2}, {후보3} 중 어디로 가실까요?"
5.	READY 단일 후보 확인
"{장소}로 이동할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."
6.	READY 음성 긍정 후
"확인했습니다. 출발하려면 버튼을 눌러주세요."
7.	READY 음성 부정 후
"알겠습니다. 목적지를 다시 말씀해 주세요."
8.	NAV → PAUSED
"일시정지되었습니다. 문제가 있으신가요?"
9.	PAUSED 재개 확인
"다시 원래 목적지로 이동할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."
10.	PAUSED 종료 확인
"안내를 종료할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."
11.	PAUSED 변경 확인
"목적지를 변경할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."
12.	PAUSED 새 목적지 변경 확인
"{장소}로 목적지를 변경할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."
13.	변경 모드 진입 후
"어디로 변경할까요? 목적지를 말씀해 주세요."
14.	출발 실패
"안내를 시작하지 못했습니다. 다시 시도해 주세요."
15.	이동 중 실패
"안내를 계속할 수 없어 중지했습니다. 필요하면 버튼을 눌러 다시 시작해 주세요."
16.	도착
"목적지에 도착했습니다."
17.	무응답 종료
"응답이 없어 안내를 종료합니다."
18.	재질문 초과 종료
"안내를 종료합니다. 필요하면 버튼을 눌러 다시 시작해 주세요."
19.	리마인드 1
"말씀해 주세요."
20.	리마인드 2
"필요하시면 말씀해 주세요."
21.	PAUSED ask_again
"종료, 변경, 다시 이동 중 하나로 말씀해 주세요."
22.	PAUSED 음성 부정 후
"알겠습니다. 원하시는 내용을 말씀해 주세요."
23.	PAUSED 음성 긍정 후
"확인했습니다. 진행하려면 버튼을 눌러주세요."
 
14. 상태 전이 규칙
14.1 LOCKED
button
1.	LOCKED -> READY
2.	phase = READY_DEST_INPUT
3.	pending 삭제
4.	READY retry count = 0
5.	TTS #1
6.	문장 종료 후 beep
7.	수음 세션 1 시작
pull
•	무시
text
•	무시
 
14.2 READY
READY_DEST_INPUT에서 text
GPT에 공식 목적지 목록과 사용자 텍스트를 전달한다.
action = propose
조건:
•	목적지 1개
•	공식 목록 안에 있음
동작:
1.	pending = PendingConfirm("start", destination, now + 30.0)
2.	phase = READY_CONFIRM
3.	TTS #5
4.	문장 종료 후 beep
5.	confirm deadline 시작
action = disambiguate
조건:
•	후보 2~3개
동작:
1.	phase = READY_DISAMBIGUATE
2.	READY retry count += 1
3.	TTS #4
4.	문장 종료 후 beep
5.	수음 시작
action = ask_again
동작:
1.	READY retry count += 1
2.	phase 유지
3.	reason이 out_of_list면 TTS #3
4.	그 외에는 TTS #2
5.	문장 종료 후 beep
6.	수음 시작
READY retry count > 3
1.	READY -> LOCKED
2.	pending 삭제
3.	TTS #18
READY_DISAMBIGUATE에서 text
•	READY_DEST_INPUT과 동일 규칙 적용
•	button 무시
•	pull 무시
READY_CONFIRM에서 text
음성 긍정
예:
•	네
•	예
•	응
•	맞아요
동작:
1.	pending 유지
2.	TTS #6
3.	문장 종료 후 beep
4.	confirm deadline을 다시 30초로 리셋
5.	버튼 대기 유지
음성 부정
예:
•	아니요
•	아니
•	틀렸어요
동작:
1.	pending 삭제
2.	phase = READY_DEST_INPUT
3.	TTS #7
4.	문장 종료 후 beep
5.	수음 시작
음성 수정
예:
•	아니요 2동이요
•	편의점 말고 1동이요
동작:
1.	pending 삭제
2.	새 텍스트를 READY_DEST_INPUT처럼 다시 GPT 처리
READY_CONFIRM에서 button
조건:
•	pending.action == start
동작:
1.	start_navigation(destination) 호출
2.	성공 시 READY -> NAV
3.	실패 시 READY 유지
4.	실패 시 TTS #14
현재 상태머신도 start 실패 시 READY에 남는다.
READY confirm timeout
기준:
•	마지막 confirm 관련 TTS 종료 후
•	beep 직후
•	30초 경과
동작:
1.	pending 삭제
2.	READY -> LOCKED
3.	TTS #17
READY 무응답 30초
동작:
1.	READY -> LOCKED
2.	pending 삭제
3.	TTS #17
 
14.3 NAV
button 또는 pull
동작 순서는 반드시 아래와 같다.
1.	stop publish 요청 1.0초
2.	nav goal cancel 요청
3.	NAV -> PAUSED
4.	phase = PAUSED_INTENT_INPUT
5.	PAUSED retry count = 0
6.	TTS #8
7.	문장 종료 후 beep
8.	수음 시작
현재 상태머신도 NAV에서 PAUSED로 가면서 stop publish와 cancel을 수행한다.
arrive
1.	NAV -> LOCKED
2.	last_target_key = None
3.	TTS #16
현재 상태머신도 도착 시 LOCKED로 복귀한다.
nav_runtime_failed
정의:
•	NavigationClient가 goal을 보내기 전 실패한 경우는 start 실패다
•	nav_runtime_failed는 이미 NAV 상태에 들어간 뒤 아래 중 하나가 발생한 경우다:
o	cancel/cleanup 강제 발생
o	nav node 예외
o	외부 로직이 “계속 안내 불가” 이벤트를 보냄
동작:
1.	NAV -> LOCKED
2.	cleanup 수행
3.	TTS #15
text
•	무시
 
14.4 PAUSED
PAUSED_INTENT_INPUT에서 button
1.	pending = PendingConfirm("resume", last_target_key, now + 30.0)
2.	phase = PAUSED_CONFIRM
3.	TTS #9
4.	문장 종료 후 beep
5.	confirm deadline 시작
PAUSED_INTENT_INPUT에서 pull
•	무시
PAUSED_INTENT_INPUT에서 text
GPT 결과에 따라 분기한다.
action = resume_propose
1.	pending = PendingConfirm("resume", last_target_key, now + 30.0)
2.	phase = PAUSED_CONFIRM
3.	TTS #9
4.	문장 종료 후 beep
5.	confirm deadline 시작
action = abort_propose
1.	pending = PendingConfirm("abort", null, now + 30.0)
2.	phase = PAUSED_CONFIRM
3.	TTS #10
4.	문장 종료 후 beep
5.	confirm deadline 시작
action = change_dest_request
1.	pending = PendingConfirm("change_ready", null, now + 30.0)
2.	phase = PAUSED_CONFIRM
3.	TTS #11
4.	문장 종료 후 beep
5.	confirm deadline 시작
action = change_dest_propose
1.	pending = PendingConfirm("change_start", destination, now + 30.0)
2.	phase = PAUSED_CONFIRM
3.	TTS #12
4.	문장 종료 후 beep
5.	confirm deadline 시작
action = ask_again
1.	PAUSED retry count += 1
2.	TTS #21
3.	문장 종료 후 beep
4.	수음 시작
PAUSED retry count > 3
1.	PAUSED -> LOCKED
2.	TTS #18
PAUSED_CONFIRM에서 text
음성 긍정
1.	pending 유지
2.	TTS #23
3.	문장 종료 후 beep
4.	confirm deadline을 다시 30초로 리셋
5.	버튼 대기 유지
음성 부정
1.	pending 삭제
2.	phase = PAUSED_INTENT_INPUT
3.	TTS #22
4.	문장 종료 후 beep
5.	수음 시작
음성 수정
1.	pending 삭제
2.	새 텍스트를 PAUSED_INTENT_INPUT처럼 다시 GPT 처리
PAUSED_CONFIRM에서 button
pending.action == resume
1.	PAUSED -> READY
2.	기존 목적지로 start 호출 준비
3.	성공 시 NAV
4.	실패 시 READY 유지
5.	실패 시 TTS #14
현재 상태머신도 resume은 READY를 거쳐 다시 start한다.
pending.action == abort
1.	PAUSED -> LOCKED
2.	last_target_key = None
3.	stop publish 1.0초
4.	cancel goal
5.	cleanup
6.	TTS "안내를 종료합니다."
현재 상태머신도 abort 시 target을 지우고 LOCKED로 간다.
pending.action == change_ready
1.	pending 삭제
2.	PAUSED -> READY
3.	phase = READY_DEST_INPUT
4.	READY retry count = 0
5.	TTS #13
6.	문장 종료 후 beep
7.	수음 시작
8.	last_target_key는 유지한다
pending.action == change_start
1.	last_target_key = 새 목적지
2.	PAUSED -> READY
3.	새 목적지 start 호출
4.	성공 시 NAV
5.	실패 시 READY 유지
6.	실패 시 TTS #14
PAUSED confirm timeout
1.	pending 삭제
2.	PAUSED -> LOCKED
3.	TTS #17
PAUSED 무응답 30초
1.	PAUSED -> LOCKED
2.	TTS #17
 
15. Navigation 연동 규칙
NavigationClient.start_navigation()은 YAML에서 좌표를 읽고, costmap clear를 시도하고, action server를 최대 5초 기다린 뒤 goal을 전송한다. 좌표가 없거나 서버 대기가 실패하면 False를 반환한다. GoalStatus.STATUS_SUCCEEDED면 is_arrived=True가 된다.
규칙:
•	destination key는 반드시 YAML key여야 한다
•	좌표가 없으면 start 실패
•	action server 5초 내 준비 안 되면 start 실패
•	start 실패 시 READY 유지
•	도착 시 LOCKED
•	pause/abort 시 stop publish 1.0초와 cancel 수행
 
16. 회전 안내 규칙
현재 navigation_client.py는 /stretch/cmd_vel의 angular.z를 보고 회전을 감지한다. 임계값은 0.4rad/s, 중복 안내 최소 간격은 5초다.
규칙:
•	abs(angular.z) > 0.4면 회전 안내 가능
•	안내 간 최소 간격 5.0초
•	angular.z > 0: "왼쪽으로 회전합니다."
•	angular.z < 0: "오른쪽으로 회전합니다."
•	NAV 상태에서만 허용
•	상태 전이는 일으키지 않음
