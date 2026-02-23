## 1) 상태 다이어그램 (ASCII)

```
                 (button)
   ┌───────────┐  start   ┌───────────┐
   │  LOCKED   │─────────▶│   READY   │
   └───────────┘          └───────────┘
        ▲                      │
        │                      │ (GPT: propose + "버튼으로 확인")
        │                      │ + (button = confirm start)
        │                      ▼
        │                  ┌───────────┐
        │                  │    NAV    │
        │                  └───────────┘
        │                      │
        │                      │ (pull)
        │                      ▼
        │                  ┌───────────┐
        │                  │  PAUSED   │
        │                  └───────────┘
        │                      │
        │                      │ (button -> resume 제안)
        │                      │ + (button = confirm resume)
        │                      ▼
        └──────────────────── NAV


[도착 시나리오]
NAV --(arrival_sec 지나거나 /arrive)--> LOCKED

[종료 시나리오(2단계)]
NAV --(button)--> (pending abort confirm) --(button)--> LOCKED
PAUSED --(GPT가 abort_propose or 사용자가 버튼 흐름)--> (pending abort confirm) --(button)--> LOCKED
```

---

## 2) “이벤트별” 상태 전이 표

### 이벤트: `button`

| 현재 상태            | pending(confirm) 있음? | 결과 상태                       | 로봇 멘트(요약)              |
| ---------------- | -------------------: | --------------------------- | ---------------------- |
| LOCKED           |                    X | READY                       | “어디로 안내해 드릴까요?”        |
| READY            |                    X | READY(유지)                   | “목적지를 말씀해 주세요”         |
| NAV              |                    X | NAV(유지) + pending=abort     | “종료할까요? (한 번 더 버튼)”    |
| PAUSED           |                    X | PAUSED(유지) + pending=resume | “다시 이동할까요? (한 번 더 버튼)” |
| READY            |        start pending | NAV                         | “확인했습니다. (목적지)로 시작”    |
| PAUSED           |       resume pending | NAV                         | “확인했습니다. 다시 이동”        |
| NAV/PAUSED/READY |        abort pending | LOCKED                      | “확인했습니다. 안내 종료”        |

---

### 이벤트: `pull`

| 현재 상태               | 결과 상태  | 로봇 멘트                |
| ------------------- | ------ | -------------------- |
| NAV                 | PAUSED | “멈췄습니다. 무슨 일 있으신가요?” |
| READY/LOCKED/PAUSED | 변화 없음  | (없음)                 |

---

### 이벤트: `text` (사용자 채팅 입력)

| 현재 상태      | 처리                                                                               |
| ---------- | -------------------------------------------------------------------------------- |
| READY      | GPT에게 “장소 추론” 요청 → propose/list/ask_again                                        |
| PAUSED     | GPT에게 “재개/변경/종료” 같은 의도 파악 요청(현재 스키마는 resume_propose/change_dest/abort_propose 등) |
| NAV/LOCKED | 기본적으로 GPT 호출은 가능하지만, 현재 코드는 주로 READY에서 의미있게 동작                                   |

---

### 이벤트: `arrive`

| 현재 상태 | 결과 상태  | 로봇 멘트            |
| ----- | ------ | ---------------- |
| NAV   | LOCKED | “(목적지)에 도착했습니다.” |

---

## 3) 코드 관점 “FSM 정의” (요약)

* 상태 집합:

  * `LOCKED`: 대기
  * `READY`: 목적지 대화/확인
  * `NAV`: 이동중(시뮬)
  * `PAUSED`: 정지(당김)

* “확인 절차”는 상태가 아니라 **PendingConfirm** 로 관리됨

  * `PendingConfirm(action=start/resume/abort, destination, deadline)`
  * 즉, **버튼이 누르는 순간**

    * pending이 있으면 “확인”으로 쓰이고
    * pending이 없으면 “제안(confirmation 요청)”을 생성함
