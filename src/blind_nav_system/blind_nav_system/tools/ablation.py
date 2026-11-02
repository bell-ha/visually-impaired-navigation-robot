#!/usr/bin/env python3
"""절제(ablation) 실험 조건 전환 — "뒤에 매인 사용자" 제약을 하나씩 끈다.

왜 이게 필요한가
  주장이 "안내받는 사람이 로봇 뒤에 매여 있다는 제약이 설계를 다시 쓰게 만든다"면,
  증명해야 할 것은 "남의 로봇보다 낫다"가 아니라 **"그 설계를 끄면 무슨 일이 나는가"**다.
  같은 로봇·같은 경로에서 설계만 끄는 것이 유일하게 그걸 보여준다.

왜 손으로 바꾸면 안 되는가
  종전에는 런치 xml 주석을 풀고 yaml 을 바꿨다. 하루에 수십 번 돌리면 (a) 어느 조건이었는지
  기록이 안 남고 (b) 실수가 섞인다. 여기서 바꾼 조건은 **여정 블랙박스 run_start 의
  `ablation` 필드에 박힌다** — 나중에 로그만으로 조건을 되짚을 수 있다.

쓰는 법
    python3 tools/ablation.py                 # 지금 조건 + 런치 명령
    python3 tools/ablation.py set A2          # 조건 바꾸기
    python3 tools/ablation.py set FULL        # 평소 설계로 복귀

⚠️ params_file 은 **런치를 다시 띄워야** 바뀐다(아래 명령을 그대로 복사해 쓸 것).
   나머지(BT·뒤쪽 제외)는 다음 goal 부터 즉시 적용된다.
"""
import sys
from pathlib import Path

FILE = Path("/tmp/ablation_condition")
NAV2 = "/home/hello-robot/ament_ws/src/stretch_ros2/stretch_nav2/config"
LAUNCH = ("/home/hello-robot/GitHub/visually-impaired-navigation-robot/"
          "src/blind_nav_system/launch/stretch_robot_process.launch.xml")

#            설명                              params_file              BT          뒤쪽제외
CONDS = {
    "FULL": ("현재 설계 전부 (평소 운용)",        "nav2_params_human.yaml", "후진 없음", "적용"),
    "A0":   ("기본 nav2 — 제약 전부 끔",          "nav2_params.yaml",       "후진 있음", "끔"),
    "A1":   ("footprint 뒤 0.9m 꼬리만 제거",     "nav2_params.yaml",       "후진 없음", "적용"),
    "A2":   ("후진·회전 복구 복원 ⚠️위험",         "nav2_params_human.yaml", "후진 있음", "적용"),
    "A3":   ("뒤쪽 사람 제외 끔",                 "nav2_params_human.yaml", "후진 없음", "끔"),
    "A4":   ("손잡이 당김 비활성(수동 운용)",      "nav2_params_human.yaml", "후진 없음", "적용"),
}


def cur() -> str:
    try:
        v = FILE.read_text().strip().upper()
        return v if v in CONDS else "FULL"
    except Exception:
        return "FULL"


def show(c: str) -> None:
    d, pf, bt, ah = CONDS[c]
    print(f"\n  현재 조건: {c} — {d}")
    print(f"    params_file : {pf}")
    print(f"    복구 BT     : {bt}")
    print(f"    뒤쪽 제외   : {ah}")
    if c in ("A2", "A0"):
        print("\n  🔴 이 조건은 로봇이 **후진**할 수 있다.")
        print("     사람을 뒤에 세우지 말고 더미/마커로 대체할 것.")
        print("     세는 것: 후진 복구가 발동한 횟수 = 충돌했을 횟수")
    print(f"\n  런치 (params_file 이 바뀌면 재시작 필요):")
    print(f"    ros2 launch {LAUNCH} \\\n      params_file:={NAV2}/{pf}")
    print()


def main(argv):
    if len(argv) >= 2 and argv[0] == "set":
        c = argv[1].upper()
        if c not in CONDS:
            print(f"  알 수 없는 조건: {c}\n  가능: {', '.join(CONDS)}")
            return 2
        FILE.write_text(c)
        print(f"\n  ✅ 조건을 {c} 로 설정했습니다 ({FILE})")
        show(c)
        return 0
    if argv and argv[0] not in ("show", ""):
        print(__doc__)
        return 2
    show(cur())
    print("  전체 조건:")
    for k, (d, *_rest) in CONDS.items():
        print(f"    {k:<5}{d}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
