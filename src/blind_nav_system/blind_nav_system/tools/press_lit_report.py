#!/usr/bin/env python3
"""누르기 스냅샷 전수 점등 집계 — snapshots/press/* 를 읽어 켜짐/꺼짐을 센다.

로봇 없이 돌아간다. 쓰는 판정 코드는 **엘베 앱의 것 그대로**다:
elevator_button_press/main.py 에서 `_lit_ratio` / `_press_lit_compare` 와
LIT_* 상수를 ast 로 떼어내 실행한다. 베껴 쓰면 두 벌이 갈라지므로 안 베낀다
(main.py 를 import 하면 rclpy·cv_bridge 까지 끌고 와서 로봇 밖에서는 안 뜬다).

  python3 tools/press_lit_report.py            # 전체 집계
  python3 tools/press_lit_report.py -v         # 회차별 한 줄씩
  python3 tools/press_lit_report.py <경로>     # 다른 스냅샷 묶음

🔴 이 숫자는 측정이다. "누르기 성공률" 이 아니다 —
   '변화 없음' 이 곧 실패가 아니라는 근거는 main.py 의 LIT_* 주석에 있다.
"""
import argparse
import ast
import json
import os
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.normpath(os.path.join(HERE, "..", "elevator_button_press", "main.py"))
CORPUS = os.path.normpath(os.path.join(HERE, "..", "snapshots", "press"))

WANT_FN = ("_lit_ratio", "_press_lit_compare")
WANT_CONST = ("LIT_R_MIN", "LIT_ON_RATIO", "LIT_DELTA_MIN", "LIT_POSE_TOL_M")
VERDICTS = ("눌러서 켜짐", "누르기 전부터 켜져 있었음",
            "켜져 있다가 꺼짐", "변화 없음")


def load_from_app(path=APP):
    """main.py 에서 점등 판정 함수·상수만 떼어 실행한 이름공간을 돌려준다."""
    tree = ast.parse(open(path, encoding="utf-8").read())
    mod = ast.Module(body=[], type_ignores=[])
    for n in tree.body:
        if isinstance(n, ast.FunctionDef) and n.name in WANT_FN:
            mod.body.append(n)
        elif isinstance(n, ast.Assign):
            for t in n.targets:
                names = ([e.id for e in t.elts] if isinstance(t, ast.Tuple)
                         else [t.id] if isinstance(t, ast.Name) else [])
                if set(names) & set(WANT_CONST):
                    mod.body.append(n)
                    break
    got_fn = {f.name for f in mod.body if isinstance(f, ast.FunctionDef)}
    missing = set(WANT_FN) - got_fn
    if missing:
        sys.exit(f"⛔ {path} 에서 {sorted(missing)} 를 못 찾았다 — 함수 이름이 바뀌었나?")
    ns = {"os": os, "json": json, "state": {}, "state_lock": threading.Lock()}
    exec(compile(ast.fix_missing_locations(mod), path, "exec"), ns)
    for c in WANT_CONST:
        if c not in ns:
            sys.exit(f"⛔ {path} 에서 상수 {c} 를 못 찾았다")
    return ns


class _Log:
    """node._dlog 자리를 메우는 최소 껍데기 — 마지막 줄만 받아 본다."""
    def __init__(self):
        self.last = ""

    def _dlog(self, m):
        self.last = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corpus", nargs="?", default=CORPUS)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    try:
        import cv2
    except ImportError:
        sys.exit("⛔ cv2 가 없다 — 이 스크립트는 사진을 직접 읽는다")

    ns = load_from_app()
    lit_ratio = ns["_lit_ratio"]
    compare = ns["_press_lit_compare"]
    print(f"판정 코드: {APP}")
    print("  임계 — R>{} / 켜짐경계 {}% / 상승 {}%p / 자세허용 {}m".format(
        ns["LIT_R_MIN"], ns["LIT_ON_RATIO"], ns["LIT_DELTA_MIN"],
        ns["LIT_POSE_TOL_M"]))
    print(f"스냅샷:   {a.corpus}\n")

    if not os.path.isdir(a.corpus):
        sys.exit(f"⛔ 폴더가 없다: {a.corpus}")

    log = _Log()
    cnt, rows, dirs = {}, [], 0
    for d in sorted(os.listdir(a.corpus)):
        p = os.path.join(a.corpus, d)
        if not os.path.isdir(p):
            continue
        dirs += 1
        metas = {}
        for tag in ("A_before", "D_after2s"):
            jp = os.path.join(p, tag + ".json")
            if not os.path.exists(jp):
                continue
            m = json.load(open(jp, encoding="utf-8"))
            if m.get("lit_ratio") is None:
                # 옛 스냅샷에는 lit_ratio 가 없다 — 사진에서 지금 계산한다.
                # (앱이 쓰는 것과 같은 함수·같은 roi_raw 좌표계)
                img = os.path.join(p, tag + ".jpg")
                fr = cv2.imread(img) if os.path.exists(img) else None
                m["lit_ratio"] = lit_ratio(fr, m.get("roi_raw"))
            metas[tag] = m
        if "D_after2s" not in metas:
            rows.append((d, "D 없음", ""))
            cnt["D 없음"] = cnt.get("D 없음", 0) + 1
            continue
        # A 의 lit_ratio 를 파일에 돌려놓지 않고 compare 에 읽히려면 임시로 적어야
        # 하므로, 옛 스냅샷은 A 를 직접 넘기는 대신 compare 와 같은 규칙을 쓴다.
        # → 대신 compare 가 파일을 읽게 하려고 A 의 계산값을 메모리에 심는다.
        _real_open = open
        _amem = json.dumps(metas.get("A_before", {}), ensure_ascii=False)

        import io as _io

        def _fake_open(f, *ar, **kw):
            if os.path.basename(str(f)) == "A_before.json":
                return _io.StringIO(_amem)
            return _real_open(f, *ar, **kw)

        ns["open"] = _fake_open
        try:
            log.last = ""
            compare(log, p, metas["D_after2s"])
        finally:
            ns["open"] = _real_open
        out = log.last or "(로그 없음)"
        v = ("측정 불가" if "측정 불가" in out
             else next((k for k in VERDICTS if k in out), "?"))
        cnt[v] = cnt.get(v, 0) + 1
        rows.append((d, v, out))

    print(f"폴더 {dirs}건\n")
    for k, n in sorted(cnt.items(), key=lambda x: -x[1]):
        print(f"  {k:24s} {n:3d}건")
    if a.verbose:
        print()
        for d, v, out in rows:
            print(f"  {d:34s} {out.replace('[LIT] ', '') or v}")


if __name__ == "__main__":
    main()
