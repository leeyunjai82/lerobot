#!/usr/bin/env python3
"""lrweb 의 모든 페이지에서 인라인 <script> 를 뽑아 node --check 로 파싱 검증.

Python 문자열 안에 JS 를 넣다 보니 따옴표 escape 가 한 번 더 벗겨져
페이지 스크립트가 통째로 죽는 사고가 있었습니다. 그 회귀를 막습니다.

    python tools_jscheck.py            # node 를 PATH 에서 찾음
    NODE=/opt/node22/bin/node python tools_jscheck.py
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

PAGES = ("/", "/collect", "/train", "/rollout", "/control", "/calib", "/setup", "/jobs")


def main():
    node = os.environ.get("NODE") or shutil.which("node") or shutil.which("nodejs")
    if not node:
        print("node 를 찾을 수 없습니다 — NODE=/path/to/node 로 지정하세요", file=sys.stderr)
        return 2
    try:
        import lrweb
        from fastapi.testclient import TestClient
    except ImportError as e:
        print(f"의존성 없음: {e}", file=sys.stderr)
        return 2

    client = TestClient(lrweb.app)
    bad = 0
    with tempfile.TemporaryDirectory() as tmp:
        chk = Path(tmp) / "chk.mjs"
        for path in PAGES:
            resp = client.get(path)
            if resp.status_code != 200:
                print(f"  {path:9s} HTTP {resp.status_code} — 건너뜀")
                continue
            for i, js in enumerate(re.findall(r"<script(?:\s[^>]*)?>(.*?)</script>", resp.text, re.S)):
                js = js.strip()
                if not js or js.startswith("{"):      # importmap(JSON)
                    continue
                chk.write_text(js)
                r = subprocess.run([node, "--check", str(chk)], capture_output=True, text=True)
                if r.returncode:
                    bad += 1
                    print(f"  {path} script#{i}: SYNTAX ERROR\n{r.stderr}")
                else:
                    print(f"  {path:9s} script#{i}  {len(js):6d} chars  OK")
    print(f"\n문법 오류 {bad}건")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
