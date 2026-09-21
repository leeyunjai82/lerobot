#!/usr/bin/env python3
# ============================================================================
#  lrweb.py v11 — LeRobot 통합 웹 툴 (단일 파일 FastAPI)
#
#  v11 (4~6단계)
#   - Control: 팔별 제어 스레드 (양팔에서 왕복 지연이 직렬로 안 쌓임)
#   - Collect: lerobot-record CLI + PTY 제거 → 이 파일을 --worker 로 띄워 record_loop() 직접 호출.
#              상태/미리보기/명령이 전부 파일(RUN_DIR) → 수집 중 카메라 미리보기, lrweb 재시작에도 세션 유지
#   - 양팔: bi_so_follower / bi_so_leader (설정 mode=bimanual). 데이터셋·체크포인트 모드 호환성 검사
#
#  v10 (1단계: 버그 픽스 + lerobot 객체 전환 + 설정 파일)
#   - Control 탭이 lerobot.robots.SOFollower / lerobot.teleoperators.SOLeader 사용
#     (기존에는 FeetechMotorsBus 직접 제어 → configure() 누락으로 gripper 보호값 미설정)
#   - 관절 한계 계산을 lerobot MotorsBus._normalize(DEGREES)와 동일하게 수정
#       mid=(range_min+range_max)/2, 분모 = resolution-1 (4095)
#       (기존 2048 / 4096 기준 → 캘리브레이션 범위가 비대칭이면 전 범위가 어긋남)
#   - 카메라도 lerobot.cameras.OpenCVCamera 사용 (해상도/fps를 record와 동일하게 강제)
#   - 모든 외부 프로세스를 argv 리스트 + shell=False 로 실행 (셸 인젝션 제거)
#   - HTML 이스케이프 전면 적용
#   - 토큰 인증 (LRWEB_AUTH=off 로 해제 가능)
#   - 하위 프로세스에서 DISPLAY/WAYLAND_DISPLAY 제거
#       → lerobot이 pynput 전역 리스너 대신 터미널 리스너를 쓰도록 강제
#         (그래야 PTY 로 넣는 n/r/q 가 먹습니다)
#   - 설정을 lrweb_config.json 으로 분리 (arms 리스트 = 양팔 확장 대비 스키마)
#
#  실행 (lerobot conda env 안에서)
#   source ~/project/lerobot/activate.sh
#   pip install fastapi uvicorn
#   nohup python ~/project/lerobot/lrweb.py > ~/project/lerobot/lrweb.log 2>&1 &
#   → 로그에 찍히는 http://<host>:8080/?token=... 로 최초 1회 접속
# ============================================================================
import asyncio
import glob
import html
import json
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pandas as pd
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import (
    FileResponse,
    HTMLResponse,
    JSONResponse,
    Response,
    StreamingResponse,
)

# ------------------------------- 경로 ---------------------------------------
HOME = Path.home()
PROJ = HOME / "project/lerobot"
DATA_ROOT = PROJ / "data/hf/lerobot/local"
OUT_ROOT = PROJ / "outputs"
JOB_DIR = PROJ / "lrweb_jobs"
MARKS_FILE = PROJ / "lrweb_marks.json"
CONFIG_FILE = PROJ / "lrweb_config.json"
TOKEN_FILE = PROJ / "lrweb_token.txt"
URDF_DIR = PROJ / "urdf"


def _lerobot_calib_root():
    """lerobot utils/constants.py 와 같은 규칙: HF_LEROBOT_CALIBRATION > HF_LEROBOT_HOME/calibration
    > HF_HOME/lerobot/calibration > ~/.cache/huggingface/lerobot/calibration.
    activate.sh 의 HF_HOME 기준이면 ~/project/lerobot/data/hf/lerobot/calibration 입니다."""
    if os.environ.get("HF_LEROBOT_CALIBRATION"):
        return Path(os.environ["HF_LEROBOT_CALIBRATION"]).expanduser()
    if os.environ.get("HF_LEROBOT_HOME"):
        return Path(os.environ["HF_LEROBOT_HOME"]).expanduser() / "calibration"
    hf_home = Path(os.environ.get("HF_HOME") or (HOME / ".cache/huggingface")).expanduser()
    return hf_home / "lerobot" / "calibration"


CALIB_ROOT = _lerobot_calib_root()
PORT = 8080

# ------------------------------- 상수 ---------------------------------------
CTL_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CONTROL_HZ = 20
FEEDBACK_HZ = 10
MAX_STEP_DEG = 2.5        # 슬라이더 제어 시 스텝당 최대 이동
FOLLOW_STEP_DEG = 6.0     # 리더 팔로우 시 스텝당 최대 이동 (반응성↑)
CTL_STREAM_FPS = 15
PREVIEW_FPS = 10          # record worker 가 미리보기 JPEG 을 갱신하는 주기
NAME_RE = re.compile(r"[A-Za-z0-9._-]+")
# record worker ↔ 웹 사이의 상태/미리보기/명령 파일. 초당 수십 회 쓰므로 tmpfs 를 우선합니다.
RUN_DIR = Path("/dev/shm/lrweb") if Path("/dev/shm").is_dir() else PROJ / "lrweb_run"

# ------------------------------- 설정 ---------------------------------------
# arms 는 처음부터 리스트입니다. 한팔이면 원소 1개(side="main"),
# 양팔이면 side="left"/"right" 2개 — 스키마를 바꾸지 않고 확장합니다.
DEFAULT_CONFIG = {
    "mode": "single",
    "robot_id": "so101",
    "fps": 30,
    "default_task": "Pick up the block and place it in the box",
    # None 이면 lerobot 쪽 상대이동 캡을 쓰지 않습니다 (Control 탭의 자체 적분기가 담당).
    # 값을 주면 send_action 마다 Present_Position 을 한 번 더 읽으므로 루프가 느려집니다.
    "max_relative_target": None,
    # 포트는 비워 둡니다 — udev 심볼릭 링크(/dev/so101_follower 같은) 를 전제하지 않습니다.
    # Setup 탭에서 스캔·판별해서 채웁니다.
    "arms": [
        {
            "side": "main",
            "follower_port": "",
            "follower_id": "follower",
            "leader_port": "",
            "leader_id": "leader",
            "cameras": {},
            "view": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        }
    ],
    # 특정 팔에 속하지 않는 카메라 (양팔에서도 접두사 없이 유지됩니다)
    "cameras": {},
}


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if CONFIG_FILE.exists():
        try:
            user = json.loads(CONFIG_FILE.read_text())
            if isinstance(user, dict):
                cfg.update(user)
        except Exception as e:
            print(f"[lrweb] 설정 파일 파싱 실패 — 기본값 사용: {e}")
    else:
        try:
            CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
            CONFIG_FILE.write_text(json.dumps(cfg, indent=2, ensure_ascii=False))
        except Exception as e:
            print(f"[lrweb] 설정 파일 생성 실패: {e}")
    if not cfg.get("arms"):
        cfg["arms"] = json.loads(json.dumps(DEFAULT_CONFIG["arms"]))
    # mode 가 1차 기준입니다. arms 길이가 안 맞으면 mode 에 맞춰 잘라내거나 채웁니다.
    cfg["mode"] = "bimanual" if str(cfg.get("mode", "")).lower() in ("bimanual", "bi", "dual") else "single"
    want = 2 if cfg["mode"] == "bimanual" else 1
    cfg["arms"] = cfg["arms"][:want]
    while len(cfg["arms"]) < want:
        cfg["arms"].append({})
    names = ("main",) if want == 1 else ("left", "right")
    for i, arm in enumerate(cfg["arms"]):
        arm["side"] = names[i]
        suffix = "" if want == 1 else f"_{names[i]}"
        arm.setdefault("cameras", {})
        arm.setdefault("follower_port", "")
        arm.setdefault("leader_port", "")
        arm.setdefault("follower_id", f"follower{suffix}")
        arm.setdefault("leader_id", f"leader{suffix}")
        # 3D 뷰 전용 배치 (제어·데이터와 무관). 기본값: 양팔은 좌우로 벌림
        dy = 0.0 if want == 1 else (0.22 if names[i] == "left" else -0.22)
        v = arm.get("view") or {}
        arm["view"] = {"x": float(v.get("x", 0.0)),      # 앞뒤 (m, + 앞)
                       "y": float(v.get("y", dy)),       # 좌우 (m, + 왼쪽)
                       "yaw_deg": float(v.get("yaw_deg", 0.0))}
    cfg.setdefault("cameras", {})
    return cfg


def all_camera_specs(cfg, arm_cfgs, bimanual):
    """{표시이름: spec} — 한팔에서는 팔 카메라와 공용 카메라를 그냥 합칩니다."""
    out = {}
    for side, arm in arm_cfgs.items():
        for name, spec in arm["cameras"].items():
            out[f"{side}_{name}" if bimanual else name] = spec
    for name, spec in cfg["cameras"].items():
        out[name] = spec
    return out


def _rebind(cfg):
    """설정에서 파생되는 전역을 다시 만듭니다 (Setup 탭 저장 시 재호출)."""
    global CFG, ARM_CFGS, SIDES, BIMANUAL, CAM_SPECS, ARMS, LEADERS
    CFG = cfg
    ARM_CFGS = {a["side"]: a for a in cfg["arms"]}
    SIDES = list(ARM_CFGS)
    BIMANUAL = len(SIDES) > 1
    CAM_SPECS = all_camera_specs(cfg, ARM_CFGS, BIMANUAL)
    ARMS = {s: ArmCtl(c) for s, c in ARM_CFGS.items()}
    LEADERS = {s: LeaderCtl(c) for s, c in ARM_CFGS.items()}


CFG = ARM_CFGS = SIDES = CAM_SPECS = ARMS = LEADERS = None
BIMANUAL = False


def ports_configured():
    return all(a.get("follower_port") for a in ARM_CFGS.values())

JOB_DIR.mkdir(parents=True, exist_ok=True)
OUT_ROOT.mkdir(parents=True, exist_ok=True)
app = FastAPI(title="lrweb")

# ----------------------------- 인증 (기본 꺼짐) -------------------------------
# 기본은 인증 없음 — 예전처럼 http://<host>:8080 으로 바로 들어갑니다.
# 켜려면 둘 중 하나:
#   LRWEB_TOKEN=원하는값 python lrweb.py     (토큰 직접 지정)
#   LRWEB_AUTH=on        python lrweb.py     (lrweb_token.txt 에 자동 생성)
def _init_token():
    tok = os.environ.get("LRWEB_TOKEN")
    if tok and tok.strip():
        return tok.strip()
    if os.environ.get("LRWEB_AUTH", "").lower() not in ("on", "1", "true", "yes"):
        return None
    if TOKEN_FILE.exists():
        tok = TOKEN_FILE.read_text().strip()
        if tok:
            return tok
    tok = secrets.token_hex(16)
    TOKEN_FILE.parent.mkdir(parents=True, exist_ok=True)
    TOKEN_FILE.write_text(tok + "\n")
    try:
        TOKEN_FILE.chmod(0o600)
    except OSError:
        pass
    return tok


AUTH_TOKEN = _init_token()
COOKIE = "lrweb_token"


def token_ok(tok):
    return bool(tok) and secrets.compare_digest(str(tok), AUTH_TOKEN)


LOGIN_PAGE = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<style>body{background:#0f1216;color:#e6ebf1;font-family:system-ui,sans-serif;
display:flex;align-items:center;justify-content:center;height:100vh;margin:0}
form{background:#161b21;border:1px solid #28303a;border-radius:10px;padding:26px;width:320px}
h1{font-size:15px;letter-spacing:.14em;margin:0 0 16px;font-family:ui-monospace,monospace}
input{width:100%;box-sizing:border-box;background:#0f1216;color:#e6ebf1;border:1px solid #28303a;
border-radius:7px;padding:10px;font-size:16px;font-family:ui-monospace,monospace}
button{margin-top:12px;width:100%;background:#2c4257;color:#dceafe;border:1px solid #5d9dd6;
border-radius:7px;padding:10px;font-size:14px;cursor:pointer}
p{color:#8b98a7;font-size:12px;line-height:1.6}</style>
<form method=get action="/"><h1>LRWEB</h1>
<input name=token placeholder="access token" autofocus autocomplete=off>
<button>접속</button>
<p>토큰은 서버의 <code>lrweb_token.txt</code> 에 있습니다.<br>
해제하려면 <code>LRWEB_AUTH=off</code> 로 실행하세요.</p></form>"""


@app.middleware("http")
async def auth_middleware(request: Request, call_next):
    if AUTH_TOKEN is None:
        return await call_next(request)
    qtok = request.query_params.get("token")
    if token_ok(qtok) or token_ok(request.cookies.get(COOKIE)) or token_ok(
        request.headers.get("x-lrweb-token")
    ):
        resp = await call_next(request)
        if token_ok(qtok):
            resp.set_cookie(COOKIE, AUTH_TOKEN, max_age=90 * 86400,
                            httponly=True, samesite="lax", path="/")
        return resp
    return HTMLResponse(LOGIN_PAGE, status_code=401)


def ws_authed(sock: WebSocket):
    return AUTH_TOKEN is None or token_ok(sock.cookies.get(COOKIE))


# ----------------------------- 유틸 -----------------------------------------
def esc(s):
    return html.escape(str(s), quote=True)


def js(v):
    """<script> 블록 안에 박는 JSON 리터럴. (HTML 속성에는 쓰지 말 것 — jsattr 사용)"""
    return json.dumps(v, ensure_ascii=False).replace("</", "<\\/")


def jsattr(v):
    """onclick="..." 같은 HTML 속성 안에 박는 JSON 리터럴.
    js() 를 그대로 쓰면 JSON 의 " 가 속성 따옴표를 닫아버립니다."""
    return html.escape(json.dumps(v, ensure_ascii=False), quote=True)


def safe_name(s):
    return bool(s) and NAME_RE.fullmatch(s) is not None


def load_json(p, default):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return default


def save_json(p, obj):
    Path(p).write_text(json.dumps(obj, indent=1, ensure_ascii=False))


def load_marks():
    return load_json(MARKS_FILE, {})


def save_marks(m):
    save_json(MARKS_FILE, m)


def _clamp_int(v, default, lo, hi):
    try:
        n = int(v)
    except (TypeError, ValueError):
        n = default
    return max(lo, min(hi, n))


_ICON_CACHE = {}


def _solid_png(size, rgb):
    """의존성 없는 단색 PNG (PIL 없을 때 폴백)."""
    import struct
    import zlib
    w = h = size
    r, g, b = rgb
    raw = b"".join(b"\x00" + bytes((r, g, b)) * w for _ in range(h))

    def chunk(t, d):
        return (struct.pack(">I", len(d)) + t + d
                + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def icon_png(size):
    """앱 아이콘 PNG 생성 (PIL 있으면 로봇팔 마크, 없으면 단색)."""
    if size in _ICON_CACHE:
        return _ICON_CACHE[size]
    try:
        import io
        from PIL import Image, ImageDraw
        S = size
        img = Image.new("RGB", (S, S), (15, 18, 22))
        d = ImageDraw.Draw(img)
        pad = int(S * 0.11)
        rad = int(S * 0.22)
        d.rounded_rectangle([pad, pad, S - pad, S - pad], radius=rad,
                            fill=(44, 66, 87), outline=(93, 157, 214), width=max(2, S // 36))
        lw = max(3, S // 12)
        pts = [(S * 0.40, S * 0.72), (S * 0.40, S * 0.50), (S * 0.63, S * 0.40)]
        d.line(pts, fill=(93, 157, 214), width=lw, joint="curve")
        for x, y in pts:
            r = lw * 0.6
            d.ellipse([x - r, y - r, x + r, y + r], fill=(93, 157, 214))
        gx, gy, gr = S * 0.63, S * 0.40, S * 0.085
        d.ellipse([gx - gr, gy - gr, gx + gr, gy + gr],
                  outline=(220, 234, 254), width=max(2, S // 40))
        buf = io.BytesIO()
        img.save(buf, "PNG")
        data = buf.getvalue()
    except Exception:
        data = _solid_png(size, (44, 66, 87))
    _ICON_CACHE[size] = data
    return data


def list_datasets():
    out = []
    if DATA_ROOT.exists():
        for d in sorted(DATA_ROOT.iterdir()):
            if (d / "meta/info.json").exists():
                info = load_json(d / "meta/info.json", {})
                out.append({"name": d.name,
                            "episodes": info.get("total_episodes", "?"),
                            "frames": info.get("total_frames", "?"),
                            "fps": info.get("fps", "?"),
                            "robot_type": info.get("robot_type", "")})
    return out


def checkpoint_robot_type(rel):
    """pretrained_model/train_config.json → dataset.repo_id → 로컬 데이터셋 robot_type (best-effort)."""
    tc = load_json(OUT_ROOT / rel / "train_config.json", {})
    repo = ((tc.get("dataset") or {}).get("repo_id") or "")
    name = repo.split("/", 1)[-1] if repo else ""
    if not safe_name(name):
        return ""
    return load_json(DATA_ROOT / name / "meta/info.json", {}).get("robot_type", "")


def list_checkpoints():
    """outputs/*/checkpoints/*/pretrained_model 탐색.
    'last'가 숫자 체크포인트를 가리키는 심볼릭 링크면 중복 제거(숫자 쪽 유지)."""
    entries = []   # (rel, is_last, real_target)
    for run in sorted(OUT_ROOT.iterdir()) if OUT_ROOT.exists() else []:
        ck = run / "checkpoints"
        if not ck.is_dir():
            continue
        for step in sorted(ck.iterdir()):
            pm = step / "pretrained_model"
            if pm.is_dir():
                rel = f"{run.name}/checkpoints/{step.name}/pretrained_model"
                entries.append((rel, step.name == "last", str(pm.resolve())))
    entries.sort(key=lambda e: (e[2], e[1]))
    seen, found = set(), []
    for rel, _is_last, real in entries:
        if real in seen:
            continue
        seen.add(real)
        found.append(rel)
    found.sort(key=lambda r: ("/last/" not in f"/{r}/", r))
    return found


def episodes_df(ds):
    if not safe_name(ds):
        return pd.DataFrame()
    files = sorted(glob.glob(str(DATA_ROOT / ds / "meta/episodes/*/*.parquet")))
    if not files:
        return pd.DataFrame()
    return pd.concat([pd.read_parquet(f) for f in files]).reset_index(drop=True)


def video_keys(df):
    return sorted({m.group(1) for c in df.columns
                   if (m := re.match(r"videos/(.+)/from_timestamp", c))})


def ep_video_segments(df, ep):
    row = df[df["episode_index"] == ep]
    if row.empty:
        return []
    row = row.iloc[0]
    segs = []
    for k in video_keys(df):
        try:
            chunk = int(row.get(f"videos/{k}/chunk_index", 0))
            fidx = int(row.get(f"videos/{k}/file_index", 0))
            segs.append({"cam": k.split(".")[-1],
                         "path": f"{k}/chunk-{chunk:03d}/file-{fidx:03d}.mp4",
                         "from": float(row.get(f"videos/{k}/from_timestamp", 0)),
                         "to": float(row.get(f"videos/{k}/to_timestamp", 0))})
        except Exception:
            continue
    return segs


# ----------------------------- lerobot CLI 인자 생성 --------------------------
def _cam_cli(specs):
    """draccus 가 파싱하는 카메라 dict 리터럴 (셸을 안 거치므로 따옴표 없음)."""
    items = []
    for name, s in specs.items():
        idx = s["index_or_path"]
        idx = idx if isinstance(idx, int) else str(idx)
        items.append(f"{name}: {{type: opencv, index_or_path: {idx}, "
                     f"width: {int(s['width'])}, height: {int(s['height'])}, fps: {int(s['fps'])}}}")
    return "{" + ", ".join(items) + "}"


def robot_name():
    """lerobot 이 데이터셋 meta/info.json 의 robot_type 에 기록하는 이름."""
    return "bi_so_follower" if BIMANUAL else "so_follower"


def bimanual_base_id(role, arm_cfgs=None):
    """양팔에서 lerobot BiSO* 는 per-arm 캘리브레이션 id 를 '{id}_left' / '{id}_right' 로 만듭니다.
    설정의 follower_id 가 'X_left' / 'X_right' 면 BiSOFollowerConfig(id='X') 가 됩니다."""
    arm_cfgs = arm_cfgs or ARM_CFGS
    left = arm_cfgs["left"][f"{role}_id"]
    right = arm_cfgs["right"][f"{role}_id"]
    if not (left.endswith("_left") and right.endswith("_right") and left[:-5] == right[:-6]):
        raise ValueError(f"양팔 {role} id 는 같은 이름에 _left / _right 를 붙여야 합니다 "
                         f"(예: {role}_left / {role}_right) — 현재 {left} / {right}")
    return left[:-5]


def _need_ports(role):
    for side, arm in ARM_CFGS.items():
        if not arm.get(f"{role}_port"):
            raise NotImplementedError(
                f"{side + ' ' if BIMANUAL else ''}{'팔로워' if role == 'follower' else '리더'} 포트가 "
                f"지정되지 않았습니다 — Setup 탭에서 먼저 설정하세요")


def robot_cli_args():
    """lerobot CLI(rollout) 용 --robot.* 인자. 한팔: so101_follower / 양팔: bi_so_follower"""
    _need_ports("follower")
    if BIMANUAL:
        L, R = ARM_CFGS["left"], ARM_CFGS["right"]
        return ["--robot.type=bi_so_follower",
                f"--robot.id={bimanual_base_id('follower')}",
                f"--robot.left_arm_config.port={L['follower_port']}",
                f"--robot.left_arm_config.cameras={_cam_cli(L['cameras'])}",
                f"--robot.right_arm_config.port={R['follower_port']}",
                f"--robot.right_arm_config.cameras={_cam_cli(R['cameras'])}",
                f"--robot.cameras={_cam_cli(CFG['cameras'])}"]
    arm = ARM_CFGS[SIDES[0]]
    cams = dict(arm["cameras"])
    cams.update(CFG["cameras"])
    return ["--robot.type=so101_follower",
            f"--robot.port={arm['follower_port']}",
            f"--robot.id={arm['follower_id']}",
            f"--robot.cameras={_cam_cli(cams)}"]


def teleop_cli_args():
    _need_ports("leader")
    if BIMANUAL:
        L, R = ARM_CFGS["left"], ARM_CFGS["right"]
        return ["--teleop.type=bi_so_leader",
                f"--teleop.id={bimanual_base_id('leader')}",
                f"--teleop.left_arm_config.port={L['leader_port']}",
                f"--teleop.right_arm_config.port={R['leader_port']}"]
    arm = ARM_CFGS[SIDES[0]]
    return ["--teleop.type=so101_leader",
            f"--teleop.port={arm['leader_port']}",
            f"--teleop.id={arm['leader_id']}"]


# ----------------------------- 작업(job) 관리 --------------------------------
def pid_alive(pid):
    if not pid:
        return False
    try:
        done, _ = os.waitpid(pid, os.WNOHANG)
        if done == pid:
            return False
    except ChildProcessError:
        pass
    except OSError:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        state = Path(f"/proc/{pid}/stat").read_text().split(")")[-1].split()[0]
        if state == "Z":
            return False
    except Exception:
        pass
    return True


def jobs_index():
    idx = []
    for jf in sorted(JOB_DIR.glob("*.json"), reverse=True):
        j = load_json(jf, {})
        if j:
            j["alive"] = pid_alive(j.get("pid"))
            idx.append(j)
    return idx


def child_env():
    """lerobot 이 pynput 전역 리스너 대신 터미널(PTY) 리스너를 쓰도록 강제.
    pynput 쪽으로 붙으면 PTY 로 넣는 n/r/q 가 조용히 버려집니다."""
    env = dict(os.environ)
    env.pop("DISPLAY", None)
    env.pop("WAYLAND_DISPLAY", None)
    env["XDG_SESSION_TYPE"] = "tty"
    return env


class JobStartError(RuntimeError):
    pass


def run_dir(jid):
    return RUN_DIR / jid


def start_job(kind, argv, cwd=None, spec=None):
    """argv 는 반드시 리스트 — shell=False 이므로 셸 인젝션이 불가능합니다.
    spec 은 worker 가 읽을 작업 명세(dict) — job json 에 같이 저장됩니다."""
    if shutil.which(argv[0]) is None:
        raise JobStartError(f"실행 파일을 찾을 수 없습니다: {argv[0]} — lerobot conda env 안에서 lrweb 를 띄웠는지 확인")
    jid = f"{kind}_{time.strftime('%m%d_%H%M%S')}"
    log = JOB_DIR / f"{jid}.log"
    # worker 가 자기 job json 을 읽으므로 프로세스보다 먼저 써야 합니다
    save_json(JOB_DIR / f"{jid}.json",
              {"id": jid, "kind": kind, "cmd": " ".join(str(a) for a in argv), "pid": None,
               "log": str(log), "started": time.strftime("%F %T"), "spec": spec})
    argv = [str(a).replace("{jid}", jid) for a in argv]
    lf = open(log, "w")
    try:
        p = subprocess.Popen(argv, cwd=cwd or str(HOME), stdin=subprocess.DEVNULL,
                             stdout=lf, stderr=subprocess.STDOUT,
                             env=child_env(), preexec_fn=os.setsid)
    finally:
        lf.close()      # 자식이 dup 를 들고 있으므로 부모 쪽은 닫습니다 (fd 누수 방지)
    j = load_json(JOB_DIR / f"{jid}.json", {})
    j["pid"] = p.pid
    j["cmd"] = " ".join(argv)
    save_json(JOB_DIR / f"{jid}.json", j)
    return jid


def send_cmd(jid, key):
    """record worker 에 n/r/q 전달 — 파일 큐. lrweb 를 재시작해도 그대로 동작합니다."""
    if not safe_name(jid):
        return False
    rd = run_dir(jid)
    try:
        rd.mkdir(parents=True, exist_ok=True)
        with open(rd / "cmd", "a") as f:
            f.write(key)
        return True
    except OSError:
        return False


def record_status(jid):
    if not safe_name(jid):
        return {}
    return load_json(run_dir(jid) / "status.json", {})


def kill_job(jid):
    if not safe_name(jid):
        return
    jf = JOB_DIR / f"{jid}.json"
    j = load_json(jf, {})
    if not j.get("pid"):
        return
    sig = signal.SIGKILL if j.get("kill_requested") else signal.SIGINT
    try:
        os.killpg(os.getpgid(j["pid"]), sig)
    except OSError:
        pass
    j["kill_requested"] = True
    save_json(jf, j)


def delete_job(jid):
    if not safe_name(jid):
        return False
    j = load_json(JOB_DIR / f"{jid}.json", {})
    if j and pid_alive(j.get("pid")):
        return False
    for suffix in (".json", ".log"):
        try:
            (JOB_DIR / f"{jid}{suffix}").unlink()
        except FileNotFoundError:
            pass
    shutil.rmtree(run_dir(jid), ignore_errors=True)
    return True


def log_tail(jid, nbytes=4000):
    if not safe_name(jid):
        return ""
    j = load_json(JOB_DIR / f"{jid}.json", {})
    try:
        with open(j["log"], "rb") as f:
            f.seek(max(-nbytes, -os.path.getsize(j["log"])), 2)
            return f.read().decode(errors="ignore")[-3200:]
    except Exception:
        return ""


# ----------------------------- 수동 제어 (Control 탭) -------------------------
class ArmCtl:
    """lerobot SOFollower 래퍼.

    SOFollower.connect() 안에서 configure() 가 돌면서 Operating_Mode / P·I·D 계수 /
    gripper 의 Max_Torque_Limit·Protection_Current·Overload_Torque 까지 세팅됩니다.
    (직접 FeetechMotorsBus 를 열면 이게 전부 빠집니다 — 그리퍼 소손 원인)

    cameras 는 비워 둡니다. SOFollower.is_connected 가 '버스 AND 모든 카메라' 라서
    카메라 하나가 빠지면 팔 제어·E-STOP 까지 같이 죽기 때문입니다.
    카메라는 CamStreamer 가 따로 담당합니다.
    """

    def __init__(self, arm_cfg):
        self.cfg = arm_cfg
        self.side = arm_cfg["side"]
        self.robot = None
        self.torque = False
        self.target = {}   # 사용자가 원하는 최종 목표 (슬라이더 / 리더)
        self.cmd = {}      # 명령 적분기 — target 을 향해 제한 속도로 이동, 엔코더와 안 섞음
        self.actual = {}   # 엔코더 실측 (표시/3D 전용)
        self.limits = {}
        self.err = ""
        self.follow = False
        self.lock = threading.Lock()   # 시리얼은 스레드 안전하지 않음 — 루프/명령 직렬화
        self.leader = None
        self.running = False
        self.thread = None

    # --- 연결 ---------------------------------------------------------------
    @property
    def connected(self):
        return self.robot is not None

    def calib_path(self):
        from lerobot.robots.so_follower import SOFollower
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
        return HF_LEROBOT_CALIBRATION / ROBOTS / SOFollower.name / f"{self.cfg['follower_id']}.json"

    def connect(self):
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        if not self.cfg.get("follower_port"):
            raise RuntimeError("팔로워 포트가 지정되지 않았습니다 — Setup 탭에서 먼저 설정하세요")
        # ensure_safe_goal_position 은 float 만 받습니다 (int 면 TypeError) — 반드시 캐스팅
        mrt = CFG.get("max_relative_target")
        mrt = float(mrt) if isinstance(mrt, (int, float)) and not isinstance(mrt, bool) else None
        cfg = SOFollowerRobotConfig(
            id=self.cfg["follower_id"],
            port=self.cfg["follower_port"],
            use_degrees=True,
            max_relative_target=mrt,
            cameras={},
        )
        robot = SOFollower(cfg)
        if not robot.calibration:
            raise RuntimeError(
                f"캘리브레이션 파일이 없습니다: {robot.calibration_fpath} — Calib 탭에서 만드세요")
        robot.connect(calibrate=False)      # calibrate=True 면 input() 에서 서버가 멈춥니다
        if not robot.bus.is_calibrated:
            # 파일과 모터 EEPROM 이 어긋난 경우 — lerobot calibrate() 의 '파일 사용' 분기와 동일
            robot.bus.write_calibration(robot.calibration)
        self.robot = robot
        self._build_limits()
        self.actual = self.read()
        self.target = dict(self.actual)
        self.cmd = dict(self.actual)

    def _build_limits(self):
        bus = self.robot.bus
        for name in CTL_JOINTS:
            if name == "gripper":
                self.limits[name] = (0.0, 100.0)     # MotorNormMode.RANGE_0_100
                continue
            c = bus.calibration.get(name)
            if c is None:
                self.limits[name] = (-170.0, 170.0)
                continue
            # lerobot MotorsBus._normalize(DEGREES) 와 동일한 식
            max_res = bus.model_resolution_table[bus.motors[name].model] - 1
            mid = (c.range_min + c.range_max) / 2
            lo = (c.range_min - mid) * 360 / max_res
            hi = (c.range_max - mid) * 360 / max_res
            if lo > hi:
                lo, hi = hi, lo
            self.limits[name] = (round(lo, 1), round(hi, 1))

    def disconnect(self):
        self.stop_loop()
        with self.lock:
            if self.robot is not None:
                try:
                    self.robot.disconnect()     # disable_torque_on_disconnect=True 가 기본
                except Exception:
                    pass
            self.robot = None
            self.torque = False
            self.follow = False
            self.err = ""

    # --- 팔별 제어 스레드 (4단계) ---------------------------------------------
    # 팔마다 스레드 하나: 리더 읽기 → 적분기 step → 주기적 실측. 팔이 늘어도 왕복 지연이
    # 직렬로 쌓이지 않습니다. WebSocket 은 상태를 퍼가기만 합니다.
    def start_loop(self, leader):
        self.leader = leader
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, name=f"arm-{self.side}", daemon=True)
        self.thread.start()

    def stop_loop(self):
        self.running = False
        t, self.thread = self.thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.5)

    def _loop(self):
        last_fb = 0.0
        while self.running and self.connected:
            t0 = time.monotonic()
            try:
                with self.lock:
                    if self.robot is None:
                        break
                    ldr = self.leader
                    if self.follow and ldr is not None and ldr.connected:
                        for n, v in ldr.read().items():
                            if n in CTL_JOINTS:
                                self.target[n] = v
                    self.step()
                    if t0 - last_fb > 1.0 / FEEDBACK_HZ:
                        self.actual = self.read()
                        last_fb = t0
                self.err = ""
            except Exception as e:
                self.err = str(e)
            time.sleep(max(0.0, 1.0 / CONTROL_HZ - (time.monotonic() - t0)))

    # --- 입출력 (호출자가 lock 을 잡거나, 루프 스레드 안에서만) -------------------
    def read(self):
        obs = self.robot.get_observation()
        return {k[:-4]: float(v) for k, v in obs.items() if k.endswith(".pos")}

    def set_torque(self, on: bool):
        with self.lock:
            (self.robot.bus.enable_torque if on else self.robot.bus.disable_torque)()
            self.torque = on
            if on:
                # 점프 방지: 현재 자세를 명령/목표의 시작점으로
                self.actual = self.read()
                self.target = dict(self.actual)
                self.cmd = dict(self.actual)

    def step(self):
        """cmd 를 target 으로 제한 속도 이동. 엔코더 값은 절대 안 섞음(떨림 방지).
        변화가 없으면 쓰지 않음 — 도달 후엔 서보가 자체 유지."""
        if not (self.robot and self.torque):
            return
        goal = {}
        cap = FOLLOW_STEP_DEG if self.follow else MAX_STEP_DEG
        for n in CTL_JOINTS:
            cur = self.cmd.get(n, self.actual.get(n, 0.0))
            lo, hi = self.limits[n]
            tgt = max(lo, min(hi, self.target.get(n, cur)))
            diff = tgt - cur
            if abs(diff) < 0.2:          # 데드밴드: 도달로 간주, 쓰기 중단
                self.cmd[n] = tgt
                continue
            nxt = cur + max(-cap, min(cap, diff))
            self.cmd[n] = nxt
            goal[n] = nxt
        if goal:
            self.robot.send_action({f"{k}.pos": v for k, v in goal.items()})


class LeaderCtl:
    """lerobot SOLeader 래퍼. connect() 안의 configure() 가 토크를 꺼 줍니다."""

    def __init__(self, arm_cfg):
        self.cfg = arm_cfg
        self.side = arm_cfg["side"]
        self.tele = None
        self.lock = threading.Lock()

    @property
    def connected(self):
        return self.tele is not None

    def connect(self):
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
        cfg = SOLeaderTeleopConfig(id=self.cfg["leader_id"],
                                   port=self.cfg["leader_port"],
                                   use_degrees=True)
        tele = SOLeader(cfg)
        if not tele.calibration:
            raise RuntimeError(
                f"리더 캘리브레이션 파일이 없습니다: {tele.calibration_fpath} — Calib 탭에서 만드세요")
        tele.connect(calibrate=False)
        if not tele.bus.is_calibrated:
            tele.bus.write_calibration(tele.calibration)
        self.tele = tele

    def read(self):
        with self.lock:
            if self.tele is None:
                return {}
            return {k[:-4]: float(v) for k, v in self.tele.get_action().items() if k.endswith(".pos")}

    def disconnect(self):
        with self.lock:
            if self.tele is not None:
                try:
                    self.tele.disconnect()
                except Exception:
                    pass
            self.tele = None


class CamStreamer:
    """Control 탭 전용 MJPEG 소스. lerobot OpenCVCamera 를 쓰므로
    해상도/fps 가 record 설정과 동일하게 강제됩니다 (raw cv2 로는 기본값이 잡혔음)."""

    def __init__(self):
        self.on = False
        self.frames = {}     # name -> jpeg bytes
        self.cams = {}
        self.thread = None

    def open(self):
        try:
            import cv2  # noqa: F401
            from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
        except ImportError:
            return False
        self.cams = {}
        for name, spec in CAM_SPECS.items():
            idx = spec["index_or_path"]
            idx = idx if isinstance(idx, int) else Path(str(idx))
            try:
                cam = OpenCVCamera(OpenCVCameraConfig(
                    index_or_path=idx, fps=int(spec["fps"]),
                    width=int(spec["width"]), height=int(spec["height"]),
                    color_mode="bgr",   # imencode 가 BGR 을 기대 — 변환 한 번 아낍니다
                ))
                cam.connect()
                self.cams[name] = cam
            except Exception as e:
                print(f"[lrweb] 카메라 '{name}' 열기 실패: {e}")
        if not self.cams:
            return False
        self.on = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def _loop(self):
        import cv2
        while self.on:
            t0 = time.monotonic()
            for name, cam in self.cams.items():
                try:
                    frame = cam.read_latest(max_age_ms=1000)
                except Exception:
                    continue
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self.frames[name] = buf.tobytes()
            time.sleep(max(0.0, 1.0 / CTL_STREAM_FPS - (time.monotonic() - t0)))
        for cam in self.cams.values():
            try:
                cam.disconnect()
            except Exception:
                pass
        self.cams = {}
        self.frames = {}

    def close(self):
        self.on = False


CAMS = CamStreamer()
CTL_OWNER = None    # 현재 Control 탭을 점유한 WebSocket

_rebind(load_config())      # ArmCtl/LeaderCtl 정의 이후에 호출해야 합니다


def any_arm_connected():
    return any(a.connected for a in ARMS.values())


# ----------------------------- 포트 / 카메라 탐색 (Setup 탭) -------------------
def _read_sysfs(p):
    try:
        return p.read_text().strip()
    except Exception:
        return ""


def usb_info(dev, cls="tty"):
    """/dev/ttyACM0 (cls=tty) 또는 /dev/video0 (cls=video4linux) → 그 뒤의 USB 장치 정보.
    sysfs 를 부모 방향으로 거슬러 올라가 idVendor 가 있는 노드를 찾습니다."""
    node = Path("/sys/class") / cls / os.path.basename(dev) / "device"
    if not node.exists():
        return {}
    p = node.resolve()
    for _ in range(8):
        if (p / "idVendor").exists():
            return {"vid": _read_sysfs(p / "idVendor"),
                    "pid": _read_sysfs(p / "idProduct"),
                    "manufacturer": _read_sysfs(p / "manufacturer"),
                    "product": _read_sysfs(p / "product"),
                    "serial": _read_sysfs(p / "serial")}
        if p.parent == p:
            break
        p = p.parent
    return {}


def _alias_map(dirname):
    """/dev/serial/by-id 등 → {실제 장치경로: 별칭경로}"""
    out = {}
    d = Path(dirname)
    if not d.is_dir():
        return out
    for link in sorted(d.iterdir()):
        try:
            out.setdefault(os.path.realpath(link), str(link))
        except OSError:
            pass
    return out


def list_serial_ports():
    """시리얼 후보 나열. udev 심볼릭 링크가 전혀 없어도 동작합니다."""
    by_id = _alias_map("/dev/serial/by-id")
    by_path = _alias_map("/dev/serial/by-path")
    devs = set()
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        devs.update(glob.glob(pat))
    devs.update(by_id)          # 위 패턴에 안 걸리는 이름까지 포함
    devs.update(by_path)
    used = {}
    for side, arm in ARM_CFGS.items():
        for role in ("follower", "leader"):
            p = arm.get(f"{role}_port")
            if p:
                used.setdefault(os.path.realpath(p) if os.path.exists(p) else p, []).append(
                    f"{side}/{role}" if BIMANUAL else role)
    out = []
    for dev in sorted(devs):
        info = usb_info(dev)
        out.append({
            "dev": dev,
            "by_id": by_id.get(dev, ""),
            "by_path": by_path.get(dev, ""),
            "usb": info,
            "label": (f"{info.get('manufacturer', '')} {info.get('product', '')}".strip()
                      or os.path.basename(dev)),
            "used_by": used.get(dev, []),
        })
    return out


CAM_SNAPS = {}      # realpath(dev) -> jpeg bytes (Setup 탭 카메라 식별용 썸네일)


def _snap_key(dev):
    dev = str(dev)
    return os.path.realpath(dev) if dev.startswith("/dev/") else dev


def camera_snapshot(dev, warm=6, width=320):
    """카메라를 잠깐 열어 한 장 찍습니다. 어느 장치가 어느 카메라인지 눈으로 확인하는 용도.
    자동 노출이 안정될 때까지 몇 장 버립니다."""
    try:
        import cv2
    except ImportError:
        return None
    d = str(dev)
    target = int(d) if d.isdigit() else d
    cap = cv2.VideoCapture(target)
    try:
        if not cap.isOpened():
            return None
        frame = None
        for _ in range(warm):
            ok, f = cap.read()
            if ok:
                frame = f
        if frame is None:
            return None
        h, w = frame.shape[:2]
        if w > width:
            frame = cv2.resize(frame, (width, max(1, int(h * width / w))))
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else None
    except Exception:
        return None
    finally:
        cap.release()


def list_video_devices():
    """카메라 후보. lerobot OpenCVCamera.find_cameras() 를 쓰되,
    실패하면 /dev/video* 나열로 폴백합니다.

    같은 모델 카메라 2개(wrist ×2)는 USB 시리얼 번호가 없으면 /dev/v4l/by-id 이름이 겹쳐
    어느 링크가 어느 카메라인지 부팅마다 바뀔 수 있습니다. 시리얼 포트와 같은 규칙으로
    시리얼 번호가 있을 때만 by-id, 없으면 by-path(물리 USB 포트 고정) 를 씁니다."""
    by_id = _alias_map("/dev/v4l/by-id")
    by_path = _alias_map("/dev/v4l/by-path")

    def entry(dev, prof=None, name=""):
        real = os.path.realpath(dev) if dev.startswith("/dev/") else dev
        info = usb_info(dev, "video4linux") if dev.startswith("/dev/") else {}
        prof = prof or {}
        return {"dev": dev, "by_id": by_id.get(real, ""), "by_path": by_path.get(real, ""),
                "usb": info, "width": prof.get("width"), "height": prof.get("height"),
                "fps": prof.get("fps"), "name": name}

    found = []
    try:
        from lerobot.cameras.opencv import OpenCVCamera
        for c in OpenCVCamera.find_cameras():
            found.append(entry(str(c.get("id")), c.get("default_stream_profile"), c.get("name", "")))
    except Exception as e:
        for dev in sorted(glob.glob("/dev/video*")):
            found.append(entry(dev, None, f"(probe 실패: {e})"))
    CAM_SNAPS.clear()
    for f in found:
        jpg = camera_snapshot(f["dev"])
        if jpg:
            CAM_SNAPS[_snap_key(f["dev"])] = jpg
            for alias in (f["by_id"], f["by_path"]):
                if alias:
                    CAM_SNAPS[_snap_key(alias)] = jpg
        f["snap"] = bool(jpg)
    return found


def _feetech_motors(norm_degrees=True):
    from lerobot.motors import Motor, MotorNormMode
    return {n: Motor(i + 1, "sts3215",
                     MotorNormMode.RANGE_0_100 if n == "gripper"
                     else (MotorNormMode.DEGREES if norm_degrees else MotorNormMode.RANGE_M100_100))
            for i, n in enumerate(CTL_JOINTS)}


def probe_port(port, full=False):
    """포트에 붙은 Feetech 모터 ID 를 나열합니다.
    full=False 면 기본 보드레이트(1 Mbps)만 — 웹에서 쓰기에 scan_port 는 너무 느립니다."""
    from lerobot.motors.feetech import FeetechMotorsBus
    if full:
        found = FeetechMotorsBus.scan_port(port)
        return {"baudrates": {str(b): sorted(ids) for b, ids in found.items()}}
    bus = FeetechMotorsBus(port, {})
    bus.connect(handshake=False)
    try:
        bus.set_baudrate(FeetechMotorsBus.default_baudrate)
        ids_models = bus.broadcast_ping() or {}
    finally:
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
    return {"baudrate": FeetechMotorsBus.default_baudrate,
            "ids": sorted(ids_models),
            "models": {str(i): m for i, m in ids_models.items()}}


class PortWatcher:
    """여러 포트를 동시에 토크 OFF 로 열어 두고 Present_Position 을 읽습니다.
    사용자가 팔 하나를 손으로 움직이면 값이 변하는 포트가 그 팔입니다 —
    leader/follower 판별의 유일하게 확실한 방법(전기적으로는 구분 불가)."""

    def __init__(self):
        self.on = False
        self.state = {}          # port -> {"pos":{}, "span":{}, "err":str}
        self.threads = []

    def start(self, ports):
        self.stop()
        self.state = {p: {"pos": {}, "span": {}, "err": ""} for p in ports}
        self.on = True
        for p in ports:
            t = threading.Thread(target=self._loop, args=(p,), daemon=True)
            t.start()
            self.threads.append(t)

    def _loop(self, port):
        from lerobot.motors.feetech import FeetechMotorsBus
        st = self.state[port]
        bus = None
        try:
            bus = FeetechMotorsBus(port, _feetech_motors())
            bus.connect(handshake=False)
            bus.disable_torque()          # 손으로 움직일 수 있게
            lo, hi = {}, {}
            while self.on:
                pos = bus.sync_read("Present_Position", normalize=False)
                for k, v in pos.items():
                    lo[k] = min(lo.get(k, v), v)
                    hi[k] = max(hi.get(k, v), v)
                st["pos"] = {k: int(v) for k, v in pos.items()}
                st["span"] = {k: int(hi[k] - lo[k]) for k in pos}
                st["err"] = ""
                time.sleep(0.1)
        except Exception as e:
            st["err"] = str(e)
        finally:
            if bus is not None:
                try:
                    bus.disconnect(disable_torque=False)
                except Exception:
                    pass

    def stop(self):
        self.on = False
        for t in self.threads:
            t.join(timeout=1.5)
        self.threads = []


WATCH = PortWatcher()


class MotorSetupSession:
    """새 팔 모터 ID 세팅 — lerobot-setup-motors 의 웹 버전.

    새 STS3215 는 전부 ID 1 이라, 모터를 **한 개씩만** 보드에 연결하고 순서대로 ID 를 씁니다.
    lerobot 과 같은 순서(gripper=6 → … → shoulder_pan=1)로 bus.setup_motor(name) 을 부릅니다.
    setup_motor 는 응답한 첫 모터를 그냥 골라 쓰므로, 쓰기 전에 응답 ID 가 정확히 1개인지 따로 확인합니다."""

    ORDER = list(reversed(CTL_JOINTS))

    def __init__(self):
        self.lock = threading.Lock()
        self._reset()

    def _reset(self):
        self.bus = None
        self.port = ""
        self.stage = "idle"      # idle | running | done | error
        self.idx = 0
        self.done = []           # [{"name", "id", "from_id", "baud"}]
        self.err = ""
        self.last = ""

    @property
    def active(self):
        return self.stage == "running"

    @property
    def current(self):
        return self.ORDER[self.idx] if self.idx < len(self.ORDER) else None

    def start(self, port):
        if self.active:
            raise RuntimeError("이미 모터 ID 세팅 진행 중")
        from lerobot.motors.feetech import FeetechMotorsBus
        self._reset()
        bus = FeetechMotorsBus(port, _feetech_motors())
        bus.connect(handshake=False)     # 아직 ID 가 안 맞으니 handshake 는 하면 안 됨
        self.bus = bus
        self.port = port
        self.stage = "running"

    def write_current(self):
        if not self.active:
            raise RuntimeError("진행 중이 아닙니다")
        name = self.current
        bus = self.bus
        target = bus.motors[name].id
        with self.lock:
            baud, cur_id = bus._find_single_motor(name)     # 보드레이트 전부 훑어 모터 1개 탐색
            bus.set_baudrate(baud)
            ids = bus.broadcast_ping() or {}
            if len(ids) > 1:
                raise RuntimeError(f"모터가 {len(ids)}개 응답합니다 (ID {sorted(ids)}) — "
                                   f"'{name}' 모터 하나만 보드에 연결하세요")
            bus.setup_motor(name, initial_baudrate=baud, initial_id=cur_id)
            bus.set_baudrate(bus.default_baudrate)
            after = bus.broadcast_ping() or {}
        if target not in after:
            raise RuntimeError(f"ID {target} 기록 후 응답이 없습니다 — 전원/배선 확인 후 다시 시도")
        self.done.append({"name": name, "id": target, "from_id": int(cur_id), "baud": int(baud)})
        self.last = f"{name}: ID {cur_id} → {target} (baud {baud} → {bus.default_baudrate})"
        self.idx += 1
        if self.idx >= len(self.ORDER):
            self.stage = "done"
            self._close()

    def _close(self):
        bus, self.bus = self.bus, None
        if bus is not None:
            with self.lock:
                try:
                    bus.disconnect(disable_torque=False)
                except Exception:
                    pass

    def cancel(self):
        self._close()
        self.stage = "idle"

    def state(self):
        return {"stage": self.stage, "port": self.port, "order": self.ORDER,
                "idx": self.idx, "current": self.current,
                "current_id": (self.idx + 1 <= len(self.ORDER)) and (len(self.ORDER) - self.idx) or None,
                "done": self.done, "err": self.err, "last": self.last}


MOTORSETUP = MotorSetupSession()


def busy_with(kinds):
    for j in jobs_index():
        if j["alive"] and j["kind"] in kinds:
            return j
    return None


def robot_busy():
    """팔(시리얼)을 쓰는 작업: record/rollout + Control 탭 수동 제어 + Setup 포트 감시"""
    if any_arm_connected():
        return {"id": "manual-control", "kind": "control", "alive": True}
    if WATCH.on:
        return {"id": "port-watch (Setup 탭)", "kind": "setup", "alive": True}
    if CALIB.active:
        return {"id": "calibration (Calib 탭)", "kind": "calib", "alive": True}
    if MOTORSETUP.active:
        return {"id": "motor-id-setup (Setup 탭)", "kind": "setup", "alive": True}
    return busy_with(("record", "rollout"))


def gpu_or_loop_busy():
    """학습 시작을 막아야 하는 작업 (Control 수동 제어는 학습과 동시 가능)"""
    return busy_with(("record", "rollout", "train"))


def exclusive_busy():
    if any_arm_connected():
        return {"id": "manual-control (Control 탭)", "kind": "control", "alive": True}
    if WATCH.on:
        return {"id": "port-watch (Setup 탭)", "kind": "setup", "alive": True}
    if CALIB.active:
        return {"id": "calibration (Calib 탭)", "kind": "calib", "alive": True}
    if MOTORSETUP.active:
        return {"id": "motor-id-setup (Setup 탭)", "kind": "setup", "alive": True}
    return busy_with(("record", "rollout", "train"))


# ----------------------------- 화면 공통 ------------------------------------
CSS = """
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#161b21">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="LRWEB">
<link rel="manifest" href="/manifest.webmanifest">
<link rel="apple-touch-icon" href="/icon-180.png">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans+KR:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap" rel="stylesheet">
<style>
:root{
  --bg:#0f1216; --surface:#161b21; --surface2:#1b222a; --line:#28303a;
  --text:#e6ebf1; --muted:#8b98a7; --dim:#5d6a79;
  --accent:#5d9dd6; --accent-dim:#2c4257;
  --ok:#57b98a; --warn:#d9a13b; --bad:#c96060;
  --mono:'IBM Plex Mono',ui-monospace,monospace;
  --sans:'IBM Plex Sans','IBM Plex Sans KR',system-ui,sans-serif;
}
*{box-sizing:border-box}
body{font-family:var(--sans);margin:0;background:var(--bg);color:var(--text);font-size:14.5px;line-height:1.55}
a{color:var(--accent);text-decoration:none} a:hover{text-decoration:underline}
.appbar{display:flex;align-items:center;gap:26px;background:var(--surface);
  border-bottom:1px solid var(--line);padding:0 22px;height:52px;
  position:sticky;top:0;z-index:10}
.brand{font-family:var(--mono);font-weight:600;font-size:13px;letter-spacing:.14em;color:var(--text)}
.brand small{color:var(--dim);font-weight:400;letter-spacing:.14em}
.nav{display:flex;gap:2px;height:100%}
.nav a{display:flex;align-items:center;padding:0 14px;color:var(--muted);
  border-bottom:2px solid transparent;font-weight:500;font-size:13.5px}
.nav a:hover{color:var(--text);text-decoration:none}
.nav a.on{color:var(--text);border-bottom-color:var(--accent)}
.statuscluster{margin-left:auto;display:flex;align-items:center;gap:10px;
  font-family:var(--mono);font-size:12px;color:var(--muted)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--dim)}
.dot.live{background:var(--ok);animation:pulse 1.6s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
@media (prefers-reduced-motion: reduce){.dot.live{animation:none}}
.wrap{padding:26px 22px 60px;max-width:1240px;margin:auto}
.eyebrow{font-family:var(--mono);font-size:11px;letter-spacing:.18em;
  text-transform:uppercase;color:var(--dim);margin:0 0 6px}
h2{margin:0 0 20px;font-size:19px;font-weight:600}
.card{background:var(--surface);border:1px solid var(--line);border-radius:10px;
  padding:18px 20px;margin-bottom:18px}
table{border-collapse:collapse;width:100%}
th{font-family:var(--mono);font-size:11px;letter-spacing:.14em;text-transform:uppercase;
  color:var(--dim);font-weight:500;text-align:left;padding:8px 12px;border-bottom:1px solid var(--line)}
td{padding:9px 12px;border-bottom:1px solid var(--line)}
tr:last-child td{border-bottom:none}
tbody tr:hover{background:var(--surface2)}
td.num{font-family:var(--mono);font-size:13px;text-align:right;color:var(--muted);
  font-variant-numeric:tabular-nums}
th.num{text-align:right}
.mono{font-family:var(--mono);font-size:13px}
.badge{display:inline-block;font-family:var(--mono);font-size:11px;letter-spacing:.06em;
  padding:2px 9px;border-radius:20px;border:1px solid transparent}
.b-ok{color:var(--ok);border-color:var(--ok);background:rgba(87,185,138,.08)}
.b-bad{color:var(--bad);border-color:var(--bad);background:rgba(201,96,96,.08)}
.b-run{color:var(--accent);border-color:var(--accent);background:rgba(93,157,214,.08)}
.b-warn{color:var(--warn);border-color:var(--warn);background:rgba(217,161,59,.08)}
button{font-family:var(--sans);font-size:13px;font-weight:500;
  background:var(--surface2);color:var(--text);border:1px solid var(--line);
  border-radius:7px;padding:7px 14px;cursor:pointer;transition:border-color .12s}
button:hover{border-color:var(--accent)}
button:focus-visible{outline:2px solid var(--accent);outline-offset:1px}
button.primary{background:var(--accent-dim);border-color:var(--accent);color:#dceafe}
button.danger{color:var(--bad);border-color:rgba(201,96,96,.5)}
button.danger:hover{border-color:var(--bad);background:rgba(201,96,96,.08)}
button.big{font-size:15px;padding:12px 22px;font-family:var(--mono)}
input,select{font-family:var(--sans);font-size:13.5px;background:var(--bg);
  color:var(--text);border:1px solid var(--line);border-radius:7px;padding:7px 10px}
input:focus,select:focus{outline:none;border-color:var(--accent)}
label.f{display:flex;flex-direction:column;gap:4px;font-size:12.5px;color:var(--muted)}
.formgrid{display:flex;gap:14px;flex-wrap:wrap;align-items:flex-end;margin-bottom:20px}
.toolbar{display:flex;gap:8px;align-items:center;margin-bottom:16px;flex-wrap:wrap}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
.vpanel{background:var(--surface);border:1px solid var(--line);border-radius:10px;overflow:hidden}
.vpanel .vhead{display:flex;justify-content:space-between;align-items:center;
  padding:8px 14px;border-bottom:1px solid var(--line);
  font-family:var(--mono);font-size:12px;color:var(--muted)}
.vpanel .vhead b{color:var(--text);font-weight:600;letter-spacing:.1em;text-transform:uppercase}
video{width:100%;display:block;background:#000}
.markbar{display:flex;align-items:center;gap:14px;flex-wrap:wrap;
  background:rgba(201,96,96,.06);border:1px solid rgba(201,96,96,.35);
  border-radius:10px;padding:12px 16px;margin-bottom:18px}
.runbar{display:flex;align-items:center;gap:14px;flex-wrap:wrap;
  background:rgba(93,157,214,.06);border:1px solid rgba(93,157,214,.4);
  border-radius:10px;padding:12px 16px;margin-bottom:18px}
pre{font-family:var(--mono);font-size:12.5px;line-height:1.5;background:#0a0d10;
  border:1px solid var(--line);padding:12px 14px;border-radius:10px;
  overflow-x:auto;max-height:340px;color:#b9c4d0}
.muted{color:var(--muted);font-size:13px}
form.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:6px 0 22px}
.chartbox{background:var(--surface);border:1px solid var(--line);border-radius:10px;padding:16px;margin-bottom:18px}
.keys{display:flex;gap:12px;margin:14px 0 18px}

/* ---- 반응형 (태블릿/모바일) ---- */
@media(max-width:820px){
  .appbar{gap:12px;padding:0 14px;height:auto;min-height:52px;flex-wrap:wrap}
  .brand{font-size:12px}
  .brand small{display:none}
  .statuscluster{font-size:11px;max-width:52vw;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
  .nav{order:3;width:100%;overflow-x:auto;-webkit-overflow-scrolling:touch;
    height:46px;gap:0;border-top:1px solid var(--line)}
  .nav a{padding:0 15px;white-space:nowrap;flex:0 0 auto}
  .wrap{padding:18px 14px 48px}
  h2{font-size:18px;margin-bottom:16px}
  button{padding:10px 15px}
  button.big{padding:13px 22px}
  input,select{font-size:16px;padding:9px 11px}   /* 16px = iOS 자동확대 방지 */
  label.f{min-width:0!important}
  .card{padding:16px 15px}
  pre{max-height:52vh}
}
@media(max-width:560px){
  .formgrid{gap:10px}
  .formgrid label.f{flex:1 1 100%}   /* 폼 필드 세로 스택 */
  form.row > *{flex:1 1 100%}
  .nav a{padding:0 13px;font-size:13px}
  table{font-size:13px}
  th,td{padding:8px 9px}
}
.fsbtn{padding:5px 9px;font-size:15px;line-height:1;background:transparent;
  border-color:var(--line);color:var(--muted);display:inline-flex;align-items:center}
.fsbtn:hover{color:var(--text);border-color:var(--accent)}
</style>
<script>
function toggleFS(){
  var el=document.documentElement;
  if(document.fullscreenElement){ (document.exitFullscreen||document.webkitExitFullscreen).call(document); }
  else{ (el.requestFullscreen||el.webkitRequestFullscreen).call(el); }
}
if('serviceWorker' in navigator){ navigator.serviceWorker.register('/sw.js').catch(function(){}); }
</script>"""


def setup_needed_html():
    """포트가 아직 지정되지 않았을 때 각 탭 상단에 띄우는 안내."""
    if ports_configured():
        return ""
    return ('<p class="badge b-warn">포트가 지정되지 않았습니다 — '
            '<a href="/setup">Setup 탭</a>에서 USB 포트를 먼저 정하세요</p>')


def nav_html(active=""):
    running = [j for j in jobs_index() if j["alive"]]
    if any_arm_connected():
        running = [{"id": "manual-control", "kind": "control"}] + running
    if running:
        j = running[0]
        extra = f' +{len(running) - 1}' if len(running) > 1 else ''
        cluster = (f'<div class=statuscluster><span class="dot live"></span>'
                   f'{esc(j["kind"].upper())} · {esc(j["id"])}{extra}</div>')
    else:
        cluster = '<div class=statuscluster><span class=dot></span>IDLE</div>'
    mode_badge = ('<span class="badge b-run">양팔</span>' if BIMANUAL
                  else '<span class="badge">한팔</span>')
    if not ports_configured():
        mode_badge += ' <a href="/setup"><span class="badge b-warn">포트 미설정</span></a>'
    cluster = cluster.replace('<div class=statuscluster>',
                              f'<div class=statuscluster>{mode_badge}&nbsp;')

    def tab(href, label, key):
        on = ' class=on' if key == active else ''
        return f'<a href="{href}"{on}>{label}</a>'

    return (f'<div class=appbar><div class=brand>LRWEB <small>/ SO-101 PIPELINE</small></div>'
            f'<div class=nav>{tab("/", "Datasets", "ds")}{tab("/collect", "Collect", "co")}'
            f'{tab("/train", "Training", "tr")}{tab("/rollout", "Rollout", "ro")}'
            f'{tab("/control", "Control", "ct")}{tab("/calib", "Calib", "cb")}{tab("/setup", "Setup", "st")}'
            f'{tab("/jobs", "Jobs", "jb")}</div>{cluster}'
            f'<button class=fsbtn title="전체화면" aria-label="전체화면" onclick="toggleFS()">'
            f'<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" '
            f'stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
            f'<path d="M8 3H5a2 2 0 0 0-2 2v3M16 3h3a2 2 0 0 1 2 2v3M8 21H5a2 2 0 0 1-2-2v-3M16 21h3a2 2 0 0 0 2-2v-3"/>'
            f'</svg></button></div>')


# ----------------------------- PWA / 키오스크 -------------------------------
@app.get("/manifest.webmanifest")
def api_manifest():
    return JSONResponse({
        "name": "LRWEB — SO-101 Pipeline", "short_name": "LRWEB",
        "start_url": "/", "scope": "/", "display": "fullscreen",
        "orientation": "landscape",
        "background_color": "#0f1216", "theme_color": "#161b21",
        "icons": [
            {"src": "/icon-192.png", "sizes": "192x192", "type": "image/png", "purpose": "any"},
            {"src": "/icon-512.png", "sizes": "512x512", "type": "image/png", "purpose": "any maskable"},
        ],
    }, media_type="application/manifest+json")


@app.get("/icon-{size}.png")
def api_icon(size: int):
    size = max(16, min(1024, size))
    return Response(content=icon_png(size), media_type="image/png")


@app.get("/sw.js")
def api_sw():
    js_src = ("self.addEventListener('install',function(e){self.skipWaiting();});"
              "self.addEventListener('activate',function(e){self.clients.claim();});"
              "self.addEventListener('fetch',function(){});")
    return Response(content=js_src, media_type="application/javascript")


# ----------------------------- 페이지: 데이터셋 -------------------------------
@app.get("/", response_class=HTMLResponse)
def index():
    def _row(d):
        mismatch = ' <span class="badge b-warn">모드 불일치</span>' \
            if d["robot_type"] and d["robot_type"] != robot_name() else ""
        return (f'<tr><td><a href="/ds/{esc(d["name"])}" class=mono>{esc(d["name"])}</a></td>'
                f'<td class=num>{esc(d["episodes"])}</td><td class=num>{esc(d["frames"])}</td>'
                f'<td class=num>{esc(d["fps"])}</td>'
                f'<td class=mono style="color:var(--muted)">{esc(d["robot_type"])}{mismatch}</td>'
                f'<td style="text-align:right">'
                f'<button class=danger onclick="delDs({jsattr(d["name"])})">삭제</button></td></tr>')
    rows = "".join(_row(d) for d in list_datasets())
    empty = ('' if rows else
             '<tr><td colspan=6 class=muted>데이터셋이 없습니다 — Collect 탭에서 수집을 시작하세요</td></tr>')
    return f"""{CSS}{nav_html('ds')}<div class=wrap>
    <p class=eyebrow>Local datasets</p><h2>Datasets</h2>
    <div class=card><table>
    <tr><th>name</th><th class=num>episodes</th><th class=num>frames</th><th class=num>fps</th>
    <th>robot</th><th></th></tr>
    {rows}{empty}</table></div>
    <p class=muted>삭제는 폴더를 통째로 지웁니다 (복구 불가). 데이터셋 이름을 입력해야 실행됩니다.</p></div>
    <script>
    async function delDs(name){{
      const typed = prompt('데이터셋 "'+name+'" 을 통째로 삭제합니다.\\n확인을 위해 이름을 그대로 입력하세요:');
      if(typed !== name){{ if(typed!==null) alert('이름 불일치 — 취소됨'); return; }}
      const r = await fetch('/api/delete_dataset/'+encodeURIComponent(name), {{method:'POST'}});
      const d = await r.json();
      if(d.error) alert(d.error); else location.reload();
    }}
    </script>"""


@app.post("/api/delete_dataset/{ds}")
def api_delete_dataset(ds: str):
    if not safe_name(ds):
        return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
    p = (DATA_ROOT / ds).resolve()
    if not str(p).startswith(str(DATA_ROOT.resolve())) or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    if not (p / "meta/info.json").exists():
        return JSONResponse({"error": "데이터셋 폴더가 아님"}, status_code=400)
    for j in jobs_index():
        if j["alive"] and ds in j.get("cmd", ""):
            return JSONResponse({"error": f"실행 중인 작업({j['id']})이 이 데이터셋을 사용 중"}, status_code=400)
    shutil.rmtree(p)
    m = load_marks()
    m.pop(ds, None)
    save_marks(m)
    return {"ok": True}


@app.get("/ds/{ds}", response_class=HTMLResponse)
def dataset_page(ds: str):
    if not safe_name(ds):
        return HTMLResponse(f"{CSS}{nav_html('ds')}<div class=wrap>잘못된 데이터셋 이름</div>", 400)
    df = episodes_df(ds)
    if df.empty:
        return HTMLResponse(f"{CSS}{nav_html('ds')}<div class=wrap>메타데이터 없음: {esc(ds)}</div>")
    marks = load_marks().get(ds, [])
    rows = []
    for _, r in df.iterrows():
        ep = int(r["episode_index"])
        badge = ' <span class="badge b-bad">bad</span>' if ep in marks else ""
        rows.append(f'<tr><td><a href="/ds/{esc(ds)}/ep/{ep}">episode '
                    f'<span class=mono>{ep}</span></a>{badge}</td>'
                    f'<td class=num>{esc(r.get("length", "?"))}</td>'
                    f'<td>{esc(r.get("tasks", ""))}</td></tr>')
    markbar = ""
    if marks:
        markbar = f"""<div class=markbar>
        <span>마킹된 에피소드 <b class=mono>{esc(sorted(marks))}</b></span>
        <button class=danger onclick="delMarked()">웹에서 바로 삭제 실행</button>
        <span class=muted>결과는 새 폴더로 생성됨 (원본 유지) · Jobs에서 진행 확인</span></div>
        <script>async function delMarked(){{
          if(!confirm('마킹된 {len(marks)}개 에피소드를 삭제 실행할까요?'))return;
          await fetch('/api/delete/'+encodeURIComponent({js(ds)}),{{method:'POST'}}); location.href='/jobs';
        }}</script>"""
    return f"""{CSS}{nav_html('ds')}<div class=wrap>
    <p class=eyebrow>Dataset</p><h2 class=mono style="font-size:17px">{esc(ds)}</h2>
    {markbar}
    <div class=card><table>
    <tr><th>episode</th><th class=num>frames</th><th>task</th></tr>{"".join(rows)}</table></div></div>"""


@app.get("/ds/{ds}/ep/{ep}", response_class=HTMLResponse)
def episode_page(ds: str, ep: int):
    if not safe_name(ds):
        return HTMLResponse(f"{CSS}{nav_html('ds')}<div class=wrap>잘못된 데이터셋 이름</div>", 400)
    df = episodes_df(ds)
    segs = ep_video_segments(df, ep)
    n_ep = int(df["episode_index"].max()) + 1 if not df.empty else 0
    marks = load_marks().get(ds, [])
    if not segs:
        cols = "<br>".join(esc(c) for c in df.columns) if not df.empty else "(none)"
        return HTMLResponse(f"{CSS}{nav_html('ds')}<div class=wrap>영상 세그먼트를 못 찾음.<br>"
                            f"<span class=muted>메타 컬럼: {cols}</span></div>")
    vids = "".join(
        f"""<div class=vpanel>
        <div class=vhead><b>{esc(s['cam'])}</b><span>{s['from']:.1f}s &rarr; {s['to']:.1f}s</span></div>
        <video src="/videos/{esc(ds)}/{esc(s['path'])}" controls muted
               data-from="{s['from']}" data-to="{s['to']}"></video></div>"""
        for s in segs)
    marked = ep in marks
    return f"""{CSS}{nav_html('ds')}<div class=wrap>
    <p class=eyebrow>{esc(ds)}</p>
    <h2>episode <span class=mono style="font-size:inherit">{ep}</span>
      <span class="badge {'b-bad' if marked else 'b-ok'}">{'bad' if marked else 'ok'}</span></h2>
    <div class=toolbar>
       <a href="/ds/{esc(ds)}/ep/{max(ep - 1, 0)}"><button>&larr; prev</button></a>
       <a href="/ds/{esc(ds)}/ep/{min(ep + 1, n_ep - 1)}"><button>next &rarr;</button></a>
       <button class=primary onclick="playAll()">&#9654; 재생</button>
       <button onclick="mark()">{'마킹 해제' if marked else '불량 마킹'}</button>
       <a href="/ds/{esc(ds)}"><button>목록</button></a>
       <span class=muted style="margin-left:auto">episode {ep} / {n_ep - 1} · 단축키 &larr;/&rarr; Space b</span></div>
    <div class=grid>{vids}</div></div>
    <script>
    const DS={js(ds)}, EP={ep}, NEP={n_ep};
    const vids=[...document.querySelectorAll('video')];
    vids.forEach(v=>{{
      v.addEventListener('loadedmetadata',()=>{{v.currentTime=parseFloat(v.dataset.from);}});
      v.addEventListener('timeupdate',()=>{{
        if(v.currentTime>=parseFloat(v.dataset.to)){{v.pause();v.currentTime=parseFloat(v.dataset.from);}}
      }});
    }});
    function playAll(){{vids.forEach(v=>{{v.currentTime=parseFloat(v.dataset.from);v.play();}});}}
    async function mark(){{
      await fetch('/api/mark/'+encodeURIComponent(DS)+'/'+EP,{{method:'POST'}});location.reload();}}
    document.addEventListener('keydown',e=>{{
      if(e.target.tagName==='INPUT')return;
      if(e.key==='ArrowLeft')location.href='/ds/'+encodeURIComponent(DS)+'/ep/'+Math.max(EP-1,0);
      if(e.key==='ArrowRight')location.href='/ds/'+encodeURIComponent(DS)+'/ep/'+Math.min(EP+1,NEP-1);
      if(e.key===' '){{e.preventDefault();playAll();}}
      if(e.key==='b')mark();
    }});
    </script>"""


@app.post("/api/mark/{ds}/{ep}")
def api_mark(ds: str, ep: int):
    if not safe_name(ds):
        return JSONResponse({"error": "잘못된 데이터셋 이름"}, status_code=400)
    m = load_marks()
    lst = set(m.get(ds, []))
    lst.symmetric_difference_update({ep})
    m[ds] = sorted(lst)
    save_marks(m)
    return {"ok": True, "marks": m[ds]}


@app.post("/api/delete/{ds}")
def api_delete(ds: str):
    if not safe_name(ds):
        return JSONResponse({"error": "잘못된 데이터셋 이름"}, status_code=400)
    marks = load_marks().get(ds, [])
    if not marks:
        return JSONResponse({"error": "no marks"}, status_code=400)
    root = DATA_ROOT / ds
    if not root.exists():
        return JSONResponse({"error": "데이터셋 없음"}, status_code=400)
    argv = ["lerobot-edit-dataset",
            "--repo_id", f"local/{ds}",
            "--root", str(root),
            "--operation.type", "delete_episodes",
            "--operation.episode_indices", str(sorted(marks))]
    try:
        jid = start_job("delete", argv)
    except JobStartError as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    m = load_marks()
    m[ds] = []
    save_marks(m)
    return {"ok": True, "job": jid}


# ----------------------------- 페이지: 수집 (Collect) ------------------------
@app.get("/collect", response_class=HTMLResponse)
def collect_page():
    rec = next((j for j in jobs_index() if j["kind"] == "record" and j["alive"]), None)
    if rec:
        return (CSS + nav_html("co") + f"<script>const JID={js(rec['id'])};</script>" + COLLECT_RUN_HTML)
    busy = exclusive_busy()
    busywarn = (f'<p class="badge b-warn">실행 중: {esc(busy["id"])} — 끝나야 수집을 시작할 수 있습니다</p>'
                if busy else "") + setup_needed_html()
    resume_opts = "".join(
        f'<option value="{esc(d["name"])}" {"" if d["robot_type"] in ("", robot_name()) else "disabled"}>'
        f'{esc(d["name"])} ({esc(d["episodes"])}ep{"" if d["robot_type"] in ("", robot_name()) else " · " + esc(d["robot_type"]) + " — 모드 불일치"})</option>'
        for d in list_datasets())
    return f"""{CSS}{nav_html('co')}<div class=wrap>
    <p class=eyebrow>Teleoperation record · {"양팔 bi_so_follower" if BIMANUAL else "한팔 so101_follower"}</p><h2>Collect</h2>
    {busywarn}
    <div class=card>
    <div class=formgrid>
      <label class=f>모드
        <select id=mode onchange="modeSw()">
          <option value=new>새 데이터셋</option>
          <option value=resume>기존에 이어서</option>
        </select></label>
      <label class=f id=f_new>데이터셋 이름
        <input id=name placeholder="pick_place_v2" size=22></label>
      <label class=f id=f_resume style="display:none">이어서 수집할 데이터셋
        <select id=resume_ds>{resume_opts}</select></label>
      <label class=f>에피소드 수 <input id=neps value=50 size=5></label>
      <label class=f>에피소드 최대(초) <input id=ept value=30 size=5></label>
      <label class=f>리셋 최대(초) <input id=rst value=15 size=5></label>
      <label class=f style="flex:1;min-width:260px">태스크 설명
        <input id=task value="{esc(CFG['default_task'])}"></label>
      <button class=primary onclick="startRec()">수집 시작</button>
    </div>
    <p class=muted>시작 즉시 에피소드 0 녹화가 시작됩니다 — 물체·리더암을 먼저 준비하세요.
    시작 후 이 페이지에서 n(다음) / r(재녹화) / q(종료) 버튼 또는 키보드로 조작하고, 카메라 미리보기가 같이 뜹니다.<br>
    카메라: <span class=mono>{esc(", ".join(CAM_SPECS) or "없음 — Setup 탭에서 등록")}</span></p>
    </div></div>
    <script>
    function modeSw(){{
      const m=document.getElementById('mode').value;
      document.getElementById('f_new').style.display = m==='new'?'':'none';
      document.getElementById('f_resume').style.display = m==='resume'?'':'none';
    }}
    async function startRec(){{
      const b={{mode:document.getElementById('mode').value,
        name:document.getElementById('name').value,
        resume_ds:document.getElementById('resume_ds')?.value||'',
        num_episodes:document.getElementById('neps').value,
        episode_time_s:document.getElementById('ept').value,
        reset_time_s:document.getElementById('rst').value,
        task:document.getElementById('task').value}};
      const r=await fetch('/api/record',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(b)}});
      const d=await r.json(); if(d.error)alert(d.error); else location.reload();
    }}
    </script>"""


@app.post("/api/record")
async def api_record(req: Request):
    busy = exclusive_busy()   # control 포함 — Control 탭이 팔을 잡고 있으면 차단
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 시작하세요"}, status_code=400)
    b = await req.json()
    task = (b.get("task") or CFG["default_task"]).strip()
    neps = _clamp_int(b.get("num_episodes"), 50, 1, 100000)
    ept = _clamp_int(b.get("episode_time_s"), 30, 1, 3600)
    rst = _clamp_int(b.get("reset_time_s"), 15, 0, 3600)
    try:
        _need_ports("follower")
        _need_ports("leader")
        if BIMANUAL:
            bimanual_base_id("follower")
            bimanual_base_id("leader")
    except (NotImplementedError, ValueError) as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    missing = [f"{side}/{role}" for side, roles in calib_status().items()
               for role, c in roles.items() if not c["ok"]]
    if missing:
        return JSONResponse({"error": "캘리브레이션 없음: " + ", ".join(missing) + " — Calib 탭에서 먼저"},
                            status_code=400)
    if not CAM_SPECS:
        return JSONResponse({"error": "카메라가 등록되지 않았습니다 — Setup 탭에서 추가하세요"}, status_code=400)
    if b.get("mode") == "resume":
        ds = (b.get("resume_ds") or "").strip()
        if not safe_name(ds):
            return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
        root = DATA_ROOT / ds
        if not (root / "meta/info.json").exists():
            return JSONResponse({"error": "데이터셋 없음"}, status_code=400)
        rt = load_json(root / "meta/info.json", {}).get("robot_type", "")
        if rt and rt != robot_name():
            return JSONResponse({"error": f"데이터셋은 {rt} 로 수집됨 — 현재 모드({robot_name()})와 다릅니다"},
                                status_code=400)
        resume, name = True, ds
    else:
        name = (b.get("name") or "").strip()
        if not safe_name(name):
            return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
        if (DATA_ROOT / name).exists():
            return JSONResponse({"error": f"이미 있는 데이터셋: {name} — 다른 이름을 쓰거나 '기존에 이어서' 선택"},
                                status_code=400)
        resume = False
    spec = {"mode": CFG["mode"], "arms": json.loads(json.dumps(CFG["arms"])),
            "cameras": json.loads(json.dumps(CFG["cameras"])), "fps": int(CFG["fps"]),
            "task": task, "num_episodes": neps, "episode_time_s": ept, "reset_time_s": rst,
            "repo_id": f"local/{name}", "root": str(DATA_ROOT / name), "resume": resume,
            "streaming_encoding": bool(CFG.get("streaming_encoding", False))}
    try:
        jid = start_job("record", [sys.executable, str(Path(__file__).resolve()), "--worker", "record", "{jid}"],
                        spec=spec)
    except JobStartError as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    return {"ok": True, "job": jid}


@app.get("/api/record_status/{jid}")
def api_record_status(jid: str):
    if not safe_name(jid):
        return JSONResponse({"error": "잘못된 작업 id"}, status_code=400)
    j = load_json(JOB_DIR / f"{jid}.json", {})
    return JSONResponse({"status": record_status(jid), "tail": log_tail(jid),
                         "alive": pid_alive(j.get("pid"))})


@app.post("/api/sendkey/{jid}/{key}")
def api_sendkey(jid: str, key: str):
    if key not in ("n", "r", "q"):
        return JSONResponse({"error": "허용되지 않은 키"}, status_code=400)
    ok = send_cmd(jid, key)
    return {"ok": ok} if ok else JSONResponse({"error": "키 전달 실패"}, status_code=400)


@app.get("/api/joblog/{jid}")
def api_joblog(jid: str):
    if not safe_name(jid):
        return JSONResponse({"error": "잘못된 작업 id"}, status_code=400)
    j = load_json(JOB_DIR / f"{jid}.json", {})
    return JSONResponse({"tail": log_tail(jid), "alive": pid_alive(j.get("pid"))})


# ----------------------------- 페이지: 학습 ----------------------------------
@app.get("/train", response_class=HTMLResponse)
def train_page():
    ds_opts = "".join(f'<option value="{esc(d["name"])}">{esc(d["name"])} ({esc(d["episodes"])}ep)</option>'
                      for d in list_datasets())
    running = [j for j in jobs_index() if j["kind"] == "train" and j["alive"]]
    run_html = "".join(
        f'<div class=runbar><span class="badge b-run">running</span> '
        f'<span class=mono>{esc(j["id"])}</span> '
        f'<button class=danger onclick="stopJob({jsattr(j["id"])})">중지</button></div>'
        for j in running)
    return f"""{CSS}{nav_html('tr')}<div class=wrap>
    <p class=eyebrow>ACT policy</p><h2>Training</h2>
    {run_html}
    <form class=row onsubmit="startTrain(event)">
      <select id=ds>{ds_opts}</select>
      <input id=name placeholder="출력 이름 (예: act_pick_place_v2)" size=26>
      <input id=steps value=80000 size=7> <span class=muted>steps</span>
      <input id=batch value=8 size=3> <span class=muted>batch</span>
      <button class=primary>학습 시작</button>
    </form>
    <div class="muted mono" id=which style="margin-bottom:8px"></div>
    <div class=chartbox><canvas id=chart height=90></canvas></div>
    <p class=eyebrow>Log tail</p><pre id=tail>...</pre></div>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script>
    async function startTrain(e){{
      e.preventDefault();
      const b={{dataset:ds.value,name:document.getElementById('name').value,
               steps:document.getElementById('steps').value,batch:document.getElementById('batch').value}};
      const r=await fetch('/api/train',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(b)}});
      const d=await r.json(); if(d.error)alert(d.error); else location.reload();
    }}
    async function stopJob(id){{
      if(!confirm('학습을 중지할까요? (한 번 더 누르면 강제종료)'))return;
      await fetch('/api/kill/'+id,{{method:'POST'}}); setTimeout(()=>location.reload(),1500);
    }}
    let chart;
    async function refresh(){{
      const r=await fetch('/api/trainlog'); const d=await r.json();
      document.getElementById('which').textContent=d.current||'로그 없음';
      document.getElementById('tail').textContent=d.tail||'';
      const xs=d.points.map(p=>p[0]),ys=d.points.map(p=>p[1]);
      if(!chart){{chart=new Chart(document.getElementById('chart'),{{type:'line',
        data:{{labels:xs,datasets:[{{label:'loss',data:ys,borderColor:'#5d9dd6',
          backgroundColor:'rgba(93,157,214,.08)',fill:true,pointRadius:0,borderWidth:1.5}}]}},
        options:{{animation:false,
          scales:{{y:{{type:'logarithmic',grid:{{color:'#28303a'}},ticks:{{color:'#8b98a7',font:{{family:'IBM Plex Mono',size:11}}}}}},
                   x:{{grid:{{display:false}},ticks:{{color:'#5d6a79',font:{{family:'IBM Plex Mono',size:10}},maxTicksLimit:10}}}}}},
          plugins:{{legend:{{display:false}}}}}}}});}}
      else{{chart.data.labels=xs;chart.data.datasets[0].data=ys;chart.update();}}
    }}
    refresh(); setInterval(refresh,5000);
    </script>"""


@app.post("/api/train")
async def api_train(req: Request):
    b = await req.json()
    busy = gpu_or_loop_busy()   # Control 수동 제어는 학습과 동시 가능
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 시작하세요"}, status_code=400)
    ds = (b.get("dataset") or "").strip()
    if not safe_name(ds):
        return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
    name = (b.get("name") or f"act_{ds}").strip()
    if not safe_name(name):
        return JSONResponse({"error": "출력 이름은 영문/숫자/._- 만"}, status_code=400)
    steps = _clamp_int(b.get("steps"), 80000, 1, 100000000)
    batch = _clamp_int(b.get("batch"), 8, 1, 4096)
    root = DATA_ROOT / ds
    if not root.exists():
        return JSONResponse({"error": "dataset not found"}, status_code=400)
    out = OUT_ROOT / name
    argv = ["python", "-m", "lerobot.scripts.lerobot_train",
            f"--dataset.repo_id=local/{ds}", f"--dataset.root={root}",
            "--policy.type=act", f"--output_dir={out}",
            f"--steps={steps}", f"--batch_size={batch}", "--num_workers=4",
            "--save_freq=10000", "--policy.push_to_hub=false"]
    try:
        jid = start_job("train", argv)
    except JobStartError as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    return {"ok": True, "job": jid}


_BIG_SUFFIX = {"": 1, "K": 1_000, "M": 1_000_000, "B": 1_000_000_000}


def _parse_step(tok):
    """lerobot 로그의 step 은 format_big_number 출력 — '80K', '1M', '900' 등."""
    m = re.fullmatch(r"([\d.]+)([KMB]?)", tok)
    if not m:
        return None
    try:
        return int(float(m.group(1)) * _BIG_SUFFIX[m.group(2)])
    except (ValueError, KeyError):
        return None


@app.get("/api/trainlog")
def api_trainlog():
    trains = [j for j in jobs_index() if j["kind"] == "train"]
    if not trains:
        return JSONResponse({"points": [], "tail": "", "current": None})
    j = next((t for t in trains if t["alive"]), trains[0])
    pts = []
    try:
        with open(j["log"], errors="ignore") as f:
            for line in f:
                m = re.search(r"step:(\S+)\s.*?loss:([\d.]+)", line)
                if m:
                    step = _parse_step(m.group(1))
                    if step is None:
                        continue
                    try:
                        pts.append([step, float(m.group(2))])
                    except ValueError:
                        pass
    except Exception:
        pass
    status = "running" if pid_alive(j["pid"]) else "finished"
    return JSONResponse({"points": pts[-2000:], "tail": log_tail(j["id"]),
                         "current": f'{j["id"]} [{status}]'})


# ----------------------------- 페이지: 추론 (Rollout) ------------------------
@app.get("/rollout", response_class=HTMLResponse)
def rollout_page():
    ro = next((j for j in jobs_index() if j["kind"] == "rollout" and j["alive"]), None)
    if ro:
        return f"""{CSS}{nav_html('ro')}<div class=wrap>
        <p class=eyebrow>Autonomous</p><h2>추론 실행 중</h2>
        <div class=runbar><span class="badge b-run">rollout</span>
          <span class=mono>{esc(ro["id"])}</span>
          <button class=danger onclick="stopRo()">중지</button>
          <span class=muted>중지(SIGINT) 시 시작 자세로 복귀 후 토크 해제됩니다</span></div>
        <p class=eyebrow>Log</p><pre id=tail>...</pre></div>
        <script>
        const JID={js(ro["id"])};
        async function stopRo(){{
          if(!confirm('추론을 중지할까요?'))return;
          await fetch('/api/kill/'+JID,{{method:'POST'}}); setTimeout(()=>location.reload(),1500);
        }}
        async function refresh(){{
          const r=await fetch('/api/joblog/'+JID); const d=await r.json();
          document.getElementById('tail').textContent=d.tail||'';
          if(!d.alive) location.reload();
        }}
        refresh(); setInterval(refresh,2000);
        </script>"""
    busy = exclusive_busy()
    busywarn = (f'<p class="badge b-warn">실행 중: {esc(busy["id"])} — 끝나야 추론을 시작할 수 있습니다</p>'
                if busy else "") + setup_needed_html()
    ckpts = list_checkpoints()
    ck_opts = ""
    for c in ckpts:
        rt = checkpoint_robot_type(c)
        bad = bool(rt) and rt != robot_name()
        ck_opts += (f'<option value="{esc(c)}" {"disabled" if bad else ""}>{esc(c)}'
                    f'{" · " + esc(rt) + " — 모드 불일치" if bad else (" · " + esc(rt) if rt else "")}</option>')
    empty = "" if ckpts else '<p class=muted>체크포인트가 없습니다 — Training에서 학습을 먼저 완료하세요</p>'
    return f"""{CSS}{nav_html('ro')}<div class=wrap>
    <p class=eyebrow>Autonomous run · {"양팔 bi_so_follower" if BIMANUAL else "한팔 so101_follower"}</p><h2>Rollout</h2>
    {busywarn}{empty}
    <div class=card>
    <div class=formgrid>
      <label class=f style="min-width:380px">체크포인트
        <select id=ckpt>{ck_opts}</select></label>
      <label class=f>실행 시간(초, 0=무한) <input id=dur value=60 size=6></label>
      <label class=f style="flex:1;min-width:260px">태스크 설명
        <input id=task value="{esc(CFG['default_task'])}"></label>
      <button class=primary onclick="startRo()" {'disabled' if not ckpts else ''}>추론 시작</button>
    </div>
    <p class=muted>시작 즉시 팔이 움직입니다 — 팔 주변을 비우고, 물체를 시연 위치에 놓으세요.
    카메라 배치는 학습 데이터 수집 때와 동일해야 합니다.</p>
    <div style="margin-top:12px;display:flex;gap:8px;align-items:center;flex-wrap:wrap">
      <button class=danger onclick="delCkpt('step')" {'disabled' if not ckpts else ''}>선택 체크포인트 삭제</button>
      <button class=danger onclick="delCkpt('run')" {'disabled' if not ckpts else ''}>출력 전체 삭제</button>
      <span class=muted>선택 = 해당 step 폴더만 · 출력 전체 = outputs/&lt;run&gt; 통째 (복구 불가)</span>
    </div>
    </div></div>
    <script>
    async function startRo(){{
      if(!confirm('팔이 즉시 자율 구동됩니다. 주변이 안전한가요?'))return;
      const b={{ckpt:document.getElementById('ckpt').value,
               duration:document.getElementById('dur').value,
               task:document.getElementById('task').value}};
      const r=await fetch('/api/rollout',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(b)}});
      const d=await r.json(); if(d.error)alert(d.error); else location.reload();
    }}
    async function delCkpt(scope){{
      const rel=document.getElementById('ckpt').value;
      if(!rel){{alert('선택된 체크포인트가 없습니다');return;}}
      const run=rel.split('/checkpoints/')[0];
      if(scope==='run'){{
        const typed=prompt('출력 "'+run+'" 을 통째로 삭제합니다 (복구 불가).\\n확인을 위해 이름을 그대로 입력하세요:');
        if(typed!==run)return;
      }}else{{
        if(!confirm('체크포인트 삭제:\\n'+rel.replace('/pretrained_model','')+'\\n삭제할까요?'))return;
      }}
      const r=await fetch('/api/delete_checkpoint',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify({{rel:rel,scope:scope}})}});
      const d=await r.json(); if(d.error)alert(d.error); else location.reload();
    }}
    </script>"""


@app.post("/api/rollout")
async def api_rollout(req: Request):
    busy = exclusive_busy()
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 시작하세요"}, status_code=400)
    b = await req.json()
    rel = b.get("ckpt", "")
    ck = (OUT_ROOT / rel).resolve()
    if not str(ck).startswith(str(OUT_ROOT.resolve())) or not ck.is_dir():
        return JSONResponse({"error": "체크포인트 없음"}, status_code=400)
    dur = _clamp_int(b.get("duration"), 60, 0, 86400)
    task = (b.get("task") or CFG["default_task"]).strip()
    rt = checkpoint_robot_type(rel)
    if rt and rt != robot_name():
        return JSONResponse({"error": f"체크포인트는 {rt} 데이터로 학습됨 — 현재 모드({robot_name()})와 다릅니다"},
                            status_code=400)
    try:
        argv = (["python", "-m", "lerobot.scripts.lerobot_rollout", f"--policy.path={ck}"] + robot_cli_args()
                + ["--strategy.type=base", f"--duration={dur}", f"--task={task}"])
    except (NotImplementedError, ValueError) as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    try:
        jid = start_job("rollout", argv)
    except JobStartError as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    return {"ok": True, "job": jid}


@app.post("/api/delete_checkpoint")
async def api_delete_checkpoint(req: Request):
    b = await req.json()
    rel = b.get("rel", "")
    scope = b.get("scope", "step")
    if "/checkpoints/" not in rel:
        return JSONResponse({"error": "체크포인트 경로 아님"}, status_code=400)
    run = rel.split("/checkpoints/")[0]
    run_path = str((OUT_ROOT / run).resolve())
    for j in jobs_index():
        if j["alive"] and run_path in j.get("cmd", ""):
            return JSONResponse({"error": f"실행 중인 작업({j['id']})이 이 출력을 사용 중"}, status_code=400)
    if scope == "run":
        target = (OUT_ROOT / run).resolve()
    else:
        step = rel.split("/checkpoints/")[1].split("/")[0]
        if step == "last":
            return JSONResponse({"error": "last는 심볼릭 링크 — 숫자 체크포인트를 선택하세요"}, status_code=400)
        target = (OUT_ROOT / run / "checkpoints" / step).resolve()
    if not str(target).startswith(str(OUT_ROOT.resolve())) or not target.exists():
        return JSONResponse({"error": "대상 없음"}, status_code=400)
    if target.is_symlink():
        return JSONResponse({"error": "심볼릭 링크는 삭제하지 않음"}, status_code=400)
    shutil.rmtree(target)
    return {"ok": True}


# ----------------------------- 페이지: Control (수동 제어) --------------------
@app.get("/control", response_class=HTMLResponse)
def control_page():
    busy = busy_with(("record", "rollout")) or (
        {"id": "port-watch (Setup 탭)"} if WATCH.on else None) or (
        {"id": "calibration (Calib 탭)"} if CALIB.active else None) or (
        {"id": "motor-id-setup (Setup 탭)"} if MOTORSETUP.active else None)
    if busy:
        return f"""{CSS}{nav_html('ct')}<div class=wrap>
        <p class=eyebrow>Manual control</p><h2>Control</h2>
        <p class="badge b-warn">실행 중: {esc(busy["id"])} — 끝나야 수동 제어를 쓸 수 있습니다</p></div>"""
    if not ports_configured():
        return f"""{CSS}{nav_html('ct')}<div class=wrap>
        <p class=eyebrow>Manual control</p><h2>Control</h2>
        {setup_needed_html()}</div>"""
    cam_panels = "".join(
        f'<div class=cw><span class=cl>{esc(n)}</span><img id="cam_{esc(n)}"></div>'
        for n in CAM_SPECS)
    if not CAM_SPECS:
        cam_panels = ""
    # f-string 표현식 안에서는 {{ 가 이스케이프가 아니라 실제 중괄호라 집합이 됩니다.
    # dict 리터럴은 f-string 밖에서 만들어야 합니다.
    views_js = js({sd: (a.get("view") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0})
                   for sd, a in ARM_CFGS.items()})
    arm_panels = "".join(
        f'<div class=armbox data-side="{esc(s)}">'
        f'{"<div class=armhead>" + esc(s) + "</div>" if BIMANUAL else ""}'
        f'<div class=sliders id="sl_{esc(s)}"></div></div>'
        for s in SIDES)
    return f"""{CSS}{nav_html('ct')}
<style>
.cmain{{display:grid;grid-template-columns:360px 1fr;gap:0;height:calc(100vh - 52px)}}
.cpanel{{border-right:1px solid var(--line);padding:18px;overflow-y:auto}}
.armhead{{font-family:var(--mono);font-size:11px;letter-spacing:.16em;text-transform:uppercase;
color:var(--dim);margin:14px 0 8px;border-bottom:1px solid var(--line);padding-bottom:4px}}
.jrow{{margin-bottom:16px}}
.jhead{{display:flex;justify-content:space-between;font-family:var(--mono);font-size:12px;margin-bottom:4px}}
.jhead .n{{color:var(--text)}} .jhead .v{{color:var(--accent)}} .jhead .a{{color:var(--dim)}}
input[type=range]{{width:100%;accent-color:var(--accent)}}
#right{{display:flex;flex-direction:column;min-height:0}}
.cams{{display:none;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:1px;
background:var(--line);border-bottom:1px solid var(--line)}}
.cams .cw{{position:relative;background:#000}}
.cams img{{width:100%;display:block;max-height:220px;object-fit:contain;background:#000}}
.cams .cl{{position:absolute;top:6px;left:10px;font-family:var(--mono);font-size:11px;
letter-spacing:.1em;text-transform:uppercase;color:#cfd8e3;text-shadow:0 0 4px #000}}
#view{{position:relative;background:#0a0d10;min-height:300px;flex:1}}
#view canvas{{display:block}}
#nourdf{{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
color:var(--dim);font-family:var(--mono);font-size:12px;text-align:center;line-height:2}}
button.estop{{background:#4a2020;border-color:var(--bad);color:#ffc9c9;font-family:var(--mono);font-weight:600}}
@media(max-width:820px){{
  .cmain{{grid-template-columns:1fr;height:auto}}
  .cpanel{{border-right:none;border-bottom:1px solid var(--line);overflow:visible}}
  #right{{min-height:auto}}
  #view{{min-height:56vh}}
}}
</style>
<div class=cmain>
  <div class=cpanel>
    <div class=toolbar>
      <span id=cst class=badge>connecting…</span>
      <button id=btrq onclick="toggleTorque()" disabled>토크 ON</button>
      <button id=bflw onclick="toggleFollow()" disabled>리더 팔로우 ON</button>
      <button class=estop onclick="estop()">E-STOP</button>
      <input type=color id=armcolor value="#e07a3f" title="로봇 색상"
             style="width:34px;height:30px;padding:2px;border-radius:7px;border:1px solid var(--line);background:var(--bg);cursor:pointer">
    </div>
    {arm_panels}
    <p class=muted>토크 OFF: 손으로 움직이면 값·3D가 따라옵니다.<br>
    토크 ON: 슬라이더가 목표 (스텝당 최대 {MAX_STEP_DEG}° 제한).<br>
    리더 팔로우: 리더 암을 손으로 움직이면 팔로워가 실시간 미러링 (슬라이더 잠금).<br>
    이 탭을 떠나면 자동으로 토크 해제 + 연결 해제됩니다.</p>
  </div>
  <div id=right>
    <div class=cams id=cams>{cam_panels}</div>
    <div id=view><div id=nourdf>{esc(URDF_DIR)}/so101.urdf 없음<br>
    URDF와 meshes/ 를 복사하면 3D 표시<br>(슬라이더 제어는 그대로 동작)</div></div>
  </div>
</div>
<script type="importmap">
{{"imports":{{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/",
"three/examples/jsm/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/",
"urdf-loader":"https://cdn.jsdelivr.net/npm/urdf-loader@0.12.6/src/URDFLoader.js"}}}}
</script>
<script type="module">
const JOINTS = {js(CTL_JOINTS)};
const SIDES  = {js(SIDES)};
const VIEWS  = {views_js};
const CAMS   = {js(list(CAM_SPECS))};
let ws=null, torque=false, follow=false;
const robots={{}};                       // side -> URDF root
const sliders={{}}, valEls={{}}, actEls={{}};   // side -> joint -> el
const cst=document.getElementById('cst'), btrq=document.getElementById('btrq');
const bflw=document.getElementById('bflw');

function buildSliders(side, lims, actual){{
  const box=document.getElementById('sl_'+side); box.innerHTML='';
  sliders[side]={{}}; valEls[side]={{}}; actEls[side]={{}};
  JOINTS.forEach(j=>{{
    const [lo,hi]=lims[j];
    const row=document.createElement('div'); row.className='jrow';
    row.innerHTML=`<div class=jhead><span class=n>${{j}}</span>
      <span><span class=v>-</span> <span class=a>(-)</span></span></div>
      <input type=range min=${{lo}} max=${{hi}} step=0.5 value=${{actual[j]??0}}>`;
    box.appendChild(row);
    const s=row.querySelector('input');
    sliders[side][j]=s;
    valEls[side][j]=row.querySelector('.v');
    actEls[side][j]=row.querySelector('.a');
    s.addEventListener('input',()=>{{
      if(ws&&ws.readyState===1){{
        const joints={{}}; JOINTS.forEach(k=>joints[k]=parseFloat(sliders[side][k].value));
        ws.send(JSON.stringify({{type:'target',side:side,joints}}));
      }}
    }});
  }});
}}
window.toggleTorque=()=>{{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({{type:'torque',on:!torque}})); }};
window.toggleFollow=()=>{{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({{type:'follow',on:!follow}})); }};
window.estop=()=>{{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({{type:'estop'}})); }};

function openWS(){{
  ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws/control');
  ws.onmessage=e=>{{
    const d=JSON.parse(e.data);
    if(d.type==='init'){{
      if(d.error){{ cst.textContent=d.error; cst.classList.add('b-bad'); return; }}
      SIDES.forEach(s=>buildSliders(s, d.arms[s].limits, d.arms[s].actual));
      cst.textContent='connected'; cst.classList.add('b-ok');
      btrq.disabled=false; bflw.disabled=false;
      if(d.cams&&d.cams.length){{
        document.getElementById('cams').style.display='grid';
        d.cams.forEach(n=>{{
          const img=document.getElementById('cam_'+n);
          if(img) img.src='/stream/'+encodeURIComponent(n);
        }});
      }}
    }}
    if(d.type==='state'){{
      torque=d.torque; follow=!!d.follow;
      btrq.textContent=torque?'토크 OFF':'토크 ON';
      btrq.disabled=follow;
      bflw.textContent=follow?'리더 팔로우 OFF':'리더 팔로우 ON';
      bflw.classList.toggle('primary',follow);
      SIDES.forEach(side=>{{
        const a=d.arms[side]; if(!a||!sliders[side])return;
        JOINTS.forEach(j=>{{
          if(valEls[side][j])valEls[side][j].textContent=(a.target[j]??0).toFixed(1);
          if(actEls[side][j])actEls[side][j].textContent='('+(a.actual[j]??0).toFixed(1)+')';
          const s=sliders[side][j]; if(!s)return;
          s.disabled=follow;
          if(follow){{
            if(a.target[j]!==undefined)s.value=a.target[j];
          }}else{{
            if(!torque&&a.actual[j]!==undefined)s.value=a.actual[j];
            if(torque&&d.synced&&a.actual[j]!==undefined)s.value=a.actual[j];
          }}
        }});
        updateRobot(side,a.actual);
      }});
      if(d.err){{cst.textContent='bus error';cst.classList.add('b-bad');}}
    }}
  }};
  ws.onclose=()=>{{ cst.textContent='disconnected'; cst.classList.remove('b-ok');
                   btrq.disabled=true; bflw.disabled=true; }};
}}
addEventListener('pagehide',()=>{{ try{{ws&&ws.close();}}catch(e){{}} }});
openWS();

// ---------- Three.js URDF ----------
let updateRobot=()=>{{}};
function showViewMsg(t){{
  const el=document.getElementById('nourdf');
  el.style.display='flex'; el.innerHTML=t;
}}
(async()=>{{
  const st=await (await fetch('/api/ctlstate')).json();
  if(!st.urdf) return;
  try{{
  document.getElementById('nourdf').style.display='none';
  const THREE=await import('three');
  const {{OrbitControls}}=await import('three/addons/controls/OrbitControls.js');
  const URDFLoader=(await import('urdf-loader')).default;
  const view=document.getElementById('view');
  const scene=new THREE.Scene(); scene.background=new THREE.Color(0x0a0d10);
  const cam=new THREE.PerspectiveCamera(50,1,0.01,10);
  const zoom=SIDES.length>1?1.7:1.0;
  cam.position.set(0.4*zoom,0.35*zoom,0.4*zoom);
  const ren=new THREE.WebGLRenderer({{antialias:true}}); view.appendChild(ren.domElement);
  const ctl=new OrbitControls(cam,ren.domElement); ctl.target.set(0,0.12,0);
  scene.add(new THREE.HemisphereLight(0xffffff,0x223344,1.1));
  const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(1,2,1); scene.add(dl);
  scene.add(new THREE.GridHelper(SIDES.length>1?1.6:1, SIDES.length>1?32:20, 0x28303a,0x1b222a));
  function resize(){{const w=view.clientWidth,h=view.clientHeight;ren.setSize(w,h);cam.aspect=w/h;cam.updateProjectionMatrix();}}
  new ResizeObserver(resize).observe(view); resize();
  const picker=document.getElementById('armcolor');
  picker.value=localStorage.getItem('armColor')||'#e07a3f';
  function applyColor(hex){{
    Object.values(robots).forEach(r=>r.traverse(o=>{{
      if(o.isMesh){{
        if(!o.userData.recolored){{
          o.material=new THREE.MeshStandardMaterial({{metalness:0.15,roughness:0.55}});
          o.userData.recolored=true;
        }}
        o.material.color.set(hex);
      }}
    }}));
    localStorage.setItem('armColor',hex);
  }}
  picker.addEventListener('input',()=>applyColor(picker.value));
  SIDES.forEach((side,i)=>{{
    const loader=new URDFLoader();
    loader.workingPath='/urdf/';
    loader.packages='/urdf';           // package://xxx/ 형태도 /urdf/로 해석
    loader.load('/urdf/so101.urdf',
      r=>{{
        // URDF 는 Z-up / X-forward. -90° 눕히면 URDF X → three X(앞), URDF Y → three -Z(왼쪽).
        // Euler 'XYZ' 는 R = Rx·Ry·Rz 라 z 성분이 먼저 적용됨 → URDF 기준 yaw 가 됩니다.
        const v=VIEWS[side]||{{x:0,y:0,yaw_deg:0}};
        r.rotation.set(-Math.PI/2, 0, v.yaw_deg*Math.PI/180);
        r.position.set(v.x, 0, -v.y);        // 앞뒤=X, 좌우=−Z
        robots[side]=r; scene.add(r); applyColor(picker.value);
      }},
      undefined,
      e=>{{console.error(e);showViewMsg('URDF 로드 실패<br>'+(e?.message||e));}});
  }});
  (function anim(){{requestAnimationFrame(anim);ctl.update();ren.render(scene,cam);}})();
  updateRobot=(side,actual)=>{{
    const robot=robots[side];
    if(!robot||!actual)return;
    JOINTS.forEach(j=>{{
      const jt=robot.joints?.[j]; if(!jt)return;
      const v=actual[j]; if(v===undefined)return;
      if(j==='gripper'){{
        const lo=jt.limit?.lower??0,hi=jt.limit?.upper??1;
        jt.setJointValue(lo+(hi-lo)*(v/100));
      }}else jt.setJointValue(v*Math.PI/180);
    }});
  }};
  }}catch(e){{ console.error(e); showViewMsg('3D 초기화 실패<br>'+(e?.message||e)); }}
}})();
</script>"""


@app.get("/api/ctlstate")
def api_ctlstate():
    return {"connected": any_arm_connected(),
            "torque": all(a.torque for a in ARMS.values()) and any_arm_connected(),
            "sides": SIDES,
            "cams": list(CAM_SPECS),
            "urdf": (URDF_DIR / "so101.urdf").exists()}


def _arms_state():
    return {s: {"actual": a.actual, "target": a.target, "limits": a.limits}
            for s, a in ARMS.items()}


@app.websocket("/ws/control")
async def ws_control(sock: WebSocket):
    global CTL_OWNER
    await sock.accept()
    if not ws_authed(sock):
        await sock.send_text(json.dumps({"type": "init", "error": "인증 필요 — 페이지를 새로고침하세요"}))
        await sock.close()
        return
    if busy_with(("record", "rollout")) or WATCH.on or CALIB.active or MOTORSETUP.active:
        await sock.send_text(json.dumps({"type": "init", "error": "record/rollout/Setup/Calib 사용 중 — 제어 불가"}))
        await sock.close()
        return
    if CTL_OWNER is not None:
        await sock.send_text(json.dumps({"type": "init", "error": "다른 브라우저가 제어 중입니다"}))
        await sock.close()
        return
    CTL_OWNER = sock
    try:
        try:
            for arm in ARMS.values():
                if not arm.connected:
                    await asyncio.to_thread(arm.connect)
        except Exception as e:
            for arm in ARMS.values():
                arm.disconnect()
            await sock.send_text(json.dumps({"type": "init", "error": f"팔 연결 실패: {e}"}))
            return

        await asyncio.to_thread(CAMS.open)
        await sock.send_text(json.dumps({
            "type": "init", "arms": _arms_state(),
            "cams": list(CAMS.cams),      # 실제로 열린 카메라만
        }))

        synced = False   # 토크 토글 직후 슬라이더 동기화 신호 1회

        def _set_torque_all(on):
            for a in ARMS.values():
                a.set_torque(on)

        def _leaders_off():
            for side, ldr in LEADERS.items():
                ARMS[side].follow = False
                ldr.disconnect()

        def _follow_on():
            for side, ldr in LEADERS.items():
                if not ldr.connected:
                    ldr.connect()
                if not ARMS[side].torque:
                    ARMS[side].set_torque(True)
                ARMS[side].follow = True

        for side, arm in ARMS.items():
            arm.start_loop(LEADERS[side])

        async def rx():
            nonlocal synced
            async for msg in sock.iter_text():
                try:
                    d = json.loads(msg)
                except ValueError:
                    continue
                kind = d.get("type")
                if kind == "target":
                    arm = ARMS.get(d.get("side"))
                    if arm:
                        for k, v in (d.get("joints") or {}).items():
                            if k in CTL_JOINTS:
                                try:
                                    arm.target[k] = float(v)
                                except (TypeError, ValueError):
                                    pass
                elif kind == "torque":
                    try:
                        await asyncio.to_thread(_set_torque_all, bool(d.get("on")))
                        synced = True
                    except Exception as e:
                        _first_arm().err = str(e)
                elif kind == "estop":
                    try:
                        await asyncio.to_thread(_leaders_off)
                        await asyncio.to_thread(_set_torque_all, False)
                    except Exception as e:
                        _first_arm().err = str(e)
                elif kind == "follow":
                    on = bool(d.get("on"))
                    try:
                        if on:
                            await asyncio.to_thread(_follow_on)
                        else:
                            await asyncio.to_thread(_leaders_off)
                        synced = True
                    except Exception as e:
                        _first_arm().err = str(e)
                        try:
                            await asyncio.to_thread(_leaders_off)
                        except Exception:
                            pass

        rx_task = asyncio.create_task(rx())
        try:
            while True:
                t0 = time.monotonic()
                any_arm = _first_arm()
                err = next((a.err for a in ARMS.values() if a.err), "")
                await sock.send_text(json.dumps({
                    "type": "state", "arms": _arms_state(),
                    "torque": any_arm.torque, "follow": any_arm.follow,
                    "synced": synced, "err": err,
                }))
                synced = False
                await asyncio.sleep(max(0.0, 1.0 / CONTROL_HZ - (time.monotonic() - t0)))
        finally:
            rx_task.cancel()
    except WebSocketDisconnect:
        pass
    finally:
        CTL_OWNER = None
        CAMS.close()                      # 탭 이탈 = 카메라 해제
        for arm in ARMS.values():
            arm.stop_loop()
        for ldr in LEADERS.values():
            ldr.disconnect()
        for arm in ARMS.values():
            arm.disconnect()              # + 토크 해제 + 시리얼 해제


def _first_arm():
    return ARMS[SIDES[0]]


# Control 카메라 MJPEG 스트림
def _mjpeg_from_files(jid, cam):
    """record worker 가 RUN_DIR 에 떨어뜨리는 JPEG 을 mtime 이 바뀔 때마다 흘려보냅니다."""
    boundary = b"--frame"
    f = run_dir(jid) / f"cam_{cam}.jpg"
    jf = JOB_DIR / f"{jid}.json"

    def gen():
        last_m, last_sent = 0.0, 0.0
        idle = 0
        while True:
            try:
                m = f.stat().st_mtime
            except OSError:
                m = 0.0
            now = time.monotonic()
            if m and (m != last_m or now - last_sent > 1.0):
                try:
                    data = f.read_bytes()
                except OSError:
                    data = b""
                if data:
                    last_m, last_sent = m, now
                    yield (boundary + b"\r\nContent-Type: image/jpeg\r\n"
                           + f"Content-Length: {len(data)}\r\n\r\n".encode() + data + b"\r\n")
            idle += 1
            if idle % 20 == 0 and not pid_alive(load_json(jf, {}).get("pid")):
                break
            time.sleep(1.0 / PREVIEW_FPS)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/stream/{cam}")
def stream_cam(cam: str):
    if not CAMS.on:
        rec = next((j for j in jobs_index() if j["kind"] == "record" and j["alive"]), None)
        if rec and (cam in CAM_SPECS or safe_name(cam)):
            return _mjpeg_from_files(rec["id"], cam)
        return JSONResponse({"error": "카메라 미가동 (Control 탭 또는 수집 중에만 스트리밍)"}, status_code=503)
    if cam not in CAM_SPECS:
        return JSONResponse({"error": "unknown camera"}, status_code=404)
    boundary = b"--frame"

    def gen():
        last, last_sent = None, 0.0
        while CAMS.on:
            f = CAMS.frames.get(cam)
            now = time.monotonic()
            # 새 프레임이면 보내고, 카메라가 멈춰도 1초마다 한 번은 재전송해
            # 브라우저가 연결을 끊지 않게 합니다.
            if f is not None and (f is not last or now - last_sent > 1.0):
                last, last_sent = f, now
                yield (boundary + b"\r\nContent-Type: image/jpeg\r\n"
                       + f"Content-Length: {len(f)}\r\n\r\n".encode() + f + b"\r\n")
            time.sleep(1.0 / CTL_STREAM_FPS)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


# URDF 정적 서빙
@app.get("/urdf/{rest:path}")
def serve_urdf(rest: str):
    p = (URDF_DIR / rest).resolve()
    if not str(p).startswith(str(URDF_DIR.resolve())) or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p)


# ----------------------------- 페이지: Setup (포트/카메라) --------------------
def calib_status():
    """설정된 id 별 캘리브레이션 파일 존재 여부."""
    out = {}
    for side, arm in ARM_CFGS.items():
        out[side] = {
            "follower": {
                "id": arm["follower_id"],
                "path": str(CALIB_ROOT / "robots/so_follower" / f"{arm['follower_id']}.json"),
                "ok": (CALIB_ROOT / "robots/so_follower" / f"{arm['follower_id']}.json").is_file()},
            "leader": {
                "id": arm["leader_id"],
                "path": str(CALIB_ROOT / "teleoperators/so_leader" / f"{arm['leader_id']}.json"),
                "ok": (CALIB_ROOT / "teleoperators/so_leader" / f"{arm['leader_id']}.json").is_file()},
        }
    return out


def _validate_cam(name, spec, seen):
    if not safe_name(name):
        return f"카메라 이름은 영문/숫자/._- 만: {name!r}"
    if name in seen:
        return f"카메라 이름 중복: {name}"
    seen.add(name)
    if not isinstance(spec, dict):
        return f"카메라 {name} 형식 오류"
    idx = spec.get("index_or_path")
    if isinstance(idx, str):
        if not idx.startswith("/dev/"):
            return f"카메라 {name} 경로는 /dev/ 로 시작해야 합니다"
    elif not isinstance(idx, int) or isinstance(idx, bool) or idx < 0:
        return f"카메라 {name} 인덱스가 잘못됨"
    for k, lo, hi in (("width", 32, 8192), ("height", 32, 8192), ("fps", 1, 240)):
        v = spec.get(k)
        if not isinstance(v, int) or isinstance(v, bool) or not lo <= v <= hi:
            return f"카메라 {name} 의 {k} 값이 잘못됨"
    return None


def validate_config(cfg):
    """저장 전 검증. 문제가 있으면 메시지를, 없으면 None 을 반환합니다."""
    if not isinstance(cfg, dict):
        return "설정 형식 오류"
    mode = cfg.get("mode")
    if mode not in ("single", "bimanual"):
        return "mode 는 single 또는 bimanual"
    arms = cfg.get("arms")
    want = 2 if mode == "bimanual" else 1
    if not isinstance(arms, list) or len(arms) != want:
        return f"mode={mode} 인데 arms 가 {len(arms) if isinstance(arms, list) else '?'}개입니다 (필요: {want})"
    sides = [a.get("side") for a in arms if isinstance(a, dict)]
    expect = {"main"} if want == 1 else {"left", "right"}
    if set(sides) != expect or len(set(sides)) != want:
        return f"mode={mode} 의 side 는 {sorted(expect)} 여야 합니다"
    if mode == "bimanual":
        try:
            by_side = {a.get("side"): a for a in arms}
            bimanual_base_id("follower", by_side)
            bimanual_base_id("leader", by_side)
        except (KeyError, TypeError, AttributeError):
            return "양팔 id 형식 오류"
        except ValueError as e:
            return str(e)
    seen_ports, cam_names, calib_ids = {}, set(), {}
    for arm in arms:
        if not isinstance(arm, dict):
            return "arms 원소 형식 오류"
        side = arm.get("side", "")
        if not safe_name(side):
            return f"side 이름이 잘못됨: {side!r}"
        for role in ("follower", "leader"):
            cid = arm.get(f"{role}_id", "")
            if not safe_name(cid):
                return f"{side}/{role} id 는 영문/숫자/._- 만"
            key = (role, cid)
            if key in calib_ids:
                return f"{role} id 중복: {cid} — 캘리브레이션 파일이 겹칩니다"
            calib_ids[key] = side
            port = (arm.get(f"{role}_port") or "").strip()
            if not port:
                continue
            if not port.startswith("/dev/"):
                return f"{side}/{role} 포트는 /dev/ 로 시작해야 합니다: {port}"
            real = os.path.realpath(port)
            if real in seen_ports:
                return f"같은 포트를 두 곳에 지정했습니다: {port} ({seen_ports[real]} 와 중복)"
            seen_ports[real] = f"{side}/{role}"
        v = arm.get("view") or {}
        for k, lim in (("x", 2.0), ("y", 2.0), ("yaw_deg", 360.0)):
            val = v.get(k, 0)
            if not isinstance(val, (int, float)) or isinstance(val, bool) or abs(float(val)) > lim:
                return f"{side} 3D 배치 {k} 값이 범위를 벗어났습니다 (±{lim:g})"
        # 양팔에서는 팔 카메라 이름에 side 접두사가 붙으므로 팔끼리 같은 이름(wrist)이 허용됩니다
        arm_cam_names = set()
        for name, spec in (arm.get("cameras") or {}).items():
            err = _validate_cam(name, spec, arm_cam_names)
            if err:
                return err
            full = f"{side}_{name}" if len(arms) > 1 else name
            if full in cam_names:
                return f"카메라 이름 충돌: {full}"
            cam_names.add(full)
    for name, spec in (cfg.get("cameras") or {}).items():
        err = _validate_cam(name, spec, set())
        if err:
            return err
        if name in cam_names:
            return f"공용 카메라 이름이 팔 카메라와 충돌: {name}"
        cam_names.add(name)
    try:
        if not 1 <= int(cfg.get("fps", 30)) <= 120:
            raise ValueError
    except (TypeError, ValueError):
        return "fps 는 1~120"
    mrt = cfg.get("max_relative_target")
    if mrt is not None and not (isinstance(mrt, (int, float)) and not isinstance(mrt, bool)
                                and 0 < float(mrt) <= 180):
        return "max_relative_target 은 비우거나 0~180 사이의 수"
    return None


@app.get("/setup", response_class=HTMLResponse)
def setup_page():
    return CSS + nav_html("st") + SETUP_HTML


@app.get("/api/setup/state")
def api_setup_state():
    busy = exclusive_busy()
    return {"config": CFG, "ports": list_serial_ports(), "calib": calib_status(),
            "busy": f"{busy['id']} 실행 중" if busy else ""}


@app.get("/api/setup/ports")
def api_setup_ports():
    return {"ports": list_serial_ports()}


@app.get("/api/setup/cameras")
def api_setup_cameras():
    if CAMS.on or busy_with(("record", "rollout")):
        return JSONResponse({"error": "카메라 사용 중 — Control/Collect 를 먼저 종료하세요"}, status_code=400)
    return {"cameras": list_video_devices()}


@app.post("/api/setup/probe")
async def api_setup_probe(req: Request):
    b = await req.json()
    port = (b.get("port") or "").strip()
    if not port.startswith("/dev/"):
        return JSONResponse({"error": "포트는 /dev/ 로 시작해야 합니다"}, status_code=400)
    if WATCH.on:
        return JSONResponse({"error": "포트 감시 중에는 probe 불가 — 감시를 먼저 중지하세요"}, status_code=400)
    busy = exclusive_busy()
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중"}, status_code=400)
    try:
        return await asyncio.to_thread(probe_port, port, bool(b.get("full")))
    except Exception as e:
        return JSONResponse({"error": f"{type(e).__name__}: {e}"}, status_code=400)


@app.post("/api/setup/watch")
async def api_setup_watch_start(req: Request):
    b = await req.json()
    ports = [p for p in (b.get("ports") or []) if isinstance(p, str) and p.startswith("/dev/")]
    if not ports:
        return JSONResponse({"error": "감시할 포트가 없습니다"}, status_code=400)
    busy = exclusive_busy()
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중"}, status_code=400)
    await asyncio.to_thread(WATCH.start, ports[:8])
    return {"ok": True, "ports": ports[:8]}


@app.get("/api/setup/watch")
def api_setup_watch_state():
    return {"on": WATCH.on, "state": WATCH.state}


@app.post("/api/setup/watch/stop")
async def api_setup_watch_stop():
    await asyncio.to_thread(WATCH.stop)
    return {"ok": True}


@app.get("/api/setup/camsnap")
def api_camsnap(dev: str):
    jpg = CAM_SNAPS.get(_snap_key(dev))
    if not jpg:
        return JSONResponse({"error": "스냅샷 없음 — 카메라 스캔을 먼저 하세요"}, status_code=404)
    return Response(content=jpg, media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.get("/api/setup/motors")
def api_motors_state():
    return MOTORSETUP.state()


@app.post("/api/setup/motors/start")
async def api_motors_start(req: Request):
    b = await req.json()
    port = (b.get("port") or "").strip()
    if not port.startswith("/dev/"):
        return JSONResponse({"error": "포트는 /dev/ 로 시작해야 합니다"}, status_code=400)
    busy = exclusive_busy()
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중"}, status_code=400)
    try:
        await asyncio.to_thread(MOTORSETUP.start, port)
    except Exception as e:
        MOTORSETUP.stage = "error"
        MOTORSETUP.err = f"{type(e).__name__}: {e}"
        return JSONResponse({"error": MOTORSETUP.err}, status_code=400)
    return {"ok": True}


@app.post("/api/setup/motors/write")
async def api_motors_write():
    try:
        await asyncio.to_thread(MOTORSETUP.write_current)
        MOTORSETUP.err = ""
    except Exception as e:
        MOTORSETUP.err = f"{type(e).__name__}: {e}"
        return JSONResponse({"error": MOTORSETUP.err}, status_code=400)
    return {"ok": True, "last": MOTORSETUP.last}


@app.post("/api/setup/motors/cancel")
async def api_motors_cancel():
    await asyncio.to_thread(MOTORSETUP.cancel)
    return {"ok": True}


@app.post("/api/setup/config")
async def api_setup_config(req: Request):
    busy = exclusive_busy()
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 저장하세요"}, status_code=400)
    b = await req.json()
    cfg = b.get("config")
    err = validate_config(cfg)
    if err:
        return JSONResponse({"error": err}, status_code=400)
    merged = json.loads(json.dumps(DEFAULT_CONFIG))
    merged.update(cfg)
    CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_FILE.write_text(json.dumps(merged, indent=2, ensure_ascii=False))
    _rebind(load_config())
    return {"ok": True, "config": CFG}


SETUP_HTML = """
<style>
.armcard{background:var(--surface);border:1px solid var(--line);border-radius:10px;
  padding:16px 18px;margin-bottom:14px}
.armcard h3{margin:0 0 12px;font-family:var(--mono);font-size:12px;letter-spacing:.16em;
  text-transform:uppercase;color:var(--accent)}
.slot{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:10px}
.slot .role{font-family:var(--mono);font-size:12px;width:74px;color:var(--muted)}
.slot input.port{flex:1;min-width:240px;font-family:var(--mono);font-size:12.5px}
.slot input.cid{width:130px;font-family:var(--mono);font-size:12.5px}
.tiny{font-size:11px;color:var(--dim);font-family:var(--mono)}
.camthumb{width:160px;height:120px;object-fit:cover;background:#000;border-radius:6px;display:block;
  border:1px solid var(--line)}
.camthumb.noimg{display:flex;align-items:center;justify-content:center;color:var(--dim);
  font-family:var(--mono);font-size:11px}
.stagebox{background:var(--surface);border:1px solid var(--accent);border-radius:10px;padding:16px 18px}
.stagebox h3{margin:0 0 6px;font-size:15px}
.stagebox .inst{color:var(--muted);margin:0 0 12px;line-height:1.7}
</style>
<div class=wrap>
<p class=eyebrow>Hardware setup</p><h2>Setup</h2>
<div id=busywarn></div>

<p class=eyebrow>1 · 모드</p>
<div class=toolbar>
  <button id=b1 onclick="setMode('single')">한팔</button>
  <button id=b2 onclick="setMode('bimanual')">양팔 (left / right)</button>
  <span class=muted id=modehint></span>
</div>
<div id=arms></div>

<p class=eyebrow>2 · USB 시리얼 포트</p>
<div class=card>
  <div class=toolbar>
    <button onclick="loadPorts()">다시 스캔</button>
    <button id=bwatch class=primary onclick="toggleWatch()">포트 감시 시작 (팔 판별)</button>
    <span class=muted>감시를 켜고 팔 하나를 손으로 움직이면 그 포트의 travel 이 올라갑니다.
      전기적으로는 leader/follower 를 구분할 수 없어서, 이게 확실한 판별 방법입니다.</span>
    <p class="badge b-warn" style="margin:8px 0 0">감시는 모든 포트의 토크를 끕니다 —
      팔로워가 들려 있으면 그대로 주저앉습니다. 팔을 받치거나 내려놓고 시작하세요.</p>
  </div>
  <table id=porttbl></table>
  <p class=muted style="margin-top:10px">
    travel = 감시 시작 이후 엔코더가 움직인 최대 폭(tick, 4096 = 1바퀴).
    probe 는 1 Mbps 기본 보드레이트만 봅니다 — 응답이 없으면 <b>전체 스캔</b>으로 보드레이트를 찾으세요.
    <b>1,2,3,4,5,6</b> 이 다 뜨면 모터 ID 세팅은 건너뛰어도 됩니다.<br>
    지정 경로는 <b>보드에 USB 시리얼 번호가 있으면</b>(sn 표시) <span class=mono>/dev/serial/by-id</span> —
    보드 자체를 따라가므로 <b>어느 USB 구멍에 꽂아도</b> 됩니다. 대신 보드를 다른 팔로 옮겨 달면 설정이 어긋나니
    보드에 sn 뒷자리를 적어 붙여 두세요.<br>
    <b>sn 이 없으면</b> 같은 모델끼리 by-id 가 겹치므로 <span class=mono>/dev/serial/by-path</span> 를 씁니다 —
    꽂은 USB 물리 포트에 고정되니 이 경우엔 <b>항상 같은 USB 구멍에</b> 꽂아야 합니다.
  </p>
</div>

<p class=eyebrow>2b · 모터 ID 세팅 — 새 팔 조립 시</p>
<div class=card>
  <p class=muted style="margin-top:0">새 STS3215 는 전부 ID 1 입니다. probe 에서 <b>1,2,3,4,5,6</b> 이 다 뜨면 이 단계는 건너뛰세요.
  하나만 뜨거나 응답이 없으면 여기서 ID 를 씁니다 — <b>모터를 한 개씩만 보드에 꽂아</b> 순서대로 진행합니다
  (여러 개가 같은 ID 1 로 붙어 있으면 응답이 충돌합니다).</p>
  <div class=toolbar>
    <select id=msport></select>
    <button id=msstart class=primary onclick="msStart()">모터 ID 세팅 시작</button>
  </div>
  <div id=msbox></div>
</div>

<p class=eyebrow>3 · 카메라</p>
<div class=card>
  <div class=toolbar>
    <button onclick="loadCams()">카메라 스캔</button>
    <span class=muted>/dev/video* 를 전부 열어 한 장씩 찍습니다 — <b>어느 장치가 어느 카메라인지 화면으로 확인</b>하세요.
      카메라 수에 따라 몇 초 걸리고, Control/Collect 실행 중에는 막힙니다.
      카메라도 같은 규칙입니다 — 시리얼 번호가 있으면 by-id, 없으면(같은 모델 2개가 겹침) by-path 라 그때는 같은 USB 구멍에 꽂아야 합니다.</span>
  </div>
  <table id=camtbl></table>
  <div style="margin-top:16px">
    <p class=eyebrow>등록된 카메라</p>
    <table id=curcamtbl></table>
    <p class=muted style="margin-top:8px">양팔이면 팔 카메라 키에 <span class=mono>left_</span> /
    <span class=mono>right_</span> 접두사가 붙고, 공용 카메라(top)는 접두사 없이 그대로 갑니다 —
    lerobot <span class=mono>bi_so_follower</span> 규칙과 같습니다.</p>
  </div>
</div>

<p class=eyebrow>4 · 기타</p>
<div class=card>
  <div class=formgrid>
    <label class=f>robot_id <input id=robot_id size=12></label>
    <label class=f>fps <input id=fps size=5></label>
    <label class=f>max_relative_target(°, 비우면 미사용) <input id=mrt size=6></label>
    <label class=f style="flex:1;min-width:240px">기본 태스크 설명 <input id=task></label>
  </div>
  <div id=calib></div>
</div>

<div class=toolbar>
  <button class=primary onclick="save()">설정 저장 &amp; 적용</button>
  <span class=muted id=savemsg></span>
</div>
<p class=eyebrow>현재 설정 (lrweb_config.json)</p><pre id=cfgdump></pre>
</div>
<script>
let CFG=null, PORTS=[], VCAMS=[], WATCHING=false, timer=null, LASTWATCH=null;
const $=id=>document.getElementById(id);
const E=s=>String(s==null?'':s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const dirty=m=>{ $('savemsg').innerHTML='<span class=b-warn>'+E(m)+' — 저장 버튼을 눌러야 적용됩니다</span>'; };

async function jget(u){ return (await fetch(u)).json(); }
async function jpost(u,b){
  const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},
                        body:JSON.stringify(b||{})});
  return r.json();
}

/* 보드에 USB 시리얼이 없으면 by-id 가 같은 모델끼리 겹칩니다 → by-path 우선 */
function stableOf(p){
  const hasSn = p.usb && p.usb.serial;
  return (hasSn ? (p.by_id || p.by_path) : (p.by_path || p.by_id)) || p.dev;
}
function sameDev(a,b){
  if(!a||!b) return false;
  if(a===b) return true;
  const f=x=>{ const p=PORTS.find(q=>q.dev===x||q.by_id===x||q.by_path===x); return p?p.dev:x; };
  return f(a)===f(b);
}

async function boot(){
  const s=await jget('/api/setup/state');
  CFG=s.config; PORTS=s.ports;
  $('busywarn').innerHTML = s.busy ? '<p class="badge b-warn">'+E(s.busy)+' — 저장/감시가 막힙니다</p>' : '';
  $('robot_id').value=CFG.robot_id||''; $('fps').value=CFG.fps||30;
  $('mrt').value=(CFG.max_relative_target==null?'':CFG.max_relative_target);
  $('task').value=CFG.default_task||'';
  renderArms(); renderPorts(); renderCurCams(); renderCalib(s.calib); dump();
  fillMsPorts(); msRefresh();
}
function dump(){ $('cfgdump').textContent=JSON.stringify(CFG,null,2); }

/* ---------- 팔 ---------- */
function setMode(m){
  if(CFG.mode===m) return;
  if(m==='bimanual'){
    const a=CFG.arms[0];
    a.side='left';
    if(a.follower_id==='follower') a.follower_id='follower_left';
    if(a.leader_id==='leader')     a.leader_id='leader_left';
    CFG.arms=[a,{side:'right',follower_port:'',follower_id:'follower_right',
                 leader_port:'',leader_id:'leader_right',cameras:{}}];
  }else{
    if(CFG.arms.length>1 && !confirm('오른팔 설정(포트·카메라)을 지웁니다. 계속할까요?')) return;
    const a=CFG.arms[0];
    a.side='main';
    if(a.follower_id==='follower_left') a.follower_id='follower';
    if(a.leader_id==='leader_left')     a.leader_id='leader';
    CFG.arms=[a];
  }
  CFG.mode=m;
  renderArms(); renderPorts(); renderCurCams(); dump(); dirty('모드 변경');
}

function renderArms(){
  const bi=CFG.mode==='bimanual';
  $('b1').className=bi?'':'primary';
  $('b2').className=bi?'primary':'';
  $('modehint').innerHTML = bi
    ? '양팔 = lerobot <span class=mono>bi_so_follower</span> / <span class=mono>bi_so_leader</span>. '
      +'calib id 는 같은 이름에 _left / _right 를 붙여야 합니다 (lerobot 이 {id}_left 로 파일을 찾습니다).'
    : '한팔 = lerobot <span class=mono>so101_follower</span> / <span class=mono>so101_leader</span>.';
  const box=$('arms'); box.innerHTML='';
  CFG.arms.forEach(a=>{
    const d=document.createElement('div'); d.className='armcard';
    d.innerHTML='<h3>'+E(a.side)+'</h3>';
    ['follower','leader'].forEach(role=>{
      const row=document.createElement('div'); row.className='slot';
      row.innerHTML='<span class=role>'+role+'</span>';
      const ip=document.createElement('input'); ip.className='port';
      ip.placeholder='비어 있음 — 아래 2번 표의 역할 드롭다운으로 지정하면 자동으로 채워집니다 (직접 입력도 가능)';
      ip.value=a[role+'_port']||'';
      ip.style.borderColor = ip.value ? 'var(--ok)' : 'var(--line)';
      ip.onchange=()=>{ a[role+'_port']=ip.value.trim(); paintRoles(); renderArms(); dump(); dirty('포트 변경'); };
      const lb=document.createElement('span'); lb.className='tiny';
      lb.textContent = CFG.mode==='bimanual' ? 'calib id (X_left / X_right)' : 'calib id';
      const id=document.createElement('input'); id.className='cid'; id.value=a[role+'_id']||'';
      id.onchange=()=>{ a[role+'_id']=id.value.trim(); dump(); dirty('캘리브 id 변경'); };
      row.appendChild(ip); row.appendChild(lb); row.appendChild(id);
      d.appendChild(row);
    });
    /* 3D 뷰 전용 배치 — 제어·수집 데이터와 무관합니다 */
    a.view = a.view || {x:0, y:0, yaw_deg:0};
    const vr=document.createElement('div'); vr.className='slot';
    vr.innerHTML='<span class=role>3D 배치</span>';
    [['x','앞뒤(m, + 앞)'],['y','좌우(m, + 왼쪽)'],['yaw_deg','회전(°, + 좌회전)']].forEach(f=>{
      const k=f[0];
      const lb=document.createElement('span'); lb.className='tiny'; lb.textContent=f[1];
      const inp=document.createElement('input'); inp.size=6; inp.value=a.view[k];
      inp.style.width='80px'; inp.style.fontFamily='var(--mono)';
      inp.onchange=()=>{ a.view[k]=parseFloat(inp.value)||0; dump(); dirty('3D 배치 변경'); };
      vr.appendChild(lb); vr.appendChild(inp);
    });
    const note=document.createElement('span'); note.className='tiny';
    note.textContent='3D 화면 표시용 — 제어·데이터에는 영향 없음';
    vr.appendChild(note);
    d.appendChild(vr);
    box.appendChild(d);
  });
}

/* ---------- 포트 ---------- */
function slotOf(dev){
  for(const a of CFG.arms)
    for(const role of ['follower','leader'])
      if(sameDev(a[role+'_port'],dev)) return a.side+'|'+role;
  return '';
}
function assign(dev,val){
  const stable=stableOf(PORTS.find(p=>p.dev===dev)||{dev});
  CFG.arms.forEach(a=>['follower','leader'].forEach(r=>{
    if(sameDev(a[r+'_port'],dev)) a[r+'_port']='';
  }));
  if(val){
    const [side,role]=val.split('|');
    const a=CFG.arms.find(x=>x.side===side);
    if(a) a[role+'_port']=stable;
  }
  renderArms(); paintRoles(); dump(); dirty('포트 지정');
}

/* 표는 포트 목록·모드가 바뀔 때만 다시 만들고, 감시 중에는 값만 덮어씁니다.
   매번 다시 그리면 열어 둔 <select> 가 닫혀서 역할을 고를 수 없습니다. */
const ROWS={};          // dev -> {idCell, travelCell, sel}
let PROBE={};           // dev -> probe 결과 HTML (다시 그려도 유지)

function renderPorts(){
  const t=$('porttbl');
  t.innerHTML='<tr><th>device</th><th>USB</th><th class=num>모터 ID</th>'
             +'<th class=num>travel</th><th>역할</th><th></th></tr>';
  Object.keys(ROWS).forEach(k=>delete ROWS[k]);
  if(!PORTS.length){
    t.innerHTML+='<tr><td colspan=6 class=muted>시리얼 장치가 없습니다 — '
                +'USB 를 꽂고 다시 스캔하세요 (권한 문제면 dialout 그룹 확인)</td></tr>';
    return;
  }
  PORTS.forEach(p=>{
    const stable=stableOf(p);
    const sn=p.usb&&p.usb.serial;
    const usb=(p.usb&&p.usb.vid)
      ? E(p.label)+'<br><span class="muted mono" style="font-size:11px">'+E(p.usb.vid)+':'+E(p.usb.pid)
        +(sn?' sn='+E(p.usb.serial):' <b class=b-warn>sn 없음</b>')+'</span>'
      : '<span class=muted>-</span>';

    const tr=document.createElement('tr');
    const c1=document.createElement('td'); c1.className='mono';
    c1.innerHTML=E(p.dev)+(stable!==p.dev?'<br><span class="tiny">'+E(stable)+'</span>':'');
    const c2=document.createElement('td'); c2.innerHTML=usb;
    const c3=document.createElement('td'); c3.className='num';
    c3.innerHTML=PROBE[p.dev]||'-';
    const c4=document.createElement('td'); c4.className='num'; c4.innerHTML='-';

    const c5=document.createElement('td');
    const sel=document.createElement('select');
    const opts=[['','미지정']];
    CFG.arms.forEach(a=>{
      opts.push([a.side+'|follower', (CFG.arms.length>1?a.side+' ':'')+'follower']);
      opts.push([a.side+'|leader',   (CFG.arms.length>1?a.side+' ':'')+'leader']);
    });
    opts.forEach(([v,l])=>{
      const o=document.createElement('option'); o.value=v; o.textContent=l; sel.appendChild(o);
    });
    sel.value=slotOf(p.dev);
    sel.onchange=()=>assign(p.dev,sel.value);
    c5.appendChild(sel);

    const c6=document.createElement('td'); c6.style.textAlign='right';
    c6.appendChild(btn('probe',()=>doProbe(p.dev,false)));
    c6.appendChild(btn('전체 스캔',()=>doProbe(p.dev,true)));

    [c1,c2,c3,c4,c5,c6].forEach(c=>tr.appendChild(c));
    t.appendChild(tr);
    ROWS[p.dev]={idCell:c3, travelCell:c4, sel:sel};
  });
  paintWatch();
}

/* 감시 상태만 갱신 — DOM 구조는 건드리지 않음 */
function paintWatch(){
  const st=LASTWATCH;
  PORTS.forEach(p=>{
    const r=ROWS[p.dev]; if(!r) return;
    const w=st?st[p.dev]:null;
    const spans=(w&&w.span)?Object.values(w.span):[];
    const travel=spans.length?Math.max.apply(null,spans):null;
    r.travelCell.innerHTML=(travel==null)?'-':'<b>'+travel+'</b>';
    r.travelCell.style.color=(travel!=null&&travel>60)?'var(--ok)':'';
    if(w&&w.err) r.idCell.innerHTML='<span class=b-bad title="'+E(w.err)+'">err</span>';
    else r.idCell.innerHTML=PROBE[p.dev]||'-';
  });
}

/* 역할 select 값만 갱신 — 다시 그리지 않음 */
function paintRoles(){
  PORTS.forEach(p=>{ const r=ROWS[p.dev]; if(r) r.sel.value=slotOf(p.dev); });
}

function btn(label,fn,cls){
  const b=document.createElement('button'); b.textContent=label;
  if(cls)b.className=cls; b.style.marginLeft='6px'; b.onclick=fn; return b;
}
function noimg(text){
  const d=document.createElement('div'); d.className='camthumb noimg'; d.textContent=text; return d;
}
/* 인라인 onerror 는 따옴표 escape 가 꼬이기 쉬워 DOM 으로 만듭니다 */
function thumb(dev, ts, failText){
  const img=document.createElement('img');
  img.className='camthumb';
  img.src='/api/setup/camsnap?dev='+encodeURIComponent(dev)+'&t='+ts;
  img.onerror=function(){ img.replaceWith(noimg(failText)); };
  return img;
}

async function loadPorts(){ PORTS=(await jget('/api/setup/ports')).ports; PROBE={}; renderPorts(); fillMsPorts(); }

async function doProbe(dev,full){
  const r=ROWS[dev]; if(!r) return;
  r.idCell.textContent='...';
  const d=await jpost('/api/setup/probe',{port:dev,full:full});
  let html;
  if(d.error){
    html='<span class=b-bad title="'+E(d.error)+'">실패</span>';
  }else if(full){
    const parts=Object.keys(d.baudrates).map(b=>b+': ['+d.baudrates[b].join(',')+']');
    html=parts.length?parts.map(E).join('<br>'):'<span class=muted>없음</span>';
  }else{
    html=d.ids.length
      ? '<span class="badge '+(d.ids.length===6?'b-ok':'b-warn')+'">'+d.ids.join(',')+'</span>'
      : '<span class=muted>응답 없음</span>';
  }
  PROBE[dev]=html; r.idCell.innerHTML=html;
}

async function toggleWatch(){
  if(WATCHING){ await stopWatch(); return; }
  const d=await jpost('/api/setup/watch',{ports:PORTS.map(p=>p.dev)});
  if(d.error){ alert(d.error); return; }
  WATCHING=true; clearWatch();
  $('bwatch').textContent='감시 중지'; $('bwatch').className='danger';
  timer=setInterval(async()=>{
    const s=await jget('/api/setup/watch');
    if(s.on){ LASTWATCH=s.state; paintWatch(); }
  },400);
}
async function stopWatch(){
  clearInterval(timer); timer=null; WATCHING=false;
  $('bwatch').textContent='포트 감시 시작 (팔 판별)'; $('bwatch').className='primary';
  await jpost('/api/setup/watch/stop');
}
/* 감시 재시작 시 이전 travel 이 남지 않게 */
function clearWatch(){ LASTWATCH=null; paintWatch(); }
addEventListener('pagehide',()=>{ if(WATCHING) navigator.sendBeacon('/api/setup/watch/stop'); });

/* ---------- 모터 ID 세팅 ---------- */
let MS=null, mstimer=null;
function fillMsPorts(){
  const sel=$('msport'); const cur=sel.value; sel.innerHTML='';
  PORTS.forEach(p=>{ const o=document.createElement('option'); o.value=p.dev; o.textContent=p.dev+(p.usb&&p.usb.product?'  ('+p.usb.product+')':''); sel.appendChild(o); });
  if(cur) sel.value=cur;
}
async function msStart(){
  const port=$('msport').value; if(!port){ alert('포트를 고르세요'); return; }
  if(!confirm('모터를 한 개씩만 보드에 연결한 상태여야 합니다. 시작할까요?')) return;
  const r=await jpost('/api/setup/motors/start',{port:port});
  if(r.error){ alert(r.error); }
  msRefresh(); if(!mstimer) mstimer=setInterval(msRefresh,1000);
}
async function msWrite(){
  const b=document.querySelector('#msbox button.primary'); if(b) b.disabled=true;
  const r=await jpost('/api/setup/motors/write'); msRefresh();
}
async function msCancel(){ await jpost('/api/setup/motors/cancel'); msRefresh(); }
async function msRefresh(){
  MS=await jget('/api/setup/motors');
  const box=$('msbox');
  if(MS.stage==='idle'){ box.innerHTML=''; if(mstimer){clearInterval(mstimer); mstimer=null;} return; }
  const doneList=MS.done.map(d=>'<span class="badge b-ok">'+E(d.name)+' = ID '+d.id+'</span>').join(' ');
  let h='<div style="margin-top:6px">'+(doneList||'<span class=muted>아직 기록된 모터 없음</span>')+'</div>';
  if(MS.stage==='running'){
    const n=MS.idx+1, total=MS.order.length, targetId=total-MS.idx;
    h+='<div class=stagebox style="margin-top:12px"><h3>'+n+' / '+total+' · <span class=mono>'+E(MS.current)+'</span> → ID '+targetId+'</h3>'
      +'<p class=inst>보드에 <b>'+E(MS.current)+'</b> 모터 <b>하나만</b> 연결하고 전원이 들어온 상태에서 아래 버튼을 누르세요. '
      +'다른 모터는 케이블을 빼 두세요. 이미 ID 를 쓴 모터도 아직 붙이지 마세요.</p>'
      +(MS.err?'<p class="badge b-bad">'+E(MS.err)+'</p>':'')
      +(MS.last?'<p class="mono muted" style="font-size:12px">'+E(MS.last)+'</p>':'')
      +'<div class=toolbar><button class="primary big" onclick="msWrite()">ID '+targetId+' 쓰기</button>'
      +'<button class=danger onclick="msCancel()">중단</button></div></div>';
  }else if(MS.stage==='done'){
    h+='<p class="badge b-ok" style="margin-top:10px">6개 모터 ID 세팅 완료 — 이제 모터를 전부 데이지체인으로 연결하고 probe 로 1~6 확인 → Calib 탭</p>'
      +'<div class=toolbar><button onclick="msCancel()">닫기</button></div>';
    if(mstimer){clearInterval(mstimer); mstimer=null;}
  }else if(MS.stage==='error'){
    h+='<p class="badge b-bad" style="margin-top:10px">'+E(MS.err)+'</p><div class=toolbar><button onclick="msCancel()">닫기</button></div>';
    if(mstimer){clearInterval(mstimer); mstimer=null;}
  }
  box.innerHTML=h;
}
addEventListener('pagehide',()=>{ if(MS&&MS.stage==='running') navigator.sendBeacon('/api/setup/motors/cancel'); });

/* ---------- 카메라 ---------- */
async function loadCams(){
  const d=await jget('/api/setup/cameras');
  if(d.error){ alert(d.error); return; }
  VCAMS=d.cameras; renderVCams();
}
function renderVCams(){
  const t=$('camtbl');
  t.innerHTML='<tr><th>화면</th><th>device</th><th>기본 해상도</th><th>대상</th><th>이름</th><th></th></tr>';
  if(!VCAMS.length){
    t.innerHTML+='<tr><td colspan=6 class=muted>스캔된 카메라 없음</td></tr>'; return;
  }
  const ts=Date.now();
  VCAMS.forEach(c=>{
    const stable=stableOf(c);      /* 시리얼 번호 있으면 by-id, 없으면 by-path (포트와 같은 규칙) */
    const sn=c.usb&&c.usb.serial;
    const tr=document.createElement('tr');
    const c0=document.createElement('td');
    c0.appendChild(c.snap ? thumb(c.dev, ts, '영상 없음') : noimg('영상 없음'));
    const c1=document.createElement('td'); c1.className='mono';
    c1.innerHTML=E(c.dev)+(stable!==c.dev?'<br><span class=tiny>'+E(stable)+'</span>':'')
      +(c.usb&&c.usb.vid?'<br><span class=tiny>'+E(c.usb.vid)+':'+E(c.usb.pid)+(sn?' sn='+E(sn):' <b class=b-warn>sn 없음 → by-path</b>')+'</span>':'');
    const c2=document.createElement('td'); c2.className='mono';
    c2.textContent=(c.width||'?')+'x'+(c.height||'?')+' @'+Math.round(c.fps||0);
    const c3=document.createElement('td');
    const sel=document.createElement('select');
    CFG.arms.forEach(a=>{
      const o=document.createElement('option');
      o.value='arm:'+a.side; o.textContent=CFG.arms.length>1?(a.side+' 팔'):'팔';
      sel.appendChild(o);
    });
    const o=document.createElement('option'); o.value='shared'; o.textContent='공용 (top 등)';
    sel.appendChild(o);
    c3.appendChild(sel);
    const c4=document.createElement('td');
    const nm=document.createElement('input'); nm.size=8; nm.value='wrist';
    sel.onchange=()=>{ nm.value = sel.value==='shared' ? 'top' : 'wrist'; };
    c4.appendChild(nm);
    const c5=document.createElement('td'); c5.style.textAlign='right';
    c5.appendChild(btn('추가',()=>addCam(stable,sel.value,nm.value.trim()),'primary'));
    [c0,c1,c2,c3,c4,c5].forEach(x=>tr.appendChild(x));
    t.appendChild(tr);
  });
}
function addCam(dev,target,name){
  if(!/^[A-Za-z0-9._-]+$/.test(name)){ alert('이름은 영문/숫자/._- 만'); return; }
  const fps=parseInt($('fps').value)||30;
  const spec={index_or_path:dev,width:640,height:480,fps:fps};
  if(target==='shared'){ CFG.cameras=CFG.cameras||{}; CFG.cameras[name]=spec; }
  else{
    const a=CFG.arms.find(x=>x.side===target.slice(4));
    a.cameras=a.cameras||{}; a.cameras[name]=spec;
  }
  renderCurCams(); dump(); dirty('카메라 추가');
}
function renderCurCams(){
  const t=$('curcamtbl');
  t.innerHTML='<tr><th>화면</th><th>키</th><th>device</th><th class=num>해상도</th><th class=num>fps</th><th></th></tr>';
  const ts=Date.now();
  let n=0;
  const groups=[];
  CFG.arms.forEach(a=>groups.push([a.cameras||{}, CFG.arms.length>1?a.side+'_':'']));
  groups.push([CFG.cameras||{}, '']);
  groups.forEach(g=>{
    const obj=g[0], prefix=g[1];
    Object.keys(obj).forEach(name=>{
      n++;
      const s=obj[name];
      const tr=document.createElement('tr');
      const c0=document.createElement('td');
      c0.appendChild(thumb(s.index_or_path, ts, '스캔 필요'));
      const c1=document.createElement('td'); c1.className='mono'; c1.textContent=prefix+name;
      const c2=document.createElement('td'); c2.className='mono';
      c2.style.fontSize='11px'; c2.textContent=s.index_or_path;
      const c3=document.createElement('td'); c3.className='num';
      c3.innerHTML='<input size=4> x <input size=4>';
      const c4=document.createElement('td'); c4.className='num'; c4.innerHTML='<input size=3>';
      const ins=[].concat([].slice.call(c3.querySelectorAll('input')),
                          [].slice.call(c4.querySelectorAll('input')));
      ins[0].value=s.width; ins[1].value=s.height; ins[2].value=s.fps;
      ins.forEach(i=>i.onchange=()=>{
        s.width=parseInt(ins[0].value)||640; s.height=parseInt(ins[1].value)||480;
        s.fps=parseInt(ins[2].value)||30; dump(); dirty('카메라 변경');
      });
      const c5=document.createElement('td'); c5.style.textAlign='right';
      c5.appendChild(btn('삭제',()=>{ delete obj[name]; renderCurCams(); dump(); dirty('카메라 삭제'); },'danger'));
      [c0,c1,c2,c3,c4,c5].forEach(x=>tr.appendChild(x));
      t.appendChild(tr);
    });
  });
  if(!n) t.innerHTML+='<tr><td colspan=6 class=muted>등록된 카메라 없음 — 위에서 스캔 후 추가하세요</td></tr>';
}

/* ---------- 캘리브레이션 상태 ---------- */
function renderCalib(cal){
  let h='<p class=eyebrow>캘리브레이션 파일</p>';
  Object.keys(cal).forEach(side=>{
    Object.keys(cal[side]).forEach(role=>{
      const c=cal[side][role];
      h+='<div class=mono style="font-size:12px;margin-bottom:5px">'
        +'<span class="badge '+(c.ok?'b-ok':'b-bad')+'">'+(c.ok?'있음':'없음')+'</span> '
        +E(side)+' / '+E(role)+' · '+E(c.id)+'.json'
        +'<br><span class=tiny>'+E(c.path)+'</span></div>';
    });
  });
  h+='<p class=muted>없으면 Control 탭이 연결되지 않습니다 — <a href="/calib">Calib 탭</a>에서 만드세요.</p>';
  $('calib').innerHTML=h;
}

/* ---------- 저장 ---------- */
async function save(){
  CFG.robot_id=$('robot_id').value.trim();
  CFG.fps=parseInt($('fps').value)||30;
  const m=$('mrt').value.trim();
  CFG.max_relative_target = m===''? null : parseFloat(m);
  CFG.default_task=$('task').value;
  if(WATCHING) await stopWatch();
  const d=await jpost('/api/setup/config',{config:CFG});
  if(d.error){ $('savemsg').innerHTML='<span class=b-bad>'+E(d.error)+'</span>'; return; }
  $('savemsg').innerHTML='<span class=b-ok>저장·적용됨</span>';
  setTimeout(function(){ location.reload(); },700);
}
boot();
</script>"""



# ----------------------------- 페이지: Calibration ----------------------------
FULL_TURN_MOTOR = "wrist_roll"       # lerobot 과 동일: 0~4095 고정
RES = 4096                           # sts3215 엔코더 해상도
RES_HALF = RES // 2
SPAN_OK_DEG = 30.0                   # 이보다 좁으면 "덜 움직임" 경고


class CalibSession:
    """lerobot SOFollower/SOLeader.calibrate() 를 웹용 상태 머신으로 풀어 쓴 것.

    원본은 input() 두 번과 record_ranges_of_motion() 의 터미널 Enter 대기로 블로킹됩니다.
    여기서는 같은 버스 프리미티브를 같은 순서로 부르되, 대기 지점을 웹 버튼으로 바꿉니다:
      connect(calibrate=False) → disable_torque → Operating_Mode=POSITION
      → [버튼] set_half_turn_homings()
      → 라이브 min/max 누적 → [버튼] write_calibration + _save_calibration
    파일 경로·포맷은 lerobot 객체의 calibration_fpath / _save_calibration 을 그대로 씁니다.
    """

    def __init__(self):
        self.lock = threading.Lock()      # 시리얼 포트는 스레드 안전하지 않음 — 모든 버스 접근을 직렬화
        self._reset()

    def _reset(self):
        self.device = None
        self.side = self.role = None
        self.stage = "idle"     # idle | homing | ranging | done | error
        self.pos, self.lo, self.hi, self.homing = {}, {}, {}, {}
        # 엔코더는 0~4095 단일 회전이라 경계를 넘으면 값이 튑니다.
        # 연속 표본의 차이로 언랩해서 '진짜 이동량' 을 누적합니다.
        self.prev_raw, self.unw = {}, {}
        self.err = ""
        self.old_calib = None   # 취소 시 모터에 되돌려 놓을 이전 캘리브레이션
        self.homed = False
        self.on = False
        self.thread = None
        self.saved_path = ""
        self.saved = {}

    @property
    def active(self):
        return self.stage in ("homing", "ranging")

    # ---- 시작 / 종료 -----------------------------------------------------------
    def start(self, side, role):
        if self.active:
            raise RuntimeError("이미 캘리브레이션 진행 중")
        arm = ARM_CFGS.get(side)
        if not arm:
            raise RuntimeError(f"알 수 없는 팔: {side}")
        port = arm.get(f"{role}_port")
        if not port:
            raise RuntimeError(f"{side}/{role} 포트가 지정되지 않았습니다 — Setup 탭에서 먼저 설정하세요")
        self._reset()
        self.side, self.role = side, role
        if role == "follower":
            from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
            dev = SOFollower(SOFollowerRobotConfig(id=arm["follower_id"], port=port,
                                                   use_degrees=True, cameras={}))
        else:
            from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
            dev = SOLeader(SOLeaderTeleopConfig(id=arm["leader_id"], port=port, use_degrees=True))
        self.old_calib = dict(dev.calibration) if dev.calibration else None
        dev.connect(calibrate=False)           # calibrate=True 면 input() 에서 멈춥니다
        try:
            from lerobot.motors.feetech import OperatingMode
            # configure() 의 torque_disabled() 가 끝나며 토크를 다시 켜므로 여기서 확실히 끕니다
            dev.bus.disable_torque()
            for m in dev.bus.motors:
                dev.bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
        except Exception:
            try:
                dev.disconnect()
            except Exception:
                pass
            raise
        self.device = dev
        self.stage = "homing"
        self.on = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.on:
            try:
                with self.lock:
                    if self.device is None:
                        break
                    pos = self.device.bus.sync_read("Present_Position", normalize=False)
                self.pos = {k: int(v) for k, v in pos.items()}
                if self.stage == "ranging":
                    for k, v in self.pos.items():
                        prev = self.prev_raw.get(k, v)
                        d = v - prev
                        if d > RES_HALF:            # 4095 → 0 방향으로 넘어감
                            d -= RES
                        elif d < -RES_HALF:         # 0 → 4095 방향으로 넘어감
                            d += RES
                        self.prev_raw[k] = v
                        u = self.unw.get(k, v) + d
                        self.unw[k] = u
                        self.lo[k] = min(self.lo.get(k, u), u)
                        self.hi[k] = max(self.hi.get(k, u), u)
                self.err = ""
            except Exception as e:
                self.err = str(e)
            time.sleep(0.1)

    def _close(self):
        self.on = False
        t, self.thread = self.thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.5)
        dev, self.device = self.device, None
        if dev is not None:
            with self.lock:
                try:
                    dev.disconnect()
                except Exception:
                    pass

    # ---- 단계 ----------------------------------------------------------------
    def set_home(self):
        if self.stage != "homing":
            raise RuntimeError("지금은 중앙 자세 단계가 아닙니다")
        with self.lock:
            # reset_calibration() 이 안에서 호출됨 — Homing_Offset=0, Min/Max 전체범위, bus.calibration 초기화
            self.homing = {k: int(v) for k, v in self.device.bus.set_half_turn_homings().items()}
            pos = self.device.bus.sync_read("Present_Position", normalize=False)
        self.homed = True
        self.pos = {k: int(v) for k, v in pos.items()}
        self.prev_raw = dict(self.pos)
        self.unw = dict(self.pos)
        self.lo = dict(self.pos)
        self.hi = dict(self.pos)
        self.stage = "ranging"

    def problems(self):
        """(안 움직인 관절, 너무 좁게 움직인 관절, 엔코더 경계를 넘은 관절)."""
        block, warn, wrap = [], [], []
        for m in CTL_JOINTS:
            if m == FULL_TURN_MOTOR:
                continue
            lo, hi = self.lo.get(m, 0), self.hi.get(m, 0)
            span = hi - lo
            # 언랩한 범위가 0~4095 를 벗어나면 중앙 자세가 가동범위의 중앙이 아니라
            # 한쪽 끝에서 엔코더가 0/4095 를 넘어간 것입니다. 이 값을 저장하면
            # 서보의 Min/Max_Position_Limit 가 엉뚱하게 잡혀 스톱을 밀게 됩니다.
            if lo < 0 or hi > RES - 1 or span >= RES:
                wrap.append(m)
            elif span <= 0:
                block.append(m)
            elif span * 360 / (RES - 1) < SPAN_OK_DEG:
                warn.append(m)
        return block, warn, wrap

    def finish(self):
        if self.stage != "ranging":
            raise RuntimeError("범위 기록 단계가 아닙니다")
        block, _, wrap = self.problems()
        if wrap:
            raise RuntimeError(
                "엔코더 경계(0/4095)를 넘은 관절: " + ", ".join(wrap)
                + " — 중앙 자세가 가동범위의 중앙이 아닙니다. 취소하고, 해당 관절을 "
                  "양 끝의 정확히 가운데에 놓은 뒤 다시 시작하세요.")
        if block:
            raise RuntimeError("아직 움직이지 않은 관절: " + ", ".join(block))
        from lerobot.motors import MotorCalibration
        dev = self.device
        calib = {}
        for m, motor in dev.bus.motors.items():
            if m == FULL_TURN_MOTOR:
                lo, hi = 0, RES - 1
            else:
                lo, hi = int(self.lo[m]), int(self.hi[m])
            calib[m] = MotorCalibration(id=motor.id, drive_mode=0,
                                        homing_offset=int(self.homing[m]),
                                        range_min=lo, range_max=hi)
        with self.lock:
            dev.bus.write_calibration(calib)
            dev.calibration = calib
            dev._save_calibration()
        self.saved_path = str(dev.calibration_fpath)
        self.saved = {m: {"homing_offset": c.homing_offset,
                          "range_min": c.range_min, "range_max": c.range_max}
                      for m, c in calib.items()}
        self.stage = "done"
        self._close()

    def cancel(self):
        if self.device is not None and self.homed and self.old_calib:
            # 중앙 자세 기록이 모터 EEPROM 을 이미 바꿨으므로 이전 값을 되돌려 놓습니다
            with self.lock:
                try:
                    self.device.bus.write_calibration(self.old_calib)
                except Exception as e:
                    self.err = f"이전 캘리브레이션 복원 실패: {e}"
        self._close()
        self.stage = "idle"       # done/error 화면의 '닫기' 도 여기로 옵니다

    def state(self):
        rows = []
        for m in CTL_JOINTS:
            r = {"name": m, "pos": self.pos.get(m), "full_turn": m == FULL_TURN_MOTOR}
            if self.stage in ("ranging", "done"):
                lo, hi = self.lo.get(m), self.hi.get(m)
                r.update({"min": lo, "max": hi,
                          "span_deg": round((hi - lo) * 360 / (RES - 1), 1) if lo is not None else None,
                          "wrapped": lo is not None and (lo < 0 or hi > RES - 1)})
            if self.homed and r["pos"] is not None:
                r["deg"] = round((r["pos"] - (RES - 1) / 2) * 360 / (RES - 1), 1)
            rows.append(r)
        block, warn, wrap = self.problems() if self.stage == "ranging" else ([], [], [])
        return {"stage": self.stage, "side": self.side, "role": self.role,
                "rows": rows, "err": self.err, "block": block, "warn": warn, "wrap": wrap,
                "saved_path": self.saved_path, "saved": self.saved,
                "span_ok_deg": SPAN_OK_DEG}


CALIB = CalibSession()


@app.get("/calib", response_class=HTMLResponse)
def calib_page():
    return CSS + nav_html("cb") + CALIB_HTML


@app.get("/api/calib/state")
def api_calib_state():
    st = CALIB.state()
    st["devices"] = calib_status()
    busy = exclusive_busy() if not CALIB.active else None
    st["busy"] = f"{busy['id']} 실행 중" if busy else ""
    st["ports_configured"] = ports_configured()
    return st


@app.post("/api/calib/start")
async def api_calib_start(req: Request):
    b = await req.json()
    side, role = b.get("side"), b.get("role")
    if role not in ("follower", "leader") or side not in ARM_CFGS:
        return JSONResponse({"error": "side/role 이 잘못됨"}, status_code=400)
    busy = exclusive_busy()
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 시작하세요"}, status_code=400)
    try:
        await asyncio.to_thread(CALIB.start, side, role)
    except Exception as e:
        CALIB.stage = "error"
        CALIB.err = f"{type(e).__name__}: {e}"
        return JSONResponse({"error": CALIB.err}, status_code=400)
    return {"ok": True}


@app.post("/api/calib/home")
async def api_calib_home():
    try:
        await asyncio.to_thread(CALIB.set_home)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    return {"ok": True}


@app.post("/api/calib/finish")
async def api_calib_finish():
    try:
        await asyncio.to_thread(CALIB.finish)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    return {"ok": True, "path": CALIB.saved_path}


@app.post("/api/calib/cancel")
async def api_calib_cancel():
    await asyncio.to_thread(CALIB.cancel)
    return {"ok": True}


CALIB_HTML = """
<style>
.devgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px;margin-bottom:18px}
.dev{background:var(--surface);border:1px solid var(--line);border-radius:10px;padding:14px 16px}
.dev h3{margin:0 0 6px;font-family:var(--mono);font-size:12px;letter-spacing:.14em;text-transform:uppercase;color:var(--accent)}
.dev .p{font-family:var(--mono);font-size:11px;color:var(--dim);word-break:break-all;margin-bottom:10px}
.stagebox{background:var(--surface);border:1px solid var(--accent);border-radius:10px;padding:18px 20px;margin-bottom:18px}
.stagebox h3{margin:0 0 6px;font-size:16px}
.stagebox .inst{color:var(--muted);margin:0 0 14px;line-height:1.7}
.bar{height:6px;background:var(--surface2);border-radius:3px;overflow:hidden;min-width:120px}
.bar i{display:block;height:100%;background:var(--accent)}
.bar.ok i{background:var(--ok)} .bar.warn i{background:var(--warn)} .bar.bad i{background:var(--bad)}
td.mono{font-variant-numeric:tabular-nums}
.stepdots{display:flex;gap:6px;align-items:center;font-family:var(--mono);font-size:11px;color:var(--dim);margin-bottom:14px}
.stepdots b{color:var(--text)}
.stepdots span.on{color:var(--accent)}
</style>
<div class=wrap>
<p class=eyebrow>Motor calibration</p><h2>Calibration</h2>
<div id=busywarn></div>
<div id=picker></div>
<div id=stage></div>
</div>
<script>
const $=id=>document.getElementById(id);
const E=s=>String(s==null?'':s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function jget(u){ return (await fetch(u)).json(); }
async function jpost(u,b){
  const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});
  return r.json();
}
let ST=null, timer=null;

function renderPicker(s){
  let h='';
  if(!s.ports_configured)
    h+='<p class="badge b-warn">포트가 지정되지 않았습니다 — <a href="/setup">Setup 탭</a>에서 먼저 정하세요</p>';
  h+='<div class=devgrid>';
  Object.keys(s.devices).forEach(side=>{
    ['follower','leader'].forEach(role=>{
      const d=s.devices[side][role];
      const busy=s.stage==='homing'||s.stage==='ranging';
      h+='<div class=dev><h3>'+E(side)+' · '+role+'</h3>'
        +'<span class="badge '+(d.ok?'b-ok':'b-bad')+'">'+(d.ok?'캘리브레이션 있음':'없음')+'</span> '
        +'<span class=mono style="font-size:12px">'+E(d.id)+'.json</span>'
        +'<div class=p>'+E(d.path)+'</div>'
        +'<button class=primary '+(busy||!s.ports_configured?'disabled':'')
        +' onclick="start(\\''+E(side)+'\\',\\''+role+'\\',this)">'+(d.ok?'다시 캘리브레이션':'캘리브레이션 시작')+'</button>'
        +'</div>';
    });
  });
  h+='</div>';
  $('picker').innerHTML=h;
}

function rows(s, withRange){
  let h='<table><tr><th>joint</th><th class=num>raw</th>'
    +(s.stage!=='homing'?'<th class=num>deg</th>':'')
    +(withRange?'<th class=num>min</th><th class=num>max</th><th class=num>span</th><th style="width:160px"></th>':'')
    +'</tr>';
  s.rows.forEach(r=>{
    const n=r.name;
    h+='<tr><td class=mono>'+n+(r.full_turn?' <span class=muted style="font-size:11px">(전체 회전, 0~4095 고정)</span>':'')+'</td>'
      +'<td class="num mono" id="c_pos_'+n+'"></td>'
      +(s.stage!=='homing'?'<td class="num mono" id="c_deg_'+n+'"></td>':'');
    if(withRange){
      if(r.full_turn){ h+='<td class=num>0</td><td class=num>4095</td><td class=num>360°</td><td></td>'; }
      else{
        h+='<td class="num mono" id="c_min_'+n+'"></td><td class="num mono" id="c_max_'+n+'"></td>'
          +'<td class="num mono" id="c_span_'+n+'"></td>'
          +'<td><div class="bar" id="c_bar_'+n+'"><i style="width:0%"></i></div></td>';
      }
    }
    h+='</tr>';
  });
  return h+'</table>';
}

/* 값만 갱신 — DOM 을 다시 만들지 않습니다.
   250ms 마다 통째로 다시 그리면 버튼/셀이 교체되며 클릭이 씹힙니다. */
function patchRows(s){
  s.rows.forEach(r=>{
    const n=r.name;
    const pos=$('c_pos_'+n); if(pos) pos.textContent = r.pos==null?'-':r.pos;
    const deg=$('c_deg_'+n); if(deg) deg.textContent = r.deg==null?'-':r.deg.toFixed(1)+'°';
    if(r.full_turn) return;
    const mn=$('c_min_'+n); if(mn) mn.textContent = r.min==null?'-':r.min;
    const mx=$('c_max_'+n); if(mx) mx.textContent = r.max==null?'-':r.max;
    const sp=r.span_deg||0;
    const spc=$('c_span_'+n);
    if(spc) spc.innerHTML = r.wrapped?'<span class=b-bad>'+sp.toFixed(1)+'° ⚠</span>':sp.toFixed(1)+'°';
    const bar=$('c_bar_'+n);
    if(bar){
      bar.className='bar '+(r.wrapped||sp<=0?'bad':(sp<s.span_ok_deg?'warn':'ok'));
      bar.firstElementChild.style.width=Math.min(100, sp/180*100)+'%';
    }
  });
}

function dots(n){
  const names=['연결','중앙 자세','범위 기록','저장'];
  return '<div class=stepdots>'+names.map((x,i)=>
    '<span class="'+(i===n?'on':'')+'">'+(i<n?'✓ ':'')+(i===n?'<b>'+x+'</b>':x)+'</span>'+(i<3?' › ':'')).join('')+'</div>';
}

function noteHtml(s){
  const wrp=(s.wrap||[]).length, blk=s.block.length, wrn=s.warn.length;
  if(wrp) return '<p class="badge b-bad">엔코더 경계(0/4095)를 넘었습니다: '+s.wrap.join(', ')
    +'<br>중앙 자세가 가동범위의 중앙이 아닙니다. <b>취소</b>하고 해당 관절을 양 끝의 정확히 가운데에 놓은 뒤 다시 시작하세요.</p>';
  if(blk) return '<p class="badge b-bad">아직 움직이지 않은 관절: '+s.block.join(', ')+'</p>';
  if(wrn) return '<p class="badge b-warn">'+s.span_ok_deg+'° 미만으로만 움직인 관절: '+s.warn.join(', ')+' — 의도한 게 아니면 더 움직이세요</p>';
  return '<p class="badge b-ok">모든 관절 기록됨 — 저장할 수 있습니다</p>';
}

/* 구조를 다시 만들지 않고 살아 있는 값만 덮어씁니다 */
function patchStage(s){
  patchRows(s);
  const n=$('cnote'); if(n) n.innerHTML=noteHtml(s);
  const e=$('cerr');
  if(e){ e.innerHTML = s.err?'<p class="badge b-bad">'+E(s.err)+'</p>':''; }
  const fb=$('cfinish');
  if(fb) fb.disabled = s.block.length>0 || (s.wrap||[]).length>0;
}

function renderStage(s){
  const box=$('stage');
  const head='<h3>'+E(s.side)+' · '+E(s.role)+'</h3>';
  const err='<div id=cerr></div>';
  if(s.stage==='homing'){
    const follower = s.role==='follower';
    box.innerHTML='<div class=stagebox>'+head+dots(1)
      +'<p class=inst>토크가 꺼져 있습니다'+(follower?' — <b>팔로워가 주저앉을 수 있으니 손으로 받치세요.</b>':'.')+'<br>'
      +'<b>모든 관절을 가동 범위의 정중앙</b>에 놓으세요 (그리퍼는 반쯤 벌린 상태). '
      +'lerobot 은 이 자세를 각 모터의 반 바퀴(2047 tick) 기준점으로 잡습니다.<br>'
      +'<b>여기서 중앙을 벗어나면</b> 반대쪽 끝까지 움직일 때 엔코더가 0/4095 를 넘어가 '
      +'기록이 망가집니다 (다음 단계에서 걸러냅니다). 한쪽 끝에 치우치지 않게 하세요.<br>'
      +'그리퍼 포함 6개 관절 전부입니다. 준비되면 아래 버튼을 누르세요.</p>'
      +err+rows(s,false)
      +'<div class=toolbar style="margin-top:14px">'
      +'<button class="primary big" onclick="home(this)">중앙 자세 기록</button>'
      +'<button class=danger onclick="cancel()">취소</button></div></div>';
  }else if(s.stage==='ranging'){
    box.innerHTML='<div class=stagebox>'+head+dots(2)
      +'<p class=inst><b>wrist_roll 을 뺀 모든 관절</b>을 한 개씩, 한쪽 끝에서 반대쪽 끝까지 <b>천천히</b> 움직이세요. '
      +'그리퍼도 완전히 열고 완전히 닫으세요.<br>'
      +'여기서 기록되는 min/max 가 그대로 관절 한계가 됩니다 — 기계적 스톱에 <b>살짝 닿기 직전</b>까지만.<br>'
      +'각 줄의 막대가 초록이 되면 충분합니다. 끝나면 <b>완료·저장</b>.</p>'
      +err+'<div id=cnote></div>'+rows(s,true)
      +'<div class=toolbar style="margin-top:14px">'
      +'<button class="primary big" id=cfinish onclick="finish(this)">완료·저장</button>'
      +'<button class=danger onclick="cancel()">취소 (이전 값 복원)</button></div></div>';
  }else if(s.stage==='done'){
    box.innerHTML='<div class=stagebox>'+head+dots(3)
      +'<p class="badge b-ok">저장됨</p><p class="mono" style="font-size:12px;color:var(--muted)">'+E(s.saved_path)+'</p>'
      +rows(s,true)
      +'<pre style="margin-top:12px">'+E(JSON.stringify(s.saved,null,2))+'</pre>'
      +'<div class=toolbar style="margin-top:14px"><button class=primary onclick="closeDone()">닫기</button>'
      +'<a href="/control"><button>Control 탭에서 확인</button></a></div></div>';
  }else if(s.stage==='error'){
    box.innerHTML='<div class=stagebox>'+head+'<p class="badge b-bad">'+E(s.err)+'</p>'
      +'<div class=toolbar><button onclick="closeDone()">닫기</button></div></div>';
  }else{
    box.innerHTML='';
  }
}

let LAST_KEY=null, LAST_PICK=null, polling=false;

/* 구조가 바뀌었을 때만 다시 그리고, 평소엔 값만 갱신 */
function apply(s){
  ST=s;
  const busy=s.busy?'<p class="badge b-warn">'+E(s.busy)+' — 끝나야 캘리브레이션을 시작할 수 있습니다</p>':'';
  if($('busywarn').innerHTML!==busy) $('busywarn').innerHTML=busy;
  const pick=JSON.stringify([s.devices, s.stage, s.ports_configured]);
  if(pick!==LAST_PICK){ LAST_PICK=pick; renderPicker(s); }
  const key=[s.stage, s.side, s.role].join('|');
  if(key!==LAST_KEY){ LAST_KEY=key; renderStage(s); }
  patchStage(s);
}

async function refresh(){
  if(polling) return;          /* 응답이 느려도 요청이 쌓이지 않게 */
  polling=true;
  try{ apply(await jget('/api/calib/state')); }
  catch(e){ /* 일시적 네트워크 오류는 무시하고 다음 주기에 */ }
  finally{ polling=false; }
}

/* setInterval 은 느린 응답에서 요청이 겹칩니다 — 끝난 뒤 다음을 예약 */
async function poll(){
  await refresh();
  timer=setTimeout(poll, 250);
}

/* 클릭 즉시 잠가서 중복 클릭·먹통 체감을 없앰 */
async function withBusy(el, fn){
  if(el){ el.disabled=true; el.dataset.t=el.textContent; el.textContent='처리 중…'; }
  try{ return await fn(); }
  finally{
    if(el && el.isConnected){ el.disabled=false; if(el.dataset.t) el.textContent=el.dataset.t; }
  }
}
async function start(side,role,el){
  await withBusy(el, async()=>{
    const r=await jpost('/api/calib/start',{side:side,role:role});
    if(r.error) alert(r.error);
    await refresh();
  });
}
async function home(el){
  await withBusy(el, async()=>{
    const r=await jpost('/api/calib/home');
    if(r.error) alert(r.error);
    await refresh();
  });
}
async function finish(el){
  if(ST&&ST.warn&&ST.warn.length&&!confirm('일부 관절이 좁게만 움직였습니다:\\n'+ST.warn.join(', ')+'\\n이대로 저장할까요?')) return;
  await withBusy(el, async()=>{
    const r=await jpost('/api/calib/finish');
    if(r.error) alert(r.error);
    await refresh();
  });
}
async function cancel(){
  if(!confirm('취소하면 지금까지 기록이 버려집니다.')) return;
  await jpost('/api/calib/cancel'); await refresh();
}
async function closeDone(){ await jpost('/api/calib/cancel'); await refresh(); }
addEventListener('pagehide',()=>{ if(ST&&(ST.stage==='homing'||ST.stage==='ranging')) navigator.sendBeacon('/api/calib/cancel'); });
poll();
</script>"""



# ----------------------------- Record worker (별도 프로세스, 5단계) ----------------
# lerobot-record 를 셸로 띄우고 PTY 로 키를 넣던 것을 없앴습니다. 대신 이 파일 자체를
#   python lrweb.py --worker record <jid>
# 로 띄워 lerobot 의 record_loop() 를 직접 부릅니다. events 딕트가 곧 n/r/q 입니다.
#   상태   : RUN_DIR/<jid>/status.json       (worker → 웹, PREVIEW_FPS 로 갱신)
#   미리보기: RUN_DIR/<jid>/cam_<name>.jpg   (worker → 웹, 원자적 교체)
#   명령   : RUN_DIR/<jid>/cmd               (웹 → worker, n/r/q 문자를 append)
# 전부 파일이라 lrweb 를 재시작해도 세션을 잃지 않습니다.

def _cam_configs(specs):
    from lerobot.cameras.opencv import OpenCVCameraConfig
    out = {}
    for name, sp in specs.items():
        idx = sp["index_or_path"]
        idx = idx if isinstance(idx, int) else Path(str(idx))
        out[name] = OpenCVCameraConfig(index_or_path=idx, fps=int(sp["fps"]),
                                       width=int(sp["width"]), height=int(sp["height"]))
    return out


def make_devices(spec):
    """spec = 시작 시점의 설정 스냅샷. (robot, teleop, 하위 팔 객체 목록) — 한팔/양팔 분기.
    하위 팔 객체 목록은 캘리브레이션 파일 확인·기록용입니다."""
    arms = {a["side"]: a for a in spec["arms"]}
    if spec["mode"] == "bimanual":
        from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
        from lerobot.robots.so_follower import SOFollowerConfig
        from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
        from lerobot.teleoperators.so_leader import SOLeaderConfig
        L, R = arms["left"], arms["right"]
        robot = BiSOFollower(BiSOFollowerConfig(
            id=bimanual_base_id("follower", arms),
            left_arm_config=SOFollowerConfig(port=L["follower_port"], cameras=_cam_configs(L["cameras"])),
            right_arm_config=SOFollowerConfig(port=R["follower_port"], cameras=_cam_configs(R["cameras"])),
            cameras=_cam_configs(spec["cameras"])))
        teleop = BiSOLeader(BiSOLeaderConfig(
            id=bimanual_base_id("leader", arms),
            left_arm_config=SOLeaderConfig(port=L["leader_port"]),
            right_arm_config=SOLeaderConfig(port=R["leader_port"])))
        subs = [robot.left_arm, robot.right_arm, teleop.left_arm, teleop.right_arm]
    else:
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
        arm = spec["arms"][0]
        cams = dict(arm["cameras"])
        cams.update(spec["cameras"])
        robot = SOFollower(SOFollowerRobotConfig(id=arm["follower_id"], port=arm["follower_port"],
                                                 use_degrees=True, cameras=_cam_configs(cams)))
        teleop = SOLeader(SOLeaderTeleopConfig(id=arm["leader_id"], port=arm["leader_port"],
                                               use_degrees=True))
        subs = [robot, teleop]
    return robot, teleop, subs


class _Preview:
    """robot.get_observation() 을 감싸 최신 카메라 프레임을 잡아두고, 별도 스레드가
    PREVIEW_FPS 로 JPEG + status.json 을 씁니다. record 루프에는 인코딩 비용을 얹지 않습니다."""

    def __init__(self, robot, rd, status):
        self.rd = rd
        self.status = status
        self.latest = {}
        self.on = True
        orig = robot.get_observation

        def tee():
            obs = orig()
            for k, v in obs.items():
                if getattr(v, "ndim", 0) == 3:
                    self.latest[k] = v
            return obs

        robot.get_observation = tee     # 인스턴스 속성이 클래스 메서드를 가림
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def write_status(self):
        st = dict(self.status)
        if st.get("t0"):
            st["elapsed"] = round(time.time() - st["t0"], 1)
        tmp = self.rd / "status.json.tmp"
        tmp.write_text(json.dumps(st))
        os.replace(tmp, self.rd / "status.json")

    def _loop(self):
        try:
            import cv2
        except ImportError:
            cv2 = None
        while self.on:
            t0 = time.monotonic()
            try:
                self.write_status()
                if cv2 is not None:
                    for k, frame in list(self.latest.items()):
                        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                                               [cv2.IMWRITE_JPEG_QUALITY, 60])
                        if ok:
                            tmp = self.rd / f"cam_{k}.jpg.tmp"
                            tmp.write_bytes(buf.tobytes())
                            os.replace(tmp, self.rd / f"cam_{k}.jpg")
            except Exception:
                pass
            time.sleep(max(0.0, 1.0 / PREVIEW_FPS - (time.monotonic() - t0)))

    def stop(self):
        self.on = False
        try:
            self.write_status()
        except Exception:
            pass


def worker_record(jid):
    import logging
    import signal as _sig
    j = load_json(JOB_DIR / f"{jid}.json", {})
    spec = j.get("spec") or {}
    rd = run_dir(jid)
    rd.mkdir(parents=True, exist_ok=True)
    status = {"phase": "starting", "episode": None, "recorded": 0,
              "num_episodes": int(spec.get("num_episodes", 0)), "t0": None, "phase_len": 0,
              "elapsed": 0.0, "err": "", "repo_id": spec.get("repo_id", ""), "cams": []}
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

    def on_key(k):
        if k == "n":
            events["exit_early"] = True
        elif k == "r":
            events["rerecord_episode"] = True
            events["exit_early"] = True
        elif k == "q":
            events["stop_recording"] = True
            events["exit_early"] = True

    alive = {"on": True}

    def poll_cmd():
        f = rd / "cmd"
        while alive["on"]:
            try:
                if f.exists():
                    txt = f.read_text()
                    f.unlink()
                    for ch in txt:
                        on_key(ch)
                        print(f"[lrweb-worker] key {ch}", flush=True)
            except OSError:
                pass
            time.sleep(0.05)

    threading.Thread(target=poll_cmd, daemon=True).start()
    _sig.signal(_sig.SIGINT, lambda *_: on_key("q"))     # Jobs 탭 '중지' = q 와 동일
    _sig.signal(_sig.SIGTERM, lambda *_: on_key("q"))

    def put(**kw):
        status.update(kw)
        try:
            tmp = rd / "status.json.tmp"
            tmp.write_text(json.dumps(status))
            os.replace(tmp, rd / "status.json")
        except OSError:
            pass

    dataset = robot = teleop = preview = None
    rc = 0
    try:
        from lerobot.utils.utils import init_logging
        init_logging()
        from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility
        from lerobot.configs.dataset import DatasetRecordConfig
        from lerobot.datasets import (LeRobotDataset, VideoEncodingManager,
                                      aggregate_pipeline_dataset_features, create_initial_features)
        from lerobot.processor import make_default_processors
        from lerobot.scripts.lerobot_record import record_loop
        from lerobot.utils.feature_utils import combine_feature_dicts

        robot, teleop, subs = make_devices(spec)
        for d in subs:
            if not d.calibration:
                raise RuntimeError(f"캘리브레이션 파일이 없습니다: {d.calibration_fpath} — Calib 탭에서 만드세요")

        tap, rap, rop = make_default_processors()
        features = combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=tap, initial_features=create_initial_features(action=robot.action_features),
                use_videos=True),
            aggregate_pipeline_dataset_features(
                pipeline=rop, initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=True))
        # 인코더/이미지라이터 기본값은 lerobot-record 와 동일하게 DatasetRecordConfig 에서 가져옵니다
        dcfg = DatasetRecordConfig(repo_id=spec["repo_id"], single_task=spec["task"], root=spec["root"],
                                   fps=int(spec["fps"]), episode_time_s=spec["episode_time_s"],
                                   reset_time_s=spec["reset_time_s"], num_episodes=int(spec["num_episodes"]),
                                   push_to_hub=False, streaming_encoding=bool(spec.get("streaming_encoding", False)))
        ncam = len(robot.cameras)
        iw_p = dcfg.num_image_writer_processes if ncam else 0
        iw_t = dcfg.num_image_writer_threads_per_camera * ncam if ncam else 0
        if spec.get("resume"):
            dataset = LeRobotDataset.resume(
                dcfg.repo_id, root=dcfg.root, batch_encoding_size=dcfg.video_encoding_batch_size,
                rgb_encoder=dcfg.rgb_encoder, depth_encoder=dcfg.depth_encoder,
                encoder_threads=dcfg.encoder_threads, streaming_encoding=dcfg.streaming_encoding,
                encoder_queue_maxsize=dcfg.encoder_queue_maxsize,
                image_writer_processes=iw_p, image_writer_threads=iw_t)
            sanity_check_dataset_robot_compatibility(dataset, robot, dcfg.fps, features)
        else:
            dataset = LeRobotDataset.create(
                dcfg.repo_id, dcfg.fps, root=dcfg.root, robot_type=robot.name, features=features,
                use_videos=True, image_writer_processes=iw_p, image_writer_threads=iw_t,
                batch_encoding_size=dcfg.video_encoding_batch_size,
                rgb_encoder=dcfg.rgb_encoder, depth_encoder=dcfg.depth_encoder,
                encoder_threads=dcfg.encoder_threads, streaming_encoding=dcfg.streaming_encoding,
                encoder_queue_maxsize=dcfg.encoder_queue_maxsize)

        put(phase="connecting")
        robot.connect(calibrate=False)     # calibrate=True 면 input() → 파이프에서 EOFError
        teleop.connect(calibrate=False)
        for d in subs:
            if not d.bus.is_calibrated:
                d.bus.write_calibration(d.calibration)
        # 미리보기 이름은 관측 키 기준 — 양팔이면 left_wrist / right_wrist / top 처럼 접두사가 붙습니다
        # (robot.cameras 는 호환용이라 양팔에서 이름이 겹칩니다)
        status["cams"] = [k for k, v in robot.observation_features.items() if isinstance(v, tuple)]
        preview = _Preview(robot, rd, status)

        fps, task = int(spec["fps"]), spec["task"]
        ept, rst, N = spec["episode_time_s"], spec["reset_time_s"], int(spec["num_episodes"])
        with VideoEncodingManager(dataset):
            recorded = 0
            while recorded < N and not events["stop_recording"]:
                put(phase="record", episode=dataset.num_episodes, recorded=recorded,
                    t0=time.time(), phase_len=ept)
                logging.info(f"Recording episode {dataset.num_episodes}")
                record_loop(robot=robot, events=events, fps=fps,
                            teleop_action_processor=tap, robot_action_processor=rap,
                            robot_observation_processor=rop, teleop=teleop, dataset=dataset,
                            control_time_s=ept, single_task=task)
                if not events["stop_recording"] and (recorded < N - 1 or events["rerecord_episode"]):
                    put(phase="reset", t0=time.time(), phase_len=rst)
                    logging.info("Reset the environment")
                    record_loop(robot=robot, events=events, fps=fps,
                                teleop_action_processor=tap, robot_action_processor=rap,
                                robot_observation_processor=rop, teleop=teleop,
                                control_time_s=rst, single_task=task)
                if events["rerecord_episode"]:
                    logging.info("Re-record episode")
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue
                put(phase="saving", t0=None)
                dataset.save_episode()
                recorded += 1
                put(recorded=recorded)
    except Exception as e:
        logging.exception("record worker failed")
        put(phase="error", err=f"{type(e).__name__}: {e}")
        rc = 1
    finally:
        alive["on"] = False
        put(phase="finalizing", t0=None)
        if dataset is not None:
            try:
                dataset.finalize()
            except Exception as e:
                logging.exception("finalize failed")
                put(err=f"finalize: {e}")
        for dev in (robot, teleop):
            try:
                if dev is not None and dev.is_connected:
                    dev.disconnect()
            except Exception:
                pass
        if preview is not None:
            preview.stop()
        put(phase="error" if rc else "done")
    return rc


def worker_main(kind, jid):
    if not safe_name(jid):
        print("bad jid", file=sys.stderr)
        return 2
    if kind == "record":
        return worker_record(jid)
    print(f"unknown worker kind: {kind}", file=sys.stderr)
    return 2


COLLECT_RUN_HTML = """
<style>
.rec{display:grid;grid-template-columns:1fr;gap:14px}
.phase{font-family:var(--mono);font-size:26px;font-weight:600;letter-spacing:.04em}
.phase.record{color:var(--bad)} .phase.reset{color:var(--warn)} .phase.saving{color:var(--accent)}
.pbar{height:10px;background:var(--surface2);border-radius:5px;overflow:hidden;margin:8px 0 4px}
.pbar i{display:block;height:100%;background:var(--accent);transition:width .2s linear}
.pbar.record i{background:var(--bad)} .pbar.reset i{background:var(--warn)}
.cams{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:8px}
.cams .cw{position:relative;background:#000;border-radius:8px;overflow:hidden}
.cams img{width:100%;display:block;aspect-ratio:4/3;object-fit:contain;background:#000}
.cams .cl{position:absolute;top:6px;left:10px;font-family:var(--mono);font-size:11px;
  letter-spacing:.1em;text-transform:uppercase;color:#cfd8e3;text-shadow:0 0 4px #000}
.bigkeys{display:flex;gap:12px;flex-wrap:wrap}
.bigkeys button{flex:1;min-width:140px;padding:18px 10px;font-size:17px}
.statline{display:flex;gap:18px;flex-wrap:wrap;font-family:var(--mono);font-size:13px;color:var(--muted)}
.statline b{color:var(--text)}
</style>
<div class=wrap>
<p class=eyebrow>Recording</p><h2>수집 진행 중</h2>
<div class=runbar><span class="badge b-run">recording</span>
  <span class=mono id=jid></span>
  <span class=mono id=repo style="color:var(--muted)"></span>
  <button class=danger onclick="stopRec()" style="margin-left:auto">강제 중지</button></div>
<div class=rec>
  <div class=card>
    <div class=phase id=phase>…</div>
    <div class=pbar id=pbar><i id=pfill style="width:0%"></i></div>
    <div class=statline>
      <span>에피소드 <b id=ep>-</b> / <b id=nep>-</b></span>
      <span>저장됨 <b id=rec>0</b></span>
      <span>경과 <b id=el>0.0</b>s / <b id=plen>-</b>s</span>
    </div>
    <p id=err class="badge b-bad" style="display:none;margin-top:10px"></p>
  </div>
  <div class=cams id=cams></div>
  <div class=bigkeys>
    <button class=primary onclick="key('n')">n &nbsp;다음 (에피소드 조기 종료)</button>
    <button onclick="key('r')">r &nbsp;재녹화</button>
    <button class=danger onclick="key('q')">q &nbsp;종료·저장</button>
  </div>
  <p class=muted>record 중: 리더암을 움직이면 팔로워가 따라가고 프레임이 기록됩니다.
  reset 중: 기록 없이 팔만 따라갑니다 — 물체를 제자리에 놓으세요. n 으로 각 단계를 조기 종료할 수 있습니다.</p>
  <p class=eyebrow>Log</p><pre id=tail>...</pre>
</div></div>
<script>
const $=id=>document.getElementById(id);
$('jid').textContent=JID;
let camsBuilt=false;
async function key(k){ await fetch('/api/sendkey/'+JID+'/'+k,{method:'POST'}); }
async function stopRec(){
  if(!confirm('강제 중지할까요? (가능하면 q 종료·저장을 쓰세요)'))return;
  await fetch('/api/kill/'+JID,{method:'POST'}); setTimeout(()=>location.reload(),1500);
}
function buildCams(names){
  const box=$('cams'); box.innerHTML='';
  names.forEach(n=>{
    const d=document.createElement('div'); d.className='cw';
    d.innerHTML='<span class=cl></span><img>';
    d.querySelector('.cl').textContent=n;
    d.querySelector('img').src='/stream/'+encodeURIComponent(n);
    box.appendChild(d);
  });
  camsBuilt=names.length>0;
}
async function refresh(){
  const d=await (await fetch('/api/record_status/'+JID)).json();
  const s=d.status||{};
  const ph=s.phase||'starting';
  const label={starting:'준비 중…',connecting:'팔·카메라 연결 중…',record:'● RECORD',reset:'RESET — 환경 정리',
               saving:'저장 중…',finalizing:'마무리 중…',done:'완료',error:'오류'}[ph]||ph;
  $('phase').textContent=label; $('phase').className='phase '+ph;
  $('pbar').className='pbar '+ph;
  const pct = (s.phase_len&&s.elapsed!=null)? Math.min(100, s.elapsed/s.phase_len*100) : (ph==='record'||ph==='reset'?0:100);
  $('pfill').style.width=pct+'%';
  $('ep').textContent = s.episode==null?'-':s.episode;
  $('nep').textContent = s.num_episodes==null?'-':s.num_episodes;
  $('rec').textContent = s.recorded==null?'0':s.recorded;
  $('el').textContent = s.elapsed==null?'0.0':Number(s.elapsed).toFixed(1);
  $('plen').textContent = s.phase_len||'-';
  $('repo').textContent = s.repo_id||'';
  if(s.err){ $('err').style.display=''; $('err').textContent=s.err; } else { $('err').style.display='none'; }
  if(!camsBuilt && s.cams && s.cams.length) buildCams(s.cams);
  $('tail').textContent=d.tail||'';
  if(!d.alive){ setTimeout(()=>location.reload(),1200); }
}
refresh(); setInterval(refresh,500);
document.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  if(['n','r','q'].includes(e.key)) key(e.key);
});
</script>"""



# ----------------------------- 페이지: Jobs ----------------------------------
@app.get("/jobs", response_class=HTMLResponse)
def jobs_page():
    rows = ""
    for j in jobs_index():
        if j["alive"]:
            st = '<span class="badge b-run">running</span>'
            act = f'<button class=danger onclick="kill({jsattr(j["id"])})">중지</button>'
        else:
            st = '<span class="badge b-ok">done</span>'
            act = f'<button onclick="delJob({jsattr(j["id"])})">삭제</button>'
        rows += (f'<tr><td class=mono>{esc(j["id"])}</td><td>{esc(j["kind"])}</td><td>{st}</td>'
                 f'<td class=mono style="color:var(--muted)">{esc(j.get("started", ""))}</td>'
                 f'<td style="text-align:right"><a href="/jobs/{esc(j["id"])}">log</a> &nbsp;{act}</td></tr>')
    empty = '' if rows else '<tr><td colspan=5 class=muted>작업 기록 없음</td></tr>'
    return f"""{CSS}{nav_html('jb')}<div class=wrap>
    <p class=eyebrow>Background processes</p><h2>Jobs</h2>
    <div class=card><table>
    <tr><th>id</th><th>kind</th><th>status</th><th>started</th><th></th></tr>{rows}{empty}</table></div>
    <p class=muted>중지 1회 = 정상 종료(SIGINT) · 한 번 더 = 강제종료 · 삭제 = 기록·로그 제거 (끝난 작업만)</p></div>
    <script>
    async function kill(id){{ await fetch('/api/kill/'+id,{{method:'POST'}}); setTimeout(()=>location.reload(),1500); }}
    async function delJob(id){{ await fetch('/api/deljob/'+id,{{method:'POST'}}); location.reload(); }}
    setTimeout(()=>location.reload(), 10000);
    </script>"""


@app.get("/jobs/{jid}", response_class=HTMLResponse)
def job_log(jid: str):
    if not safe_name(jid):
        return HTMLResponse(f"{CSS}{nav_html('jb')}<div class=wrap>잘못된 작업 id</div>", 400)
    j = load_json(JOB_DIR / f"{jid}.json", {})
    txt = ""
    try:
        txt = Path(j["log"]).read_text(errors="ignore")[-8000:]
    except Exception:
        pass
    return f"""{CSS}{nav_html('jb')}<div class=wrap>
    <p class=eyebrow>Job log</p><h2 class=mono style="font-size:17px">{esc(jid)}</h2>
    <pre style="max-height:none">{esc(txt) or '(로그 없음)'}</pre>
    <p class="muted mono">cmd: {esc(j.get('cmd', ''))}</p>
    <script>setTimeout(()=>location.reload(),5000)</script></div>"""


@app.post("/api/kill/{jid}")
def api_kill(jid: str):
    kill_job(jid)
    return {"ok": True}


@app.post("/api/deljob/{jid}")
def api_deljob(jid: str):
    ok = delete_job(jid)
    return {"ok": ok} if ok else JSONResponse({"error": "실행 중인 작업"}, status_code=400)


# ----------------------------- 비디오 서빙 -----------------------------------
@app.get("/videos/{ds}/{rest:path}")
def serve_video(ds: str, rest: str):
    if not safe_name(ds):
        return JSONResponse({"error": "not found"}, status_code=404)
    p = (DATA_ROOT / ds / "videos" / rest).resolve()
    if not str(p).startswith(str(DATA_ROOT.resolve())) or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p, media_type="video/mp4")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "--worker":
        sys.exit(worker_main(sys.argv[2], sys.argv[3]))
    print(f"data : {DATA_ROOT}\nouts : {OUT_ROOT}\njobs : {JOB_DIR}\nrun  : {RUN_DIR}")
    print(f"conf : {CONFIG_FILE}  (mode={CFG['mode']}, arms={SIDES}, cams={list(CAM_SPECS)})")
    if AUTH_TOKEN:
        print(f"open : http://<host>:{PORT}/?token={AUTH_TOKEN}   (token: {TOKEN_FILE})")
    else:
        print(f"open : http://<host>:{PORT}/   (인증 없음 — 켜려면 LRWEB_AUTH=on)")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
