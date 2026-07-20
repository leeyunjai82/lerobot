#!/usr/bin/env python3
# ============================================================================
#  lrweb.py v9.0 — LeRobot 통합 웹 툴 (단일 파일 FastAPI)
#
#  v7 추가
#   - Collect: 웹에서 데이터 수집 시작/조작 — n(다음)/r(재녹화)/q(종료) 버튼
#              (record 프로세스를 PTY로 띄워 키 입력을 주입)
#   - Rollout: 체크포인트 선택 → 자율 구동 시작/중지, 로그 확인
#   - Control 탭: 팔로워 수동 제어 + 카메라 MJPEG 스트리밍 (탭 진입=연결, 이탈=해제)
#   - record/rollout/train/Control 자원 기반 배타 (학습+Control 동시 허용)
#
#  실행 (lerobot conda env 안에서)
#   source ~/project/lerobot/activate.sh
#   pip install fastapi uvicorn
#   nohup python ~/project/lerobot/lrweb.py > ~/project/lerobot/lrweb.log 2>&1 &
#   → http://<thor-ip>:8080
# ============================================================================
import asyncio, json, re, glob, os, pty as _pty, signal, shutil, subprocess, time, shlex
from pathlib import Path

import pandas as pd
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse, Response

# ------------------------------- 설정 ---------------------------------------
HOME = Path.home()
DATA_ROOT = HOME / "project/lerobot/data/hf/lerobot/local"
OUT_ROOT = HOME / "project/lerobot/outputs"
JOB_DIR = HOME / "project/lerobot/lrweb_jobs"
MARKS_FILE = HOME / "project/lerobot/lrweb_marks.json"
PORT = 8080

CAMERAS = ("{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, "
           "top: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}")
ROBOT_ARGS = ("--robot.type=so101_follower --robot.port=/dev/so101_follower "
              f"--robot.id=follower --robot.cameras=\"{CAMERAS}\"")
TELEOP_ARGS = "--teleop.type=so101_leader --teleop.port=/dev/so101_leader --teleop.id=leader"
DEFAULT_TASK = "Pick up the block and place it in the box"

FOLLOWER_PORT = "/dev/so101_follower"
CALIB_FILE = HOME / "project/lerobot/data/hf/lerobot/calibration/robots/so_follower/follower.json"
URDF_DIR = HOME / "project/lerobot/urdf"      # so101.urdf + meshes/ 를 두면 3D 표시
CONTROL_HZ = 20
FEEDBACK_HZ = 10
MAX_STEP_DEG = 2.5
CTL_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CTL_CAMS = {"wrist": 0, "top": 2}     # Control 탭 스트리밍용 (record 설정과 동일)
CTL_STREAM_FPS = 15
# ---------------------------------------------------------------------------

JOB_DIR.mkdir(parents=True, exist_ok=True)
OUT_ROOT.mkdir(parents=True, exist_ok=True)
app = FastAPI(title="lrweb")

# ----------------------------- 유틸 -----------------------------------------
def load_json(p, default):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return default

def save_json(p, obj):
    Path(p).write_text(json.dumps(obj, indent=1, ensure_ascii=False))

def load_marks():  return load_json(MARKS_FILE, {})
def save_marks(m): save_json(MARKS_FILE, m)

_ICON_CACHE = {}
def _solid_png(size, rgb):
    """의존성 없는 단색 PNG (PIL 없을 때 폴백)."""
    import struct, zlib
    w = h = size; r, g, b = rgb
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
        pad = int(S * 0.11); rad = int(S * 0.22)
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
        buf = io.BytesIO(); img.save(buf, "PNG"); data = buf.getvalue()
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
                            "fps": info.get("fps", "?")})
    return out

def _clamp_int(v, default, lo, hi):
    try:
        n = int(v)
    except (TypeError, ValueError):
        n = default
    return max(lo, min(hi, n))

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
    # 같은 실제 경로는 하나만 — 숫자 체크포인트(is_last=False)를 우선 채택
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
    row = row.iloc[0]; segs = []
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

# ----------------------------- 작업(job) 관리 --------------------------------
PTYS = {}   # jid -> pty master fd (record 키 입력용, lrweb 프로세스 생존 동안만 유효)

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

# ----------------------------- 수동 제어 (Control 탭) -------------------------
class Arm:
    def __init__(self):
        self.bus = None
        self.torque = False
        self.target = {}      # 사용자가 원하는 최종 목표 (슬라이더)
        self.cmd = {}         # 명령 적분기 — target을 향해 제한 속도로 이동, 엔코더와 섞지 않음
        self.actual = {}      # 엔코더 실측 (표시/3D 전용)
        self.limits = {}
        self.err = ""

    def connect(self):
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorNormMode, MotorCalibration
        motors = {n: Motor(i + 1, "sts3215",
                           MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES)
                  for i, n in enumerate(CTL_JOINTS)}
        calib = {}
        if CALIB_FILE.exists():
            raw = json.loads(CALIB_FILE.read_text())
            for n, c in raw.items():
                if n in motors:
                    calib[n] = MotorCalibration(**c)
        self.bus = FeetechMotorsBus(FOLLOWER_PORT, motors, calibration=calib or None)
        self.bus.connect()
        for n in CTL_JOINTS:
            self.limits[n] = (0.0, 100.0) if n == "gripper" else (-170.0, 170.0)
        for n, c in (calib or {}).items():
            if n == "gripper":
                continue
            lo = (c.range_min - 2048) / 4096 * 360
            hi = (c.range_max - 2048) / 4096 * 360
            if lo > hi:
                lo, hi = hi, lo
            self.limits[n] = (round(lo, 1), round(hi, 1))
        self.actual = self.read()
        self.target = dict(self.actual)
        self.cmd = dict(self.actual)

    def disconnect(self):
        if self.bus:
            try: self.bus.disable_torque()
            except Exception: pass
            try: self.bus.disconnect()
            except Exception: pass
        self.bus = None
        self.torque = False
        self.err = ""

    def read(self):
        return {k: float(v) for k, v in self.bus.sync_read("Present_Position").items()}

    def set_torque(self, on: bool):
        (self.bus.enable_torque if on else self.bus.disable_torque)()
        self.torque = on
        if on:
            # 점프 방지: 현재 자세를 명령/목표의 시작점으로
            self.actual = self.read()
            self.target = dict(self.actual)
            self.cmd = dict(self.actual)

    def step(self):
        """cmd를 target으로 제한 속도 이동. 엔코더 값은 절대 섞지 않음(떨림 방지).
        변화가 없으면 버스에 쓰지 않음 — 도달 후엔 서보가 자체 유지."""
        if not (self.bus and self.torque):
            return
        goal = {}
        for n in CTL_JOINTS:
            cur = self.cmd.get(n, self.actual.get(n, 0.0))
            lo, hi = self.limits[n]
            tgt = max(lo, min(hi, self.target.get(n, cur)))
            diff = tgt - cur
            if abs(diff) < 0.2:          # 데드밴드: 도달로 간주, 쓰기 중단
                self.cmd[n] = tgt
                continue
            nxt = cur + max(-MAX_STEP_DEG, min(MAX_STEP_DEG, diff))
            self.cmd[n] = nxt
            goal[n] = nxt
        if goal:
            self.bus.sync_write("Goal_Position", goal)

ARM = Arm()
CTL_OWNER = None    # 현재 Control 탭을 점유한 WebSocket

class Cams:
    """Control 탭 전용 카메라 스트리머 — 리더 스레드 1개가 JPEG 최신본 유지"""
    def __init__(self):
        self.on = False
        self.frames = {}     # name -> jpeg bytes
        self.thread = None

    def open(self):
        import threading
        try:
            import cv2
        except ImportError:
            return False
        self.caps = {}
        for name, idx in CTL_CAMS.items():
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.caps[name] = cap
            else:
                cap.release()
        if not self.caps:
            return False
        self.on = True
        def loop():
            import cv2, time as _t
            while self.on:
                t0 = _t.monotonic()
                for name, cap in self.caps.items():
                    ok, frame = cap.read()
                    if ok:
                        ok2, buf = cv2.imencode(".jpg", frame,
                                                [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok2:
                            self.frames[name] = buf.tobytes()
                _t.sleep(max(0.0, 1.0 / CTL_STREAM_FPS - (_t.monotonic() - t0)))
            for cap in self.caps.values():
                cap.release()
            self.caps = {}
        self.thread = threading.Thread(target=loop, daemon=True)
        self.thread.start()
        return True

    def close(self):
        self.on = False
        self.frames = {}

CAMS = Cams()

def busy_with(kinds):
    for j in jobs_index():
        if j["alive"] and j["kind"] in kinds:
            return j
    return None

def robot_busy():
    """팔(시리얼)을 쓰는 작업: record/rollout + Control 탭 수동 제어"""
    if ARM.bus is not None:
        return {"id": "manual-control", "kind": "control", "alive": True}
    return busy_with(("record", "rollout"))

def gpu_or_loop_busy():
    """학습 시작을 막아야 하는 작업 (Control 수동 제어는 학습과 동시 가능)"""
    return busy_with(("record", "rollout", "train"))

def exclusive_busy():
    if ARM.bus is not None:
        return {"id": "manual-control (Control 탭)", "kind": "control", "alive": True}
    return busy_with(("record", "rollout", "train"))

def start_job(kind, cmd, cwd=None, use_pty=False):
    jid = f"{kind}_{time.strftime('%m%d_%H%M%S')}"
    log = JOB_DIR / f"{jid}.log"
    lf = open(log, "w")
    if use_pty:
        master, slave = _pty.openpty()
        p = subprocess.Popen(cmd, shell=True, cwd=cwd or str(HOME),
                             stdin=slave, stdout=lf, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        os.close(slave)
        PTYS[jid] = master
    else:
        p = subprocess.Popen(cmd, shell=True, cwd=cwd or str(HOME),
                             stdout=lf, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
    save_json(JOB_DIR / f"{jid}.json",
              {"id": jid, "kind": kind, "cmd": cmd, "pid": p.pid,
               "log": str(log), "started": time.strftime("%F %T")})
    return jid

def send_key(jid, key):
    fd = PTYS.get(jid)
    if fd is None:
        return False
    try:
        os.write(fd, key.encode())
        return True
    except OSError:
        return False

def kill_job(jid):
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
    j = load_json(JOB_DIR / f"{jid}.json", {})
    if j and pid_alive(j.get("pid")):
        return False
    for suffix in (".json", ".log"):
        try:
            (JOB_DIR / f"{jid}{suffix}").unlink()
        except FileNotFoundError:
            pass
    PTYS.pop(jid, None)
    return True

def log_tail(jid, nbytes=4000):
    j = load_json(JOB_DIR / f"{jid}.json", {})
    try:
        with open(j["log"], "rb") as f:
            f.seek(max(-nbytes, -os.path.getsize(j["log"])), 2)
            return f.read().decode(errors="ignore")[-3200:]
    except Exception:
        return ""

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
  border-color:var(--line);color:var(--muted)}
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

def nav_html(active=""):
    running = [j for j in jobs_index() if j["alive"]]
    if ARM.bus is not None:
        running = [{"id": "manual-control", "kind": "control"}] + running
    if running:
        j = running[0]
        extra = f' +{len(running)-1}' if len(running) > 1 else ''
        cluster = (f'<div class=statuscluster><span class="dot live"></span>'
                   f'{j["kind"].upper()} · {j["id"]}{extra}</div>')
    else:
        cluster = '<div class=statuscluster><span class=dot></span>IDLE</div>'
    def tab(href, label, key):
        on = ' class=on' if key == active else ''
        return f'<a href="{href}"{on}>{label}</a>'
    return (f'<div class=appbar><div class=brand>LRWEB <small>/ SO-101 PIPELINE</small></div>'
            f'<div class=nav>{tab("/","Datasets","ds")}{tab("/collect","Collect","co")}'
            f'{tab("/train","Training","tr")}{tab("/rollout","Rollout","ro")}'
            f'{tab("/control","Control","ct")}{tab("/jobs","Jobs","jb")}</div>{cluster}'
            f'<button class=fsbtn title="전체화면" aria-label="전체화면" onclick="toggleFS()">⛶</button></div>')

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
    js = ("self.addEventListener('install',function(e){self.skipWaiting();});"
          "self.addEventListener('activate',function(e){self.clients.claim();});"
          "self.addEventListener('fetch',function(){});")
    return Response(content=js, media_type="application/javascript")

# ----------------------------- 페이지: 데이터셋 -------------------------------
@app.get("/", response_class=HTMLResponse)
def index():
    rows = "".join(
        f'<tr><td><a href="/ds/{d["name"]}" class=mono>{d["name"]}</a></td>'
        f'<td class=num>{d["episodes"]}</td><td class=num>{d["frames"]}</td><td class=num>{d["fps"]}</td>'
        f'<td style="text-align:right"><button class=danger onclick="delDs(\'{d["name"]}\')">삭제</button></td></tr>'
        for d in list_datasets())
    empty = '' if rows else '<tr><td colspan=5 class=muted>데이터셋이 없습니다 — Collect 탭에서 수집을 시작하세요</td></tr>'
    return f"""{CSS}{nav_html('ds')}<div class=wrap>
    <p class=eyebrow>Local datasets</p><h2>Datasets</h2>
    <div class=card><table>
    <tr><th>name</th><th class=num>episodes</th><th class=num>frames</th><th class=num>fps</th><th></th></tr>
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
    p = (DATA_ROOT / ds).resolve()
    if not str(p).startswith(str(DATA_ROOT.resolve())) or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    if not (p / "meta/info.json").exists():
        return JSONResponse({"error": "데이터셋 폴더가 아님"}, status_code=400)
    for j in jobs_index():
        if j["alive"] and ds in j.get("cmd", ""):
            return JSONResponse({"error": f"실행 중인 작업({j['id']})이 이 데이터셋을 사용 중"}, status_code=400)
    shutil.rmtree(p)
    m = load_marks(); m.pop(ds, None); save_marks(m)
    return {"ok": True}

@app.get("/ds/{ds}", response_class=HTMLResponse)
def dataset_page(ds: str):
    df = episodes_df(ds)
    if df.empty:
        return HTMLResponse(f"{CSS}{nav_html('ds')}<div class=wrap>메타데이터 없음: {ds}</div>")
    marks = load_marks().get(ds, [])
    rows = []
    for _, r in df.iterrows():
        ep = int(r["episode_index"])
        badge = ' <span class="badge b-bad">bad</span>' if ep in marks else ""
        rows.append(f'<tr><td><a href="/ds/{ds}/ep/{ep}">episode <span class=mono>{ep}</span></a>{badge}</td>'
                    f'<td class=num>{r.get("length","?")}</td><td>{r.get("tasks","")}</td></tr>')
    markbar = ""
    if marks:
        markbar = f"""<div class=markbar>
        <span>마킹된 에피소드 <b class=mono>{sorted(marks)}</b></span>
        <button class=danger onclick="delMarked()">웹에서 바로 삭제 실행</button>
        <span class=muted>결과는 새 폴더로 생성됨 (원본 유지) · Jobs에서 진행 확인</span></div>
        <script>async function delMarked(){{
          if(!confirm('마킹된 {len(marks)}개 에피소드를 삭제 실행할까요?'))return;
          await fetch('/api/delete/{ds}',{{method:'POST'}}); location.href='/jobs';
        }}</script>"""
    return f"""{CSS}{nav_html('ds')}<div class=wrap>
    <p class=eyebrow>Dataset</p><h2 class=mono style="font-size:17px">{ds}</h2>
    {markbar}
    <div class=card><table>
    <tr><th>episode</th><th class=num>frames</th><th>task</th></tr>{"".join(rows)}</table></div></div>"""

@app.get("/ds/{ds}/ep/{ep}", response_class=HTMLResponse)
def episode_page(ds: str, ep: int):
    df = episodes_df(ds)
    segs = ep_video_segments(df, ep)
    n_ep = int(df["episode_index"].max()) + 1 if not df.empty else 0
    marks = load_marks().get(ds, [])
    if not segs:
        cols = "<br>".join(df.columns) if not df.empty else "(none)"
        return HTMLResponse(f"{CSS}{nav_html('ds')}<div class=wrap>영상 세그먼트를 못 찾음.<br>"
                            f"<span class=muted>메타 컬럼: {cols}</span></div>")
    vids = "".join(
        f"""<div class=vpanel>
        <div class=vhead><b>{s['cam']}</b><span>{s['from']:.1f}s &rarr; {s['to']:.1f}s</span></div>
        <video src="/videos/{ds}/{s['path']}" controls muted
               data-from="{s['from']}" data-to="{s['to']}"></video></div>"""
        for s in segs)
    marked = ep in marks
    return f"""{CSS}{nav_html('ds')}<div class=wrap>
    <p class=eyebrow>{ds}</p>
    <h2>episode <span class=mono style="font-size:inherit">{ep}</span>
      <span class="badge {'b-bad' if marked else 'b-ok'}">{'bad' if marked else 'ok'}</span></h2>
    <div class=toolbar>
       <a href="/ds/{ds}/ep/{max(ep-1,0)}"><button>&larr; prev</button></a>
       <a href="/ds/{ds}/ep/{min(ep+1,n_ep-1)}"><button>next &rarr;</button></a>
       <button class=primary onclick="playAll()">&#9654; 재생</button>
       <button onclick="mark()">{'마킹 해제' if marked else '불량 마킹'}</button>
       <a href="/ds/{ds}"><button>목록</button></a>
       <span class=muted style="margin-left:auto">episode {ep} / {n_ep-1} · 단축키 &larr;/&rarr; Space b</span></div>
    <div class=grid>{vids}</div></div>
    <script>
    const vids=[...document.querySelectorAll('video')];
    vids.forEach(v=>{{
      v.addEventListener('loadedmetadata',()=>{{v.currentTime=parseFloat(v.dataset.from);}});
      v.addEventListener('timeupdate',()=>{{
        if(v.currentTime>=parseFloat(v.dataset.to)){{v.pause();v.currentTime=parseFloat(v.dataset.from);}}
      }});
    }});
    function playAll(){{vids.forEach(v=>{{v.currentTime=parseFloat(v.dataset.from);v.play();}});}}
    async function mark(){{await fetch('/api/mark/{ds}/{ep}',{{method:'POST'}});location.reload();}}
    document.addEventListener('keydown',e=>{{
      if(e.target.tagName==='INPUT')return;
      if(e.key==='ArrowLeft')location.href='/ds/{ds}/ep/{max(ep-1,0)}';
      if(e.key==='ArrowRight')location.href='/ds/{ds}/ep/{min(ep+1,n_ep-1)}';
      if(e.key===' '){{e.preventDefault();playAll();}}
      if(e.key==='b')mark();
    }});
    </script>"""

@app.post("/api/mark/{ds}/{ep}")
def api_mark(ds: str, ep: int):
    m = load_marks(); lst = set(m.get(ds, []))
    lst.symmetric_difference_update({ep})
    m[ds] = sorted(lst); save_marks(m)
    return {"ok": True, "marks": m[ds]}

@app.post("/api/delete/{ds}")
def api_delete(ds: str):
    marks = load_marks().get(ds, [])
    if not marks:
        return JSONResponse({"error": "no marks"}, status_code=400)
    root = DATA_ROOT / ds
    cmd = (f"lerobot-edit-dataset --repo_id local/{ds} --root {shlex.quote(str(root))} "
           f"--operation.type delete_episodes "
           f"--operation.episode_indices \"{sorted(marks)}\"")
    jid = start_job("delete", cmd)
    m = load_marks(); m[ds] = []; save_marks(m)
    return {"ok": True, "job": jid}

# ----------------------------- 페이지: 수집 (Collect) ------------------------
@app.get("/collect", response_class=HTMLResponse)
def collect_page():
    rec = next((j for j in jobs_index() if j["kind"] == "record" and j["alive"]), None)
    if rec:
        has_pty = rec["id"] in PTYS
        keywarn = "" if has_pty else ('<p class="badge b-warn">키 입력 불가 — lrweb 재시작 이후의 세션이라 '
                                      '시간 기반으로만 진행됩니다. 종료는 중지 버튼 사용</p>')
        keys = ""
        if has_pty:
            keys = """<div class=keys>
            <button class="primary big" onclick="key('n')">n &nbsp;다음</button>
            <button class=big onclick="key('r')">r &nbsp;재녹화</button>
            <button class="danger big" onclick="key('q')">q &nbsp;종료·저장</button></div>"""
        return f"""{CSS}{nav_html('co')}<div class=wrap>
        <p class=eyebrow>Recording</p><h2>수집 진행 중</h2>
        <div class=runbar><span class="badge b-run">recording</span>
          <span class=mono>{rec["id"]}</span>
          <button class=danger onclick="stopRec('{rec["id"]}')">강제 중지</button></div>
        {keywarn}{keys}
        <p class=eyebrow>Log</p><pre id=tail>...</pre></div>
        <script>
        async function key(k){{ await fetch('/api/sendkey/{rec["id"]}/'+k,{{method:'POST'}}); }}
        async function stopRec(id){{
          if(!confirm('강제 중지할까요? (가능하면 q 종료·저장을 쓰세요)'))return;
          await fetch('/api/kill/'+id,{{method:'POST'}}); setTimeout(()=>location.reload(),1500);
        }}
        async function refresh(){{
          const r=await fetch('/api/joblog/{rec["id"]}'); const d=await r.json();
          document.getElementById('tail').textContent=d.tail||'';
          if(!d.alive) location.reload();
        }}
        refresh(); setInterval(refresh,2000);
        document.addEventListener('keydown',e=>{{
          if(e.target.tagName==='INPUT')return;
          if(['n','r','q'].includes(e.key)) key(e.key);
        }});
        </script>"""
    busy = exclusive_busy()
    busywarn = (f'<p class="badge b-warn">실행 중: {busy["id"]} — 끝나야 수집을 시작할 수 있습니다</p>'
                if busy else "")
    resume_opts = "".join(f'<option value="{d["name"]}">{d["name"]} ({d["episodes"]}ep)</option>'
                          for d in list_datasets())
    return f"""{CSS}{nav_html('co')}<div class=wrap>
    <p class=eyebrow>Teleoperation record</p><h2>Collect</h2>
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
        <input id=task value="{DEFAULT_TASK}"></label>
      <button class=primary onclick="startRec()">수집 시작</button>
    </div>
    <p class=muted>시작 즉시 에피소드 0 녹화가 시작됩니다 — 물체·리더암을 먼저 준비하세요.
    시작 후 이 페이지에서 n(다음) / r(재녹화) / q(종료) 버튼 또는 키보드로 조작합니다.</p>
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
    busy = exclusive_busy()   # control 포함 — lrctl이 팔을 잡고 있으면 차단
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 시작하세요"}, status_code=400)
    b = await req.json()
    task = (b.get("task") or DEFAULT_TASK).replace('"', "'")
    neps = _clamp_int(b.get("num_episodes"), 50, 1, 100000)
    ept = _clamp_int(b.get("episode_time_s"), 30, 1, 3600)
    rst = _clamp_int(b.get("reset_time_s"), 15, 0, 3600)
    if b.get("mode") == "resume":
        ds = (b.get("resume_ds") or "").strip()
        if not re.fullmatch(r"[A-Za-z0-9._-]+", ds):
            return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
        root = DATA_ROOT / ds
        if not root.exists():
            return JSONResponse({"error": "데이터셋 없음"}, status_code=400)
        ds_args = (f"--dataset.repo_id=local/{ds} --dataset.root={shlex.quote(str(root))} "
                   f"--resume=true")
    else:
        name = (b.get("name") or "").strip()
        if not re.fullmatch(r"[A-Za-z0-9._-]+", name or ""):
            return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
        ds_args = f"--dataset.repo_id=local/{name}"
    cmd = (f"lerobot-record {ROBOT_ARGS} {TELEOP_ARGS} {ds_args} "
           f"--dataset.num_episodes={neps} --dataset.single_task={shlex.quote(task)} "
           f"--dataset.episode_time_s={ept} --dataset.reset_time_s={rst} "
           f"--dataset.push_to_hub=false --dataset.fps=30")
    jid = start_job("record", cmd, use_pty=True)
    return {"ok": True, "job": jid}

@app.post("/api/sendkey/{jid}/{key}")
def api_sendkey(jid: str, key: str):
    if key not in ("n", "r", "q"):
        return JSONResponse({"error": "허용되지 않은 키"}, status_code=400)
    ok = send_key(jid, key)
    return {"ok": ok} if ok else JSONResponse({"error": "키 전달 실패 (PTY 없음)"}, status_code=400)

@app.get("/api/joblog/{jid}")
def api_joblog(jid: str):
    j = load_json(JOB_DIR / f"{jid}.json", {})
    return JSONResponse({"tail": log_tail(jid), "alive": pid_alive(j.get("pid"))})

# ----------------------------- 페이지: 학습 ----------------------------------
@app.get("/train", response_class=HTMLResponse)
def train_page():
    ds_opts = "".join(f'<option value="{d["name"]}">{d["name"]} ({d["episodes"]}ep)</option>'
                      for d in list_datasets())
    running = [j for j in jobs_index() if j["kind"] == "train" and j["alive"]]
    run_html = "".join(
        f'<div class=runbar><span class="badge b-run">running</span> <span class=mono>{j["id"]}</span> '
        f'<button class=danger onclick="stopJob(\'{j["id"]}\')">중지</button></div>'
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
    busy = gpu_or_loop_busy()   # lrctl 수동 제어(control)는 학습과 동시 가능
    if busy:
        return JSONResponse({"error": f"{busy['id']} 실행 중 — 종료 후 시작하세요"}, status_code=400)
    ds = (b.get("dataset") or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9._-]+", ds):
        return JSONResponse({"error": "데이터셋 이름은 영문/숫자/._- 만"}, status_code=400)
    name = (b.get("name") or f"act_{ds}").strip()
    if not re.fullmatch(r"[A-Za-z0-9._-]+", name):
        return JSONResponse({"error": "출력 이름은 영문/숫자/._- 만"}, status_code=400)
    steps = _clamp_int(b.get("steps"), 80000, 1, 100000000)
    batch = _clamp_int(b.get("batch"), 8, 1, 4096)
    root = DATA_ROOT / ds
    if not root.exists():
        return JSONResponse({"error": "dataset not found"}, status_code=400)
    out = OUT_ROOT / name
    cmd = (f"lerobot-train --dataset.repo_id=local/{ds} --dataset.root={shlex.quote(str(root))} "
           f"--policy.type=act --output_dir={shlex.quote(str(out))} "
           f"--steps={steps} --batch_size={batch} --num_workers=4 "
           f"--save_freq=10000 --policy.push_to_hub=false")
    jid = start_job("train", cmd)
    return {"ok": True, "job": jid}

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
                    try:
                        pts.append([int(m.group(1).replace("K", "000")), float(m.group(2))])
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
          <span class=mono>{ro["id"]}</span>
          <button class=danger onclick="stopRo('{ro["id"]}')">중지</button>
          <span class=muted>중지(SIGINT) 시 시작 자세로 복귀 후 토크 해제됩니다</span></div>
        <p class=eyebrow>Log</p><pre id=tail>...</pre></div>
        <script>
        async function stopRo(id){{
          if(!confirm('추론을 중지할까요?'))return;
          await fetch('/api/kill/'+id,{{method:'POST'}}); setTimeout(()=>location.reload(),1500);
        }}
        async function refresh(){{
          const r=await fetch('/api/joblog/{ro["id"]}'); const d=await r.json();
          document.getElementById('tail').textContent=d.tail||'';
          if(!d.alive) location.reload();
        }}
        refresh(); setInterval(refresh,2000);
        </script>"""
    busy = exclusive_busy()
    busywarn = (f'<p class="badge b-warn">실행 중: {busy["id"]} — 끝나야 추론을 시작할 수 있습니다</p>'
                if busy else "")
    ckpts = list_checkpoints()
    ck_opts = "".join(f'<option value="{c}">{c}</option>' for c in ckpts)
    empty = "" if ckpts else '<p class=muted>체크포인트가 없습니다 — Training에서 학습을 먼저 완료하세요</p>'
    return f"""{CSS}{nav_html('ro')}<div class=wrap>
    <p class=eyebrow>Autonomous run</p><h2>Rollout</h2>
    {busywarn}{empty}
    <div class=card>
    <div class=formgrid>
      <label class=f style="min-width:380px">체크포인트
        <select id=ckpt>{ck_opts}</select></label>
      <label class=f>실행 시간(초, 0=무한) <input id=dur value=60 size=6></label>
      <label class=f style="flex:1;min-width:260px">태스크 설명
        <input id=task value="{DEFAULT_TASK}"></label>
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
    task = (b.get("task") or DEFAULT_TASK).replace('"', "'")
    cmd = (f"lerobot-rollout --policy.path={shlex.quote(str(ck))} {ROBOT_ARGS} "
           f"--strategy.type=base --duration={dur} --task={shlex.quote(task)}")
    jid = start_job("rollout", cmd)
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
    busy = busy_with(("record", "rollout"))
    if busy:
        return f"""{CSS}{nav_html('ct')}<div class=wrap>
        <p class=eyebrow>Manual control</p><h2>Control</h2>
        <p class="badge b-warn">실행 중: {busy["id"]} — 끝나야 수동 제어를 쓸 수 있습니다</p></div>"""
    joints_js = json.dumps(CTL_JOINTS)
    return f"""{CSS}{nav_html('ct')}
<style>
.cmain{{display:grid;grid-template-columns:360px 1fr;gap:0;height:calc(100vh - 52px - 0px)}}
.cpanel{{border-right:1px solid var(--line);padding:18px;overflow-y:auto}}
.jrow{{margin-bottom:16px}}
.jhead{{display:flex;justify-content:space-between;font-family:var(--mono);font-size:12px;margin-bottom:4px}}
.jhead .n{{color:var(--text)}} .jhead .v{{color:var(--accent)}} .jhead .a{{color:var(--dim)}}
input[type=range]{{width:100%;accent-color:var(--accent)}}
#right{{display:flex;flex-direction:column;min-height:0}}
.cams{{display:none;grid-template-columns:1fr 1fr;gap:1px;background:var(--line);border-bottom:1px solid var(--line)}}
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
      <button class=estop onclick="estop()">E-STOP</button>
      <input type=color id=armcolor value="#e07a3f" title="로봇 색상"
             style="width:34px;height:30px;padding:2px;border-radius:7px;border:1px solid var(--line);background:var(--bg);cursor:pointer">
    </div>
    <div id=sliders></div>
    <p class=muted>토크 OFF: 손으로 움직이면 값·3D가 따라옵니다.<br>
    토크 ON: 슬라이더가 목표 (스텝당 최대 {MAX_STEP_DEG}° 제한).<br>
    이 탭을 떠나면 자동으로 토크 해제 + 연결 해제됩니다.</p>
  </div>
  <div id=right>
    <div class=cams id=cams>
      <div class=cw><span class=cl>wrist</span><img id=cam_wrist></div>
      <div class=cw><span class=cl>top</span><img id=cam_top></div>
    </div>
    <div id=view><div id=nourdf>~/project/lerobot/urdf/so101.urdf 없음<br>
    so101-sim의 URDF와 meshes/ 를 복사하면 3D 표시<br>(슬라이더 제어는 그대로 동작)</div></div>
  </div>
</div>
<script type="importmap">
{{"imports":{{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/",
"three/examples/jsm/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/",
"urdf-loader":"https://cdn.jsdelivr.net/npm/urdf-loader@0.12.6/src/URDFLoader.js"}}}}
</script>
<script type="module">
const JOINTS = {joints_js};
let ws=null, torque=false, robot=null;
const sliders={{}}, valEls={{}}, actEls={{}};
const cst=document.getElementById('cst'), btrq=document.getElementById('btrq');

function buildSliders(lims, actual){{
  const box=document.getElementById('sliders'); box.innerHTML='';
  JOINTS.forEach(j=>{{
    const [lo,hi]=lims[j];
    const row=document.createElement('div'); row.className='jrow';
    row.innerHTML=`<div class=jhead><span class=n>${{j}}</span>
      <span><span class=v id=v_${{j}}>-</span> <span class=a id=a_${{j}}>(-)</span></span></div>
      <input type=range min=${{lo}} max=${{hi}} step=0.5 value=${{actual[j]??0}}>`;
    box.appendChild(row);
    const s=row.querySelector('input');
    sliders[j]=s; valEls[j]=row.querySelector('#v_'+j); actEls[j]=row.querySelector('#a_'+j);
    s.addEventListener('input',()=>{{
      if(ws&&ws.readyState===1){{
        const joints={{}}; JOINTS.forEach(k=>joints[k]=parseFloat(sliders[k].value));
        ws.send(JSON.stringify({{type:'target',joints}}));
      }}
    }});
  }});
}}
window.toggleTorque=()=>{{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({{type:'torque',on:!torque}})); }};
window.estop=()=>{{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({{type:'estop'}})); }};

function openWS(){{
  ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws/control');
  ws.onmessage=e=>{{
    const d=JSON.parse(e.data);
    if(d.type==='init'){{
      if(d.error){{ cst.textContent=d.error; cst.classList.add('b-bad'); return; }}
      buildSliders(d.limits,d.actual);
      cst.textContent='connected'; cst.classList.add('b-ok'); btrq.disabled=false;
      if(d.cams){{
        document.getElementById('cams').style.display='grid';
        document.getElementById('cam_wrist').src='/stream/wrist';
        document.getElementById('cam_top').src='/stream/top';
      }}
    }}
    if(d.type==='state'){{
      torque=d.torque;
      btrq.textContent=torque?'토크 OFF':'토크 ON';
      JOINTS.forEach(j=>{{
        if(valEls[j])valEls[j].textContent=(d.target[j]??0).toFixed(1);
        if(actEls[j])actEls[j].textContent='('+(d.actual[j]??0).toFixed(1)+')';
        if(!torque&&sliders[j]&&d.actual[j]!==undefined)sliders[j].value=d.actual[j];
        if(torque&&d.synced&&sliders[j]&&d.actual[j]!==undefined)sliders[j].value=d.actual[j];
      }});
      if(d.err){{cst.textContent='bus error';cst.classList.add('b-bad');}}
      updateRobot(d.actual);
    }}
  }};
  ws.onclose=()=>{{ cst.textContent='disconnected'; cst.classList.remove('b-ok'); btrq.disabled=true; }};
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
  const cam=new THREE.PerspectiveCamera(50,1,0.01,10); cam.position.set(0.4,0.35,0.4);
  const ren=new THREE.WebGLRenderer({{antialias:true}}); view.appendChild(ren.domElement);
  const ctl=new OrbitControls(cam,ren.domElement); ctl.target.set(0,0.12,0);
  scene.add(new THREE.HemisphereLight(0xffffff,0x223344,1.1));
  const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(1,2,1); scene.add(dl);
  scene.add(new THREE.GridHelper(1,20,0x28303a,0x1b222a));
  function resize(){{const w=view.clientWidth,h=view.clientHeight;ren.setSize(w,h);cam.aspect=w/h;cam.updateProjectionMatrix();}}
  new ResizeObserver(resize).observe(view); resize();
  const picker=document.getElementById('armcolor');
  picker.value=localStorage.getItem('armColor')||'#e07a3f';
  function applyColor(hex){{
    if(!robot)return;
    robot.traverse(o=>{{
      if(o.isMesh){{
        if(!o.userData.recolored){{
          o.material=new THREE.MeshStandardMaterial({{metalness:0.15,roughness:0.55}});
          o.userData.recolored=true;
        }}
        o.material.color.set(hex);
      }}
    }});
    localStorage.setItem('armColor',hex);
  }}
  picker.addEventListener('input',()=>applyColor(picker.value));
  const loader=new URDFLoader();
  loader.workingPath='/urdf/';
  loader.packages='/urdf';           // package://xxx/ 형태도 /urdf/로 해석
  loader.load('/urdf/so101.urdf',
    r=>{{robot=r;robot.rotation.x=-Math.PI/2;scene.add(robot);applyColor(picker.value);}},
    undefined,
    e=>{{console.error(e);showViewMsg('URDF 로드 실패<br>'+(e?.message||e));}});
  (function anim(){{requestAnimationFrame(anim);ctl.update();ren.render(scene,cam);}})();
  updateRobot=(actual)=>{{
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
    return {"connected": ARM.bus is not None, "torque": ARM.torque,
            "urdf": (URDF_DIR / "so101.urdf").exists()}

@app.websocket("/ws/control")
async def ws_control(sock: WebSocket):
    global CTL_OWNER
    await sock.accept()
    if busy_with(("record", "rollout")):
        await sock.send_text(json.dumps({"type": "init", "error": "record/rollout 실행 중 — 제어 불가"}))
        await sock.close(); return
    if CTL_OWNER is not None:
        await sock.send_text(json.dumps({"type": "init", "error": "다른 브라우저가 제어 중입니다"}))
        await sock.close(); return
    CTL_OWNER = sock
    try:
        if ARM.bus is None:
            try:
                await asyncio.to_thread(ARM.connect)
            except Exception as e:
                ARM.disconnect()
                await sock.send_text(json.dumps({"type": "init", "error": f"팔 연결 실패: {e}"}))
                return
        cams_ok = await asyncio.to_thread(CAMS.open)
        await sock.send_text(json.dumps({"type": "init", "limits": ARM.limits,
                                         "actual": ARM.actual, "cams": bool(cams_ok)}))

        synced = False   # 토크 토글 직후 슬라이더 동기화 신호 1회

        async def rx():
            nonlocal synced
            async for msg in sock.iter_text():
                try:
                    d = json.loads(msg)
                except ValueError:
                    continue
                if d.get("type") == "target":
                    for k, v in d.get("joints", {}).items():
                        if k in CTL_JOINTS:
                            ARM.target[k] = float(v)
                elif d.get("type") == "torque":
                    try:
                        await asyncio.to_thread(ARM.set_torque, bool(d.get("on")))
                        synced = True
                    except Exception as e:
                        ARM.err = str(e)
                elif d.get("type") == "estop":
                    try:
                        await asyncio.to_thread(ARM.set_torque, False)
                    except Exception as e:
                        ARM.err = str(e)

        rx_task = asyncio.create_task(rx())
        last_fb = 0.0
        try:
            while True:
                t0 = time.monotonic()
                if ARM.bus:
                    try:
                        await asyncio.to_thread(ARM.step)
                        if t0 - last_fb > 1.0 / FEEDBACK_HZ:
                            ARM.actual = await asyncio.to_thread(ARM.read)
                            last_fb = t0
                            ARM.err = ""
                    except Exception as e:
                        ARM.err = str(e)
                await sock.send_text(json.dumps(
                    {"type": "state", "actual": ARM.actual, "target": ARM.target,
                     "torque": ARM.torque, "synced": synced, "err": ARM.err}))
                synced = False
                dt = time.monotonic() - t0
                await asyncio.sleep(max(0.0, 1.0 / CONTROL_HZ - dt))
        finally:
            rx_task.cancel()
    except WebSocketDisconnect:
        pass
    finally:
        CTL_OWNER = None
        CAMS.close()       # 탭 이탈 = 카메라 해제
        ARM.disconnect()   #          + 토크 해제 + 시리얼 해제

# Control 카메라 MJPEG 스트림
from fastapi.responses import StreamingResponse

@app.get("/stream/{cam}")
def stream_cam(cam: str):
    if not CAMS.on:
        return JSONResponse({"error": "카메라 미가동 (Control 탭에서만 스트리밍)"}, status_code=503)
    if cam not in CTL_CAMS:
        return JSONResponse({"error": "unknown camera"}, status_code=404)
    boundary = b"--frame"
    def gen():
        import time as _t
        while CAMS.on:
            f = CAMS.frames.get(cam)
            if f:
                yield (boundary + b"\r\nContent-Type: image/jpeg\r\n"
                       + f"Content-Length: {len(f)}\r\n\r\n".encode() + f + b"\r\n")
            _t.sleep(1.0 / CTL_STREAM_FPS)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

# URDF 정적 서빙
@app.get("/urdf/{rest:path}")
def serve_urdf(rest: str):
    p = (URDF_DIR / rest).resolve()
    if not str(p).startswith(str(URDF_DIR.resolve())) or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p)

# ----------------------------- 페이지: Jobs ----------------------------------
@app.get("/jobs", response_class=HTMLResponse)
def jobs_page():
    rows = ""
    for j in jobs_index():
        if j["alive"]:
            st = '<span class="badge b-run">running</span>'
            act = f'<button class=danger onclick="kill(\'{j["id"]}\')">중지</button>'
        else:
            st = '<span class="badge b-ok">done</span>'
            act = f'<button onclick="delJob(\'{j["id"]}\')">삭제</button>'
        rows += (f'<tr><td class=mono>{j["id"]}</td><td>{j["kind"]}</td><td>{st}</td>'
                 f'<td class=mono style="color:var(--muted)">{j.get("started","")}</td>'
                 f'<td style="text-align:right"><a href="/jobs/{j["id"]}">log</a> &nbsp;{act}</td></tr>')
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
    j = load_json(JOB_DIR / f"{jid}.json", {})
    txt = ""
    try:
        txt = Path(j["log"]).read_text(errors="ignore")[-8000:]
    except Exception:
        pass
    return f"""{CSS}{nav_html('jb')}<div class=wrap>
    <p class=eyebrow>Job log</p><h2 class=mono style="font-size:17px">{jid}</h2>
    <pre style="max-height:none">{txt or '(로그 없음)'}</pre>
    <p class="muted mono">cmd: {j.get('cmd','')}</p>
    <script>setTimeout(()=>location.reload(),5000)</script></div>"""

@app.post("/api/kill/{jid}")
def api_kill(jid: str):
    kill_job(jid); return {"ok": True}

@app.post("/api/deljob/{jid}")
def api_deljob(jid: str):
    ok = delete_job(jid)
    return {"ok": ok} if ok else JSONResponse({"error": "실행 중인 작업"}, status_code=400)

# ----------------------------- 비디오 서빙 -----------------------------------
@app.get("/videos/{ds}/{rest:path}")
def serve_video(ds: str, rest: str):
    p = (DATA_ROOT / ds / "videos" / rest).resolve()
    if not str(p).startswith(str(DATA_ROOT.resolve())) or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p, media_type="video/mp4")

if __name__ == "__main__":
    print(f"data: {DATA_ROOT}\nouts: {OUT_ROOT}\njobs: {JOB_DIR}")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
