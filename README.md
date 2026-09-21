# lerobot-tools — SO-101 / Jetson Thor pipeline

HuggingFace [lerobot](https://github.com/huggingface/lerobot) 기반 SO-101 로봇팔
수집·학습·추론 파이프라인용 웹 툴. (본 repo는 HF lerobot 자체가 아니며, 운용 도구 모음입니다)

목표는 **명령어 없이 웹에서 다 되게** 하는 것입니다. 진행 상황은 아래 [로드맵](#로드맵) 참고.

## 구성

| 파일 | 설명 |
|---|---|
| `lrweb.py` | 통합 웹툴 — Datasets / Collect / Training / Rollout / Control (port 8080) |
| `activate.sh` | conda env 활성화 + `HF_HOME` 설정 |
| `urdf/` | SO-101 URDF/STL (Control 탭 3D 뷰용) |
| `lerobot_conda.sh` | 새 기기 셋업 (conda env, PyTorch, lerobot 핀 커밋, 의존성) |
| `tools_jscheck.py` | 모든 페이지의 인라인 JS 를 `node --check` 로 파싱 검증 |

실행하면 `~/project/lerobot/` 아래에 자동 생성되는 것:

| 파일 | 설명 |
|---|---|
| `lrweb_config.json` | 포트·카메라·fps 설정 (아래 참고) |
| `lrweb_jobs/` | 백그라운드 작업 기록·로그 |
| `lrweb_marks.json` | 불량 에피소드 마킹 |

## lrweb 기능

- **Datasets**: 데이터셋·에피소드 목록, 에피소드 단위 영상 재생(청크 mp4 구간 시킹),
  불량 마킹 → 일괄 삭제, 데이터셋 통삭제
- **Collect**: 수집을 별도 worker 프로세스에서 lerobot `record_loop()` 로 직접 실행 —
  n(다음)/r(재녹화)/q(종료) 버튼, 에피소드/경과 진행바, **수집 중 카메라 미리보기**.
  lrweb 를 재시작해도 세션이 유지됩니다 (CLI·PTY 없음)
- **Training**: ACT 학습 시작/중지, loss 차트, 로그 tail
- **Rollout**: 체크포인트 자동 스캔 → 자율 구동 시작/중지
- **Control**: 팔로워 수동 제어 — 슬라이더(속도 제한), 토크/E-STOP,
  리더 팔로우, 카메라 MJPEG 스트리밍, Three.js URDF 3D. 팔마다 제어 스레드가 따로 돌아
  양팔에서도 왕복 지연이 쌓이지 않습니다. 탭 이탈 시 자동 해제
- **Setup**: USB 시리얼 포트 스캔·probe(모터 ID 확인)·**포트 감시로 leader/follower 판별**,
  **새 팔 모터 ID 세팅**(`lerobot-setup-motors` 의 웹 버전 — 모터 한 개씩 꽂고 gripper=6 → shoulder_pan=1),
  **카메라 스캔 시 썸네일 촬영**(어느 `/dev/videoN` 이 어느 카메라인지 눈으로 확인)·등록,
  한팔/양팔 모드 전환, 캘리브레이션 파일 상태 — 전부 웹에서
- **Calib**: 팔로워/리더 캘리브레이션을 웹에서 — 중앙 자세 기록 → 라이브 min/max 표시 → 저장.
  `lerobot-calibrate` 와 같은 버스 호출 순서, 같은 파일 경로·포맷
- record/rollout/train/control 자원 기반 상호 배타 (학습+수동제어는 동시 허용)

## 설정 — `lrweb_config.json`

첫 실행 때 **포트가 비어 있는** 기본값으로 생성됩니다. udev 심볼릭 링크(`/dev/so101_follower` 같은)를
전제하지 않습니다 — **Setup 탭**에서 스캔·판별해서 채웁니다. 손으로 편집해도 됩니다.

```json
{
  "mode": "bimanual",
  "robot_id": "so101",
  "fps": 30,
  "default_task": "Pick up the block and place it in the box",
  "max_relative_target": null,
  "arms": [
    { "side": "left",
      "follower_port": "/dev/serial/by-path/pci-0000:00-usb-0:1.1:1.0", "follower_id": "follower_left",
      "leader_port":   "/dev/serial/by-path/pci-0000:00-usb-0:1.2:1.0", "leader_id":   "leader_left",
      "cameras": { "wrist": { "index_or_path": "/dev/v4l/by-id/usb-...-video-index0", "width": 640, "height": 480, "fps": 30 } } },
    { "side": "right",
      "follower_port": "/dev/serial/by-path/pci-0000:00-usb-0:1.3:1.0", "follower_id": "follower_right",
      "leader_port":   "/dev/serial/by-path/pci-0000:00-usb-0:1.4:1.0", "leader_id":   "leader_right",
      "cameras": { "wrist": { "index_or_path": "/dev/v4l/by-id/usb-...-video-index0", "width": 640, "height": 480, "fps": 30 } } }
  ],
  "cameras": { "top": { "index_or_path": "/dev/v4l/by-id/usb-...-video-index0", "width": 640, "height": 480, "fps": 30 } }
}
```

- **`mode`** 가 1차 기준입니다. `single` 이면 `arms` 1개(`side: "main"`), `bimanual` 이면 2개(`left`/`right`).
  안 맞으면 로드 시 `mode` 에 맞춰 잘라내거나 빈 팔을 채웁니다
- 양팔에서 팔 카메라 키는 `left_wrist` / `right_wrist` 로 접두사가 붙고, 최상위 `cameras`(top)는
  접두사 없이 그대로 — lerobot `bi_so_follower` 규칙과 동일합니다
- `follower_id` / `leader_id` → 캘리브레이션 파일 이름.
  팔로워 `$HF_HOME/lerobot/calibration/robots/so_follower/<follower_id>.json`,
  리더 `.../teleoperators/so_leader/<leader_id>.json`. 양팔은 같은 디렉터리에 `_left` / `_right` 로
- `max_relative_target` — lerobot 쪽 상대이동 캡(도). 켜면 `send_action` 마다
  `Present_Position` 을 한 번 더 읽어 제어 루프가 느려집니다. 기본 `null`
- 양팔 모드: 수집은 `bi_so_follower` / `bi_so_leader` 로, 추론은 같은 타입의 CLI 인자로 돕니다.
  **calib id 는 `X_left` / `X_right` 형식**이어야 합니다 — lerobot `BiSOFollower` 가 per-arm
  캘리브레이션 파일을 `{id}_left.json` / `{id}_right.json` 으로 찾기 때문입니다 (Setup 탭이 검증)
- 데이터셋 `meta/info.json` 의 `robot_type` (`so_follower` / `bi_so_follower`) 이 현재 모드와 다르면
  이어서 수집·추론이 막히고 목록에 **모드 불일치** 배지가 뜹니다. 학습은 모드와 무관합니다

### 포트 지정 — 왜 `by-path` 인가

같은 컨트롤러 보드 2개(양팔이면 4개)는 전기적으로 구분이 안 됩니다. 재부팅하면 `/dev/ttyACM0`, `1` 순서도 바뀝니다.

- `/dev/serial/by-id/…` 는 USB 시리얼 번호 기반 — 보드가 시리얼 번호를 안 내보내면(Setup 탭에 **sn 없음**)
  같은 모델끼리 이름이 겹쳐서 못 씁니다
- `/dev/serial/by-path/…` 는 **꽂은 USB 물리 포트** 기반 — 항상 유일합니다. 대신 **팔을 항상 같은 USB 구멍에 꽂아야** 합니다

Setup 탭은 시리얼 번호가 있으면 `by-id`, 없으면 `by-path` 를 자동으로 고릅니다. 카메라도 같은 규칙입니다.

**실측 (Seeed SO-ARM101 Pro Assembled Kit, 2026-09)**: 동봉된 Servo Driver Board 는
`1a86:55d3` (WCH CH343/CH9102 계열) 이고 **보드마다 고유 시리얼 번호가 있습니다**
(`sn=5B90102742` 등). 따라서 `by-id` 가 잡히고 **USB 구멍을 가릴 필요가 없습니다.**
대신 경로가 보드를 따라가므로, 보드에 sn 뒷자리를 적어 붙여 두고 다른 팔로 옮겨 달지 마세요.
같은 키트의 모터는 **ID 1~6 이 이미 들어 있어** 모터 ID 세팅 단계를 건너뛸 수 있었습니다
(probe 에서 `1,2,3,4,5,6` 확인).

**leader / follower 판별**: Setup 탭에서 *포트 감시* 를 켜면 모든 후보 포트를 토크 OFF 로 열고
엔코더를 읽습니다. 팔 하나를 손으로 움직이면 그 포트의 `travel` 값이 올라갑니다 → 그 줄의 역할 선택.
⚠️ 토크가 꺼지므로 팔로워가 들려 있으면 주저앉습니다. 받치거나 내려놓고 시작하세요.

## 새 팔을 붙일 때

1. **Setup → probe**: `1,2,3,4,5,6` 이 다 뜨면 ID 세팅 완료. `1` 하나만 뜨거나 응답 없음이면 ↓
2. **Setup → 모터 ID 세팅**: 모터를 **한 개씩만** 보드에 연결하고 안내 순서대로 `ID 쓰기`
   (새 STS3215 는 전부 ID 1 이라 여러 개를 같이 붙이면 응답이 충돌합니다).
   이미 ID 를 쓴 모터가 아직 붙어 있으면 거부합니다
3. **Setup → 포트 감시·역할 지정 → 저장**
4. **Calib** — 이전 팔의 캘리브레이션은 그 팔의 엔코더 값이라 재사용 불가. 같은 id 로 저장하면 덮어씁니다

## 캘리브레이션 — Calib 탭

`lerobot-calibrate` 가 하는 일을 그대로, 터미널 `input()` 대기만 웹 버튼으로 바꾼 것입니다.

| 단계 | 화면 | 내부 (lerobot 과 동일) |
|---|---|---|
| 연결 | 팔 선택 → 시작 | `connect(calibrate=False)` → `disable_torque()` → `Operating_Mode=POSITION` |
| 중앙 자세 | 모든 관절을 가동 범위 중앙에 → **중앙 자세 기록** | `bus.set_half_turn_homings()` |
| 범위 기록 | wrist_roll 빼고 관절마다 양 끝까지 → 막대가 초록이면 충분 | `Present_Position` (raw) 폴링, min/max 누적 |
| 저장 | **완료·저장** | `bus.write_calibration()` + `_save_calibration()` → `<id>.json` |

- 안 움직인 관절(min == max)이 있으면 저장이 막힙니다 (lerobot 도 여기서 `ValueError`)
- 30° 미만으로만 움직인 관절은 경고만 하고 저장은 허용합니다
- `wrist_roll` 은 0~4095 고정 (전체 회전)
- **취소**하면 중앙 자세 기록으로 이미 바뀐 모터 EEPROM 을 이전 캘리브레이션 값으로 되돌립니다
- ⚠️ 시작하면 토크가 꺼집니다. 팔로워는 손으로 받치세요
- 기록되는 min/max 가 그대로 관절 한계가 됩니다 — 기계적 스톱에 **닿기 직전**까지만 움직이세요

## 수집 worker — 왜 CLI 를 안 쓰나

`lerobot-record` 는 터미널 키보드(n/r/q)로 조작합니다. 예전엔 PTY 로 키를 밀어넣었는데,
X11 세션이면 lerobot 이 pynput 전역 리스너를 골라 PTY 입력이 조용히 버려지는 문제가 있었습니다.

지금은 `python lrweb.py --worker record <jid>` 로 **이 파일 자체를 worker 로 띄워** lerobot 의
`record_loop()` 를 직접 부릅니다. 웹 버튼이 `events` 딕트(`exit_early` / `rerecord_episode` /
`stop_recording`)를 그대로 건드립니다. 데이터셋 생성·`VideoEncodingManager`·`save_episode()` 흐름은
`lerobot_record.record()` 와 동일하고, 인코더/이미지라이터 기본값도 `DatasetRecordConfig` 에서 가져옵니다.

worker ↔ 웹은 전부 파일입니다 (`/dev/shm/lrweb/<jid>/`, 없으면 `~/project/lerobot/lrweb_run/`):

| 파일 | 방향 | 내용 |
|---|---|---|
| `status.json` | worker → 웹 | phase(record/reset/saving…), 에피소드, 경과, 오류 |
| `cam_<name>.jpg` | worker → 웹 | 최신 프레임 (10 fps, 원자적 교체) → `/stream/<name>` 으로 MJPEG |
| `cmd` | 웹 → worker | n / r / q 문자 append |

그래서 lrweb 를 재시작해도 진행 중인 수집을 다시 붙잡을 수 있고, Jobs 탭 '중지'(SIGINT)는 q 와 같습니다.
데이터셋 이름은 입력한 그대로 씁니다 (CLI 처럼 타임스탬프를 붙이지 않음) — 같은 이름이 있으면 거부.

## 접속

기본은 인증 없음입니다. 그냥 `http://<host>:8080/` 으로 들어가면 됩니다.

필요하면 토큰 인증을 켤 수 있습니다 (시작 로그에 찍히는 URL로 1회 접속하면
쿠키가 저장되어 이후에는 주소만으로 들어갑니다):

```bash
LRWEB_TOKEN=원하는값 python lrweb.py   # 토큰 직접 지정
LRWEB_AUTH=on        python lrweb.py   # lrweb_token.txt 에 자동 생성
```

## 환경

- lerobot commit: `e40b58a8dfa9e7b86918c374791599d070518d11`
- Python 3.12 (miniforge), torch 2.9 cu130
  - Jetson Thor(aarch64): `pip install torch --index-url https://pypi.jetson-ai-lab.io/sbsa/cu130`
    + `nvidia-jetpack-dev`, NVPL, cuDSS
  - x86(RTX 5090 등): `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130`
- 공통: `pip install -e "lerobot-src[feetech,training]"` 후
  `pip uninstall -y torchcodec && pip install "av>=15.0.0,<16.0.0"` (pyav 디코딩으로 통일)

## 실행

```bash
source activate.sh
pip install fastapi uvicorn
nohup python lrweb.py > lrweb.log 2>&1 &
```

## 로드맵

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | 버그 픽스 + lerobot 객체(`SOFollower`/`SOLeader`) 전환 + 설정 파일 | ✅ |
| 2 | Setup 탭 — USB 포트 스캔·probe·leader/follower 판별·카메라·모드 전환을 웹에서 | ✅ |
| 3 | Calibration 탭 — 웹에서 캘리브레이션 (`lerobot-calibrate` 불필요) | ✅ |
| 4 | Control 탭 팔별 스레드 분리 | ✅ |
| 5 | record worker 프로세스 — PTY 제거, `record_loop()` 직접 호출, 수집 중 카메라 미리보기 | ✅ |
| 6 | 양팔 (`bi_so_follower` / `bi_so_leader`) | ✅ |

### 1단계에서 고친 것

- **관절 한계 계산이 lerobot 정규화와 달랐음.** `(raw-2048)/4096*360` 을 쓰고 있었는데
  lerobot `MotorsBus._normalize(DEGREES)` 는 `(raw-mid)*360/(resolution-1)`,
  `mid=(range_min+range_max)/2` 입니다. 캘리브레이션 범위가 2048 중심이 아니면
  (보통 아닙니다) 슬라이더 전 범위가 어긋납니다 — 예: range 1000~3500 이면 17.8° 편차
- **`configure()` 누락.** `FeetechMotorsBus` 를 직접 열면 `Operating_Mode`, `P_Coefficient`,
  그리고 gripper 의 `Max_Torque_Limit` / `Protection_Current` / `Overload_Torque` 가
  전부 안 잡힙니다 (그리퍼가 풀 토크로 물고 버팀). `SOFollower` 를 쓰면 `connect()` 가 처리
- **셸 인젝션.** `/api/delete/{ds}` 가 검증 없는 이름을 `shell=True` 명령 문자열에
  넣고 있었습니다. 모든 외부 프로세스를 argv 리스트 + `shell=False` 로 전환
- **HTML 이스케이프.** 데이터셋 이름이 HTML·JS 에 그대로 들어가고 있었습니다
  (옵션으로 토큰 인증도 추가 — 기본은 꺼짐)
- **PTY 키가 조용히 무시되는 경우.** lerobot `init_keyboard_listener()` 는 X11 세션이면
  pynput 전역 리스너를 씁니다. 그러면 PTY 로 넣는 n/r/q 가 아무 데도 안 갑니다.
  자식 프로세스 환경에서 `DISPLAY`/`WAYLAND_DISPLAY` 를 제거해 터미널 리스너로 고정
- **카메라 해상도.** raw `cv2.VideoCapture` 라 기본 해상도로 열렸습니다.
  `OpenCVCamera` 로 바꿔 record 설정과 동일한 해상도/fps 강제
- train 로그 step 파싱이 `1M` 같은 접미사에서 탈락하던 것, PTY/로그 fd 누수

## License

- 코드(`lrweb.py` 등): MIT — [LICENSE](LICENSE)
- `urdf/` 의 URDF/STL: [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
  (Apache 2.0) 기반, 경로 평탄화 수정 — [urdf/LICENSE.md](urdf/LICENSE.md)
