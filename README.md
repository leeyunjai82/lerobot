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

실행하면 `~/project/lerobot/` 아래에 자동 생성되는 것:

| 파일 | 설명 |
|---|---|
| `lrweb_config.json` | 포트·카메라·fps 설정 (아래 참고) |
| `lrweb_jobs/` | 백그라운드 작업 기록·로그 |
| `lrweb_marks.json` | 불량 에피소드 마킹 |

## lrweb 기능

- **Datasets**: 데이터셋·에피소드 목록, 에피소드 단위 영상 재생(청크 mp4 구간 시킹),
  불량 마킹 → 일괄 삭제, 데이터셋 통삭제
- **Collect**: 웹에서 `lerobot-record` 실행·조작 — n(다음)/r(재녹화)/q(종료) 버튼 (PTY 키 주입)
- **Training**: ACT 학습 시작/중지, loss 차트, 로그 tail
- **Rollout**: 체크포인트 자동 스캔 → 자율 구동 시작/중지
- **Control**: 팔로워 수동 제어 — 슬라이더(속도 제한), 토크/E-STOP,
  리더 팔로우, 카메라 MJPEG 스트리밍, Three.js URDF 3D. 탭 이탈 시 자동 해제
- **Setup**: USB 시리얼 포트 스캔·probe(모터 ID 확인)·**포트 감시로 leader/follower 판별**,
  카메라 스캔·등록, 한팔/양팔 모드 전환, 캘리브레이션 파일 상태 — 전부 웹에서
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
- 양팔 모드에서 수집·학습·추론은 아직 막혀 있습니다 (로드맵 6단계). 설정과 Control 탭 표시는 됩니다

### 포트 지정 — 왜 `by-path` 인가

같은 컨트롤러 보드 2개(양팔이면 4개)는 전기적으로 구분이 안 됩니다. 재부팅하면 `/dev/ttyACM0`, `1` 순서도 바뀝니다.

- `/dev/serial/by-id/…` 는 USB 시리얼 번호 기반 — 보드가 시리얼 번호를 안 내보내면(Setup 탭에 **sn 없음**)
  같은 모델끼리 이름이 겹쳐서 못 씁니다
- `/dev/serial/by-path/…` 는 **꽂은 USB 물리 포트** 기반 — 항상 유일합니다. 대신 **팔을 항상 같은 USB 구멍에 꽂아야** 합니다

Setup 탭은 시리얼 번호가 있으면 `by-id`, 없으면 `by-path` 를 자동으로 고릅니다.
카메라도 같은 이유로 `/dev/v4l/by-id/…` 를 우선합니다.

**leader / follower 판별**: Setup 탭에서 *포트 감시* 를 켜면 모든 후보 포트를 토크 OFF 로 열고
엔코더를 읽습니다. 팔 하나를 손으로 움직이면 그 포트의 `travel` 값이 올라갑니다 → 그 줄의 역할 선택.
⚠️ 토크가 꺼지므로 팔로워가 들려 있으면 주저앉습니다. 받치거나 내려놓고 시작하세요.

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
| 4 | Control 탭 팔별 스레드 분리 | |
| 5 | record worker 프로세스 — PTY 제거, `record_loop()` 직접 호출, 수집 중 카메라 미리보기 | |
| 6 | 양팔 (`bi_so_follower` / `bi_so_leader`) | |

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
