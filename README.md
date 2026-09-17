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
| `lrweb_token.txt` | 접속 토큰 (권한 600) |
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
- record/rollout/train/control 자원 기반 상호 배타 (학습+수동제어는 동시 허용)

## 설정 — `lrweb_config.json`

첫 실행 때 기본값으로 생성됩니다. `arms` 는 **처음부터 리스트**입니다 —
한팔이면 원소 1개(`side: "main"`), 양팔이면 `"left"`/`"right"` 2개로 늘어납니다.

```json
{
  "mode": "single",
  "robot_id": "so101",
  "fps": 30,
  "default_task": "Pick up the block and place it in the box",
  "max_relative_target": null,
  "arms": [
    {
      "side": "main",
      "follower_port": "/dev/so101_follower",
      "follower_id": "follower",
      "leader_port": "/dev/so101_leader",
      "leader_id": "leader",
      "cameras": { "wrist": { "index_or_path": 0, "width": 640, "height": 480, "fps": 30 } }
    }
  ],
  "cameras": { "top": { "index_or_path": 2, "width": 640, "height": 480, "fps": 30 } }
}
```

- `follower_id` / `leader_id` → 캘리브레이션 파일 이름입니다.
  팔로워는 `$HF_HOME/lerobot/calibration/robots/so_follower/<follower_id>.json`,
  리더는 `.../teleoperators/so_leader/<leader_id>.json`
- `cameras`(최상위) = 특정 팔에 속하지 않는 카메라. 한팔에서는 팔 카메라와 그냥 합쳐집니다
- `max_relative_target` — lerobot 쪽 상대이동 캡(도). 켜면 `send_action` 마다
  `Present_Position` 을 한 번 더 읽으므로 제어 루프가 느려집니다. 기본 `null`
  (Control 탭의 자체 속도 제한 적분기가 담당)
- `arms` 를 2개로 늘리면 아직 `NotImplementedError` 입니다 (로드맵 6단계)

## 접속 / 인증

기본으로 토큰 인증이 켜져 있습니다. 시작 로그에 찍히는 URL로 최초 1회 접속하면
쿠키가 저장되어 이후에는 그냥 `http://<host>:8080/` 으로 들어갑니다.

```
open : http://<host>:8080/?token=xxxxxxxx   (token: /home/<user>/project/lerobot/lrweb_token.txt)
```

- 토큰 고정: `LRWEB_TOKEN=...`
- 인증 해제: `LRWEB_AUTH=off` — **신뢰된 네트워크에서만**. 이 웹툴은 데이터셋 삭제와
  로봇 구동 권한을 그대로 노출합니다

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
head -5 lrweb.log        # 접속 URL(토큰 포함) 확인
```

## 로드맵

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | 버그 픽스 + lerobot 객체(`SOFollower`/`SOLeader`) 전환 + 설정 파일 | ✅ |
| 2 | Setup 탭 — USB 포트 스캔·probe·leader/follower 판별을 웹에서 | |
| 3 | Calibration 탭 — 웹에서 캘리브레이션 (`lerobot-calibrate` 불필요) | |
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
- **HTML 이스케이프 / 무인증.** 데이터셋 이름이 HTML·JS 에 그대로 들어가고 있었고,
  `0.0.0.0:8080` 이 무인증이었습니다
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
