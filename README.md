# lerobot-tools — SO-101 / Jetson Thor pipeline

HuggingFace [lerobot](https://github.com/huggingface/lerobot) 기반 SO-101 로봇팔
수집·학습·추론 파이프라인용 웹 툴. (본 repo는 HF lerobot 자체가 아니며, 운용 도구 모음입니다)

## 구성

| 파일 | 설명 |
|---|---|
| `lrweb.py` | 통합 웹툴 — Datasets / Collect / Training / Rollout / Control (port 8080) |
| `activate.sh` | conda env 활성화 + `HF_HOME` 설정 |
| `urdf/` | SO-101 URDF/STL (Control 탭 3D 뷰용) |

## lrweb 기능

- **Datasets**: 데이터셋·에피소드 목록, 에피소드 단위 영상 재생(청크 mp4 구간 시킹),
  불량 마킹 → 일괄 삭제, 데이터셋 통삭제
- **Collect**: 웹에서 `lerobot-record` 실행·조작 — n(다음)/r(재녹화)/q(종료) 버튼 (PTY 키 주입)
- **Training**: ACT 학습 시작/중지, loss 차트, 로그 tail
- **Rollout**: 체크포인트 자동 스캔 → 자율 구동 시작/중지
- **Control**: 팔로워 수동 제어 — 슬라이더(속도 제한), 토크/E-STOP/복귀,
  카메라 MJPEG 스트리밍, Three.js URDF 3D. 탭 이탈 시 자동 해제
- record/rollout/train/control 자원 기반 상호 배타 (학습+수동제어는 동시 허용)

## 환경

- lerobot commit: `<git -C lerobot-src rev-parse HEAD 값>`
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
# http://<host>:8080
```

## License

- 코드(`lrweb.py` 등): MIT — [LICENSE](LICENSE)
- `urdf/` 의 URDF/STL: [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
  (Apache 2.0) 기반, 경로 평탄화 수정 — [urdf/LICENSE.md](urdf/LICENSE.md)
