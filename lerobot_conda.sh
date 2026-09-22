#!/usr/bin/env bash
# ============================================================================
#  LeRobot + lrweb 환경 셋업 (conda, docker 미사용)
#  Jetson Thor(aarch64 / CUDA 13) 와 x86_64(CUDA 13) 를 자동 분기합니다.
#
#  새 기기에서:
#     mkdir -p ~/project && cd ~/project
#     git clone https://github.com/leeyunjai82/lerobot.git lerobot
#     cd lerobot
#     chmod +x lerobot_conda.sh
#     sudo -v
#     nohup ./lerobot_conda.sh > /dev/null 2>&1 &
#     tail -f lerobot_conda.log
#
#  끝나면:
#     source ~/project/lerobot/activate.sh
#     cd ~/project/lerobot && nohup python lrweb.py > lrweb.log 2>&1 &
#     → http://<ip>:8080/setup 에서 포트·카메라 지정, /calib 에서 캘리브레이션
#
#  핵심 주의사항 (Thor):
#   1) PyTorch 를 pip 기본 인덱스에서 받으면 CUDA 를 못 잡습니다.
#      aarch64-sbsa / CUDA 13 전용 휠 인덱스를 써야 합니다.
#   2) 그 휠이 cp312 라서 conda 파이썬도 3.12 여야 합니다.
# ============================================================================
set -Eeuo pipefail

# ------------------------------- 설정 ---------------------------------------
WORKDIR="${HOME}/project/lerobot"          # 이 레포 (lrweb.py 가 있는 곳)
LOGFILE="${WORKDIR}/lerobot_conda.log"
CONDA_DIR="${HOME}/miniforge3"
ENV_NAME="lerobot"
PY_VER="3.12"                              # Thor 휠이 cp312. 바꾸지 말 것
LEROBOT_SRC="${WORKDIR}/lerobot-src"
LEROBOT_COMMIT="e40b58a8dfa9e7b86918c374791599d070518d11"   # README 와 동일. lrweb 가 이 API 에 맞춰져 있음
DATA_DIR="${WORKDIR}/data"

ARCH="$(uname -m)"
case "${ARCH}" in
  aarch64)
    MINIFORGE="Miniforge3-Linux-aarch64.sh"
    TORCH_INDEXES=(
      "https://pypi.jetson-ai-lab.io/sbsa/cu130"
      "https://pypi.jetson-ai-lab.io/sbsa/cu129"
      "https://pypi.jetson-ai-lab.dev/sbsa/cu130"
    )
    TORCH_PKGS="torch torchvision torchaudio"
    ;;
  x86_64)
    MINIFORGE="Miniforge3-Linux-x86_64.sh"
    TORCH_INDEXES=("https://download.pytorch.org/whl/cu130")
    TORCH_PKGS="torch torchvision"
    ;;
  *) echo "지원하지 않는 아키텍처: ${ARCH}"; exit 1 ;;
esac
# ---------------------------------------------------------------------------

mkdir -p "${WORKDIR}" "${DATA_DIR}"
exec > >(tee -a "${LOGFILE}") 2>&1

log()  { echo -e "\n[$(date '+%F %T')] === $* ==="; }
warn() { echo "[$(date '+%F %T')] !! $*"; }
die()  { echo "[$(date '+%F %T')] XX 치명적 실패: $*"; exit 1; }
trap 'warn "line ${LINENO} 오류. 로그: ${LOGFILE}"' ERR

log "시작. arch=${ARCH} 작업경로=${WORKDIR}"
[[ -f "${WORKDIR}/lrweb.py" ]] || die "${WORKDIR}/lrweb.py 가 없습니다. 이 레포를 ~/project/lerobot 에 clone 한 뒤 실행하세요"

# ------------------------------------------------------------- 0. 시스템 의존성
log "0. 시스템 패키지"
sudo apt-get update -qq
sudo apt-get install -y \
  git cmake build-essential pkg-config \
  ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
  libgl1 libglib2.0-0 libusb-1.0-0-dev v4l-utils \
  python3-pip curl

# 시리얼(모터 보드)·카메라 권한. udev 심볼릭 링크는 만들지 않습니다 — 포트는 웹 Setup 탭에서 지정합니다.
sudo usermod -aG dialout,video "${USER}" || true
sudo tee /etc/udev/rules.d/99-so101.rules >/dev/null <<'RULES'
# USB-serial 보드 권한 (WCH CH34x / Silicon Labs CP210x / FTDI). 어느 칩인지는 lsusb 로 확인.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"
RULES
sudo udevadm control --reload-rules && sudo udevadm trigger || true

# --------------------------------------------------------------- 1. miniforge
log "1. miniforge 설치"
if [[ ! -d "${CONDA_DIR}" ]]; then
  curl -fsSL -o /tmp/miniforge.sh \
    "https://github.com/conda-forge/miniforge/releases/latest/download/${MINIFORGE}"
  bash /tmp/miniforge.sh -b -p "${CONDA_DIR}"
else
  echo "이미 설치됨: ${CONDA_DIR}"
fi

# shellcheck disable=SC1091
source "${CONDA_DIR}/etc/profile.d/conda.sh"
conda config --set always_yes true --set changeps1 false || true

# ---------------------------------------------------------------- 2. conda env
log "2. conda 환경 생성 (python ${PY_VER})"
if conda env list | grep -qE "^${ENV_NAME}\s"; then
  echo "환경 이미 존재: ${ENV_NAME}"
else
  conda create -n "${ENV_NAME}" "python=${PY_VER}" -y
fi
conda activate "${ENV_NAME}"
python -V
python -m pip install --upgrade pip setuptools wheel

# ------------------------------------------------------------------ 3. PyTorch
log "3. PyTorch 설치 (${ARCH}, CUDA 13 휠)"
TORCH_OK=0
for idx in "${TORCH_INDEXES[@]}"; do
  echo "--- 인덱스 시도: ${idx}"
  # shellcheck disable=SC2086
  if pip install --index-url "${idx}" ${TORCH_PKGS}; then
    TORCH_OK=1
    echo "--- 성공: ${idx}"
    break
  fi
  warn "실패: ${idx}"
done
(( TORCH_OK == 0 )) && die "PyTorch 설치 실패. 인덱스 경로 확인 후 TORCH_INDEXES 수정"

check_cuda() {
python - <<'PY'
import torch, sys
print("torch      :", torch.__version__)
print("cuda avail :", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device     :", torch.cuda.get_device_name(0))
else:
    print("!! CUDA 미인식 — 잘못된 휠입니다.")
    sys.exit(1)
PY
}
log "3-1. CUDA 인식 확인"
check_cuda || die "CUDA 미인식. 여기서 멈춥니다"

# ------------------------------------------------------------------ 4. LeRobot
log "4. LeRobot 소스 (commit ${LEROBOT_COMMIT:0:8})"
if [[ ! -d "${LEROBOT_SRC}/.git" ]]; then
  git clone https://github.com/huggingface/lerobot.git "${LEROBOT_SRC}"
fi
git -C "${LEROBOT_SRC}" fetch --all --tags || warn "fetch 실패, 로컬 트리 사용"
git -C "${LEROBOT_SRC}" checkout -q "${LEROBOT_COMMIT}" \
  || die "lerobot commit ${LEROBOT_COMMIT} 체크아웃 실패"
cd "${LEROBOT_SRC}"

log "4-1. lerobot[feetech,training] 설치 (torch 는 위 휠 유지)"
# torch 를 pip 기본 인덱스 것으로 덮어쓰지 않도록 현재 버전으로 핀
python - <<'PY' > /tmp/torch-constraint.txt
import torch, torchvision
print(f"torch=={torch.__version__}")
print(f"torchvision=={torchvision.__version__}")
PY
cat /tmp/torch-constraint.txt
PIP_CONSTRAINT=/tmp/torch-constraint.txt pip install -e ".[feetech,training]" \
  || die "lerobot 설치 실패 (로그 확인)"

# torchcodec 은 Jetson 에서 문제를 일으켜 pyav 디코딩으로 통일합니다 (README 와 동일)
pip uninstall -y torchcodec || true
pip install "av>=15.0.0,<16.0.0"

log "4-2. lrweb 의존성"
pip install "fastapi<1.0" uvicorn

log "4-3. torch 가 덮어써지지 않았는지 재확인"
check_cuda || die "lerobot 설치 과정에서 torch 가 CPU 휠로 바뀌었습니다. pip uninstall -y torch torchvision 후 3단계 인덱스로 재설치"

# ----------------------------------------------------------------- 5. 임포트 검증
log "5. 최종 검증"
python - <<'PY'
import torch
print("torch   :", torch.__version__, "| cuda:", torch.cuda.is_available())
import lerobot
from lerobot.robots.so_follower import SOFollower
from lerobot.robots.bi_so_follower import BiSOFollower
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.scripts.lerobot_record import record_loop
import fastapi, uvicorn, cv2, av
print("lerobot :", "OK (so_follower / bi_so_follower / feetech / record_loop)")
print("fastapi :", fastapi.__version__, "| cv2:", cv2.__version__, "| av:", av.__version__)
PY

# ------------------------------------------------------------ 6. 활성화 헬퍼
log "6. activate.sh"
# 경로를 $HOME 기준으로 남깁니다 — 절대경로를 박으면 기기마다 파일이 달라져
# git 에서 매번 diff 가 뜨고, 다른 기기에서는 경로가 틀립니다.
h() { printf '%s' "${1/#${HOME}/\$HOME}"; }
cat > "${WORKDIR}/activate.sh" <<EOF
#!/usr/bin/env bash
source "$(h "${CONDA_DIR}")/etc/profile.d/conda.sh"
conda activate ${ENV_NAME}
export HF_HOME="$(h "${DATA_DIR}")/hf"
cd "$(h "${LEROBOT_SRC}")"
EOF
chmod +x "${WORKDIR}/activate.sh"

# ------------------------------------------------------------------- 완료
log "완료"
cat <<EOF

  conda env : ${ENV_NAME}  (python ${PY_VER})
  lerobot   : ${LEROBOT_SRC} @ ${LEROBOT_COMMIT:0:8}
  데이터    : ${DATA_DIR}   (HF_HOME=${DATA_DIR}/hf → 캘리브레이션은 \$HF_HOME/lerobot/calibration)
  활성화    : source ${WORKDIR}/activate.sh

  --- lrweb 실행 ---
  source ${WORKDIR}/activate.sh
  cd ${WORKDIR} && nohup python lrweb.py > lrweb.log 2>&1 &
  → http://<ip>:8080

  --- 웹에서 순서대로 ---
  Setup   : 한팔/양팔 → 포트 스캔 → 포트 감시로 팔 판별 → 카메라 스캔·추가 → 저장
  Calib   : 팔로워·리더 각각 (양팔이면 4개)
  Control : 슬라이더 범위 확인
  Collect : 수집

  * dialout / video 그룹 반영을 위해 재로그인(또는 재부팅) 한 번 필요합니다.
  * 학습 시 GPU 를 쓰는 다른 서비스(vLLM 등)는 내리세요.

EOF
