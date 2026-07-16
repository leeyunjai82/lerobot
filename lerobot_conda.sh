#!/usr/bin/env bash
# ============================================================================
#  Jetson AGX Thor : LeRobot 환경 (conda, docker 미사용)
#
#  위치: /home/circulus/project/lerobot/lerobot_conda.sh
#
#  핵심 주의사항 2가지:
#   1) PyTorch를 pip 기본 인덱스에서 받으면 CUDA를 못 잡습니다.
#      반드시 aarch64-sbsa / CUDA 13 전용 휠 인덱스를 써야 합니다.
#   2) 그 휠이 cp312로 빌드되어 있어서 conda 파이썬도 3.12여야 합니다.
#
#  실행:
#     mkdir -p ~/project/lerobot && cd ~/project/lerobot
#     chmod +x lerobot_conda.sh
#     sudo -v
#     nohup ./lerobot_conda.sh > /dev/null 2>&1 &
#     tail -f lerobot_conda.log
# ============================================================================
set -Eeuo pipefail

# ------------------------------- 설정 ---------------------------------------
PROJECT_ROOT="/home/circulus/project"
WORKDIR="${PROJECT_ROOT}/lerobot"
LOGFILE="${WORKDIR}/lerobot_conda.log"
CONDA_DIR="${HOME}/miniforge3"
ENV_NAME="lerobot"
PY_VER="3.12"                       # Thor 휠이 cp312. 바꾸지 말 것
LEROBOT_SRC="${WORKDIR}/lerobot-src"
DATA_DIR="${WORKDIR}/data"

# Thor(sbsa/CUDA13)용 PyTorch 휠 인덱스 후보
TORCH_INDEXES=(
  "https://pypi.jetson-ai-lab.io/sbsa/cu130"
  "https://pypi.jetson-ai-lab.io/sbsa/cu129"
  "https://pypi.jetson-ai-lab.dev/sbsa/cu130"
)
# ---------------------------------------------------------------------------

mkdir -p "${WORKDIR}" "${DATA_DIR}"
exec > >(tee -a "${LOGFILE}") 2>&1

log()  { echo -e "\n[$(date '+%F %T')] === $* ==="; }
warn() { echo "[$(date '+%F %T')] !! $*"; }
die()  { echo "[$(date '+%F %T')] XX 치명적 실패: $*"; exit 1; }
trap 'warn "line ${LINENO} 오류. 로그: ${LOGFILE}"' ERR

log "시작. 작업경로=${WORKDIR}"

# ------------------------------------------------------------- 0. 시스템 의존성
log "0. 시스템 패키지"
sudo apt-get update -qq
sudo apt-get install -y \
  git cmake build-essential pkg-config \
  ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
  libgl1 libglib2.0-0 libusb-1.0-0-dev \
  python3-pip curl

# 시리얼 포트 권한 (SO-101 리더/팔로워)
sudo usermod -aG dialout "${USER}" || true
sudo tee /etc/udev/rules.d/99-so101.rules >/dev/null <<'RULES'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"
RULES
sudo udevadm control --reload-rules && sudo udevadm trigger || true

# --------------------------------------------------------------- 1. miniforge
log "1. miniforge 설치"
if [[ ! -d "${CONDA_DIR}" ]]; then
  curl -fsSL -o /tmp/miniforge.sh \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
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

# -------------------------------------------------------- 3. PyTorch (Thor 전용)
log "3. PyTorch 설치 (aarch64-sbsa / CUDA 13 전용 휠)"
TORCH_OK=0
for idx in "${TORCH_INDEXES[@]}"; do
  echo "--- 인덱스 시도: ${idx}"
  if pip install --index-url "${idx}" torch torchvision torchaudio; then
    TORCH_OK=1
    echo "--- 성공: ${idx}"
    break
  fi
  warn "실패: ${idx}"
done
(( TORCH_OK == 0 )) && die "PyTorch 설치 실패. https://pypi.jetson-ai-lab.io 에서 sbsa/cuXXX 경로 확인 후 TORCH_INDEXES 수정"

log "3-1. CUDA 인식 확인"
python - <<'PY'
import torch, sys
print("torch      :", torch.__version__)
print("cuda avail :", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device     :", torch.cuda.get_device_name(0))
else:
    print("!! CUDA 미인식 — 잘못된 휠입니다. 여기서 멈추고 인덱스 다시 확인하세요.")
    sys.exit(1)
PY

# ------------------------------------------------------------------ 4. LeRobot
log "4. LeRobot 설치 (소스, feetech 서보 지원 포함)"
if [[ ! -d "${LEROBOT_SRC}" ]]; then
  git clone https://github.com/huggingface/lerobot.git "${LEROBOT_SRC}"
else
  git -C "${LEROBOT_SRC}" pull --ff-only || warn "pull 실패, 기존 트리 사용"
fi
cd "${LEROBOT_SRC}"

# 중요: torch를 재설치해서 덮어쓰지 않도록 방지
cat > /tmp/no-torch-constraint.txt <<'CON'
torch
torchvision
torchaudio
CON
pip install -e ".[feetech]" --no-deps
pip install -e ".[feetech]" 2>/dev/null || {
  warn "전체 의존성 설치 중 일부 실패. 개별 설치 재시도"
  pip install \
    "datasets" "huggingface_hub" "opencv-python" "imageio[ffmpeg]" \
    "av" "einops" "gymnasium" "hydra-core" "termcolor" "wandb" \
    "deepdiff" "draccus" "jsonlines" "packaging" "rerun-sdk" \
    "feetech-servo-sdk" || warn "일부 패키지 실패 (로그 확인)"
}

log "4-1. torch가 덮어써지지 않았는지 재확인"
python - <<'PY'
import torch
print("torch      :", torch.__version__)
print("cuda avail :", torch.cuda.is_available())
if not torch.cuda.is_available():
    print("!! lerobot 설치 과정에서 torch가 CPU 휠로 덮어써졌습니다.")
    print("   조치: pip uninstall -y torch torchvision torchaudio 후 3단계 인덱스로 재설치")
PY

# ----------------------------------------------------------------- 5. 임포트 검증
log "5. 최종 검증"
python - <<'PY'
import torch
print("torch :", torch.__version__, "| cuda:", torch.cuda.is_available())
try:
    import lerobot
    print("lerobot: OK")
except Exception as e:
    print("lerobot import 실패:", e)
PY

# ------------------------------------------------------------ 6. 활성화 헬퍼
log "6. 헬퍼 생성"
cat > "${WORKDIR}/activate.sh" <<EOF
#!/usr/bin/env bash
# 사용: source ~/project/lerobot/activate.sh
source ${CONDA_DIR}/etc/profile.d/conda.sh
conda activate ${ENV_NAME}
export HF_HOME=${DATA_DIR}/hf
cd ${LEROBOT_SRC}
echo "lerobot 환경 활성화됨 (python \$(python -V 2>&1))"
EOF
chmod +x "${WORKDIR}/activate.sh"

# ------------------------------------------------------------------- 완료
log "완료"
cat <<EOF

  conda env : ${ENV_NAME}  (python ${PY_VER})
  소스      : ${LEROBOT_SRC}
  데이터    : ${DATA_DIR}
  활성화    : source ${WORKDIR}/activate.sh

  --- 팔 연결 후 ---
  source ${WORKDIR}/activate.sh
  lerobot-find-port
  lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=follower
  lerobot-calibrate --teleop.type=so101_leader  --teleop.port=/dev/ttyACM1 --teleop.id=leader

  * dialout 그룹 반영을 위해 재로그인 한 번 필요할 수 있습니다.
  * 학습 시에는 사내 LLM을 내리세요: docker stop vllm-server

EOF
