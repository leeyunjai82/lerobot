#!/usr/bin/env bash
# ============================================================================
#  LeRobot + arm-lab 환경 셋업 (conda, docker 미사용)
#  세 가지 플랫폼을 자동 분기합니다.
#    thor  : Jetson Thor (aarch64 / CUDA 13)
#    cuda  : x86_64 + NVIDIA GPU (CUDA 13)
#    intel : x86_64 + NVIDIA 없음 (Intel Core Ultra — Meteor Lake 이상 권장)
#            → PyTorch CPU 휠 + OpenVINO/NNCF. 추론은 NPU/GPU/CPU 를 OpenVINO 로 씁니다.
#            학습은 이 경로에서 CPU 로만 돌아 매우 느립니다 — 학습은 CUDA 기기에서 하세요.
#  강제 지정: ARMLAB_PLATFORM=intel ./lerobot_conda.sh   (thor | cuda | intel)
#
#  새 기기에서:
#     mkdir -p ~/project && cd ~/project
#     git clone https://github.com/themakerrobot/arm-lab.git arm-lab
#     cd arm-lab
#     chmod +x lerobot_conda.sh
#     sudo -v
#     nohup ./lerobot_conda.sh > /dev/null 2>&1 &
#     tail -f lerobot_conda.log
#
#  끝나면:
#     source ~/project/arm-lab/activate.sh      # conda 활성화 + ~/project/arm-lab 으로 이동
#     nohup python main.py > arm-lab.log 2>&1 &
#     → http://<ip>:8080/setup 에서 포트·카메라 지정, /calib 에서 캘리브레이션
#
#  핵심 주의사항 (Thor):
#   1) PyTorch 를 pip 기본 인덱스에서 받으면 CUDA 를 못 잡습니다.
#      aarch64-sbsa / CUDA 13 전용 휠 인덱스를 써야 합니다.
#   2) 그 휠이 cp312 라서 conda 파이썬도 3.12 여야 합니다.
# ============================================================================
set -Eeuo pipefail

# ------------------------------- 설정 ---------------------------------------
WORKDIR="${HOME}/project/arm-lab"          # 이 레포 (main.py 가 있는 곳)
LOGFILE="${WORKDIR}/lerobot_conda.log"
CONDA_DIR="${HOME}/miniforge3"
ENV_NAME="arm-lab"
PY_VER="3.12"                              # Thor 휠이 cp312. 바꾸지 말 것
LEROBOT_SRC="${WORKDIR}/lerobot-src"
LEROBOT_COMMIT="e40b58a8dfa9e7b86918c374791599d070518d11"   # README 와 동일. arm-lab 이 이 API 에 맞춰져 있음
DATA_DIR="${WORKDIR}/data"

ARCH="$(uname -m)"
PLATFORM="${ARMLAB_PLATFORM:-}"
if [[ -z "${PLATFORM}" ]]; then
  if [[ "${ARCH}" == "aarch64" ]]; then
    PLATFORM="thor"
  elif command -v nvidia-smi >/dev/null 2>&1 || lspci 2>/dev/null | grep -qi 'nvidia'; then
    PLATFORM="cuda"
  else
    PLATFORM="intel"
  fi
fi
case "${PLATFORM}" in
  thor)
    MINIFORGE="Miniforge3-Linux-aarch64.sh"
    TORCH_INDEXES=(
      "https://pypi.jetson-ai-lab.io/sbsa/cu130"
      "https://pypi.jetson-ai-lab.io/sbsa/cu129"
      "https://pypi.jetson-ai-lab.dev/sbsa/cu130"
    )
    TORCH_PKGS="torch torchvision torchaudio"
    ;;
  cuda)
    MINIFORGE="Miniforge3-Linux-x86_64.sh"
    TORCH_INDEXES=("https://download.pytorch.org/whl/cu130")
    # lerobot e40b58a 가 torch<2.12 / torchvision<0.27 을 요구합니다 — 최신 휠을 받으면 4-1 에서 충돌
    TORCH_PKGS="torch<2.12 torchvision<0.27"
    ;;
  intel)
    [[ "${ARCH}" == "x86_64" ]] || { echo "intel 플랫폼은 x86_64 전용입니다 (현재 ${ARCH})"; exit 1; }
    MINIFORGE="Miniforge3-Linux-x86_64.sh"
    TORCH_INDEXES=("https://download.pytorch.org/whl/cpu" "https://pypi.org/simple")
    TORCH_PKGS="torch<2.12 torchvision<0.27"
    ;;
  *) echo "ARMLAB_PLATFORM 은 thor | cuda | intel 중 하나: ${PLATFORM}"; exit 1 ;;
esac
# ---------------------------------------------------------------------------

mkdir -p "${WORKDIR}" "${DATA_DIR}"
exec > >(tee -a "${LOGFILE}") 2>&1

log()  { echo -e "\n[$(date '+%F %T')] === $* ==="; }
warn() { echo "[$(date '+%F %T')] !! $*"; }
die()  { echo "[$(date '+%F %T')] XX 치명적 실패: $*"; exit 1; }
trap 'warn "line ${LINENO} 오류. 로그: ${LOGFILE}"' ERR

log "시작. arch=${ARCH} platform=${PLATFORM} 작업경로=${WORKDIR}"
[[ -f "${WORKDIR}/main.py" ]] || die "${WORKDIR}/main.py 가 없습니다. 이 레포를 ~/project/arm-lab 에 clone 한 뒤 실행하세요"

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
log "3. PyTorch 설치 (${PLATFORM})"
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
if [[ "${PLATFORM}" == "intel" ]]; then
python - <<'PY'
import torch
print("torch      :", torch.__version__, "(Intel 경로 — CPU 휠. 추론 가속은 OpenVINO 가 담당)")
PY
return
fi
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
log "3-1. torch 확인 (thor/cuda 는 CUDA 인식 필수)"
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

log "4-1. lerobot[feetech,dynamixel,training,diffusion,smolvla] 설치 (torch 는 위 휠 유지)"
# torch 를 pip 기본 인덱스 것으로 덮어쓰지 않도록 현재 버전으로 핀.
# 로컬 버전 꼬리표(+cu130 / +cpu)는 뗍니다 — 붙어 있으면 pip 가 "Cannot install None" 으로 해석을 포기합니다.
# (PEP 440: 'torch==2.11.0' 은 설치된 2.11.0+cu130 을 그대로 만족합니다)
python - <<'PY' > /tmp/torch-constraint.txt
import torch, torchvision
print(f"torch=={torch.__version__.split('+')[0]}")
print(f"torchvision=={torchvision.__version__.split('+')[0]}")
PY
cat /tmp/torch-constraint.txt
# feetech = SO-ARM101 (STS3215), dynamixel = ROBOTIS OMX (XL430/XL330)
# diffusion / smolvla = Training 탭에서 고를 수 있는 정책 (diffusers / transformers)
PIP_CONSTRAINT=/tmp/torch-constraint.txt pip install -e ".[feetech,dynamixel,training,diffusion,smolvla]" \
  || die "lerobot 설치 실패 (로그 확인)"

# torchcodec 은 Jetson 에서 문제를 일으켜 pyav 디코딩으로 통일합니다 (README 와 동일)
pip uninstall -y torchcodec || true
pip install "av>=15.0.0,<16.0.0"

log "4-2. arm-lab 의존성"
pip install "fastapi<1.0" uvicorn

log "4-3. 양팔 OMX 플러그인 (bi_omx_follower / bi_omx_leader)"
# lerobot 에 양팔 OMX 가 없어서 이 레포의 plugins/ 를 설치합니다. 설치된 패키지 이름이
# lerobot_robot_* / lerobot_teleoperator_* 이면 lerobot-record/rollout/train 이 자동으로 읽습니다.
PIP_CONSTRAINT=/tmp/torch-constraint.txt pip install --no-deps \
  -e "${WORKDIR}/plugins/lerobot_robot_bi_omx" -e "${WORKDIR}/plugins/lerobot_teleoperator_bi_omx" \
  || die "양팔 OMX 플러그인 설치 실패"

log "4-4. torch 가 덮어써지지 않았는지 재확인"
check_cuda || die "lerobot 설치 과정에서 torch 가 CPU 휠로 바뀌었습니다. pip uninstall -y torch torchvision 후 3단계 인덱스로 재설치"

if [[ "${PLATFORM}" == "intel" ]]; then
  log "4-5. OpenVINO / NNCF (Intel NPU·GPU·CPU 추론)"
  PIP_CONSTRAINT=/tmp/torch-constraint.txt pip install "openvino>=2025.4" "nncf>=2.19" \
    || die "openvino 설치 실패"
  # /dev/accel(NPU), /dev/dri(GPU) 접근 권한
  sudo usermod -aG render,video "${USER}" || true

  log "4-6. Intel NPU / GPU 드라이버 점검 (설치는 하지 않습니다 — 안내만)"
  CPU_NAME="$(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ *//')"
  echo "CPU : ${CPU_NAME}"
  if ! grep -qi 'Core(TM) Ultra' <<<"${CPU_NAME}"; then
    warn "Core Ultra 가 아닙니다 — NPU 가 없을 수 있습니다. OpenVINO GPU/CPU 로만 추론합니다"
  fi
  if [[ -e /dev/accel/accel0 ]]; then
    echo "NPU : /dev/accel/accel0 있음 (intel_vpu 커널 드라이버 로드됨)"
  else
    warn "NPU 장치(/dev/accel/accel0) 없음. 커널 intel_vpu 모듈과 사용자 공간 드라이버가 필요합니다."
    warn "  → https://github.com/intel/linux-npu-driver/releases 에서 Ubuntu 버전에 맞는 .deb 설치 후 재부팅"
    warn "  (커널/드라이버 버전 조합은 해당 릴리스 노트로 확인 필요)"
  fi
  if ls /dev/dri/renderD* >/dev/null 2>&1; then
    echo "GPU : $(ls /dev/dri/renderD* | tr '\n' ' ')"
    dpkg -l 2>/dev/null | grep -qE 'intel-opencl-icd|libze-intel-gpu1|intel-level-zero-gpu' \
      || warn "Intel GPU 컴퓨트 런타임(intel-opencl-icd / level-zero) 미설치 — OpenVINO GPU 를 쓰려면 https://github.com/intel/compute-runtime/releases 참고"
  else
    warn "GPU render 노드(/dev/dri/renderD*) 없음"
  fi
fi

# ----------------------------------------------------------------- 5. 임포트 검증
log "5. 최종 검증"
python - <<'PY'
import torch
print("torch   :", torch.__version__, "| cuda:", torch.cuda.is_available())
try:
    import openvino as ov
    print("openvino:", ov.__version__, "| devices:", ov.Core().available_devices)
except ImportError:
    print("openvino: (미설치 — intel 플랫폼에서만 설치)")
import lerobot
from lerobot.robots.so_follower import SOFollower
from lerobot.robots.bi_so_follower import BiSOFollower
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.omx_follower import OmxFollower
from lerobot.motors.dynamixel import DynamixelMotorsBus
import dynamixel_sdk  # noqa: F401
import lerobot_robot_bi_omx, lerobot_teleoperator_bi_omx  # noqa: F401
from lerobot.scripts.lerobot_record import record_loop
import fastapi, uvicorn, cv2, av
print("lerobot :", "OK (so_follower / bi_so_follower / feetech / omx / bi_omx / dynamixel / record_loop)")
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
cd "$(h "${WORKDIR}")"
EOF
chmod +x "${WORKDIR}/activate.sh"

# ------------------------------------------------------------------- 완료
log "완료"
cat <<EOF

  플랫폼    : ${PLATFORM}
  conda env : ${ENV_NAME}  (python ${PY_VER})
  lerobot   : ${LEROBOT_SRC} @ ${LEROBOT_COMMIT:0:8}
  데이터    : ${DATA_DIR}   (HF_HOME=${DATA_DIR}/hf → 캘리브레이션은 \$HF_HOME/lerobot/calibration)
  활성화    : source ${WORKDIR}/activate.sh

  --- arm-lab 실행 ---
  source ${WORKDIR}/activate.sh      # conda 활성화 + ${WORKDIR} 로 이동
  nohup python main.py > arm-lab.log 2>&1 &
  → http://<ip>:8080

  --- 웹에서 순서대로 ---
  Setup   : 한팔/양팔 → 포트 스캔 → 포트 감시로 팔 판별 → 카메라 스캔·추가 → 저장
  Calib   : 팔로워·리더 각각 (양팔이면 4개)
  Control : 슬라이더 범위 확인
  Collect : 수집

  * dialout / video 그룹 반영을 위해 재로그인(또는 재부팅) 한 번 필요합니다.
  * 학습 시 GPU 를 쓰는 다른 서비스(vLLM 등)는 내리세요.
  * intel: Training 탭 → 체크포인트 "OpenVINO 변환" → Rollout 탭에서 추론 엔진 NPU/GPU/CPU 선택.
           render 그룹 반영(NPU/GPU 권한)을 위해 재로그인 필요.

EOF
