#!/usr/bin/env bash
# =============================================================================
# Claudia ASR Service - Python 3.11 Venv Setup Script
# Phase 1.0: Install Python 3.11 venv + ASR dependencies for Jetson Orin NX
#
# Usage: bash scripts/setup_asr_venv.sh
#
# Platform: Ubuntu 20.04 (aarch64), Jetson Orin NX, CUDA 11.4
# CRITICAL: onnxruntime >= 1.20.0 crashes on Jetson Orin NX (SIGABRT in CPU
#           topology detection). Pinned to 1.18.1.
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
VENV_DIR="/home/m1ng/claudia/.venvs/asr-py311"
REQUIREMENTS_FILE="/home/m1ng/claudia/src/claudia/audio/asr_service/requirements.txt"
TORCH_INDEX_URL="https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/"
PYTHON_BIN="python3.11"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

log_info()  { echo -e "${CYAN}[INFO]${NC} $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ---------------------------------------------------------------------------
# Step 1: Check Python 3.11
# ---------------------------------------------------------------------------
log_info "Step 1/6: Checking Python 3.11 availability..."

if ! command -v "$PYTHON_BIN" &>/dev/null; then
    log_error "Python 3.11 not found."
    echo ""
    echo "Install Python 3.11 on Ubuntu 20.04:"
    echo "  sudo add-apt-repository ppa:deadsnakes/ppa"
    echo "  sudo apt update"
    echo "  sudo apt install python3.11 python3.11-venv python3.11-dev"
    echo ""
    echo "After installing, re-run this script."
    exit 1
fi

PY_VERSION=$("$PYTHON_BIN" --version 2>&1)
log_ok "Found $PY_VERSION"

# Also verify venv module is available
if ! "$PYTHON_BIN" -m venv --help &>/dev/null; then
    log_error "python3.11-venv module not installed."
    echo "  sudo apt install python3.11-venv"
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 2: Create venv (idempotent)
# ---------------------------------------------------------------------------
log_info "Step 2/6: Setting up virtual environment at $VENV_DIR"

if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/bin/python3" ]; then
    log_ok "Venv already exists, skipping creation"
else
    mkdir -p "$(dirname "$VENV_DIR")"
    "$PYTHON_BIN" -m venv "$VENV_DIR"
    log_ok "Venv created at $VENV_DIR"
fi

# Activate venv
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
log_ok "Venv activated ($(python3 --version))"

# Upgrade pip
log_info "Upgrading pip..."
pip install --quiet --upgrade pip

# ---------------------------------------------------------------------------
# Step 3: Install dependencies (order matters)
# ---------------------------------------------------------------------------
log_info "Step 3/6: Installing dependencies..."

# 3a. PyTorch from JetPack index
log_info "  Installing torch (JetPack CUDA 11.4 wheel)..."
pip install --quiet torch --extra-index-url "$TORCH_INDEX_URL"

# 3b. torchaudio from same index
log_info "  Installing torchaudio..."
pip install --quiet torchaudio --extra-index-url "$TORCH_INDEX_URL"

# 3c. qwen-asr
log_info "  Installing qwen-asr..."
pip install --quiet qwen-asr

# 3d. silero-vad
log_info "  Installing silero-vad..."
pip install --quiet silero-vad

# 3e. numpy, soundfile
log_info "  Installing numpy, soundfile..."
pip install --quiet numpy soundfile

# 3f. Pin onnxruntime (CRITICAL: >= 1.20.0 crashes on Jetson Orin NX)
log_info "  Installing onnxruntime==1.18.1 (pinned for Jetson compatibility)..."
pip install --quiet "onnxruntime==1.18.1"

log_ok "All dependencies installed"

# ---------------------------------------------------------------------------
# Step 4: Verify imports
# ---------------------------------------------------------------------------
log_info "Step 4/6: Verifying imports..."

VERIFY_FAILED=0

# torch + CUDA
if python3 -c "import torch; assert torch.cuda.is_available(), 'CUDA not available'; print(f'torch {torch.__version__}, CUDA {torch.version.cuda}')"; then
    log_ok "torch + CUDA OK"
else
    log_error "torch CUDA verification failed"
    VERIFY_FAILED=1
fi

# torchaudio
if python3 -c "import torchaudio; print(f'torchaudio {torchaudio.__version__}')"; then
    log_ok "torchaudio OK"
else
    log_error "torchaudio import failed"
    VERIFY_FAILED=1
fi

# qwen-asr
if python3 -c "from qwen_asr import Qwen3ASRModel; print('qwen_asr OK')"; then
    log_ok "qwen-asr OK"
else
    log_warn "qwen-asr import failed (may need model download)"
    # Not fatal - model might need downloading separately
fi

# silero-vad
if python3 -c "import torch; model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False); print('silero-vad OK')" 2>/dev/null; then
    log_ok "silero-vad OK"
else
    log_warn "silero-vad model load failed (will download on first use)"
fi

# onnxruntime version check
if python3 -c "import onnxruntime; v=onnxruntime.__version__; print(f'onnxruntime {v}'); assert v.startswith('1.18'), f'Expected 1.18.x, got {v}'"; then
    log_ok "onnxruntime 1.18.1 OK (safe for Jetson)"
else
    log_error "onnxruntime version check failed"
    VERIFY_FAILED=1
fi

# numpy, soundfile
if python3 -c "import numpy, soundfile; print(f'numpy {numpy.__version__}, soundfile {soundfile.__version__}')"; then
    log_ok "numpy + soundfile OK"
else
    log_error "numpy/soundfile import failed"
    VERIFY_FAILED=1
fi

if [ "$VERIFY_FAILED" -eq 1 ]; then
    log_error "Some verifications failed. Check output above."
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 5: Download Qwen3-ASR-0.6B model to local cache
# ---------------------------------------------------------------------------
ASR_MODEL_NAME="${CLAUDIA_ASR_MODEL:-Qwen/Qwen3-ASR-0.6B}"
log_info "Step 5/6: Downloading ASR model ($ASR_MODEL_NAME) to local cache..."

if python3 -c "
from transformers import AutoProcessor, AutoModelForSpeechSeq2Seq
import os

model_name = '$ASR_MODEL_NAME'
cache_dir = os.path.expanduser('~/.cache/huggingface/hub')

# Check if model is already cached by looking for config.json
# This avoids re-downloading on idempotent runs
try:
    processor = AutoProcessor.from_pretrained(model_name)
    print(f'Processor loaded: {model_name}')
except Exception:
    print(f'Processor not cached, downloading {model_name}...')
    processor = AutoProcessor.from_pretrained(model_name)

try:
    # Only download weights, don't load to GPU (save VRAM during setup)
    import torch
    model = AutoModelForSpeechSeq2Seq.from_pretrained(
        model_name,
        torch_dtype=torch.float32,
        device_map='cpu',
    )
    print(f'Model cached: {model_name}')
    del model
except Exception as e:
    print(f'Model download via transformers failed: {e}')
    # Try qwen-asr specific download
    try:
        from qwen_asr import Qwen3ASRModel
        m = Qwen3ASRModel(model_name, device='cpu')
        print(f'Model cached via qwen_asr: {model_name}')
        del m
    except Exception as e2:
        print(f'qwen_asr download also failed: {e2}')
        raise
" 2>&1; then
    log_ok "ASR model cached locally"
else
    log_warn "Model download failed (may need internet or manual download)"
    log_warn "You can download later: python3 -c \"from qwen_asr import Qwen3ASRModel; Qwen3ASRModel('$ASR_MODEL_NAME')\""
    # Not fatal - model can be downloaded on first use
fi

# ---------------------------------------------------------------------------
# Step 6: Update .gitignore
# ---------------------------------------------------------------------------
log_info "Step 6/6: Checking .gitignore for .venvs/..."

GITIGNORE="/home/m1ng/claudia/.gitignore"
if grep -qxF '.venvs/' "$GITIGNORE" 2>/dev/null; then
    log_ok ".venvs/ already in .gitignore"
else
    echo "" >> "$GITIGNORE"
    echo "# ASR Python 3.11 virtual environment" >> "$GITIGNORE"
    echo ".venvs/" >> "$GITIGNORE"
    log_ok "Added .venvs/ to .gitignore"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "============================================"
log_ok "ASR venv setup complete!"
echo "============================================"
echo ""
echo "Venv location: $VENV_DIR"
echo "Activate:      source $VENV_DIR/bin/activate"
echo "Requirements:  $REQUIREMENTS_FILE"
echo ""
echo "Next steps:"
echo "  1. Run Phase 0 mic test: python3 scripts/validation/audio/go2_mic_ssh_test.py"
echo "  2. Run ASR smoke test:   $VENV_DIR/bin/python3 scripts/validation/audio/asr_inference_test.py"
echo ""
