#!/usr/bin/env bash
# =============================================================================
# Claudia ASR Service - Dependencies Setup Script
# Install faster-whisper + silero-vad on system Python 3.8
#
# Usage: bash scripts/setup_asr_venv.sh
#
# Platform: Ubuntu 20.04 (aarch64), Jetson Orin NX, CUDA 11.4
# NOTE: faster-whisper uses CTranslate2, NOT PyTorch for inference.
#       No separate venv or CUDA PyTorch wheel needed.
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
REQUIREMENTS_FILE="/home/m1ng/claudia/src/claudia/audio/asr_service/requirements.txt"
PYTHON_BIN="python3"

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
# Step 1: Check Python 3.8
# ---------------------------------------------------------------------------
log_info "Step 1/5: Checking Python availability..."

if ! command -v "$PYTHON_BIN" &>/dev/null; then
    log_error "Python 3 not found."
    exit 1
fi

PY_VERSION=$("$PYTHON_BIN" --version 2>&1)
log_ok "Found $PY_VERSION"

# ---------------------------------------------------------------------------
# Step 2: Install dependencies
# ---------------------------------------------------------------------------
log_info "Step 2/5: Installing dependencies..."

# 2a. faster-whisper (CTranslate2 backend)
log_info "  Installing faster-whisper..."
pip3 install --user --quiet faster-whisper

# 2b. silero-vad (uses torch internally for CPU VAD)
log_info "  Installing silero-vad..."
pip3 install --user --quiet silero-vad

# 2c. numpy, soundfile
log_info "  Installing numpy, soundfile..."
pip3 install --user --quiet numpy soundfile

log_ok "All dependencies installed"

# ---------------------------------------------------------------------------
# Step 3: Verify imports
# ---------------------------------------------------------------------------
log_info "Step 3/5: Verifying imports..."

VERIFY_FAILED=0

# faster-whisper + ctranslate2
if python3 -c "from faster_whisper import WhisperModel; import ctranslate2; print(f'faster-whisper OK, ctranslate2 {ctranslate2.__version__}')"; then
    log_ok "faster-whisper + ctranslate2 OK"
else
    log_error "faster-whisper import failed"
    VERIFY_FAILED=1
fi

# silero-vad
if python3 -c "import torch; model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False); print('silero-vad OK')" 2>/dev/null; then
    log_ok "silero-vad OK"
else
    log_warn "silero-vad model load failed (will download on first use)"
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
# Step 4: Download Whisper model to local cache
# ---------------------------------------------------------------------------
ASR_MODEL_SIZE="${CLAUDIA_ASR_MODEL:-small}"
log_info "Step 4/5: Downloading Whisper model ($ASR_MODEL_SIZE) to local cache..."

if python3 -c "
from faster_whisper import WhisperModel
model_size = '$ASR_MODEL_SIZE'
print(f'Downloading whisper-{model_size}...')
model = WhisperModel(model_size, device='cpu', compute_type='int8')
print(f'Model cached: whisper-{model_size}')
del model
" 2>&1; then
    log_ok "Whisper model cached locally"
else
    log_warn "Model download failed (may need internet)"
    log_warn "Model will be downloaded on first use"
fi

# ---------------------------------------------------------------------------
# Step 5: Update .gitignore
# ---------------------------------------------------------------------------
log_info "Step 5/5: Checking .gitignore..."

GITIGNORE="/home/m1ng/claudia/.gitignore"
if grep -qxF '.venvs/' "$GITIGNORE" 2>/dev/null; then
    log_ok ".venvs/ already in .gitignore"
else
    log_ok ".gitignore OK (no venv needed)"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "============================================"
log_ok "ASR setup complete!"
echo "============================================"
echo ""
echo "Backend:    faster-whisper (CTranslate2)"
echo "Model:      whisper-$ASR_MODEL_SIZE"
echo "Python:     $(python3 --version)"
echo "Device:     CPU (INT8 quantization)"
echo ""
echo "Environment variables (optional):"
echo "  CLAUDIA_ASR_MODEL=small|medium|large-v3  (default: small)"
echo "  CLAUDIA_ASR_DEVICE=cpu|cuda               (default: cpu)"
echo "  CLAUDIA_ASR_COMPUTE_TYPE=int8|float16      (default: int8)"
echo ""
echo "Next steps:"
echo "  1. Run Phase 0 mic test: python3 scripts/validation/audio/go2_mic_ssh_test.py"
echo "  2. Run ASR smoke test:   python3 scripts/validation/audio/asr_inference_test.py"
echo ""
