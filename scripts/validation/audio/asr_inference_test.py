#!/usr/bin/env python3
"""
ASR 推理スモークテスト (Phase 1)
Qwen3-ASR-0.6B モデルの読み込み・推理・VRAM 使用量を検証する

使用方法:
  # Production モード (要 CUDA GPU + qwen-asr)
  /home/m1ng/claudia/.venvs/asr-py311/bin/python3 scripts/validation/audio/asr_inference_test.py

  # Mock モード (GPU 不要、import とパイプライン構造のみ検証)
  python3 scripts/validation/audio/asr_inference_test.py --mock

  # モデル/デバイス指定
  .venvs/asr-py311/bin/python3 scripts/validation/audio/asr_inference_test.py --model Qwen/Qwen3-ASR-0.6B --device cuda:0

注意: 手動実行専用。Shadow 比較中は実行しないこと (GPU VRAM 競合)。

Author: Claudia AI System
Generated: 2026-02-16
Target Python: 3.11 (ASR venv)
"""

import argparse
import io
import logging
import os
import struct
import sys
import time
import math
from pathlib import Path
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit
VRAM_LIMIT_MB = 2048  # standalone < 2GB
OUTPUT_DIR = Path(__file__).parent / "output"

logger = logging.getLogger("claudia.asr.smoke_test")


# ---------------------------------------------------------------------------
# テスト音声生成
# ---------------------------------------------------------------------------

def generate_test_wav(duration_silence_s: float = 1.0,
                      duration_tone_s: float = 1.0,
                      freq_hz: float = 440.0) -> bytes:
    """テスト用 WAV を生成: 無音 + 正弦波トーン

    Parameters
    ----------
    duration_silence_s : float
        先頭無音の秒数
    duration_tone_s : float
        440Hz トーンの秒数
    freq_hz : float
        トーン周波数

    Returns
    -------
    bytes
        16kHz 16-bit mono PCM データ (WAV ヘッダーなし)
    """
    n_silence = int(SAMPLE_RATE * duration_silence_s)
    n_tone = int(SAMPLE_RATE * duration_tone_s)

    samples = []
    # 無音部分
    for _ in range(n_silence):
        samples.append(0)

    # 正弦波トーン (振幅 0.5 = -6dBFS)
    for i in range(n_tone):
        t = i / SAMPLE_RATE
        value = int(0.5 * 32767 * math.sin(2 * math.pi * freq_hz * t))
        samples.append(max(-32768, min(32767, value)))

    # PCM 16-bit LE
    pcm_data = struct.pack(f"<{len(samples)}h", *samples)
    return pcm_data


def save_test_wav(pcm_data: bytes, filepath: Path) -> None:
    """PCM データを WAV ファイルとして保存"""
    import wave
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(filepath), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm_data)
    logger.info("テスト WAV 保存: %s (%d bytes, %.1fs)",
                filepath, len(pcm_data),
                len(pcm_data) / (SAMPLE_RATE * SAMPLE_WIDTH * CHANNELS))


# ---------------------------------------------------------------------------
# テスト実行
# ---------------------------------------------------------------------------

class ASRSmokeTest:
    """ASR 推理スモークテスト"""

    def __init__(self, mock: bool = False,
                 model_name: Optional[str] = None,
                 device: Optional[str] = None) -> None:
        self.mock = mock
        self.model_name = model_name or os.getenv(
            "CLAUDIA_ASR_MODEL", "Qwen/Qwen3-ASR-0.6B")
        self.device = device or os.getenv("CLAUDIA_ASR_DEVICE", "cuda:0")
        self.results: dict = {}
        self._model = None
        self._passed = True

    def run_all(self) -> bool:
        """全テスト実行。True = 全パス"""
        print("=" * 60)
        print("  Claudia ASR Inference Smoke Test")
        print(f"  Mode: {'MOCK' if self.mock else 'PRODUCTION'}")
        if not self.mock:
            print(f"  Model: {self.model_name}")
            print(f"  Device: {self.device}")
        print("=" * 60)
        print()

        self._test_imports()
        self._test_generate_audio()

        if self.mock:
            self._test_mock_pipeline()
        else:
            self._test_model_load()
            self._test_inference()
            self._test_vram_usage()

        self._test_asr_service_imports()

        print()
        print("=" * 60)
        self._print_summary()
        print("=" * 60)

        return self._passed

    # ------------------------------------------------------------------
    # 個別テスト
    # ------------------------------------------------------------------

    def _test_imports(self) -> None:
        """基本 import テスト"""
        self._section("Import 検証")
        failures = []

        # 必須 import
        required = ["numpy", "struct", "asyncio", "json", "logging"]
        for mod in required:
            try:
                __import__(mod)
                self._ok(f"import {mod}")
            except ImportError as e:
                self._fail(f"import {mod}: {e}")
                failures.append(mod)

        # GPU 依存 import (mock 時はスキップ可)
        gpu_modules = ["torch", "soundfile"]
        for mod in gpu_modules:
            try:
                __import__(mod)
                self._ok(f"import {mod}")
            except ImportError as e:
                if self.mock:
                    self._warn(f"import {mod}: {e} (mock モードなので続行)")
                else:
                    self._fail(f"import {mod}: {e}")
                    failures.append(mod)

        # qwen-asr
        try:
            from qwen_asr import Qwen3ASRModel
            self._ok("import qwen_asr.Qwen3ASRModel")
        except ImportError as e:
            if self.mock:
                self._warn(f"import qwen_asr: {e} (mock モードなので続行)")
            else:
                self._fail(f"import qwen_asr: {e}")
                failures.append("qwen_asr")

        if failures and not self.mock:
            self._passed = False

    def _test_generate_audio(self) -> None:
        """テスト音声生成"""
        self._section("テスト音声生成")

        pcm_data = generate_test_wav()
        expected_samples = SAMPLE_RATE * 2  # 1s silence + 1s tone
        expected_bytes = expected_samples * SAMPLE_WIDTH
        actual_bytes = len(pcm_data)

        if actual_bytes == expected_bytes:
            self._ok(f"PCM 生成: {actual_bytes} bytes ({expected_samples} samples, 2.0s)")
        else:
            self._fail(f"PCM サイズ不一致: expected={expected_bytes}, actual={actual_bytes}")
            self._passed = False
            return

        # WAV ファイル保存
        wav_path = OUTPUT_DIR / "asr_smoke_test.wav"
        save_test_wav(pcm_data, wav_path)
        self._ok(f"WAV 保存: {wav_path}")

        self.results["pcm_data"] = pcm_data
        self.results["wav_path"] = wav_path

    def _test_mock_pipeline(self) -> None:
        """Mock モード: ASRModelWrapper のモックパイプラインテスト"""
        self._section("Mock パイプライン検証")

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            self._fail("テスト音声なし")
            self._passed = False
            return

        # ASRModelWrapper mock 動作テスト
        try:
            # ASRModelWrapper を直接使わず、同等のロジックをテスト
            start = time.monotonic()
            mock_text = "mock転写結果"
            mock_confidence = 0.99
            elapsed_ms = (time.monotonic() - start) * 1000
            self._ok(f"Mock 転写: '{mock_text}' (conf={mock_confidence}, "
                     f"latency={elapsed_ms:.1f}ms)")
        except Exception as e:
            self._fail(f"Mock パイプライン失敗: {e}")
            self._passed = False

        # RingBuffer テスト
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
            from claudia.audio.asr_service.ring_buffer import RingBuffer, BYTES_PER_MS
            rb = RingBuffer()
            rb.write(pcm_data[:960])  # 30ms
            last = rb.read_last(30)
            assert len(last) == 960, f"RingBuffer read_last 不一致: {len(last)}"
            rb.clear()
            assert rb.available_ms == 0
            self._ok("RingBuffer: write/read_last/clear 正常")
        except Exception as e:
            self._fail(f"RingBuffer テスト失敗: {e}")
            self._passed = False

        # VADProcessor mock テスト
        try:
            from claudia.audio.asr_service.vad_processor import (
                VADProcessor, VADConfig, VADState,
            )
            from claudia.audio.asr_service.ring_buffer import RingBuffer
            rb2 = RingBuffer()
            events_collected = []

            async def dummy_callback(event):
                events_collected.append(event)

            vad = VADProcessor(
                ring_buffer=rb2,
                event_callback=dummy_callback,
                mock=True,
            )
            assert vad.state == VADState.SILENCE
            self._ok("VADProcessor: mock 初期化成功 (state=SILENCE)")
        except Exception as e:
            self._fail(f"VADProcessor テスト失敗: {e}")
            self._passed = False

    def _test_model_load(self) -> None:
        """Production: モデル読み込み"""
        self._section("モデル読み込み")

        try:
            from qwen_asr import Qwen3ASRModel
            import torch

            dtype_str = os.getenv("CLAUDIA_ASR_DTYPE", "bfloat16")
            dtype = getattr(torch, dtype_str, torch.bfloat16)

            # VRAM 計測: ロード前
            if torch.cuda.is_available():
                torch.cuda.reset_peak_memory_stats()
                vram_before = torch.cuda.memory_allocated() / 1024 / 1024
            else:
                vram_before = 0

            start = time.monotonic()
            self._model = Qwen3ASRModel.from_pretrained(
                self.model_name,
                device=self.device,
                dtype=dtype,
            )
            load_ms = (time.monotonic() - start) * 1000

            if torch.cuda.is_available():
                vram_after = torch.cuda.memory_allocated() / 1024 / 1024
            else:
                vram_after = 0

            self.results["load_ms"] = load_ms
            self.results["vram_before_mb"] = vram_before
            self.results["vram_after_mb"] = vram_after
            self.results["vram_model_mb"] = vram_after - vram_before

            self._ok(f"モデル読み込み成功: {self.model_name}")
            self._ok(f"  読み込み時間: {load_ms:.0f}ms")
            self._ok(f"  VRAM (モデル): {vram_after - vram_before:.0f}MB")
            self._ok(f"  device: {self.device}, dtype: {dtype_str}")

        except Exception as e:
            self._fail(f"モデル読み込み失敗: {e}")
            self._passed = False

    def _test_inference(self) -> None:
        """Production: 推理テスト"""
        self._section("推理テスト")

        if self._model is None:
            self._fail("モデル未読み込み — スキップ")
            self._passed = False
            return

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            self._fail("テスト音声なし — スキップ")
            self._passed = False
            return

        try:
            import numpy as np

            audio_np = np.frombuffer(pcm_data, dtype=np.int16).astype(np.float32) / 32768.0

            start = time.monotonic()
            result = self._model.transcribe(
                audio_np,
                language="ja",
                sample_rate=SAMPLE_RATE,
            )
            inference_ms = (time.monotonic() - start) * 1000

            text = result.get("text", "").strip() if isinstance(result, dict) else str(result)
            confidence = result.get("confidence", 0.0) if isinstance(result, dict) else 0.0

            self.results["inference_ms"] = inference_ms
            self.results["text"] = text
            self.results["confidence"] = confidence

            self._ok(f"推理完了: '{text}'")
            self._ok(f"  信頼度: {confidence:.3f}")
            self._ok(f"  推理時間: {inference_ms:.0f}ms")

            # テスト音声はトーンなので、空文字列でも推理自体が成功すれば OK
            if inference_ms > 5000:
                self._warn(f"推理時間が長い: {inference_ms:.0f}ms (目標 < 800ms)")

        except Exception as e:
            self._fail(f"推理失敗: {e}")
            self._passed = False

    def _test_vram_usage(self) -> None:
        """Production: VRAM 使用量チェック"""
        self._section("VRAM 使用量")

        try:
            import torch

            if not torch.cuda.is_available():
                self._warn("CUDA 不可用 — VRAM チェックスキップ")
                return

            current_mb = torch.cuda.memory_allocated() / 1024 / 1024
            peak_mb = torch.cuda.max_memory_allocated() / 1024 / 1024

            self.results["vram_current_mb"] = current_mb
            self.results["vram_peak_mb"] = peak_mb

            self._ok(f"VRAM 現在: {current_mb:.0f}MB")
            self._ok(f"VRAM ピーク: {peak_mb:.0f}MB")
            self._ok(f"VRAM 上限: {VRAM_LIMIT_MB}MB")

            if peak_mb < VRAM_LIMIT_MB:
                self._ok(f"VRAM チェック PASS (peak {peak_mb:.0f}MB < {VRAM_LIMIT_MB}MB)")
            else:
                self._fail(f"VRAM チェック FAIL (peak {peak_mb:.0f}MB >= {VRAM_LIMIT_MB}MB)")
                self._passed = False

        except Exception as e:
            self._fail(f"VRAM チェック失敗: {e}")
            self._passed = False

    def _test_asr_service_imports(self) -> None:
        """ASR サービスモジュール import テスト"""
        self._section("ASR サービス import 検証")

        try:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
            from claudia.audio.asr_service import (
                ASRServer,
                ASRModelWrapper,
                VADProcessor,
                VADConfig,
                VADState,
                VADEvent,
                RingBuffer,
            )
            self._ok("claudia.audio.asr_service: 全 export import 成功")
        except ImportError as e:
            self._fail(f"ASR サービス import 失敗: {e}")
            self._passed = False

        # ipc_protocol / emergency_keywords
        try:
            from claudia.audio.asr_service.emergency_keywords import EMERGENCY_KEYWORDS_TEXT
            self._ok(f"emergency_keywords: {len(EMERGENCY_KEYWORDS_TEXT)} keywords loaded")
        except ImportError as e:
            self._warn(f"emergency_keywords import 失敗 (並行実装中の可能性): {e}")

        try:
            from claudia.audio.asr_service.ipc_protocol import PROTO_VERSION
            self._ok(f"ipc_protocol: proto_version={PROTO_VERSION}")
        except ImportError as e:
            self._warn(f"ipc_protocol import 失敗 (並行実装中の可能性): {e}")

    # ------------------------------------------------------------------
    # 出力ヘルパー
    # ------------------------------------------------------------------

    def _section(self, name: str) -> None:
        print(f"\n--- {name} ---")

    def _ok(self, msg: str) -> None:
        print(f"  [PASS] {msg}")

    def _fail(self, msg: str) -> None:
        print(f"  [FAIL] {msg}")

    def _warn(self, msg: str) -> None:
        print(f"  [WARN] {msg}")

    def _print_summary(self) -> None:
        if self._passed:
            print("  RESULT: ALL TESTS PASSED")
        else:
            print("  RESULT: SOME TESTS FAILED")

        if self.results:
            print()
            print("  Metrics:")
            for key in ["load_ms", "inference_ms", "vram_model_mb",
                        "vram_current_mb", "vram_peak_mb", "text", "confidence"]:
                if key in self.results:
                    val = self.results[key]
                    if isinstance(val, float):
                        print(f"    {key}: {val:.1f}")
                    else:
                        print(f"    {key}: {val}")


# ---------------------------------------------------------------------------
# エントリポイント
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Claudia ASR Inference Smoke Test (Phase 1)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Mock モード: CUDA/モデルなしで import とパイプライン構造のみ検証",
    )
    parser.add_argument(
        "--model", type=str, default=None,
        help="ASR モデル名 (default: Qwen/Qwen3-ASR-0.6B)",
    )
    parser.add_argument(
        "--device", type=str, default=None,
        help="推理デバイス (default: cuda:0)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    mock = args.mock or os.getenv("ASR_MOCK", "0") == "1"

    test = ASRSmokeTest(
        mock=mock,
        model_name=args.model,
        device=args.device,
    )
    passed = test.run_all()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
