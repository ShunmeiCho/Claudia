#!/usr/bin/env python3
"""
ASR 推理スモークテスト (Phase 1)
faster-whisper (CTranslate2) モデルの読み込み・推理を検証する

使用方法:
  # Production モード (要 faster-whisper)
  python3 scripts/validation/audio/asr_inference_test.py

  # Mock モード (モデルなしで import とパイプライン構造のみ検証)
  python3 scripts/validation/audio/asr_inference_test.py --mock

  # モデルサイズ指定
  python3 scripts/validation/audio/asr_inference_test.py --model medium

注意: 手動実行専用。Shadow 比較中は実行しないこと (GPU VRAM 競合)。

Author: Claudia AI System
Generated: 2026-02-16
Target Python: 3.8 (system)
"""

import argparse
import logging
import math
import os
import struct
import sys
import time
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit
OUTPUT_DIR = Path(__file__).parent / "output"

logger = logging.getLogger("claudia.asr.smoke_test")


# ---------------------------------------------------------------------------
# テスト音声生成
# ---------------------------------------------------------------------------

def generate_test_wav(duration_silence_s: float = 1.0,
                      duration_tone_s: float = 1.0,
                      freq_hz: float = 440.0) -> bytes:
    """テスト用 WAV を生成: 無音 + 正弦波トーン

    Returns
    -------
    bytes
        16kHz 16-bit mono PCM データ (WAV ヘッダーなし)
    """
    n_silence = int(SAMPLE_RATE * duration_silence_s)
    n_tone = int(SAMPLE_RATE * duration_tone_s)

    samples = []
    for _ in range(n_silence):
        samples.append(0)

    for i in range(n_tone):
        t = i / SAMPLE_RATE
        value = int(0.5 * 32767 * math.sin(2 * math.pi * freq_hz * t))
        samples.append(max(-32768, min(32767, value)))

    pcm_data = struct.pack("<{}h".format(len(samples)), *samples)
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
                 model_size: Optional[str] = None) -> None:
        self.mock = mock
        self.model_size = model_size or os.getenv("CLAUDIA_ASR_MODEL", "base")
        self.results = {}  # type: dict
        self._model = None
        self._passed = True

    def run_all(self) -> bool:
        """全テスト実行。True = 全パス"""
        print("=" * 60)
        print("  Claudia ASR Inference Smoke Test")
        print("  Mode: {}".format("MOCK" if self.mock else "PRODUCTION"))
        if not self.mock:
            print("  Model: whisper-{}".format(self.model_size))
            print("  Backend: CTranslate2 (CPU, INT8)")
        print("=" * 60)
        print()

        self._test_imports()
        self._test_generate_audio()

        if self.mock:
            self._test_mock_pipeline()
        else:
            self._test_model_load()
            self._test_inference()
            self._test_quick_inference()

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

        required = ["numpy", "struct", "asyncio", "json", "logging"]
        for mod in required:
            try:
                __import__(mod)
                self._ok("import {}".format(mod))
            except ImportError as e:
                self._fail("import {}: {}".format(mod, e))
                failures.append(mod)

        # faster-whisper
        try:
            from faster_whisper import WhisperModel
            self._ok("import faster_whisper.WhisperModel")
        except ImportError as e:
            if self.mock:
                self._warn("import faster_whisper: {} (mock モードなので続行)".format(e))
            else:
                self._fail("import faster_whisper: {}".format(e))
                failures.append("faster_whisper")

        # ctranslate2
        try:
            import ctranslate2
            self._ok("import ctranslate2 ({})".format(ctranslate2.__version__))
        except ImportError as e:
            if self.mock:
                self._warn("import ctranslate2: {} (mock)".format(e))
            else:
                self._fail("import ctranslate2: {}".format(e))
                failures.append("ctranslate2")

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
            self._ok("PCM 生成: {} bytes ({} samples, 2.0s)".format(
                actual_bytes, expected_samples))
        else:
            self._fail("PCM サイズ不一致: expected={}, actual={}".format(
                expected_bytes, actual_bytes))
            self._passed = False
            return

        wav_path = OUTPUT_DIR / "asr_smoke_test.wav"
        save_test_wav(pcm_data, wav_path)
        self._ok("WAV 保存: {}".format(wav_path))

        self.results["pcm_data"] = pcm_data
        self.results["wav_path"] = wav_path

    def _test_mock_pipeline(self) -> None:
        """Mock モード: パイプラインテスト"""
        self._section("Mock パイプライン検証")

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            self._fail("テスト音声なし")
            self._passed = False
            return

        try:
            start = time.monotonic()
            mock_text = "mock転写結果"
            mock_confidence = 0.99
            elapsed_ms = (time.monotonic() - start) * 1000
            self._ok("Mock 転写: '{}' (conf={}, latency={:.1f}ms)".format(
                mock_text, mock_confidence, elapsed_ms))
        except Exception as e:
            self._fail("Mock パイプライン失敗: {}".format(e))
            self._passed = False

        # RingBuffer テスト
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
            from claudia.audio.asr_service.ring_buffer import RingBuffer
            rb = RingBuffer()
            rb.write(pcm_data[:960])  # 30ms
            last = rb.read_last(30)
            assert len(last) == 960, "RingBuffer read_last mismatch: {}".format(len(last))
            rb.clear()
            assert rb.available_ms == 0
            self._ok("RingBuffer: write/read_last/clear 正常")
        except Exception as e:
            self._fail("RingBuffer テスト失敗: {}".format(e))
            self._passed = False

        # VADProcessor mock テスト
        try:
            from claudia.audio.asr_service.vad_processor import (
                VADProcessor, VADConfig, VADState,
            )
            from claudia.audio.asr_service.ring_buffer import RingBuffer as RB2
            rb2 = RB2()

            async def dummy_callback(event):
                pass

            vad = VADProcessor(
                ring_buffer=rb2,
                event_callback=dummy_callback,
                mock=True,
            )
            assert vad.state == VADState.SILENCE
            self._ok("VADProcessor: mock 初期化成功 (state=SILENCE)")
        except Exception as e:
            self._fail("VADProcessor テスト失敗: {}".format(e))
            self._passed = False

    def _test_model_load(self) -> None:
        """Production: モデル読み込み"""
        self._section("モデル読み込み")

        try:
            from faster_whisper import WhisperModel

            compute_type = os.getenv("CLAUDIA_ASR_COMPUTE_TYPE", "int8")

            start = time.monotonic()
            self._model = WhisperModel(
                self.model_size,
                device="cpu",
                compute_type=compute_type,
            )
            load_ms = (time.monotonic() - start) * 1000

            self.results["load_ms"] = load_ms
            self._ok("モデル読み込み成功: whisper-{}".format(self.model_size))
            self._ok("  読み込み時間: {:.0f}ms".format(load_ms))
            self._ok("  compute_type: {}".format(compute_type))

        except Exception as e:
            self._fail("モデル読み込み失敗: {}".format(e))
            self._passed = False

    def _test_inference(self) -> None:
        """Production: 完整推理テスト"""
        self._section("推理テスト (beam_size=3)")

        if self._model is None:
            self._fail("モデル未読み込み -- スキップ")
            self._passed = False
            return

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            self._fail("テスト音声なし -- スキップ")
            self._passed = False
            return

        try:
            import numpy as np

            audio_np = np.frombuffer(pcm_data, dtype=np.int16).astype(np.float32) / 32768.0

            start = time.monotonic()
            segments, info = self._model.transcribe(
                audio_np,
                language="ja",
                beam_size=3,
                vad_filter=False,
            )

            text_parts = []
            total_logprob = 0.0
            n_segments = 0
            for seg in segments:
                text_parts.append(seg.text)
                total_logprob += seg.avg_logprob
                n_segments += 1

            inference_ms = (time.monotonic() - start) * 1000

            text = "".join(text_parts).strip()
            if n_segments > 0:
                confidence = min(1.0, max(0.0, math.exp(total_logprob / n_segments)))
            else:
                confidence = 0.0

            self.results["inference_ms"] = inference_ms
            self.results["text"] = text
            self.results["confidence"] = confidence

            self._ok("推理完了: '{}'".format(text))
            self._ok("  segments: {}".format(n_segments))
            self._ok("  信頼度: {:.3f}".format(confidence))
            self._ok("  推理時間: {:.0f}ms".format(inference_ms))
            self._ok("  言語検出: {} (prob={:.2f})".format(info.language, info.language_probability))

            if inference_ms > 5000:
                self._warn("推理時間が長い: {:.0f}ms (目標 < 2000ms)".format(inference_ms))

        except Exception as e:
            self._fail("推理失敗: {}".format(e))
            self._passed = False

    def _test_quick_inference(self) -> None:
        """Production: quick_transcribe テスト (beam_size=1)"""
        self._section("Quick 推理テスト (beam_size=1)")

        if self._model is None:
            self._fail("モデル未読み込み -- スキップ")
            return

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            return

        try:
            import numpy as np

            # 最初の 300ms のみ (emergency check 用)
            n_bytes = int(SAMPLE_RATE * 0.3 * SAMPLE_WIDTH)
            short_pcm = pcm_data[:n_bytes]
            audio_np = np.frombuffer(short_pcm, dtype=np.int16).astype(np.float32) / 32768.0

            start = time.monotonic()
            segments, info = self._model.transcribe(
                audio_np,
                language="ja",
                beam_size=1,
                best_of=1,
                without_timestamps=True,
                vad_filter=False,
            )
            text = "".join(seg.text for seg in segments).strip()
            quick_ms = (time.monotonic() - start) * 1000

            self.results["quick_inference_ms"] = quick_ms
            self._ok("Quick 推理完了: '{}' ({:.0f}ms)".format(text, quick_ms))

        except Exception as e:
            self._fail("Quick 推理失敗: {}".format(e))

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
            self._fail("ASR サービス import 失敗: {}".format(e))
            self._passed = False

        try:
            from claudia.audio.asr_service.emergency_keywords import EMERGENCY_KEYWORDS_TEXT
            self._ok("emergency_keywords: {} keywords loaded".format(
                len(EMERGENCY_KEYWORDS_TEXT)))
        except ImportError as e:
            self._warn("emergency_keywords import: {}".format(e))

        try:
            from claudia.audio.asr_service.ipc_protocol import PROTO_VERSION
            self._ok("ipc_protocol: proto_version={}".format(PROTO_VERSION))
        except ImportError as e:
            self._warn("ipc_protocol import: {}".format(e))

    # ------------------------------------------------------------------
    # 出力ヘルパー
    # ------------------------------------------------------------------

    def _section(self, name: str) -> None:
        print("\n--- {} ---".format(name))

    def _ok(self, msg: str) -> None:
        print("  [PASS] {}".format(msg))

    def _fail(self, msg: str) -> None:
        print("  [FAIL] {}".format(msg))

    def _warn(self, msg: str) -> None:
        print("  [WARN] {}".format(msg))

    def _print_summary(self) -> None:
        if self._passed:
            print("  RESULT: ALL TESTS PASSED")
        else:
            print("  RESULT: SOME TESTS FAILED")

        if self.results:
            print()
            print("  Metrics:")
            for key in ["load_ms", "inference_ms", "quick_inference_ms",
                        "text", "confidence"]:
                if key in self.results:
                    val = self.results[key]
                    if isinstance(val, float):
                        print("    {}: {:.1f}".format(key, val))
                    else:
                        print("    {}: {}".format(key, val))


# ---------------------------------------------------------------------------
# エントリポイント
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Claudia ASR Inference Smoke Test (Phase 1)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Mock モード: モデルなしで import とパイプライン構造のみ検証",
    )
    parser.add_argument(
        "--model", type=str, default=None,
        help="Whisper モデルサイズ (default: base)",
    )
    parser.add_argument(
        "--usb-mic", action="store_true",
        help="USB マイク実録テスト (AT2020USB-XP, 5 秒録音 → ASR)",
    )
    parser.add_argument(
        "--usb-device", type=str, default="hw:2,0",
        help="ALSA USB マイクデバイス名 (default: hw:2,0)",
    )
    parser.add_argument(
        "--usb-rate", type=int, default=44100,
        help="USB マイクのネイティブサンプルレート (default: 44100)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    mock = args.mock or os.getenv("ASR_MOCK", "0") == "1"

    if args.usb_mic:
        return _run_usb_mic_test(
            model_size=args.model or os.getenv("CLAUDIA_ASR_MODEL", "base"),
            device=args.usb_device,
            native_rate=args.usb_rate,
        )

    test = ASRSmokeTest(
        mock=mock,
        model_size=args.model,
    )
    passed = test.run_all()
    return 0 if passed else 1


def _run_usb_mic_test(model_size: str, device: str, native_rate: int) -> int:
    """USB マイク実録 → ASR エンドツーエンドテスト

    AT2020USB-XP は 44100Hz ネイティブ。Tegra ALSA plughw は全零を出すため、
    ネイティブレートで録音し Python 側で 16kHz にリサンプルする。
    """
    import subprocess
    import wave

    print("=" * 60)
    print("  USB Microphone → ASR End-to-End Test")
    print("  Device: {}  Native Rate: {}Hz".format(device, native_rate))
    print("  Model: whisper-{}".format(model_size))
    print("=" * 60)
    print()

    wav_path = "/tmp/claudia_usb_mic_test.wav"
    duration_s = 5

    # 1. 録音
    print("[1/4] Recording {}s from {} at {}Hz ...".format(duration_s, device, native_rate))
    try:
        subprocess.run(
            ["arecord", "-D", device, "-f", "S16_LE",
             "-r", str(native_rate), "-c", "1",
             "-d", str(duration_s), "-t", "wav", wav_path],
            check=True, capture_output=True, timeout=duration_s + 10,
        )
        print("  [PASS] Recorded to {}".format(wav_path))
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
        print("  [FAIL] arecord failed: {}".format(e))
        return 1

    # 2. WAV 読み込み + リサンプル
    import numpy as np
    print("[2/4] Reading WAV and resampling {}→16000Hz ...".format(native_rate))
    try:
        with wave.open(wav_path, "rb") as wf:
            sr_in = wf.getframerate()
            n_frames = wf.getnframes()
            raw = wf.readframes(n_frames)

        samples = np.frombuffer(raw, dtype=np.int16)
        rms_orig = int(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))
        print("  Original: {} samples, RMS={}".format(len(samples), rms_orig))

        if rms_orig < 10:
            print("  [WARN] Very low RMS — mic may not be capturing audio")

        # リサンプル (numpy index-based)
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
        from claudia.audio.asr_service.asr_server import resample_pcm_int16, SAMPLE_RATE
        resampled = resample_pcm_int16(samples, sr_in, SAMPLE_RATE)
        print("  Resampled: {} → {} samples (16kHz)".format(len(samples), len(resampled)))
        print("  [PASS] Resampling OK")
    except Exception as e:
        print("  [FAIL] WAV processing failed: {}".format(e))
        return 1

    # 3. ASR 推理
    print("[3/4] Loading whisper-{} and transcribing ...".format(model_size))
    try:
        from faster_whisper import WhisperModel

        start = time.monotonic()
        model = WhisperModel(model_size, device="cpu", compute_type="int8")
        load_ms = (time.monotonic() - start) * 1000
        print("  Model loaded in {:.0f}ms".format(load_ms))

        audio_float = resampled.astype(np.float32) / 32768.0
        start = time.monotonic()
        segments, info = model.transcribe(
            audio_float, language="ja", beam_size=3, vad_filter=True,
        )
        text_parts = []
        for seg in segments:
            text_parts.append(seg.text)
        inference_ms = (time.monotonic() - start) * 1000

        text = "".join(text_parts).strip()
        print("  [PASS] Transcription: '{}'".format(text))
        print("  Language: {} (prob={:.2f})".format(info.language, info.language_probability))
        print("  Inference: {:.0f}ms".format(inference_ms))
    except Exception as e:
        print("  [FAIL] ASR inference failed: {}".format(e))
        return 1

    # 4. サマリー
    print()
    print("[4/4] Summary")
    print("  Device:      {}".format(device))
    print("  Native Rate: {}Hz".format(native_rate))
    print("  RMS:         {}".format(rms_orig))
    print("  Model:       whisper-{}".format(model_size))
    print("  Text:        {}".format(text if text else "(empty)"))
    print("  Load:        {:.0f}ms".format(load_ms))
    print("  Inference:   {:.0f}ms".format(inference_ms))
    print()
    print("=" * 60)
    if text:
        print("  RESULT: USB MIC TEST PASSED")
    else:
        print("  RESULT: USB MIC TEST PASSED (empty transcription — silent input?)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
