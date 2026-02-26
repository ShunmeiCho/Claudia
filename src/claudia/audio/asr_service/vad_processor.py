#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VAD State Machine + Emergency Fast Detector

State transitions: SILENCE -> SPEECH_START -> SPEECH_CONTINUE -> SPEECH_END -> SILENCE

- silero-vad (CPU) detects voice activity
- At 300ms triggers Emergency fast detector (short segment ASR -> keyword matching)
- After speech ends, returns accumulated audio for full ASR transcription

Dependencies:
- silero-vad: loaded via torch.hub, CPU inference
- ring_buffer: ring audio buffer
- emergency_keywords: keyword list (implemented by ipc-protocol agent)
"""

import enum
import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, List, Optional, Protocol, Tuple

from .ring_buffer import RingBuffer, BYTES_PER_MS

logger = logging.getLogger("claudia.asr.vad")


# ======================================================================
# VAD State Enum
# ======================================================================

class VADState(enum.Enum):
    """VAD finite state machine states"""
    SILENCE = "silence"
    SPEECH_START = "speech_start"
    SPEECH_CONTINUE = "speech_continue"
    SPEECH_END = "speech_end"


# ======================================================================
# VAD Configuration Parameters (plan section 1.6)
# ======================================================================

@dataclass(frozen=True)
class VADConfig:
    """VAD parameter configuration, corresponding to plan section 1.6"""
    threshold: float = 0.45
    min_speech_ms: int = 250
    max_speech_ms: int = 15000
    silence_padding_ms: int = 300
    pre_speech_buffer_ms: int = 300
    emergency_check_ms: int = 300
    sample_rate: int = 16000


# ======================================================================
# VAD Events
# ======================================================================

@dataclass
class VADEvent:
    """Event emitted by the VAD state machine"""
    event_type: str          # "vad_start" | "vad_end" | "emergency" | "transcript_request"
    utterance_id: str = ""
    duration_ms: int = 0
    audio_data: bytes = b""  # Contains complete speech segment audio on vad_end
    keyword: str = ""        # Matched keyword on emergency
    confidence: float = 0.0  # Confidence from quick_transcribe on emergency


# ======================================================================
# ASR Quick Transcription Interface (injected, avoids circular dependency)
# ======================================================================

class QuickTranscriber(Protocol):
    """Short segment ASR transcriber interface, implementation injected by asr_server"""
    def quick_transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        """Perform quick transcription on a short audio segment.

        Returns
        -------
        (text, confidence) : Tuple[str, float]
            Transcription text and confidence. On failure: text="" confidence=0.0
        """
        ...


class NullTranscriber:
    """Null implementation, used in mock mode or when ASR is unavailable"""
    def quick_transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        return ("", 0.0)


# ======================================================================
# Utterance ID Generator
# ======================================================================

class UtteranceIDGenerator:
    """Generates utterance IDs in "utt_YYYYMMDD_HHMMSS_NNN" format"""

    def __init__(self) -> None:
        self._counter = 0
        self._last_date = ""

    def next_id(self) -> str:
        now = datetime.now()
        date_str = now.strftime("%Y%m%d_%H%M%S")
        # Counter increments within each second; resets on second boundary
        if date_str != self._last_date:
            self._counter = 0
            self._last_date = date_str
        self._counter += 1
        return f"utt_{date_str}_{self._counter:03d}"


# ======================================================================
# VAD Processor
# ======================================================================

class VADProcessor:
    """silero-vad voice activity detection + Emergency fast detector

    Parameters
    ----------
    ring_buffer : RingBuffer
        Shared ring audio buffer reference
    event_callback : callable
        VAD event callback async def callback(event: VADEvent) -> None
    quick_transcriber : Optional[QuickTranscriber]
        Short segment ASR interface. When None, emergency detector only does fallback (no ASR text)
    config : VADConfig
        VAD parameters
    mock : bool
        When True, does not load silero-vad model, uses simple energy threshold detection
    """

    def __init__(
        self,
        ring_buffer: RingBuffer,
        event_callback: Callable,
        quick_transcriber: Optional[Any] = None,
        config: Optional[VADConfig] = None,
        mock: bool = False,
    ) -> None:
        self._ring = ring_buffer
        self._callback = event_callback
        self._transcriber = quick_transcriber or NullTranscriber()
        self._config = config or VADConfig()
        self._mock = mock

        # Thread safety: process_frame() runs in executor thread pool,
        # reset() runs from event loop. Lock protects all shared state.
        self._state_lock = threading.Lock()

        # State machine
        self._state = VADState.SILENCE
        self._speech_start_ms: float = 0.0    # Speech start monotonic time (ms)
        self._silence_start_ms: float = 0.0   # Silence start monotonic time (ms)
        self._accumulated_ms: int = 0          # Accumulated milliseconds for current speech segment
        self._emergency_checked = False
        self._current_utterance_id = ""

        # Speech segment audio buffer (collects complete speech segment for ASR)
        self._speech_audio_chunks: List[bytes] = []

        # silero-vad model
        self._vad_model = None
        if not mock:
            self._load_vad_model()

        # Utterance ID generator
        self._id_gen = UtteranceIDGenerator()

        # Emergency keywords (lazy import, ipc-protocol agent creates in parallel)
        self._emergency_keywords: List[str] = []
        self._load_emergency_keywords()

        logger.info("VAD processor initialized (mock=%s, threshold=%.2f)",
                     mock, self._config.threshold)

    # ------------------------------------------------------------------
    # Model Loading
    # ------------------------------------------------------------------

    def _load_vad_model(self) -> None:
        """Load silero-vad model (CPU)"""
        try:
            import torch
            model, utils = torch.hub.load(
                repo_or_dir="snakers4/silero-vad",
                model="silero_vad",
                force_reload=False,
                trust_repo=True,
            )
            self._vad_model = model
            self._get_speech_prob = utils[0]  # get_speech_timestamps not needed, use model directly
            logger.info("silero-vad model loaded (CPU)")
        except Exception as e:
            logger.warning("silero-vad loading failed, falling back to energy detection: %s", e)
            self._mock = True

    def _load_emergency_keywords(self) -> None:
        """Load emergency keyword list"""
        try:
            from .emergency_keywords import EMERGENCY_KEYWORDS_TEXT
            self._emergency_keywords = list(EMERGENCY_KEYWORDS_TEXT)
            logger.info("Emergency keywords loaded (%d items)", len(self._emergency_keywords))
        except ImportError:
            # ipc-protocol agent may not have created the file yet
            logger.warning("emergency_keywords module not found, using built-in fallback list")
            self._emergency_keywords = [
                "とまれ", "とめて", "とまって", "やめて", "ストップ", "stop",
                "きんきゅうていし", "止まれ", "止めて", "緊急停止",
            ]

    # ------------------------------------------------------------------
    # Core: Process Audio Frame
    # ------------------------------------------------------------------

    def process_frame(self, frame: bytes) -> List[VADEvent]:
        """Process one frame of PCM audio data (typically 30ms / 480 samples).

        Returns a list of all VAD events produced by this frame. Caller is responsible for dispatching events.

        Parameters
        ----------
        frame : bytes
            16kHz, 16-bit, mono PCM data (typically 960 bytes = 30ms)

        Returns
        -------
        List[VADEvent]
            Events produced by this frame (may be empty)
        """
        frame_ms = len(frame) // BYTES_PER_MS
        # Speech detection does not access shared state — run outside lock
        is_speech = self._detect_speech(frame)
        now_ms = time.monotonic() * 1000

        events: List[VADEvent] = []

        with self._state_lock:
            if self._state == VADState.SILENCE:
                if is_speech:
                    self._transition_to_speech_start(now_ms)
                    self._speech_audio_chunks = []
                    # Include pre-speech buffer
                    pre_audio = self._ring.read_last(self._config.pre_speech_buffer_ms)
                    if pre_audio:
                        self._speech_audio_chunks.append(pre_audio)
                    self._speech_audio_chunks.append(frame)
                    self._current_utterance_id = self._id_gen.next_id()
                    events.append(VADEvent(
                        event_type="vad_start",
                        utterance_id=self._current_utterance_id,
                    ))

            elif self._state in (VADState.SPEECH_START, VADState.SPEECH_CONTINUE):
                self._speech_audio_chunks.append(frame)
                self._accumulated_ms += frame_ms

                if is_speech:
                    self._silence_start_ms = 0.0
                    if self._state == VADState.SPEECH_START:
                        self._state = VADState.SPEECH_CONTINUE

                    # Emergency fast detector: triggers once at 300ms
                    if (self._accumulated_ms >= self._config.emergency_check_ms
                            and not self._emergency_checked):
                        emergency_events = self._run_emergency_check()
                        events.extend(emergency_events)
                        self._emergency_checked = True

                    # Maximum speech duration protection
                    if self._accumulated_ms >= self._config.max_speech_ms:
                        events.extend(self._end_speech(now_ms, forced=True))

                else:
                    # Silence frame
                    if self._silence_start_ms == 0.0:
                        self._silence_start_ms = now_ms

                    silence_duration = now_ms - self._silence_start_ms
                    if silence_duration >= self._config.silence_padding_ms:
                        # Silence exceeded threshold, speech segment ended
                        if self._accumulated_ms >= self._config.min_speech_ms:
                            events.extend(self._end_speech(now_ms, forced=False))
                        else:
                            # Too short, discard (likely noise)
                            logger.info(
                                "Short utterance discarded: %dms < %dms (utt=%s)",
                                self._accumulated_ms,
                                self._config.min_speech_ms,
                                self._current_utterance_id,
                            )
                            self._reset_state()

        return events

    # ------------------------------------------------------------------
    # State Transitions
    # ------------------------------------------------------------------

    def _transition_to_speech_start(self, now_ms: float) -> None:
        """SILENCE -> SPEECH_START"""
        self._state = VADState.SPEECH_START
        self._speech_start_ms = now_ms
        self._silence_start_ms = 0.0
        self._accumulated_ms = 0
        self._emergency_checked = False

    def _end_speech(self, now_ms: float, forced: bool) -> List[VADEvent]:
        """SPEECH_CONTINUE -> SPEECH_END -> SILENCE, returns end events"""
        self._state = VADState.SPEECH_END

        duration_ms = self._accumulated_ms
        audio_data = b"".join(self._speech_audio_chunks)

        if forced:
            logger.info("Max speech duration (%dms) forced end for utterance %s",
                        self._config.max_speech_ms, self._current_utterance_id)

        events = [
            VADEvent(
                event_type="vad_end",
                utterance_id=self._current_utterance_id,
                duration_ms=duration_ms,
            ),
            VADEvent(
                event_type="transcript_request",
                utterance_id=self._current_utterance_id,
                duration_ms=duration_ms,
                audio_data=audio_data,
            ),
        ]

        self._reset_state()
        return events

    def _reset_state(self) -> None:
        """Reset state machine to SILENCE"""
        self._state = VADState.SILENCE
        self._speech_start_ms = 0.0
        self._silence_start_ms = 0.0
        self._accumulated_ms = 0
        self._emergency_checked = False
        self._speech_audio_chunks = []
        self._current_utterance_id = ""

    # ------------------------------------------------------------------
    # Emergency Fast Detector
    # ------------------------------------------------------------------

    def _run_emergency_check(self) -> List[VADEvent]:
        """Run emergency dual-layer detection at 300ms.

        Layer 2 (main path): quick_transcribe -> keyword matching
        Layer 1 (fallback): when quick_transcribe fails, quick_text="", matching naturally misses
        """
        events: List[VADEvent] = []

        # Read the most recent emergency_check_ms milliseconds of audio
        audio_chunk = self._ring.read_last(self._config.emergency_check_ms)
        if not audio_chunk:
            return events

        # Layer 2: Short segment ASR quick_transcribe
        quick_text = ""
        confidence = 0.0
        try:
            quick_text, confidence = self._transcriber.quick_transcribe(audio_chunk)
        except (TimeoutError, RuntimeError) as e:
            # Layer 1 fallback: ASR failed, quick_text="" -> keyword matching naturally misses
            logger.warning("Emergency quick_transcribe failed (Layer 1 fallback): %s", e)
            quick_text = ""
            confidence = 0.0

        if not quick_text:
            return events

        # Normalize and match keywords
        normalized = quick_text.strip().lower()
        matched_keyword = ""
        for kw in self._emergency_keywords:
            if kw in normalized:
                matched_keyword = kw
                break

        if matched_keyword:
            logger.warning("Emergency keyword detected: '%s' (original: '%s', conf=%.2f)",
                           matched_keyword, quick_text, confidence)
            events.append(VADEvent(
                event_type="emergency",
                utterance_id=self._current_utterance_id,
                keyword=matched_keyword,
                confidence=confidence,
            ))

        return events

    # ------------------------------------------------------------------
    # Speech Detection (silero-vad or energy fallback)
    # ------------------------------------------------------------------

    def _detect_speech(self, frame: bytes) -> bool:
        """Detect whether a single frame is speech.

        Normal mode: silero-vad model inference
        Mock/fallback mode: simple energy threshold
        """
        if self._mock or self._vad_model is None:
            return self._detect_speech_energy(frame)
        return self._detect_speech_silero(frame)

    def _detect_speech_silero(self, frame: bytes) -> bool:
        """silero-vad model inference"""
        try:
            import torch
            import numpy as np

            # PCM 16-bit -> float32 tensor
            audio_np = np.frombuffer(frame, dtype=np.int16).astype(np.float32) / 32768.0
            audio_tensor = torch.from_numpy(audio_np)

            # silero-vad requires specific frame lengths: 256/512/768 samples @16kHz
            # 30ms = 480 samples -> not a standard frame length, needs padding to 512
            if len(audio_tensor) < 512:
                audio_tensor = torch.nn.functional.pad(
                    audio_tensor, (0, 512 - len(audio_tensor))
                )

            speech_prob = self._vad_model(audio_tensor, self._config.sample_rate).item()
            return speech_prob >= self._config.threshold

        except Exception as e:
            logger.debug("silero-vad inference error, falling back to energy detection: %s", e)
            return self._detect_speech_energy(frame)

    def _detect_speech_energy(self, frame: bytes) -> bool:
        """Simple energy threshold speech detection (mock/fallback mode)"""
        if len(frame) < 2:
            return False
        # Calculate RMS energy
        import struct
        n_samples = len(frame) // 2
        samples = struct.unpack(f"<{n_samples}h", frame[:n_samples * 2])
        rms = (sum(s * s for s in samples) / n_samples) ** 0.5
        # Threshold 300: corresponds to ~-40dBFS, above typical ambient noise
        return rms > 300

    # ------------------------------------------------------------------
    # Public Properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> VADState:
        """Current VAD state"""
        with self._state_lock:
            return self._state

    @property
    def is_speaking(self) -> bool:
        """Whether currently in a speech segment"""
        with self._state_lock:
            return self._state in (VADState.SPEECH_START, VADState.SPEECH_CONTINUE)

    def reset(self) -> None:
        """External forced reset (e.g., when TTS echo gate is enabled)"""
        with self._state_lock:
            if self._state != VADState.SILENCE:
                logger.info("VAD external reset (current state: %s)", self._state.value)
            self._reset_state()
        # Reset silero-vad internal state (outside lock — model reset is independent)
        if self._vad_model is not None:
            try:
                self._vad_model.reset_states()
            except Exception:
                pass
