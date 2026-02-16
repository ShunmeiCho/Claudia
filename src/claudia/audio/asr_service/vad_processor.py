#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VAD 状态机 + Emergency 快速器

状态流转: SILENCE → SPEECH_START → SPEECH_CONTINUE → SPEECH_END → SILENCE

- silero-vad (CPU) 检测语音活动
- 300ms 处触发 Emergency 快速器（短片段 ASR → 关键词匹配）
- 语音结束后返回累积音频供完整 ASR 转写

依赖:
- silero-vad: torch.hub 加载，CPU 推理
- ring_buffer: 环形音频缓冲
- emergency_keywords: 关键词列表（ipc-protocol agent 实现）
"""

import enum
import logging
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, List, Optional, Protocol, Tuple

from .ring_buffer import RingBuffer, BYTES_PER_MS

logger = logging.getLogger("claudia.asr.vad")


# ======================================================================
# VAD 状态枚举
# ======================================================================

class VADState(enum.Enum):
    """VAD 有限状态机状态"""
    SILENCE = "silence"
    SPEECH_START = "speech_start"
    SPEECH_CONTINUE = "speech_continue"
    SPEECH_END = "speech_end"


# ======================================================================
# VAD 配置参数（plan section 1.6）
# ======================================================================

@dataclass(frozen=True)
class VADConfig:
    """VAD 参数配置，对应 plan section 1.6"""
    threshold: float = 0.5
    min_speech_ms: int = 300
    max_speech_ms: int = 15000
    silence_padding_ms: int = 500
    pre_speech_buffer_ms: int = 300
    emergency_check_ms: int = 300
    sample_rate: int = 16000


# ======================================================================
# VAD 事件
# ======================================================================

@dataclass
class VADEvent:
    """VAD 状态机发出的事件"""
    event_type: str          # "vad_start" | "vad_end" | "emergency" | "transcript_request"
    utterance_id: str = ""
    duration_ms: int = 0
    audio_data: bytes = b""  # vad_end 时包含完整语音段音频
    keyword: str = ""        # emergency 时匹配到的关键词
    confidence: float = 0.0  # emergency 时的置信度（quick_transcribe 提供）


# ======================================================================
# ASR 快速转写接口（注入，避免循环依赖）
# ======================================================================

class QuickTranscriber(Protocol):
    """短片段 ASR 转写器接口，由 asr_server 注入实现"""
    def quick_transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        """对短音频片段做快速转写。

        Returns
        -------
        (text, confidence) : Tuple[str, float]
            转写文本和置信度。失败时 text="" confidence=0.0
        """
        ...


class NullTranscriber:
    """空实现，mock 模式或 ASR 不可用时使用"""
    def quick_transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        return ("", 0.0)


# ======================================================================
# utterance ID 生成器
# ======================================================================

class UtteranceIDGenerator:
    """生成 "utt_YYYYMMDD_HHMMSS_NNN" 格式的 utterance ID"""

    def __init__(self) -> None:
        self._counter = 0
        self._last_date = ""

    def next_id(self) -> str:
        now = datetime.now()
        date_str = now.strftime("%Y%m%d_%H%M%S")
        # 每秒内计数器递增；跨秒重置
        if date_str != self._last_date:
            self._counter = 0
            self._last_date = date_str
        self._counter += 1
        return f"utt_{date_str}_{self._counter:03d}"


# ======================================================================
# VAD 处理器
# ======================================================================

class VADProcessor:
    """silero-vad 语音活动检测 + Emergency 快速器

    Parameters
    ----------
    ring_buffer : RingBuffer
        共享环形音频缓冲区引用
    event_callback : callable
        VAD 事件回调 async def callback(event: VADEvent) -> None
    quick_transcriber : Optional[QuickTranscriber]
        短片段 ASR 接口。None 时 emergency 快速器仅做 fallback（无 ASR 文本）
    config : VADConfig
        VAD 参数
    mock : bool
        True 时不加载 silero-vad 模型，使用简单能量阈值检测
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

        # 状态机
        self._state = VADState.SILENCE
        self._speech_start_ms: float = 0.0    # 语音开始的 monotonic 时间（ms）
        self._silence_start_ms: float = 0.0   # 进入静音的 monotonic 时间（ms）
        self._accumulated_ms: int = 0          # 当前语音段累积毫秒
        self._emergency_checked = False
        self._current_utterance_id = ""

        # 语音段音频缓冲（收集完整语音段用于 ASR）
        self._speech_audio_chunks: List[bytes] = []

        # silero-vad 模型
        self._vad_model = None
        if not mock:
            self._load_vad_model()

        # utterance ID 生成器
        self._id_gen = UtteranceIDGenerator()

        # Emergency 关键词（延迟导入，ipc-protocol agent 并行创建）
        self._emergency_keywords: List[str] = []
        self._load_emergency_keywords()

        logger.info("🎤 VAD 处理器初始化完成 (mock=%s, threshold=%.2f)",
                     mock, self._config.threshold)

    # ------------------------------------------------------------------
    # 模型加载
    # ------------------------------------------------------------------

    def _load_vad_model(self) -> None:
        """加载 silero-vad 模型（CPU）"""
        try:
            import torch
            model, utils = torch.hub.load(
                repo_or_dir="snakers4/silero-vad",
                model="silero_vad",
                force_reload=False,
                trust_repo=True,
            )
            self._vad_model = model
            self._get_speech_prob = utils[0]  # get_speech_timestamps 不需要，直接用模型
            logger.info("🎤 silero-vad 模型加载完成 (CPU)")
        except Exception as e:
            logger.warning("⚠️ silero-vad 加载失败，降级为能量检测: %s", e)
            self._mock = True

    def _load_emergency_keywords(self) -> None:
        """加载 Emergency 关键词列表"""
        try:
            from .emergency_keywords import EMERGENCY_KEYWORDS_TEXT
            self._emergency_keywords = list(EMERGENCY_KEYWORDS_TEXT)
            logger.info("🚨 Emergency 关键词加载完成 (%d 个)", len(self._emergency_keywords))
        except ImportError:
            # ipc-protocol agent 可能还未创建该文件
            logger.warning("⚠️ emergency_keywords 模块未找到，使用内置 fallback 列表")
            self._emergency_keywords = [
                "とまれ", "とめて", "とまって", "やめて", "ストップ", "stop",
                "きんきゅうていし", "止まれ", "止めて", "緊急停止",
            ]

    # ------------------------------------------------------------------
    # 核心: 处理音频帧
    # ------------------------------------------------------------------

    def process_frame(self, frame: bytes) -> List[VADEvent]:
        """处理一帧 PCM 音频数据（通常 30ms / 480 samples）。

        返回本帧产生的所有 VAD 事件列表。调用者负责发送事件。

        Parameters
        ----------
        frame : bytes
            16kHz, 16-bit, mono PCM 数据（通常 960 字节 = 30ms）

        Returns
        -------
        List[VADEvent]
            本帧产生的事件（可能为空）
        """
        frame_ms = len(frame) // BYTES_PER_MS
        is_speech = self._detect_speech(frame)
        now_ms = time.monotonic() * 1000

        events: List[VADEvent] = []

        if self._state == VADState.SILENCE:
            if is_speech:
                self._transition_to_speech_start(now_ms)
                self._speech_audio_chunks = []
                # 包含预语音缓冲
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

                # Emergency 快速器: 300ms 时触发一次
                if (self._accumulated_ms >= self._config.emergency_check_ms
                        and not self._emergency_checked):
                    emergency_events = self._run_emergency_check()
                    events.extend(emergency_events)
                    self._emergency_checked = True

                # 最大语音时长保护
                if self._accumulated_ms >= self._config.max_speech_ms:
                    events.extend(self._end_speech(now_ms, forced=True))

            else:
                # 静音帧
                if self._silence_start_ms == 0.0:
                    self._silence_start_ms = now_ms

                silence_duration = now_ms - self._silence_start_ms
                if silence_duration >= self._config.silence_padding_ms:
                    # 静音持续超过阈值，语音段结束
                    if self._accumulated_ms >= self._config.min_speech_ms:
                        events.extend(self._end_speech(now_ms, forced=False))
                    else:
                        # 太短，丢弃（可能是噪声）
                        self._reset_state()

        return events

    # ------------------------------------------------------------------
    # 状态转换
    # ------------------------------------------------------------------

    def _transition_to_speech_start(self, now_ms: float) -> None:
        """SILENCE → SPEECH_START"""
        self._state = VADState.SPEECH_START
        self._speech_start_ms = now_ms
        self._silence_start_ms = 0.0
        self._accumulated_ms = 0
        self._emergency_checked = False

    def _end_speech(self, now_ms: float, forced: bool) -> List[VADEvent]:
        """SPEECH_CONTINUE → SPEECH_END → SILENCE，返回结束事件"""
        self._state = VADState.SPEECH_END

        duration_ms = self._accumulated_ms
        audio_data = b"".join(self._speech_audio_chunks)

        if forced:
            logger.info("⏱️ 最大语音时长 (%dms) 强制结束 utterance %s",
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
        """重置状态机到 SILENCE"""
        self._state = VADState.SILENCE
        self._speech_start_ms = 0.0
        self._silence_start_ms = 0.0
        self._accumulated_ms = 0
        self._emergency_checked = False
        self._speech_audio_chunks = []
        self._current_utterance_id = ""

    # ------------------------------------------------------------------
    # Emergency 快速器
    # ------------------------------------------------------------------

    def _run_emergency_check(self) -> List[VADEvent]:
        """在 300ms 处运行 Emergency 双层检测。

        Layer 2 (主路径): quick_transcribe → 关键词匹配
        Layer 1 (fallback): quick_transcribe 失败时 quick_text=""，匹配自然 miss
        """
        events: List[VADEvent] = []

        # 读取最近 emergency_check_ms 毫秒音频
        audio_chunk = self._ring.read_last(self._config.emergency_check_ms)
        if not audio_chunk:
            return events

        # Layer 2: 短片段 ASR quick_transcribe
        quick_text = ""
        confidence = 0.0
        try:
            quick_text, confidence = self._transcriber.quick_transcribe(audio_chunk)
        except (TimeoutError, RuntimeError) as e:
            # Layer 1 fallback: ASR 失败，quick_text="" → 关键词匹配自然 miss
            logger.warning("⚠️ Emergency quick_transcribe 失败 (Layer 1 fallback): %s", e)
            quick_text = ""
            confidence = 0.0

        if not quick_text:
            return events

        # 归一化并匹配关键词
        normalized = quick_text.strip().lower()
        matched_keyword = ""
        for kw in self._emergency_keywords:
            if kw in normalized:
                matched_keyword = kw
                break

        if matched_keyword:
            logger.warning("🚨 Emergency 关键词检出: '%s' (原文: '%s', conf=%.2f)",
                           matched_keyword, quick_text, confidence)
            events.append(VADEvent(
                event_type="emergency",
                utterance_id=self._current_utterance_id,
                keyword=matched_keyword,
                confidence=confidence,
            ))

        return events

    # ------------------------------------------------------------------
    # 语音检测（silero-vad 或能量 fallback）
    # ------------------------------------------------------------------

    def _detect_speech(self, frame: bytes) -> bool:
        """检测单帧是否为语音。

        正常模式: silero-vad 模型推理
        Mock/降级模式: 简单能量阈值
        """
        if self._mock or self._vad_model is None:
            return self._detect_speech_energy(frame)
        return self._detect_speech_silero(frame)

    def _detect_speech_silero(self, frame: bytes) -> bool:
        """silero-vad 模型推理"""
        try:
            import torch
            import numpy as np

            # PCM 16-bit → float32 tensor
            audio_np = np.frombuffer(frame, dtype=np.int16).astype(np.float32) / 32768.0
            audio_tensor = torch.from_numpy(audio_np)

            # silero-vad 需要特定帧长: 256/512/768 samples @16kHz
            # 30ms = 480 samples → 不是标准帧长，需要 padding 到 512
            if len(audio_tensor) < 512:
                audio_tensor = torch.nn.functional.pad(
                    audio_tensor, (0, 512 - len(audio_tensor))
                )

            speech_prob = self._vad_model(audio_tensor, self._config.sample_rate).item()
            return speech_prob >= self._config.threshold

        except Exception as e:
            logger.debug("silero-vad 推理异常，降级能量检测: %s", e)
            return self._detect_speech_energy(frame)

    def _detect_speech_energy(self, frame: bytes) -> bool:
        """简单能量阈值语音检测（mock/fallback 模式）"""
        if len(frame) < 2:
            return False
        # 计算 RMS 能量
        import struct
        n_samples = len(frame) // 2
        samples = struct.unpack(f"<{n_samples}h", frame[:n_samples * 2])
        rms = (sum(s * s for s in samples) / n_samples) ** 0.5
        # 阈值 300: 对应 ~-40dBFS，一般环境噪声以上
        return rms > 300

    # ------------------------------------------------------------------
    # 公开属性
    # ------------------------------------------------------------------

    @property
    def state(self) -> VADState:
        """当前 VAD 状态"""
        return self._state

    @property
    def is_speaking(self) -> bool:
        """是否在语音段中"""
        return self._state in (VADState.SPEECH_START, VADState.SPEECH_CONTINUE)

    def reset(self) -> None:
        """外部强制重置（例如 TTS 回声门控开启时）"""
        if self._state != VADState.SILENCE:
            logger.info("🔇 VAD 外部重置 (当前状态: %s)", self._state.value)
        self._reset_state()
        # 重置 silero-vad 内部状态
        if self._vad_model is not None:
            try:
                self._vad_model.reset_states()
            except Exception:
                pass
