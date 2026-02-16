#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASR 服务主入口 — 独立 Python 3.11 子进程

3 路单向 UDS:
- /tmp/claudia_audio.sock  (接收 PCM 音频流)
- /tmp/claudia_asr_result.sock (发送 JSON Lines: transcript/emergency/heartbeat)
- /tmp/claudia_asr_ctrl.sock   (接收 JSON Lines: tts_start/tts_end/shutdown)

启动后加载 Qwen3-ASR-0.6B 模型，发送 handshake ready 消息。
支持 --mock 标志或 ASR_MOCK=1 环境变量，mock 模式不加载 CUDA 模型。
"""

import argparse
import asyncio
import json
import logging
import os
import signal
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

from .ring_buffer import RingBuffer, BYTES_PER_MS
from .vad_processor import VADProcessor, VADConfig, VADEvent

logger = logging.getLogger("claudia.asr.server")

# ======================================================================
# 常量
# ======================================================================

AUDIO_SOCKET = "/tmp/claudia_audio.sock"
RESULT_SOCKET = "/tmp/claudia_asr_result.sock"
CTRL_SOCKET = "/tmp/claudia_asr_ctrl.sock"

HEARTBEAT_INTERVAL_S = 5
TTS_GATE_TIMEOUT_S = 30
PROTO_VERSION = "1.0"

# PCM 参数: 16kHz, 16-bit, mono
SAMPLE_RATE = 16000
FRAME_MS = 30
FRAME_BYTES = FRAME_MS * BYTES_PER_MS  # 960 bytes = 30ms


# ======================================================================
# ASR 模型包装器
# ======================================================================

class ASRModelWrapper:
    """Qwen3-ASR-0.6B 模型包装器

    封装模型加载和推理，支持 mock 模式。
    """

    def __init__(self, mock: bool = False) -> None:
        self._mock = mock
        self._model: Optional[Any] = None
        self._processor: Optional[Any] = None
        self._vram_mb: int = 0

    def load(self) -> None:
        """加载 ASR 模型到 GPU"""
        if self._mock:
            logger.info("🧪 ASR mock 模式，跳过模型加载")
            self._vram_mb = 0
            return

        try:
            from qwen_asr import Qwen3ASRModel
            import torch

            model_name = os.getenv("CLAUDIA_ASR_MODEL", "Qwen/Qwen3-ASR-0.6B")
            device = os.getenv("CLAUDIA_ASR_DEVICE", "cuda:0")
            dtype_str = os.getenv("CLAUDIA_ASR_DTYPE", "bfloat16")
            dtype = getattr(torch, dtype_str, torch.bfloat16)

            logger.info("🧠 ASR 模型加载中: %s (device=%s, dtype=%s)",
                        model_name, device, dtype_str)

            self._model = Qwen3ASRModel.from_pretrained(
                model_name,
                device=device,
                dtype=dtype,
            )

            # 估算 VRAM 占用
            if torch.cuda.is_available():
                self._vram_mb = int(torch.cuda.memory_allocated() / 1024 / 1024)
            else:
                self._vram_mb = 600  # 估算值

            logger.info("🧠 ASR 模型加载完成 (VRAM ~%dMB)", self._vram_mb)

        except Exception as e:
            logger.error("❌ ASR 模型加载失败: %s", e)
            logger.warning("⚠️ 降级为 mock 模式")
            self._mock = True
            self._vram_mb = 0

    def transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        """完整语音段 ASR 转写

        Parameters
        ----------
        audio_data : bytes
            16kHz 16-bit mono PCM

        Returns
        -------
        (text, confidence) : Tuple[str, float]
        """
        if self._mock:
            return ("mock転写結果", 0.99)

        try:
            import numpy as np
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            result = self._model.transcribe(
                audio_np,
                language="ja",
                sample_rate=SAMPLE_RATE,
            )

            text = result.get("text", "").strip()
            confidence = result.get("confidence", 0.0)
            return (text, confidence)

        except Exception as e:
            logger.error("❌ ASR 推理失败: %s", e)
            return ("", 0.0)

    def quick_transcribe(self, audio_data: bytes) -> Tuple[str, float]:
        """短片段快速转写（Emergency 快速器用）"""
        return self.transcribe(audio_data)

    @property
    def vram_mb(self) -> int:
        return self._vram_mb

    @property
    def is_mock(self) -> bool:
        return self._mock


# ======================================================================
# ASR 服务器
# ======================================================================

class ASRServer:
    """ASR 主服务

    管理 3 路 UDS、VAD 处理、ASR 推理、TTS 回声门控。
    """

    def __init__(self, mock: bool = False) -> None:
        self._mock = mock
        self._running = False

        # ASR 模型
        self._asr = ASRModelWrapper(mock=mock)

        # 音频环形缓冲区
        self._ring = RingBuffer()

        # 结果写入器（result socket 连接后赋值）
        self._result_writer: Optional[asyncio.StreamWriter] = None
        self._result_lock = asyncio.Lock()

        # TTS 回声门控
        self._tts_gate = False
        self._tts_gate_timer: Optional[asyncio.TimerHandle] = None

        # VAD 处理器（模型加载后初始化）
        self._vad: Optional[VADProcessor] = None

        # 心跳任务
        self._heartbeat_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]

        # 服务器引用（关闭时用）
        self._audio_server: Optional[asyncio.AbstractServer] = None
        self._ctrl_server: Optional[asyncio.AbstractServer] = None
        self._result_server: Optional[asyncio.AbstractServer] = None

    # ------------------------------------------------------------------
    # 启动 / 关闭
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """启动 ASR 服务"""
        logger.info("🚀 ASR 服务启动中 (mock=%s)...", self._mock)

        # 1. 加载 ASR 模型
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._asr.load)

        # 2. 初始化 VAD
        self._vad = VADProcessor(
            ring_buffer=self._ring,
            event_callback=self._handle_vad_event,
            quick_transcriber=self._asr,
            mock=self._mock,
        )

        # 3. 清理旧 socket 文件
        for sock_path in (AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET):
            if os.path.exists(sock_path):
                os.unlink(sock_path)

        # 4. 启动 3 路 UDS 服务器
        self._result_server = await asyncio.start_unix_server(
            self._handle_result_connection, path=RESULT_SOCKET,
        )
        self._audio_server = await asyncio.start_unix_server(
            self._handle_audio_connection, path=AUDIO_SOCKET,
        )
        self._ctrl_server = await asyncio.start_unix_server(
            self._handle_ctrl_connection, path=CTRL_SOCKET,
        )

        self._running = True

        # 5. 启动心跳
        self._heartbeat_task = asyncio.ensure_future(self._heartbeat_loop())

        logger.info("✅ ASR 服务就绪 (audio=%s, result=%s, ctrl=%s)",
                     AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET)

    async def shutdown(self) -> None:
        """优雅关闭"""
        if not self._running:
            return

        logger.info("🛑 ASR 服务关闭中...")
        self._running = False

        # 取消心跳
        if self._heartbeat_task and not self._heartbeat_task.done():
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass

        # 取消 TTS 门控定时器
        if self._tts_gate_timer:
            self._tts_gate_timer.cancel()

        # 关闭服务器
        for server in (self._audio_server, self._result_server, self._ctrl_server):
            if server:
                server.close()
                await server.wait_closed()

        # 关闭结果写入器
        if self._result_writer:
            try:
                self._result_writer.close()
                await self._result_writer.wait_closed()
            except Exception:
                pass

        # 清理 socket 文件
        for sock_path in (AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET):
            if os.path.exists(sock_path):
                try:
                    os.unlink(sock_path)
                except OSError:
                    pass

        logger.info("✅ ASR 服务已关闭")

    # ------------------------------------------------------------------
    # UDS 连接处理
    # ------------------------------------------------------------------

    async def _handle_result_connection(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Result socket 连接处理: 保存 writer 引用，发送 handshake"""
        logger.info("📡 Result socket 客户端已连接")
        self._result_writer = writer

        # 发送 handshake ready 消息
        model_name = "mock" if self._mock else "qwen3-asr-0.6b"
        await self._emit_result({
            "type": "ready",
            "model": model_name,
            "vram_mb": self._asr.vram_mb,
            "proto_version": PROTO_VERSION,
        })

        # 保持连接直到关闭
        try:
            while self._running:
                # result socket 是单向 ASR→Main，不需要读
                # 但需要检测断开
                data = await reader.read(1)
                if not data:
                    break
                await asyncio.sleep(0.1)
        except (asyncio.CancelledError, ConnectionError):
            pass
        finally:
            logger.info("📡 Result socket 客户端断开")
            self._result_writer = None

    async def _handle_audio_connection(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Audio socket 连接处理: 接收 PCM 流 → ring buffer → VAD"""
        logger.info("🎙️ Audio socket 客户端已连接")

        try:
            while self._running:
                # 读取一帧 PCM 数据 (30ms = 960 bytes)
                data = await reader.readexactly(FRAME_BYTES)
                if not data:
                    break

                # 写入环形缓冲区
                self._ring.write(data)

                # TTS 回声门控: 播放期间不做 VAD
                if self._tts_gate:
                    continue

                # VAD 处理
                if self._vad:
                    events = self._vad.process_frame(data)
                    for event in events:
                        await self._handle_vad_event(event)

        except asyncio.IncompleteReadError:
            logger.info("🎙️ Audio 流结束 (不完整帧)")
        except (asyncio.CancelledError, ConnectionError):
            pass
        finally:
            logger.info("🎙️ Audio socket 客户端断开")
            writer.close()

    async def _handle_ctrl_connection(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Control socket 连接处理: 接收 JSON Lines 控制消息"""
        logger.info("🎛️ Control socket 客户端已连接")

        try:
            while self._running:
                line = await reader.readline()
                if not line:
                    break

                try:
                    msg = json.loads(line.decode("utf-8").strip())
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    logger.warning("⚠️ 无效控制消息: %s", e)
                    continue

                msg_type = msg.get("type", "")
                await self._handle_ctrl_message(msg_type, msg)

        except (asyncio.CancelledError, ConnectionError):
            pass
        finally:
            logger.info("🎛️ Control socket 客户端断开")
            writer.close()

    # ------------------------------------------------------------------
    # 控制消息处理
    # ------------------------------------------------------------------

    async def _handle_ctrl_message(self, msg_type: str, msg: Dict[str, Any]) -> None:
        """处理控制消息"""
        if msg_type == "tts_start":
            logger.info("🔇 TTS 回声门控: 开启")
            self._tts_gate = True
            # 重置 VAD（丢弃当前语音段）
            if self._vad:
                self._vad.reset()
            # 设置超时保护
            loop = asyncio.get_event_loop()
            if self._tts_gate_timer:
                self._tts_gate_timer.cancel()
            self._tts_gate_timer = loop.call_later(
                TTS_GATE_TIMEOUT_S, self._force_gate_open,
            )

        elif msg_type == "tts_end":
            logger.info("🔊 TTS 回声门控: 关闭")
            self._tts_gate = False
            if self._tts_gate_timer:
                self._tts_gate_timer.cancel()
                self._tts_gate_timer = None

        elif msg_type == "shutdown":
            reason = msg.get("reason", "requested")
            logger.info("🛑 收到 shutdown 控制消息 (reason=%s)", reason)
            await self.shutdown()

        else:
            logger.warning("⚠️ 未知控制消息类型: %s", msg_type)

    def _force_gate_open(self) -> None:
        """TTS 门控超时保护: 30s 后强制恢复 VAD + 审计"""
        logger.warning("⏱️ TTS 回声门控超时 (%ds)，强制恢复", TTS_GATE_TIMEOUT_S)
        self._tts_gate = False
        self._tts_gate_timer = None
        # 异步发送审计事件
        asyncio.ensure_future(self._emit_result({
            "type": "gate_timeout_audit",
            "ts": time.time(),
        }))

    # ------------------------------------------------------------------
    # VAD 事件处理
    # ------------------------------------------------------------------

    async def _handle_vad_event(self, event: VADEvent) -> None:
        """处理 VAD 状态机发出的事件"""
        if event.event_type == "vad_start":
            await self._emit_result({
                "type": "vad_start",
                "utterance_id": event.utterance_id,
            })

        elif event.event_type == "vad_end":
            await self._emit_result({
                "type": "vad_end",
                "duration_ms": event.duration_ms,
                "utterance_id": event.utterance_id,
            })

        elif event.event_type == "emergency":
            logger.warning("🚨 Emergency 事件: keyword='%s' (conf=%.2f, utt=%s)",
                           event.keyword, event.confidence, event.utterance_id)
            await self._emit_result({
                "type": "emergency",
                "keyword": event.keyword,
                "confidence": event.confidence,
                "utterance_id": event.utterance_id,
            })

        elif event.event_type == "transcript_request":
            # 完整语音段 ASR 转写（在线程池中执行，避免阻塞事件循环）
            await self._run_full_transcription(event)

    async def _run_full_transcription(self, event: VADEvent) -> None:
        """完整语音段 ASR 转写"""
        utterance_id = event.utterance_id
        audio_data = event.audio_data
        duration_ms = event.duration_ms

        if not audio_data:
            return

        logger.info("🧠 ASR 转写开始: utt=%s, duration=%dms, audio=%d bytes",
                     utterance_id, duration_ms, len(audio_data))

        start_time = time.monotonic()

        # 在线程池中运行 ASR 推理
        loop = asyncio.get_event_loop()
        text, confidence = await loop.run_in_executor(
            None, self._asr.transcribe, audio_data,
        )

        asr_latency_ms = int((time.monotonic() - start_time) * 1000)

        if text:
            logger.info("📝 ASR 结果: '%s' (conf=%.2f, latency=%dms, utt=%s)",
                         text, confidence, asr_latency_ms, utterance_id)
            await self._emit_result({
                "type": "transcript",
                "text": text,
                "confidence": confidence,
                "duration_ms": duration_ms,
                "asr_latency_ms": asr_latency_ms,
                "utterance_id": utterance_id,
            })
        else:
            logger.warning("⚠️ ASR 转写为空 (utt=%s, latency=%dms)",
                           utterance_id, asr_latency_ms)

    # ------------------------------------------------------------------
    # 结果发送
    # ------------------------------------------------------------------

    async def _emit_result(self, msg: Dict[str, Any]) -> None:
        """通过 result socket 发送 JSON Lines 消息"""
        async with self._result_lock:
            if self._result_writer is None:
                return
            try:
                line = json.dumps(msg, ensure_ascii=False) + "\n"
                self._result_writer.write(line.encode("utf-8"))
                await self._result_writer.drain()
            except (ConnectionError, OSError) as e:
                logger.warning("⚠️ Result socket 写入失败: %s", e)
                self._result_writer = None

    # ------------------------------------------------------------------
    # 心跳
    # ------------------------------------------------------------------

    async def _heartbeat_loop(self) -> None:
        """定时发送心跳消息"""
        try:
            while self._running:
                await asyncio.sleep(HEARTBEAT_INTERVAL_S)
                if self._running:
                    await self._emit_result({
                        "type": "heartbeat",
                        "ts": time.time(),
                    })
        except asyncio.CancelledError:
            pass


# ======================================================================
# 入口
# ======================================================================

def _setup_logging() -> None:
    """配置日志"""
    log_level = os.getenv("CLAUDIA_ASR_LOG_LEVEL", "INFO").upper()
    logging.basicConfig(
        level=getattr(logging, log_level, logging.INFO),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def _parse_args() -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="Claudia ASR Service")
    parser.add_argument(
        "--mock", action="store_true",
        help="Mock 模式: 不加载 CUDA 模型，用于测试",
    )
    return parser.parse_args()


async def _async_main(mock: bool) -> None:
    """异步主函数"""
    server = ASRServer(mock=mock)

    # 信号处理
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, lambda: asyncio.ensure_future(server.shutdown()))

    await server.start()

    # 等待直到关闭
    try:
        while server._running:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        await server.shutdown()


def main() -> None:
    """同步入口"""
    _setup_logging()
    args = _parse_args()
    mock = args.mock or os.getenv("ASR_MOCK", "0") == "1"

    if mock:
        logger.info("🧪 ASR 服务以 mock 模式启动")
    else:
        logger.info("🧠 ASR 服务以 production 模式启动")

    asyncio.run(_async_main(mock))


if __name__ == "__main__":
    main()
