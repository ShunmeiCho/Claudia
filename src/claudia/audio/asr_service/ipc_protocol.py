#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IPC 协议层: 消息工厂 + Unix Domain Socket 工具

3 路单向 Socket 架构:
  - /tmp/claudia_audio.sock      (AudioCapture → ASR, raw PCM)
  - /tmp/claudia_asr_result.sock (ASR → Main, JSON Lines)
  - /tmp/claudia_asr_ctrl.sock   (Main → ASR, JSON Lines)

协议版本: major 相同则兼容 (1.0 和 1.1 互通, 2.0 不兼容)
消息格式: JSON Lines (每行一条 JSON, \\n 分隔)
"""

import asyncio
import json
import logging
import os
import time
from typing import AsyncIterator, Callable, Optional, Tuple

logger = logging.getLogger(__name__)

# === 协议版本 ===
PROTO_VERSION = "1.0"

# === Socket 路径 ===
AUDIO_SOCKET = "/tmp/claudia_audio.sock"
ASR_RESULT_SOCKET = "/tmp/claudia_asr_result.sock"
ASR_CTRL_SOCKET = "/tmp/claudia_asr_ctrl.sock"

# === 消息类型 (Result: ASR → Main) ===
MSG_READY = "ready"
MSG_HEARTBEAT = "heartbeat"
MSG_TRANSCRIPT = "transcript"
MSG_EMERGENCY = "emergency"
MSG_VAD_START = "vad_start"
MSG_VAD_END = "vad_end"
MSG_ERROR = "error"
MSG_GATE_TIMEOUT_AUDIT = "gate_timeout_audit"

# === 控制类型 (Control: Main → ASR) ===
CTRL_TTS_START = "tts_start"
CTRL_TTS_END = "tts_end"
CTRL_SHUTDOWN = "shutdown"


# ============================================================
# 消息工厂 (纯函数, 无副作用)
# ============================================================

def create_handshake(model: str, vram_mb: int) -> dict:
    """ASR サービス起動時の ready メッセージ"""
    return {
        "type": MSG_READY,
        "model": model,
        "vram_mb": vram_mb,
        "proto_version": PROTO_VERSION,
    }


def create_transcript_msg(
    text: str,
    confidence: float,
    duration_ms: int,
    asr_latency_ms: int,
    utterance_id: str,
) -> dict:
    """ASR 転写結果メッセージ"""
    return {
        "type": MSG_TRANSCRIPT,
        "text": text,
        "confidence": confidence,
        "duration_ms": duration_ms,
        "asr_latency_ms": asr_latency_ms,
        "utterance_id": utterance_id,
    }


def create_emergency_msg(
    keyword: str,
    confidence: float,
    utterance_id: str,
) -> dict:
    """緊急停止検出メッセージ"""
    return {
        "type": MSG_EMERGENCY,
        "keyword": keyword,
        "confidence": confidence,
        "utterance_id": utterance_id,
    }


def create_heartbeat() -> dict:
    """ハートビートメッセージ (watchdog 用)"""
    return {
        "type": MSG_HEARTBEAT,
        "ts": time.time(),
    }


def create_vad_event(event_type: str, **kwargs) -> dict:
    """VAD イベントメッセージ (vad_start / vad_end)

    Args:
        event_type: "vad_start" or "vad_end"
        **kwargs: vad_end の場合 duration_ms を含む
    """
    msg: dict = {"type": event_type}
    if event_type == MSG_VAD_END and "duration_ms" in kwargs:
        msg["duration_ms"] = kwargs["duration_ms"]
    return msg


def create_error_msg(msg: str) -> dict:
    """エラーメッセージ"""
    return {
        "type": MSG_ERROR,
        "msg": msg,
    }


def create_gate_timeout_audit() -> dict:
    """TTS ゲートタイムアウト監査メッセージ (30s 保護)"""
    return {
        "type": MSG_GATE_TIMEOUT_AUDIT,
        "ts": time.time(),
    }


# ============================================================
# 协议验证
# ============================================================

def validate_proto_version(version: str) -> bool:
    """プロトコルバージョン互換性チェック

    major バージョンが一致すれば互換とみなす。
    例: "1.0" と "1.1" → True, "1.0" と "2.0" → False
    """
    try:
        remote_major = version.split(".")[0]
        local_major = PROTO_VERSION.split(".")[0]
        return remote_major == local_major
    except (AttributeError, IndexError):
        return False


# ============================================================
# JSON Lines エンコード/デコード
# ============================================================

def encode_message(msg: dict) -> bytes:
    """dict → JSON bytes + 改行 (JSON Lines 形式)"""
    return json.dumps(msg, ensure_ascii=False).encode("utf-8") + b"\n"


def decode_message(line: bytes) -> dict:
    """JSON bytes 行 → dict

    Raises:
        json.JSONDecodeError: 不正な JSON
        UnicodeDecodeError: 不正なエンコーディング
    """
    return json.loads(line.decode("utf-8").strip())


# ============================================================
# Socket ユーティリティ (asyncio ベース)
# ============================================================

def cleanup_socket(path: str) -> None:
    """残存ソケットファイルの削除"""
    try:
        if os.path.exists(path):
            os.unlink(path)
            logger.debug("古いソケットファイルを削除: %s", path)
    except OSError as e:
        logger.warning("ソケットファイル削除失敗: %s: %s", path, e)


async def create_uds_server(
    path: str,
    handler: Callable,
) -> asyncio.AbstractServer:
    """Unix Domain Socket サーバーを作成

    古いソケットファイルがあれば先に削除する。

    Args:
        path: ソケットファイルパス
        handler: asyncio.start_unix_server の client_connected_cb
                 (reader, writer) を受け取るコールバック

    Returns:
        asyncio.Server インスタンス
    """
    cleanup_socket(path)
    server = await asyncio.start_unix_server(handler, path=path)
    logger.info("UDS サーバー起動: %s", path)
    return server


async def connect_uds(
    path: str,
    retries: int = 5,
    delay: float = 0.5,
) -> Tuple[asyncio.StreamReader, asyncio.StreamWriter]:
    """Unix Domain Socket に接続 (リトライ付き)

    Args:
        path: ソケットファイルパス
        retries: 最大リトライ回数
        delay: リトライ間隔 (秒, 指数バックオフ)

    Returns:
        (StreamReader, StreamWriter) タプル

    Raises:
        ConnectionError: 全リトライ失敗時
    """
    last_error: Optional[Exception] = None
    current_delay = delay

    for attempt in range(1, retries + 1):
        try:
            reader, writer = await asyncio.open_unix_connection(path)
            logger.debug("UDS 接続成功: %s (試行 %d)", path, attempt)
            return reader, writer
        except (FileNotFoundError, ConnectionRefusedError, OSError) as e:
            last_error = e
            if attempt < retries:
                logger.debug(
                    "UDS 接続失敗 (試行 %d/%d): %s: %s, %.1fs 後リトライ",
                    attempt, retries, path, e, current_delay,
                )
                await asyncio.sleep(current_delay)
                current_delay = min(current_delay * 2, 10.0)

    raise ConnectionError(
        f"UDS 接続失敗 ({retries} 回リトライ後): {path}: {last_error}"
    )


async def read_json_lines(
    reader: asyncio.StreamReader,
) -> AsyncIterator[dict]:
    """StreamReader から JSON Lines を非同期イテレート

    接続切断または空行で終了。不正な JSON 行はスキップしてログ出力。

    Yields:
        パースされた dict
    """
    while True:
        try:
            line = await reader.readline()
        except (ConnectionResetError, BrokenPipeError):
            logger.warning("JSON Lines 読取中に接続切断")
            break

        if not line:
            # EOF
            break

        line = line.strip()
        if not line:
            continue

        try:
            yield decode_message(line)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            logger.warning("不正な JSON 行をスキップ: %s (エラー: %s)", line[:100], e)


async def write_json_line(
    writer: asyncio.StreamWriter,
    msg: dict,
) -> None:
    """StreamWriter に 1 つの JSON メッセージを書き込み

    書き込み失敗時はログ出力して例外を伝播。
    """
    data = encode_message(msg)
    writer.write(data)
    await writer.drain()
