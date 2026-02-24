#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IPC 协议层: 消息工厂 + Unix Domain Socket 工具

3 路单向 Socket 架构 ($XDG_RUNTIME_DIR/claudia/ 下):
  - audio.sock  (AudioCapture → ASR, raw PCM)
  - result.sock (ASR → Main, JSON Lines)
  - ctrl.sock   (Main → ASR, JSON Lines)

协议版本: major 相同则兼容 (1.0 和 1.1 互通, 2.0 不兼容)
消息格式: JSON Lines (每行一条 JSON, \\n 分隔)
"""

import asyncio
import json
import logging
import os
import stat
import time
from typing import Any, AsyncIterator, Callable, Optional, Tuple

logger = logging.getLogger(__name__)

# === 协议版本 ===
PROTO_VERSION = "1.0"

# === Socket 路径 ===
# セキュリティ: /tmp ではなく $XDG_RUNTIME_DIR (通常 /run/user/<uid>, 0700) を使用。
# 同一ユーザー以外のアクセスを OS レベルで遮断する。
# $XDG_RUNTIME_DIR が未設定/不可書の場合は /tmp/claudia_ipc_<uid> にフォールバック。
# UID サフィックスにより、マルチユーザー環境でもディレクトリ衝突を回避する。
def _socket_dir():
    # type: () -> str
    candidates = []

    # 優先: $XDG_RUNTIME_DIR/claudia (OS レベル 0700 保証)
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime and os.path.isdir(runtime):
        candidates.append(os.path.join(runtime, "claudia"))

    # フォールバック: /tmp/claudia_ipc_<uid> (UID 隔離)
    candidates.append("/tmp/claudia_ipc_{}".format(os.getuid()))

    for d in candidates:
        try:
            os.makedirs(d, mode=0o700, exist_ok=True)
            # 既存ディレクトリのパーミッションを強制修復
            # (他ユーザーが先に緩いパーミッションで作成した場合の防御)
            dir_stat = os.stat(d)
            if dir_stat.st_uid != os.getuid():
                # 他ユーザー所有のディレクトリは使用しない
                logger.warning("ソケットディレクトリ %s は他ユーザー所有、スキップ", d)
                continue
            if stat.S_IMODE(dir_stat.st_mode) != 0o700:
                os.chmod(d, 0o700)
            return d
        except OSError as e:
            logger.warning("ソケットディレクトリ作成失敗: %s: %s", d, e)
            continue

    # 全候補失敗 — 最終手段として例外を投げる
    raise RuntimeError(
        "IPC ソケットディレクトリを作成できません: {}".format(candidates)
    )

_SOCK_DIR = _socket_dir()
AUDIO_SOCKET = os.path.join(_SOCK_DIR, "audio.sock")
ASR_RESULT_SOCKET = os.path.join(_SOCK_DIR, "result.sock")
ASR_CTRL_SOCKET = os.path.join(_SOCK_DIR, "ctrl.sock")
SESSION_TOKEN_FILE = os.path.join(_SOCK_DIR, ".ctrl_token")

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


# === ctrl socket セッション認証 ===
# 同一ユーザー内の不正プロセスによる ctrl メッセージ送信を防止する。
# ASR サーバー起動時にトークンを生成しファイルに書き込み (0o600)、
# ctrl クライアントはトークンファイルを読んでメッセージに含める。


def generate_session_token():
    # type: () -> str
    """ctrl socket 認証用セッショントークンを生成 (16-byte hex)"""
    return os.urandom(16).hex()


def create_ctrl_message(msg_type, token, **kwargs):
    # type: (str, str, **Any) -> dict
    """認証トークン付き ctrl メッセージを生成"""
    msg = {"type": msg_type, "token": token}
    msg.update(kwargs)
    return msg


def validate_session_token(msg, expected_token):
    # type: (dict, str) -> bool
    """ctrl メッセージのセッショントークンを検証"""
    return msg.get("token") == expected_token


def read_session_token():
    # type: () -> Optional[str]
    """トークンファイルからセッショントークンを読み取り"""
    try:
        with open(SESSION_TOKEN_FILE, "r") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


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
    # ソケットファイルのパーミッションを owner-only に制限
    # (同一マシン上の他ユーザーからの不正接続を防止)
    try:
        os.chmod(path, stat.S_IRUSR | stat.S_IWUSR)  # 0o600
    except OSError as e:
        logger.warning("ソケットパーミッション設定失敗: %s: %s", path, e)
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
