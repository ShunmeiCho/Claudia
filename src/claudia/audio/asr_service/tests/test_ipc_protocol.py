#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ipc_protocol 单元测试"""

import asyncio
import json
import os
import tempfile
import time

import pytest

from claudia.audio.asr_service.ipc_protocol import (
    # 常量
    PROTO_VERSION,
    AUDIO_SOCKET,
    ASR_RESULT_SOCKET,
    ASR_CTRL_SOCKET,
    MSG_READY,
    MSG_HEARTBEAT,
    MSG_TRANSCRIPT,
    MSG_EMERGENCY,
    MSG_VAD_START,
    MSG_VAD_END,
    MSG_ERROR,
    MSG_GATE_TIMEOUT_AUDIT,
    CTRL_TTS_START,
    CTRL_TTS_END,
    CTRL_SHUTDOWN,
    # 消息工厂
    create_handshake,
    create_transcript_msg,
    create_emergency_msg,
    create_heartbeat,
    create_vad_event,
    create_error_msg,
    create_gate_timeout_audit,
    # 协议
    validate_proto_version,
    encode_message,
    decode_message,
    # Socket
    cleanup_socket,
    create_uds_server,
    connect_uds,
    read_json_lines,
    write_json_line,
)


# ============================================================
# encode / decode 往返
# ============================================================

class TestEncodeDecode:

    def test_roundtrip_simple(self):
        msg = {"type": "heartbeat", "ts": 1234567890.123}
        encoded = encode_message(msg)
        assert encoded.endswith(b"\n")
        decoded = decode_message(encoded)
        assert decoded == msg

    def test_roundtrip_unicode(self):
        msg = {"type": "transcript", "text": "止まれ", "confidence": 0.95}
        encoded = encode_message(msg)
        decoded = decode_message(encoded)
        assert decoded["text"] == "止まれ"

    def test_encode_produces_single_line(self):
        msg = {"key": "value", "nested": {"a": 1}}
        encoded = encode_message(msg)
        lines = encoded.strip().split(b"\n")
        assert len(lines) == 1

    def test_decode_strips_whitespace(self):
        raw = b'  {"type": "test"}  \n'
        decoded = decode_message(raw)
        assert decoded["type"] == "test"

    def test_decode_invalid_json_raises(self):
        with pytest.raises(json.JSONDecodeError):
            decode_message(b"not json\n")

    def test_decode_invalid_encoding_raises(self):
        with pytest.raises(UnicodeDecodeError):
            decode_message(b"\xff\xfe invalid")


# ============================================================
# 消息工厂: type フィールド
# ============================================================

class TestMessageFactoryTypes:

    def test_handshake_type(self):
        msg = create_handshake("qwen3-asr-0.6b", 1200)
        assert msg["type"] == MSG_READY

    def test_transcript_type(self):
        msg = create_transcript_msg("座って", 0.95, 1200, 650, "utt_001")
        assert msg["type"] == MSG_TRANSCRIPT

    def test_emergency_type(self):
        msg = create_emergency_msg("とまれ", 0.85, "utt_002")
        assert msg["type"] == MSG_EMERGENCY

    def test_heartbeat_type(self):
        msg = create_heartbeat()
        assert msg["type"] == MSG_HEARTBEAT

    def test_vad_start_type(self):
        msg = create_vad_event("vad_start")
        assert msg["type"] == MSG_VAD_START

    def test_vad_end_type(self):
        msg = create_vad_event("vad_end", duration_ms=800)
        assert msg["type"] == MSG_VAD_END

    def test_error_type(self):
        msg = create_error_msg("something broke")
        assert msg["type"] == MSG_ERROR

    def test_gate_timeout_audit_type(self):
        msg = create_gate_timeout_audit()
        assert msg["type"] == MSG_GATE_TIMEOUT_AUDIT


# ============================================================
# create_handshake
# ============================================================

class TestCreateHandshake:

    def test_includes_proto_version(self):
        msg = create_handshake("qwen3-asr-0.6b", 1200)
        assert "proto_version" in msg
        assert msg["proto_version"] == PROTO_VERSION

    def test_includes_model_and_vram(self):
        msg = create_handshake("qwen3-asr-0.6b", 1200)
        assert msg["model"] == "qwen3-asr-0.6b"
        assert msg["vram_mb"] == 1200


# ============================================================
# create_transcript_msg
# ============================================================

class TestCreateTranscriptMsg:

    def test_all_required_fields(self):
        msg = create_transcript_msg(
            text="座って",
            confidence=0.95,
            duration_ms=1200,
            asr_latency_ms=650,
            utterance_id="utt_20260216_134500_001",
        )
        assert msg["text"] == "座って"
        assert msg["confidence"] == 0.95
        assert msg["duration_ms"] == 1200
        assert msg["asr_latency_ms"] == 650
        assert msg["utterance_id"] == "utt_20260216_134500_001"

    def test_matches_plan_format(self):
        """消息格式与 plan section 1.3 一致"""
        msg = create_transcript_msg("座って", 0.95, 1200, 650, "utt_001")
        required_keys = {"type", "text", "confidence", "duration_ms",
                         "asr_latency_ms", "utterance_id"}
        assert required_keys.issubset(msg.keys())


# ============================================================
# create_emergency_msg
# ============================================================

class TestCreateEmergencyMsg:

    def test_all_required_fields(self):
        msg = create_emergency_msg("とまれ", 0.85, "utt_002")
        assert msg["keyword"] == "とまれ"
        assert msg["confidence"] == 0.85
        assert msg["utterance_id"] == "utt_002"

    def test_matches_plan_format(self):
        msg = create_emergency_msg("とまれ", 0.85, "utt_002")
        required_keys = {"type", "keyword", "confidence", "utterance_id"}
        assert required_keys.issubset(msg.keys())


# ============================================================
# create_heartbeat
# ============================================================

class TestCreateHeartbeat:

    def test_has_timestamp(self):
        before = time.time()
        msg = create_heartbeat()
        after = time.time()
        assert before <= msg["ts"] <= after


# ============================================================
# create_vad_event
# ============================================================

class TestCreateVadEvent:

    def test_vad_start_no_duration(self):
        msg = create_vad_event("vad_start")
        assert msg == {"type": "vad_start"}
        assert "duration_ms" not in msg

    def test_vad_end_with_duration(self):
        msg = create_vad_event("vad_end", duration_ms=800)
        assert msg["type"] == "vad_end"
        assert msg["duration_ms"] == 800

    def test_vad_end_without_duration(self):
        """duration_ms 未指定时不含该字段"""
        msg = create_vad_event("vad_end")
        assert "duration_ms" not in msg

    def test_vad_start_ignores_duration_kwarg(self):
        msg = create_vad_event("vad_start", duration_ms=999)
        assert "duration_ms" not in msg


# ============================================================
# create_error_msg
# ============================================================

class TestCreateErrorMsg:

    def test_message_preserved(self):
        msg = create_error_msg("CUDA OOM: out of memory")
        assert msg["msg"] == "CUDA OOM: out of memory"


# ============================================================
# create_gate_timeout_audit
# ============================================================

class TestCreateGateTimeoutAudit:

    def test_has_timestamp(self):
        before = time.time()
        msg = create_gate_timeout_audit()
        after = time.time()
        assert before <= msg["ts"] <= after


# ============================================================
# validate_proto_version
# ============================================================

class TestValidateProtoVersion:

    def test_same_version(self):
        assert validate_proto_version("1.0") is True

    def test_compatible_minor_diff(self):
        assert validate_proto_version("1.1") is True
        assert validate_proto_version("1.99") is True

    def test_incompatible_major_diff(self):
        assert validate_proto_version("2.0") is False
        assert validate_proto_version("0.1") is False

    def test_invalid_format(self):
        assert validate_proto_version("") is False

    def test_none_input(self):
        assert validate_proto_version(None) is False

    def test_no_dot(self):
        # "1" → split(".")[0] = "1" → matches "1"
        assert validate_proto_version("1") is True

    def test_non_numeric(self):
        assert validate_proto_version("abc") is False


# ============================================================
# Socket 常量
# ============================================================

class TestSocketPaths:

    def test_audio_socket_path(self):
        assert AUDIO_SOCKET == "/tmp/claudia_audio.sock"

    def test_result_socket_path(self):
        assert ASR_RESULT_SOCKET == "/tmp/claudia_asr_result.sock"

    def test_ctrl_socket_path(self):
        assert ASR_CTRL_SOCKET == "/tmp/claudia_asr_ctrl.sock"


# ============================================================
# cleanup_socket
# ============================================================

class TestCleanupSocket:

    def test_removes_existing_file(self, tmp_path):
        sock_path = str(tmp_path / "test.sock")
        with open(sock_path, "w") as f:
            f.write("placeholder")
        assert os.path.exists(sock_path)
        cleanup_socket(sock_path)
        assert not os.path.exists(sock_path)

    def test_no_error_if_missing(self, tmp_path):
        sock_path = str(tmp_path / "nonexistent.sock")
        # 不应抛出异常
        cleanup_socket(sock_path)

    def test_no_error_if_dir_missing(self):
        cleanup_socket("/tmp/nonexistent_dir_abc123/test.sock")


# ============================================================
# UDS server/client + JSON Lines (集成式)
# ============================================================

class TestUdsServerClient:
    """asyncio UDS 服务器 + 客户端 + JSON Lines 读写集成测试"""

    @pytest.mark.asyncio
    async def test_server_client_roundtrip(self, tmp_path):
        sock_path = str(tmp_path / "test_roundtrip.sock")
        received = []

        async def handler(reader, writer):
            async for msg in read_json_lines(reader):
                received.append(msg)
            writer.close()

        server = await create_uds_server(sock_path, handler)

        reader, writer = await connect_uds(sock_path, retries=3, delay=0.1)
        await write_json_line(writer, {"type": "heartbeat", "ts": 1.0})
        await write_json_line(writer, {"type": "transcript", "text": "座って"})
        writer.close()
        await asyncio.sleep(0.1)

        server.close()
        await server.wait_closed()

        assert len(received) == 2
        assert received[0]["type"] == "heartbeat"
        assert received[1]["text"] == "座って"

    @pytest.mark.asyncio
    async def test_connect_uds_retries_on_missing_socket(self, tmp_path):
        sock_path = str(tmp_path / "missing.sock")
        with pytest.raises(ConnectionError):
            await connect_uds(sock_path, retries=2, delay=0.05)

    @pytest.mark.asyncio
    async def test_read_json_lines_skips_invalid(self, tmp_path):
        """不正な JSON 行はスキップ、有効な行のみ yield"""
        sock_path = str(tmp_path / "test_invalid.sock")
        received = []

        async def handler(reader, writer):
            async for msg in read_json_lines(reader):
                received.append(msg)
            writer.close()

        server = await create_uds_server(sock_path, handler)

        reader, writer = await connect_uds(sock_path, retries=3, delay=0.1)
        # 有効 → 不正 → 有効
        writer.write(encode_message({"type": "first"}))
        writer.write(b"this is not json\n")
        writer.write(encode_message({"type": "third"}))
        await writer.drain()
        writer.close()
        await asyncio.sleep(0.1)

        server.close()
        await server.wait_closed()

        assert len(received) == 2
        assert received[0]["type"] == "first"
        assert received[1]["type"] == "third"

    @pytest.mark.asyncio
    async def test_create_uds_server_cleans_stale_socket(self, tmp_path):
        sock_path = str(tmp_path / "stale.sock")
        with open(sock_path, "w") as f:
            f.write("stale")

        async def handler(reader, writer):
            writer.close()

        server = await create_uds_server(sock_path, handler)
        server.close()
        await server.wait_closed()
