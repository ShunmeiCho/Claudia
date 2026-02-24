#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ipc_protocol 单元测试"""

import asyncio
import json
import os
import stat
import tempfile
import time

import pytest

from claudia.audio.asr_service.ipc_protocol import (
    # 常量
    PROTO_VERSION,
    AUDIO_SOCKET,
    ASR_RESULT_SOCKET,
    ASR_CTRL_SOCKET,
    SESSION_TOKEN_FILE,
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
    # セッション認証
    generate_session_token,
    create_ctrl_message,
    validate_session_token,
    read_session_token,
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
        msg = {"type": "transcript", "text": "\u6b62\u307e\u308c", "confidence": 0.95}
        encoded = encode_message(msg)
        decoded = decode_message(encoded)
        assert decoded["text"] == "\u6b62\u307e\u308c"

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
        msg = create_transcript_msg("\u5ea7\u3063\u3066", 0.95, 1200, 650, "utt_001")
        assert msg["type"] == MSG_TRANSCRIPT

    def test_emergency_type(self):
        msg = create_emergency_msg("\u3068\u307e\u308c", 0.85, "utt_002")
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
            text="\u5ea7\u3063\u3066",
            confidence=0.95,
            duration_ms=1200,
            asr_latency_ms=650,
            utterance_id="utt_20260216_134500_001",
        )
        assert msg["text"] == "\u5ea7\u3063\u3066"
        assert msg["confidence"] == 0.95
        assert msg["duration_ms"] == 1200
        assert msg["asr_latency_ms"] == 650
        assert msg["utterance_id"] == "utt_20260216_134500_001"

    def test_matches_plan_format(self):
        msg = create_transcript_msg("\u5ea7\u3063\u3066", 0.95, 1200, 650, "utt_001")
        required_keys = {"type", "text", "confidence", "duration_ms",
                         "asr_latency_ms", "utterance_id"}
        assert required_keys.issubset(msg.keys())


# ============================================================
# create_emergency_msg
# ============================================================

class TestCreateEmergencyMsg:

    def test_all_required_fields(self):
        msg = create_emergency_msg("\u3068\u307e\u308c", 0.85, "utt_002")
        assert msg["keyword"] == "\u3068\u307e\u308c"
        assert msg["confidence"] == 0.85
        assert msg["utterance_id"] == "utt_002"

    def test_matches_plan_format(self):
        msg = create_emergency_msg("\u3068\u307e\u308c", 0.85, "utt_002")
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
        # "1" -> split(".")[0] = "1" -> matches "1"
        assert validate_proto_version("1") is True

    def test_non_numeric(self):
        assert validate_proto_version("abc") is False


# ============================================================
# Socket 常量
# ============================================================

class TestSocketPaths:

    def test_audio_socket_in_claudia_dir(self):
        assert AUDIO_SOCKET.endswith("/audio.sock")
        assert "claudia" in AUDIO_SOCKET

    def test_result_socket_in_claudia_dir(self):
        assert ASR_RESULT_SOCKET.endswith("/result.sock")
        assert "claudia" in ASR_RESULT_SOCKET

    def test_ctrl_socket_in_claudia_dir(self):
        assert ASR_CTRL_SOCKET.endswith("/ctrl.sock")
        assert "claudia" in ASR_CTRL_SOCKET

    def test_sockets_share_directory(self):
        import os
        dirs = {os.path.dirname(p) for p in [AUDIO_SOCKET, ASR_RESULT_SOCKET, ASR_CTRL_SOCKET]}
        assert len(dirs) == 1, "all sockets must be in the same directory"


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
        cleanup_socket(sock_path)

    def test_no_error_if_dir_missing(self):
        cleanup_socket("/tmp/nonexistent_dir_abc123/test.sock")


# ============================================================
# セッション認証: token 生成 / 検証 / 読取
# ============================================================

class TestGenerateSessionToken:

    def test_returns_hex_string(self):
        token = generate_session_token()
        assert isinstance(token, str)
        int(token, 16)  # must parse as hex

    def test_length_is_32_hex_chars(self):
        """16 bytes = 32 hex chars"""
        token = generate_session_token()
        assert len(token) == 32

    def test_uniqueness(self):
        tokens = {generate_session_token() for _ in range(10)}
        assert len(tokens) == 10


class TestCreateCtrlMessage:

    def test_includes_type_and_token(self):
        msg = create_ctrl_message("shutdown", "abc123")
        assert msg["type"] == "shutdown"
        assert msg["token"] == "abc123"

    def test_extra_kwargs_included(self):
        msg = create_ctrl_message("shutdown", "abc", reason="exit")
        assert msg["reason"] == "exit"

    def test_no_extra_keys_without_kwargs(self):
        msg = create_ctrl_message("tts_start", "tok")
        assert set(msg.keys()) == {"type", "token"}


class TestValidateSessionToken:

    def test_valid_token(self):
        assert validate_session_token({"token": "abc"}, "abc") is True

    def test_invalid_token(self):
        assert validate_session_token({"token": "abc"}, "xyz") is False

    def test_missing_token_field(self):
        assert validate_session_token({"type": "shutdown"}, "abc") is False

    def test_empty_message(self):
        assert validate_session_token({}, "abc") is False


class TestReadSessionToken:

    def test_reads_from_file(self, tmp_path):
        token_file = tmp_path / "test_token"
        token_file.write_text("deadbeef1234")
        import claudia.audio.asr_service.ipc_protocol as proto
        orig = proto.SESSION_TOKEN_FILE
        try:
            proto.SESSION_TOKEN_FILE = str(token_file)
            result = read_session_token()
            assert result == "deadbeef1234"
        finally:
            proto.SESSION_TOKEN_FILE = orig

    def test_returns_none_on_missing_file(self, tmp_path):
        import claudia.audio.asr_service.ipc_protocol as proto
        orig = proto.SESSION_TOKEN_FILE
        try:
            proto.SESSION_TOKEN_FILE = str(tmp_path / "nonexistent")
            result = read_session_token()
            assert result is None
        finally:
            proto.SESSION_TOKEN_FILE = orig

    def test_strips_whitespace(self, tmp_path):
        token_file = tmp_path / "test_token"
        token_file.write_text("  abc123  \n")
        import claudia.audio.asr_service.ipc_protocol as proto
        orig = proto.SESSION_TOKEN_FILE
        try:
            proto.SESSION_TOKEN_FILE = str(token_file)
            result = read_session_token()
            assert result == "abc123"
        finally:
            proto.SESSION_TOKEN_FILE = orig


class TestSessionTokenFilePath:

    def test_token_file_in_socket_dir(self):
        assert os.path.dirname(SESSION_TOKEN_FILE) == os.path.dirname(ASR_CTRL_SOCKET)

    def test_token_file_is_hidden(self):
        assert os.path.basename(SESSION_TOKEN_FILE).startswith(".")


# ============================================================
# _socket_dir() 分岐カバレッジ
# ============================================================

class TestSocketDirBranches:
    """_socket_dir() の XDG 回退/owner 検査/symlink 拒否の分岐テスト"""

    def test_xdg_unavailable_falls_back_to_tmp(self, tmp_path, monkeypatch):
        """XDG_RUNTIME_DIR 未設定時 /tmp/claudia_ipc_<uid> にフォールバック"""
        import claudia.audio.asr_service.ipc_protocol as proto
        monkeypatch.delenv("XDG_RUNTIME_DIR", raising=False)
        # _socket_dir は module-level で呼ばれ済みだが、関数を直接テスト
        result = proto._socket_dir()
        uid = os.getuid()
        assert result == "/tmp/claudia_ipc_{}".format(uid)

    def test_xdg_unwritable_falls_back_to_tmp(self, tmp_path, monkeypatch):
        """XDG 配下の makedirs が OSError → /tmp フォールバック

        chmod 0o500 は root/CAP_DAC_OVERRIDE で無視されるため、
        monkeypatch で makedirs を直接失敗させて確実にテストする。
        """
        import claudia.audio.asr_service.ipc_protocol as proto
        xdg_dir = str(tmp_path / "fake_xdg")
        os.makedirs(xdg_dir, mode=0o700)
        monkeypatch.setenv("XDG_RUNTIME_DIR", xdg_dir)
        xdg_candidate = os.path.join(xdg_dir, "claudia")
        _real_makedirs = os.makedirs

        def _failing_makedirs(path, **kwargs):
            if path == xdg_candidate:
                raise OSError(13, "Permission denied", path)
            return _real_makedirs(path, **kwargs)

        monkeypatch.setattr(os, "makedirs", _failing_makedirs)
        result = proto._socket_dir()
        uid = os.getuid()
        assert result == "/tmp/claudia_ipc_{}".format(uid)

    def test_symlink_rejected(self, tmp_path, monkeypatch):
        """symlink ディレクトリは拒否され次の候補にフォールバック"""
        import claudia.audio.asr_service.ipc_protocol as proto
        monkeypatch.delenv("XDG_RUNTIME_DIR", raising=False)
        uid = os.getuid()
        target_name = "/tmp/claudia_ipc_{}".format(uid)
        # 既存ディレクトリがある場合は一時退避
        backup = None
        if os.path.exists(target_name) and not os.path.islink(target_name):
            backup = target_name + "_backup_test"
            os.rename(target_name, backup)
        elif os.path.islink(target_name):
            os.unlink(target_name)
        try:
            # symlink を作成 (target は tmp_path 内の自分所有ディレクトリ)
            real_dir = str(tmp_path / "real_target")
            os.makedirs(real_dir, mode=0o700)
            os.symlink(real_dir, target_name)
            # _socket_dir は symlink を拒否すべき → RuntimeError (全候補失敗)
            with pytest.raises(RuntimeError):
                proto._socket_dir()
        finally:
            if os.path.islink(target_name):
                os.unlink(target_name)
            if backup:
                os.rename(backup, target_name)

    def test_other_user_dir_skipped(self, tmp_path, monkeypatch):
        """他ユーザー所有ディレクトリはスキップされ次の候補へ

        非 root では他 UID のディレクトリを作れないため、
        os.lstat の返り値を monkeypatch で偽装する。
        """
        import claudia.audio.asr_service.ipc_protocol as proto
        monkeypatch.delenv("XDG_RUNTIME_DIR", raising=False)
        uid = os.getuid()
        target_name = "/tmp/claudia_ipc_{}".format(uid)
        _real_lstat = os.lstat

        class _FakeStat:
            """1 回目の lstat だけ他ユーザー UID を返す"""
            def __init__(self, real):
                self.st_mode = real.st_mode
                self.st_uid = uid + 1  # 他ユーザー偽装
                self.st_size = real.st_size

        call_count = [0]

        def _patched_lstat(path):
            result = _real_lstat(path)
            if path == target_name and call_count[0] == 0:
                call_count[0] += 1
                return _FakeStat(result)
            return result

        monkeypatch.setattr(os, "lstat", _patched_lstat)
        # 唯一の候補 (/tmp/claudia_ipc_<uid>) が owner 不一致でスキップ → RuntimeError
        with pytest.raises(RuntimeError):
            proto._socket_dir()

    def test_permissions_repaired(self, tmp_path, monkeypatch):
        """0o700 以外のパーミッションは自動修復される"""
        import claudia.audio.asr_service.ipc_protocol as proto
        test_dir = str(tmp_path / "claudia")
        os.makedirs(test_dir, mode=0o755)
        assert stat.S_IMODE(os.lstat(test_dir).st_mode) == 0o755
        # XDG を test_dir の親に向ける
        monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
        result = proto._socket_dir()
        assert result == test_dir
        assert stat.S_IMODE(os.lstat(test_dir).st_mode) == 0o700


# ============================================================
# UDS server/client + JSON Lines (集成式)
# pytest 4.6.9 互換: loop.run_until_complete() で実行
# (pytest-asyncio は pytest >= 7.0 が必要で ROS2 環境と非互換)
# ============================================================

def _run_async(coro):
    """Python 3.8 / pytest 4.6.9 互換の async テストランナー"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class TestUdsServerClient:

    def test_server_client_roundtrip(self, tmp_path):
        async def _test():
            sock_path = str(tmp_path / "test_roundtrip.sock")
            received = []

            async def handler(reader, writer):
                async for msg in read_json_lines(reader):
                    received.append(msg)
                writer.close()

            server = await create_uds_server(sock_path, handler)

            reader, writer = await connect_uds(sock_path, retries=3, delay=0.1)
            await write_json_line(writer, {"type": "heartbeat", "ts": 1.0})
            await write_json_line(writer, {"type": "transcript", "text": "\u5ea7\u3063\u3066"})
            writer.close()
            await asyncio.sleep(0.1)

            server.close()
            await server.wait_closed()

            assert len(received) == 2
            assert received[0]["type"] == "heartbeat"
            assert received[1]["text"] == "\u5ea7\u3063\u3066"

        _run_async(_test())

    def test_connect_uds_retries_on_missing_socket(self, tmp_path):
        async def _test():
            sock_path = str(tmp_path / "missing.sock")
            with pytest.raises(ConnectionError):
                await connect_uds(sock_path, retries=2, delay=0.05)

        _run_async(_test())

    def test_read_json_lines_skips_invalid(self, tmp_path):
        async def _test():
            sock_path = str(tmp_path / "test_invalid.sock")
            received = []

            async def handler(reader, writer):
                async for msg in read_json_lines(reader):
                    received.append(msg)
                writer.close()

            server = await create_uds_server(sock_path, handler)

            reader, writer = await connect_uds(sock_path, retries=3, delay=0.1)
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

        _run_async(_test())

    def test_create_uds_server_cleans_stale_socket(self, tmp_path):
        async def _test():
            sock_path = str(tmp_path / "stale.sock")
            with open(sock_path, "w") as f:
                f.write("stale")

            async def handler(reader, writer):
                writer.close()

            server = await create_uds_server(sock_path, handler)
            server.close()
            await server.wait_closed()

        _run_async(_test())
