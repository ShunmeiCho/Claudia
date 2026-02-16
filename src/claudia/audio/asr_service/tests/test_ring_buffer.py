#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RingBuffer 单元测试"""

import threading
import pytest

from claudia.audio.asr_service.ring_buffer import (
    RingBuffer,
    BYTES_PER_MS,
    DEFAULT_BUFFER_SIZE,
)


# ============================================================
# 基本 write + read_last
# ============================================================

class TestWriteAndReadLast:
    """write 后 read_last 能正确取回数据"""

    def test_write_then_read_last_returns_data(self):
        buf = RingBuffer(capacity=1024)
        data = b"\x01\x02\x03\x04" * 8  # 32 bytes = 1ms
        buf.write(data)
        result = buf.read_last(1)
        assert result == data

    def test_read_last_multiple_writes(self):
        buf = RingBuffer(capacity=4096)
        chunk_a = bytes(range(256)) * (BYTES_PER_MS // 256 + 1)
        chunk_a = chunk_a[:BYTES_PER_MS]  # 正好 1ms
        chunk_b = bytes([0xFF]) * BYTES_PER_MS  # 正好 1ms
        buf.write(chunk_a)
        buf.write(chunk_b)
        result = buf.read_last(2)
        assert result == chunk_a + chunk_b

    def test_read_last_zero_ms_returns_empty(self):
        buf = RingBuffer(capacity=1024)
        buf.write(b"\xAA" * 64)
        assert buf.read_last(0) == b""

    def test_read_last_empty_buffer_returns_empty(self):
        buf = RingBuffer(capacity=1024)
        assert buf.read_last(100) == b""


# ============================================================
# read_last 超出可用数据
# ============================================================

class TestReadLastExceedingAvailable:
    """请求超出已写入数据量时，返回实际可用部分"""

    def test_returns_all_available_when_request_exceeds(self):
        buf = RingBuffer(capacity=4096)
        data = b"\xBB" * BYTES_PER_MS * 2  # 2ms
        buf.write(data)
        # 请求 100ms 但只有 2ms
        result = buf.read_last(100)
        assert result == data

    def test_available_ms_reflects_written_amount(self):
        buf = RingBuffer(capacity=4096)
        buf.write(b"\x00" * BYTES_PER_MS * 3)
        assert buf.available_ms == 3


# ============================================================
# read_range
# ============================================================

class TestReadRange:
    """read_range(start_ms, end_ms) 读取指定区间"""

    def test_read_range_returns_correct_slice(self):
        buf = RingBuffer(capacity=BYTES_PER_MS * 10)
        # 写入 5ms 数据，每 ms 不同字节
        for i in range(5):
            buf.write(bytes([i]) * BYTES_PER_MS)
        # read_range(3, 1) = 3ms前 到 1ms前 = 第 2、3 ms 的数据
        result = buf.read_range(3, 1)
        expected = bytes([2]) * BYTES_PER_MS + bytes([3]) * BYTES_PER_MS
        assert result == expected

    def test_read_range_invalid_order_returns_empty(self):
        """start_ms <= end_ms 时返回空"""
        buf = RingBuffer(capacity=1024)
        buf.write(b"\x00" * BYTES_PER_MS * 5)
        assert buf.read_range(1, 3) == b""
        assert buf.read_range(2, 2) == b""

    def test_read_range_exceeding_available_truncates(self):
        buf = RingBuffer(capacity=BYTES_PER_MS * 10)
        buf.write(b"\xCC" * BYTES_PER_MS * 3)  # 3ms
        # 请求 10ms 前到 0ms 前，但只有 3ms
        result = buf.read_range(10, 0)
        assert len(result) == BYTES_PER_MS * 3


# ============================================================
# 回绕 (wrap-around)
# ============================================================

class TestWrapAround:
    """写入超出容量时正确回绕，覆盖最老数据"""

    def test_wrap_around_overwrites_oldest(self):
        cap = BYTES_PER_MS * 4  # 4ms 容量
        buf = RingBuffer(capacity=cap)
        # 写入 6ms，前 2ms 会被覆盖
        for i in range(6):
            buf.write(bytes([i]) * BYTES_PER_MS)
        # read_last(4) 应取到第 2,3,4,5 ms
        result = buf.read_last(4)
        expected = (
            bytes([2]) * BYTES_PER_MS
            + bytes([3]) * BYTES_PER_MS
            + bytes([4]) * BYTES_PER_MS
            + bytes([5]) * BYTES_PER_MS
        )
        assert result == expected

    def test_available_ms_capped_at_capacity(self):
        cap = BYTES_PER_MS * 4
        buf = RingBuffer(capacity=cap)
        buf.write(b"\x00" * BYTES_PER_MS * 10)
        assert buf.available_ms == 4

    def test_write_larger_than_capacity(self):
        """单次写入超出容量，只保留末尾 capacity 字节"""
        cap = BYTES_PER_MS * 2
        buf = RingBuffer(capacity=cap)
        big_data = b""
        for i in range(5):
            big_data += bytes([i]) * BYTES_PER_MS
        buf.write(big_data)
        result = buf.read_last(2)
        expected = bytes([3]) * BYTES_PER_MS + bytes([4]) * BYTES_PER_MS
        assert result == expected


# ============================================================
# clear
# ============================================================

class TestClear:
    """clear() 重置缓冲区"""

    def test_clear_resets_available(self):
        buf = RingBuffer(capacity=4096)
        buf.write(b"\x00" * BYTES_PER_MS * 5)
        assert buf.available_ms == 5
        buf.clear()
        assert buf.available_ms == 0

    def test_clear_then_read_returns_empty(self):
        buf = RingBuffer(capacity=4096)
        buf.write(b"\xFF" * BYTES_PER_MS * 3)
        buf.clear()
        assert buf.read_last(10) == b""

    def test_write_after_clear_works(self):
        buf = RingBuffer(capacity=4096)
        buf.write(b"\xAA" * BYTES_PER_MS)
        buf.clear()
        new_data = b"\xBB" * BYTES_PER_MS
        buf.write(new_data)
        assert buf.read_last(1) == new_data


# ============================================================
# pre_speech_buffer_ms
# ============================================================

class TestPreSpeechBuffer:
    """pre_speech_buffer_ms 构造参数"""

    def test_default_pre_speech_buffer(self):
        buf = RingBuffer()
        assert buf._pre_speech_bytes == 300 * BYTES_PER_MS

    def test_custom_pre_speech_buffer(self):
        buf = RingBuffer(pre_speech_buffer_ms=500)
        assert buf._pre_speech_bytes == 500 * BYTES_PER_MS

    def test_zero_pre_speech_buffer(self):
        buf = RingBuffer(pre_speech_buffer_ms=0)
        assert buf._pre_speech_bytes == 0


# ============================================================
# capacity_ms
# ============================================================

class TestCapacityMs:

    def test_default_capacity_ms(self):
        buf = RingBuffer()
        assert buf.capacity_ms == 30_000  # 30 秒

    def test_custom_capacity_ms(self):
        buf = RingBuffer(capacity=BYTES_PER_MS * 100)
        assert buf.capacity_ms == 100


# ============================================================
# 线程安全
# ============================================================

class TestThreadSafety:
    """并发写入不会崩溃或破坏数据一致性"""

    def test_concurrent_writes_no_crash(self):
        buf = RingBuffer(capacity=BYTES_PER_MS * 100)
        errors = []

        def writer(thread_id: int):
            try:
                for _ in range(50):
                    buf.write(bytes([thread_id]) * BYTES_PER_MS)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert not errors
        # 8 线程 * 50 次 = 400ms 写入，但容量只有 100ms
        assert buf.available_ms == 100

    def test_concurrent_read_write_no_crash(self):
        buf = RingBuffer(capacity=BYTES_PER_MS * 50)
        errors = []
        stop = threading.Event()

        def writer():
            try:
                while not stop.is_set():
                    buf.write(b"\x00" * BYTES_PER_MS)
            except Exception as e:
                errors.append(e)

        def reader():
            try:
                while not stop.is_set():
                    buf.read_last(10)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=writer),
            threading.Thread(target=reader),
            threading.Thread(target=reader),
        ]
        for t in threads:
            t.start()

        import time
        time.sleep(0.2)
        stop.set()

        for t in threads:
            t.join()

        assert not errors
