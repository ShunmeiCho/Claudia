#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
环形音频缓冲区 — 保存最近 30 秒 16kHz/16bit/mono PCM 数据。

线程安全: 使用 threading.Lock 保护所有读写操作，
可在 asyncio 上下文中通过 run_in_executor 安全调用。
"""

import threading
from typing import Optional

# 16kHz, 16-bit mono: 每毫秒 32 字节
BYTES_PER_MS = 16000 * 2 // 1000  # = 32
DEFAULT_DURATION_S = 30
DEFAULT_BUFFER_SIZE = DEFAULT_DURATION_S * 16000 * 2  # 960,000 bytes


class RingBuffer:
    """固定大小环形缓冲区，用于连续 PCM 音频流。

    Parameters
    ----------
    capacity : int
        缓冲区容量（字节）。默认 960,000（30 秒 @16kHz/16bit/mono）。
    pre_speech_buffer_ms : int
        预语音缓冲毫秒数。read_last / read_range 始终保证此区间可用。
    """

    def __init__(
        self,
        capacity: int = DEFAULT_BUFFER_SIZE,
        pre_speech_buffer_ms: int = 300,
    ) -> None:
        self._capacity = capacity
        self._buf = bytearray(capacity)
        self._write_pos = 0       # 下一个写入位置（循环）
        self._total_written = 0   # 累计写入字节数（不回绕）
        self._lock = threading.Lock()
        self._pre_speech_bytes = pre_speech_buffer_ms * BYTES_PER_MS

    # ------------------------------------------------------------------
    # 写入
    # ------------------------------------------------------------------

    def write(self, data: bytes) -> None:
        """追加 PCM 数据到缓冲区。超出容量时覆盖最老数据。"""
        if not data:
            return

        n = len(data)
        with self._lock:
            if n >= self._capacity:
                # 数据量 >= 整个缓冲区：只保留最后 capacity 字节
                tail = data[-self._capacity:]
                self._buf[:] = tail
                self._write_pos = 0
                self._total_written += n
                return

            end = self._write_pos + n
            if end <= self._capacity:
                self._buf[self._write_pos:end] = data
            else:
                # 需要回绕
                first_part = self._capacity - self._write_pos
                self._buf[self._write_pos:] = data[:first_part]
                self._buf[:n - first_part] = data[first_part:]

            self._write_pos = end % self._capacity
            self._total_written += n

    # ------------------------------------------------------------------
    # 读取
    # ------------------------------------------------------------------

    def read_last(self, ms: int) -> bytes:
        """读取最近 *ms* 毫秒的音频数据。

        如果缓冲区中可用数据不足，返回实际可用部分。
        """
        request_bytes = ms * BYTES_PER_MS
        with self._lock:
            available = min(self._total_written, self._capacity)
            nbytes = min(request_bytes, available)
            if nbytes == 0:
                return b""
            return self._read_tail(nbytes)

    def read_range(self, start_ms: int, end_ms: int) -> bytes:
        """读取从当前写入位置往回 [start_ms, end_ms) 毫秒区间的音频。

        start_ms / end_ms 都是"距离当前时刻的毫秒偏移"，即 0 = 现在。
        例: read_range(500, 200) 读取 500ms 前到 200ms 前的数据。

        如果请求范围超出可用数据，自动截断。
        """
        if start_ms <= end_ms:
            return b""

        start_bytes = start_ms * BYTES_PER_MS
        end_bytes = end_ms * BYTES_PER_MS
        range_bytes = start_bytes - end_bytes

        with self._lock:
            available = min(self._total_written, self._capacity)
            # 截断到可用范围
            actual_start = min(start_bytes, available)
            actual_end = min(end_bytes, available)
            if actual_start <= actual_end:
                return b""

            nbytes = actual_start - actual_end
            # 从 write_pos 往回 actual_start 字节处开始，读 nbytes 字节
            read_start = (self._write_pos - actual_start) % self._capacity
            return self._read_from(read_start, nbytes)

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """清空缓冲区。"""
        with self._lock:
            self._write_pos = 0
            self._total_written = 0
            # 不需要清零 bytearray，write_pos/total_written 控制有效范围

    @property
    def available_ms(self) -> int:
        """当前缓冲区中可用的音频毫秒数。"""
        with self._lock:
            available = min(self._total_written, self._capacity)
        return available // BYTES_PER_MS

    @property
    def capacity_ms(self) -> int:
        """缓冲区总容量（毫秒）。"""
        return self._capacity // BYTES_PER_MS

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _read_tail(self, nbytes: int) -> bytes:
        """从当前写入位置往回读 nbytes 字节。调用者需持有 _lock。"""
        start = (self._write_pos - nbytes) % self._capacity
        return self._read_from(start, nbytes)

    def _read_from(self, start: int, nbytes: int) -> bytes:
        """从 start 位置读取 nbytes 字节。调用者需持有 _lock。"""
        end = start + nbytes
        if end <= self._capacity:
            return bytes(self._buf[start:end])
        else:
            first_part = self._capacity - start
            return bytes(self._buf[start:]) + bytes(self._buf[:nbytes - first_part])
