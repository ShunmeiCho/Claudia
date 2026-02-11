#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDK 状态提供器 — 通过 Unitree SDK2 获取真实机器人状态

替代 ROS2 state_monitor，避免同进程 DDS domain 冲突。

数据获取方式:
  - 姿态(mode): SportClient.GetState(全5键) RPC 调用，提取 "state" 字段
  - 电量(soc):  ChannelSubscriber("rt/lowstate") DDS 订阅 → LowState_.bms_state.soc

注意: Go2 固件 GetState 要求全字段查询（单键返回空响应体），
      必须用 ["state","bodyHeight","footRaiseHeight","speedLevel","gait"]。
      电量只能通过 LowState_ 订阅获取，GetState 无 "battery" key。

接口兼容: 返回 SDKStateSnapshot 对象，字段与 SystemStateInfo 一致。
"""

import time
import logging
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Any

# Go2 固件要求全字段查询（单键查询返回空响应体导致 JSONDecodeError）
GETSTATE_FULL_KEYS = ["state", "bodyHeight", "footRaiseHeight", "speedLevel", "gait"]


@dataclass
class SDKStateSnapshot:
    """SDK 状态快照 — SystemStateInfo 兼容子集

    只包含 SafetyCompiler 和 production_brain 实际使用的字段，
    不引用 ROS2 相关的 SystemState/SystemLEDPriority 枚举。
    """
    battery_level: float = 0.5       # 0.0-1.0 归一化
    battery_voltage: float = 25.0
    is_charging: bool = False
    is_standing: bool = False
    is_moving: bool = False
    temperature: float = 40.0
    timestamp: float = 0.0
    source: str = "sdk"              # "sdk" | "sdk_partial" | "sdk_fallback"
    confidence: float = 1.0
    # 细粒度数据可用性标志（解决 sdk_partial 歧义）
    state_ok: bool = False           # GetState RPC 是否成功（姿态数据可信）
    battery_ok: bool = False         # LowState DDS 是否有数据（电量数据可信）
    # 兼容字段（state_monitor 有，brain/audit 可能读）
    error_codes: list = field(default_factory=list)
    current_gait: str = "unknown"
    network_status: str = "connected"
    sdk_connection: bool = True


class SDKStateProvider:
    """通过 Unitree SDK2 获取机器人状态（无 ROS2 依赖）

    姿态数据: SportClient.GetState(GETSTATE_FULL_KEYS) RPC
    电量数据: ChannelSubscriber("rt/lowstate") DDS 订阅

    设计约束:
      - 不创建 ROS2 节点，不触发 rclpy.init()
      - RPC 调用通过 rpc_call_fn 回调，复用 brain 的 _rpc_call 锁
      - DDS 订阅使用 Unitree SDK 的 ChannelSubscriber（同 domain，不冲突）
      - 查询失败时返回 conservative fallback
      - 线程安全: _cached_state 通过 Lock 保护

    source 语义:
      - "sdk":          state 和 battery 都可用（完整数据）
      - "sdk_partial":  只有 state 或只有 battery（部分数据）
      - "sdk_fallback": 全部失败（保守安全值）
    """

    # state 值到姿态映射（Unitree Go2 SDK: GetState 返回的 "state" 字段）
    # 0=idle/damped, 1=standing(balancestand), 2=walking, ...
    STANDING_MODES = {1, 2, 3, 4, 5, 6, 7, 8, 9}  # 站立/移动模式
    MOVING_MODES = {2, 3, 4, 5, 6, 7, 8, 9}        # 移动中模式

    def __init__(self, rpc_call_fn: Callable, logger: Optional[logging.Logger] = None):
        """
        Args:
            rpc_call_fn: brain._rpc_call 回调函数
            logger: 日志器
        """
        self.rpc_call_fn = rpc_call_fn
        self.logger = logger or logging.getLogger("SDKStateProvider")

        self._lock = threading.Lock()
        self._cached_state = self._make_conservative_fallback()
        self._last_success_time = 0.0
        self._consecutive_failures = 0

        # 轮询控制
        self._polling = False
        self._poll_thread = None
        self._poll_interval = 2.0

        # 兼容字段: state_monitor 接口
        self.is_ros_initialized = False  # 始终 False — 不使用 ROS2

        # LowState DDS 订阅（电量来源）
        self._lowstate_sub = None
        self._battery_soc = None       # 0-100 整数, None=尚未收到
        self._battery_voltage = None   # 浮点电压
        self._battery_lock = threading.Lock()
        self._init_lowstate_subscriber()

    def _init_lowstate_subscriber(self):
        """初始化 LowState DDS 订阅（电量数据来源）

        使用 Unitree SDK 的 ChannelSubscriber，复用已有的 DDS domain。
        不需要 rclpy，不会产生 domain 冲突。
        """
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

            self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._lowstate_sub.Init(self._on_lowstate, 10)
            self.logger.info("LowState DDS 订阅已启动 (电量/电压)")
        except ImportError as e:
            self.logger.warning("LowState 订阅不可用（SDK import 失败）: {}".format(e))
            self._lowstate_sub = None
        except Exception as e:
            self.logger.warning("LowState 订阅初始化失败: {}".format(e))
            self._lowstate_sub = None

    def _on_lowstate(self, msg):
        """LowState DDS 回调 — 提取 BMS 电量和电压"""
        try:
            with self._battery_lock:
                # bms_state.soc: 0-100 整数（电池百分比）
                if hasattr(msg, 'bms_state') and hasattr(msg.bms_state, 'soc'):
                    self._battery_soc = int(msg.bms_state.soc)
                # power_v: 浮点电压
                if hasattr(msg, 'power_v'):
                    self._battery_voltage = float(msg.power_v)
        except Exception:
            pass  # 回调中不抛异常

    def _make_conservative_fallback(self):
        # type: () -> SDKStateSnapshot
        """conservative fallback — 查询失败时的安全默认值

        battery=0.50 (中等限制)，is_standing=False (保守)。
        """
        return SDKStateSnapshot(
            battery_level=0.50,
            is_standing=False,
            temperature=40.0,
            timestamp=time.time(),
            source="sdk_fallback",
            confidence=0.3,
        )

    def query_state(self):
        # type: () -> SDKStateSnapshot
        """主动查询一次机器人状态

        通过 SportClient.GetState(GETSTATE_FULL_KEYS) 获取姿态，
        通过 LowState DDS 订阅获取电量。
        """
        snapshot = SDKStateSnapshot(timestamp=time.time())
        state_ok = False
        battery_ok = False

        # 1. 查询运动模式: GetState(全5键) — Go2 固件不支持单键查询
        try:
            result = self.rpc_call_fn("GetState", GETSTATE_FULL_KEYS)
            if isinstance(result, tuple) and len(result) >= 2:
                code, data = result[0], result[1]
                if code == 0 and isinstance(data, dict):
                    mode = data.get("state", data.get("mode", -1))
                    if isinstance(mode, (int, float)):
                        mode = int(mode)
                        snapshot.is_standing = mode in self.STANDING_MODES
                        snapshot.is_moving = mode in self.MOVING_MODES
                        snapshot.current_gait = "mode_{}".format(mode)
                        state_ok = True
                elif code == 0 and isinstance(data, (list, tuple)) and len(data) > 0:
                    mode = data[0] if isinstance(data[0], (int, float)) else -1
                    mode = int(mode)
                    snapshot.is_standing = mode in self.STANDING_MODES
                    snapshot.is_moving = mode in self.MOVING_MODES
                    state_ok = True
                else:
                    self.logger.debug("GetState(state) 返回码: {}".format(code))
            else:
                self.logger.debug("GetState(state) 返回格式异常: {}".format(type(result)))
        except Exception as e:
            self.logger.debug("GetState(state) 查询失败: {}".format(e))

        # 2. 从 LowState DDS 订阅获取电量
        with self._battery_lock:
            soc = self._battery_soc
            voltage = self._battery_voltage

        if soc is not None and 0 <= soc <= 100:
            snapshot.battery_level = soc / 100.0
            battery_ok = True
        if voltage is not None and voltage > 0:
            snapshot.battery_voltage = voltage

        # 3. 写入细粒度标志 + 设置 source
        snapshot.state_ok = state_ok
        snapshot.battery_ok = battery_ok
        if state_ok and battery_ok:
            snapshot.source = "sdk"
            snapshot.confidence = 1.0
        elif state_ok or battery_ok:
            snapshot.source = "sdk_partial"
            snapshot.confidence = 0.5
        else:
            # 全部失败 → conservative fallback
            fallback = self._make_conservative_fallback()
            with self._lock:
                self._cached_state = fallback
                self._consecutive_failures += 1
                if self._consecutive_failures >= 5:
                    self.logger.warning(
                        "连续 {} 次查询全部失败，保持 conservative fallback".format(
                            self._consecutive_failures
                        )
                    )
            return fallback

        # 更新缓存
        with self._lock:
            self._cached_state = snapshot
            self._last_success_time = time.time()
            self._consecutive_failures = 0

        return snapshot

    def get_current_state(self):
        """获取最新状态（兼容 state_monitor 接口）"""
        with self._lock:
            cached = self._cached_state

        # 缓存过期（>10s）且没有后台轮询 → 触发一次查询
        if time.time() - cached.timestamp > 10.0 and not self._polling:
            return self.query_state()

        return cached

    def start_polling(self, interval=2.0):
        # type: (float) -> None
        """启动后台轮询"""
        if self._polling:
            return

        self._poll_interval = interval
        self._polling = True
        self._poll_thread = threading.Thread(
            target=self._poll_worker,
            daemon=True,
            name="sdk-state-poll"
        )
        self._poll_thread.start()
        self.logger.info("SDK 状态轮询已启动 (间隔 {:.1f}s)".format(interval))

    def stop_polling(self):
        """停止后台轮询"""
        self._polling = False
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=3.0)
        self.logger.info("SDK 状态轮询已停止")

    def _poll_worker(self):
        """后台轮询线程"""
        while self._polling:
            try:
                self.query_state()
            except Exception as e:
                self.logger.warning("SDK 状态轮询异常: {}".format(e))
                with self._lock:
                    self._consecutive_failures += 1
                    if self._consecutive_failures >= 5:
                        self._cached_state = self._make_conservative_fallback()
                        self.logger.warning(
                            "连续 {} 次轮询失败，切换到 conservative fallback".format(
                                self._consecutive_failures
                            )
                        )
            time.sleep(self._poll_interval)
