#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sdk_state_provider.py — SDKStateProvider 单元测试

验证:
  - GetState(全5键) 查询返回正确姿态
  - LowState DDS 回调正确提取电量
  - 查询失败返回 conservative fallback
  - 电量归一化 (SOC 0-100 → 0.0-1.0)
  - mode 映射 (state → is_standing/is_moving)
  - source 字段正确标记 (sdk/sdk_partial/sdk_fallback)
  - state_ok/battery_ok 细粒度标志
  - 轮询启停
"""

import sys
import os
import time
import threading
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.sdk_state_provider import (
    SDKStateProvider, SDKStateSnapshot, GETSTATE_FULL_KEYS,
)


def _full_state_response(state_val=1):
    """构造 GetState 全键响应（模拟 Go2 固件返回格式）"""
    return (0, {
        "state": state_val,
        "bodyHeight": 0.0,
        "footRaiseHeight": 0.09,
        "speedLevel": 0,
        "gait": 0,
    })


class TestSDKStateSnapshot:
    """SDKStateSnapshot 数据结构测试"""

    def test_default_values(self):
        snap = SDKStateSnapshot()
        assert snap.battery_level == 0.5
        assert snap.is_standing is False
        assert snap.is_moving is False
        assert snap.source == "sdk"
        assert snap.confidence == 1.0
        assert snap.is_charging is False
        assert snap.sdk_connection is True
        assert snap.error_codes == []
        # 新增: 细粒度标志默认值
        assert snap.state_ok is False
        assert snap.battery_ok is False

    def test_conservative_fallback_values(self):
        """fallback 使用保守值: battery=0.50, is_standing=False"""
        snap = SDKStateSnapshot(
            battery_level=0.50, is_standing=False,
            source="sdk_fallback", confidence=0.3
        )
        assert snap.battery_level == 0.50
        assert snap.is_standing is False
        assert snap.source == "sdk_fallback"


class TestSDKStateProviderQuery:
    """SDKStateProvider.query_state() 测试"""

    def _make_provider(self, rpc_fn, soc=None, voltage=None):
        """创建 provider, mock 掉 LowState 订阅"""
        with patch.object(SDKStateProvider, '_init_lowstate_subscriber'):
            provider = SDKStateProvider(rpc_call_fn=rpc_fn)
        # 手动设置电量数据（模拟 DDS 回调已收到）
        if soc is not None:
            provider._battery_soc = soc
        if voltage is not None:
            provider._battery_voltage = voltage
        return provider

    def test_normal_query_state_and_battery(self):
        """正常查询: state=1(standing), soc=80 — 全键查询"""
        def mock_rpc(method, args, **kw):
            if method == "GetState":
                return _full_state_response(1)
            return (0, {})

        provider = self._make_provider(mock_rpc, soc=80, voltage=28.5)
        state = provider.query_state()
        assert state.is_standing is True
        assert state.is_moving is False  # state=1 是 standing 不是 moving
        assert abs(state.battery_level - 0.80) < 0.01
        assert abs(state.battery_voltage - 28.5) < 0.1
        assert state.source == "sdk"
        assert state.confidence == 1.0
        assert state.state_ok is True
        assert state.battery_ok is True

    def test_walking_mode(self):
        """state=2 → 站立+移动"""
        def mock_rpc(method, args, **kw):
            if method == "GetState":
                return _full_state_response(2)
            return (0, {})

        provider = self._make_provider(mock_rpc, soc=50)
        state = provider.query_state()
        assert state.is_standing is True
        assert state.is_moving is True

    def test_idle_mode(self):
        """state=0 → 非站立/非移动"""
        def mock_rpc(method, args, **kw):
            if method == "GetState":
                return _full_state_response(0)
            return (0, {})

        provider = self._make_provider(mock_rpc, soc=60)
        state = provider.query_state()
        assert state.is_standing is False
        assert state.is_moving is False

    def test_battery_normalization_100_to_1(self):
        """SOC 100 归一化到 1.0"""
        def mock_rpc(method, args, **kw):
            return _full_state_response(0)

        provider = self._make_provider(mock_rpc, soc=100)
        state = provider.query_state()
        assert abs(state.battery_level - 1.0) < 0.01

    def test_battery_soc_zero(self):
        """SOC 0 归一化到 0.0"""
        def mock_rpc(method, args, **kw):
            return _full_state_response(0)

        provider = self._make_provider(mock_rpc, soc=0)
        state = provider.query_state()
        assert abs(state.battery_level - 0.0) < 0.01

    def test_state_query_failure_battery_ok(self):
        """state 查询失败但 battery 有数据 → source=sdk_partial, state_ok=False, battery_ok=True"""
        def mock_rpc(method, args, **kw):
            raise Exception("RPC timeout")

        provider = self._make_provider(mock_rpc, soc=75)
        state = provider.query_state()
        assert state.is_standing is False  # 默认
        assert abs(state.battery_level - 0.75) < 0.01
        assert state.source == "sdk_partial"
        assert state.state_ok is False
        assert state.battery_ok is True

    def test_state_ok_no_battery(self):
        """state 查询成功但没有 battery 数据 → source=sdk_partial, state_ok=True, battery_ok=False"""
        def mock_rpc(method, args, **kw):
            return _full_state_response(1)

        provider = self._make_provider(mock_rpc, soc=None)
        state = provider.query_state()
        assert state.is_standing is True
        assert state.source == "sdk_partial"
        assert state.confidence == 0.5
        assert state.state_ok is True
        assert state.battery_ok is False

    def test_both_failure_returns_fallback(self):
        """state 查询失败 + 无 battery → conservative fallback"""
        def mock_rpc(method, args, **kw):
            raise Exception("全部失败")

        provider = self._make_provider(mock_rpc, soc=None)
        state = provider.query_state()
        assert state.source == "sdk_fallback"
        assert abs(state.battery_level - 0.50) < 0.01
        assert state.is_standing is False

    def test_list_format_state(self):
        """某些固件版本 GetState 返回 list 而非 dict"""
        def mock_rpc(method, args, **kw):
            if method == "GetState":
                return (0, [1])  # list 格式
            return (0, {})

        provider = self._make_provider(mock_rpc, soc=75)
        state = provider.query_state()
        assert state.is_standing is True
        assert abs(state.battery_level - 0.75) < 0.01

    def test_getstate_uses_full_keys(self):
        """验证 query_state 使用全 5 键查询（Go2 固件要求）"""
        captured_args = []
        def mock_rpc(method, args, **kw):
            captured_args.append((method, args))
            return _full_state_response(1)

        provider = self._make_provider(mock_rpc, soc=80)
        provider.query_state()
        assert len(captured_args) == 1
        assert captured_args[0][0] == "GetState"
        assert captured_args[0][1] == GETSTATE_FULL_KEYS


class TestLowStateCallback:
    """LowState DDS 回调测试"""

    def _make_provider(self):
        with patch.object(SDKStateProvider, '_init_lowstate_subscriber'):
            return SDKStateProvider(rpc_call_fn=lambda *a, **kw: (0, {}))

    def test_on_lowstate_extracts_soc(self):
        """回调提取 bms_state.soc"""
        provider = self._make_provider()
        msg = MagicMock()
        msg.bms_state.soc = 82
        msg.power_v = 27.3
        provider._on_lowstate(msg)
        assert provider._battery_soc == 82
        assert abs(provider._battery_voltage - 27.3) < 0.1

    def test_on_lowstate_missing_bms(self):
        """回调处理无 bms_state 的消息不崩溃"""
        provider = self._make_provider()
        msg = MagicMock(spec=[])  # 无任何属性
        provider._on_lowstate(msg)
        assert provider._battery_soc is None


class TestSDKStateProviderCache:
    """缓存和轮询测试"""

    def _make_provider(self, rpc_fn=None, soc=80):
        rpc_fn = rpc_fn or (lambda *a, **kw: _full_state_response(1))
        with patch.object(SDKStateProvider, '_init_lowstate_subscriber'):
            provider = SDKStateProvider(rpc_call_fn=rpc_fn)
        provider._battery_soc = soc
        return provider

    def test_get_current_state_returns_cached(self):
        """get_current_state 返回缓存状态"""
        provider = self._make_provider()
        provider.query_state()
        state = provider.get_current_state()
        assert state.is_standing is True
        assert abs(state.battery_level - 0.80) < 0.01

    def test_stale_cache_triggers_query(self):
        """缓存过期（>10s）且没轮询 → 触发主动查询"""
        call_count = [0]
        def mock_rpc(*a, **kw):
            call_count[0] += 1
            return _full_state_response(0)

        provider = self._make_provider(rpc_fn=mock_rpc)
        provider._cached_state.timestamp = time.time() - 15.0
        provider.get_current_state()
        assert call_count[0] > 0

    def test_polling_start_stop(self):
        """轮询启停不崩溃，stop_polling 可快速唤醒轮询线程"""
        provider = self._make_provider()
        provider.start_polling(interval=0.1)
        assert not provider._stop_event.is_set()
        assert provider._poll_thread is not None and provider._poll_thread.is_alive()
        time.sleep(0.3)
        provider.stop_polling()
        assert provider._stop_event.is_set()

    def test_consecutive_failures_switch_to_fallback(self):
        """连续失败 → 缓存切换到 conservative fallback"""
        def fail_rpc(*a, **kw):
            raise Exception("连接失败")

        provider = self._make_provider(rpc_fn=fail_rpc, soc=None)

        # 模拟 5 次全部失败
        for _ in range(5):
            provider.query_state()

        state = provider.get_current_state()
        assert state.source == "sdk_fallback"
        assert abs(state.battery_level - 0.50) < 0.01
        assert state.is_standing is False


class TestSDKStateProviderInterface:
    """接口兼容性测试"""

    def _make_provider(self):
        with patch.object(SDKStateProvider, '_init_lowstate_subscriber'):
            return SDKStateProvider(rpc_call_fn=lambda *a, **kw: (0, {}))

    def test_has_is_ros_initialized(self):
        """SDKStateProvider 必须有 is_ros_initialized=False"""
        provider = self._make_provider()
        assert hasattr(provider, 'is_ros_initialized')
        assert provider.is_ros_initialized is False

    def test_get_current_state_interface(self):
        """get_current_state() 返回对象有 battery_level/is_standing/source/state_ok/battery_ok"""
        provider = self._make_provider()
        state = provider.get_current_state()
        assert hasattr(state, 'battery_level')
        assert hasattr(state, 'is_standing')
        assert hasattr(state, 'source')
        assert hasattr(state, 'state_ok')
        assert hasattr(state, 'battery_ok')

    def test_source_field_present(self):
        """source 字段用于 production_brain 的来源判断"""
        provider = self._make_provider()
        state = provider.get_current_state()
        assert state.source in ("sdk", "sdk_fallback", "sdk_partial")


class TestModeOkBatteryOkLogic:
    """验证 state_ok/battery_ok 判定逻辑和细粒度标志"""

    def _make_provider(self, rpc_fn, soc=None):
        with patch.object(SDKStateProvider, '_init_lowstate_subscriber'):
            provider = SDKStateProvider(rpc_call_fn=rpc_fn)
        provider._battery_soc = soc
        return provider

    def test_both_ok_is_sdk(self):
        """state+battery 都成功 → source=sdk, confidence=1.0, state_ok=True, battery_ok=True"""
        provider = self._make_provider(
            lambda *a, **kw: _full_state_response(1), soc=90
        )
        state = provider.query_state()
        assert state.source == "sdk"
        assert state.confidence == 1.0
        assert state.state_ok is True
        assert state.battery_ok is True

    def test_only_state_ok_is_partial(self):
        """只有 state 成功 → source=sdk_partial, state_ok=True, battery_ok=False"""
        provider = self._make_provider(
            lambda *a, **kw: _full_state_response(1), soc=None
        )
        state = provider.query_state()
        assert state.source == "sdk_partial"
        assert state.state_ok is True
        assert state.battery_ok is False

    def test_only_battery_ok_is_partial(self):
        """只有 battery 成功 → source=sdk_partial, state_ok=False, battery_ok=True"""
        def fail_rpc(*a, **kw):
            raise Exception("RPC fail")
        provider = self._make_provider(fail_rpc, soc=85)
        state = provider.query_state()
        assert state.source == "sdk_partial"
        assert state.state_ok is False
        assert state.battery_ok is True

    def test_neither_ok_is_fallback(self):
        """都失败 → source=sdk_fallback"""
        def fail_rpc(*a, **kw):
            raise Exception("RPC fail")
        provider = self._make_provider(fail_rpc, soc=None)
        state = provider.query_state()
        assert state.source == "sdk_fallback"
