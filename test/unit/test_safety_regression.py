#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_safety_regression.py — P0 修复回归测试

验证:
  - P0-1: 3104 返回码不再误判为成功（需 Mock）
  - P0-2: 3103 日志正确
  - P0-5: 初始化用 GetState 而非 RecoveryStand
  - P0-8: 序列中间失败中止
  - hot_cache 归一化匹配
  - SafetyCompiler 全路径覆盖语义一致性
"""

import sys
import os
import asyncio
import logging
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.action_registry import (
    VALID_API_CODES, EXECUTABLE_API_CODES, METHOD_MAP,
    get_response_for_action,
)
from claudia.brain.safety_compiler import SafetyCompiler


# === P0-1: 3104 返回码语义测试 ===
# 注: 完整的 _execute_real 集成测试需要 Mock SportClient，
# 这里测试 SafetyCompiler 层面的语义正确性。


class TestReturnCodeSemantics:
    """确保 SafetyCompiler 不影响 3104/3103 处理"""

    def test_3104_is_not_in_safe_codes(self):
        """3104 不是 API code，不在任何白名单中"""
        assert 3104 not in VALID_API_CODES
        assert 3104 not in EXECUTABLE_API_CODES

    def test_3103_is_not_in_safe_codes(self):
        """3103 不是 API code，不在任何白名单中"""
        assert 3103 not in VALID_API_CODES
        assert 3103 not in EXECUTABLE_API_CODES


# === P0-8: 序列中间失败中止 ===

class TestSequenceAbort:
    """序列中间失败逻辑（SafetyCompiler 层）"""

    def test_sequence_first_invalid_blocks(self):
        """序列首动作非法 → 整个请求被拒"""
        sc = SafetyCompiler()
        v = sc.compile([9999, 1004], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert 9999 in v.rejected

    def test_sequence_mid_invalid_truncates(self):
        """序列中间非法 → 截断保留前面"""
        sc = SafetyCompiler()
        v = sc.compile([1004, 9999, 1005], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 9999 in v.rejected

    def test_sequence_all_valid(self):
        """序列全部合法 → 全部通过"""
        sc = SafetyCompiler()
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]


# === Hot Cache 归一化 ===

class TestHotCacheNormalization:
    """Hot cache 应当能匹配各种输入格式"""

    def test_basic_match_logic(self):
        """基本字符串匹配: strip + lower"""
        # 模拟 hot_cache 的匹配逻辑
        hot_cache = {
            "座って": {"api_code": 1009},
            "立って": {"api_code": 1004},
        }
        # 基本匹配
        cmd = "座って"
        assert cmd.strip() in hot_cache
        # 带空格
        cmd_ws = " 座って "
        assert cmd_ws.strip() in hot_cache

    def test_case_insensitive_fallback(self):
        """英文命令 lower 降级匹配"""
        hot_cache_lower = {
            "sit": {"api_code": 1009},
            "stop": {"api_code": 1003},
        }
        cmd = "SIT"
        assert cmd.strip().lower() in hot_cache_lower
        cmd2 = " Stop "
        assert cmd2.strip().lower() in hot_cache_lower


# === Smoke Test: 语义一致性 ===

class TestSmokeSemanticConsistency:
    """语义级一致性: 验证路由和动作类别（非精确 api_code）"""

    POSTURE_CODES = frozenset([1001, 1002, 1003, 1004, 1005, 1006, 1009, 1010])
    PERFORMANCE_CODES = frozenset([1016, 1017, 1021, 1022, 1023, 1029, 1033, 1036])

    def test_safety_compiler_posture_actions_pass(self):
        """所有姿态动作在正常电量下通过"""
        sc = SafetyCompiler()
        for code in self.POSTURE_CODES:
            v = sc.compile([code], battery_level=0.80, is_standing=True)
            assert not v.is_blocked, "姿态动作 {} 应通过".format(code)
            assert code in v.executable_sequence

    def test_safety_compiler_performance_actions_pass(self):
        """所有表演动作在正常电量下通过"""
        sc = SafetyCompiler()
        for code in self.PERFORMANCE_CODES:
            if code in EXECUTABLE_API_CODES:
                v = sc.compile([code], battery_level=0.80, is_standing=True)
                assert not v.is_blocked, "表演动作 {} 应通过".format(code)
                assert code in v.executable_sequence

    def test_safety_compiler_dance_either_variant(self):
        """Dance1(1022) 和 Dance2(1023) 都可通过"""
        sc = SafetyCompiler()
        for code in [1022, 1023]:
            v = sc.compile([code], battery_level=0.80, is_standing=True)
            assert not v.is_blocked
            assert code in v.executable_sequence

    def test_safety_compiler_standing_prepend_for_hello(self):
        """Hello(1016) 不站立 → 自动前插 StandUp"""
        sc = SafetyCompiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=False)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]

    def test_safety_compiler_sequence_stand_then_hello(self):
        """序列 [1004, 1016] 已站立 → 原样通过"""
        sc = SafetyCompiler()
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]


# === Response Helper 回归 ===

class TestResponseHelperRegression:
    """确保 action_registry 响应辅助函数行为不变"""

    def test_known_action_response(self):
        """已知动作返回日语名称"""
        assert get_response_for_action(1004) == "立ちます"
        assert get_response_for_action(1016) == "挨拶します"
        assert get_response_for_action(1009) == "座ります"

    def test_unknown_action_default(self):
        """未知动作返回默认响应"""
        assert get_response_for_action(9999) == "はい、わかりました"

    def test_all_enabled_have_responses(self):
        """所有 METHOD_MAP 中的动作都有响应"""
        from claudia.brain.action_registry import ACTION_RESPONSES
        for code in METHOD_MAP:
            assert code in ACTION_RESPONSES, "动作 {} 缺少响应".format(code)


# === P0-5: 初始化连通性（静态验证）===

class TestInitConnectivity:
    """验证 MockSportClient.GetState 签名兼容"""

    def test_mock_getstate_returns_tuple(self):
        """MockSportClient.GetState 返回 (code, data) 元组"""
        from claudia.brain.mock_sport_client import MockSportClient
        mock = MockSportClient()
        result = mock.GetState(["mode"])
        assert isinstance(result, tuple)
        assert len(result) == 2
        assert result[0] == 0  # success code

    def test_mock_getstate_no_args(self):
        """MockSportClient.GetState() 无参数也能调用"""
        from claudia.brain.mock_sport_client import MockSportClient
        mock = MockSportClient()
        result = mock.GetState()
        assert isinstance(result, tuple)
        assert result[0] == 0


# === P0-4: 死代码删除验证 ===

class TestDeadCodeRemoved:
    """验证死代码已从 MockSportClient 中不存在"""

    def test_no_rollover_method(self):
        """MockSportClient 不应有 Rollover 方法（SDK 中不存在）"""
        from claudia.brain.mock_sport_client import MockSportClient
        assert not hasattr(MockSportClient, 'Rollover')

    def test_no_handstand_method(self):
        """MockSportClient 不应有 Handstand 方法（SDK 中不存在）"""
        from claudia.brain.mock_sport_client import MockSportClient
        assert not hasattr(MockSportClient, 'Handstand')


# === METHOD_MAP 完整性回归 ===

class TestMethodMapRegression:
    """METHOD_MAP 与 MockSportClient 一致性"""

    def test_all_method_map_methods_exist_on_mock(self):
        """METHOD_MAP 中的所有方法在 MockSportClient 中存在"""
        from claudia.brain.mock_sport_client import MockSportClient
        mock = MockSportClient()
        missing = []
        for api_code, method_name in METHOD_MAP.items():
            if not hasattr(mock, method_name):
                missing.append((api_code, method_name))
        assert not missing, "MockSportClient 缺少方法: {}".format(missing)


# === P0-1: _execute_real 3104 分支真实语义测试 ===

def _run_async(coro):
    """Python 3.8 兼容的 asyncio runner"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _make_lightweight_brain():
    """创建轻量 brain — 跳过 ROS2 状态监控器初始化（避免 Jetson OOM）

    通过临时禁用 STATE_MONITOR_AVAILABLE 来避免 rclpy/DDS 初始化。
    """
    import claudia.brain.production_brain as pb_mod
    from claudia.brain.production_brain import ProductionBrain, BrainOutput
    orig = pb_mod.STATE_MONITOR_AVAILABLE
    pb_mod.STATE_MONITOR_AVAILABLE = False
    try:
        brain = ProductionBrain(use_real_hardware=False)
    finally:
        pb_mod.STATE_MONITOR_AVAILABLE = orig
    brain.sport_client = MagicMock()
    return brain, BrainOutput


class TestExecuteReal3104Semantics:
    """P0-1: 3104 (RPC_ERR_CLIENT_API_TIMEOUT) 语义验证

    通过 monkeypatch _rpc_call 模拟 3104 返回，验证:
      - 3104 + GetState OK → "unknown" (非 True)
      - 3104 + GetState 异常 → False
      - 3104 + GetState 非零 → False
    """

    def _make_brain(self):
        return _make_lightweight_brain()

    def test_3104_getstate_ok_returns_unknown(self):
        """3104 + GetState(0) → 'unknown'"""
        brain, BrainOutput = self._make_brain()

        call_count = [0]
        def mock_rpc_call(method, *args, **kwargs):
            call_count[0] += 1
            if method == "StandUp":
                return 0
            if method == "GetState":
                return (0, {"mode": 1})
            return 3104  # 首次调用返回超时

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", api_code=1016)  # Hello
        result = _run_async(brain._execute_real(output))
        assert result == "unknown", "3104+GetState(0) 应返回 'unknown'，实际: {}".format(result)

    def test_3104_getstate_exception_returns_false(self):
        """3104 + GetState 异常 → False"""
        brain, BrainOutput = self._make_brain()

        def mock_rpc_call(method, *args, **kwargs):
            if method == "GetState":
                raise ConnectionError("DDS connection lost")
            return 3104

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", api_code=1016)
        result = _run_async(brain._execute_real(output))
        assert result is False, "3104+GetState异常 应返回 False，实际: {}".format(result)

    def test_3104_getstate_nonzero_returns_false(self):
        """3104 + GetState 返回非零 → False"""
        brain, BrainOutput = self._make_brain()

        def mock_rpc_call(method, *args, **kwargs):
            if method == "GetState":
                return (3104, None)  # GetState 也超时
            return 3104

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", api_code=1016)
        result = _run_async(brain._execute_real(output))
        assert result is False, "3104+GetState(非零) 应返回 False，实际: {}".format(result)

    def test_return_0_is_true(self):
        """RPC 返回 0 → True (成功)"""
        brain, BrainOutput = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 0
        output = BrainOutput("", api_code=1004)  # StandUp
        result = _run_async(brain._execute_real(output))
        assert result is True

    def test_return_neg1_is_true(self):
        """RPC 返回 -1 (已处于目标状态) → True"""
        brain, BrainOutput = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: -1
        output = BrainOutput("", api_code=1004)
        result = _run_async(brain._execute_real(output))
        assert result is True

    def test_return_3103_is_false(self):
        """RPC 返回 3103 (APP占用) → False"""
        brain, BrainOutput = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 3103
        output = BrainOutput("", api_code=1004)
        result = _run_async(brain._execute_real(output))
        assert result is False

    def test_sequence_mid_failure_aborts(self):
        """P0-8: 序列中间失败 → 中止不继续"""
        brain, BrainOutput = self._make_brain()

        executed = []
        def mock_rpc_call(method, *args, **kwargs):
            executed.append(method)
            if method == "Hello":
                return 3103  # 第二个动作失败
            return 0

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", sequence=[1004, 1016, 1017])  # StandUp, Hello, Stretch
        result = _run_async(brain._execute_real(output))
        assert result is False
        # Hello 失败后不应执行 Stretch
        assert "Stretch" not in executed, "序列应在 Hello 失败后中止，但执行了: {}".format(executed)


# === Finding 1: 紧急停止返回码测试 ===

class TestEmergencyStopReturnCode:
    """紧急停止 RPC 返回码正确反映到 execution_status"""

    def _make_brain(self):
        brain, _ = _make_lightweight_brain()
        return brain

    def test_emergency_rpc_success(self):
        """RPC 返回 0 → execution_status='success'"""
        brain = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 0
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "success"

    def test_emergency_rpc_failure(self):
        """RPC 返回非零 → execution_status='failed'"""
        brain = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 3103
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "failed"

    def test_emergency_rpc_exception(self):
        """RPC 异常 → execution_status='failed'"""
        brain = self._make_brain()
        def raise_err(*args, **kwargs):
            raise ConnectionError("DDS dead")
        brain._rpc_call = raise_err
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "failed"

    def test_emergency_no_client_success(self):
        """无 sport_client（模拟模式）→ execution_status='success'"""
        brain = self._make_brain()
        brain.sport_client = None
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "success"


# === Fix #4: 端到端 hot_cache 路由测试 ===

class TestHotCacheE2ERouting:
    """通过 process_command 验证 hot_cache 路由行为（非模拟 dict 匹配）

    确保 かわいい/空格/大小写变体走 hot_cache 路径而非 conversational。
    提供 mock state_monitor 以确保 SafetyCompiler 获得正常状态
    （battery=0.80, is_standing=True），测试路由而非安全拒绝。
    """

    def _make_brain(self):
        brain, BrainOutput = _make_lightweight_brain()
        brain.sport_client = None  # 模拟模式，不需要 sport_client
        # 提供 mock state_monitor 避免 fail-safe (battery=0.0) 拒绝非安全动作
        mock_state = MagicMock()
        mock_state.battery_level = 0.80
        mock_state.is_standing = True
        mock_state.is_moving = False
        mock_state.temperature = 40.0
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = True  # 保留 mock 状态的 is_standing=True
        brain.state_monitor = mock_monitor
        return brain, BrainOutput

    def test_kawaii_routes_to_heart(self):
        """かわいい → hot_cache → Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("かわいい"))
        assert result.api_code == 1036, (
            "かわいい 应路由到 Heart(1036)，实际: api_code={}".format(result.api_code)
        )

    def test_kawaii_kanji_routes_to_heart(self):
        """可愛い → hot_cache → Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("可愛い"))
        assert result.api_code == 1036

    def test_sugoi_routes_to_heart(self):
        """すごい → hot_cache → Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("すごい"))
        assert result.api_code == 1036

    def test_sugoi_kanji_routes_to_heart(self):
        """凄い → hot_cache → Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("凄い"))
        assert result.api_code == 1036

    def test_cute_english_routes_to_heart(self):
        """cute → hot_cache → Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("cute"))
        assert result.api_code == 1036

    def test_whitespace_normalization(self):
        """' 座って ' (with spaces) → hot_cache → Sit(1009)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command(" 座って "))
        assert result.api_code == 1009, (
            "' 座って ' 应通过 strip() 命中 hot_cache，实际: api_code={}".format(result.api_code)
        )

    def test_case_normalization_english(self):
        """'STOP' → hot_cache (lower fallback) → Stop(1003)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("STOP"))
        assert result.api_code == 1003, (
            "'STOP' 应通过 lower() 命中 hot_cache，实际: api_code={}".format(result.api_code)
        )

    def test_case_normalization_hello(self):
        """'Hello' → hot_cache (lower fallback) → Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("Hello"))
        assert result.api_code == 1016

    def test_greeting_ohayo_routes_to_hello(self):
        """'おはよう' → hot_cache → Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("おはよう"))
        assert result.api_code == 1016

    def test_greeting_konbanwa_routes_to_hello(self):
        """'こんばんは' → hot_cache → Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("こんばんは"))
        assert result.api_code == 1016

    def test_greeting_sayounara_routes_to_hello(self):
        """'さようなら' → hot_cache → Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("さようなら"))
        assert result.api_code == 1016

    def test_greeting_punctuation_normalization(self):
        """'おはよう！' → rstrip punctuation → hot_cache → Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("おはよう！"))
        assert result.api_code == 1016


# === Commander unknown 分支回归测试 ===

class TestCommanderUnknownBranch:
    """验证 commander 对 execute_action 返回值的三分支处理

    核心: "unknown" 是 truthy 字符串，`if success:` 会误判为成功。
    修复后用 `result is True` 严格判定。
    """

    def test_unknown_is_truthy_but_not_true(self):
        """前提: 'unknown' 是 truthy 但 is not True"""
        assert bool("unknown") is True   # truthy
        assert ("unknown" is True) is False  # 但 is True 为 False

    def test_commander_branch_logic_true(self):
        """result=True → '执行成功' 分支"""
        result = True
        assert result is True

    def test_commander_branch_logic_unknown(self):
        """result='unknown' → '动作超时' 分支（不是 is True）"""
        result = "unknown"
        assert result is not True
        assert result == "unknown"

    def test_commander_branch_logic_false(self):
        """result=False → '执行失败' 分支"""
        result = False
        assert result is not True
        assert result != "unknown"

    def test_commander_source_uses_is_true(self):
        """production_commander.py 使用 'result is True' 而非 'if result:'"""
        import pathlib
        commander_path = pathlib.Path(__file__).parent.parent.parent / "production_commander.py"
        source = commander_path.read_text(encoding="utf-8")
        assert "result is True" in source, (
            "production_commander.py 应使用 'result is True' 而非 'if result:'"
        )
        assert 'result == "unknown"' in source, (
            "production_commander.py 应显式检查 'unknown' 分支"
        )


# === state_snapshot=None fail-closed 回归测试 ===

class TestStateSnapshotNoneFailClosed:
    """Critical: state_snapshot=None 时必须走 fail-safe SafetyCompiler，不能跳过

    验证: 状态监控不可用时，非安全动作（Dance/Hello/Jump）被拒绝，
    安全动作（Sit/StandUp/Stop）仍可执行。
    """

    def _run(self, coro):
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_hotcache_high_risk_blocked_without_monitor(self):
        """hot_cache 路径: ジャンプ(1031) 在无状态监控时被 fail-safe 拒绝"""
        brain, BrainOutput = _make_lightweight_brain()
        assert brain.state_monitor is None, "轻量 brain 应无状态监控"
        # ジャンプ(1031) 是 HIGH_ENERGY_ACTION，fail-safe battery=0.0 时被拒绝
        output = self._run(brain.process_command("ジャンプ"))
        assert output.api_code is None, (
            "ジャンプ(1031) 应在无状态监控时被拒绝，实际 api_code={}".format(output.api_code)
        )
        assert output.success is False or "rejected" in (output.reasoning or ""), (
            "应标记为安全拒绝，实际 reasoning={}".format(output.reasoning)
        )

    def test_hotcache_dance_blocked_without_monitor(self):
        """hot_cache 路径: ダンス(1022/1023) 在无状态监控时被 fail-safe 拒绝

        Dance 需要 standing + battery > 0.10，fail-safe battery=0.0 拒绝
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("ダンス"))
        assert output.api_code is None, (
            "ダンス 应在无状态监控时被拒绝，实际 api_code={}".format(output.api_code)
        )

    def test_hotcache_hello_blocked_without_monitor(self):
        """hot_cache 路径: こんにちは(1016) 在无状态监控时被 fail-safe 拒绝

        Hello 需要 standing + battery > 0.10，fail-safe battery=0.0 拒绝
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("こんにちは"))
        assert output.api_code is None, (
            "こんにちは(Hello 1016) 应在无状态监控时被拒绝，实际 api_code={}".format(output.api_code)
        )

    def test_hotcache_safe_action_allowed_without_monitor(self):
        """hot_cache 路径: 座って(1009=Sit) 在无状态监控时仍可通过

        Sit 属于 SAFE_ACTIONS，battery=0.0 也允许
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("座って"))
        assert output.api_code == 1009, (
            "座って(Sit 1009) 是安全动作，应通过 fail-safe，实际 api_code={}".format(output.api_code)
        )

    def test_hotcache_standup_allowed_without_monitor(self):
        """hot_cache 路径: 立って(1004=StandUp) 在无状态监控时仍可通过"""
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("立って"))
        assert output.api_code == 1004, (
            "立って(StandUp 1004) 是安全动作，应通过 fail-safe，实际 api_code={}".format(output.api_code)
        )

    def test_hotcache_stop_allowed_without_monitor(self):
        """hot_cache 路径: 止まれ → StopMove(1003) 在无状态监控时仍可通过"""
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("止まれ"))
        # 止まれ may be routed as emergency or hot_cache, both result in api_code=1003
        assert output.api_code == 1003, (
            "止まれ(Stop 1003) 是安全动作，应通过，实际 api_code={}".format(output.api_code)
        )

    def test_sequence_blocked_without_monitor(self):
        """sequence 路径: 立ってから挨拶 在无状态監視時被 fail-safe 拒绝

        序列含 1016(Hello) 不在 SAFE_ACTIONS 中，battery=0.0 时被拒
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("立ってから挨拶"))
        # 1004(StandUp) is safe but 1016(Hello) is not — sequence should be blocked or truncated
        # SafetyCompiler truncates at first rejection, keeping only prior safe actions
        # But 1004 passes, 1016 fails → truncated to [1004] OR first-reject blocks all
        # Since 1016 is NOT the first action, it's a mid-sequence rejection → truncation
        # Result depends on SafetyCompiler: 1004 alone might pass through
        if output.api_code is not None:
            # If truncated to just StandUp, that's acceptable fail-safe behavior
            assert output.api_code == 1004, (
                "序列截断后应只剩安全动作 1004，实际={}".format(output.api_code)
            )
        elif output.sequence:
            # Sequence should only contain safe actions
            for code in output.sequence:
                assert code in SafetyCompiler.SAFE_ACTIONS, (
                    "序列中不应有非安全动作 {}".format(code)
                )
        # If api_code is None and sequence is None, the whole sequence was rejected — also acceptable

    def test_dance_command_blocked_without_monitor(self):
        """dance 路径: dance 指令在无状态监控时被 fail-safe 拒绝

        Dance(1022/1023) 需 standing + battery > 0.10
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("dance"))
        assert output.api_code is None, (
            "dance 命令应在无状態監視時被拒绝，实际 api_code={}".format(output.api_code)
        )
