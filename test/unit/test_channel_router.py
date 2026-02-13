#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_channel_router.py — PR2 Slice B: ChannelRouter 骨架验证

验证:
  - B1: RouterMode 枚举和 BRAIN_ROUTER_MODE 环境变量
  - B2: Legacy 模式透传现有 LLM 路径
  - B3: Dual 模式: action → template / a=null → voice
  - B4: Shadow 模式: legacy 为主，dual 并行观测
  - B5: SafetyCompiler 在所有模式下仍然验证（Invariant 1）
  - B6: 无效 BRAIN_ROUTER_MODE → 降级 legacy
  - B7: RouterResult 数据完整性
"""

import sys
import os
import asyncio
import logging
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import AsyncMock, MagicMock, patch

from claudia.brain.channel_router import (
    ChannelRouter, RouterMode, RouterResult, MAX_SEQUENCE_LENGTH,
)
from claudia.brain.production_brain import ProductionBrain, BrainOutput, _pae_depth
from claudia.brain.audit_routes import (
    ROUTE_LLM_7B, ROUTE_ACTION_CHANNEL, ROUTE_VOICE_CHANNEL,
    ROUTE_SHADOW, ROUTE_ACTION_FALLBACK,
)


# === 辅助工具 ===

def _make_brain():
    """创建最小化 mock brain"""
    with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
        brain = ProductionBrain.__new__(ProductionBrain)
    brain.logger = logging.getLogger("test_router")
    brain._command_lock = asyncio.Lock()
    brain.sport_client = None
    brain.use_real_hardware = False
    brain.model_7b = "test-model-7b"
    brain.safety_compiler = MagicMock()
    brain.state_monitor = None
    brain.audit_logger = None
    brain.last_posture_standing = False
    brain.EMERGENCY_COMMANDS = {}
    return brain


# === B1: RouterMode 枚举 ===

class TestRouterMode:
    """验证 RouterMode 枚举值"""

    def test_legacy_mode(self):
        assert RouterMode("legacy") == RouterMode.LEGACY

    def test_dual_mode(self):
        assert RouterMode("dual") == RouterMode.DUAL

    def test_shadow_mode(self):
        assert RouterMode("shadow") == RouterMode.SHADOW

    def test_invalid_mode_raises(self):
        import pytest
        with pytest.raises(ValueError):
            RouterMode("invalid")


# === B2: Legacy 模式 ===

class TestLegacyRoute:
    """验证 Legacy 模式透传 7B LLM"""

    def test_legacy_delegates_to_ollama_v2(self):
        """Legacy 模式调用 brain._call_ollama_v2"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(
                router.route("おすわり"))
            assert isinstance(result, RouterResult)
            assert result.api_code == 1009
            assert result.response == "座ります"
            assert result.route == ROUTE_LLM_7B
        finally:
            loop.close()

    def test_legacy_none_response(self):
        """Legacy 模式 LLM 无响应时返回默认文本"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.api_code is None
            assert result.sequence is None
            assert "理解できませんでした" in result.response
        finally:
            loop.close()

    def test_legacy_returns_sequence(self):
        """Legacy 模式返回序列"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"r": "立ってから座ります", "s": [1004, 1009]}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("立ってから座って"))
            assert result.sequence == [1004, 1009]
            assert result.api_code is None or result.api_code == result.sequence
        finally:
            loop.close()


# === B3: Dual 模式 ===

class TestDualRoute:
    """验证 Dual 模式: action 通道 + voice 回退"""

    def test_dual_action_returns_template(self):
        """Dual: action 通道返回有效 api_code → 模板响应"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        call_count = {"action": 0, "legacy": 0}

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                call_count["action"] += 1
                return {"a": 1009}
            else:
                call_count["legacy"] += 1
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            assert result.api_code == 1009
            assert result.route == ROUTE_ACTION_CHANNEL
            assert call_count["action"] == 1
            # 不应调用 legacy（action 成功）
            assert call_count["legacy"] == 0
            # 响应应为模板（非 LLM 生成）
            assert len(result.response) > 0
        finally:
            loop.close()

    def test_dual_a_null_goes_voice(self):
        """Dual: a=null → voice 回退获取文本响应（Invariant 2）"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": None}
            else:
                return {"r": "私はClaudiaです", "a": None}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("お名前は？"))
            assert result.api_code is None
            assert result.route == ROUTE_VOICE_CHANNEL
            # Invariant 2: 响应不为空
            assert len(result.response) > 0
            assert "Claudia" in result.response
        finally:
            loop.close()

    def test_dual_action_timeout_fallback(self):
        """Dual: action 通道超时 → 回退 legacy"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return None  # 超时
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            # 回退到 legacy 成功
            assert result.api_code == 1009
            assert result.route == ROUTE_ACTION_FALLBACK
        finally:
            loop.close()

    def test_dual_invalid_api_code_fallback(self):
        """Dual: action 通道返回非法 api_code → 回退"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 9999}  # 非法代码
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            assert result.route == ROUTE_ACTION_FALLBACK
        finally:
            loop.close()

    def test_dual_sequence_valid(self):
        """Dual: action 通道返回有效序列"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": [1004, 1016]}
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("立ってから挨拶"))
            assert result.sequence == [1004, 1016]
            assert result.route == ROUTE_ACTION_CHANNEL
        finally:
            loop.close()


# === B4: Shadow 模式 ===

class TestShadowRoute:
    """验证 Shadow 模式: legacy 为主，dual 并行"""

    def test_shadow_returns_legacy_result(self):
        """Shadow 模式总是返回 legacy 结果"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 1016}  # dual 选 hello
            else:
                return {"r": "座ります", "a": 1009}  # legacy 选 sit

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            # 主路径是 legacy
            assert result.api_code == 1009
            assert result.route == ROUTE_SHADOW
        finally:
            loop.close()

    def test_shadow_has_comparison(self):
        """Shadow 模式包含 shadow_comparison 数据"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 1016}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.shadow_comparison is not None
            sc = result.shadow_comparison
            assert sc["legacy_api_code"] == 1009
            assert sc["dual_api_code"] == 1016
            assert sc["raw_agreement"] is False  # 不一致
        finally:
            loop.close()

    def test_shadow_agreement_when_same(self):
        """Shadow: legacy 和 dual 一致时 raw_agreement=True"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            sc = result.shadow_comparison
            assert sc["raw_agreement"] is True
        finally:
            loop.close()

    def test_shadow_timeout_recorded(self):
        """Shadow: wait_for 超时 → dual_status='timeout'（Invariant 4）"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                await asyncio.sleep(10)  # 超时
                return {"a": 1009}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            # 用较短超时测试（monkey-patch _build_shadow_comparison 的超时）
            original = router._build_shadow_comparison

            async def fast_shadow(legacy_result, dual_task):
                try:
                    dual_result = await asyncio.wait_for(
                        asyncio.shield(dual_task), timeout=0.1)
                    return {"dual_status": "ok", "raw_agreement": True}
                except asyncio.TimeoutError:
                    if not dual_task.done():
                        dual_task.cancel()
                    return {
                        "legacy_api_code": legacy_result.api_code,
                        "dual_api_code": None,
                        "dual_sequence": None,
                        "dual_status": "timeout",
                        "raw_agreement": False,
                        "high_risk_divergence": False,
                        "legacy_ms": 0, "dual_ms": 100,
                    }

            router._build_shadow_comparison = fast_shadow
            result = loop.run_until_complete(router.route("test"))
            sc = result.shadow_comparison
            assert sc["dual_status"] == "timeout"
            assert sc["raw_agreement"] is False
        finally:
            loop.close()

    def test_shadow_action_channel_internal_timeout_encoded(self):
        """action channel 内部超时(raw=None) → dual_status='timeout'

        当 _call_ollama_v2 返回 None（Ollama 自身超时）且 allow_fallback=False 时，
        _action_channel 返回 _action_status="timeout"。
        _build_shadow_comparison 应读取 _action_status，编码 dual_status="timeout"。
        """
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return None  # Ollama 内部超时
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("座って"))
            sc = result.shadow_comparison
            assert sc is not None, "shadow_comparison 应存在"
            assert sc["dual_status"] == "timeout", \
                "action channel 内部超时: dual_status='timeout', got: {}".format(
                    sc["dual_status"])
            assert sc["raw_agreement"] is False
        finally:
            loop.close()

    def test_shadow_invalid_output_encoded(self):
        """action channel 非法 api_code → dual_status='invalid_output'

        Finding #2: 模型输出非法 api_code 时，shadow 应记录 dual_status="invalid_output"，
        而不是将清零后的 None 与正常 a=null 混淆。
        """
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 9999}  # 非法 api_code
            else:
                return {"r": "やります", "a": None}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("何かして"))
            sc = result.shadow_comparison
            assert sc is not None
            assert sc["dual_status"] == "invalid_output", \
                "非法 api_code: dual_status='invalid_output', got: {}".format(
                    sc["dual_status"])
            assert sc["raw_agreement"] is False
        finally:
            loop.close()

    def test_shadow_ok_status_on_normal_response(self):
        """正常 action channel 返回 → dual_status='ok'"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 1009}  # 合法
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("座って"))
            sc = result.shadow_comparison
            assert sc is not None
            assert sc["dual_status"] == "ok"
        finally:
            loop.close()


# === B5: SafetyCompiler 在所有模式下验证（Invariant 1） ===

class TestSafetyCompilerIntegration:
    """验证 SafetyCompiler 在 Dual/Shadow 路径仍然生效"""

    def test_router_result_feeds_safety_compiler(self):
        """Dual 模式的 RouterResult 经过 SafetyCompiler 验证"""
        brain = _make_brain()
        brain._router_mode = RouterMode.DUAL
        brain._channel_router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 1030}  # FrontFlip（高风险）
            return {"r": "やります", "a": 1030}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        # 模拟 SafetyCompiler 拒绝高风险动作
        mock_verdict = MagicMock()
        mock_verdict.is_blocked = True
        mock_verdict.block_reason = "high_energy_blocked"
        mock_verdict.response_override = "バッテリー不足です"
        brain.safety_compiler.compile.return_value = mock_verdict

        # 模拟 _log_audit
        brain._log_audit = MagicMock()

        result = brain._apply_safety_to_router_result(
            "前方宙返り",
            RouterResult(
                api_code=1030, sequence=None,
                response="フロントフリップします",
                route=ROUTE_ACTION_CHANNEL,
                request_id="test123",
            ),
            state_snapshot=None,
            snapshot_monotonic_ts=None,
            start_time=time.time(),
        )
        # SafetyCompiler 拒绝了
        assert result.api_code is None
        assert "バッテリー" in result.response
        brain.safety_compiler.compile.assert_called_once()


# === B6: 无效 BRAIN_ROUTER_MODE ===

class TestInvalidRouterMode:
    """验证无效模式降级为 legacy"""

    def test_invalid_mode_in_brain_init(self):
        """无效 BRAIN_ROUTER_MODE → 降级 legacy"""
        with patch.dict(os.environ, {"BRAIN_ROUTER_MODE": "invalid_mode"}):
            with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
                brain = ProductionBrain.__new__(ProductionBrain)
            brain.logger = logging.getLogger("test")

            # 模拟 __init__ 中的路由器初始化逻辑
            router_mode_str = os.getenv("BRAIN_ROUTER_MODE", "legacy")
            try:
                mode = RouterMode(router_mode_str)
            except ValueError:
                mode = RouterMode.LEGACY
            assert mode == RouterMode.LEGACY


# === B7: RouterResult 属性 ===

class TestRouterResultProperties:
    """验证 RouterResult 数据完整性"""

    def test_has_action_with_api_code(self):
        r = RouterResult(api_code=1009, sequence=None, response="",
                         route=ROUTE_LLM_7B)
        assert r.has_action is True

    def test_has_action_with_sequence(self):
        r = RouterResult(api_code=None, sequence=[1004, 1009], response="",
                         route=ROUTE_LLM_7B)
        assert r.has_action is True

    def test_no_action_null(self):
        r = RouterResult(api_code=None, sequence=None, response="test",
                         route=ROUTE_LLM_7B)
        assert r.has_action is False

    def test_no_action_empty_sequence(self):
        r = RouterResult(api_code=None, sequence=[], response="test",
                         route=ROUTE_LLM_7B)
        assert r.has_action is False

    def test_shadow_comparison_optional(self):
        r = RouterResult(api_code=None, sequence=None, response="",
                         route=ROUTE_LLM_7B)
        assert r.shadow_comparison is None

    def test_request_id_default_empty(self):
        r = RouterResult(api_code=None, sequence=None, response="",
                         route=ROUTE_LLM_7B)
        assert r.request_id == ""


# === Fix #4: 序列校验 ===

class TestSequenceValidation:
    """验证 action 通道的序列校验逻辑"""

    def test_sequence_items_validated(self):
        """序列中每项都检查 VALID_API_CODES"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": [1004, 1009]}  # 全部合法
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.sequence == [1004, 1009]
        finally:
            loop.close()

    def test_sequence_invalid_items_filtered(self):
        """非法项被过滤，合法项保留"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": [1004, 9999, 1009]}  # 9999 非法
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.sequence == [1004, 1009]
            assert result.route == ROUTE_ACTION_CHANNEL
        finally:
            loop.close()

    def test_sequence_all_invalid_fallback(self):
        """序列全部非法 → 回退 legacy"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": [9999, 8888]}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.route == ROUTE_ACTION_FALLBACK
        finally:
            loop.close()

    def test_sequence_length_limit(self):
        """超过 MAX_SEQUENCE_LENGTH 的序列被截断"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        long_seq = [1004, 1009, 1016, 1017, 1022]  # 5 项
        assert len(long_seq) > MAX_SEQUENCE_LENGTH

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": long_seq}
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert len(result.sequence) == MAX_SEQUENCE_LENGTH
            assert result.sequence == long_seq[:MAX_SEQUENCE_LENGTH]
        finally:
            loop.close()

    def test_sequence_empty_fallback(self):
        """空序列 [] → 回退 legacy → legacy 有动作则 ROUTE_ACTION_FALLBACK"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": []}
            else:
                return {"r": "座ります", "a": 1009}  # legacy 返回有效动作

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            # action 通道空序列 → 回退 legacy → legacy 有动作 → 模板响应
            assert result.route == ROUTE_ACTION_FALLBACK
            assert result.api_code == 1009
        finally:
            loop.close()

    def test_sequence_non_list_fallback(self):
        """s 不是列表 → 回退"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"s": "not_a_list"}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.route == ROUTE_ACTION_FALLBACK
        finally:
            loop.close()


# === Shadow 高风险分歧检测（Fix #3）===

class TestShadowHighRiskDivergence:
    """验证 shadow_comparison 的 high_risk_divergence 检测"""

    def test_high_risk_divergence_flagged(self):
        """legacy 选高风险动作，dual 不选 → high_risk_divergence=True"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 1009}  # dual: Sit（安全）
            else:
                return {"r": "フリップ!", "a": 1030}  # legacy: FrontFlip（高风险）

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            sc = result.shadow_comparison
            assert sc["high_risk_divergence"] is True
        finally:
            loop.close()

    def test_no_divergence_both_safe(self):
        """两侧都选安全动作 → high_risk_divergence=False"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v1":
                return {"a": 1009}  # Sit
            else:
                return {"r": "立ちます", "a": 1004}  # StandUp

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            sc = result.shadow_comparison
            assert sc["high_risk_divergence"] is False
        finally:
            loop.close()


# === 审查修复: Fix #1 — Action 通道 token 预算 ===

class TestActionChannelTokenBudget:
    """Fix #1: Action 通道使用 num_predict=30, num_ctx=1024"""

    def test_action_channel_passes_low_token_budget(self):
        """_action_channel() 调用 _call_ollama_v2 时传入紧凑参数"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        captured_kwargs = {}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            captured_kwargs['num_predict'] = num_predict
            captured_kwargs['num_ctx'] = num_ctx
            return {"a": 1009}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(router._action_channel("おすわり", "test1"))
            assert captured_kwargs['num_predict'] == 30, (
                "Action 通道应传 num_predict=30, 实际={}".format(
                    captured_kwargs['num_predict']))
            assert captured_kwargs['num_ctx'] == 1024, (
                "Action 通道应传 num_ctx=1024, 实际={}".format(
                    captured_kwargs['num_ctx']))
        finally:
            loop.close()

    def test_legacy_route_uses_default_budget(self):
        """_legacy_route() 调用 _call_ollama_v2 时使用默认参数"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        captured_kwargs = {}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            captured_kwargs['num_predict'] = num_predict
            captured_kwargs['num_ctx'] = num_ctx
            return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(router._legacy_route("おすわり", "test1"))
            assert captured_kwargs['num_predict'] == 100, (
                "Legacy 路由应用默认 num_predict=100, 实际={}".format(
                    captured_kwargs['num_predict']))
            assert captured_kwargs['num_ctx'] == 2048, (
                "Legacy 路由应用默认 num_ctx=2048, 实际={}".format(
                    captured_kwargs['num_ctx']))
        finally:
            loop.close()


# === 审查修复: Fix #2 — a+s 冲突归一化 ===

class TestActionSequenceConflict:
    """Fix #2: a 和 s 同时出现时 s 优先"""

    def test_both_a_and_s_present_s_wins(self):
        """a=1009 + s=[1004,1016] 同时出现 → s 优先，a 被清除"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            if model == "claudia-action-v1":
                return {"a": 1009, "s": [1004, 1016]}  # 冲突!
            return {"r": "ok", "a": None}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            # s 优先: sequence 保留，api_code 被清除
            assert result.sequence == [1004, 1016]
            assert result.api_code is None
            # 模板响应应基于 sequence，不是 api_code
            assert "立ちます" in result.response
            assert "挨拶します" in result.response
        finally:
            loop.close()

    def test_only_a_present_works_normally(self):
        """只有 a，无 s → 正常处理"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            if model == "claudia-action-v1":
                return {"a": 1009}
            return {"r": "ok", "a": None}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.api_code == 1009
            assert result.sequence is None
        finally:
            loop.close()


# === 审查修复: Fix #3 — 避免双重 legacy 调用 ===

class TestNoDoubleLegacyCall:
    """Fix #3: action fallback 后不再重复调用 legacy"""

    def test_action_fail_legacy_conversational_no_double_call(self):
        """Action 模型失败 → legacy 返回纯对话 → 不再调 _voice_fallback"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)
        call_count = {"legacy": 0}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            if model == "claudia-action-v1":
                return None  # action 模型失败
            else:
                call_count["legacy"] += 1
                return {"r": "今日はいい天気ですね", "a": None}  # 纯对话

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("天気について"))
            # legacy 应该只被调用 1 次（action fallback），不是 2 次
            assert call_count["legacy"] == 1, (
                "Legacy LLM 应只调用 1 次, 实际 {} 次".format(call_count["legacy"]))
            assert result.route == ROUTE_ACTION_FALLBACK
            assert "天気" in result.response
        finally:
            loop.close()

    def test_action_null_normal_voice_fallback_still_works(self):
        """Action 正常返回 a=null → 走 voice_fallback（这不是 fallback 路由）"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)
        call_count = {"total": 0}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            call_count["total"] += 1
            if model == "claudia-action-v1":
                return {"a": None}  # 正常: 无动作
            else:
                return {"r": "今日はいい天気ですね", "a": None}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("天気について"))
            # action(1次) + voice_fallback(1次) = 2 次 LLM 调用（正常）
            assert call_count["total"] == 2
            assert result.route == ROUTE_VOICE_CHANNEL
        finally:
            loop.close()

    def test_action_fallback_with_action_no_extra_voice(self):
        """Action 失败 → legacy 返回有动作 → 直接使用，无额外 voice 调用"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)
        call_count = {"legacy": 0}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048):
            if model == "claudia-action-v1":
                return None  # action 失败
            else:
                call_count["legacy"] += 1
                return {"r": "座ります", "a": 1009}  # legacy 返回有动作

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            # legacy 只调用 1 次（fallback），有动作所以不进 voice
            assert call_count["legacy"] == 1
            assert result.route == ROUTE_ACTION_FALLBACK
            assert result.api_code == 1009
        finally:
            loop.close()


# === 审查修复: Fix #4 — 审计 success 语义 ===

class TestAuditSuccessSemantics:
    """Fix #4: success 反映流水线成功，不是'有无动作'"""

    def test_conversational_is_success(self):
        """纯对话命令（无动作）→ success=True"""
        brain = _make_brain()
        brain._log_audit = MagicMock()
        brain.audit_logger = MagicMock()

        from claudia.brain.audit_logger import AuditEntry
        from dataclasses import asdict

        # 构造纯对话 output（无 api_code/sequence）
        output = BrainOutput(response="今日はいい天気ですね", api_code=None)

        # 手动调用 _log_audit 并检查 success 计算
        # 重新启用 _log_audit（去掉 mock）
        brain._log_audit = ProductionBrain._log_audit.__get__(brain)

        brain._log_audit(
            "天気について", output, route=ROUTE_LLM_7B,
            elapsed_ms=100.0, cache_hit=False,
            model_used="7b", current_state=None,
            llm_output='{"r":"今日はいい天気ですね","a":null}',
            safety_verdict="ok",
        )

        # 验证 audit_logger.log_entry 被调用
        brain.audit_logger.log_entry.assert_called_once()
        entry = brain.audit_logger.log_entry.call_args[0][0]
        # 关键: 纯对话应 success=True（流水线正常完成）
        assert entry.success is True, (
            "纯对话命令应 success=True, 实际={}".format(entry.success))

    def test_action_is_success(self):
        """有动作命令 → success=True"""
        brain = _make_brain()
        brain.audit_logger = MagicMock()
        brain._log_audit = ProductionBrain._log_audit.__get__(brain)

        output = BrainOutput(response="座ります", api_code=1009)
        brain._log_audit(
            "おすわり", output, route=ROUTE_LLM_7B,
            elapsed_ms=100.0, cache_hit=False,
            model_used="7b", current_state=None,
            llm_output='{"r":"座ります","a":1009}',
            safety_verdict="ok",
        )

        entry = brain.audit_logger.log_entry.call_args[0][0]
        assert entry.success is True

    def test_safety_rejected_is_success(self):
        """安全拒绝 → success=True（系统正确地拒绝了危险动作）"""
        brain = _make_brain()
        brain.audit_logger = MagicMock()
        brain._log_audit = ProductionBrain._log_audit.__get__(brain)

        output = BrainOutput(
            response="バッテリー不足です", api_code=None,
            reasoning="router_safety_rejected")
        brain._log_audit(
            "フリップして", output, route=ROUTE_ACTION_CHANNEL,
            elapsed_ms=100.0, cache_hit=False,
            model_used="dual", current_state=None,
            llm_output='{"a":1030}',
            safety_verdict="rejected:high_energy_blocked",
        )

        entry = brain.audit_logger.log_entry.call_args[0][0]
        # 安全拒绝也是 success=True（流水线正常运行，正确拒绝了）
        assert entry.success is True, (
            "安全拒绝应 success=True, 实际={}".format(entry.success))
