#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_commander_wakeup.py — 唤醒动画单元测试

验证:
  - 环境变量门控（默认关闭，COMMANDER_WAKEUP_ANIMATION=1 启用）
  - 非硬件模式跳过
  - StandUp 成功 (code=0) → Stretch → 姿态更新
  - StandUp 已站立 (code=-1) → Stretch → 姿态更新
  - StandUp 超时 (code=3104) → _verify_standing_after_unknown → 条件继续
  - StandUp 失败 (code=3103) → 跳过 Stretch
  - 异常不阻塞启动
  - 审计日志写入
"""

import sys
import os
import asyncio
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import MagicMock, AsyncMock, patch, call
from types import SimpleNamespace


def _run(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _make_commander(use_real_hardware=True, sport_client=True, router_mode="legacy"):
    """构建 mock Commander 实例"""
    from production_commander import ProductionCommander

    with patch.object(ProductionCommander, '__init__', lambda self, **kw: None):
        cmd = ProductionCommander()

    cmd.brain = MagicMock()
    cmd.brain.use_real_hardware = use_real_hardware
    cmd.brain.sport_client = MagicMock() if sport_client else None
    cmd.brain._update_posture_tracking = MagicMock()
    cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=True)
    cmd.brain.model_7b = "test-model"

    # router mode
    from src.claudia.brain.channel_router import RouterMode
    cmd.brain._router_mode = RouterMode(router_mode) if router_mode != "legacy" else RouterMode.LEGACY
    cmd.brain._channel_router = MagicMock()
    cmd.brain._channel_router._action_model = "test-action-model"

    return cmd


# === 门控测试 ===

class TestWakeupGating:
    """环境变量和硬件模式门控"""

    def test_skipped_when_env_not_set(self):
        """默认不设置 COMMANDER_WAKEUP_ANIMATION 时跳过动画"""
        cmd = _make_commander()
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)
        _run(cmd._wakeup_animation())
        cmd.brain._rpc_call.assert_not_called()

    def test_skipped_when_env_is_zero(self):
        """COMMANDER_WAKEUP_ANIMATION=0 时跳过动画"""
        cmd = _make_commander()
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "0"
        try:
            _run(cmd._wakeup_animation())
            cmd.brain._rpc_call.assert_not_called()
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_skipped_when_not_hardware(self):
        """模拟模式跳过动画"""
        cmd = _make_commander(use_real_hardware=False)
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"
        try:
            _run(cmd._wakeup_animation())
            cmd.brain._rpc_call.assert_not_called()
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_skipped_when_no_sport_client(self):
        """SportClient 不可用时跳过动画"""
        cmd = _make_commander(sport_client=False)
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"
        try:
            _run(cmd._wakeup_animation())
            cmd.brain._rpc_call.assert_not_called()
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_enabled_when_env_is_one(self):
        """COMMANDER_WAKEUP_ANIMATION=1 时执行动画"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 0]  # StandUp, Stretch
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"
        try:
            with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
                with patch.object(cmd, '_log_wakeup_audit'):
                    _run(cmd._wakeup_animation())
            assert cmd.brain._rpc_call.call_count == 2
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)


# === StandUp 返回码分支 ===

class TestWakeupStandUpBranches:
    """StandUp 返回码的不同处理路径"""

    def setup_method(self):
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"

    def teardown_method(self):
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_standup_success_updates_posture(self):
        """StandUp code=0 → 更新姿态 + 执行 Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 0]  # StandUp=0, Stretch=0
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)
        assert cmd.brain._rpc_call.call_count == 2

    def test_standup_already_standing_updates_posture(self):
        """StandUp code=-1 (已站立) → 更新姿态 + 执行 Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [-1, 0]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)
        assert cmd.brain._rpc_call.call_count == 2

    def test_standup_3104_verifies_then_continues(self):
        """StandUp code=3104 → _verify_standing_after_unknown → 确认后继续"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3104, 0]  # StandUp=3104, Stretch=0
        cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=True)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._verify_standing_after_unknown.assert_called_once()
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)
        assert cmd.brain._rpc_call.call_count == 2

    def test_standup_3104_unconfirmed_skips_stretch(self):
        """StandUp code=3104 + 未确认站立 → 跳过 Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3104]
        cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=False)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_not_called()
        assert cmd.brain._rpc_call.call_count == 1  # 只调了 StandUp

    def test_standup_failure_skips_stretch(self):
        """StandUp code=3103 (失败) → 跳过 Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3103]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_not_called()
        assert cmd.brain._rpc_call.call_count == 1

    def test_standup_tuple_return(self):
        """_rpc_call 返回 tuple (code, data) 时正确提取 code"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [(0, {}), (0, {})]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)


# === 异常安全 ===

class TestWakeupExceptionSafety:
    """异常不阻塞启动"""

    def setup_method(self):
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"

    def teardown_method(self):
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_rpc_exception_does_not_propagate(self):
        """_rpc_call 异常被捕获，不阻塞启动"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = RuntimeError("DDS 断连")
        with patch.object(cmd, '_log_wakeup_audit'):
            _run(cmd._wakeup_animation())  # 不应抛出


# === 审计日志 ===

class TestWakeupAudit:
    """审计日志记录"""

    def setup_method(self):
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"

    def teardown_method(self):
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_audit_logged_on_success(self):
        """成功完成后记录审计"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 0]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.route == "startup"
                assert entry.input_command == "__wakeup__"
                assert entry.success is True
                assert entry.sequence == [1004, 1017]

    def test_audit_logged_on_standup_failure(self):
        """StandUp 失败后也记录审计"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3103]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.success is False


# === Action 模型预热 ===

class TestActionModelWarmup:
    """Dual/Shadow 模式下预热 Action 模型"""

    def test_legacy_mode_no_action_warmup(self):
        """Legacy 模式只预热 7B 模型"""
        cmd = _make_commander(router_mode="legacy")
        call_count = 0

        def mock_chat(**kwargs):
            nonlocal call_count
            call_count += 1
            return {'message': {'content': '{}'}}

        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.dict('sys.modules', {'ollama': MagicMock(chat=mock_chat)}):
                _run(cmd._warmup_model())

        assert call_count == 1  # 只预热了 7B

    def test_shadow_mode_warms_action_model(self):
        """Shadow 模式预热 7B + Action 模型"""
        cmd = _make_commander(router_mode="shadow")
        warmed_models = []

        def mock_chat(**kwargs):
            warmed_models.append(kwargs.get('model'))
            return {'message': {'content': '{}'}}

        with patch.dict('sys.modules', {'ollama': MagicMock(chat=mock_chat)}):
            _run(cmd._warmup_model())

        assert len(warmed_models) == 2
        assert warmed_models[0] == "test-model"
        assert warmed_models[1] == "test-action-model"
