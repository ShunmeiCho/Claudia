#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_commander_migration.py — PR2 Slice A: Commander 迁移验证

验证:
  - A1: Commander 使用 process_and_execute（不再直接调用 process_command + execute_action）
  - A2: execution_status 完整性（success/unknown/failed/skipped）
  - A3: 软废弃警告（contextvars 协程安全）
"""

import sys
import os
import asyncio
import logging
import re

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import AsyncMock, MagicMock, patch
from claudia.brain.production_brain import ProductionBrain, BrainOutput, _pae_depth


# === A1: Commander 源码级检查 ===

class TestCommanderSourceMigration:
    """确认 commander 不再直接调用 process_command + execute_action"""

    def test_commander_uses_process_and_execute(self):
        """production_commander.py 中不应有 self.brain.process_command 的直接调用"""
        commander_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'production_commander.py'
        )
        with open(commander_path, 'r', encoding='utf-8') as f:
            source = f.read()

        # 不应有直调 self.brain.process_command（warmup 注释中的引用除外）
        # 排除注释行和 warmup 方法中的说明性注释
        lines = source.split('\n')
        violations = []
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            # 跳过注释
            if stripped.startswith('#') or stripped.startswith('//'):
                continue
            # 检查直接调用（不是方法定义）
            if 'self.brain.process_command(' in stripped:
                violations.append(f"line {i}: {stripped}")

        assert not violations, (
            "Commander 仍直接调用 brain.process_command:\n"
            + "\n".join(violations)
        )

    def test_commander_no_direct_execute_action(self):
        """production_commander.py 中不应有 self.brain.execute_action 的直接调用"""
        commander_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'production_commander.py'
        )
        with open(commander_path, 'r', encoding='utf-8') as f:
            source = f.read()

        lines = source.split('\n')
        violations = []
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            if stripped.startswith('#'):
                continue
            if 'self.brain.execute_action(' in stripped:
                violations.append(f"line {i}: {stripped}")

        assert not violations, (
            "Commander 仍直接调用 brain.execute_action:\n"
            + "\n".join(violations)
        )


# === A2: execution_status 完整性 ===

class TestExecutionStatus:
    """验证 process_and_execute 的 execution_status 语义"""

    def _make_brain(self):
        """创建最小化 mock brain 用于测试"""
        with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
            brain = ProductionBrain.__new__(ProductionBrain)
        brain.logger = logging.getLogger("test")
        brain._command_lock = asyncio.Lock()
        brain.sport_client = None
        brain.use_real_hardware = False
        brain.EMERGENCY_COMMANDS = {
            "止まれ": "止まります",
            "stop": "止まります",
        }
        return brain

    def test_execution_status_skipped_for_text_only(self):
        """纯文本回复（无 api_code/sequence）→ execution_status='skipped'"""
        brain = self._make_brain()
        text_output = BrainOutput(response="こんにちは")

        async def mock_process_command(cmd):
            return text_output

        brain.process_command = mock_process_command

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("hello"))
            assert result.execution_status == "skipped", (
                f"Expected 'skipped', got '{result.execution_status}'"
            )
        finally:
            loop.close()

    def test_execution_status_success(self):
        """动作执行成功 → execution_status='success'"""
        brain = self._make_brain()
        action_output = BrainOutput(response="座ります", api_code=1009)

        async def mock_process_command(cmd):
            return action_output

        async def mock_execute_action(output):
            return True

        brain.process_command = mock_process_command
        brain.execute_action = mock_execute_action

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("おすわり"))
            assert result.execution_status == "success"
        finally:
            loop.close()

    def test_execution_status_unknown(self):
        """动作超时 → execution_status='unknown'"""
        brain = self._make_brain()
        action_output = BrainOutput(response="ダンスします", api_code=1022)

        async def mock_process_command(cmd):
            return action_output

        async def mock_execute_action(output):
            return "unknown"

        brain.process_command = mock_process_command
        brain.execute_action = mock_execute_action

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("踊って"))
            assert result.execution_status == "unknown"
        finally:
            loop.close()

    def test_execution_status_failed(self):
        """动作执行失败 → execution_status='failed'"""
        brain = self._make_brain()
        action_output = BrainOutput(response="立ちます", api_code=1004)

        async def mock_process_command(cmd):
            return action_output

        async def mock_execute_action(output):
            return False

        brain.process_command = mock_process_command
        brain.execute_action = mock_execute_action

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("立って"))
            assert result.execution_status == "failed"
        finally:
            loop.close()

    def test_emergency_sets_execution_status(self):
        """紧急停止 → execution_status 不为 None"""
        brain = self._make_brain()

        async def mock_handle_emergency(cmd):
            return BrainOutput(
                response="緊急停止しました",
                api_code=1003,
                execution_status="success",
            )

        brain._handle_emergency = mock_handle_emergency

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("止まれ"))
            assert result.execution_status is not None
        finally:
            loop.close()


# === A3: 软废弃警告（contextvars 安全） ===

class TestDeprecationWarning:
    """验证 process_command 在 process_and_execute 外调用时产生警告"""

    def _make_brain(self):
        with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
            brain = ProductionBrain.__new__(ProductionBrain)
        brain.logger = logging.getLogger("test_deprecation")
        brain._command_lock = asyncio.Lock()
        brain.sport_client = None
        brain.use_real_hardware = False
        brain.safety_compiler = None
        brain.state_monitor = None
        brain.audit_logger = None
        brain.model_7b = "test-model"
        brain.last_posture_standing = False
        brain.EMERGENCY_COMMANDS = {}
        return brain

    def test_deprecation_warning_logged(self):
        """直接调用 process_command 应产生警告日志"""
        # 确保 _pae_depth 为 0（非 process_and_execute 上下文）
        assert _pae_depth.get(0) == 0, "Test precondition: _pae_depth should be 0"

        brain = self._make_brain()
        with patch.object(brain.logger, 'warning') as mock_warn:
            # 由于 process_command 内部逻辑复杂，mock 掉后续部分
            # 只需验证第一行的警告被触发
            try:
                loop = asyncio.new_event_loop()
                loop.run_until_complete(brain.process_command("test"))
            except Exception:
                pass  # process_command 内部会因缺少其他属性而失败
            finally:
                loop.close()

            # 检查是否有警告
            warn_calls = [
                str(c) for c in mock_warn.call_args_list
                if 'process_and_execute' in str(c)
            ]
            assert len(warn_calls) > 0, (
                "Expected deprecation warning about process_and_execute, "
                f"got calls: {mock_warn.call_args_list}"
            )

    def test_no_warning_inside_pae_context(self):
        """在 process_and_execute 内调用 process_command 不应产生警告"""
        brain = self._make_brain()

        # 模拟 process_and_execute 的上下文
        token = _pae_depth.set(1)
        try:
            with patch.object(brain.logger, 'warning') as mock_warn:
                try:
                    loop = asyncio.new_event_loop()
                    loop.run_until_complete(brain.process_command("test"))
                except Exception:
                    pass
                finally:
                    loop.close()

                # 不应有 process_and_execute 相关的警告
                pae_warns = [
                    str(c) for c in mock_warn.call_args_list
                    if 'process_and_execute' in str(c)
                ]
                assert len(pae_warns) == 0, (
                    f"Unexpected deprecation warning inside PAE context: {pae_warns}"
                )
        finally:
            _pae_depth.reset(token)

    def test_contextvars_coroutine_isolation(self):
        """不同协程的 _pae_depth 互不干扰"""
        results = {}

        async def check_depth(name, expected):
            results[name] = _pae_depth.get(0) == expected

        async def run():
            # 主协程设置 depth=1
            token = _pae_depth.set(1)
            try:
                # 子任务应继承 depth=1
                task1 = asyncio.ensure_future(check_depth("inherited", 1))
                await task1

                # 子任务内部修改不影响父
                async def modify_and_check():
                    inner_token = _pae_depth.set(99)
                    results["inner"] = _pae_depth.get(0) == 99
                    _pae_depth.reset(inner_token)

                task2 = asyncio.ensure_future(modify_and_check())
                await task2

                # 父协程 depth 仍为 1
                results["parent_unchanged"] = _pae_depth.get(0) == 1
            finally:
                _pae_depth.reset(token)

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(run())
        finally:
            loop.close()

        assert results.get("inherited"), "Child task should inherit parent's _pae_depth"
        assert results.get("inner"), "Inner task should see its own modified _pae_depth"
        assert results.get("parent_unchanged"), "Parent's _pae_depth should be unchanged"
