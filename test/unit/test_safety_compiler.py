#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_safety_compiler.py — SafetyCompiler 单元测试

验证统一安全管线的全部路径:
  - Step 0a: 输入契约（电量校验）
  - Step 0b: 状态新鲜度
  - Step 1: 白名单
  - Step 2: 高风险策略门控
  - Step 3: 电量门控（3 阶梯: 0.10/0.20/0.30）
  - Step 4: 站立前置
  - Step 5: 序列截断（首拒/中拒/降级）
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.safety_compiler import SafetyCompiler, SafetyVerdict
from claudia.brain.action_registry import (
    EXECUTABLE_API_CODES, VALID_API_CODES,
    HIGH_ENERGY_ACTIONS, REQUIRE_STANDING,
)


def make_compiler(**kwargs):
    """创建 SafetyCompiler 实例（默认 allow_high_risk=False）"""
    defaults = dict(downgrade_target=1023, snapshot_max_age=5.0, allow_high_risk=False)
    defaults.update(kwargs)
    return SafetyCompiler(**defaults)


# === Step 0a: 输入契约 ===

class TestInputContract:
    """电量输入校验 — fail-safe 拒绝"""

    def test_battery_none_blocked(self):
        """battery_level=None → is_blocked"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=None, is_standing=True)
        assert v.is_blocked
        assert "缺失" in v.block_reason

    def test_battery_string_blocked(self):
        """battery_level='high' → is_blocked"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level="high", is_standing=True)
        assert v.is_blocked

    def test_battery_over_1_blocked(self):
        """battery_level=80.0 → is_blocked（不自动 /100 修正）"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=80.0, is_standing=True)
        assert v.is_blocked
        assert "1.0" in v.block_reason
        assert v.response_override is not None

    def test_battery_exactly_1_passes(self):
        """battery_level=1.0 → 不 blocked"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=1.0, is_standing=True)
        assert not v.is_blocked
        assert 1004 in v.executable_sequence

    def test_battery_negative_clamped(self):
        """battery_level=-0.1 → 修正为 0.0，有 warning"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=-0.1, is_standing=True)
        assert not v.is_blocked
        assert any("< 0" in w for w in v.warnings)
        # 1004 是 SAFE_ACTIONS，在 battery=0.0 下仍可通过
        assert 1004 in v.executable_sequence

    def test_battery_1_01_blocked(self):
        """battery_level=1.01 → is_blocked (边界)"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=1.01, is_standing=True)
        assert v.is_blocked


# === Step 0b: 状态新鲜度 ===

class TestSnapshotFreshness:
    """快照新鲜度检查"""

    def test_fresh_snapshot_passes(self):
        """新鲜快照（1s 前）不影响"""
        sc = make_compiler()
        ts = time.monotonic() - 1.0  # 1 秒前
        v = sc.compile([1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=ts)
        assert not v.is_blocked
        assert 1016 in v.executable_sequence

    def test_stale_snapshot_filters_to_safe(self):
        """过期快照 → 非安全动作被过滤，安全动作保留"""
        sc = make_compiler()
        ts = time.monotonic() - 10.0  # 10 秒前（> 5s 阈值）
        # 1004 是安全动作，1016(Hello) 需站立（非安全类别中）
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=ts)
        assert not v.is_blocked
        assert 1004 in v.executable_sequence
        # 1016 不在 SAFE_ACTIONS 中，应被过滤掉
        assert 1016 not in v.executable_sequence
        assert any("过期" in w for w in v.warnings)

    def test_stale_snapshot_all_unsafe_blocked(self):
        """过期快照 + 全部非安全动作 → is_blocked"""
        sc = make_compiler()
        ts = time.monotonic() - 10.0
        v = sc.compile([1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=ts)
        assert v.is_blocked
        assert "过期" in v.block_reason

    def test_no_timestamp_skips_check(self):
        """snapshot_timestamp=None → 跳过新鲜度检查"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=None)
        assert not v.is_blocked
        assert 1016 in v.executable_sequence


# === Step 1: 白名单 ===

class TestWhitelist:
    """白名单校验"""

    def test_valid_action_passes(self):
        """正常动作通过白名单"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]

    def test_invalid_first_action_blocked(self):
        """首动作不在白名单 → 整个请求被拒"""
        sc = make_compiler()
        v = sc.compile([9999], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert 9999 in v.rejected
        assert "首动作" in v.block_reason

    def test_invalid_mid_action_truncated(self):
        """序列中间非法 → 截断保留已通过部分"""
        sc = make_compiler()
        v = sc.compile([1004, 9999, 1005], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 9999 in v.rejected

    def test_disabled_action_blocked(self):
        """禁用动作（1042 LeftFlip）不在白名单"""
        sc = make_compiler()
        assert 1042 not in EXECUTABLE_API_CODES
        v = sc.compile([1042], battery_level=0.80, is_standing=True)
        assert v.is_blocked

    def test_parameterized_with_safe_defaults_passes(self):
        """Pose(1028) 有 safe_default_params → 在 EXECUTABLE_API_CODES 中"""
        sc = make_compiler()
        assert 1028 in EXECUTABLE_API_CODES
        assert 1028 not in VALID_API_CODES  # 但不在 LLM 白名单中
        v = sc.compile([1028], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert 1028 in v.executable_sequence

    def test_parameterized_without_defaults_blocked(self):
        """Move(1008) 无 safe_default_params → 不在 EXECUTABLE_API_CODES 中"""
        sc = make_compiler()
        assert 1008 not in EXECUTABLE_API_CODES
        v = sc.compile([1008], battery_level=0.80, is_standing=True)
        assert v.is_blocked


# === Step 2: 高风险策略门控 ===

class TestHighRiskGate:
    """高风险动作策略门控"""

    def test_high_risk_blocked_by_default(self):
        """allow_high_risk=False → FrontFlip(1030) 被拒"""
        sc = make_compiler(allow_high_risk=False)
        v = sc.compile([1030], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert 1030 in v.rejected
        assert "高リスク" in v.block_reason

    def test_high_risk_allowed_when_enabled(self):
        """allow_high_risk=True → FrontFlip(1030) 通过"""
        sc = make_compiler(allow_high_risk=True)
        v = sc.compile([1030], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert 1030 in v.executable_sequence

    def test_all_high_energy_blocked(self):
        """所有 HIGH_ENERGY_ACTIONS 被默认策略拒绝"""
        sc = make_compiler(allow_high_risk=False)
        for code in HIGH_ENERGY_ACTIONS:
            if code in EXECUTABLE_API_CODES:  # 只测已启用的
                v = sc.compile([code], battery_level=0.80, is_standing=True)
                assert v.is_blocked, "HIGH_ENERGY {} 应被拒绝".format(code)

    def test_high_risk_mid_sequence_truncated(self):
        """序列中间遇到高风险 → 截断"""
        sc = make_compiler(allow_high_risk=False)
        v = sc.compile([1004, 1030], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 1030 in v.rejected


# === Step 3: 电量门控 ===

class TestBatteryGate:
    """电量门控三阶梯"""

    # --- 阶梯 1: ≤0.10 仅安全动作 ---

    def test_critical_battery_safe_action_passes(self):
        """battery=0.08, StandUp(1004) → 通过（安全动作）"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.08, is_standing=False)
        assert not v.is_blocked
        assert 1004 in v.executable_sequence

    def test_critical_battery_unsafe_action_blocked(self):
        """battery=0.08, Hello(1016) → 拒绝（非安全动作）"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.08, is_standing=True)
        assert v.is_blocked
        assert "电量" in v.block_reason

    def test_critical_battery_dance_blocked(self):
        """battery=0.05, Dance1(1022) → 拒绝"""
        sc = make_compiler()
        v = sc.compile([1022], battery_level=0.05, is_standing=True)
        assert v.is_blocked

    # --- 阶梯 2: ≤0.20 禁高能 ---

    def test_low_battery_normal_action_passes(self):
        """battery=0.15, Hello(1016) → 通过（非高能）"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.15, is_standing=True)
        assert not v.is_blocked
        assert 1016 in v.executable_sequence

    def test_low_battery_high_energy_blocked(self):
        """battery=0.15, FrontFlip(1030) → 拒绝（高能+低电量双重门控）"""
        sc = make_compiler(allow_high_risk=True)  # 允许高风险才能到电量门控
        v = sc.compile([1030], battery_level=0.15, is_standing=True)
        assert v.is_blocked

    # --- 阶梯 3: ≤0.30 高能降级 ---

    def test_moderate_battery_high_energy_downgraded(self):
        """battery=0.25, FrontFlip(1030) → 降级为 Dance2(1023)"""
        sc = make_compiler(allow_high_risk=True)  # 先过高风险门控
        v = sc.compile([1030], battery_level=0.25, is_standing=True)
        assert not v.is_blocked
        assert 1023 in v.executable_sequence  # downgrade_target
        assert 1030 not in v.executable_sequence
        assert any("降级" in w for w in v.warnings)

    def test_custom_downgrade_target(self):
        """自定义降级目标: 1022(Dance1)"""
        sc = make_compiler(allow_high_risk=True, downgrade_target=1022)
        v = sc.compile([1030], battery_level=0.25, is_standing=True)
        assert 1022 in v.executable_sequence

    # --- 充足电量 ---

    def test_full_battery_all_passes(self):
        """battery=0.80 → 所有正常动作通过"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]

    # --- 边界值 ---

    def test_boundary_0_10_safe_passes(self):
        """battery=0.10, 安全动作通过"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.10, is_standing=False)
        assert not v.is_blocked

    def test_boundary_0_10_unsafe_blocked(self):
        """battery=0.10, 非安全动作被拒（<=0.10）"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.10, is_standing=True)
        assert v.is_blocked

    def test_boundary_0_11_unsafe_passes(self):
        """battery=0.11, 非安全非高能动作通过（>0.10 且 <=0.20 阶梯只禁高能）"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.11, is_standing=True)
        assert not v.is_blocked


# === Step 4: 站立前置 ===

class TestStandingPrerequisite:
    """站立前置自动插入"""

    def test_standing_action_auto_prepend(self):
        """不站立 + 需站立动作 → 自动前插 StandUp(1004)"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=False)
        assert not v.is_blocked
        assert v.executable_sequence[0] == 1004  # auto StandUp
        assert v.executable_sequence[1] == 1016
        assert 1004 in v.auto_prepend

    def test_already_standing_no_prepend(self):
        """已站立 → 不前插"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=True)
        assert v.executable_sequence == [1016]
        assert not v.auto_prepend

    def test_non_standing_action_no_prepend(self):
        """不需要站立的动作 → 不前插"""
        sc = make_compiler()
        assert 1021 not in REQUIRE_STANDING  # Wallow 不需站立
        v = sc.compile([1021], battery_level=0.80, is_standing=False)
        assert v.executable_sequence == [1021]
        assert not v.auto_prepend

    def test_sequence_single_prepend(self):
        """序列中多个需站立动作 → 只前插一次 StandUp"""
        sc = make_compiler()
        v = sc.compile([1016, 1017], battery_level=0.80, is_standing=False)
        assert v.executable_sequence.count(1004) == 1
        assert v.executable_sequence == [1004, 1016, 1017]

    def test_standdown_no_prepend(self):
        """StandDown(1005) 不需站立 → 不前插"""
        sc = make_compiler()
        assert 1005 not in REQUIRE_STANDING
        v = sc.compile([1005], battery_level=0.80, is_standing=True)
        assert v.executable_sequence == [1005]
        assert not v.auto_prepend


# === Step 5: 序列编译 ===

class TestSequenceCompilation:
    """序列编译（截断/降级/合并）"""

    def test_normal_sequence(self):
        """正常序列全部通过"""
        sc = make_compiler()
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]

    def test_first_rejected_blocks_all(self):
        """首动作被拒 → 整个序列被拒"""
        sc = make_compiler()
        v = sc.compile([9999, 1004], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert v.executable_sequence == []

    def test_mid_rejected_truncates(self):
        """中间被拒 → 截断保留前面"""
        sc = make_compiler()
        v = sc.compile([1004, 9999, 1005], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]

    def test_mid_battery_rejected_truncates(self):
        """中间电量拒绝 → 截断"""
        sc = make_compiler()
        # battery=0.08 → 1004(安全)通过, 1016(非安全)被拒
        v = sc.compile([1004, 1016], battery_level=0.08, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 1016 in v.rejected

    def test_mid_downgrade_replaces(self):
        """中间降级 → 替换后继续"""
        sc = make_compiler(allow_high_risk=True)
        # battery=0.25 → 1004 通过, 1030(高能) 降级为 1023
        v = sc.compile([1004, 1030], battery_level=0.25, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1023]  # 1030 降级为 1023

    def test_empty_actions(self):
        """空动作列表 → 空结果，不 blocked"""
        sc = make_compiler()
        v = sc.compile([], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == []

    def test_sequence_with_prepend_and_downgrade(self):
        """复合场景: 站立前置 + 降级"""
        sc = make_compiler(allow_high_risk=True)
        # 不站立, battery=0.25, [1030(高能需站立)] →
        # 降级为 1023(Dance2, 也需站立) → 前插 StandUp
        v = sc.compile([1030], battery_level=0.25, is_standing=False)
        assert not v.is_blocked
        assert v.executable_sequence[0] == 1004  # auto StandUp
        assert 1023 in v.executable_sequence      # downgraded


# === SafetyVerdict 数据完整性 ===

class TestVerdictIntegrity:
    """SafetyVerdict 字段正确性"""

    def test_blocked_has_reason_and_override(self):
        """被拒 → 有 block_reason + response_override"""
        sc = make_compiler()
        v = sc.compile([9999], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert len(v.block_reason) > 0
        assert v.response_override is not None
        assert len(v.response_override) > 0

    def test_passed_has_no_override(self):
        """通过 → response_override=None"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.response_override is None

    def test_warnings_accumulate(self):
        """多个触发条件 → warnings 累加"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=-0.5, is_standing=False)
        # battery<0 修正 + battery<=0.10 拒绝
        assert len(v.warnings) >= 1

    def test_default_verdict_clean(self):
        """默认 SafetyVerdict 所有字段干净"""
        v = SafetyVerdict()
        assert v.executable_sequence == []
        assert v.auto_prepend == []
        assert v.rejected == []
        assert v.warnings == []
        assert not v.is_blocked
        assert v.block_reason == ""
        assert v.response_override is None
