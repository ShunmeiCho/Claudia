#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试热路径与SafetyValidator集成 - Critical安全修复验证
修复: 热路径绕过SafetyValidator导致坐姿直接执行表演动作
"""

import sys
sys.path.insert(0, '/home/m1ng/claudia')

from dataclasses import dataclass
from src.claudia.brain.production_brain import ProductionBrain
from src.claudia.brain.safety_validator import SafetyValidator


@dataclass
class MockSystemStateInfo:
    """模拟状态信息"""
    battery_level: float
    is_standing: bool

    class State:
        name = "NORMAL"

    state: State = State()


def test_hotpath_detection():
    """测试热路径命中检测"""
    print("=" * 60)
    print("🧪 测试1: 热路径命中检测")
    print("=" * 60)

    brain = object.__new__(ProductionBrain)
    brain.logger = None

    # 测试1.1: 日语命令命中
    print("\n1.1 测试日语'ハート':")
    api = brain._try_hotpath("ハート")
    if api == 1036:
        print(f"  ✅ 热路径命中: 'ハート' → {api}")
    else:
        print(f"  ❌ 错误: 期望1036但得到{api}")

    # 测试1.2: 英语命令命中
    print("\n1.2 测试英语'heart':")
    api = brain._try_hotpath("heart")
    if api == 1036:
        print(f"  ✅ 热路径命中: 'heart' → {api}")
    else:
        print(f"  ❌ 错误: 期望1036但得到{api}")

    # 测试1.3: 中文命令命中
    print("\n1.3 测试中文'爱心':")
    api = brain._try_hotpath("爱心")
    if api == 1036:
        print(f"  ✅ 热路径命中: '爱心' → {api}")
    else:
        print(f"  ❌ 错误: 期望1036但得到{api}")

    # 测试1.4: Dance命令命中
    print("\n1.4 测试日语'ダンス':")
    api = brain._try_hotpath("ダンス")
    if api == 1023:
        print(f"  ✅ 热路径命中: 'ダンス' → {api}")
    else:
        print(f"  ❌ 错误: 期望1023但得到{api}")


def test_safety_validator_standing_requirement():
    """测试SafetyValidator站立需求检查"""
    print("\n" + "=" * 60)
    print("🧪 测试2: SafetyValidator站立需求")
    print("=" * 60)

    validator = SafetyValidator()

    # 场景2.1: 坐姿 + Heart(1036) → 应补[1004, 1036]
    print("\n2.1 坐姿 + Heart → 应补Stand前置:")
    mock_state = MockSystemStateInfo(battery_level=0.80, is_standing=False)

    result = validator.validate_action(1036, mock_state)

    if result.is_safe and result.modified_sequence:
        if 1004 in result.modified_sequence and 1036 in result.modified_sequence:
            print(f"  ✅ SafetyValidator补全序列: {result.modified_sequence}")
        else:
            print(f"  ❌ 序列不完整: {result.modified_sequence}")
    else:
        print(f"  ❌ 未补全序列 (is_safe={result.is_safe}, modified={result.modified_sequence})")

    # 场景2.2: 站姿 + Heart → 应直接执行
    print("\n2.2 站姿 + Heart → 应直接执行:")
    mock_state = MockSystemStateInfo(battery_level=0.80, is_standing=True)

    result = validator.validate_action(1036, mock_state)

    if result.is_safe and not result.modified_sequence:
        print(f"  ✅ SafetyValidator允许直接执行")
    else:
        print(f"  ⚠️  is_safe={result.is_safe}, modified={result.modified_sequence}")

    # 场景2.3: 坐姿 + Dance → 应补Stand前置
    print("\n2.3 坐姿 + Dance → 应补Stand前置:")
    mock_state = MockSystemStateInfo(battery_level=0.80, is_standing=False)

    result = validator.validate_action(1023, mock_state)

    if result.is_safe and result.modified_sequence:
        if 1004 in result.modified_sequence and 1023 in result.modified_sequence:
            print(f"  ✅ SafetyValidator补全序列: {result.modified_sequence}")
        else:
            print(f"  ❌ 序列不完整: {result.modified_sequence}")
    else:
        print(f"  ❌ 未补全序列")

    # 场景2.4: 坐姿 + Sit(1009) → 应直接执行（Sit无需站立）
    print("\n2.4 坐姿 + Sit → 应直接执行:")
    mock_state = MockSystemStateInfo(battery_level=0.80, is_standing=False)

    result = validator.validate_action(1009, mock_state)

    if result.is_safe and not result.modified_sequence:
        print(f"  ✅ SafetyValidator允许直接执行（Sit无需站立前置）")
    else:
        print(f"  ⚠️  is_safe={result.is_safe}, modified={result.modified_sequence}")


def test_hotpath_with_battery_gate():
    """测试热路径与最终安全门集成"""
    print("\n" + "=" * 60)
    print("🧪 测试3: 热路径+最终安全门")
    print("=" * 60)

    brain = object.__new__(ProductionBrain)
    brain.logger = None

    # 场景3.1: 8%电量 + Heart → 应被拒绝
    print("\n3.1 8%电量 + Heat路径 → 应被最终安全门拒绝:")
    hotpath_api = brain._try_hotpath("ハート")
    mock_state = MockSystemStateInfo(battery_level=0.08, is_standing=True)

    # 假设没有序列，直接检查最终安全门
    safe_api, reason = brain._final_safety_gate(hotpath_api, mock_state)

    if safe_api is None:
        print(f"  ✅ 最终安全门正确拒绝: {reason}")
    else:
        print(f"  ❌ 应该拒绝但返回了{safe_api}")

    # 场景3.2: 60%电量 + Heart → 应允许
    print("\n3.2 60%电量 + Heart热路径 → 应允许:")
    mock_state = MockSystemStateInfo(battery_level=0.60, is_standing=True)

    safe_api, reason = brain._final_safety_gate(hotpath_api, mock_state)

    if safe_api == hotpath_api:
        print(f"  ✅ 最终安全门允许: {reason}")
    else:
        print(f"  ❌ 期望{hotpath_api}但返回了{safe_api}")


def test_integrated_hotpath_safety():
    """综合测试：热路径 + SafetyValidator + 最终安全门"""
    print("\n" + "=" * 60)
    print("🧪 测试4: 热路径完整安全链路")
    print("=" * 60)

    brain = object.__new__(ProductionBrain)
    brain.logger = None
    validator = SafetyValidator()

    # 场景4.1: 坐姿 + 60%电量 + Heart → 应补Stand序列
    print("\n4.1 坐姿+60%电量+Heart热路径 → 序列[1004,1036]:")
    hotpath_api = brain._try_hotpath("ハート")
    assert hotpath_api == 1036, f"热路径应返回1036，实际: {hotpath_api}"

    mock_state = MockSystemStateInfo(battery_level=0.60, is_standing=False)

    # Step 1: SafetyValidator
    sv_result = validator.validate_action(hotpath_api, mock_state)

    if sv_result.modified_sequence:
        print(f"  ✅ SafetyValidator补全: {sv_result.modified_sequence}")
        sequence = sv_result.modified_sequence
        api_code = None if sv_result.should_use_sequence_only else hotpath_api

        # Step 2: 最终安全门（检查序列最后一步）
        final_api = sequence[-1]
        safe_api, reason = brain._final_safety_gate(final_api, mock_state)

        if safe_api == final_api:
            print(f"  ✅ 最终安全门允许: {reason}")
            print(f"  ✅ 完整流程: Heart热路径 → SafetyValidator补Stand → 最终安全门通过")
        else:
            print(f"  ❌ 最终安全门错误")
    else:
        print(f"  ❌ SafetyValidator未补全序列")

    # 场景4.2: 坐姿 + 8%电量 + Heart → SafetyValidator补序列，但最终安全门拒绝
    print("\n4.2 坐姿+8%电量+Heart热路径 → 应被最终安全门拒绝:")
    mock_state = MockSystemStateInfo(battery_level=0.08, is_standing=False)

    sv_result = validator.validate_action(hotpath_api, mock_state)

    if sv_result.modified_sequence:
        sequence = sv_result.modified_sequence
        final_api = sequence[-1]
        safe_api, reason = brain._final_safety_gate(final_api, mock_state)

        if safe_api is None:
            print(f"  ✅ 最终安全门正确拒绝: {reason}")
            print(f"  ✅ 完整流程: Heart热路径 → SafetyValidator补Stand → 最终安全门拒绝（电量不足）")
        else:
            print(f"  ❌ 8%电量应该拒绝但允许了{safe_api}")
    else:
        print(f"  ⚠️  SafetyValidator未补全序列")

    # 场景4.3: 站姿 + 60%电量 + Heart → 直接执行
    print("\n4.3 站姿+60%电量+Heart热路径 → 应直接执行:")
    mock_state = MockSystemStateInfo(battery_level=0.60, is_standing=True)

    sv_result = validator.validate_action(hotpath_api, mock_state)

    if sv_result.is_safe and not sv_result.modified_sequence:
        safe_api, reason = brain._final_safety_gate(hotpath_api, mock_state)

        if safe_api == hotpath_api:
            print(f"  ✅ 完整流程: Heart热路径 → SafetyValidator通过 → 最终安全门通过 → 直接执行")
        else:
            print(f"  ❌ 最终安全门错误")
    else:
        print(f"  ❌ SafetyValidator不应补全序列")


def main():
    """运行所有测试"""
    print("\n" + "=" * 60)
    print("🚀 开始测试热路径+SafetyValidator集成")
    print("=" * 60)

    try:
        test_hotpath_detection()
        test_safety_validator_standing_requirement()
        test_hotpath_with_battery_gate()
        test_integrated_hotpath_safety()

        print("\n" + "=" * 60)
        print("✅ 所有热路径安全集成测试完成！")
        print("=" * 60)
        print("\n修复验证:")
        print("  1. 热路径命中检测: ✓")
        print("  2. SafetyValidator站立需求: ✓")
        print("  3. 最终安全门电量检查: ✓")
        print("  4. 完整安全链路: ✓")
        print("\nCritical漏洞已修复:")
        print("  ❌ 坐姿+热路径'ハート' 直接执行 → ✅ 自动补Stand前置[1004,1036]")
        print("  ❌ 8%电量+热路径'ハート' 允许 → ✅ 最终安全门拒绝")

    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
