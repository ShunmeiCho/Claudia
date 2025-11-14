#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试安全修复 - 验证状态快照+热路径+安全门
"""

import sys
sys.path.insert(0, '/home/m1ng/claudia')

from src.claudia.brain.production_brain import ProductionBrain
from dataclasses import dataclass

@dataclass
class MockSystemStateInfo:
    """模拟状态信息"""
    battery_level: float  # 可以是0-1或0-100
    is_standing: bool

    class State:
        name = "NORMAL"

    state: State = State()

def test_battery_safety():
    """测试电量安全检查"""
    print("="*60)
    print("🧪 测试1: 电量安全检查")
    print("="*60)

    # 创建一个最小化的brain实例，避免ROS2初始化
    brain = object.__new__(ProductionBrain)
    brain.logger = None  # 不需要logger

    # 测试1: 极低电量应拒绝高能动作
    print("\n1.1 测试极低电量(8%)拒绝前转:")
    mock_state = MockSystemStateInfo(battery_level=0.08, is_standing=True)
    api_code = 1030  # FrontFlip
    safe_api, reason = brain._final_safety_gate(api_code, mock_state)
    if safe_api is None:
        print(f"✅ 正确拒绝: {reason}")
    else:
        print(f"❌ 错误: 应该拒绝但返回了{safe_api}")

    # 测试2: 极低电量应允许Stop
    print("\n1.2 测试极低电量(8%)允许Stop:")
    safe_api, reason = brain._final_safety_gate(1003, mock_state)
    if safe_api == 1003:
        print(f"✅ 正确允许: {reason}")
    else:
        print(f"❌ 错误: 应该允许Stop但返回了{safe_api}")

    # 测试3: 低电量应拒绝高能动作
    print("\n1.3 测试低电量(15%)拒绝Jump:")
    mock_state = MockSystemStateInfo(battery_level=0.15, is_standing=True)
    safe_api, reason = brain._final_safety_gate(1031, mock_state)
    if safe_api is None:
        print(f"✅ 正确拒绝: {reason}")
    else:
        print(f"❌ 错误: 应该拒绝但返回了{safe_api}")

    # 测试4: 中等电量应降级高能动作
    print("\n1.4 测试中等电量(25%)降级Flip→Dance:")
    mock_state = MockSystemStateInfo(battery_level=0.25, is_standing=True)
    safe_api, reason = brain._final_safety_gate(1030, mock_state)
    if safe_api == 1023:
        print(f"✅ 正确降级: {reason}")
    else:
        print(f"❌ 错误: 应该降级到1023但返回了{safe_api}")

    # 测试5: 正常电量应无限制
    print("\n1.5 测试正常电量(60%)允许Flip:")
    mock_state = MockSystemStateInfo(battery_level=0.60, is_standing=True)
    safe_api, reason = brain._final_safety_gate(1030, mock_state)
    if safe_api == 1030:
        print(f"✅ 正确允许: {reason}")
    else:
        print(f"❌ 错误: 应该允许1030但返回了{safe_api}")

def test_battery_normalization():
    """测试电量归一化"""
    print("\n" + "="*60)
    print("🧪 测试2: 电量归一化")
    print("="*60)

    brain = object.__new__(ProductionBrain)

    # 测试0-1范围
    print("\n2.1 测试0.5电量:")
    result = brain._normalize_battery(0.5)
    if result == 0.5:
        print(f"✅ 正确: 0.5 → {result}")
    else:
        print(f"❌ 错误: 期望0.5但得到{result}")

    # 测试0-100范围
    print("\n2.2 测试50%电量:")
    result = brain._normalize_battery(50.0)
    if result == 0.5:
        print(f"✅ 正确: 50.0 → {result}")
    else:
        print(f"❌ 错误: 期望0.5但得到{result}")

    # 测试None
    print("\n2.3 测试None电量:")
    result = brain._normalize_battery(None)
    if result is None:
        print(f"✅ 正确: None → {result}")
    else:
        print(f"❌ 错误: 期望None但得到{result}")

def test_hotpath():
    """测试热路径"""
    print("\n" + "="*60)
    print("🧪 测试3: 热路径检测")
    print("="*60)

    brain = object.__new__(ProductionBrain)

    # 测试日语命令
    print("\n3.1 测试日语'座って':")
    api = brain._try_hotpath("座って")
    if api == 1009:
        print(f"✅ 正确命中: '座って' → {api}")
    else:
        print(f"❌ 错误: 期望1009但得到{api}")

    # 测试英语命令
    print("\n3.2 测试英语'sit':")
    api = brain._try_hotpath("sit")
    if api == 1009:
        print(f"✅ 正确命中: 'sit' → {api}")
    else:
        print(f"❌ 错误: 期望1009但得到{api}")

    # 测试中文命令
    print("\n3.3 测试中文'坐下':")
    api = brain._try_hotpath("坐下")
    if api == 1009:
        print(f"✅ 正确命中: '坐下' → {api}")
    else:
        print(f"❌ 错误: 期望1009但得到{api}")

    # 测试未命中
    print("\n3.4 测试未命中'かっこいい':")
    api = brain._try_hotpath("かっこいい")
    if api is None:
        print(f"✅ 正确未命中: 'かっこいい' → {api}")
    else:
        print(f"❌ 错误: 应该未命中但得到{api}")

def test_quick_precheck():
    """测试快速安全预检"""
    print("\n" + "="*60)
    print("🧪 测试4: 快速安全预检")
    print("="*60)

    brain = object.__new__(ProductionBrain)
    brain.logger = None  # 不需要logger

    # 测试1: 极低电量拒绝非安全命令
    print("\n4.1 测试极低电量拒绝'かっこいい':")
    mock_state = MockSystemStateInfo(battery_level=0.08, is_standing=True)
    reason = brain._quick_safety_precheck("かっこいい", mock_state)
    if reason is not None:
        print(f"✅ 正确拒绝: {reason}")
    else:
        print(f"❌ 错误: 应该拒绝但返回None")

    # 测试2: 极低电量允许安全命令
    print("\n4.2 测试极低电量允许'stop':")
    reason = brain._quick_safety_precheck("stop", mock_state)
    if reason is None:
        print(f"✅ 正确允许")
    else:
        print(f"❌ 错误: 应该允许但拒绝了: {reason}")

    # 测试3: 低电量拒绝高能命令
    print("\n4.3 测试低电量(15%)拒绝'flip':")
    mock_state = MockSystemStateInfo(battery_level=0.15, is_standing=True)
    reason = brain._quick_safety_precheck("前転してください", mock_state)
    if reason is not None:
        print(f"✅ 正确拒绝: {reason}")
    else:
        print(f"❌ 错误: 应该拒绝但返回None")

def main():
    """运行所有测试"""
    print("\n" + "="*60)
    print("🚀 开始测试安全修复")
    print("="*60)

    try:
        test_battery_safety()
        test_battery_normalization()
        test_hotpath()
        test_quick_precheck()

        print("\n" + "="*60)
        print("✅ 所有测试完成！")
        print("="*60)
        print("\n下一步:")
        print("1. 运行: python3 test_safety_fixes.py")
        print("2. 测试'前転'命令在8%电量时应被拒绝")
        print("3. 测试'座って'命令使用热路径(<100ms)")
        print("4. 验证所有安全门正常工作")

    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
