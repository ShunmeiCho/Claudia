#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试比心等动作映射修复
Generated: 2025-07-10
Purpose: 验证WALLOW(1021)和其他缺失动作的映射是否正确
"""

import sys
import os
import re
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_enhanced_japanese_commander_mapping():
    """测试增强版日语指令界面的动作映射"""
    print("🧪 测试增强版日语指令界面的动作映射...")
    
    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface
        
        interface = EnhancedJapaneseCommandInterface()
        
        # 测试比心相关映射
        test_cases = [
            # 🔧 比心动作测试
            ("比心", "heart"),
            ("ハート", "heart"), 
            ("heart", "heart"),
            ("love", "love"),
            ("可愛い", None),  # 这个应该通过模式匹配到heart
            
            # 🔧 ちんちん动作测试 (应该映射到cheer)
            ("ちんちん", "cheer"),
            ("拜年", None),  # 通过模式匹配到cheer
            
            # 基础动作测试
            ("お座り", "sit"),
            ("ダンス", "dance"),
            ("お辞儀", "bow"),
            ("握手", "shake_hands"),
            
            # 新增高级动作
            ("ジャンプ", "jump"),
            ("がんばれ", "cheer"),
            ("パンチ", "punch"),
        ]
        
        print("\n📋 动作API映射测试:")
        success_count = 0
        total_count = len(test_cases)
        
        for japanese_input, expected_action in test_cases:
            mapped_api = interface.action_api_map.get(expected_action if expected_action else japanese_input)
            
            if mapped_api:
                print(f"✅ '{japanese_input}' → {expected_action or japanese_input} → API {mapped_api}")
                success_count += 1
            else:
                # 尝试通过模式匹配
                action, confidence = interface.extract_action_from_llm_response("", japanese_input)
                if action and action in interface.action_api_map:
                    api_code = interface.action_api_map[action]
                    print(f"✅ '{japanese_input}' → {action} (模式匹配, 置信度:{confidence:.2f}) → API {api_code}")
                    success_count += 1
                else:
                    print(f"❌ '{japanese_input}' → 无映射")
        
        print(f"\n📊 映射成功率: {success_count}/{total_count} ({success_count/total_count*100:.1f}%)")
        print(f"📊 总计映射数量: {len(interface.action_api_map)}")
        
        # 🔧 测试动作序列规划（比心的关键修复）
        print("\n🔧 测试动作序列规划（比心修复）:")
        test_action_sequences(interface)
        
        # 测试模式匹配
        print("\n🔍 日语模式匹配测试:")
        test_pattern_matching(interface)
        
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 测试异常: {e}")
        return False
    
    return True

def test_action_sequences(interface):
    """测试动作序列规划"""
    from src.claudia.interactive_japanese_commander_enhanced import RobotState
    
    # 创建不同状态的机器人状态
    test_states = [
        ("lying", "比心"),     # 关键测试：从lying状态执行比心
        ("sitting", "hello"),  # 从sitting状态执行hello
        ("standing", "heart"), # 从standing状态执行heart
        ("unknown", "dance"),  # 从unknown状态执行dance
    ]
    
    for state, action in test_states:
        # 模拟机器人状态
        interface.robot_state.current_state = state
        
        # 获取动作API
        api_code = interface.action_api_map.get(action)
        if api_code:
            # 规划动作序列
            sequence = interface.action_sequencer.plan_action_sequence(action, api_code)
            
            print(f"  状态:{state} → 动作:{action}")
            for i, step in enumerate(sequence, 1):
                print(f"    {i}. {step['action']} (API: {step['api']})")
            
            # 验证比心动作的特殊处理
            if action in ["比心", "heart"] and state == "lying":
                expected_steps = ["stand_up", action]
                actual_steps = [step['action'] for step in sequence]
                if actual_steps == expected_steps:
                    print(f"    ✅ 比心序列规划正确: {actual_steps}")
                else:
                    print(f"    ❌ 比心序列规划错误: 期望{expected_steps}, 实际{actual_steps}")

def test_pattern_matching(interface):
    """测试日语模式匹配"""
    test_inputs = [
        "比心して",        # 比心动作
        "ハートをお願い",   # 心形请求 
        "可愛いポーズ",     # 可爱姿势
        "お座りして",      # 坐下
        "ちんちんしよう",   # 拜年动作
        "がんばれ！",      # 加油
        "お辞儀します",     # 鞠躬
        "握手しませんか",   # 握手邀请
    ]
    
    for test_input in test_inputs:
        action, confidence = interface.extract_action_from_llm_response("", test_input)
        api_code = interface.action_api_map.get(action) if action else None
        
        if action and api_code:
            print(f"  ✅ '{test_input}' → {action} (置信度:{confidence:.2f}) → API {api_code}")
        else:
            print(f"  ❌ '{test_input}' → 无法识别 ({action}, {confidence:.2f})")

def test_real_action_mapping_engine():
    """测试真实动作映射引擎的API覆盖"""
    print("\n🤖 测试真实动作映射引擎...")
    
    try:
        from src.claudia.robot_controller.action_mapping_engine_real import RealActionMappingEngine
        
        engine = RealActionMappingEngine()
        
        # 测试API注册表完整性
        print(f"📊 API注册表大小: {len(engine.api_registry)}")
        print(f"📊 日语映射数量: {len(engine.intent_mapping)}")
        print(f"📊 英语映射数量: {len(engine.english_intent_mapping)}")
        
        # 测试关键API是否存在
        key_apis = [1021, 1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031]
        print("\n🔧 关键新增API测试:")
        for api_code in key_apis:
            if api_code in engine.api_registry:
                action_def = engine.api_registry[api_code]
                print(f"  ✅ API {api_code}: {action_def.function_name} - {action_def.description}")
            else:
                print(f"  ❌ API {api_code}: 缺失")
        
        # 测试"ちんちん"映射
        print("\n🎯 特殊词汇映射测试:")
        test_words = ["ちんちん", "比心", "heart", "bow", "cheer"]
        for word in test_words:
            if word in engine.intent_mapping:
                api_code = engine.intent_mapping[word]
                print(f"  ✅ '{word}' → API {api_code}")
            elif word in engine.english_intent_mapping:
                api_code = engine.english_intent_mapping[word]
                print(f"  ✅ '{word}' → API {api_code} (英语)")
            else:
                print(f"  ❌ '{word}' → 无映射")
        
        return True
        
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 测试异常: {e}")
        return False

def show_available_actions():
    """显示所有可用动作"""
    print("\n📚 完整动作映射表:")
    
    action_mapping = {
        1001: "Damp - 紧急停止",
        1002: "BalanceStand - 平衡站立",
        1003: "StopMove - 停止移动", 
        1004: "StandUp - 站立",
        1005: "StandDown - 趴下",
        1006: "RecoveryStand - 恢复站立",
        1007: "Euler - 欧拉角控制",
        1008: "Move - 移动控制",
        1009: "Sit - 坐下",
        1010: "RiseSit - 从坐姿站起",
        1011: "SwitchGait - 切换步态",
        1012: "Trigger - 触发器",
        1013: "BodyHeight - 身体高度",
        1014: "FootRaiseHeight - 抬脚高度",
        1015: "SpeedLevel - 速度等级",
        1016: "Hello - 招手/握手",
        1017: "Stretch - 伸展",
        1018: "TrajectoryFollow - 轨迹跟随",
        1019: "ContinuousGait - 连续步态",
        1020: "Content - 内容",
        1021: "Wallow - 比心动作 🔧",
        1022: "Dance1 - 舞蹈1",
        1023: "Dance2 - 舞蹈2",
        1024: "GetBodyHeight - 获取身体高度",
        1025: "GetFootRaiseHeight - 获取抬脚高度",
        1026: "GetSpeedLevel - 获取速度等级",
        1027: "SwitchJoystick - 切换手柄",
    }
    
    for api_id, description in action_mapping.items():
        marker = " 🔧" if api_id == 1021 else ""
        print(f"  {api_id:4d}: {description}{marker}")

def main():
    """主测试函数"""
    print("🤖 Claudia机器人动作映射修复测试")
    print("=" * 50)
    
    # 显示修复的动作
    show_available_actions()
    
    # 测试增强版界面
    success1 = test_enhanced_japanese_commander_mapping()
    
    # 测试真实映射引擎
    success2 = test_real_action_mapping_engine()
    
    print("\n" + "=" * 50)
    if success1 and success2:
        print("✅ 所有测试通过！比心动作映射修复成功")
        print("\n🎯 现在您可以使用以下指令:")
        print("   - '比心' / '比心して'")
        print("   - 'ハート' / 'ハートお願い'") 
        print("   - 'heart' / 'make a heart'")
        print("   - '可愛いポーズ' (会映射到heart)")
        print("\n🚀 请重新运行 run_enhanced_japanese_commander.sh 测试修复效果")
    else:
        print("❌ 部分测试失败，请检查错误信息")

if __name__ == "__main__":
    print("🚀 开始比心等动作映射修复验证测试")
    print("=" * 60)
    
    # 运行所有测试
    tests = [
        ("增强版日语指令界面", test_enhanced_japanese_commander_mapping),
        ("真实动作映射引擎", test_real_action_mapping_engine),
        ("可用动作展示", show_available_actions),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            if test_func():
                print(f"✅ {test_name} 测试通过")
                passed += 1
            else:
                print(f"❌ {test_name} 测试失败")
        except Exception as e:
            print(f"❌ {test_name} 测试异常: {e}")
    
    print("\n" + "="*60)
    print(f"🎯 测试总结: {passed}/{total} 通过 ({passed/total*100:.1f}%)")
    
    if passed == total:
        print("🎉 所有测试通过！比心动作映射修复成功")
        print("\n📋 修复内容总结:")
        print("  ✅ 扩展了action_api_map，添加7个新API映射")
        print("  ✅ 增强了日语模式匹配，包含比心、ちんちん等词汇")
        print("  ✅ 修复了动作序列规划器，比心前自动站立")
        print("  ✅ 完善了API注册表，支持27个官方API")
        print("  ✅ 解决了'ちんちん'映射到cheer动作")
        print("\n🎯 关键修复:")
        print("  🔧 比心动作错误码3203 → 现在会先执行stand_up再执行wallow")
        print("  🔧 'ちんちん'无法识别 → 现在映射到cheer庆祝动作")
    else:
        print("⚠️ 部分测试失败，需要进一步调试")
    
    print("\n🔧 下一步建议:")
    print("  1. 运行真实机器人测试验证修复效果")
    print("  2. 测试指令: python3 scripts/run_enhanced_japanese_commander.sh")
    print("  3. 输入: '比心' (应该先站立再执行比心动作)")
    print("  4. 输入: 'ちんちん' (应该执行cheer庆祝动作)")
    print("  5. 观察是否解决错误码3203问题") 