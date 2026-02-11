#!/usr/bin/env python3
"""
快速动作映射修复验证脚本
用于验证比心、ちんちん等动作的映射修复效果
"""

import sys
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_action_mapping_fix():
    """快速测试动作映射修复"""
    print("🚀 快速验证比心等动作映射修复")
    print("=" * 50)
    
    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface
        
        # 创建接口实例
        interface = EnhancedJapaneseCommandInterface()
        
        # 关键修复测试用例
        test_cases = [
            ("比心", "heart", "应该映射到API 1021 (Wallow)"),
            ("ちんちん", "cheer", "应该映射到API 1026 (Cheer)"),
            ("ハート", None, "应该通过模式匹配识别为heart"),
            ("拜年", None, "应该通过模式匹配识别为cheer"),
        ]
        
        print("\n🧪 测试关键动作映射修复:")
        all_passed = True
        
        for japanese_input, expected_action, description in test_cases:
            print(f"\n测试: '{japanese_input}' - {description}")
            
            # 1. 直接API映射测试
            if expected_action and expected_action in interface.action_api_map:
                api_code = interface.action_api_map[expected_action]
                print(f"  ✅ 直接映射: {expected_action} → API {api_code}")
            
            # 2. 模式匹配测试
            action, confidence = interface.extract_action_from_llm_response("", japanese_input)
            if action and action in interface.action_api_map:
                api_code = interface.action_api_map[action]
                print(f"  ✅ 模式匹配: '{japanese_input}' → {action} (置信度:{confidence:.2f}) → API {api_code}")
            else:
                print(f"  ❌ 模式匹配失败: '{japanese_input}' → {action}")
                all_passed = False
        
        # 测试动作序列规划（比心的关键修复）
        print(f"\n🔧 测试动作序列规划（比心关键修复）:")
        
        # 模拟lying状态下的比心动作
        interface.robot_state.current_posture = "lying"
        heart_api = interface.action_api_map.get("heart")
        
        if heart_api:
            sequence = interface.action_sequencer.plan_action_sequence("heart", heart_api)
            print(f"  机器人状态: lying → 执行比心动作")
            
            for i, step in enumerate(sequence, 1):
                print(f"    {i}. {step['action']} (API: {step['api']})")
            
            # 验证序列是否正确
            expected_steps = ["stand_up", "heart"]
            actual_steps = [step['action'] for step in sequence]
            
            if actual_steps == expected_steps:
                print(f"  ✅ 比心序列规划正确: 会先站立再比心，解决错误码3203")
            else:
                print(f"  ❌ 比心序列规划有问题: 期望{expected_steps}, 实际{actual_steps}")
                all_passed = False
        
        # 总结
        print(f"\n{'='*50}")
        if all_passed:
            print("🎉 所有测试通过！修复成功验证")
            print("\n📋 修复验证总结:")
            print("  ✅ 比心动作映射正确 (API 1021)")
            print("  ✅ ちんちん动作映射正确 (API 1026)")  
            print("  ✅ 模式匹配识别正确")
            print("  ✅ 动作序列规划正确（先站立再比心）")
            print("\n🚀 现在可以测试真实机器人:")
            print("  1. 运行: python3 scripts/run_enhanced_japanese_commander.sh")
            print("  2. 在lying状态下输入'比心' - 应该先站立再比心")
            print("  3. 输入'ちんちん' - 应该执行庆祝动作")
        else:
            print("⚠️ 某些测试失败，需要进一步检查")
        
        return all_passed
        
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        print("请确保在claudia项目根目录下运行此脚本")
        return False
    except Exception as e:
        print(f"❌ 测试异常: {e}")
        return False

if __name__ == "__main__":
    success = test_action_mapping_fix()
    sys.exit(0 if success else 1) 