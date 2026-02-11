#!/usr/bin/env python3
"""
验证Dance返回码修复效果
测试3104等返回码现在是否被正确识别为成功
"""

import sys
from pathlib import Path

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent.parent))

def test_return_code_logic():
    """测试新的返回码判断逻辑"""
    
    print("🔧 测试返回码判断逻辑修复")
    print("=" * 40)
    
    # 导入修复后的类
    from src.claudia.robot_controller.action_mapping_engine_real import RealRobotController
    
    # 创建控制器实例（不需要真实连接）
    controller = RealRobotController()
    
    # 测试案例
    test_cases = [
        # (返回码, 方法名, 预期结果, 描述)
        (0, "Sit", True, "传统成功码"),
        (3104, "Dance1", True, "Dance1完成码（修复前会失败）"),
        (3105, "Dance2", True, "Dance2完成码"),
        (3106, "Hello", True, "其他完成码"),
        (1, "Sit", False, "真正的错误码"),
        (999, "Dance1", False, "未知错误码"),
    ]
    
    print(f"{'返回码':<8} {'方法':<10} {'预期':<6} {'实际':<6} {'状态':<8} {'描述'}")
    print("-" * 60)
    
    all_passed = True
    
    for return_code, method_name, expected, description in test_cases:
        actual = controller._is_command_successful(return_code, method_name)
        
        status = "✅ PASS" if actual == expected else "❌ FAIL"
        if actual != expected:
            all_passed = False
        
        print(f"{return_code:<8} {method_name:<10} {expected!s:<6} {actual!s:<6} {status:<8} {description}")
    
    print("-" * 60)
    print(f"总体结果: {'✅ 全部通过' if all_passed else '❌ 存在失败'}")
    
    if all_passed:
        print("\n🎉 修复成功！现在Dance命令的返回码3104会被正确识别为成功")
        print("💡 这意味着跳舞动作不再会被误报为错误")
    else:
        print("\n⚠️ 修复可能存在问题，请检查代码")
    
    return all_passed

if __name__ == "__main__":
    test_return_code_logic() 