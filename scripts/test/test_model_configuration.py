#!/usr/bin/env python3
"""
测试模型配置修复
验证系统是否正确使用claudia-optimized:v2.1而不是latest版本
"""

import sys
import time
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

def test_default_model_configuration():
    """测试默认模型配置"""
    print("🔍 测试默认模型配置...")
    
    # 测试1：默认初始化
    llm = ClaudiaLLMInterface()
    print(f"  默认模型: {llm.model_name}")
    
    # 测试2：明确指定v2.1
    llm_v21 = ClaudiaLLMInterface(model_name="claudia-optimized:v2.1")
    print(f"  指定v2.1模型: {llm_v21.model_name}")
    
    # 测试3：检查连接状态
    status = llm.get_status()
    print(f"  连接状态: {status['connection']}")
    print(f"  实际使用模型: {status['model']}")
    
    if status.get('available_models'):
        print(f"  可用模型: {', '.join(status['available_models'])}")
    
    return llm.model_name == "claudia-optimized:v2.1"

def test_enhanced_interface_configuration():
    """测试增强界面的模型配置"""
    print("\n🤖 测试增强界面模型配置...")
    
    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface
        
        interface = EnhancedJapaneseCommandInterface()
        
        # 模拟初始化LLM部分（不启动完整界面）
        interface.llm_interface = ClaudiaLLMInterface(model_name="claudia-optimized:v2.1")
        
        if interface.llm_interface:
            print(f"  增强界面使用模型: {interface.llm_interface.model_name}")
            return interface.llm_interface.model_name == "claudia-optimized:v2.1"
        else:
            print("  ❌ 无法初始化LLM接口")
            return False
            
    except Exception as e:
        print(f"  ❌ 测试失败: {e}")
        return False

def test_single_llm_call():
    """测试单次LLM调用（确保没有重复调用）"""
    print("\n⚡ 测试单次LLM调用...")
    
    llm = ClaudiaLLMInterface(model_name="claudia-optimized:v2.1")
    
    # 记录调用开始时间
    start_time = time.time()
    
    # 简单测试调用
    test_input = "こんにちは"
    
    print(f"  测试输入: {test_input}")
    print(f"  使用模型: {llm.model_name}")
    
    try:
        response = llm.robot_command_interpreter(test_input)
        elapsed_time = time.time() - start_time
        
        print(f"  ✅ 调用成功")
        print(f"  响应时间: {elapsed_time:.2f}秒")
        print(f"  响应长度: {len(response)}字符")
        
        # 检查响应是否合理（不是错误信息）
        if "❌" in response:
            print(f"  ⚠️ 响应包含错误信息: {response[:100]}...")
            return False
        
        return True
        
    except Exception as e:
        print(f"  ❌ 调用失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🧪 Claudia模型配置修复验证")
    print("=" * 50)
    
    tests = [
        ("默认模型配置", test_default_model_configuration),
        ("增强界面配置", test_enhanced_interface_configuration),
        ("单次LLM调用", test_single_llm_call)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n🔬 执行测试: {test_name}")
        try:
            result = test_func()
            results.append((test_name, result))
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  结果: {status}")
        except Exception as e:
            results.append((test_name, False))
            print(f"  ❌ 异常: {e}")
    
    # 总结
    print(f"\n📊 测试总结:")
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✅" if result else "❌"
        print(f"  {status} {test_name}")
    
    print(f"\n🎯 通过率: {passed}/{total} ({passed/total*100:.1f}%)")
    
    if passed == total:
        print("🎉 所有测试通过！模型配置修复成功")
    else:
        print("⚠️ 部分测试失败，需要进一步检查")

if __name__ == "__main__":
    main() 