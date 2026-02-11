#!/usr/bin/env python3
"""
快速LLM测试脚本 - 验证超时问题修复
"""

import sys
import time
from pathlib import Path

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

def quick_test():
    """快速测试LLM性能"""
    print("🔧 快速LLM测试 - 验证超时修复")
    print("=" * 40)
    
    # 初始化接口
    llm = ClaudiaLLMInterface()
    
    # 显示配置
    print(f"模型: {llm.model_name}")
    print(f"超时: {llm.request_timeout}秒")
    print(f"参数: {llm.model_params}")
    
    # 测试简单命令
    test_commands = ["座る", "こんにちは", "立って"]
    
    for i, cmd in enumerate(test_commands, 1):
        print(f"\n[{i}] 测试: '{cmd}'")
        
        start_time = time.time()
        try:
            response = llm.generate_response(f"指令: {cmd}")
            end_time = time.time()
            
            duration = end_time - start_time
            success = not response.startswith("❌")
            
            status = "✅ 成功" if success else "❌ 失败"
            print(f"  {status} - {duration:.2f}秒")
            print(f"  响应: {response[:50]}...")
            
            if not success:
                print(f"  ⚠️ 失败原因需要进一步调查")
                break
                
        except Exception as e:
            print(f"  ❌ 异常: {e}")
            break
    
    print(f"\n🎯 测试完成")

if __name__ == "__main__":
    quick_test() 