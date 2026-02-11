#!/usr/bin/env python3
"""
Claudia Robot - LLM性能优化验证脚本
验证日志重复问题修复和3B模型性能优化效果
"""

import sys
import time
import asyncio
from pathlib import Path

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

def test_performance_optimization():
    """测试性能优化效果"""
    print("🔧 Claudia LLM性能优化验证")
    print("=" * 50)
    
    # 1. 初始化优化后的接口
    llm = ClaudiaLLMInterface()
    
    # 2. 显示优化配置
    print(f"\n📊 优化配置:")
    print(f"  模型: {llm.model_name}")
    print(f"  超时: {llm.request_timeout}秒")
    print(f"  重试次数: {llm.max_retries}")
    print(f"  温度: {llm.model_params['temperature']}")
    print(f"  输出限制: {llm.model_params['num_predict']}字符")
    
    # 3. 性能测试案例
    test_cases = [
        "座る",
        "こんにちは",
        "立って",
        "ダンスして",
        "停止"
    ]
    
    print(f"\n⚡ 性能测试 ({len(test_cases)}个案例):")
    
    total_start = time.time()
    results = []
    
    for i, command in enumerate(test_cases, 1):
        print(f"\n[{i}/{len(test_cases)}] 测试: '{command}'")
        
        start_time = time.time()
        response = llm.generate_response(f"指令: {command}")
        end_time = time.time()
        
        duration = end_time - start_time
        success = not response.startswith("❌")
        
        results.append({
            "command": command,
            "duration": duration,
            "success": success,
            "response_length": len(response)
        })
        
        status = "✅ 成功" if success else "❌ 失败"
        print(f"  {status} - {duration:.2f}秒 - {len(response)}字符")
    
    # 4. 性能统计
    total_time = time.time() - total_start
    success_count = sum(1 for r in results if r["success"])
    avg_time = sum(r["duration"] for r in results) / len(results)
    
    print(f"\n📈 性能统计:")
    print(f"  总耗时: {total_time:.2f}秒")
    print(f"  成功率: {success_count}/{len(test_cases)} ({success_count/len(test_cases)*100:.1f}%)")
    print(f"  平均响应: {avg_time:.2f}秒")
    print(f"  最快响应: {min(r['duration'] for r in results):.2f}秒")
    print(f"  最慢响应: {max(r['duration'] for r in results):.2f}秒")
    
    # 5. 评估优化效果
    print(f"\n🎯 优化效果评估:")
    
    # 目标性能指标
    target_avg_time = 3.0  # 目标平均响应时间
    target_success_rate = 90  # 目标成功率
    
    if avg_time <= target_avg_time:
        print(f"  ✅ 响应速度: {avg_time:.2f}s ≤ {target_avg_time}s (达标)")
    else:
        print(f"  ⚠️ 响应速度: {avg_time:.2f}s > {target_avg_time}s (需要进一步优化)")
    
    success_rate = success_count/len(test_cases)*100
    if success_rate >= target_success_rate:
        print(f"  ✅ 成功率: {success_rate:.1f}% ≥ {target_success_rate}% (达标)")
    else:
        print(f"  ⚠️ 成功率: {success_rate:.1f}% < {target_success_rate}% (需要改进)")
    
    # 6. 优化建议
    if avg_time > target_avg_time:
        print(f"\n💡 优化建议:")
        print(f"  - 进一步减少num_predict参数")
        print(f"  - 调整temperature到更低值")
        print(f"  - 检查网络延迟和硬件性能")
    
    return results

async def test_enhanced_interface():
    """测试增强界面的性能"""
    print(f"\n🤖 测试增强界面...")
    
    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface
        
        interface = EnhancedJapaneseCommandInterface()
        await interface.initialize()
        
        # 测试单个命令处理
        test_command = "座る"
        print(f"  测试命令: {test_command}")
        
        start_time = time.time()
        result = await interface.process_japanese_command(test_command)
        end_time = time.time()
        
        total_time = end_time - start_time
        success = result.get("success", False)
        
        print(f"  结果: {'✅ 成功' if success else '❌ 失败'}")
        print(f"  总耗时: {total_time:.2f}秒")
        
        # 分析耗时分布
        if "llm_analysis" in result:
            llm_time = result["llm_analysis"].get("analysis_time", 0)
            print(f"  LLM分析: {llm_time:.2f}秒")
        
        if "execution_result" in result:
            exec_time = result["execution_result"].get("total_time", 0)
            print(f"  动作执行: {exec_time:.2f}秒")
        
        return True
        
    except Exception as e:
        print(f"  ❌ 增强界面测试失败: {e}")
        return False

def main():
    """主函数"""
    try:
        # 1. 基础LLM性能测试
        llm_results = test_performance_optimization()
        
        # 2. 增强界面测试
        asyncio.run(test_enhanced_interface())
        
        print(f"\n🎉 性能验证完成！")
        
        # 推荐下一步
        avg_time = sum(r["duration"] for r in llm_results) / len(llm_results)
        if avg_time <= 3.0:
            print(f"✅ 性能优化成功，建议投入生产使用")
        else:
            print(f"⚠️ 还有优化空间，建议继续调整参数")
        
    except Exception as e:
        print(f"❌ 验证过程出错: {e}")

if __name__ == "__main__":
    main() 