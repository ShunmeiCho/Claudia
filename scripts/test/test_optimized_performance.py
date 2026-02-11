#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia机器人项目 - 性能优化验证测试
测试优化后的模型性能改善情况
"""

import sys
import time
import statistics
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

class PerformanceTestSuite:
    """性能测试套件"""
    
    def __init__(self):
        self.results = {
            "response_times": [],
            "success_count": 0,
            "error_count": 0,
            "model_info": None
        }
        
        # 测试命令集
        self.test_commands = [
            "立って",
            "座って", 
            "こんにちは",
            "停止",
            "ダンスして",
            "伏せ"
        ]
    
    def run_performance_test(self):
        """运行性能测试"""
        print("🧪 Claudia性能优化验证测试")
        print("=" * 50)
        print(f"测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"测试命令数: {len(self.test_commands)}")
        print()
        
        # 初始化LLM接口
        try:
            llm = ClaudiaLLMInterface()
            print(f"🤖 已连接模型: {llm.model_name}")
            
            # 获取模型状态
            status = llm.get_status()
            self.results["model_info"] = status
            print(f"📊 连接状态: {status.get('connection', '未知')}")
            print()
            
        except Exception as e:
            print(f"❌ LLM初始化失败: {e}")
            return
        
        # 执行测试
        print("🎯 开始性能测试...")
        print("-" * 30)
        
        total_start_time = time.time()
        
        for i, command in enumerate(self.test_commands, 1):
            print(f"[{i}/{len(self.test_commands)}] 测试: '{command}'")
            
            # 测试单个命令
            start_time = time.time()
            try:
                result = llm.robot_command_interpreter(command)
                response_time = time.time() - start_time
                
                self.results["response_times"].append(response_time)
                self.results["success_count"] += 1
                
                # 显示结果
                status_emoji = "✅" if response_time < 15 else "⚠️" if response_time < 30 else "❌"
                print(f"  {status_emoji} 耗时: {response_time:.2f}秒")
                print(f"  📝 响应: {result[:60]}{'...' if len(result) > 60 else ''}")
                
            except Exception as e:
                self.results["error_count"] += 1
                print(f"  ❌ 错误: {str(e)}")
            
            print()
        
        total_time = time.time() - total_start_time
        
        # 分析结果
        self._analyze_results(total_time)
    
    def _analyze_results(self, total_time):
        """分析测试结果"""
        print("📊 性能分析结果")
        print("=" * 50)
        
        if not self.results["response_times"]:
            print("❌ 没有成功的测试结果")
            return
        
        # 基础统计
        response_times = self.results["response_times"]
        avg_time = statistics.mean(response_times)
        min_time = min(response_times)
        max_time = max(response_times)
        median_time = statistics.median(response_times)
        
        print(f"🎯 测试统计:")
        print(f"  总测试数: {len(self.test_commands)}")
        print(f"  成功数: {self.results['success_count']}")
        print(f"  失败数: {self.results['error_count']}")
        print(f"  成功率: {(self.results['success_count']/len(self.test_commands)*100):.1f}%")
        
        print(f"\n⏱️ 响应时间:")
        print(f"  平均: {avg_time:.2f}秒")
        print(f"  最快: {min_time:.2f}秒")
        print(f"  最慢: {max_time:.2f}秒")
        print(f"  中位数: {median_time:.2f}秒")
        print(f"  总耗时: {total_time:.2f}秒")
        
        # 性能等级评估
        print(f"\n🏆 性能等级:")
        if avg_time <= 5:
            grade = "A+ 极佳"
            emoji = "🚀"
        elif avg_time <= 10:
            grade = "A 良好"
            emoji = "✅"
        elif avg_time <= 20:
            grade = "B 一般"
            emoji = "⚠️"
        elif avg_time <= 35:
            grade = "C 偏慢"
            emoji = "🐌"
        else:
            grade = "D 过慢"
            emoji = "❌"
        
        print(f"  {emoji} {grade} (平均 {avg_time:.2f}秒)")
        
        # 与优化前对比
        print(f"\n📈 优化效果对比:")
        baseline_time = 40.0  # 优化前的基线时间
        improvement = ((baseline_time - avg_time) / baseline_time) * 100
        print(f"  优化前: ~{baseline_time:.1f}秒")
        print(f"  优化后: {avg_time:.2f}秒")
        if improvement > 0:
            print(f"  改善: {improvement:.1f}% 更快 🎉")
        else:
            print(f"  退化: {abs(improvement):.1f}% 更慢 😰")
        
        # 模型信息
        model_info = self.results.get("model_info", {})
        print(f"\n🤖 模型信息:")
        print(f"  模型: {model_info.get('model', '未知')}")
        print(f"  超时设置: {model_info.get('timeout', '未知')}秒")
        
        # 建议
        print(f"\n💡 优化建议:")
        if avg_time > 15:
            print("  - 考虑使用更轻量的模型")
            print("  - 减少上下文长度")
            print("  - 启用命令缓存")
        elif avg_time > 8:
            print("  - 可以进一步优化参数配置")
            print("  - 考虑预处理常用命令")
        else:
            print("  - 性能已达到优秀水平！")
            print("  - 可以考虑添加更多复杂功能")

def main():
    """主函数"""
    try:
        tester = PerformanceTestSuite()
        tester.run_performance_test()
        
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 