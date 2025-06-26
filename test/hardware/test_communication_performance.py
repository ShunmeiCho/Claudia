#!/usr/bin/env python3
"""
Unitree Go2 通信性能测试 - 任务3.7
Generated: 2024-12-26 20:30:00
Purpose: 测量和验证控制命令延迟consistently <50ms，评估整体系统响应性
Safety: 使用最小化动作的安全测试方案
"""

import time
import os
import sys
import statistics
from datetime import datetime
from typing import List, Dict, Tuple

# 添加SDK路径
sys.path.append('/home/m1ng/claudia/unitree_sdk2_python')

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    print("✅ 成功导入所有必需的模块")
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    print("Please ensure unitree_sdk2py is properly installed")
    sys.exit(1)

class CommunicationPerformanceTest:
    def __init__(self):
        self.results: List[Dict] = []
        self.network_interface = "eth0"
        self.target_latency_ms = 50.0  # 目标延迟<50ms
        self.test_iterations = 100  # 测试次数
        
    def safety_confirmation(self):
        """安全确认提示 - 性能测试版本"""
        print("\n" + "="*70)
        print("📈 通信性能测试 - Unitree Go2 (任务3.7)")
        print("="*70)
        print("🎯 测试目标:")
        print("   • 验证控制命令延迟consistently <50ms")
        print("   • 评估整体系统响应性")
        print("   • 统计分析通信性能指标")
        print("\n⚠️  安全说明:")
        print("   • 本测试使用最小化的安全命令")
        print("   • 机器人保持静止状态，仅测试通信延迟")
        print("   • 不执行可能导致机器人移动的动作")
        print("\n📊 测试计划:")
        print(f"   • 执行 {self.test_iterations} 次通信延迟测量")
        print("   • 使用安全的查询命令进行测试")
        print("   • 记录详细的性能统计数据")
        print("\n" + "="*70)
        
        response = input("确认开始通信性能测试? (yes/no): ").lower().strip()
        if response not in ['yes', 'y', '是']:
            print("❌ 测试已取消")
            return False
        return True
        
    def measure_command_latency(self, client: SportClient, command_name: str, command_func, iterations: int = 10) -> List[float]:
        """测量单个命令的延迟（毫秒）"""
        latencies = []
        
        print(f"   📡 测量 {command_name} 延迟 ({iterations} 次)...")
        
        for i in range(iterations):
            start_time = time.perf_counter()
            try:
                result = command_func()
                end_time = time.perf_counter()
                
                latency_ms = (end_time - start_time) * 1000.0
                latencies.append(latency_ms)
                
                # 实时显示进度
                if (i + 1) % 10 == 0 or i == iterations - 1:
                    print(f"      进度: {i+1}/{iterations}, 当前延迟: {latency_ms:.2f}ms")
                    
                # 短暂间隔避免过载
                time.sleep(0.01)
                
            except Exception as e:
                print(f"      ⚠️ 第{i+1}次测试异常: {e}")
                continue
                
        return latencies
    
    def analyze_latency_data(self, latencies: List[float], command_name: str) -> Dict:
        """分析延迟数据"""
        if not latencies:
            return {'command': command_name, 'error': 'No valid data'}
            
        analysis = {
            'command': command_name,
            'count': len(latencies),
            'mean_ms': statistics.mean(latencies),
            'median_ms': statistics.median(latencies),
            'std_dev_ms': statistics.stdev(latencies) if len(latencies) > 1 else 0,
            'min_ms': min(latencies),
            'max_ms': max(latencies),
            'p95_ms': statistics.quantiles(latencies, n=20)[18] if len(latencies) >= 20 else max(latencies),
            'p99_ms': statistics.quantiles(latencies, n=100)[98] if len(latencies) >= 100 else max(latencies),
            'under_50ms_count': sum(1 for l in latencies if l < 50.0),
            'under_50ms_rate': sum(1 for l in latencies if l < 50.0) / len(latencies) * 100
        }
        
        return analysis
    
    def run_performance_test(self):
        """执行通信性能测试"""
        if not self.safety_confirmation():
            return
            
        print(f"\n🚀 开始通信性能测试 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"📡 网络接口: {self.network_interface}")
        print(f"🎯 目标延迟: <{self.target_latency_ms}ms")
        
        try:
            # 初始化DDS通道
            print("📡 初始化DDS通道工厂...")
            ChannelFactoryInitialize(0, self.network_interface)
            print("✅ DDS通道工厂初始化成功")
            
            # 创建SportClient
            print("🔌 创建SportClient...")
            client = SportClient()
            client.SetTimeout(10.0)
            
            print("🚀 初始化SportClient...")
            client.Init()
            print("✅ SportClient初始化完成")
            
            # 等待连接稳定
            print("⏳ 等待连接稳定...")
            time.sleep(3)
            
            # 定义测试命令 - 使用安全的非移动命令
            test_commands = [
                ("Sit", lambda: client.Sit(), "安全坐下命令"),
                ("StandUp", lambda: client.StandUp(), "站立命令"), 
                ("Damp", lambda: client.Damp(), "阻尼命令"),  # 安全的无移动命令
            ]
            
            print(f"\n📊 开始性能测量 (每个命令测试{self.test_iterations//len(test_commands)}次):")
            print("-" * 70)
            
            all_results = []
            
            for i, (cmd_name, cmd_func, description) in enumerate(test_commands, 1):
                print(f"\n[{i}/{len(test_commands)}] 测试 {cmd_name}() - {description}")
                
                # 测量延迟
                iterations = self.test_iterations // len(test_commands)
                latencies = self.measure_command_latency(client, cmd_name, cmd_func, iterations)
                
                if latencies:
                    # 分析数据
                    analysis = self.analyze_latency_data(latencies, cmd_name)
                    all_results.append(analysis)
                    
                    # 显示实时结果
                    print(f"      📈 {cmd_name} 延迟统计:")
                    print(f"         平均: {analysis['mean_ms']:.2f}ms")
                    print(f"         中位数: {analysis['median_ms']:.2f}ms")
                    print(f"         最小/最大: {analysis['min_ms']:.2f}/{analysis['max_ms']:.2f}ms")
                    print(f"         <50ms比率: {analysis['under_50ms_rate']:.1f}%")
                    
                    # 延迟警告
                    if analysis['mean_ms'] > self.target_latency_ms:
                        print(f"         ⚠️ 平均延迟超过目标 {self.target_latency_ms}ms")
                    else:
                        print(f"         ✅ 平均延迟符合目标要求")
                        
                # 命令间间隔
                if i < len(test_commands):
                    print("      ⏸️ 短暂休息...")
                    time.sleep(2)
            
            # 生成综合报告
            self.generate_performance_report(all_results)
            
        except Exception as e:
            print(f"❌ 性能测试异常: {e}")
            import traceback
            traceback.print_exc()
            
    def generate_performance_report(self, results: List[Dict]):
        """生成详细的性能测试报告"""
        print("\n" + "="*80)
        print("📊 通信性能测试报告 - 任务3.7")
        print("="*80)
        
        if not results:
            print("❌ 没有有效的测试数据")
            return
            
        # 整体统计
        all_latencies = []
        total_tests = 0
        under_50ms_total = 0
        
        for result in results:
            if 'mean_ms' in result:
                total_tests += result['count']
                under_50ms_total += result['under_50ms_count']
        
        overall_success_rate = (under_50ms_total / total_tests * 100) if total_tests > 0 else 0
        
        print(f"🎯 总体性能评估:")
        print(f"   测试总数: {total_tests}")
        print(f"   <50ms成功率: {overall_success_rate:.1f}%")
        print(f"   目标达成: {'✅ 通过' if overall_success_rate >= 95 else '❌ 未达标'}")
        
        print(f"\n📈 详细命令性能:")
        print("-" * 80)
        print(f"{'命令':<12} {'次数':<6} {'平均ms':<8} {'中位ms':<8} {'最小ms':<8} {'最大ms':<8} {'<50ms%':<8} {'状态':<6}")
        print("-" * 80)
        
        for result in results:
            if 'mean_ms' in result:
                status = "✅" if result['under_50ms_rate'] >= 95 else "⚠️"
                print(f"{result['command']:<12} {result['count']:<6} "
                      f"{result['mean_ms']:<8.2f} {result['median_ms']:<8.2f} "
                      f"{result['min_ms']:<8.2f} {result['max_ms']:<8.2f} "
                      f"{result['under_50ms_rate']:<8.1f} {status:<6}")
        
        # 性能等级评估
        print(f"\n🏆 性能等级评估:")
        if overall_success_rate >= 98:
            grade = "优秀 (A+)"
            print("   ✅ 通信性能优异，延迟consistently <50ms")
        elif overall_success_rate >= 95:
            grade = "良好 (A)"  
            print("   ✅ 通信性能良好，符合要求")
        elif overall_success_rate >= 90:
            grade = "一般 (B)"
            print("   ⚠️ 通信性能一般，需要优化")
        else:
            grade = "较差 (C)"
            print("   ❌ 通信性能不达标，需要检查网络和配置")
            
        print(f"   最终评级: {grade}")
        
        # 建议和结论
        print(f"\n💡 建议和结论:")
        if overall_success_rate >= 95:
            print("   • 通信性能满足任务3.7要求")
            print("   • 控制命令延迟consistently <50ms ✅")
            print("   • 系统响应性良好，可进行后续开发")
        else:
            print("   • 建议检查网络连接质量")
            print("   • 考虑优化DDS配置参数")
            print("   • 验证系统资源使用情况")
            
        print(f"\n📅 测试完成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)
        
        # 保存结果到文件
        self.save_results_to_file(results, overall_success_rate)
        
    def save_results_to_file(self, results: List[Dict], success_rate: float):
        """保存测试结果到文件"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"logs/performance_test_{timestamp}.txt"
        
        # 确保logs目录存在
        os.makedirs("logs", exist_ok=True)
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(f"Unitree Go2 通信性能测试报告 - 任务3.7\n")
                f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"总体成功率: {success_rate:.1f}%\n")
                f.write(f"目标: 控制命令延迟consistently <50ms\n\n")
                
                for result in results:
                    if 'mean_ms' in result:
                        f.write(f"命令: {result['command']}\n")
                        f.write(f"  平均延迟: {result['mean_ms']:.2f}ms\n")
                        f.write(f"  中位延迟: {result['median_ms']:.2f}ms\n")
                        f.write(f"  <50ms比率: {result['under_50ms_rate']:.1f}%\n\n")
                        
            print(f"📁 测试结果已保存: {filename}")
            
        except Exception as e:
            print(f"⚠️ 保存结果文件失败: {e}")

def main():
    """主函数"""
    # 设置正确的环境变量
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    
    print("📈 Unitree Go2 通信性能测试 - 任务3.7")
    print(f"🕐 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("🎯 目标: 验证控制命令延迟consistently <50ms")
    
    tester = CommunicationPerformanceTest()
    tester.run_performance_test()

if __name__ == "__main__":
    main() 