#!/usr/bin/env python3
"""
LED控制系统测试框架演示
验证任务6.5的测试框架功能
"""

import sys
import time
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# 导入测试组件
import test.led_system.test_config as test_config
import test.led_system.data_collector as data_collector
import test.led_system.led_test_base as led_test_base

def demo_test_config():
    """演示测试配置功能"""
    print("🔧 ===== 测试配置演示 =====")
    
    config = test_config.get_led_test_config()
    config.print_config_summary()
    
    # 测试配置保存
    config.save_config()
    
    print("✅ 测试配置演示完成\n")

def demo_data_collector():
    """演示数据收集功能"""
    print("📊 ===== 数据收集器演示 =====")
    
    collector = data_collector.get_led_test_collector()
    
    # 开始测试会话
    collector.start_test_session("demo_test", {"type": "demo"})
    
    # 模拟一些性能数据
    for i in range(10):
        collector.record_performance_data(
            "demo_test",
            response_time=50 + i * 2,
            cpu_usage=20 + i,
            memory_usage=100 + i * 0.5,
            success=i < 9  # 最后一次失败
        )
        time.sleep(0.1)
    
    # 记录一些指标
    collector.record_metric("demo_metric", 42, "units", "demo_test", "demo")
    
    # 记录一个错误
    collector.record_error("demo_test", "demo_error", "这是一个演示错误")
    
    # 结束会话
    collector.end_test_session("demo_test")
    
    # 显示实时统计
    stats = collector.get_real_time_stats()
    print("📈 实时统计数据:")
    for key, value in stats.items():
        print(f"   {key}: {value}")
    
    # 生成报告
    report_file = collector.generate_report()
    data_file = collector.save_data()
    
    print(f"📋 报告已生成: {report_file}")
    print(f"💾 数据已保存: {data_file}")
    
    print("✅ 数据收集器演示完成\n")

def demo_test_base():
    """演示测试基础类功能"""
    print("🧪 ===== 测试基础类演示 =====")
    
    # 创建一个简单的测试类演示
    class DemoTest(led_test_base.LEDTestBase):
        def demo_performance_test(self):
            """演示性能测试"""
            
            def mock_led_operation():
                """模拟LED操作"""
                time.sleep(0.05)  # 模拟50ms操作
                return True
            
            # 测量性能
            result, duration, success = self.measure_performance(
                "mock_operation", mock_led_operation
            )
            
            print(f"🔄 模拟操作结果: 耗时 {duration:.2f}ms, 成功: {success}")
            
            # 验证响应时间
            try:
                self.assert_response_time("mock_operation", 100.0)  # 100ms阈值
                print("✅ 响应时间验证通过")
            except Exception as e:
                print(f"❌ 响应时间验证失败: {e}")
            
            return True
        
        def demo_stress_test(self):
            """演示压力测试"""
            
            def stress_operation():
                """压力测试操作"""
                time.sleep(0.01)  # 快速操作
            
            # 运行压力测试
            stress_result = self.run_stress_test(
                stress_operation, 
                iterations=20, 
                max_duration=2.0
            )
            
            print(f"🔥 压力测试结果: {stress_result}")
            
            return True
    
    # 运行演示测试
    demo_test = DemoTest()
    demo_test.setUp()
    
    try:
        demo_test.demo_performance_test()
        demo_test.demo_stress_test()
        
        print("✅ 测试基础类演示完成")
    finally:
        demo_test.tearDown()
    
    print()

def demo_led_system_availability():
    """演示LED系统可用性检查"""
    print("🤖 ===== LED系统可用性检查 =====")
    
    try:
        # 检查Unitree硬件可用性
        from claudia.robot_controller.unitree_messages import UnitreeMessages
        hardware_available = UnitreeMessages.is_available()
        
        print(f"🔌 Unitree硬件: {'✅ 可用' if hardware_available else '⚠️ 模拟模式'}")
        
        if hardware_available:
            method = UnitreeMessages.get_import_method()
            print(f"📡 导入方法: {method}")
    except Exception as e:
        print(f"❌ Unitree模块检查失败: {e}")
    
    try:
        # 检查LED控制系统
        from claudia.robot_controller import create_claudia_led_system
        led_system = create_claudia_led_system()
        
        if led_system:
            print("🔆 LED控制系统: ✅ 可创建")
            led_system.initialize()
            print("🔧 LED系统初始化: ✅ 成功")
            
            # 测试一个LED模式
            if hasattr(led_system, 'wake_confirm'):
                led_system.wake_confirm()
                print("💡 LED模式测试: ✅ wake_confirm 执行成功")
            
            led_system.cleanup()
            print("🧹 LED系统清理: ✅ 完成")
        else:
            print("🔆 LED控制系统: ❌ 创建失败")
    
    except Exception as e:
        print(f"❌ LED控制系统检查失败: {e}")
    
    print("✅ LED系统可用性检查完成\n")

def main():
    """主演示函数"""
    print("🎯 LED控制系统测试框架演示")
    print("📅 任务6.5: 全面测试、验证和性能优化")
    print("=" * 60)
    print()
    
    try:
        # 运行各个演示
        demo_test_config()
        demo_data_collector()
        demo_test_base()
        demo_led_system_availability()
        
        print("🎉 ===== 演示完成 =====")
        print("✅ LED测试框架各组件功能正常")
        print("🚀 准备进行完整的LED控制系统测试")
        
    except KeyboardInterrupt:
        print("\n⏹️ 演示被用户中断")
    except Exception as e:
        print(f"\n💥 演示过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 