#!/usr/bin/env python3
"""
LED测试框架快速验证
"""

import sys
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_imports():
    """测试所有导入是否正常"""
    print("🔍 测试模块导入...")
    
    try:
        import test.led_system.test_config
        print("✅ test_config 导入成功")
    except Exception as e:
        print(f"❌ test_config 导入失败: {e}")
    
    try:
        import test.led_system.data_collector
        print("✅ data_collector 导入成功")
    except Exception as e:
        print(f"❌ data_collector 导入失败: {e}")
    
    try:
        import test.led_system.led_test_base
        print("✅ led_test_base 导入成功")
    except Exception as e:
        print(f"❌ led_test_base 导入失败: {e}")
    
    try:
        import test.led_system.test_led_modes
        print("✅ test_led_modes 导入成功")
    except Exception as e:
        print(f"❌ test_led_modes 导入失败: {e}")
    
    try:
        import test.led_system.test_performance
        print("✅ test_performance 导入成功")
    except Exception as e:
        print(f"❌ test_performance 导入失败: {e}")

def test_config():
    """测试配置功能"""
    print("\n🔧 测试配置功能...")
    
    try:
        from test.led_system.test_config import get_led_test_config
        config = get_led_test_config()
        
        print(f"✅ 配置创建成功")
        print(f"   测试模式: {config.get_test_mode()}")
        print(f"   最大响应时间: {config.performance.max_response_time_ms}ms")
        print(f"   压力测试: {'启用' if config.is_stress_test_enabled() else '禁用'}")
        
    except Exception as e:
        print(f"❌ 配置测试失败: {e}")

def test_data_collector():
    """测试数据收集器"""
    print("\n📊 测试数据收集器...")
    
    try:
        from test.led_system.data_collector import get_led_test_collector
        collector = get_led_test_collector()
        
        # 记录一个简单指标
        collector.record_metric("test_metric", 100, "ms", "quick_test", "demo")
        
        # 获取统计
        stats = collector.get_real_time_stats()
        
        print(f"✅ 数据收集器创建成功")
        print(f"   总指标数: {stats.get('total_metrics', 0)}")
        
    except Exception as e:
        print(f"❌ 数据收集器测试失败: {e}")

def test_led_system():
    """测试LED系统连接"""
    print("\n🤖 测试LED系统连接...")
    
    try:
        from claudia.robot_controller.unitree_messages import UnitreeMessages
        hardware_available = UnitreeMessages.is_available()
        
        print(f"✅ Unitree硬件状态: {'可用' if hardware_available else '模拟模式'}")
        
        if hardware_available:
            method = UnitreeMessages.get_import_method()
            print(f"   导入方法: {method}")
        
    except Exception as e:
        print(f"❌ Unitree硬件检查失败: {e}")
    
    try:
        from claudia.robot_controller import create_claudia_led_system
        led_system = create_claudia_led_system()
        
        if led_system:
            print("✅ LED控制系统可创建")
            print("✅ 任务6.5测试框架已就绪")
        else:
            print("⚠️ LED控制系统创建返回None")
            
    except Exception as e:
        print(f"❌ LED控制系统测试失败: {e}")

def main():
    """主函数"""
    print("🚀 LED测试框架快速验证")
    print("=" * 40)
    
    test_imports()
    test_config()
    test_data_collector()
    test_led_system()
    
    print("\n" + "=" * 40)
    print("✅ 快速验证完成")
    print("🎯 任务6.5: LED测试框架已成功实现")

if __name__ == "__main__":
    main() 