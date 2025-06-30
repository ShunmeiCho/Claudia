#!/usr/bin/env python3
# scripts/validation/imu/test_imu_validation_fix.py
# Generated: 2025-06-27 12:15:30 CST
# Purpose: 测试IMU验证修复，检查方法缺失问题

import sys
import os
from pathlib import Path
import logging

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(Path(__file__).parent / "imu_validation"))

def test_method_availability():
    """测试关键方法是否可用"""
    print("🔍 测试IMU验证方法可用性...")
    
    try:
        # 测试导入
        from imu_validation.static_tester import IMUStaticTester
        from imu_validation.dynamic_tester import IMUDynamicTester  
        from imu_validation.calibration_analyzer import IMUCalibrationAnalyzer
        from imu_validation.data_collector import IMUDataCollector
        from imu_validation.visualizer import IMUVisualizer
        from imu_validation.imu_config import IMUConfig
        print("✅ 所有模块导入成功")
        
        # 检查方法存在性
        methods_to_check = [
            (IMUStaticTester, 'run_static_stability_test'),
            (IMUDynamicTester, 'run_dynamic_response_test'),
            (IMUCalibrationAnalyzer, 'run_comprehensive_calibration_analysis'),
            (IMUDataCollector, 'get_collected_data'),
            (IMUDataCollector, 'stop_collection'),
            (IMUDataCollector, 'get_real_time_metrics'),
            (IMUVisualizer, 'get_plot_statistics'),
            (IMUVisualizer, 'save_current_plots'),
            (IMUVisualizer, 'stop_visualization'),
        ]
        
        missing_methods = []
        
        for cls, method_name in methods_to_check:
            if hasattr(cls, method_name):
                print(f"✅ {cls.__name__}.{method_name} - 存在")
            else:
                print(f"❌ {cls.__name__}.{method_name} - 缺失")
                missing_methods.append(f"{cls.__name__}.{method_name}")
        
        if missing_methods:
            print(f"\n⚠️ 发现 {len(missing_methods)} 个缺失方法:")
            for method in missing_methods:
                print(f"  - {method}")
            return False
        else:
            print("\n🎉 所有必需方法都已实现!")
            return True
            
    except Exception as e:
        print(f"❌ 导入测试失败: {e}")
        return False

def test_configuration_loading():
    """测试配置加载"""
    print("\n🔧 测试配置加载...")
    
    try:
        from imu_validation.main_validation_script import IMUValidationSuite
        
        # 测试默认配置
        suite = IMUValidationSuite()
        print("✅ 默认配置加载成功")
        
        # 检查重要配置项
        required_config_keys = [
            'test_parameters',
            'quality_thresholds', 
            'imu_config',
            'visualization_config'
        ]
        
        missing_configs = []
        for key in required_config_keys:
            if key in suite.config:
                print(f"✅ 配置项 {key} - 存在")
            else:
                print(f"❌ 配置项 {key} - 缺失")
                missing_configs.append(key)
        
        if missing_configs:
            print(f"\n⚠️ 缺失配置项: {missing_configs}")
            return False
        else:
            print("\n🎉 所有配置项完整!")
            return True
            
    except Exception as e:
        print(f"❌ 配置加载测试失败: {e}")
        return False

def test_mock_validation():
    """测试模拟验证流程"""
    print("\n🧪 测试模拟验证流程...")
    
    try:
        # 创建模拟配置
        mock_config = {
            'test_parameters': {
                'static_test': {'duration_seconds': 5},
                'dynamic_test': {'duration_seconds': 10},
                'calibration_analysis': {'gravity_reference': 9.81}
            },
            'quality_thresholds': {
                'accuracy': {'gravity_error_max_percent': 5}
            },
            'imu_config': {
                'sampling_rate_hz': 100,
                'buffer_size': 1000
            },
            'visualization_config': {
                'window_size': 100,
                'update_rate_hz': 10
            }
        }
        
        # 验证配置结构
        if all(key in mock_config for key in ['test_parameters', 'quality_thresholds', 'imu_config']):
            print("✅ 模拟配置结构正确")
        else:
            print("❌ 模拟配置结构不完整")
            return False
        
        # 测试类实例化（不连接硬件）
        from imu_validation.data_collector import IMUDataCollector
        
        collector = IMUDataCollector(mock_config)
        print("✅ 数据采集器实例化成功")
        
        # 测试基本方法调用
        stats = collector.get_real_time_metrics()
        if isinstance(stats, dict):
            print("✅ 实时指标获取成功")
        else:
            print("❌ 实时指标获取失败")
            return False
        
        print("\n🎉 模拟验证流程测试通过!")
        return True
        
    except Exception as e:
        print(f"❌ 模拟验证测试失败: {e}")
        return False

def print_usage_guide():
    """打印使用指导"""
    print("\n" + "="*60)
    print("📋 IMU验证操作指导")
    print("="*60)
    
    print("\n🔧 测试目的说明:")
    print("1. 静态稳定性测试 (Static Stability):")
    print("   • 用途: 验证IMU在静止时的精度和稳定性")
    print("   • 操作: 保持机器人完全静止60秒 (不需要移动)")
    print("   • 检测: 重力精度、传感器噪声、温度漂移")
    
    print("\n2. 动态响应测试 (Dynamic Response):")
    print("   • 用途: 验证IMU对运动的响应速度和准确性")  
    print("   • 操作: 轻柔地移动机器人进行俯仰、横滚、偏航运动")
    print("   • 检测: 响应时间、跟踪精度、动态范围")
    
    print("\n3. 校准质量测试 (Calibration Quality):")
    print("   • 用途: 验证工厂校准状态和多轴耦合")
    print("   • 操作: 将机器人放置在6个标准姿态:")
    print("     - 正常站立")
    print("     - 左侧倾斜90度") 
    print("     - 右侧倾斜90度")
    print("     - 前倾90度") 
    print("     - 后倾90度")
    print("     - 倒置180度")
    print("   • 检测: 比例因子、交叉轴耦合、校准质量")
    
    print("\n🚀 运行修复后的验证:")
    print("cd scripts/validation/imu/imu_validation")
    print("python3 main_validation_script.py")
    
    print("\n⚠️ 注意事项:")
    print("• 确保机器人已正确连接和初始化")
    print("• 动态测试时请缓慢移动，避免剧烈振动")
    print("• 校准测试需要足够的操作空间")
    print("• 每个姿态保持10-15秒稳定")

def main():
    """主函数"""
    print("🧰 IMU验证修复测试工具")
    print("="*40)
    
    # 设置日志
    logging.basicConfig(level=logging.WARNING)
    
    # 运行测试
    test_results = []
    
    test_results.append(("方法可用性", test_method_availability()))
    test_results.append(("配置加载", test_configuration_loading()))
    test_results.append(("模拟验证", test_mock_validation()))
    
    # 汇总结果
    print("\n" + "="*60)
    print("📊 测试结果汇总")
    print("="*60)
    
    passed = 0
    total = len(test_results)
    
    for test_name, result in test_results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总体结果: {passed}/{total} 测试通过")
    
    if passed == total:
        print("🎉 所有测试通过! IMU验证方法缺失问题已修复")
        print_usage_guide()
        return 0
    else:
        print("⚠️ 部分测试失败，请检查修复情况")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 