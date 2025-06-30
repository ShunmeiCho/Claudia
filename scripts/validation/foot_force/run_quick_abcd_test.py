#!/usr/bin/env python3
# scripts/validation/foot_force/run_quick_abcd_test.py
# Generated: 2025-06-26 19:10:00
# Purpose: Unitree Go2 足端力传感器快速ABCD验证测试

import os
import sys
import json
import time
from pathlib import Path
from datetime import datetime

# 设置环境变量
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

# 添加项目路径
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

def test_abcd_components():
    """测试ABCD各个组件是否正常工作"""
    
    print("🧪 Unitree Go2 足端力传感器 ABCD 组件测试")
    print("="*60)
    
    results = {
        'phase_a': False,
        'phase_b': False,
        'phase_c': False,
        'phase_d': False
    }
    
    # 测试阶段A: 数据读取框架
    print("\n🔍 测试阶段A: 数据读取框架")
    try:
        sys.path.append(str(Path(__file__).parent / "foot_force_validation"))
        from foot_force_validation.foot_force_config import FootForceConfig
        
        config_path = Path(__file__).parent / "foot_force_validation" / "validation_config.json"
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # 模拟测试FootForceConfig
        foot_config = FootForceConfig(network_interface="eth0")
        print("  ✅ FootForceConfig 初始化成功")
        results['phase_a'] = True
        
    except Exception as e:
        print(f"  ❌ 阶段A测试失败: {e}")
    
    # 测试阶段B: 静态验证
    print("\n⚖️ 测试阶段B: 静态验证框架")
    try:
        from foot_force_validation.static_tester import StaticFootForceTester
        print("  ✅ StaticFootForceTester 导入成功")
        results['phase_b'] = True
        
    except Exception as e:
        print(f"  ❌ 阶段B测试失败: {e}")
    
    # 测试阶段C: 动态测试
    print("\n🏃 测试阶段C: 动态测试框架")
    try:
        from foot_force_validation.dynamic_tester import DynamicFootForceTester
        print("  ✅ DynamicFootForceTester 导入成功")
        results['phase_c'] = True
        
    except Exception as e:
        print(f"  ❌ 阶段C测试失败: {e}")
    
    # 测试阶段D: 综合可视化
    print("\n📊 测试阶段D: 综合可视化框架")
    try:
        from foot_force_validation.comprehensive_dashboard import ComprehensiveFootForceDashboard
        print("  ✅ ComprehensiveFootForceDashboard 导入成功")
        results['phase_d'] = True
        
    except Exception as e:
        print(f"  ❌ 阶段D测试失败: {e}")
    
    # 汇总结果
    print("\n📋 测试结果汇总:")
    print("="*60)
    
    success_count = sum(results.values())
    total_count = len(results)
    
    for phase, success in results.items():
        status = "✅ 通过" if success else "❌ 失败"
        print(f"  {phase.upper()}: {status}")
    
    print(f"\n总体结果: {success_count}/{total_count} 阶段通过")
    
    if success_count == total_count:
        print("🎉 所有ABCD组件测试通过！可以运行完整验证流程。")
        return True
    else:
        print("⚠️ 部分组件测试失败，请检查相关模块。")
        return False

def run_minimal_test():
    """运行最小化的足端力测试"""
    print("\n🔬 运行最小化足端力验证测试")
    print("-"*60)
    
    try:
        # 阶段A: 模拟数据读取
        print("📊 阶段A: 模拟数据读取测试...")
        import numpy as np
        
        # 模拟足端力数据
        mock_data = {
            'timestamp': time.time(),
            'foot_forces': [[10.0, 5.0, 40.0] for _ in range(4)],  # 4个足端
            'contact_states': [True, True, True, True],
            'total_force': 160.0
        }
        print(f"  ✅ 模拟数据生成: 总力 {mock_data['total_force']}N")
        
        # 阶段B: 模拟静态验证
        print("🧪 阶段B: 模拟静态验证测试...")
        static_score = 85.0
        print(f"  ✅ 模拟静态验证: 评分 {static_score}")
        
        # 阶段C: 模拟动态测试
        print("🏃 阶段C: 模拟动态测试...")
        dynamic_scores = [82.0, 78.5, 85.2]
        avg_dynamic = np.mean(dynamic_scores)
        print(f"  ✅ 模拟动态测试: 平均评分 {avg_dynamic:.1f}")
        
        # 阶段D: 模拟报告生成
        print("📊 阶段D: 模拟报告生成...")
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'static_score': static_score,
            'dynamic_score': avg_dynamic,
            'overall_score': static_score * 0.6 + avg_dynamic * 0.4
        }
        
        # 保存模拟报告
        output_dir = Path("scripts/validation/foot_force/foot_force_validation/output")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        report_file = output_dir / f"mock_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)
        
        print(f"  ✅ 模拟报告已保存: {report_file.name}")
        
        print(f"\n🎯 模拟测试完成:")
        print(f"   总体评分: {report_data['overall_score']:.1f}")
        print(f"   报告文件: {report_file}")
        
        return True
        
    except Exception as e:
        print(f"❌ 最小化测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🚀 启动快速ABCD组件测试")
    
    # 1. 测试组件导入
    components_ok = test_abcd_components()
    
    # 2. 运行最小化测试
    minimal_test_ok = run_minimal_test()
    
    # 3. 给出建议
    print("\n💡 建议:")
    
    if components_ok and minimal_test_ok:
        print("✅ 所有测试通过！您可以运行完整的ABCD验证:")
        print("   python3 scripts/validation/foot_force/run_complete_validation.py")
    elif components_ok:
        print("⚠️ 组件测试通过，但需要连接机器人进行实际测试")
        print("   请确保机器人网络连接正常，然后运行完整验证")
    else:
        print("❌ 组件测试失败，请检查:")
        print("   1. 确保所有依赖包已安装")
        print("   2. 检查 cyclonedds 环境配置")
        print("   3. 验证 Python 路径设置")
    
    print(f"\n📁 输出目录: scripts/validation/foot_force/foot_force_validation/output/")

if __name__ == "__main__":
    main() 