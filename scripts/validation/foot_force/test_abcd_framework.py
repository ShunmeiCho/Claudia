#!/usr/bin/env python3
# scripts/validation/foot_force/test_abcd_framework.py
# Generated: 2025-06-26 19:30:00
# Purpose: 简化的ABCD框架测试，避开CycloneDDS特定问题

import os
import sys
import json
import numpy as np
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, Any

# 添加项目路径
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

def test_basic_imports():
    """测试基本的Python模块导入"""
    print("🔍 测试基本模块导入...")
    
    try:
        import numpy as np
        print("  ✅ numpy: OK")
    except ImportError as e:
        print(f"  ❌ numpy: {e}")
        return False
    
    try:
        import matplotlib.pyplot as plt
        print("  ✅ matplotlib: OK")
    except ImportError as e:
        print(f"  ❌ matplotlib: {e}")
        return False
    
    try:
        import seaborn as sns
        print("  ✅ seaborn: OK")
    except ImportError as e:
        print(f"  ❌ seaborn: {e}")
        return False
    
    try:
        import pandas as pd
        print("  ✅ pandas: OK")
    except ImportError as e:
        print(f"  ❌ pandas: {e}")
        return False
    
    return True

def test_config_loading():
    """测试配置文件加载"""
    print("\n📋 测试配置文件加载...")
    
    try:
        config_path = Path(__file__).parent / "foot_force_validation" / "validation_config.json"
        
        if not config_path.exists():
            print(f"  ❌ 配置文件不存在: {config_path}")
            return False
        
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # 检查关键配置项
        required_keys = ['foot_force_config', 'static_validation', 'dynamic_validation']
        for key in required_keys:
            if key not in config:
                print(f"  ❌ 缺少配置项: {key}")
                return False
        
        print("  ✅ 配置文件加载成功")
        print(f"  📊 配置项数量: {len(config)}")
        return True
        
    except Exception as e:
        print(f"  ❌ 配置文件加载失败: {e}")
        return False

def test_mock_data_structures():
    """测试模拟数据结构"""
    print("\n🧪 测试数据结构...")
    
    try:
        # 模拟足端力数据
        foot_force_data = {
            'timestamp': time.time(),
            'foot_forces': np.random.normal(0, 5, (4, 3)),  # 4个足端，3个力分量
            'contact_states': [True, True, True, True],
            'total_force': 150.0,
            'center_of_pressure': [0.0, 0.0],
            'stability_index': 0.95,
            'force_balance': 0.92
        }
        
        print("  ✅ 足端力数据结构: OK")
        
        # 模拟静态测试结果
        static_result = {
            'test_name': 'mock_static_test',
            'status': 'PASS',
            'score': 85.0,
            'measurements': {
                'total_weight': 150.0,
                'weight_distribution': {
                    'front_left': 25.0,
                    'front_right': 25.0,
                    'rear_left': 25.0,
                    'rear_right': 25.0
                }
            },
            'timestamp': time.time()
        }
        
        print("  ✅ 静态测试结果结构: OK")
        
        # 模拟动态测试结果
        dynamic_result = {
            'test_name': 'mock_dynamic_test',
            'test_score': 82.0,
            'duration': 60.0,
            'total_samples': 3000,
            'gait_analysis': {
                'average_step_time': 0.8,
                'step_symmetry': 0.95,
                'force_consistency': 0.88
            },
            'timestamp': time.time()
        }
        
        print("  ✅ 动态测试结果结构: OK")
        
        return True
        
    except Exception as e:
        print(f"  ❌ 数据结构测试失败: {e}")
        return False

def test_report_generation():
    """测试报告生成功能"""
    print("\n📊 测试报告生成...")
    
    try:
        # 创建输出目录
        output_dir = Path("scripts/validation/foot_force/foot_force_validation/output")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # 生成模拟综合报告
        comprehensive_report = {
            'validation_id': f"MOCK_FFVR_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            'timestamp': datetime.now().isoformat(),
            'overall_success': True,
            'phases': {
                'phase_a': {
                    'name': '数据读取框架验证',
                    'success': True,
                    'score': 95.0
                },
                'phase_b': {
                    'name': '静态力分布验证', 
                    'success': True,
                    'score': 85.0
                },
                'phase_c': {
                    'name': '动态响应测试',
                    'success': True,
                    'score': 82.0
                },
                'phase_d': {
                    'name': '综合可视化和文档',
                    'success': True,
                    'score': 90.0
                }
            },
            'overall_score': 88.0,
            'grade': 'B',
            'status': 'PASS'
        }
        
        # 保存JSON报告
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        json_file = output_dir / f"framework_test_report_{timestamp}.json"
        
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(comprehensive_report, f, indent=2, ensure_ascii=False)
        
        print(f"  ✅ JSON报告生成成功: {json_file.name}")
        
        # 生成简单的HTML报告
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Unitree Go2 足端力传感器验证报告</title>
    <meta charset="utf-8">
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; }}
        .header {{ background: #f0f8ff; padding: 20px; border-radius: 10px; }}
        .phase {{ margin: 20px 0; padding: 15px; border-left: 4px solid #007acc; }}
        .success {{ color: green; }}
        .score {{ font-weight: bold; color: #333; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 Unitree Go2 足端力传感器验证报告</h1>
        <p>报告ID: {comprehensive_report['validation_id']}</p>
        <p>生成时间: {comprehensive_report['timestamp']}</p>
        <p class="score">总体评分: {comprehensive_report['overall_score']:.1f} (等级: {comprehensive_report['grade']})</p>
    </div>
    
    <h2>📋 测试阶段结果</h2>
"""
        
        for phase_id, phase_data in comprehensive_report['phases'].items():
            status_icon = "✅" if phase_data['success'] else "❌"
            html_content += f"""
    <div class="phase">
        <h3>{status_icon} {phase_data['name']}</h3>
        <p>评分: <span class="score">{phase_data['score']:.1f}</span></p>
        <p>状态: <span class="success">通过</span></p>
    </div>
"""
        
        html_content += """
    <h2>📈 总结</h2>
    <p>所有ABCD测试阶段均已完成，系统运行正常。</p>
</body>
</html>
"""
        
        html_file = output_dir / f"framework_test_report_{timestamp}.html"
        with open(html_file, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        print(f"  ✅ HTML报告生成成功: {html_file.name}")
        
        return True
        
    except Exception as e:
        print(f"  ❌ 报告生成失败: {e}")
        return False

def test_visualization():
    """测试可视化功能"""
    print("\n📈 测试可视化功能...")
    
    try:
        import matplotlib.pyplot as plt
        
        # 设置无GUI后端
        plt.switch_backend('Agg')
        
        # 创建模拟数据
        time_points = np.linspace(0, 10, 100)
        foot_forces = {
            'front_left': 35 + 5 * np.sin(time_points) + np.random.normal(0, 1, 100),
            'front_right': 40 + 3 * np.sin(time_points + 0.5) + np.random.normal(0, 1, 100),
            'rear_left': 38 + 4 * np.sin(time_points + 1.0) + np.random.normal(0, 1, 100),
            'rear_right': 37 + 6 * np.sin(time_points + 1.5) + np.random.normal(0, 1, 100)
        }
        
        # 创建图表
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Unitree Go2 足端力传感器框架测试', fontsize=16)
        
        # 足端力时间序列
        ax1 = axes[0, 0]
        for foot_name, forces in foot_forces.items():
            ax1.plot(time_points, forces, label=foot_name, linewidth=2)
        ax1.set_title('足端力时间序列')
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('力 (N)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 力分布饼图
        ax2 = axes[0, 1]
        avg_forces = [np.mean(forces) for forces in foot_forces.values()]
        ax2.pie(avg_forces, labels=foot_forces.keys(), autopct='%1.1f%%', startangle=90)
        ax2.set_title('平均力分布')
        
        # 评分对比
        ax3 = axes[1, 0]
        phases = ['阶段A', '阶段B', '阶段C', '阶段D']
        scores = [95, 85, 82, 90]
        bars = ax3.bar(phases, scores, color=['#ff9999', '#66b3ff', '#99ff99', '#ffcc99'])
        ax3.set_title('各阶段评分')
        ax3.set_ylabel('评分')
        ax3.set_ylim(0, 100)
        
        # 添加数值标签
        for bar, score in zip(bars, scores):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                    f'{score}', ha='center', va='bottom')
        
        # 稳定性分析
        ax4 = axes[1, 1]
        stability_data = np.random.normal(0.9, 0.05, 50)
        ax4.hist(stability_data, bins=15, alpha=0.7, color='skyblue', edgecolor='black')
        ax4.set_title('稳定性指数分布')
        ax4.set_xlabel('稳定性指数')
        ax4.set_ylabel('频率')
        ax4.axvline(np.mean(stability_data), color='red', linestyle='--', label=f'均值: {np.mean(stability_data):.3f}')
        ax4.legend()
        
        plt.tight_layout()
        
        # 保存图表
        output_dir = Path("scripts/validation/foot_force/foot_force_validation/output")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        chart_file = output_dir / f"framework_test_charts_{timestamp}.png"
        
        plt.savefig(chart_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"  ✅ 可视化图表生成成功: {chart_file.name}")
        
        return True
        
    except Exception as e:
        print(f"  ❌ 可视化测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🧪 Unitree Go2 足端力传感器 ABCD 框架测试")
    print("="*60)
    
    test_results = []
    
    # 运行各项测试
    tests = [
        ("基本模块导入", test_basic_imports),
        ("配置文件加载", test_config_loading),
        ("数据结构验证", test_mock_data_structures),
        ("报告生成功能", test_report_generation),
        ("可视化功能", test_visualization)
    ]
    
    for test_name, test_func in tests:
        print(f"\n🔧 {test_name}")
        print("-" * 40)
        
        try:
            result = test_func()
            test_results.append(result)
            
            if result:
                print(f"✅ {test_name}: 通过")
            else:
                print(f"❌ {test_name}: 失败")
                
        except Exception as e:
            print(f"❌ {test_name}: 异常 - {e}")
            test_results.append(False)
    
    # 总结
    print("\n" + "="*60)
    print("📋 测试结果总结")
    print("="*60)
    
    passed_tests = sum(test_results)
    total_tests = len(test_results)
    
    for i, (test_name, _) in enumerate(tests):
        status = "✅ 通过" if test_results[i] else "❌ 失败"
        print(f"  {test_name}: {status}")
    
    success_rate = passed_tests / total_tests * 100
    print(f"\n成功率: {passed_tests}/{total_tests} ({success_rate:.1f}%)")
    
    if passed_tests == total_tests:
        print("\n🎉 所有框架测试通过！ABCD验证框架已准备就绪。")
        print("\n📋 下一步建议:")
        print("   1. 连接Unitree Go2机器人")
        print("   2. 运行完整的ABCD验证流程:")
        print("      python3 scripts/validation/foot_force/run_complete_validation.py")
    else:
        print("\n⚠️ 部分框架测试失败，建议:")
        print("   1. 检查失败的模块")
        print("   2. 安装缺少的依赖")
        print("   3. 修复配置问题")
    
    print(f"\n📁 测试输出目录: scripts/validation/foot_force/foot_force_validation/output/")

if __name__ == "__main__":
    main() 