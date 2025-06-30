#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/static_validation.py
# Generated: 2025-06-27 14:27:00 CST
# Purpose: Unitree Go2 足端力传感器静态验证主脚本

import os
import sys
import time
import json
import logging
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional

# 添加模块路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from foot_force_config import FootForceConfig
from data_collector import FootForceDataCollector
from static_tester import StaticForceTester, StaticTestResult
from visualizer import FootForceVisualizer
from analyzer import FootForceAnalyzer

def setup_logging(log_level: str = 'INFO', log_file: Optional[str] = None) -> None:
    """设置日志系统"""
    
    # 创建日志目录
    log_dir = Path(__file__).parent / 'logs'
    log_dir.mkdir(exist_ok=True)
    
    # 配置日志格式
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    
    # 设置日志级别
    level = getattr(logging, log_level.upper(), logging.INFO)
    
    # 配置根日志器
    handlers = [logging.StreamHandler(sys.stdout)]
    
    if log_file:
        handlers.append(logging.FileHandler(log_file))
    else:
        # 默认日志文件
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_log_file = log_dir / f'static_validation_{timestamp}.log'
        handlers.append(logging.FileHandler(default_log_file))
    
    logging.basicConfig(
        level=level,
        format=log_format,
        handlers=handlers
    )

def load_configuration(config_path: str) -> Dict[str, Any]:
    """加载配置文件"""
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        logging.info(f"配置文件加载成功: {config_path}")
        return config
        
    except Exception as e:
        logging.error(f"加载配置文件失败: {e}")
        raise

def validate_environment() -> bool:
    """验证运行环境"""
    
    logger = logging.getLogger(__name__)
    
    try:
        # 检查必要的模块导入
        import numpy as np
        import matplotlib.pyplot as plt
        import scipy
        
        logger.info("✅ 必要模块导入成功")
        
        # 检查输出目录
        output_dir = Path(__file__).parent / 'output'
        output_dir.mkdir(exist_ok=True)
        
        # 检查日志目录
        log_dir = Path(__file__).parent / 'logs'
        log_dir.mkdir(exist_ok=True)
        
        logger.info("✅ 目录结构验证成功")
        
        return True
        
    except Exception as e:
        logger.error(f"❌ 环境验证失败: {e}")
        return False

def run_zero_load_validation(tester: StaticForceTester, config: Dict[str, Any]) -> StaticTestResult:
    """运行零负载验证"""
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("🔧 阶段B.1: 零负载验证")
    logger.info("=" * 60)
    
    # 获取测试参数
    static_config = config.get('static_validation', {})
    zero_load_duration = static_config.get('zero_load_test_duration', 30.0)
    
    logger.info(f"开始零负载测试，持续时间: {zero_load_duration}秒")
    logger.info("⚠️ 请确保机器人处于悬空状态，足端不接触任何表面！")
    
    # 倒计时
    for i in range(5, 0, -1):
        logger.info(f"测试将在 {i} 秒后开始...")
        time.sleep(1.0)
    
    # 执行零负载测试
    result = tester.run_zero_load_test(zero_load_duration)
    
    # 显示结果
    logger.info(f"零负载测试完成")
    logger.info(f"评分: {result.score:.1f}/100")
    logger.info(f"状态: {result.status}")
    
    if result.recommendations:
        logger.info("建议:")
        for rec in result.recommendations:
            logger.info(f"  • {rec}")
    
    return result

def run_static_standing_validation(tester: StaticForceTester, config: Dict[str, Any]) -> StaticTestResult:
    """运行静态站立验证"""
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("🤖 阶段B.2: 静态站立验证")
    logger.info("=" * 60)
    
    # 获取测试参数
    static_config = config.get('static_validation', {})
    standing_duration = static_config.get('static_standing_duration', 60.0)
    expected_weight = static_config.get('expected_total_force', 150.0)
    
    logger.info(f"开始静态站立测试，持续时间: {standing_duration}秒")
    logger.info(f"期望总重量: {expected_weight}N")
    logger.info("⚠️ 请确保机器人处于正常站立状态，四足平稳接触地面！")
    
    # 倒计时
    for i in range(5, 0, -1):
        logger.info(f"测试将在 {i} 秒后开始...")
        time.sleep(1.0)
    
    # 执行静态站立测试
    result = tester.run_static_standing_test(standing_duration)
    
    # 显示结果
    logger.info(f"静态站立测试完成")
    logger.info(f"评分: {result.score:.1f}/100")
    logger.info(f"状态: {result.status}")
    
    if result.measurements:
        total_weight = result.measurements.get('total_weight', 0)
        weight_dist = result.measurements.get('weight_distribution', {})
        
        logger.info(f"测量总重量: {total_weight:.1f}N")
        logger.info("重量分布:")
        for foot, percentage in weight_dist.items():
            logger.info(f"  {foot}: {percentage:.1f}%")
    
    if result.recommendations:
        logger.info("建议:")
        for rec in result.recommendations:
            logger.info(f"  • {rec}")
    
    return result

def run_zero_drift_analysis(tester: StaticForceTester, config: Dict[str, Any]):
    """运行零点漂移分析"""
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("📊 阶段B.3: 零点漂移分析")
    logger.info("=" * 60)
    
    # 获取测试参数
    static_config = config.get('static_validation', {})
    drift_duration = static_config.get('zero_drift_duration', 300.0)
    
    logger.info(f"开始零点漂移分析，持续时间: {drift_duration}秒")
    logger.info("⚠️ 请保持机器人悬空状态，分析传感器零点稳定性！")
    
    # 倒计时
    for i in range(5, 0, -1):
        logger.info(f"分析将在 {i} 秒后开始...")
        time.sleep(1.0)
    
    # 执行零点漂移分析
    drift_result = tester.run_zero_drift_analysis(drift_duration)
    
    # 显示结果
    logger.info(f"零点漂移分析完成")
    logger.info(f"最大漂移: {drift_result.max_drift:.3f}N")
    logger.info(f"漂移稳定性评分: {drift_result.drift_stability:.1f}/100")
    
    if drift_result.drift_values:
        logger.info("各足端漂移情况:")
        for foot, drift in drift_result.drift_values.items():
            drift_magnitude = (drift[0]**2 + drift[1]**2 + drift[2]**2)**0.5
            logger.info(f"  {foot}: {drift_magnitude:.3f}N")
    
    return drift_result

def run_comprehensive_analysis(analyzer: FootForceAnalyzer, data_collector: FootForceDataCollector, 
                             output_dir: Path) -> Dict[str, Any]:
    """运行综合数据分析"""
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("📈 阶段B.4: 综合数据分析")
    logger.info("=" * 60)
    
    # 获取收集的数据
    collected_data = data_collector.get_data()
    
    if len(collected_data) < 10:
        logger.error("数据不足，无法进行综合分析")
        return {}
    
    logger.info(f"分析数据点数: {len(collected_data)}")
    
    # 执行综合分析
    analysis_report = analyzer.generate_comprehensive_report(collected_data)
    
    # 保存分析报告
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    report_file = output_dir / f'comprehensive_analysis_{timestamp}.json'
    
    if analyzer.save_analysis_report(analysis_report, str(report_file)):
        logger.info(f"综合分析报告已保存: {report_file}")
    
    # 导出CSV数据
    csv_file = output_dir / f'static_validation_data_{timestamp}.csv'
    if analyzer.export_to_csv(collected_data, str(csv_file)):
        logger.info(f"原始数据已导出: {csv_file}")
    
    # 显示关键结果
    if 'overall_assessment' in analysis_report:
        assessment = analysis_report['overall_assessment']
        logger.info("综合评估结果:")
        logger.info(f"  数据质量评分: {assessment.get('data_quality_score', 0):.1f}/100")
        logger.info(f"  传感器一致性: {assessment.get('sensor_consistency_score', 0):.1f}/100")
        logger.info(f"  稳定性评分: {assessment.get('stability_score', 0):.1f}/100")
        logger.info(f"  总体评分: {assessment.get('overall_score', 0):.1f}/100")
        
        if assessment.get('key_findings'):
            logger.info("关键发现:")
            for finding in assessment['key_findings']:
                logger.info(f"  • {finding}")
    
    return analysis_report

def generate_visualizations(visualizer: FootForceVisualizer, data_collector: FootForceDataCollector,
                          test_results: List[StaticTestResult], output_dir: Path) -> bool:
    """生成可视化图表"""
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("📊 阶段B.5: 生成可视化图表")
    logger.info("=" * 60)
    
    try:
        # 获取数据
        collected_data = data_collector.get_data()
        
        if len(collected_data) < 10:
            logger.error("数据不足，无法生成可视化图表")
            return False
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 1. 静态分析图表
        static_plot_file = output_dir / f'static_analysis_{timestamp}.png'
        if visualizer.plot_static_analysis(collected_data, str(static_plot_file)):
            logger.info(f"✅ 静态分析图表已生成: {static_plot_file}")
        
        # 2. 3D力分布图
        force_3d_file = output_dir / f'force_distribution_3d_{timestamp}.png'
        if visualizer.plot_force_distribution_3d(collected_data, str(force_3d_file)):
            logger.info(f"✅ 3D力分布图已生成: {force_3d_file}")
        
        # 3. 测试结果汇总仪表板
        dashboard_file = output_dir / f'validation_dashboard_{timestamp}.png'
        if visualizer.create_summary_dashboard(test_results, str(dashboard_file)):
            logger.info(f"✅ 验证汇总仪表板已生成: {dashboard_file}")
        
        return True
        
    except Exception as e:
        logger.error(f"生成可视化图表失败: {e}")
        return False

def generate_final_report(test_results: List[StaticTestResult], analysis_report: Dict[str, Any],
                         output_dir: Path) -> bool:
    """生成最终验证报告"""
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("📋 生成最终验证报告")
    logger.info("=" * 60)
    
    try:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 整合所有结果
        final_report = {
            'validation_timestamp': datetime.now().isoformat(),
            'validation_type': 'static_validation',
            'test_summary': {
                'total_tests': len(test_results),
                'passed_tests': sum(1 for r in test_results if r.status == 'PASS'),
                'warning_tests': sum(1 for r in test_results if r.status == 'WARNING'),
                'failed_tests': sum(1 for r in test_results if r.status == 'FAIL'),
                'average_score': sum(r.score for r in test_results) / len(test_results) if test_results else 0
            },
            'test_results': [r.__dict__ for r in test_results],
            'comprehensive_analysis': analysis_report,
            'final_assessment': {}
        }
        
        # 生成最终评估
        avg_score = final_report['test_summary']['average_score']
        pass_rate = final_report['test_summary']['passed_tests'] / len(test_results) * 100 if test_results else 0
        
        if avg_score >= 85 and pass_rate >= 80:
            final_status = "PASS"
            status_desc = "传感器静态验证通过，可以投入正式使用"
        elif avg_score >= 70 and pass_rate >= 60:
            final_status = "WARNING"
            status_desc = "传感器静态验证基本通过，建议进一步优化"
        else:
            final_status = "FAIL"
            status_desc = "传感器静态验证未通过，需要重新校准或检修"
        
        final_report['final_assessment'] = {
            'status': final_status,
            'description': status_desc,
            'overall_score': avg_score,
            'pass_rate': pass_rate,
            'recommendations': []
        }
        
        # 收集所有建议
        all_recommendations = []
        for result in test_results:
            all_recommendations.extend(result.recommendations)
        
        # 去重并添加到最终报告
        unique_recommendations = list(set(all_recommendations))
        final_report['final_assessment']['recommendations'] = unique_recommendations
        
        # 保存最终报告
        report_file = output_dir / f'static_validation_final_report_{timestamp}.json'
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(final_report, f, indent=2, ensure_ascii=False)
        
        logger.info(f"✅ 最终验证报告已保存: {report_file}")
        
        # 显示最终结果
        logger.info("=" * 60)
        logger.info("🎯 静态验证最终结果")
        logger.info("=" * 60)
        logger.info(f"状态: {final_status}")
        logger.info(f"描述: {status_desc}")
        logger.info(f"总体评分: {avg_score:.1f}/100")
        logger.info(f"通过率: {pass_rate:.1f}%")
        logger.info(f"测试项目: {len(test_results)}个")
        
        if unique_recommendations:
            logger.info("主要建议:")
            for rec in unique_recommendations[:5]:  # 显示前5个建议
                logger.info(f"  • {rec}")
        
        return True
        
    except Exception as e:
        logger.error(f"生成最终验证报告失败: {e}")
        return False

def main():
    """主函数"""
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Unitree Go2 足端力传感器静态验证')
    parser.add_argument('--config', default='validation_config.json', help='配置文件路径')
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='日志级别')
    parser.add_argument('--log-file', help='日志文件路径')
    parser.add_argument('--skip-visualization', action='store_true', help='跳过可视化生成')
    parser.add_argument('--test-mode', action='store_true', help='测试模式（较短的测试时间）')
    
    args = parser.parse_args()
    
    # 设置日志
    setup_logging(args.log_level, args.log_file)
    logger = logging.getLogger(__name__)
    
    logger.info("=" * 80)
    logger.info("🦾 Unitree Go2 足端力传感器静态验证系统")
    logger.info("=" * 80)
    logger.info(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    try:
        # 1. 验证环境
        if not validate_environment():
            logger.error("环境验证失败，退出程序")
            return 1
        
        # 2. 加载配置
        config = load_configuration(args.config)
        
        # 测试模式调整
        if args.test_mode:
            logger.info("⚠️ 测试模式：使用较短的测试时间")
            config['static_validation']['zero_load_test_duration'] = 10.0
            config['static_validation']['static_standing_duration'] = 20.0
            config['static_validation']['zero_drift_duration'] = 60.0
        
        # 3. 初始化组件
        logger.info("🔧 初始化系统组件...")
        
        # 足端力配置
        foot_force_config = FootForceConfig(config)
        logger.info("✅ 足端力配置初始化完成")
        
        # 数据收集器
        data_collector = FootForceDataCollector(config, foot_force_config)
        logger.info("✅ 数据收集器初始化完成")
        
        # 静态测试器
        static_tester = StaticForceTester(config, foot_force_config)
        logger.info("✅ 静态测试器初始化完成")
        
        # 可视化器
        visualizer = FootForceVisualizer(config)
        logger.info("✅ 可视化器初始化完成")
        
        # 数据分析器
        analyzer = FootForceAnalyzer(config)
        logger.info("✅ 数据分析器初始化完成")
        
        # 输出目录
        output_dir = Path(__file__).parent / 'output'
        output_dir.mkdir(exist_ok=True)
        
        logger.info("🚀 所有组件初始化完成，开始静态验证...")
        
        # 4. 执行验证测试
        test_results = []
        
        # 4.1 零负载验证
        zero_load_result = run_zero_load_validation(static_tester, config)
        test_results.append(zero_load_result)
        
        # 4.2 静态站立验证
        standing_result = run_static_standing_validation(static_tester, config)
        test_results.append(standing_result)
        
        # 4.3 零点漂移分析
        drift_result = run_zero_drift_analysis(static_tester, config)
        
        # 4.4 综合数据分析
        analysis_report = run_comprehensive_analysis(analyzer, data_collector, output_dir)
        
        # 5. 生成可视化图表
        if not args.skip_visualization:
            generate_visualizations(visualizer, data_collector, test_results, output_dir)
        else:
            logger.info("⏭️ 跳过可视化生成")
        
        # 6. 生成最终报告
        generate_final_report(test_results, analysis_report, output_dir)
        
        logger.info("=" * 80)
        logger.info("🎉 静态验证完成！")
        logger.info(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info(f"结果文件保存在: {output_dir}")
        logger.info("=" * 80)
        
        return 0
        
    except KeyboardInterrupt:
        logger.warning("⚠️ 用户中断操作")
        return 130
        
    except Exception as e:
        logger.error(f"❌ 静态验证执行失败: {e}")
        return 1

if __name__ == '__main__':
    exit(main()) 