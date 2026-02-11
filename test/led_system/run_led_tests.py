 #!/usr/bin/env python3
"""
LED控制系统测试框架主运行器
任务6.5: 全面测试、验证和性能优化

提供统一的测试执行、报告生成和结果分析
"""

import os
import sys
import time
import argparse
import unittest
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any, Optional

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from .test_config import get_led_test_config, reset_led_test_config
from .data_collector import get_led_test_collector, reset_led_test_collector
from .test_led_modes import LEDModesFunctionalTest, LEDModesStressTest
from .test_performance import LEDPerformanceBenchmark, LEDPerformanceRegression


class LEDTestRunner:
    """LED测试框架主运行器"""
    
    def __init__(self, config_overrides: Optional[Dict[str, Any]] = None):
        """初始化测试运行器"""
        self.config = get_led_test_config()
        self.collector = get_led_test_collector()
        
        # 应用配置覆盖
        if config_overrides:
            self._apply_config_overrides(config_overrides)
        
        self.test_suites = {
            'functional': self._create_functional_suite,
            'performance': self._create_performance_suite,
            'stress': self._create_stress_suite,
            'regression': self._create_regression_suite,
            'all': self._create_all_suites
        }
        
        print(f"🤖 LED控制系统测试框架 v1.0")
        print(f"📅 初始化时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⚙️ 测试模式: {self.config.get_test_mode()}")
    
    def _apply_config_overrides(self, overrides: Dict[str, Any]):
        """应用配置覆盖"""
        for key, value in overrides.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
                print(f"🔧 配置覆盖: {key} = {value}")
    
    def run_tests(self, test_type: str = "all", verbosity: int = 2, 
                 output_dir: Optional[str] = None) -> Dict[str, Any]:
        """运行指定类型的测试"""
        
        print(f"\n{'='*80}")
        print(f"🚀 开始执行 {test_type.upper()} 测试")
        print(f"{'='*80}")
        
        # 创建测试套件
        if test_type not in self.test_suites:
            raise ValueError(f"不支持的测试类型: {test_type}. 支持的类型: {list(self.test_suites.keys())}")
        
        suite_creator = self.test_suites[test_type]
        
        if test_type == 'all':
            # 运行所有测试套件
            results = {}
            for suite_name in ['functional', 'performance', 'stress', 'regression']:
                if suite_name == 'stress' and not self.config.is_stress_test_enabled():
                    print(f"⏭️ 跳过 {suite_name} 测试（未启用压力测试）")
                    continue
                
                print(f"\n📋 运行 {suite_name.upper()} 测试套件...")
                suite = self.test_suites[suite_name]()
                result = self._run_test_suite(suite, verbosity)
                results[suite_name] = result
            
            # 合并所有结果
            combined_result = self._combine_results(results)
            
        else:
            # 运行单个测试套件
            if test_type == 'stress' and not self.config.is_stress_test_enabled():
                print("⚠️ 压力测试已禁用，请检查配置")
                return {'error': 'stress_tests_disabled'}
            
            suite = suite_creator()
            combined_result = self._run_test_suite(suite, verbosity)
        
        # 生成报告
        self._generate_test_reports(output_dir)
        
        # 显示测试总结
        self._display_test_summary(combined_result)
        
        return combined_result
    
    def _create_functional_suite(self) -> unittest.TestSuite:
        """创建功能测试套件"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDModesFunctionalTest))
        return suite
    
    def _create_performance_suite(self) -> unittest.TestSuite:
        """创建性能测试套件"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDPerformanceBenchmark))
        return suite
    
    def _create_stress_suite(self) -> unittest.TestSuite:
        """创建压力测试套件"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDModesStressTest))
        return suite
    
    def _create_regression_suite(self) -> unittest.TestSuite:
        """创建回归测试套件"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDPerformanceRegression))
        return suite
    
    def _create_all_suites(self) -> unittest.TestSuite:
        """创建所有测试套件"""
        # 这个方法不会被直接调用，因为 'all' 类型在 run_tests 中特殊处理
        return unittest.TestSuite()
    
    def _run_test_suite(self, suite: unittest.TestSuite, verbosity: int) -> Dict[str, Any]:
        """运行测试套件并返回结果"""
        start_time = time.time()
        
        # 创建测试运行器
        runner = unittest.TextTestRunner(
            verbosity=verbosity,
            stream=sys.stdout,
            buffer=False
        )
        
        # 运行测试
        result = runner.run(suite)
        
        end_time = time.time()
        duration = end_time - start_time
        
        # 统计结果
        test_results = {
            'tests_run': result.testsRun,
            'failures': len(result.failures),
            'errors': len(result.errors),
            'skipped': len(result.skipped) if hasattr(result, 'skipped') else 0,
            'success_count': result.testsRun - len(result.failures) - len(result.errors),
            'success_rate': ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100) if result.testsRun > 0 else 0,
            'duration': duration,
            'failure_details': [{'test': str(test), 'traceback': traceback} for test, traceback in result.failures],
            'error_details': [{'test': str(test), 'traceback': traceback} for test, traceback in result.errors]
        }
        
        return test_results
    
    def _combine_results(self, results: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
        """合并多个测试结果"""
        combined = {
            'tests_run': 0,
            'failures': 0,
            'errors': 0,
            'skipped': 0,
            'success_count': 0,
            'total_duration': 0,
            'suite_results': results,
            'failure_details': [],
            'error_details': []
        }
        
        for suite_name, result in results.items():
            if 'error' in result:
                continue
                
            combined['tests_run'] += result['tests_run']
            combined['failures'] += result['failures']
            combined['errors'] += result['errors']
            combined['skipped'] += result['skipped']
            combined['success_count'] += result['success_count']
            combined['total_duration'] += result['duration']
            combined['failure_details'].extend(result['failure_details'])
            combined['error_details'].extend(result['error_details'])
        
        combined['success_rate'] = (combined['success_count'] / combined['tests_run'] * 100) if combined['tests_run'] > 0 else 0
        
        return combined
    
    def _generate_test_reports(self, output_dir: Optional[str] = None):
        """生成测试报告"""
        if output_dir:
            self.collector.output_dir = Path(output_dir)
        
        # 生成HTML报告
        html_report = self.collector.generate_report()
        
        # 保存JSON数据
        json_data = self.collector.save_data(format_type="json")
        
        # 保存CSV数据
        csv_data = self.collector.save_data(format_type="csv")
        
        print(f"\n📊 测试报告已生成:")
        print(f"   📋 HTML报告: {html_report}")
        print(f"   📄 JSON数据: {json_data}")
        print(f"   📈 CSV数据: {csv_data}")
    
    def _display_test_summary(self, results: Dict[str, Any]):
        """显示测试总结"""
        print(f"\n{'='*80}")
        print(f"📊 LED控制系统测试总结")
        print(f"{'='*80}")
        
        if 'error' in results:
            print(f"❌ 测试执行错误: {results['error']}")
            return
        
        # 基本统计
        print(f"🧪 总测试数: {results['tests_run']}")
        print(f"✅ 成功: {results['success_count']}")
        print(f"❌ 失败: {results['failures']}")
        print(f"🔥 错误: {results['errors']}")
        print(f"⏭️ 跳过: {results['skipped']}")
        print(f"📈 成功率: {results['success_rate']:.1f}%")
        print(f"⏱️ 总耗时: {results['total_duration']:.2f}秒")
        
        # 状态判定
        if results['success_rate'] >= 95:
            status_icon = "🎉"
            status_text = "优秀"
        elif results['success_rate'] >= 90:
            status_icon = "✅"
            status_text = "良好"
        elif results['success_rate'] >= 80:
            status_icon = "⚠️"
            status_text = "一般"
        else:
            status_icon = "❌"
            status_text = "需要改进"
        
        print(f"\n{status_icon} 总体评估: {status_text}")
        
        # 套件详情（如果有多个套件）
        if 'suite_results' in results:
            print(f"\n📋 分套件结果:")
            for suite_name, suite_result in results['suite_results'].items():
                if 'error' in suite_result:
                    print(f"   {suite_name}: ❌ {suite_result['error']}")
                else:
                    rate = suite_result['success_rate']
                    icon = "✅" if rate >= 95 else "⚠️" if rate >= 80 else "❌"
                    print(f"   {suite_name}: {icon} {suite_result['success_count']}/{suite_result['tests_run']} ({rate:.1f}%)")
        
        # 实时统计
        real_time_stats = self.collector.get_real_time_stats()
        if real_time_stats:
            print(f"\n📈 性能指标:")
            if 'response_time' in real_time_stats:
                rt = real_time_stats['response_time']
                print(f"   响应时间: 平均 {rt.get('overall_avg', 0):.2f}ms, 最大 {rt.get('overall_max', 0):.2f}ms")
            
            if 'cpu_usage' in real_time_stats:
                cpu = real_time_stats['cpu_usage']
                print(f"   CPU使用: 平均 {cpu.get('avg', 0):.1f}%, 最大 {cpu.get('max', 0):.1f}%")
            
            if 'memory_usage' in real_time_stats:
                mem = real_time_stats['memory_usage']
                print(f"   内存使用: 平均 {mem.get('avg', 0):.1f}MB, 最大 {mem.get('max', 0):.1f}MB")
        
        # 失败详情（如果有）
        if results['failures'] > 0 or results['errors'] > 0:
            print(f"\n🔍 失败详情:")
            for failure in results['failure_details'][:3]:  # 只显示前3个
                print(f"   ❌ {failure['test']}")
            
            for error in results['error_details'][:3]:  # 只显示前3个
                print(f"   🔥 {error['test']}")
            
            if len(results['failure_details']) + len(results['error_details']) > 6:
                print(f"   ... 更多详情请查看完整报告")
        
        print(f"\n{'='*80}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="LED控制系统测试框架")
    
    parser.add_argument('--type', '-t', 
                       choices=['functional', 'performance', 'stress', 'regression', 'all'],
                       default='all',
                       help='测试类型 (默认: all)')
    
    parser.add_argument('--verbosity', '-v',
                       type=int, choices=[0, 1, 2], default=2,
                       help='详细程度 (0=静默, 1=正常, 2=详细)')
    
    parser.add_argument('--output', '-o',
                       help='输出目录 (默认: logs/led_tests)')
    
    parser.add_argument('--hardware', 
                       action='store_true',
                       help='启用硬件测试模式')
    
    parser.add_argument('--stress',
                       action='store_true', 
                       help='启用压力测试')
    
    parser.add_argument('--performance-samples',
                       type=int, default=50,
                       help='性能测试采样数量 (默认: 50)')
    
    parser.add_argument('--max-response-time',
                       type=float, default=200.0,
                       help='最大响应时间阈值(ms) (默认: 200.0)')
    
    args = parser.parse_args()
    
    # 准备配置覆盖
    config_overrides = {}
    
    if args.hardware:
        config_overrides['hardware.hardware_required'] = True
        config_overrides['hardware.mock_hardware'] = False
    
    if args.stress:
        config_overrides['stress_tests_enabled'] = True
    
    if args.performance_samples != 50:
        config_overrides['performance.performance_samples'] = args.performance_samples
    
    if args.max_response_time != 200.0:
        config_overrides['performance.max_response_time_ms'] = args.max_response_time
    
    try:
        # 重置收集器确保干净的测试环境
        reset_led_test_collector()
        
        # 创建测试运行器
        runner = LEDTestRunner(config_overrides)
        
        # 运行测试
        results = runner.run_tests(
            test_type=args.type,
            verbosity=args.verbosity,
            output_dir=args.output
        )
        
        # 根据结果设置退出码
        if 'error' in results:
            sys.exit(1)
        elif results['failures'] > 0 or results['errors'] > 0:
            sys.exit(1)
        else:
            sys.exit(0)
            
    except KeyboardInterrupt:
        print("\n⏹️ 测试被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"\n💥 测试框架错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()