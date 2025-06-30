#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/main_validation_script.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU主验证脚本 - 完整的IMU传感器验证流程

import os
import sys
import time
import json
import logging
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional

# 添加模块路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

from imu_config import IMUConfig, IMUSpec
from data_collector import IMUDataCollector
from visualizer import IMUVisualizer
from static_tester import IMUStaticTester
from dynamic_tester import IMUDynamicTester
from calibration_analyzer import IMUCalibrationAnalyzer

class IMUValidationSuite:
    """IMU验证测试套件"""
    
    def __init__(self, config_file: str = None):
        """
        初始化验证套件
        
        Args:
            config_file: 配置文件路径
        """
        self.logger = self._setup_logging()
        self.config = self._load_config(config_file)
        
        # 初始化组件
        self.imu_config = None
        self.data_collector = None
        self.visualizer = None
        self.static_tester = None
        self.dynamic_tester = None
        self.calibration_analyzer = None
        
        # 测试结果
        self.test_results = {
            'test_info': {
                'start_time': datetime.now().isoformat(),
                'test_version': '1.0.0',
                'robot_model': 'Unitree Go2',
                'test_operator': os.getenv('USER', 'unknown')
            },
            'initialization': {},
            'static_test': {},
            'dynamic_test': {},
            'calibration_analysis': {},
            'visualization_test': {},
            'overall_assessment': {}
        }
        
    def _setup_logging(self) -> logging.Logger:
        """设置日志记录"""
        logger = logging.getLogger('IMUValidation')
        logger.setLevel(logging.INFO)
        
        # 创建日志目录
        log_dir = Path('logs/imu_validation')
        log_dir.mkdir(parents=True, exist_ok=True)
        
        # 文件处理器
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = log_dir / f'imu_validation_{timestamp}.log'
        
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)
        
        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # 格式化器
        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def _load_config(self, config_file: str = None) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            if config_file and Path(config_file).exists():
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.logger.info(f"已加载配置文件: {config_file}")
            else:
                # 使用默认配置文件
                default_config = current_dir / 'validation_config.json'
                if default_config.exists():
                    with open(default_config, 'r', encoding='utf-8') as f:
                        config = json.load(f)
                        self.logger.info(f"已加载默认配置: {default_config}")
                else:
                    self.logger.warning("未找到配置文件，使用内置默认配置")
                    config = self._get_default_config()
            
            return config
            
        except Exception as e:
            self.logger.error(f"配置文件加载失败: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            "imu_config": {
                "sampling_rate_hz": 100,
                "test_duration_seconds": 30,
                "timeout_seconds": 10,
                "network_interface": "eth0"
            },
            "test_parameters": {
                "static_test": {
                    "duration_seconds": 60,
                    "stability_threshold": {
                        "accelerometer_std_max": 0.05,
                        "gyroscope_std_max": 0.1,
                        "quaternion_drift_max": 0.01
                    }
                },
                "dynamic_test": {
                    "duration_seconds": 120,
                    "response_tests": ["pitch_test", "roll_test", "yaw_test"],
                    "response_threshold_ms": 50
                }
            },
            "visualization": {
                "real_time_plots": True,
                "plot_update_interval_ms": 100,
                "max_plot_points": 500,
                "enable_3d_orientation": True
            },
            "quality_thresholds": {
                "accuracy": {
                    "gravity_error_max_percent": 2.0
                },
                "noise_levels": {
                    "accelerometer_noise_max": 0.02,
                    "gyroscope_noise_max": 0.01
                }
            }
        }
    
    def run_full_validation(self) -> Dict[str, Any]:
        """运行完整的IMU验证流程"""
        self.logger.info("🚀 " + "=" * 60)
        self.logger.info("🚀 开始Unitree Go2 IMU完整验证流程")
        self.logger.info("🚀 " + "=" * 60)
        
        try:
            # 1. 初始化IMU系统
            self.logger.info("\n🔧 阶段1: 初始化IMU系统")
            if not self._initialize_imu_system():
                self.test_results['overall_assessment']['status'] = 'INITIALIZATION_FAILED'
                return self.test_results
            
            # 2. 静态稳定性测试
            self.logger.info("\n📊 阶段2: 静态稳定性测试")
            static_results = self._run_static_stability_test()
            self.test_results['static_test'] = static_results
            
            # 3. 动态响应测试
            self.logger.info("\n🎯 阶段3: 动态响应测试")
            dynamic_results = self._run_dynamic_validation()
            self.test_results['dynamic_test'] = dynamic_results
            
            # 4. 校准验证分析
            self.logger.info("\n⚙️  阶段4: 校准验证分析")
            calibration_results = self._run_calibration_validation()
            self.test_results['calibration_analysis'] = calibration_results
            
            # 5. 可视化验证
            self.logger.info("\n📈 阶段5: 可视化功能验证")
            visualization_results = self._run_visualization_validation()
            self.test_results['visualization_test'] = visualization_results
            
            # 6. 生成综合评估
            self.logger.info("\n📋 阶段6: 生成综合评估报告")
            overall_assessment = self._generate_overall_assessment()
            self.test_results['overall_assessment'] = overall_assessment
            
            # 7. 保存测试结果
            self._save_test_results()
            
            self.logger.info("\n✅ " + "=" * 60)
            self.logger.info("✅ IMU验证流程完成")
            self.logger.info(f"✅ 总体状态: {overall_assessment.get('status', 'UNKNOWN')}")
            self.logger.info("✅ " + "=" * 60)
            
            return self.test_results
            
        except Exception as e:
            self.logger.error(f"验证流程失败: {e}")
            self.test_results['overall_assessment']['status'] = 'VALIDATION_ERROR'
            self.test_results['overall_assessment']['error'] = str(e)
            return self.test_results
    
    def _initialize_imu_system(self) -> bool:
        """初始化IMU系统"""
        try:
            self.logger.info("初始化IMU配置...")
            
            # 创建IMU配置
            self.imu_config = IMUConfig(self.config)
            
            # 初始化IMU连接
            success = self.imu_config.initialize_imu()
            
            if not success:
                self.logger.error("IMU初始化失败")
                self.test_results['initialization']['status'] = 'FAILED'
                return False
            
            # 创建数据采集器
            self.data_collector = IMUDataCollector(self.config, self.imu_config)
            
            # 创建测试器
            self.logger.info("创建测试组件...")
            
            # 导入测试器类
            from static_tester import IMUStaticTester
            from dynamic_tester import IMUDynamicTester
            from calibration_analyzer import IMUCalibrationAnalyzer
            from visualizer import IMUVisualizer
            
            # 创建测试器实例
            self.static_tester = IMUStaticTester(self.imu_config, self.data_collector, self.config)
            self.dynamic_tester = IMUDynamicTester(self.imu_config, self.data_collector, self.config)
            self.calibration_analyzer = IMUCalibrationAnalyzer(self.imu_config, self.data_collector, self.config)
            self.visualizer = IMUVisualizer(self.imu_config, self.data_collector, self.config)
            
            # 验证初始化状态
            test_reading = self.imu_config.get_latest_reading()
            
            if test_reading:
                self.logger.info("✅ IMU系统初始化成功")
                self.logger.info(f"📊 当前加速度: {test_reading.accelerometer}")
                self.logger.info(f"🌀 当前陀螺仪: {test_reading.gyroscope}")
                self.logger.info(f"🧭 当前姿态: {test_reading.quaternion}")
                
                self.test_results['initialization']['status'] = 'SUCCESS'
                self.test_results['initialization']['imu_specs'] = {
                    'sampling_rate_hz': self.imu_config.target_spec.sampling_rate_hz,
                    'accelerometer_range': self.imu_config.target_spec.accelerometer_range,
                    'gyroscope_range': self.imu_config.target_spec.gyroscope_range
                }
                
                return True
            else:
                self.logger.error("IMU数据读取测试失败")
                self.test_results['initialization']['status'] = 'NO_DATA'
                return False
                
        except Exception as e:
            self.logger.error(f"IMU系统初始化异常: {e}")
            self.test_results['initialization']['status'] = 'ERROR'
            self.test_results['initialization']['error'] = str(e)
            return False
    
    def _run_static_stability_test(self) -> Dict:
        """运行静态稳定性测试"""
        self.logger.info("开始静态稳定性测试...")
        
        try:
            # 检查静态测试器是否可用
            if not self.static_tester:
                self.logger.error("静态测试器未初始化")
                return {'status': 'FAIL', 'error': '静态测试器未初始化'}
            
            # 运行静态稳定性测试 - 使用正确的方法名
            static_results = self.static_tester.run_static_stability_test()
            
            # 转换StaticTestResults为字典格式
            result_dict = {
                'status': static_results.test_status,
                'test_duration': static_results.test_duration,
                'sample_count': static_results.sample_count,
                'valid_samples': static_results.valid_samples,
                'accelerometer_stability': static_results.accelerometer_stability,
                'gyroscope_stability': static_results.gyroscope_stability,
                'quaternion_stability': static_results.quaternion_stability,
                'gravity_accuracy': static_results.gravity_accuracy,
                'bias_analysis': static_results.bias_analysis,
                'noise_analysis': static_results.noise_analysis,
                'temperature_analysis': static_results.temperature_analysis,
                'pass_criteria': static_results.pass_criteria,
                'recommendations': static_results.recommendations
            }
            
            self.logger.info(f"静态测试完成，状态: {static_results.test_status}")
            return result_dict
                
        except Exception as e:
            self.logger.error(f"静态测试失败: {e}")
            return {'status': 'ERROR', 'error': str(e)}
    
    def _run_dynamic_validation(self) -> Dict[str, Any]:
        """运行动态验证测试"""
        try:
            self.logger.info("开始动态响应测试...")
            
            # 运行动态测试
            dynamic_results = self.dynamic_tester.run_dynamic_response_test("comprehensive")
            
            # 转换结果为字典格式
            result_dict = {}
            
            for test_name, test_result in dynamic_results.items():
                result_dict[test_name] = {
                    'test_duration': test_result.test_duration,
                    'sample_count': test_result.sample_count,
                    'response_time_ms': test_result.response_time_ms,
                    'rise_time_ms': test_result.rise_time_ms,
                    'settling_time_ms': test_result.settling_time_ms,
                    'overshoot_percent': test_result.overshoot_percent,
                    'tracking_accuracy': test_result.tracking_accuracy,
                    'dynamic_range': test_result.dynamic_range,
                    'frequency_response': test_result.frequency_response,
                    'test_status': test_result.test_status,
                    'pass_criteria': test_result.pass_criteria,
                    'recommendations': test_result.recommendations
                }
            
            overall_status = dynamic_results.get('overall', type('', (), {'test_status': 'UNKNOWN'})).test_status
            self.logger.info(f"动态测试完成，整体状态: {overall_status}")
            
            return result_dict
            
        except Exception as e:
            self.logger.error(f"动态验证测试失败: {e}")
            return {
                'overall': {
                    'test_status': 'ERROR',
                    'error': str(e),
                    'recommendations': ['动态测试执行失败，检查系统状态']
                }
            }
    
    def _run_calibration_validation(self) -> Dict[str, Any]:
        """运行校准验证分析"""
        try:
            self.logger.info("开始校准验证分析...")
            
            # 运行校准分析
            calibration_results = self.calibration_analyzer.run_comprehensive_calibration_analysis()
            
            # 转换结果为字典格式
            result_dict = {
                'calibration_type': calibration_results.calibration_type,
                'sample_count': calibration_results.sample_count,
                'test_duration': calibration_results.test_duration,
                'accelerometer_calibration': calibration_results.accelerometer_calibration,
                'gravity_calibration': calibration_results.gravity_calibration,
                'accel_bias': calibration_results.accel_bias,
                'accel_scale_factor': calibration_results.accel_scale_factor,
                'gyroscope_calibration': calibration_results.gyroscope_calibration,
                'gyro_bias': calibration_results.gyro_bias,
                'gyro_noise_characteristics': calibration_results.gyro_noise_characteristics,
                'attitude_calibration': calibration_results.attitude_calibration,
                'quaternion_consistency': calibration_results.quaternion_consistency,
                'euler_accuracy': calibration_results.euler_accuracy,
                'temperature_compensation': calibration_results.temperature_compensation,
                'calibration_quality': calibration_results.calibration_quality,
                'test_status': calibration_results.test_status,
                'pass_criteria': calibration_results.pass_criteria,
                'recommendations': calibration_results.recommendations
            }
            
            self.logger.info(f"校准分析完成，状态: {calibration_results.test_status}")
            
            return result_dict
            
        except Exception as e:
            self.logger.error(f"校准验证分析失败: {e}")
            return {
                'test_status': 'ERROR',
                'error': str(e),
                'recommendations': ['校准分析执行失败，检查系统状态']
            }
    
    def _run_visualization_validation(self) -> Dict[str, Any]:
        """运行可视化验证"""
        try:
            self.logger.info("开始可视化功能验证...")
            
            # 启动可视化
            viz_success = self.visualizer.start_visualization("all")
            
            if not viz_success:
                return {
                    'test_status': 'FAIL',
                    'error': '可视化启动失败',
                    'recommendations': ['检查matplotlib依赖和显示环境']
                }
            
            # 短期数据采集用于可视化测试
            self.logger.info("收集可视化测试数据...")
            collect_success = self.data_collector.start_collection(20.0)  # 20秒
            
            if collect_success:
                # 等待采集完成
                time.sleep(25)
                
                # 停止采集
                self.data_collector.stop_collection()
                
                # 获取可视化统计
                viz_stats = self.visualizer.get_plot_statistics()
                
                # 保存可视化图像
                output_dir = f"output/imu_validation/{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                self.visualizer.save_current_plots(output_dir)
                
                # 停止可视化
                self.visualizer.stop_visualization()
                
                result_dict = {
                    'test_status': 'PASS',
                    'visualization_started': True,
                    'data_points_plotted': viz_stats.get('data_points', 0),
                    'plot_statistics': viz_stats,
                    'output_directory': output_dir,
                    'recommendations': ['可视化功能正常工作']
                }
                
            else:
                result_dict = {
                    'test_status': 'FAIL',
                    'error': '可视化数据采集失败',
                    'recommendations': ['检查数据采集器状态']
                }
            
            self.logger.info(f"可视化验证完成，状态: {result_dict['test_status']}")
            
            return result_dict
            
        except Exception as e:
            self.logger.error(f"可视化验证失败: {e}")
            return {
                'test_status': 'ERROR',
                'error': str(e),
                'recommendations': ['可视化验证执行失败，检查依赖和环境']
            }
    
    def _generate_overall_assessment(self) -> Dict[str, Any]:
        """生成综合评估"""
        try:
            # 收集各阶段状态
            init_status = self.test_results['initialization'].get('status', 'UNKNOWN')
            static_status = self.test_results['static_test'].get('status', 'UNKNOWN')
            dynamic_status = self.test_results['dynamic_test'].get('overall', {}).get('test_status', 'UNKNOWN')
            calib_status = self.test_results['calibration_analysis'].get('test_status', 'UNKNOWN')
            viz_status = self.test_results['visualization_test'].get('test_status', 'UNKNOWN')
            
            all_statuses = [init_status, static_status, dynamic_status, calib_status, viz_status]
            
            # 确定整体状态
            if init_status != 'SUCCESS':
                overall_status = 'INITIALIZATION_FAILED'
            elif 'ERROR' in all_statuses:
                overall_status = 'ERROR'
            elif 'FAIL' in all_statuses:
                overall_status = 'FAIL'
            elif 'WARNING' in all_statuses:
                overall_status = 'WARNING'
            elif all(status in ['PASS', 'SUCCESS'] for status in all_statuses):
                overall_status = 'PASS'
            else:
                overall_status = 'PARTIAL'
            
            # 收集所有建议
            all_recommendations = []
            
            for test_key in ['static_test', 'calibration_analysis']:
                recommendations = self.test_results.get(test_key, {}).get('recommendations', [])
                all_recommendations.extend(recommendations)
            
            # 动态测试建议
            dynamic_tests = self.test_results.get('dynamic_test', {})
            for test_name, test_data in dynamic_tests.items():
                if isinstance(test_data, dict) and 'recommendations' in test_data:
                    all_recommendations.extend(test_data['recommendations'])
            
            # 可视化建议
            viz_recommendations = self.test_results.get('visualization_test', {}).get('recommendations', [])
            all_recommendations.extend(viz_recommendations)
            
            # 去重
            unique_recommendations = list(set(all_recommendations))
            
            # 生成总结
            test_summary = {
                'initialization': init_status,
                'static_stability': static_status,
                'dynamic_response': dynamic_status,
                'calibration_quality': calib_status,
                'visualization': viz_status
            }
            
            # 计算通过率
            passed_tests = sum(1 for status in all_statuses if status in ['PASS', 'SUCCESS'])
            pass_rate = (passed_tests / len(all_statuses)) * 100
            
            assessment = {
                'status': overall_status,
                'test_summary': test_summary,
                'pass_rate_percent': pass_rate,
                'total_recommendations': len(unique_recommendations),
                'critical_issues': [rec for rec in unique_recommendations if any(word in rec.lower() for word in ['失败', '错误', '无法', '不足', '过高', '过低'])],
                'recommendations': unique_recommendations,
                'test_completion_time': datetime.now().isoformat(),
                'overall_conclusion': self._generate_conclusion(overall_status, pass_rate)
            }
            
            return assessment
            
        except Exception as e:
            self.logger.error(f"综合评估生成失败: {e}")
            return {
                'status': 'ASSESSMENT_ERROR',
                'error': str(e),
                'test_completion_time': datetime.now().isoformat()
            }
    
    def _generate_conclusion(self, status: str, pass_rate: float) -> str:
        """生成结论文本"""
        if status == 'PASS':
            return f"IMU验证完全通过，通过率{pass_rate:.1f}%。所有传感器指标符合要求，可用于生产环境。"
        elif status == 'WARNING':
            return f"IMU验证基本通过，通过率{pass_rate:.1f}%。存在次要问题，建议关注相关建议。"
        elif status == 'FAIL':
            return f"IMU验证失败，通过率{pass_rate:.1f}%。存在严重问题，需要修复后重新测试。"
        elif status == 'INITIALIZATION_FAILED':
            return "IMU初始化失败，无法进行完整验证。检查硬件连接和驱动程序。"
        else:
            return f"IMU验证部分完成，通过率{pass_rate:.1f}%。请查看详细结果和建议。"
    
    def _save_test_results(self):
        """保存测试结果"""
        try:
            # 创建输出目录
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = Path(f'output/imu_validation/{timestamp}')
            output_dir.mkdir(parents=True, exist_ok=True)
            
            # 保存完整测试结果
            results_file = output_dir / 'imu_validation_results.json'
            with open(results_file, 'w', encoding='utf-8') as f:
                json.dump(self.test_results, f, indent=2, ensure_ascii=False)
            
            # 保存简化报告
            report_file = output_dir / 'imu_validation_report.txt'
            self._generate_text_report(report_file)
            
            self.logger.info(f"📄 测试结果已保存到: {output_dir}")
            self.logger.info(f"📊 详细结果: {results_file}")
            self.logger.info(f"📋 简化报告: {report_file}")
            
        except Exception as e:
            self.logger.error(f"保存测试结果失败: {e}")
    
    def _generate_text_report(self, report_file: Path):
        """生成文本报告"""
        try:
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write("=" * 80 + "\n")
                f.write("Unitree Go2 IMU验证报告\n")
                f.write("=" * 80 + "\n\n")
                
                # 基本信息
                test_info = self.test_results['test_info']
                f.write("🔧 测试信息:\n")
                f.write(f"  测试时间: {test_info['start_time']}\n")
                f.write(f"  机器人型号: {test_info['robot_model']}\n")
                f.write(f"  测试版本: {test_info['test_version']}\n")
                f.write(f"  操作员: {test_info['test_operator']}\n\n")
                
                # 总体结果
                overall = self.test_results['overall_assessment']
                f.write("📊 总体评估:\n")
                f.write(f"  状态: {overall.get('status', 'UNKNOWN')}\n")
                f.write(f"  通过率: {overall.get('pass_rate_percent', 0):.1f}%\n")
                f.write(f"  结论: {overall.get('overall_conclusion', '无')}\n\n")
                
                # 各阶段结果
                f.write("📋 详细结果:\n")
                test_summary = overall.get('test_summary', {})
                for test_name, status in test_summary.items():
                    f.write(f"  {test_name}: {status}\n")
                f.write("\n")
                
                # 建议
                recommendations = overall.get('recommendations', [])
                if recommendations:
                    f.write("💡 建议:\n")
                    for i, rec in enumerate(recommendations, 1):
                        f.write(f"  {i}. {rec}\n")
                
                f.write("\n" + "=" * 80 + "\n")
                
        except Exception as e:
            self.logger.error(f"生成文本报告失败: {e}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Unitree Go2 IMU验证工具')
    parser.add_argument('--config', '-c', help='配置文件路径')
    parser.add_argument('--test-type', '-t', 
                       choices=['full', 'static', 'dynamic', 'calibration', 'visualization'],
                       default='full', help='测试类型')
    parser.add_argument('--output', '-o', help='输出目录')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')
    
    args = parser.parse_args()
    
    try:
        # 创建验证套件
        suite = IMUValidationSuite(args.config)
        
        if args.verbose:
            suite.logger.setLevel(logging.DEBUG)
        
        # 运行验证
        if args.test_type == 'full':
            results = suite.run_full_validation()
        else:
            suite.logger.info(f"运行单项测试: {args.test_type}")
            # 这里可以添加单项测试的逻辑
            results = suite.run_full_validation()  # 暂时还是运行完整验证
        
        # 输出结果摘要
        overall_status = results.get('overall_assessment', {}).get('status', 'UNKNOWN')
        pass_rate = results.get('overall_assessment', {}).get('pass_rate_percent', 0)
        
        print("\n" + "=" * 60)
        print("🏁 IMU验证完成")
        print(f"📊 状态: {overall_status}")
        print(f"✅ 通过率: {pass_rate:.1f}%")
        print("=" * 60)
        
        # 返回适当的退出码
        if overall_status in ['PASS', 'WARNING']:
            sys.exit(0)
        else:
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
        sys.exit(2)
    except Exception as e:
        print(f"\n❌ 验证过程失败: {e}")
        sys.exit(3)

if __name__ == "__main__":
    main() 