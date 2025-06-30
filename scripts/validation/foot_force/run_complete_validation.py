#!/usr/bin/env python3
# scripts/validation/foot_force/run_complete_validation.py
# Generated: 2025-06-26 19:00:00
# Purpose: Unitree Go2 足端力传感器完整ABCD验证流程

import os
import sys
import time
import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

# 添加项目路径
project_root = Path(__file__).parent.parent.parent.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "scripts" / "validation" / "foot_force" / "foot_force_validation"))

# 设置环境变量
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

try:
    from foot_force_validation.foot_force_config import FootForceConfig
    from foot_force_validation.basic_test import main as run_basic_test
    from foot_force_validation.static_validation import main as run_static_validation
    from foot_force_validation.dynamic_tester import DynamicFootForceTester
    from foot_force_validation.comprehensive_dashboard import ComprehensiveFootForceDashboard
except ImportError as e:
    print(f"导入错误: {e}")
    print("请确保您在正确的目录中运行此脚本")
    sys.exit(1)

class CompleteFootForceValidation:
    """完整的足端力传感器验证流程"""
    
    def __init__(self, output_dir: str = "output"):
        """初始化完整验证流程"""
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 设置日志
        self.setup_logging()
        
        # 加载配置
        config_path = Path(__file__).parent / "foot_force_validation" / "validation_config.json"
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)
        
        # 初始化组件
        self.foot_force_config = None
        self.test_results = {
            'phase_a': None,
            'phase_b': None, 
            'phase_c': None,
            'phase_d': None
        }
        
        self.logger.info("完整足端力验证流程初始化完成")
    
    def setup_logging(self):
        """设置日志系统"""
        log_dir = self.output_dir / "logs"
        log_dir.mkdir(exist_ok=True)
        
        log_file = log_dir / f"complete_validation_{self.timestamp}.log"
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file, encoding='utf-8'),
                logging.StreamHandler()
            ]
        )
        
        self.logger = logging.getLogger(__name__)
    
    def run_complete_validation(self) -> Dict[str, Any]:
        """运行完整的ABCD验证流程"""
        print("\n" + "="*80)
        print("🤖 Unitree Go2 足端力传感器完整验证流程")
        print(f"📅 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)
        
        overall_success = True
        
        try:
            # 阶段A: 数据读取框架验证
            print("\n🔍 阶段A: 数据读取框架验证")
            phase_a_success = self.run_phase_a()
            if not phase_a_success:
                overall_success = False
                print("❌ 阶段A失败，但继续执行后续测试")
            
            # 阶段B: 静态力分布验证
            print("\n⚖️ 阶段B: 静态力分布验证")
            phase_b_success = self.run_phase_b()
            if not phase_b_success:
                overall_success = False
                print("❌ 阶段B失败，但继续执行后续测试")
            
            # 阶段C: 动态响应测试
            print("\n🏃 阶段C: 动态响应测试")
            phase_c_success = self.run_phase_c()
            if not phase_c_success:
                overall_success = False
                print("❌ 阶段C失败，但继续执行后续测试")
            
            # 阶段D: 综合可视化和文档
            print("\n📊 阶段D: 综合可视化和文档生成")
            phase_d_success = self.run_phase_d()
            if not phase_d_success:
                overall_success = False
                print("❌ 阶段D失败")
            
            # 生成最终报告
            final_report = self.generate_final_report(overall_success)
            
            print("\n" + "="*80)
            if overall_success:
                print("✅ 完整验证流程执行成功！")
            else:
                print("⚠️ 验证流程完成，但存在部分失败")
            print(f"📄 最终报告: {final_report}")
            print("="*80)
            
            return {
                'success': overall_success,
                'timestamp': self.timestamp,
                'test_results': self.test_results,
                'final_report': final_report
            }
            
        except Exception as e:
            self.logger.error(f"完整验证流程失败: {e}")
            print(f"❌ 验证流程异常终止: {e}")
            return {
                'success': False,
                'error': str(e),
                'timestamp': self.timestamp
            }
    
    def run_phase_a(self) -> bool:
        """运行阶段A: 数据读取框架验证"""
        try:
            print("  📋 初始化足端力传感器配置...")
            
            # 初始化FootForceConfig
            self.foot_force_config = FootForceConfig(
                sampling_rate=self.config['foot_force_config']['sampling_rate_hz'],
                force_threshold=self.config['foot_force_config']['force_threshold'],
                max_force_per_foot=self.config['foot_force_config']['max_force_per_foot']
            )
            
            print("  🔌 建立机器人连接...")
            if not self.foot_force_config.initialize_connection():
                print("  ❌ 机器人连接失败")
                return False
            
            print("  📊 测试数据读取能力...")
            # 简短的数据读取测试
            test_start = time.time()
            test_duration = 5.0
            sample_count = 0
            
            while time.time() - test_start < test_duration:
                reading = self.foot_force_config.get_latest_reading()
                if reading:
                    sample_count += 1
                time.sleep(0.01)
            
            if sample_count > 0:
                sample_rate = sample_count / test_duration
                print(f"  ✅ 数据读取成功，采样率: {sample_rate:.1f} Hz")
                
                self.test_results['phase_a'] = {
                    'success': True,
                    'sample_rate': sample_rate,
                    'duration': test_duration,
                    'samples': sample_count
                }
                return True
            else:
                print("  ❌ 未能接收到足端力数据")
                return False
                
        except Exception as e:
            self.logger.error(f"阶段A执行失败: {e}")
            print(f"  ❌ 阶段A执行异常: {e}")
            return False
    
    def run_phase_b(self) -> bool:
        """运行阶段B: 静态力分布验证"""
        try:
            if not self.foot_force_config:
                print("  ❌ 足端力配置未初始化")
                return False
            
            print("  🧪 运行静态验证测试...")
            
            # 运行静态验证（简化版本）
            from foot_force_validation.static_tester import StaticFootForceTester
            static_tester = StaticFootForceTester(self.config, self.foot_force_config)
            
            # 运行快速静态测试
            static_results = static_tester.run_quick_static_test()
            
            if static_results and static_results.get('success', False):
                print(f"  ✅ 静态验证完成，评分: {static_results.get('final_score', 0):.1f}")
                self.test_results['phase_b'] = static_results
                return True
            else:
                print("  ❌ 静态验证失败")
                self.test_results['phase_b'] = {'success': False, 'error': '静态测试执行失败'}
                return False
                
        except Exception as e:
            self.logger.error(f"阶段B执行失败: {e}")
            print(f"  ❌ 阶段B执行异常: {e}")
            return False
    
    def run_phase_c(self) -> bool:
        """运行阶段C: 动态响应测试"""
        try:
            if not self.foot_force_config:
                print("  ❌ 足端力配置未初始化")
                return False
            
            print("  🏃 初始化动态测试器...")
            dynamic_tester = DynamicFootForceTester(self.config, self.foot_force_config)
            
            print("  🎯 运行动态测试套件...")
            print("\n  " + "-"*60)
            print("  ⚠️  请准备执行以下动态测试:")
            print("     1. 缓慢行走测试 (60秒)")
            print("     2. 正常行走测试 (45秒)")
            print("     3. 冲击测试 (30秒)")
            print("  " + "-"*60)
            
            # 询问用户是否准备好
            response = input("\n  是否准备开始动态测试？(y/N): ").strip().lower()
            if response not in ['y', 'yes', '是']:
                print("  ⏸️ 用户取消动态测试")
                self.test_results['phase_c'] = {'success': False, 'cancelled': True}
                return False
            
            # 运行动态测试套件
            dynamic_results = dynamic_tester.run_dynamic_test_suite()
            
            if dynamic_results:
                avg_score = sum(r.test_score for r in dynamic_results.values()) / len(dynamic_results)
                print(f"\n  ✅ 动态测试完成，平均评分: {avg_score:.1f}")
                
                # 保存结果
                results_file = dynamic_tester.save_dynamic_test_results(
                    dynamic_results, str(self.output_dir)
                )
                
                self.test_results['phase_c'] = {
                    'success': True,
                    'test_results': {name: {
                        'test_score': result.test_score,
                        'duration': result.duration,
                        'total_samples': result.total_samples
                    } for name, result in dynamic_results.items()},
                    'average_score': avg_score,
                    'results_file': results_file
                }
                return True
            else:
                print("  ❌ 动态测试失败")
                return False
                
        except Exception as e:
            self.logger.error(f"阶段C执行失败: {e}")
            print(f"  ❌ 阶段C执行异常: {e}")
            return False
    
    def run_phase_d(self) -> bool:
        """运行阶段D: 综合可视化和文档生成"""
        try:
            print("  📊 初始化综合仪表板...")
            dashboard = ComprehensiveFootForceDashboard(self.config, str(self.output_dir))
            
            print("  📈 生成综合验证报告...")
            report_file = dashboard.generate_comprehensive_report(
                static_results=self.test_results['phase_b'],
                dynamic_results=self.test_results['phase_c'],
                foot_force_config=self.foot_force_config
            )
            
            if report_file:
                print(f"  ✅ 综合报告生成完成: {Path(report_file).name}")
                self.test_results['phase_d'] = {
                    'success': True,
                    'report_file': report_file
                }
                return True
            else:
                print("  ❌ 综合报告生成失败")
                return False
                
        except Exception as e:
            self.logger.error(f"阶段D执行失败: {e}")
            print(f"  ❌ 阶段D执行异常: {e}")
            return False
    
    def generate_final_report(self, overall_success: bool) -> str:
        """生成最终测试报告"""
        try:
            report_data = {
                'validation_id': f"COMPLETE_FFVR_{self.timestamp}",
                'timestamp': datetime.now().isoformat(),
                'overall_success': overall_success,
                'phases': {
                    'phase_a': {
                        'name': '数据读取框架验证',
                        'success': self.test_results['phase_a'] is not None and self.test_results['phase_a'].get('success', False),
                        'results': self.test_results['phase_a']
                    },
                    'phase_b': {
                        'name': '静态力分布验证',
                        'success': self.test_results['phase_b'] is not None and self.test_results['phase_b'].get('success', False),
                        'results': self.test_results['phase_b']
                    },
                    'phase_c': {
                        'name': '动态响应测试',
                        'success': self.test_results['phase_c'] is not None and self.test_results['phase_c'].get('success', False),
                        'results': self.test_results['phase_c']
                    },
                    'phase_d': {
                        'name': '综合可视化和文档',
                        'success': self.test_results['phase_d'] is not None and self.test_results['phase_d'].get('success', False),
                        'results': self.test_results['phase_d']
                    }
                },
                'config': self.config,
            }
            
            # 计算汇总信息
            successful_phases = sum(1 for phase in report_data['phases'].values() if phase['success'])
            report_data['summary'] = {
                'total_phases': 4,
                'successful_phases': successful_phases,
                'completion_rate': successful_phases / 4 * 100
            }
            
            # 保存最终报告
            report_file = self.output_dir / f"final_validation_report_{self.timestamp}.json"
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report_data, f, indent=2, ensure_ascii=False)
            
            return str(report_file)
            
        except Exception as e:
            self.logger.error(f"生成最终报告失败: {e}")
            return ""
    
    def cleanup(self):
        """清理资源"""
        if self.foot_force_config:
            try:
                self.foot_force_config.cleanup()
            except:
                pass


def main():
    """主函数"""
    print("🚀 启动Unitree Go2足端力传感器完整验证流程")
    
    # 检查环境
    print("🔧 检查运行环境...")
    
    # 检查网络连接
    import subprocess
    try:
        result = subprocess.run(['ping', '-c', '1', '192.168.123.161'], 
                              capture_output=True, timeout=5)
        if result.returncode != 0:
            print("⚠️ 警告: 无法ping通机器人IP 192.168.123.161")
            response = input("是否继续执行测试？(y/N): ").strip().lower()
            if response not in ['y', 'yes', '是']:
                print("❌ 用户取消测试")
                return
    except:
        print("⚠️ 网络检查失败，继续执行测试")
    
    # 创建输出目录
    output_dir = Path("scripts/validation/foot_force/foot_force_validation/output")
    
    # 运行完整验证
    validator = CompleteFootForceValidation(str(output_dir))
    
    try:
        results = validator.run_complete_validation()
        
        print(f"\n📋 验证结果摘要:")
        print(f"   成功状态: {'✅ 成功' if results['success'] else '❌ 失败'}")
        print(f"   时间戳: {results['timestamp']}")
        
        if 'final_report' in results:
            print(f"   最终报告: {results['final_report']}")
            
    except KeyboardInterrupt:
        print("\n⏸️ 用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试执行异常: {e}")
    finally:
        validator.cleanup()


if __name__ == "__main__":
    main() 