#!/usr/bin/env python3
"""
有线网络环境下的足端力传感器验证脚本
针对网线长度限制优化的测试方案
Generated: 2025-06-27 16:58:15
"""

import sys
import time
import logging
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from foot_force_validation.foot_force_config import FootForceConfig
from foot_force_validation.data_collector import FootForceDataCollector  
from foot_force_validation.static_tester import StaticFootForceTester
from foot_force_validation.dynamic_tester import DynamicFootForceTester
from foot_force_validation.comprehensive_dashboard import ComprehensiveFootForceDashboard

class WiredNetworkValidator:
    """有线网络环境下的足端力验证器"""
    
    def __init__(self):
        """初始化验证器"""
        self.setup_logging()
        self.robot_ip = "192.168.123.161"  # 确认的机器人IP
        self.local_ip = "192.168.123.18"   # 本机IP
        self.config = None
        self.results = {}
        
        self.logger.info(f"有线网络验证器初始化完成")
        self.logger.info(f"机器人IP: {self.robot_ip}")
        self.logger.info(f"本机IP: {self.local_ip}")
    
    def setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
    
    def test_connection(self):
        """测试机器人连接"""
        self.logger.info("🔌 测试机器人连接...")
        
        try:
            self.config = FootForceConfig()
            # 尝试建立连接
            connection_test = self.config.test_connection()
            if connection_test:
                self.logger.info("✅ 机器人连接成功")
                return True
            else:
                self.logger.warning("⚠️ 机器人连接测试失败，但配置已初始化")
                return False
        except Exception as e:
            self.logger.error(f"❌ 连接测试异常: {e}")
            return False
    
    def run_stationary_tests(self):
        """运行静态测试（适合有线网络）"""
        self.logger.info("🧪 开始静态测试（适合有线网络环境）...")
        
        try:
            # 静态测试不需要移动，适合有线网络
            static_tester = StaticFootForceTester(self.config)
            
            # 零负载测试（机器人静止）
            self.logger.info("📊 零负载测试 - 机器人保持静止...")
            zero_load_result = static_tester.run_zero_load_test(duration=10.0)
            
            # 静态站立测试
            self.logger.info("🏠 静态站立测试 - 机器人正常站立...")
            standing_result = static_tester.run_static_standing_test(duration=15.0)
            
            # 小幅度重心转移测试（适合有线限制）
            self.logger.info("⚖️ 重心转移测试 - 小幅度移动...")
            weight_shift_result = static_tester.run_weight_shift_test(duration=20.0)
            
            self.results['static'] = {
                'zero_load': zero_load_result,
                'standing': standing_result,
                'weight_shift': weight_shift_result
            }
            
            self.logger.info("✅ 静态测试完成")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ 静态测试失败: {e}")
            self.results['static'] = {'error': str(e)}
            return False
    
    def run_limited_dynamic_tests(self):
        """运行受限动态测试（考虑网线长度）"""
        self.logger.info("🏃 开始受限动态测试（适应网线长度限制）...")
        
        try:
            dynamic_tester = DynamicFootForceTester(self.config)
            
            # 原地踏步测试（不需要移动）
            self.logger.info("👣 原地踏步测试...")
            marching_result = self._run_marching_test(dynamic_tester)
            
            # 小幅度摆动测试（网线范围内）
            self.logger.info("🌊 小幅度摆动测试...")
            sway_result = self._run_sway_test(dynamic_tester)
            
            # 坐下-站起测试（垂直运动）
            self.logger.info("📈 坐下-站起测试...")
            sit_stand_result = self._run_sit_stand_test(dynamic_tester)
            
            self.results['dynamic'] = {
                'marching': marching_result,
                'sway': sway_result,
                'sit_stand': sit_stand_result
            }
            
            self.logger.info("✅ 受限动态测试完成")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ 动态测试失败: {e}")
            self.results['dynamic'] = {'error': str(e)}
            return False
    
    def _run_marching_test(self, dynamic_tester):
        """原地踏步测试"""
        print("\n" + "="*60)
        print("🚶 原地踏步测试")
        print("请让机器人进行原地踏步动作，测试时间: 30秒")
        print("优点: 无需移动，网线不受限制")
        print("="*60)
        
        input("按回车键开始原地踏步测试...")
        
        # 模拟原地踏步测试
        return dynamic_tester.run_single_test("marching_in_place", duration=30.0)
    
    def _run_sway_test(self, dynamic_tester):
        """小幅度摆动测试"""
        print("\n" + "="*60)
        print("🌊 小幅度摆动测试")
        print("请让机器人进行小幅度左右摆动，测试时间: 20秒")
        print("移动范围: 网线长度内（建议<1米）")
        print("="*60)
        
        input("按回车键开始摆动测试...")
        
        return dynamic_tester.run_single_test("limited_sway", duration=20.0)
    
    def _run_sit_stand_test(self, dynamic_tester):
        """坐下-站起测试"""
        print("\n" + "="*60)
        print("📈 坐下-站起测试")
        print("请让机器人进行坐下-站起动作，测试时间: 25秒")
        print("优点: 主要是垂直运动，网线影响最小")
        print("="*60)
        
        input("按回车键开始坐下-站起测试...")
        
        return dynamic_tester.run_single_test("sit_stand_cycle", duration=25.0)
    
    def generate_wired_report(self):
        """生成针对有线网络的报告"""
        self.logger.info("📊 生成有线网络环境验证报告...")
        
        try:
            dashboard = ComprehensiveFootForceDashboard(self.config)
            
            # 生成专门的有线网络报告
            report = dashboard.generate_wired_network_report(
                static_results=self.results.get('static'),
                dynamic_results=self.results.get('dynamic'),
                network_config={
                    'robot_ip': self.robot_ip,
                    'local_ip': self.local_ip,
                    'connection_type': 'wired_ethernet',
                    'limitations': 'cable_length_restricted_movement'
                }
            )
            
            self.logger.info("✅ 有线网络报告生成完成")
            return report
            
        except Exception as e:
            self.logger.error(f"❌ 报告生成失败: {e}")
            return None
    
    def run_complete_wired_validation(self):
        """运行完整的有线网络验证流程"""
        print("\n" + "="*80)
        print("🚀 Unitree Go2 有线网络环境足端力传感器验证")
        print("📅 针对网线长度限制优化的测试方案")
        print("="*80)
        
        # 阶段1: 连接测试
        print("\n🔌 阶段1: 机器人连接测试")
        if not self.test_connection():
            print("❌ 连接失败，尝试继续进行模拟测试...")
        
        # 阶段2: 静态测试（不受网线限制）
        print("\n🧪 阶段2: 静态测试（适合有线网络）")
        self.run_stationary_tests()
        
        # 阶段3: 受限动态测试
        print("\n🏃 阶段3: 受限动态测试（考虑网线长度）")
        self.run_limited_dynamic_tests()
        
        # 阶段4: 生成报告
        print("\n📊 阶段4: 生成有线网络专用报告")
        report = self.generate_wired_report()
        
        # 总结
        print("\n" + "="*80)
        print("📋 有线网络验证完成总结:")
        print(f"   🌐 机器人IP: {self.robot_ip}")
        print(f"   💻 本机IP: {self.local_ip}")
        print("   ✅ 静态测试: 完全适合有线网络")
        print("   ⚡ 动态测试: 已针对网线限制优化")
        print("   📄 专用报告: 包含网络环境分析")
        print("="*80)
        
        return report

if __name__ == "__main__":
    validator = WiredNetworkValidator()
    validator.run_complete_wired_validation() 