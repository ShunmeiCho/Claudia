#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/basic_test.py
# Generated: 2025-06-27 14:09:30 CST
# Purpose: 足端力传感器数据读取框架基础测试

import os
import sys
import time
import json
import logging
import argparse
from pathlib import Path
from typing import Dict, Any
from datetime import datetime

# 添加模块路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from foot_force_config import FootForceConfig, FootForceReading, create_default_config, format_force_reading
from data_collector import FootForceDataCollector, FootForceData

class FootForceBasicTest:
    """足端力传感器基础测试类"""
    
    def __init__(self, config_file: str = "validation_config.json"):
        """
        初始化测试
        
        Args:
            config_file: 配置文件路径
        """
        self.setup_logging()
        self.logger = logging.getLogger(__name__)
        
        # 加载配置
        self.config = self.load_config(config_file)
        
        # 初始化组件
        self.foot_force_config: FootForceConfig = None
        self.data_collector: FootForceDataCollector = None
        
        # 输出目录
        self.output_dir = Path("output") / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.logger.info("足端力传感器基础测试初始化完成")
    
    def setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/foot_force_basic_test.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
    
    def load_config(self, config_file: str) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            config_path = Path(__file__).parent / config_file
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            self.logger.info(f"配置文件加载成功: {config_path}")
            return config
        except Exception as e:
            self.logger.error(f"配置文件加载失败: {e}")
            # 返回默认配置
            return self.get_default_config()
    
    def get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            "general": {
                "network_interface": "eth0",
                "domain_id": 0
            },
            "foot_force_config": {
                "sampling_rate_hz": 500,
                "force_threshold": 5.0,
                "max_force_per_foot": 200.0
            },
            "data_collection": {
                "default_duration": 10.0,
                "quality_threshold": 0.75
            }
        }
    
    def initialize_foot_force_system(self) -> bool:
        """初始化足端力传感器系统"""
        try:
            self.logger.info("初始化足端力传感器系统...")
            
            # 创建FootForceConfig
            network_interface = self.config["general"]["network_interface"]
            self.foot_force_config = create_default_config(network_interface)
            
            # 初始化连接
            domain_id = self.config["general"]["domain_id"]
            if not self.foot_force_config.initialize_connection(domain_id):
                self.logger.error("足端力传感器连接初始化失败")
                return False
            
            # 创建数据收集器
            self.data_collector = FootForceDataCollector(
                config=self.config,
                foot_force_config=self.foot_force_config
            )
            
            self.logger.info("足端力传感器系统初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"初始化足端力传感器系统失败: {e}")
            return False
    
    def test_connection(self) -> bool:
        """测试连接"""
        self.logger.info("测试足端力传感器连接...")
        
        try:
            # 尝试获取一次数据
            reading = self.foot_force_config.get_latest_reading()
            if reading:
                self.logger.info("连接测试成功，收到数据:")
                self.logger.info(format_force_reading(reading))
                return True
            else:
                self.logger.warning("连接测试未收到数据")
                return False
                
        except Exception as e:
            self.logger.error(f"连接测试失败: {e}")
            return False
    
    def test_data_collection(self, duration: float = 10.0) -> bool:
        """测试数据收集"""
        self.logger.info(f"开始数据收集测试，持续时间: {duration}秒")
        
        try:
            # 添加实时数据显示回调
            self.data_collector.add_data_callback(self.real_time_data_callback)
            
            # 开始收集
            if not self.data_collector.start_collection(duration):
                self.logger.error("数据收集启动失败")
                return False
            
            self.logger.info("数据收集已启动，等待完成...")
            
            # 实时监控
            start_time = time.time()
            last_report_time = start_time
            
            while self.data_collector.is_collecting:
                current_time = time.time()
                
                # 每5秒显示一次实时指标
                if current_time - last_report_time >= 5.0:
                    metrics = self.data_collector.get_real_time_metrics()
                    self.logger.info(f"实时指标: FPS={metrics['current_fps']:.1f}, "
                                   f"样本数={metrics['total_samples']}, "
                                   f"数据质量={metrics['data_quality']:.2f}, "
                                   f"接触状态={metrics['contact_summary']}")
                    last_report_time = current_time
                
                time.sleep(1.0)
            
            # 获取最终指标
            final_metrics = self.data_collector.stop_collection()
            self.logger.info("数据收集完成")
            self.print_collection_metrics(final_metrics)
            
            return True
            
        except Exception as e:
            self.logger.error(f"数据收集测试失败: {e}")
            return False
    
    def real_time_data_callback(self, data: FootForceData):
        """实时数据回调（仅在调试模式下显示详细信息）"""
        # 简化的实时显示，避免日志过多
        if len(self.data_collector.raw_data) % 100 == 0:  # 每100个样本显示一次
            self.logger.debug(f"样本 #{len(self.data_collector.raw_data)}: "
                            f"总力={data.total_force:.1f}N, "
                            f"稳定性={data.stability_index:.2f}, "
                            f"平衡={data.force_balance:.2f}")
    
    def print_collection_metrics(self, metrics):
        """打印收集指标"""
        self.logger.info("=== 数据收集指标 ===")
        self.logger.info(f"收集时长: {metrics.collection_duration:.2f}秒")
        self.logger.info(f"总样本数: {metrics.total_samples}")
        self.logger.info(f"平均采样率: {metrics.avg_sampling_rate:.1f} Hz")
        self.logger.info(f"数据质量评分: {metrics.data_quality_score:.2f}")
        
        if metrics.force_stats:
            self.logger.info("足端力统计:")
            for foot_label, stats in metrics.force_stats.items():
                self.logger.info(f"  {foot_label}: 平均Fz={stats['mean_fz']:.2f}N, "
                               f"接触事件={stats['contact_events']}")
        
        if metrics.total_force_stats:
            self.logger.info(f"总力统计: 平均={metrics.total_force_stats['mean']:.2f}N, "
                           f"范围=[{metrics.total_force_stats['min']:.1f}, "
                           f"{metrics.total_force_stats['max']:.1f}]N")
    
    def save_test_results(self) -> bool:
        """保存测试结果"""
        try:
            self.logger.info("保存测试结果...")
            
            # 保存JSON格式数据
            json_path = self.output_dir / "foot_force_test_data.json"
            if self.data_collector.save_data(str(json_path)):
                self.logger.info(f"JSON数据已保存: {json_path}")
            
            # 保存CSV格式数据
            csv_path = self.output_dir / "foot_force_test_data.csv"
            if self.data_collector.save_data_csv(str(csv_path)):
                self.logger.info(f"CSV数据已保存: {csv_path}")
            
            # 保存统计信息
            stats_path = self.output_dir / "test_statistics.json"
            stats = self.data_collector.get_statistics()
            with open(stats_path, 'w', encoding='utf-8') as f:
                json.dump(stats, f, indent=2, ensure_ascii=False)
            self.logger.info(f"统计信息已保存: {stats_path}")
            
            # 保存测试配置
            config_path = self.output_dir / "test_config.json"
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
            
            return True
            
        except Exception as e:
            self.logger.error(f"保存测试结果失败: {e}")
            return False
    
    def test_calibration(self) -> bool:
        """测试校准功能"""
        self.logger.info("开始校准测试...")
        
        try:
            # 执行零点校准
            calibration_duration = self.config.get("static_validation", {}).get("calibration_duration", 5.0)
            
            if self.foot_force_config.zero_calibration(calibration_duration):
                self.logger.info("零点校准成功")
                
                # 显示校准结果
                foot_info = self.foot_force_config.get_foot_info()
                self.logger.info("校准偏移值:")
                for foot_label, offset in foot_info['calibration_offset'].items():
                    self.logger.info(f"  {foot_label}: {offset}")
                
                return True
            else:
                self.logger.error("零点校准失败")
                return False
                
        except Exception as e:
            self.logger.error(f"校准测试失败: {e}")
            return False
    
    def run_basic_test(self, test_duration: float = None) -> bool:
        """运行基础测试"""
        self.logger.info("=== 开始足端力传感器基础测试 ===")
        
        try:
            # 1. 初始化系统
            if not self.initialize_foot_force_system():
                return False
            
            # 2. 测试连接
            if not self.test_connection():
                return False
            
            # 3. 测试校准（可选）
            calibration_test = input("是否执行校准测试? (y/n): ").lower().strip() == 'y'
            if calibration_test:
                self.test_calibration()
            
            # 4. 数据收集测试
            if test_duration is None:
                test_duration = self.config["data_collection"]["default_duration"]
            
            if not self.test_data_collection(test_duration):
                return False
            
            # 5. 保存结果
            if not self.save_test_results():
                return False
            
            self.logger.info("=== 足端力传感器基础测试完成 ===")
            self.logger.info(f"测试结果已保存到: {self.output_dir}")
            
            return True
            
        except KeyboardInterrupt:
            self.logger.info("用户中断测试")
            return False
        except Exception as e:
            self.logger.error(f"基础测试失败: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        try:
            if self.data_collector and self.data_collector.is_collecting:
                self.data_collector.stop_collection()
            
            if self.foot_force_config:
                self.foot_force_config.cleanup()
            
            self.logger.info("资源清理完成")
            
        except Exception as e:
            self.logger.error(f"清理资源时出错: {e}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="足端力传感器基础测试")
    parser.add_argument('--duration', type=float, default=10.0, 
                       help='数据收集持续时间(秒)')
    parser.add_argument('--config', type=str, default='validation_config.json',
                       help='配置文件路径')
    parser.add_argument('--debug', action='store_true',
                       help='启用调试模式')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 创建输出目录
    os.makedirs('logs', exist_ok=True)
    os.makedirs('output', exist_ok=True)
    
    # 运行测试
    test = FootForceBasicTest(args.config)
    success = test.run_basic_test(args.duration)
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main() 