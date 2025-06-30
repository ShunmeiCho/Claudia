#!/usr/bin/env python3
# 简化的IMU验证测试（无硬件依赖）

import time
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Dict, Any

@dataclass
class MockIMUReading:
    """模拟IMU读数"""
    timestamp: float
    accelerometer: Tuple[float, float, float]
    gyroscope: Tuple[float, float, float] 
    quaternion: Tuple[float, float, float, float]
    temperature: int = 25

class MockIMUValidator:
    """模拟IMU验证器"""
    
    def __init__(self):
        self.readings = []
        
    def generate_mock_data(self, duration: float, sample_rate: float = 100) -> List[MockIMUReading]:
        """生成模拟数据"""
        readings = []
        num_samples = int(duration * sample_rate)
        
        for i in range(num_samples):
            t = time.time() + i / sample_rate
            
            # 模拟静态数据（主要是重力）
            accel = (
                np.random.normal(0, 0.01),  # X轴噪声
                np.random.normal(0, 0.01),  # Y轴噪声  
                np.random.normal(-9.81, 0.02)  # Z轴重力+噪声
            )
            
            # 模拟静态陀螺仪（接近零）
            gyro = (
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001)
            )
            
            # 模拟四元数（接近单位四元数）
            quat = (
                np.random.normal(1, 0.001),
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001),  
                np.random.normal(0, 0.001)
            )
            
            reading = MockIMUReading(t, accel, gyro, quat)
            readings.append(reading)
            
        return readings
    
    def analyze_static_stability(self, readings: List[MockIMUReading]) -> Dict[str, Any]:
        """分析静态稳定性"""
        if not readings:
            return {'status': 'FAIL', 'error': '无数据'}
            
        # 提取数据
        accels = np.array([r.accelerometer for r in readings])
        gyros = np.array([r.gyroscope for r in readings])
        
        # 计算统计
        accel_std = np.std(accels, axis=0)
        gyro_std = np.std(gyros, axis=0)
        gravity_mag = np.mean(np.linalg.norm(accels, axis=1))
        
        # 评估
        accel_stable = np.max(accel_std) < 0.05
        gyro_stable = np.max(gyro_std) < 0.01
        gravity_accurate = abs(gravity_mag - 9.81) < 0.2
        
        return {
            'status': 'PASS' if all([accel_stable, gyro_stable, gravity_accurate]) else 'FAIL',
            'accelerometer_std': accel_std.tolist(),
            'gyroscope_std': gyro_std.tolist(), 
            'gravity_magnitude': float(gravity_mag),
            'test_duration': readings[-1].timestamp - readings[0].timestamp,
            'sample_count': len(readings),
            'pass_criteria': {
                'accelerometer_stability': accel_stable,
                'gyroscope_stability': gyro_stable,
                'gravity_accuracy': gravity_accurate
            }
        }
    
    def run_mock_validation(self) -> Dict[str, Any]:
        """运行模拟验证"""
        print("🧪 开始模拟IMU验证...")
        
        # 生成模拟数据
        print("📊 生成模拟数据...")
        readings = self.generate_mock_data(10.0)  # 10秒数据
        
        # 分析
        print("🔍 分析静态稳定性...")
        static_results = self.analyze_static_stability(readings)
        
        # 生成报告
        print("📋 生成验证报告...")
        report = {
            'test_type': 'mock_validation',
            'test_timestamp': time.time(),
            'static_stability': static_results,
            'overall_status': static_results['status']
        }
        
        return report

def main():
    """主函数"""
    print("=" * 50)
    print("🧪 简化IMU验证测试")
    print("=" * 50)
    
    validator = MockIMUValidator()
    report = validator.run_mock_validation()
    
    print("\n📊 验证结果:")
    print(f"状态: {report['overall_status']}")
    print(f"样本数: {report['static_stability']['sample_count']}")
    print(f"重力测量: {report['static_stability']['gravity_magnitude']:.3f} m/s²")
    print(f"加速度标准差: {report['static_stability']['accelerometer_std']}")
    
    return 0 if report['overall_status'] == 'PASS' else 1

if __name__ == "__main__":
    exit(main())
