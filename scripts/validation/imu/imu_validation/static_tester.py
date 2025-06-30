#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/static_tester.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU静态稳定性测试

import time
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector, CollectionMetrics

@dataclass
class StaticTestResults:
    """静态测试结果"""
    test_duration: float = 0.0
    sample_count: int = 0
    valid_samples: int = 0
    
    # 稳定性指标
    accelerometer_stability: Dict[str, float] = field(default_factory=dict)
    gyroscope_stability: Dict[str, float] = field(default_factory=dict)
    quaternion_stability: Dict[str, float] = field(default_factory=dict)
    
    # 精度指标
    gravity_accuracy: Dict[str, float] = field(default_factory=dict)
    bias_analysis: Dict[str, float] = field(default_factory=dict)
    noise_analysis: Dict[str, float] = field(default_factory=dict)
    
    # 温度分析（如可用）
    temperature_analysis: Dict[str, float] = field(default_factory=dict)
    
    # 测试评估
    test_status: str = "UNKNOWN"
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)

class IMUStaticTester:
    """IMU静态测试器"""
    
    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        初始化静态测试器
        
        Args:
            imu_config: IMU配置实例
            data_collector: 数据采集器实例
            config: 验证配置
        """
        self.imu_config = imu_config
        self.data_collector = data_collector
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # 测试配置
        self.test_config = config.get("test_parameters", {}).get("static_test", {})
        self.quality_thresholds = config.get("quality_thresholds", {})
        
        # 测试参数
        self.test_duration = self.test_config.get("duration_seconds", 60)
        self.stability_thresholds = self.test_config.get("stability_threshold", {})
        
    def run_static_stability_test(self) -> StaticTestResults:
        """
        运行静态稳定性测试
        
        Returns:
            StaticTestResults: 测试结果
        """
        self.logger.info("=" * 50)
        self.logger.info("开始IMU静态稳定性测试")
        self.logger.info("=" * 50)
        
        results = StaticTestResults()
        
        try:
            # 确保IMU已初始化
            if not self.imu_config.is_initialized:
                self.logger.error("IMU未初始化，无法进行静态测试")
                results.test_status = "FAIL"
                return results
            
            self.logger.info(f"测试持续时间: {self.test_duration}秒")
            self.logger.info("请确保机器人处于静止状态...")
            
            # 等待几秒让用户准备
            self.logger.info("5秒后开始测试...")
            time.sleep(5)
            
            # 开始数据采集
            self.logger.info("开始数据采集...")
            success = self.data_collector.start_collection(self.test_duration)
            
            if not success:
                self.logger.error("无法启动数据采集")
                results.test_status = "FAIL"
                return results
            
            # 等待采集完成
            start_time = time.time()
            while self.data_collector.is_collecting:
                elapsed = time.time() - start_time
                if elapsed >= self.test_duration + 10:  # 额外10秒超时
                    self.logger.warning("数据采集超时，强制停止")
                    break
                
                # 显示进度
                if int(elapsed) % 10 == 0:
                    self.logger.info(f"测试进度: {elapsed:.1f}/{self.test_duration}秒")
                
                time.sleep(1)
            
            # 停止采集并获取指标
            collection_metrics = self.data_collector.stop_collection()
            
            # 获取采集的数据
            collected_data = self.data_collector.get_collected_data()
            
            if not collected_data:
                self.logger.error("未采集到数据")
                results.test_status = "FAIL"
                return results
            
            self.logger.info(f"采集完成，共获得 {len(collected_data)} 个样本")
            
            # 分析数据
            results = self._analyze_static_data(collected_data, collection_metrics)
            
            # 评估测试结果
            self._evaluate_static_results(results)
            
            self.logger.info("=" * 50)
            self.logger.info(f"静态测试完成，状态: {results.test_status}")
            self.logger.info("=" * 50)
            
            return results
            
        except Exception as e:
            self.logger.error(f"静态测试失败: {e}")
            results.test_status = "ERROR"
            return results
    
    def _analyze_static_data(self, data: List[IMUReading], collection_metrics: CollectionMetrics) -> StaticTestResults:
        """分析静态测试数据"""
        results = StaticTestResults()
        
        try:
            results.test_duration = collection_metrics.collection_duration
            results.sample_count = len(data)
            results.valid_samples = collection_metrics.valid_samples
            
            # 提取传感器数据
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])
            quat_data = np.array([[r.quaternion[0], r.quaternion[1], r.quaternion[2], r.quaternion[3]] for r in data])
            
            # 分析加速度计稳定性
            results.accelerometer_stability = self._analyze_accelerometer_stability(accel_data)
            
            # 分析陀螺仪稳定性
            results.gyroscope_stability = self._analyze_gyroscope_stability(gyro_data)
            
            # 分析四元数稳定性
            results.quaternion_stability = self._analyze_quaternion_stability(quat_data)
            
            # 分析重力精度
            results.gravity_accuracy = self._analyze_gravity_accuracy(accel_data)
            
            # 分析偏置
            results.bias_analysis = self._analyze_bias(accel_data, gyro_data)
            
            # 分析噪声
            results.noise_analysis = self._analyze_noise(accel_data, gyro_data)
            
            # 温度分析（如果有温度数据）
            temp_data = [r.temperature for r in data if r.temperature is not None]
            if temp_data:
                results.temperature_analysis = self._analyze_temperature(temp_data)
            
            return results
            
        except Exception as e:
            self.logger.error(f"静态数据分析失败: {e}")
            results.test_status = "ERROR"
            return results
    
    def _analyze_accelerometer_stability(self, accel_data: np.ndarray) -> Dict[str, float]:
        """分析加速度计稳定性"""
        try:
            stability = {}
            
            # 计算各轴标准差
            stability['std_x'] = np.std(accel_data[:, 0])
            stability['std_y'] = np.std(accel_data[:, 1])
            stability['std_z'] = np.std(accel_data[:, 2])
            
            # 计算总标准差
            stability['std_total'] = np.sqrt(stability['std_x']**2 + stability['std_y']**2 + stability['std_z']**2)
            
            # 计算幅值稳定性
            magnitudes = np.linalg.norm(accel_data, axis=1)
            stability['magnitude_mean'] = np.mean(magnitudes)
            stability['magnitude_std'] = np.std(magnitudes)
            stability['magnitude_variation'] = stability['magnitude_std'] / stability['magnitude_mean'] * 100  # 百分比
            
            # 计算峰值到峰值变化
            stability['peak_to_peak_x'] = np.max(accel_data[:, 0]) - np.min(accel_data[:, 0])
            stability['peak_to_peak_y'] = np.max(accel_data[:, 1]) - np.min(accel_data[:, 1])
            stability['peak_to_peak_z'] = np.max(accel_data[:, 2]) - np.min(accel_data[:, 2])
            
            # 计算平均值
            stability['mean_x'] = np.mean(accel_data[:, 0])
            stability['mean_y'] = np.mean(accel_data[:, 1])
            stability['mean_z'] = np.mean(accel_data[:, 2])
            
            return stability
            
        except Exception as e:
            self.logger.error(f"加速度计稳定性分析失败: {e}")
            return {}
    
    def _analyze_gyroscope_stability(self, gyro_data: np.ndarray) -> Dict[str, float]:
        """分析陀螺仪稳定性"""
        try:
            stability = {}
            
            # 计算各轴标准差
            stability['std_x'] = np.std(gyro_data[:, 0])
            stability['std_y'] = np.std(gyro_data[:, 1])
            stability['std_z'] = np.std(gyro_data[:, 2])
            
            # 计算总标准差
            stability['std_total'] = np.sqrt(stability['std_x']**2 + stability['std_y']**2 + stability['std_z']**2)
            
            # 计算平均偏置（静态时应接近零）
            stability['bias_x'] = np.mean(gyro_data[:, 0])
            stability['bias_y'] = np.mean(gyro_data[:, 1])
            stability['bias_z'] = np.mean(gyro_data[:, 2])
            stability['bias_magnitude'] = np.sqrt(stability['bias_x']**2 + stability['bias_y']**2 + stability['bias_z']**2)
            
            # 计算峰值到峰值变化
            stability['peak_to_peak_x'] = np.max(gyro_data[:, 0]) - np.min(gyro_data[:, 0])
            stability['peak_to_peak_y'] = np.max(gyro_data[:, 1]) - np.min(gyro_data[:, 1])
            stability['peak_to_peak_z'] = np.max(gyro_data[:, 2]) - np.min(gyro_data[:, 2])
            
            # 计算角度漂移（积分）
            if len(gyro_data) > 1:
                # 假设采样率为100Hz（从配置获取）
                dt = 1.0 / self.imu_config.target_spec.sampling_rate_hz
                
                # 计算角度漂移（简单积分）
                drift_x = np.cumsum(gyro_data[:, 0]) * dt
                drift_y = np.cumsum(gyro_data[:, 1]) * dt
                drift_z = np.cumsum(gyro_data[:, 2]) * dt
                
                stability['drift_x_deg'] = np.degrees(drift_x[-1] - drift_x[0])
                stability['drift_y_deg'] = np.degrees(drift_y[-1] - drift_y[0])
                stability['drift_z_deg'] = np.degrees(drift_z[-1] - drift_z[0])
                stability['total_drift_deg'] = np.sqrt(stability['drift_x_deg']**2 + stability['drift_y_deg']**2 + stability['drift_z_deg']**2)
            
            return stability
            
        except Exception as e:
            self.logger.error(f"陀螺仪稳定性分析失败: {e}")
            return {}
    
    def _analyze_quaternion_stability(self, quat_data: np.ndarray) -> Dict[str, float]:
        """分析四元数稳定性"""
        try:
            stability = {}
            
            # 计算四元数各分量稳定性
            stability['std_w'] = np.std(quat_data[:, 0])
            stability['std_x'] = np.std(quat_data[:, 1])
            stability['std_y'] = np.std(quat_data[:, 2])
            stability['std_z'] = np.std(quat_data[:, 3])
            
            # 计算四元数幅值稳定性（应始终接近1）
            magnitudes = np.linalg.norm(quat_data, axis=1)
            stability['magnitude_mean'] = np.mean(magnitudes)
            stability['magnitude_std'] = np.std(magnitudes)
            stability['magnitude_error'] = abs(stability['magnitude_mean'] - 1.0)
            
            # 计算欧拉角稳定性
            euler_angles = []
            for quat in quat_data:
                roll, pitch, yaw = self.imu_config.quaternion_to_euler(tuple(quat))
                euler_angles.append([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])
            
            euler_array = np.array(euler_angles)
            stability['euler_std_roll'] = np.std(euler_array[:, 0])
            stability['euler_std_pitch'] = np.std(euler_array[:, 1])
            stability['euler_std_yaw'] = np.std(euler_array[:, 2])
            
            # 计算姿态漂移
            if len(euler_array) > 1:
                stability['orientation_drift_roll'] = euler_array[-1, 0] - euler_array[0, 0]
                stability['orientation_drift_pitch'] = euler_array[-1, 1] - euler_array[0, 1]
                stability['orientation_drift_yaw'] = euler_array[-1, 2] - euler_array[0, 2]
                stability['total_orientation_drift'] = np.sqrt(
                    stability['orientation_drift_roll']**2 + 
                    stability['orientation_drift_pitch']**2 + 
                    stability['orientation_drift_yaw']**2
                )
            
            return stability
            
        except Exception as e:
            self.logger.error(f"四元数稳定性分析失败: {e}")
            return {}
    
    def _analyze_gravity_accuracy(self, accel_data: np.ndarray) -> Dict[str, float]:
        """分析重力精度"""
        try:
            accuracy = {}
            
            # 计算加速度幅值
            magnitudes = np.linalg.norm(accel_data, axis=1)
            
            # 重力参考值
            gravity_ref = self.test_config.get("gravity_reference", 9.81)
            
            # 平均重力测量值
            accuracy['measured_gravity'] = np.mean(magnitudes)
            accuracy['gravity_error'] = accuracy['measured_gravity'] - gravity_ref
            accuracy['gravity_error_percent'] = abs(accuracy['gravity_error']) / gravity_ref * 100
            
            # 重力测量稳定性
            accuracy['gravity_std'] = np.std(magnitudes)
            accuracy['gravity_variation_percent'] = accuracy['gravity_std'] / accuracy['measured_gravity'] * 100
            
            # 重力方向分析（Z轴应主要受重力影响）
            accuracy['gravity_distribution_x'] = abs(np.mean(accel_data[:, 0]))
            accuracy['gravity_distribution_y'] = abs(np.mean(accel_data[:, 1]))
            accuracy['gravity_distribution_z'] = abs(np.mean(accel_data[:, 2]))
            
            # 计算倾斜角度（相对于重力）
            mean_accel = np.mean(accel_data, axis=0)
            mean_accel_norm = mean_accel / np.linalg.norm(mean_accel)
            
            # 假设重力沿负Z方向
            gravity_vector = np.array([0, 0, -1])
            dot_product = np.dot(mean_accel_norm, gravity_vector)
            accuracy['tilt_angle_deg'] = np.degrees(np.arccos(np.clip(dot_product, -1.0, 1.0)))
            
            return accuracy
            
        except Exception as e:
            self.logger.error(f"重力精度分析失败: {e}")
            return {}
    
    def _analyze_bias(self, accel_data: np.ndarray, gyro_data: np.ndarray) -> Dict[str, float]:
        """分析传感器偏置"""
        try:
            bias = {}
            
            # 加速度计偏置分析
            # 静态时，加速度计应测量重力
            gravity_ref = self.test_config.get("gravity_reference", 9.81)
            
            mean_accel = np.mean(accel_data, axis=0)
            expected_accel = np.array([0, 0, -gravity_ref])  # 假设Z轴向上
            
            bias['accel_bias_x'] = mean_accel[0] - expected_accel[0]
            bias['accel_bias_y'] = mean_accel[1] - expected_accel[1]
            bias['accel_bias_z'] = mean_accel[2] - expected_accel[2]
            bias['accel_bias_magnitude'] = np.linalg.norm([bias['accel_bias_x'], bias['accel_bias_y'], bias['accel_bias_z']])
            
            # 陀螺仪偏置分析
            # 静态时，陀螺仪应读数为零
            mean_gyro = np.mean(gyro_data, axis=0)
            
            bias['gyro_bias_x'] = mean_gyro[0]
            bias['gyro_bias_y'] = mean_gyro[1]
            bias['gyro_bias_z'] = mean_gyro[2]
            bias['gyro_bias_magnitude'] = np.linalg.norm(mean_gyro)
            
            # 偏置稳定性（Allan方差简化版本）
            bias['accel_bias_stability_x'] = np.std(accel_data[:, 0])
            bias['accel_bias_stability_y'] = np.std(accel_data[:, 1])
            bias['accel_bias_stability_z'] = np.std(accel_data[:, 2])
            
            bias['gyro_bias_stability_x'] = np.std(gyro_data[:, 0])
            bias['gyro_bias_stability_y'] = np.std(gyro_data[:, 1])
            bias['gyro_bias_stability_z'] = np.std(gyro_data[:, 2])
            
            return bias
            
        except Exception as e:
            self.logger.error(f"偏置分析失败: {e}")
            return {}
    
    def _analyze_noise(self, accel_data: np.ndarray, gyro_data: np.ndarray) -> Dict[str, float]:
        """分析传感器噪声"""
        try:
            noise = {}
            
            # 移除趋势（去趋势化）
            def detrend(data):
                """简单的线性去趋势"""
                n = len(data)
                t = np.arange(n)
                coeffs = np.polyfit(t, data, 1)
                trend = np.polyval(coeffs, t)
                return data - trend
            
            # 加速度计噪声分析
            accel_detrended = np.array([detrend(accel_data[:, i]) for i in range(3)]).T
            
            noise['accel_noise_x'] = np.std(accel_detrended[:, 0])
            noise['accel_noise_y'] = np.std(accel_detrended[:, 1])
            noise['accel_noise_z'] = np.std(accel_detrended[:, 2])
            noise['accel_noise_total'] = np.sqrt(np.sum([noise[f'accel_noise_{ax}']**2 for ax in ['x', 'y', 'z']]))
            
            # 陀螺仪噪声分析
            gyro_detrended = np.array([detrend(gyro_data[:, i]) for i in range(3)]).T
            
            noise['gyro_noise_x'] = np.std(gyro_detrended[:, 0])
            noise['gyro_noise_y'] = np.std(gyro_detrended[:, 1])
            noise['gyro_noise_z'] = np.std(gyro_detrended[:, 2])
            noise['gyro_noise_total'] = np.sqrt(np.sum([noise[f'gyro_noise_{ax}']**2 for ax in ['x', 'y', 'z']]))
            
            # 信噪比估算
            accel_signal = np.mean(np.linalg.norm(accel_data, axis=1))
            gyro_signal = np.mean(np.linalg.norm(gyro_data, axis=1))
            
            if noise['accel_noise_total'] > 0:
                noise['accel_snr_db'] = 20 * np.log10(accel_signal / noise['accel_noise_total'])
            else:
                noise['accel_snr_db'] = float('inf')
            
            if noise['gyro_noise_total'] > 0:
                noise['gyro_snr_db'] = 20 * np.log10(gyro_signal / noise['gyro_noise_total'])
            else:
                noise['gyro_snr_db'] = float('inf')
            
            return noise
            
        except Exception as e:
            self.logger.error(f"噪声分析失败: {e}")
            return {}
    
    def _analyze_temperature(self, temp_data: List[float]) -> Dict[str, float]:
        """分析温度数据"""
        try:
            temp_analysis = {}
            
            temp_analysis['mean_temperature'] = statistics.mean(temp_data)
            temp_analysis['temperature_std'] = statistics.stdev(temp_data) if len(temp_data) > 1 else 0
            temp_analysis['temperature_range'] = max(temp_data) - min(temp_data)
            temp_analysis['temperature_drift'] = temp_data[-1] - temp_data[0] if len(temp_data) > 1 else 0
            
            return temp_analysis
            
        except Exception as e:
            self.logger.error(f"温度分析失败: {e}")
            return {}
    
    def _evaluate_static_results(self, results: StaticTestResults):
        """评估静态测试结果"""
        try:
            pass_criteria = {}
            recommendations = []
            
            # 评估加速度计稳定性
            accel_std_threshold = self.stability_thresholds.get("accelerometer_std_max", 0.05)
            accel_pass = all(results.accelerometer_stability.get(f'std_{ax}', 0) < accel_std_threshold 
                           for ax in ['x', 'y', 'z'])
            pass_criteria['accelerometer_stability'] = accel_pass
            
            if not accel_pass:
                recommendations.append("加速度计稳定性不足，检查机械振动或传感器校准")
            
            # 评估陀螺仪稳定性
            gyro_std_threshold = self.stability_thresholds.get("gyroscope_std_max", 0.1)
            gyro_pass = all(results.gyroscope_stability.get(f'std_{ax}', 0) < gyro_std_threshold 
                          for ax in ['x', 'y', 'z'])
            pass_criteria['gyroscope_stability'] = gyro_pass
            
            if not gyro_pass:
                recommendations.append("陀螺仪稳定性不足，检查温度稳定性或传感器校准")
            
            # 评估重力精度
            gravity_error_threshold = self.quality_thresholds.get("accuracy", {}).get("gravity_error_max_percent", 2)
            gravity_pass = results.gravity_accuracy.get('gravity_error_percent', 100) < gravity_error_threshold
            pass_criteria['gravity_accuracy'] = gravity_pass
            
            if not gravity_pass:
                recommendations.append("重力测量精度不足，需要校准加速度计")
            
            # 评估四元数稳定性
            quat_drift_threshold = self.stability_thresholds.get("quaternion_drift_max", 0.01)
            quat_pass = results.quaternion_stability.get('magnitude_error', 1) < quat_drift_threshold
            pass_criteria['quaternion_stability'] = quat_pass
            
            if not quat_pass:
                recommendations.append("四元数稳定性不足，检查姿态融合算法")
            
            # 评估噪声水平
            accel_noise_threshold = self.quality_thresholds.get("noise_levels", {}).get("accelerometer_noise_max", 0.02)
            gyro_noise_threshold = self.quality_thresholds.get("noise_levels", {}).get("gyroscope_noise_max", 0.01)
            
            accel_noise_pass = results.noise_analysis.get('accel_noise_total', 1) < accel_noise_threshold
            gyro_noise_pass = results.noise_analysis.get('gyro_noise_total', 1) < gyro_noise_threshold
            
            pass_criteria['accelerometer_noise'] = accel_noise_pass
            pass_criteria['gyroscope_noise'] = gyro_noise_pass
            
            if not accel_noise_pass:
                recommendations.append("加速度计噪声过高，检查电磁干扰或滤波设置")
            
            if not gyro_noise_pass:
                recommendations.append("陀螺仪噪声过高，检查电源稳定性或滤波设置")
            
            # 确定总体状态
            all_passed = all(pass_criteria.values())
            critical_passed = all([
                pass_criteria.get('accelerometer_stability', False),
                pass_criteria.get('gyroscope_stability', False),
                pass_criteria.get('gravity_accuracy', False)
            ])
            
            if all_passed:
                results.test_status = "PASS"
            elif critical_passed:
                results.test_status = "WARNING"
            else:
                results.test_status = "FAIL"
            
            if not recommendations:
                recommendations.append("所有静态测试指标均符合要求")
            
            results.pass_criteria = pass_criteria
            results.recommendations = recommendations
            
        except Exception as e:
            self.logger.error(f"评估静态测试结果失败: {e}")
            results.test_status = "ERROR"
            results.recommendations = [f"评估失败: {str(e)}"] 