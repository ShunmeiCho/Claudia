#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/calibration_analyzer.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU校准验证和分析

import time
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
import json
from pathlib import Path

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector, CollectionMetrics

@dataclass
class CalibrationResults:
    """校准分析结果"""
    calibration_type: str = ""
    sample_count: int = 0
    test_duration: float = 0.0
    
    # 加速度计校准
    accelerometer_calibration: Dict[str, float] = field(default_factory=dict)
    gravity_calibration: Dict[str, float] = field(default_factory=dict)
    accel_bias: Dict[str, float] = field(default_factory=dict)
    accel_scale_factor: Dict[str, float] = field(default_factory=dict)
    accel_cross_axis: Dict[str, float] = field(default_factory=dict)
    
    # 陀螺仪校准
    gyroscope_calibration: Dict[str, float] = field(default_factory=dict)
    gyro_bias: Dict[str, float] = field(default_factory=dict)
    gyro_scale_factor: Dict[str, float] = field(default_factory=dict)
    gyro_noise_characteristics: Dict[str, float] = field(default_factory=dict)
    
    # 姿态校准
    attitude_calibration: Dict[str, float] = field(default_factory=dict)
    quaternion_consistency: Dict[str, float] = field(default_factory=dict)
    euler_accuracy: Dict[str, float] = field(default_factory=dict)
    
    # 温度补偿（如可用）
    temperature_compensation: Dict[str, float] = field(default_factory=dict)
    
    # 校准质量指标
    calibration_quality: Dict[str, float] = field(default_factory=dict)
    
    # 测试评估
    test_status: str = "UNKNOWN"
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)

class IMUCalibrationAnalyzer:
    """IMU校准分析器"""
    
    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        初始化校准分析器
        
        Args:
            imu_config: IMU配置实例
            data_collector: 数据采集器实例
            config: 验证配置
        """
        self.imu_config = imu_config
        self.data_collector = data_collector
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # 校准配置
        self.calib_config = config.get("test_parameters", {}).get("calibration_analysis", {})
        self.quality_thresholds = config.get("quality_thresholds", {})
        
        # 校准参数
        self.gravity_reference = self.calib_config.get("gravity_reference", 9.81)
        self.gravity_tolerance = self.calib_config.get("gravity_tolerance", 0.2)
        self.bias_samples = self.calib_config.get("bias_analysis_samples", 1000)
        self.enable_temperature = self.calib_config.get("temperature_analysis", True)
        
    def run_comprehensive_calibration_analysis(self) -> CalibrationResults:
        """
        运行综合校准分析
        
        Returns:
            CalibrationResults: 校准分析结果
        """
        self.logger.info("=" * 50)
        self.logger.info("开始IMU校准验证分析")
        self.logger.info("=" * 50)
        
        results = CalibrationResults()
        results.calibration_type = "comprehensive"
        
        try:
            # 确保IMU已初始化
            if not self.imu_config.is_initialized:
                self.logger.error("IMU未初始化，无法进行校准分析")
                results.test_status = "FAIL"
                return results
            
            # 收集校准分析数据
            self.logger.info("收集校准分析数据...")
            self.logger.info("请确保机器人在多个不同姿态下保持静止...")
            
            calibration_data = self._collect_calibration_data()
            
            if not calibration_data:
                self.logger.error("校准数据收集失败")
                results.test_status = "FAIL"
                return results
            
            results.sample_count = len(calibration_data)
            
            # 执行各项校准分析
            self.logger.info("执行加速度计校准分析...")
            results = self._analyze_accelerometer_calibration(results, calibration_data)
            
            self.logger.info("执行陀螺仪校准分析...")
            results = self._analyze_gyroscope_calibration(results, calibration_data)
            
            self.logger.info("执行姿态校准分析...")
            results = self._analyze_attitude_calibration(results, calibration_data)
            
            # 温度补偿分析（如有温度数据）
            if self.enable_temperature:
                self.logger.info("执行温度补偿分析...")
                results = self._analyze_temperature_compensation(results, calibration_data)
            
            # 计算整体校准质量
            self.logger.info("计算校准质量指标...")
            results = self._calculate_calibration_quality(results, calibration_data)
            
            # 评估校准结果
            self._evaluate_calibration_results(results)
            
            self.logger.info("=" * 50)
            self.logger.info(f"校准分析完成，状态: {results.test_status}")
            self.logger.info("=" * 50)
            
            return results
            
        except Exception as e:
            self.logger.error(f"校准分析失败: {e}")
            results.test_status = "ERROR"
            return results
    
    def _collect_calibration_data(self) -> List[IMUReading]:
        """收集校准数据"""
        try:
            all_data = []
            
            # 定义测试姿态
            positions = [
                "水平静置（正常姿态）",
                "左侧倾斜90度",
                "右侧倾斜90度", 
                "前倾90度",
                "后倾90度",
                "倒置180度"
            ]
            
            for i, position in enumerate(positions):
                self.logger.info(f"\n位置 {i+1}/{len(positions)}: {position}")
                self.logger.info("请调整机器人到指定位置并保持静止...")
                self.logger.info("15秒后开始数据采集...")
                time.sleep(15)
                
                # 收集该位置的数据
                self.logger.info("开始数据采集...")
                success = self.data_collector.start_collection(30.0)  # 30秒采集
                
                if not success:
                    self.logger.error(f"位置 {position} 数据采集启动失败")
                    continue
                
                # 等待采集完成
                start_time = time.time()
                while self.data_collector.is_collecting:
                    elapsed = time.time() - start_time
                    if elapsed >= 40:  # 40秒超时
                        self.logger.warning("数据采集超时")
                        break
                    
                    if int(elapsed) % 5 == 0:
                        self.logger.info(f"采集进度: {elapsed:.1f}/30.0秒")
                    
                    time.sleep(1)
                
                # 获取数据
                self.data_collector.stop_collection()
                position_data = self.data_collector.get_collected_data()
                
                if position_data:
                    # 添加位置标识
                    for reading in position_data:
                        reading.position_id = i
                    
                    all_data.extend(position_data)
                    self.logger.info(f"位置 {position} 采集完成，获得 {len(position_data)} 个样本")
                else:
                    self.logger.warning(f"位置 {position} 未获得数据")
                
                # 位置间休息
                if i < len(positions) - 1:
                    self.logger.info("10秒后继续下一个位置...")
                    time.sleep(10)
            
            self.logger.info(f"校准数据收集完成，总样本数: {len(all_data)}")
            return all_data
            
        except Exception as e:
            self.logger.error(f"校准数据收集失败: {e}")
            return []
    
    def _analyze_accelerometer_calibration(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """分析加速度计校准"""
        try:
            if not data:
                return results
            
            # 提取加速度计数据
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            
            # 计算重力校准精度
            magnitudes = np.linalg.norm(accel_data, axis=1)
            
            results.gravity_calibration['measured_gravity_mean'] = np.mean(magnitudes)
            results.gravity_calibration['measured_gravity_std'] = np.std(magnitudes)
            results.gravity_calibration['gravity_error'] = results.gravity_calibration['measured_gravity_mean'] - self.gravity_reference
            results.gravity_calibration['gravity_error_percent'] = abs(results.gravity_calibration['gravity_error']) / self.gravity_reference * 100
            
            # 分析各轴偏置
            mean_accel = np.mean(accel_data, axis=0)
            results.accel_bias['bias_x'] = mean_accel[0]
            results.accel_bias['bias_y'] = mean_accel[1]
            results.accel_bias['bias_z'] = mean_accel[2]
            results.accel_bias['bias_magnitude'] = np.linalg.norm(mean_accel)
            
            # 分析比例因子（如果有多个姿态的数据）
            if hasattr(data[0], 'position_id'):
                results = self._analyze_scale_factors(results, accel_data, data, 'accelerometer')
            
            # 交叉轴耦合分析
            results = self._analyze_cross_axis_coupling(results, accel_data, 'accelerometer')
            
            # 加速度计非线性度
            results.accelerometer_calibration['nonlinearity'] = self._calculate_nonlinearity(accel_data)
            
            return results
            
        except Exception as e:
            self.logger.error(f"加速度计校准分析失败: {e}")
            return results
    
    def _analyze_gyroscope_calibration(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """分析陀螺仪校准"""
        try:
            if not data:
                return results
            
            # 提取陀螺仪数据
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])
            
            # 静态偏置分析
            mean_gyro = np.mean(gyro_data, axis=0)
            std_gyro = np.std(gyro_data, axis=0)
            
            results.gyro_bias['bias_x'] = mean_gyro[0]
            results.gyro_bias['bias_y'] = mean_gyro[1]
            results.gyro_bias['bias_z'] = mean_gyro[2]
            results.gyro_bias['bias_magnitude'] = np.linalg.norm(mean_gyro)
            
            # 偏置稳定性
            results.gyro_bias['bias_stability_x'] = std_gyro[0]
            results.gyro_bias['bias_stability_y'] = std_gyro[1]
            results.gyro_bias['bias_stability_z'] = std_gyro[2]
            
            # 噪声特性分析
            results = self._analyze_gyro_noise_characteristics(results, gyro_data)
            
            # 比例因子分析（如有动态数据）
            if hasattr(data[0], 'position_id'):
                results = self._analyze_scale_factors(results, gyro_data, data, 'gyroscope')
            
            # 交叉轴耦合
            results = self._analyze_cross_axis_coupling(results, gyro_data, 'gyroscope')
            
            # 角度随机游走
            results.gyro_noise_characteristics['angle_random_walk'] = self._calculate_angle_random_walk(gyro_data)
            
            return results
            
        except Exception as e:
            self.logger.error(f"陀螺仪校准分析失败: {e}")
            return results
    
    def _analyze_attitude_calibration(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """分析姿态校准"""
        try:
            if not data:
                return results
            
            # 提取四元数数据
            quat_data = np.array([[r.quaternion[0], r.quaternion[1], r.quaternion[2], r.quaternion[3]] for r in data])
            
            # 四元数一致性检查
            quat_magnitudes = np.linalg.norm(quat_data, axis=1)
            results.quaternion_consistency['magnitude_mean'] = np.mean(quat_magnitudes)
            results.quaternion_consistency['magnitude_std'] = np.std(quat_magnitudes)
            results.quaternion_consistency['magnitude_error'] = abs(results.quaternion_consistency['magnitude_mean'] - 1.0)
            
            # 欧拉角精度分析
            euler_data = []
            for quat in quat_data:
                roll, pitch, yaw = self.imu_config.quaternion_to_euler(tuple(quat))
                euler_data.append([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])
            
            euler_array = np.array(euler_data)
            
            # 计算欧拉角稳定性
            results.euler_accuracy['roll_std'] = np.std(euler_array[:, 0])
            results.euler_accuracy['pitch_std'] = np.std(euler_array[:, 1])
            results.euler_accuracy['yaw_std'] = np.std(euler_array[:, 2])
            
            # 如果有位置标识，分析不同姿态的精度
            if hasattr(data[0], 'position_id'):
                results = self._analyze_attitude_accuracy_by_position(results, euler_array, data)
            
            # 姿态融合质量
            results.attitude_calibration['fusion_quality'] = self._calculate_attitude_fusion_quality(quat_data, euler_array)
            
            return results
            
        except Exception as e:
            self.logger.error(f"姿态校准分析失败: {e}")
            return results
    
    def _analyze_temperature_compensation(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """分析温度补偿"""
        try:
            # 提取温度数据
            temp_data = [r.temperature for r in data if r.temperature is not None]
            
            if not temp_data:
                self.logger.info("无温度数据，跳过温度补偿分析")
                return results
            
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])
            
            # 温度范围
            results.temperature_compensation['temp_range'] = max(temp_data) - min(temp_data)
            results.temperature_compensation['temp_mean'] = statistics.mean(temp_data)
            results.temperature_compensation['temp_std'] = statistics.stdev(temp_data) if len(temp_data) > 1 else 0
            
            # 温度系数分析（简化）
            if results.temperature_compensation['temp_range'] > 5:  # 温度变化超过5度时分析
                temp_array = np.array(temp_data)
                
                # 加速度计温度系数
                for i, axis in enumerate(['x', 'y', 'z']):
                    temp_coeff = np.polyfit(temp_array, accel_data[:, i], 1)[0]  # 线性拟合斜率
                    results.temperature_compensation[f'accel_temp_coeff_{axis}'] = temp_coeff
                
                # 陀螺仪温度系数
                for i, axis in enumerate(['x', 'y', 'z']):
                    temp_coeff = np.polyfit(temp_array, gyro_data[:, i], 1)[0]
                    results.temperature_compensation[f'gyro_temp_coeff_{axis}'] = temp_coeff
            else:
                self.logger.info("温度变化范围不足，无法分析温度系数")
            
            return results
            
        except Exception as e:
            self.logger.error(f"温度补偿分析失败: {e}")
            return results
    
    def _analyze_scale_factors(self, results: CalibrationResults, sensor_data: np.ndarray, 
                             data: List[IMUReading], sensor_type: str) -> CalibrationResults:
        """分析比例因子"""
        try:
            # 按位置分组数据
            position_groups = {}
            for i, reading in enumerate(data):
                if hasattr(reading, 'position_id'):
                    pos_id = reading.position_id
                    if pos_id not in position_groups:
                        position_groups[pos_id] = []
                    position_groups[pos_id].append(i)
            
            if len(position_groups) < 2:
                return results
            
            # 计算每个位置的平均值
            position_means = {}
            for pos_id, indices in position_groups.items():
                position_means[pos_id] = np.mean(sensor_data[indices], axis=0)
            
            # 分析比例因子（简化分析）
            scale_factors = []
            for axis in range(3):
                axis_values = [mean[axis] for mean in position_means.values()]
                if len(set(np.sign(axis_values))) > 1:  # 有正负值变化
                    scale_factor = np.std(axis_values) / np.mean(np.abs(axis_values))
                    scale_factors.append(scale_factor)
            
            if scale_factors:
                key_prefix = 'accel_scale_factor' if sensor_type == 'accelerometer' else 'gyro_scale_factor'
                target_dict = results.accel_scale_factor if sensor_type == 'accelerometer' else results.gyro_scale_factor
                
                target_dict['mean_scale_factor'] = np.mean(scale_factors)
                target_dict['scale_factor_std'] = np.std(scale_factors) if len(scale_factors) > 1 else 0
                target_dict['scale_factor_error_percent'] = abs(target_dict['mean_scale_factor'] - 1.0) * 100
            
            return results
            
        except Exception as e:
            self.logger.error(f"{sensor_type}比例因子分析失败: {e}")
            return results
    
    def _analyze_cross_axis_coupling(self, results: CalibrationResults, sensor_data: np.ndarray, 
                                   sensor_type: str) -> CalibrationResults:
        """分析交叉轴耦合"""
        try:
            # 计算交叉轴相关性
            correlation_matrix = np.corrcoef(sensor_data.T)
            
            key_prefix = 'accel_cross_axis' if sensor_type == 'accelerometer' else 'gyro_cross_axis' 
            target_dict = results.accel_cross_axis if sensor_type == 'accelerometer' else getattr(results, 'gyro_cross_axis', {})
            
            # 非对角线元素（交叉耦合）
            target_dict['xy_coupling'] = abs(correlation_matrix[0, 1])
            target_dict['xz_coupling'] = abs(correlation_matrix[0, 2])
            target_dict['yz_coupling'] = abs(correlation_matrix[1, 2])
            target_dict['max_coupling'] = max(target_dict['xy_coupling'], target_dict['xz_coupling'], target_dict['yz_coupling'])
            
            # 设置回结果
            if sensor_type == 'accelerometer':
                results.accel_cross_axis = target_dict
            else:
                setattr(results, 'gyro_cross_axis', target_dict)
            
            return results
            
        except Exception as e:
            self.logger.error(f"{sensor_type}交叉轴耦合分析失败: {e}")
            return results
    
    def _analyze_gyro_noise_characteristics(self, results: CalibrationResults, gyro_data: np.ndarray) -> CalibrationResults:
        """分析陀螺仪噪声特性"""
        try:
            # 白噪声密度
            for i, axis in enumerate(['x', 'y', 'z']):
                std_dev = np.std(gyro_data[:, i])
                # 假设采样率100Hz，白噪声密度 = 标准差 * sqrt(采样率)
                sampling_rate = self.imu_config.target_spec.sampling_rate_hz
                noise_density = std_dev * np.sqrt(sampling_rate)
                results.gyro_noise_characteristics[f'noise_density_{axis}'] = noise_density
            
            # 总体噪声水平
            total_noise = np.sqrt(np.sum([results.gyro_noise_characteristics[f'noise_density_{ax}']**2 for ax in ['x', 'y', 'z']]))
            results.gyro_noise_characteristics['total_noise_density'] = total_noise
            
            return results
            
        except Exception as e:
            self.logger.error(f"陀螺仪噪声特性分析失败: {e}")
            return results
    
    def _calculate_nonlinearity(self, data: np.ndarray) -> float:
        """计算非线性度"""
        try:
            # 简化的非线性度分析
            # 计算各轴数据的二阶多项式拟合残差
            nonlinearities = []
            
            for axis in range(data.shape[1]):
                axis_data = data[:, axis]
                x = np.arange(len(axis_data))
                
                # 线性拟合
                linear_fit = np.polyfit(x, axis_data, 1)
                linear_values = np.polyval(linear_fit, x)
                
                # 二次拟合
                quad_fit = np.polyfit(x, axis_data, 2)
                quad_values = np.polyval(quad_fit, x)
                
                # 非线性度 = 二次项贡献 / 总信号范围
                nonlinearity = np.std(quad_values - linear_values) / (np.max(axis_data) - np.min(axis_data))
                nonlinearities.append(nonlinearity)
            
            return np.mean(nonlinearities)
            
        except Exception as e:
            self.logger.error(f"非线性度计算失败: {e}")
            return 0.0
    
    def _calculate_angle_random_walk(self, gyro_data: np.ndarray) -> float:
        """计算角度随机游走"""
        try:
            # 简化的角度随机游走计算
            # ARW = 陀螺仪噪声标准差 / sqrt(积分时间)
            dt = 1.0 / self.imu_config.target_spec.sampling_rate_hz
            
            # 计算各轴的ARW
            arw_values = []
            for axis in range(gyro_data.shape[1]):
                noise_std = np.std(gyro_data[:, axis])
                arw = noise_std / np.sqrt(dt)  # 度/sqrt(小时)转换需要适当的单位转换
                arw_values.append(arw)
            
            return np.mean(arw_values)
            
        except Exception as e:
            self.logger.error(f"角度随机游走计算失败: {e}")
            return 0.0
    
    def _analyze_attitude_accuracy_by_position(self, results: CalibrationResults, euler_array: np.ndarray, 
                                             data: List[IMUReading]) -> CalibrationResults:
        """按位置分析姿态精度"""
        try:
            # 按位置分组
            position_groups = {}
            for i, reading in enumerate(data):
                if hasattr(reading, 'position_id'):
                    pos_id = reading.position_id
                    if pos_id not in position_groups:
                        position_groups[pos_id] = []
                    position_groups[pos_id].append(i)
            
            # 分析每个位置的姿态精度
            position_accuracies = []
            for pos_id, indices in position_groups.items():
                position_euler = euler_array[indices]
                
                # 计算该位置的姿态标准差
                roll_std = np.std(position_euler[:, 0])
                pitch_std = np.std(position_euler[:, 1])
                yaw_std = np.std(position_euler[:, 2])
                
                total_std = np.sqrt(roll_std**2 + pitch_std**2 + yaw_std**2)
                position_accuracies.append(total_std)
            
            results.euler_accuracy['position_consistency'] = np.mean(position_accuracies)
            results.euler_accuracy['worst_position_accuracy'] = max(position_accuracies)
            results.euler_accuracy['best_position_accuracy'] = min(position_accuracies)
            
            return results
            
        except Exception as e:
            self.logger.error(f"按位置姿态精度分析失败: {e}")
            return results
    
    def _calculate_attitude_fusion_quality(self, quat_data: np.ndarray, euler_data: np.ndarray) -> float:
        """计算姿态融合质量"""
        try:
            # 计算四元数一致性
            quat_magnitudes = np.linalg.norm(quat_data, axis=1)
            magnitude_consistency = 1.0 - np.std(quat_magnitudes)
            
            # 计算欧拉角平滑性
            euler_derivatives = np.abs(np.diff(euler_data, axis=0))
            smoothness = 1.0 / (1.0 + np.mean(euler_derivatives))
            
            # 综合质量分数
            fusion_quality = 0.7 * magnitude_consistency + 0.3 * smoothness
            
            return max(0, min(1, fusion_quality))
            
        except Exception as e:
            self.logger.error(f"姿态融合质量计算失败: {e}")
            return 0.0
    
    def _calculate_calibration_quality(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """计算校准质量指标"""
        try:
            quality_scores = []
            
            # 重力精度质量
            gravity_error = results.gravity_calibration.get('gravity_error_percent', 100)
            gravity_quality = max(0, 1 - gravity_error / 5.0)  # 5%误差为0分
            quality_scores.append(gravity_quality)
            
            # 陀螺仪偏置质量
            gyro_bias_mag = results.gyro_bias.get('bias_magnitude', 1.0)
            bias_quality = max(0, 1 - gyro_bias_mag / 0.1)  # 0.1 rad/s偏置为0分
            quality_scores.append(bias_quality)
            
            # 四元数一致性质量
            quat_error = results.quaternion_consistency.get('magnitude_error', 1.0)
            quat_quality = max(0, 1 - quat_error / 0.1)  # 0.1误差为0分
            quality_scores.append(quat_quality)
            
            # 计算总体质量分数
            overall_quality = np.mean(quality_scores)
            
            results.calibration_quality['overall_score'] = overall_quality
            results.calibration_quality['gravity_quality'] = gravity_quality
            results.calibration_quality['bias_quality'] = bias_quality
            results.calibration_quality['quaternion_quality'] = quat_quality
            
            # 质量等级
            if overall_quality >= 0.9:
                results.calibration_quality['quality_grade'] = "EXCELLENT"
            elif overall_quality >= 0.8:
                results.calibration_quality['quality_grade'] = "GOOD"
            elif overall_quality >= 0.6:
                results.calibration_quality['quality_grade'] = "FAIR"
            else:
                results.calibration_quality['quality_grade'] = "POOR"
            
            return results
            
        except Exception as e:
            self.logger.error(f"校准质量计算失败: {e}")
            return results
    
    def _evaluate_calibration_results(self, results: CalibrationResults):
        """评估校准结果"""
        try:
            pass_criteria = {}
            recommendations = []
            
            # 评估重力精度
            gravity_error_threshold = self.quality_thresholds.get("accuracy", {}).get("gravity_error_max_percent", 2)
            gravity_pass = results.gravity_calibration.get('gravity_error_percent', 100) < gravity_error_threshold
            pass_criteria['gravity_accuracy'] = gravity_pass
            
            if not gravity_pass:
                recommendations.append("重力测量精度不符合要求，需要重新校准加速度计")
            
            # 评估陀螺仪偏置
            gyro_bias_threshold = 0.05  # 0.05 rad/s
            gyro_bias_pass = results.gyro_bias.get('bias_magnitude', 1.0) < gyro_bias_threshold
            pass_criteria['gyroscope_bias'] = gyro_bias_pass
            
            if not gyro_bias_pass:
                recommendations.append("陀螺仪偏置过大，需要偏置校准")
            
            # 评估四元数一致性
            quat_error_threshold = 0.01
            quat_pass = results.quaternion_consistency.get('magnitude_error', 1.0) < quat_error_threshold
            pass_criteria['quaternion_consistency'] = quat_pass
            
            if not quat_pass:
                recommendations.append("四元数一致性不足，检查姿态融合算法")
            
            # 评估交叉轴耦合
            max_coupling_threshold = 0.1  # 10%
            accel_coupling_pass = results.accel_cross_axis.get('max_coupling', 1.0) < max_coupling_threshold
            pass_criteria['accelerometer_coupling'] = accel_coupling_pass
            
            if not accel_coupling_pass:
                recommendations.append("加速度计交叉轴耦合过高，需要校准修正")
            
            # 评估整体校准质量
            overall_quality = results.calibration_quality.get('overall_score', 0)
            quality_pass = overall_quality > 0.7  # 70%质量阈值
            pass_criteria['overall_quality'] = quality_pass
            
            if not quality_pass:
                recommendations.append("整体校准质量不足，建议进行全面重新校准")
            
            # 确定总体状态
            critical_tests = ['gravity_accuracy', 'gyroscope_bias', 'quaternion_consistency']
            critical_passed = all(pass_criteria.get(test, False) for test in critical_tests)
            all_passed = all(pass_criteria.values())
            
            if all_passed:
                results.test_status = "PASS"
            elif critical_passed:
                results.test_status = "WARNING"
            else:
                results.test_status = "FAIL"
            
            if not recommendations:
                recommendations.append("所有校准指标均符合要求")
            
            results.pass_criteria = pass_criteria
            results.recommendations = recommendations
            
        except Exception as e:
            self.logger.error(f"校准结果评估失败: {e}")
            results.test_status = "ERROR"
            results.recommendations = [f"评估失败: {str(e)}"] 