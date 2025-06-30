#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/dynamic_tester.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU动态响应测试

import time
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
import threading

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector, CollectionMetrics

@dataclass
class DynamicTestResults:
    """动态测试结果"""
    test_duration: float = 0.0
    test_type: str = ""
    sample_count: int = 0
    valid_samples: int = 0
    
    # 响应性指标
    response_time_ms: float = 0.0
    rise_time_ms: float = 0.0
    settling_time_ms: float = 0.0
    overshoot_percent: float = 0.0
    
    # 动态精度指标
    tracking_accuracy: Dict[str, float] = field(default_factory=dict)
    dynamic_range: Dict[str, float] = field(default_factory=dict)
    linearity: Dict[str, float] = field(default_factory=dict)
    
    # 频率响应
    frequency_response: Dict[str, float] = field(default_factory=dict)
    
    # 测试评估
    test_status: str = "UNKNOWN"
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)

class IMUDynamicTester:
    """IMU动态测试器"""
    
    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        初始化动态测试器
        
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
        self.test_config = config.get("test_parameters", {}).get("dynamic_test", {})
        self.quality_thresholds = config.get("quality_thresholds", {})
        
        # 测试参数
        self.test_duration = self.test_config.get("duration_seconds", 120)
        self.response_tests = self.test_config.get("response_tests", ["pitch_test", "roll_test", "yaw_test", "translation_test"])
        self.response_threshold_ms = self.test_config.get("response_threshold_ms", 50)
        
    def run_dynamic_response_test(self, test_type: str = "comprehensive") -> Dict[str, DynamicTestResults]:
        """
        运行动态响应测试
        
        Args:
            test_type: 测试类型 ("comprehensive", "quick", "specific")
            
        Returns:
            Dict[str, DynamicTestResults]: 各测试类型的结果
        """
        self.logger.info("=" * 50)
        self.logger.info("开始IMU动态响应测试")
        self.logger.info("=" * 50)
        
        all_results = {}
        
        try:
            # 确保IMU已初始化
            if not self.imu_config.is_initialized:
                self.logger.error("IMU未初始化，无法进行动态测试")
                return all_results
            
            if test_type == "comprehensive":
                test_list = self.response_tests
            elif test_type == "quick":
                test_list = ["pitch_test", "translation_test"]  # 快速测试
            else:
                test_list = [test_type] if test_type in self.response_tests else ["pitch_test"]
            
            for test_name in test_list:
                self.logger.info(f"\n运行 {test_name}...")
                result = self._run_single_dynamic_test(test_name)
                all_results[test_name] = result
                
                # 短暂休息
                if len(test_list) > 1:
                    self.logger.info("5秒后继续下一个测试...")
                    time.sleep(5)
            
            # 生成综合评估
            overall_result = self._generate_overall_assessment(all_results)
            all_results["overall"] = overall_result
            
            self.logger.info("=" * 50)
            self.logger.info("动态测试完成")
            self.logger.info("=" * 50)
            
            return all_results
            
        except Exception as e:
            self.logger.error(f"动态测试失败: {e}")
            return all_results
    
    def _run_single_dynamic_test(self, test_name: str) -> DynamicTestResults:
        """运行单个动态测试"""
        result = DynamicTestResults()
        result.test_type = test_name
        
        try:
            self.logger.info(f"准备进行 {test_name} 测试")
            
            # 获取测试指导信息
            test_instructions = self._get_test_instructions(test_name)
            self.logger.info(f"测试指导: {test_instructions}")
            
            # 等待用户准备
            self.logger.info("10秒后开始测试，请准备...")
            time.sleep(10)
            
            # 开始数据采集
            self.logger.info("开始数据采集...")
            test_duration = min(60, self.test_duration // len(self.response_tests))  # 单个测试时间
            
            success = self.data_collector.start_collection(test_duration)
            
            if not success:
                self.logger.error("无法启动数据采集")
                result.test_status = "FAIL"
                return result
            
            # 监控测试进度
            self._monitor_test_progress(test_duration, test_name)
            
            # 停止采集并获取数据
            collection_metrics = self.data_collector.stop_collection()
            collected_data = self.data_collector.get_collected_data()
            
            if not collected_data:
                self.logger.error("未采集到数据")
                result.test_status = "FAIL"
                return result
            
            self.logger.info(f"{test_name} 采集完成，共获得 {len(collected_data)} 个样本")
            
            # 分析动态响应
            result = self._analyze_dynamic_response(collected_data, collection_metrics, test_name)
            
            # 评估测试结果
            self._evaluate_dynamic_results(result)
            
            return result
            
        except Exception as e:
            self.logger.error(f"{test_name} 测试失败: {e}")
            result.test_status = "ERROR"
            return result
    
    def _get_test_instructions(self, test_name: str) -> str:
        """获取测试指导"""
        instructions = {
            "pitch_test": "请缓慢前后倾斜机器人（pitch轴旋转），然后快速回到水平位置",
            "roll_test": "请缓慢左右倾斜机器人（roll轴旋转），然后快速回到水平位置",
            "yaw_test": "请缓慢左右转动机器人（yaw轴旋转），然后快速回到原始方向",
            "translation_test": "请平稳地前后、左右移动机器人，避免旋转"
        }
        
        return instructions.get(test_name, "按照标准动作进行测试")
    
    def _monitor_test_progress(self, duration: float, test_name: str):
        """监控测试进度"""
        start_time = time.time()
        
        while self.data_collector.is_collecting:
            elapsed = time.time() - start_time
            
            if elapsed >= duration + 10:  # 额外10秒超时
                self.logger.warning(f"{test_name} 测试超时，强制停止")
                break
            
            # 显示进度和实时指标
            if int(elapsed) % 10 == 0:
                metrics = self.data_collector.get_real_time_metrics()
                self.logger.info(f"{test_name} 进度: {elapsed:.1f}/{duration}秒, 采样率: {metrics.get('current_fps', 0):.1f}Hz")
            
            time.sleep(1)
    
    def _analyze_dynamic_response(self, data: List[IMUReading], collection_metrics: CollectionMetrics, test_name: str) -> DynamicTestResults:
        """分析动态响应数据"""
        result = DynamicTestResults()
        result.test_type = test_name
        result.test_duration = collection_metrics.collection_duration
        result.sample_count = len(data)
        result.valid_samples = collection_metrics.valid_samples
        
        try:
            # 提取传感器数据
            timestamps = np.array([r.timestamp for r in data])
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])
            quat_data = np.array([[r.quaternion[0], r.quaternion[1], r.quaternion[2], r.quaternion[3]] for r in data])
            
            # 转换为欧拉角
            euler_data = []
            for quat in quat_data:
                roll, pitch, yaw = self.imu_config.quaternion_to_euler(tuple(quat))
                euler_data.append([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])
            euler_data = np.array(euler_data)
            
            # 根据测试类型分析不同的响应
            if test_name == "pitch_test":
                result = self._analyze_rotation_response(result, timestamps, euler_data[:, 1], gyro_data[:, 1], "pitch")
            elif test_name == "roll_test":
                result = self._analyze_rotation_response(result, timestamps, euler_data[:, 0], gyro_data[:, 0], "roll")
            elif test_name == "yaw_test":
                result = self._analyze_rotation_response(result, timestamps, euler_data[:, 2], gyro_data[:, 2], "yaw")
            elif test_name == "translation_test":
                result = self._analyze_translation_response(result, timestamps, accel_data)
            
            # 通用动态分析
            result = self._analyze_general_dynamics(result, timestamps, accel_data, gyro_data, euler_data)
            
            return result
            
        except Exception as e:
            self.logger.error(f"动态响应分析失败: {e}")
            result.test_status = "ERROR"
            return result
    
    def _analyze_rotation_response(self, result: DynamicTestResults, timestamps: np.ndarray, 
                                 angle_data: np.ndarray, gyro_data: np.ndarray, axis: str) -> DynamicTestResults:
        """分析旋转响应"""
        try:
            # 检测运动事件
            motion_events = self._detect_motion_events(angle_data, threshold=5.0)  # 5度阈值
            
            if motion_events:
                # 分析最大的运动事件
                largest_event = max(motion_events, key=lambda x: x['magnitude'])
                
                # 计算响应时间
                gyro_response_time = self._calculate_gyro_response_time(
                    timestamps, gyro_data, largest_event['start_idx'], largest_event['end_idx']
                )
                
                result.response_time_ms = gyro_response_time * 1000
                
                # 分析阶跃响应特性
                step_response = self._analyze_step_response(
                    timestamps[largest_event['start_idx']:largest_event['end_idx']], 
                    angle_data[largest_event['start_idx']:largest_event['end_idx']]
                )
                
                result.rise_time_ms = step_response.get('rise_time', 0) * 1000
                result.settling_time_ms = step_response.get('settling_time', 0) * 1000
                result.overshoot_percent = step_response.get('overshoot_percent', 0)
                
                # 跟踪精度
                result.tracking_accuracy[f'{axis}_correlation'] = self._calculate_tracking_correlation(angle_data, gyro_data, timestamps)
                result.tracking_accuracy[f'{axis}_phase_delay_ms'] = self._calculate_phase_delay(angle_data, gyro_data, timestamps) * 1000
            
            # 动态范围
            result.dynamic_range[f'{axis}_angle_range'] = np.max(angle_data) - np.min(angle_data)
            result.dynamic_range[f'{axis}_gyro_range'] = np.max(gyro_data) - np.min(gyro_data)
            
            return result
            
        except Exception as e:
            self.logger.error(f"旋转响应分析失败: {e}")
            return result
    
    def _analyze_translation_response(self, result: DynamicTestResults, timestamps: np.ndarray, 
                                    accel_data: np.ndarray) -> DynamicTestResults:
        """分析平移响应"""
        try:
            # 计算加速度幅值
            accel_magnitude = np.linalg.norm(accel_data, axis=1)
            
            # 检测加速度事件
            baseline = np.median(accel_magnitude)
            motion_threshold = baseline * 0.1  # 10%变化阈值
            
            motion_events = self._detect_acceleration_events(accel_magnitude, motion_threshold)
            
            if motion_events:
                # 分析最大的加速度事件
                largest_event = max(motion_events, key=lambda x: x['magnitude'])
                
                # 计算响应时间（加速度变化到稳定的时间）
                response_time = self._calculate_acceleration_response_time(
                    timestamps, accel_magnitude, largest_event['start_idx'], largest_event['end_idx']
                )
                
                result.response_time_ms = response_time * 1000
            
            # 平移动态范围
            for i, axis in enumerate(['x', 'y', 'z']):
                result.dynamic_range[f'accel_{axis}_range'] = np.max(accel_data[:, i]) - np.min(accel_data[:, i])
            
            result.dynamic_range['accel_magnitude_range'] = np.max(accel_magnitude) - np.min(accel_magnitude)
            
            # 线性度分析（加速度与期望的线性关系）
            result.linearity['acceleration_linearity'] = self._calculate_acceleration_linearity(accel_data)
            
            return result
            
        except Exception as e:
            self.logger.error(f"平移响应分析失败: {e}")
            return result
    
    def _analyze_general_dynamics(self, result: DynamicTestResults, timestamps: np.ndarray, 
                                accel_data: np.ndarray, gyro_data: np.ndarray, euler_data: np.ndarray) -> DynamicTestResults:
        """通用动态分析"""
        try:
            # 频率响应分析
            sampling_rate = len(timestamps) / (timestamps[-1] - timestamps[0])
            
            # 分析各轴的频率响应
            for i, axis in enumerate(['x', 'y', 'z']):
                # 陀螺仪频率响应
                gyro_freq_response = self._analyze_frequency_response(gyro_data[:, i], sampling_rate)
                result.frequency_response[f'gyro_{axis}_bandwidth_hz'] = gyro_freq_response.get('bandwidth', 0)
                result.frequency_response[f'gyro_{axis}_peak_freq_hz'] = gyro_freq_response.get('peak_frequency', 0)
                
                # 加速度计频率响应
                accel_freq_response = self._analyze_frequency_response(accel_data[:, i], sampling_rate)
                result.frequency_response[f'accel_{axis}_bandwidth_hz'] = accel_freq_response.get('bandwidth', 0)
                result.frequency_response[f'accel_{axis}_peak_freq_hz'] = accel_freq_response.get('peak_frequency', 0)
            
            return result
            
        except Exception as e:
            self.logger.error(f"通用动态分析失败: {e}")
            return result
    
    def _detect_motion_events(self, data: np.ndarray, threshold: float) -> List[Dict[str, Any]]:
        """检测运动事件"""
        try:
            events = []
            
            # 计算变化率
            diff = np.abs(np.diff(data))
            motion_mask = diff > threshold
            
            # 查找连续的运动段
            in_motion = False
            start_idx = 0
            
            for i, is_moving in enumerate(motion_mask):
                if is_moving and not in_motion:
                    # 运动开始
                    start_idx = i
                    in_motion = True
                elif not is_moving and in_motion:
                    # 运动结束
                    end_idx = i
                    magnitude = np.max(np.abs(data[start_idx:end_idx])) - np.min(np.abs(data[start_idx:end_idx]))
                    
                    events.append({
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'magnitude': magnitude,
                        'duration': end_idx - start_idx
                    })
                    
                    in_motion = False
            
            return events
            
        except Exception as e:
            self.logger.error(f"运动事件检测失败: {e}")
            return []
    
    def _detect_acceleration_events(self, data: np.ndarray, threshold: float) -> List[Dict[str, Any]]:
        """检测加速度事件"""
        try:
            events = []
            baseline = np.median(data)
            
            # 检测显著偏离基线的区域
            deviation = np.abs(data - baseline)
            event_mask = deviation > threshold
            
            # 查找连续的事件段
            in_event = False
            start_idx = 0
            
            for i, is_event in enumerate(event_mask):
                if is_event and not in_event:
                    start_idx = i
                    in_event = True
                elif not is_event and in_event:
                    end_idx = i
                    magnitude = np.max(data[start_idx:end_idx]) - np.min(data[start_idx:end_idx])
                    
                    events.append({
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'magnitude': magnitude,
                        'duration': end_idx - start_idx
                    })
                    
                    in_event = False
            
            return events
            
        except Exception as e:
            self.logger.error(f"加速度事件检测失败: {e}")
            return []
    
    def _calculate_gyro_response_time(self, timestamps: np.ndarray, gyro_data: np.ndarray, 
                                    start_idx: int, end_idx: int) -> float:
        """计算陀螺仪响应时间"""
        try:
            if end_idx <= start_idx:
                return 0.0
            
            # 找到陀螺仪信号开始变化的时间点
            segment = gyro_data[start_idx:end_idx]
            baseline = np.mean(gyro_data[max(0, start_idx-10):start_idx])
            
            # 找到信号达到变化幅度10%的时间点
            peak_value = np.max(np.abs(segment))
            threshold = baseline + 0.1 * (peak_value - baseline)
            
            response_idx = start_idx
            for i, value in enumerate(segment):
                if abs(value) > threshold:
                    response_idx = start_idx + i
                    break
            
            response_time = timestamps[response_idx] - timestamps[start_idx]
            return max(0, response_time)
            
        except Exception as e:
            self.logger.error(f"陀螺仪响应时间计算失败: {e}")
            return 0.0
    
    def _calculate_acceleration_response_time(self, timestamps: np.ndarray, accel_data: np.ndarray, 
                                           start_idx: int, end_idx: int) -> float:
        """计算加速度响应时间"""
        try:
            if end_idx <= start_idx:
                return 0.0
            
            # 简化的响应时间计算
            segment_duration = timestamps[end_idx] - timestamps[start_idx]
            
            # 假设响应时间为事件持续时间的10%（经验值）
            response_time = segment_duration * 0.1
            
            return response_time
            
        except Exception as e:
            self.logger.error(f"加速度响应时间计算失败: {e}")
            return 0.0
    
    def _analyze_step_response(self, timestamps: np.ndarray, data: np.ndarray) -> Dict[str, float]:
        """分析阶跃响应特性"""
        try:
            if len(data) < 10:
                return {}
            
            # 简化的阶跃响应分析
            initial_value = np.mean(data[:5])
            final_value = np.mean(data[-5:])
            step_size = abs(final_value - initial_value)
            
            if step_size < 1e-6:
                return {}
            
            # 上升时间（10%到90%）
            threshold_10 = initial_value + 0.1 * step_size
            threshold_90 = initial_value + 0.9 * step_size
            
            idx_10 = np.argmax(np.abs(data - initial_value) >= 0.1 * step_size)
            idx_90 = np.argmax(np.abs(data - initial_value) >= 0.9 * step_size)
            
            rise_time = timestamps[idx_90] - timestamps[idx_10] if idx_90 > idx_10 else 0
            
            # 超调量
            if final_value > initial_value:
                overshoot = np.max(data) - final_value
            else:
                overshoot = final_value - np.min(data)
            
            overshoot_percent = (overshoot / step_size) * 100 if step_size > 0 else 0
            
            # 稳定时间（简化为数据长度的80%）
            settling_time = (timestamps[-1] - timestamps[0]) * 0.8
            
            return {
                'rise_time': rise_time,
                'settling_time': settling_time,
                'overshoot_percent': overshoot_percent
            }
            
        except Exception as e:
            self.logger.error(f"阶跃响应分析失败: {e}")
            return {}
    
    def _calculate_tracking_correlation(self, angle_data: np.ndarray, gyro_data: np.ndarray, 
                                      timestamps: np.ndarray) -> float:
        """计算跟踪相关性"""
        try:
            # 计算角度的数值导数
            dt = np.mean(np.diff(timestamps))
            angle_rate = np.gradient(angle_data, dt)
            
            # 转换为相同单位（角度/秒 vs 弧度/秒）
            gyro_deg = np.degrees(gyro_data)
            
            # 计算相关系数
            correlation = np.corrcoef(angle_rate, gyro_deg)[0, 1]
            
            return correlation if not np.isnan(correlation) else 0.0
            
        except Exception as e:
            self.logger.error(f"跟踪相关性计算失败: {e}")
            return 0.0
    
    def _calculate_phase_delay(self, angle_data: np.ndarray, gyro_data: np.ndarray, 
                             timestamps: np.ndarray) -> float:
        """计算相位延迟"""
        try:
            # 简化的互相关分析
            dt = np.mean(np.diff(timestamps))
            angle_rate = np.gradient(angle_data, dt)
            gyro_deg = np.degrees(gyro_data)
            
            # 计算互相关
            correlation = np.correlate(angle_rate, gyro_deg, mode='full')
            delay_samples = np.argmax(correlation) - len(gyro_deg) + 1
            delay_time = delay_samples * dt
            
            return delay_time
            
        except Exception as e:
            self.logger.error(f"相位延迟计算失败: {e}")
            return 0.0
    
    def _calculate_acceleration_linearity(self, accel_data: np.ndarray) -> float:
        """计算加速度线性度"""
        try:
            # 简化的线性度分析：计算各轴之间的线性相关性
            correlations = []
            
            for i in range(3):
                for j in range(i+1, 3):
                    corr = np.corrcoef(accel_data[:, i], accel_data[:, j])[0, 1]
                    if not np.isnan(corr):
                        correlations.append(abs(corr))
            
            # 返回平均线性度
            return np.mean(correlations) if correlations else 0.0
            
        except Exception as e:
            self.logger.error(f"加速度线性度计算失败: {e}")
            return 0.0
    
    def _analyze_frequency_response(self, data: np.ndarray, sampling_rate: float) -> Dict[str, float]:
        """分析频率响应"""
        try:
            # 简化的频率分析
            fft = np.fft.fft(data)
            freqs = np.fft.fftfreq(len(data), 1/sampling_rate)
            
            # 只考虑正频率
            positive_freqs = freqs[:len(freqs)//2]
            magnitude = np.abs(fft[:len(fft)//2])
            
            # 找到峰值频率
            peak_idx = np.argmax(magnitude)
            peak_frequency = positive_freqs[peak_idx]
            
            # 估算带宽（-3dB点）
            peak_magnitude = magnitude[peak_idx]
            threshold = peak_magnitude * 0.707  # -3dB
            
            bandwidth_indices = np.where(magnitude >= threshold)[0]
            if len(bandwidth_indices) > 1:
                bandwidth = positive_freqs[bandwidth_indices[-1]] - positive_freqs[bandwidth_indices[0]]
            else:
                bandwidth = 0
            
            return {
                'peak_frequency': peak_frequency,
                'bandwidth': bandwidth,
                'peak_magnitude': peak_magnitude
            }
            
        except Exception as e:
            self.logger.error(f"频率响应分析失败: {e}")
            return {}
    
    def _generate_overall_assessment(self, all_results: Dict[str, DynamicTestResults]) -> DynamicTestResults:
        """生成综合评估"""
        overall = DynamicTestResults()
        overall.test_type = "overall_assessment"
        
        try:
            # 收集所有测试的状态
            statuses = [result.test_status for result in all_results.values()]
            
            # 确定整体状态
            if all(status == "PASS" for status in statuses):
                overall.test_status = "PASS"
            elif any(status == "FAIL" for status in statuses):
                overall.test_status = "FAIL"
            elif any(status == "WARNING" for status in statuses):
                overall.test_status = "WARNING"
            else:
                overall.test_status = "UNKNOWN"
            
            # 汇总响应时间
            response_times = [result.response_time_ms for result in all_results.values() if result.response_time_ms > 0]
            if response_times:
                overall.response_time_ms = statistics.mean(response_times)
            
            # 汇总建议
            all_recommendations = []
            for result in all_results.values():
                all_recommendations.extend(result.recommendations)
            
            overall.recommendations = list(set(all_recommendations))  # 去重
            
            if not overall.recommendations:
                overall.recommendations = ["所有动态测试均符合要求"]
            
            return overall
            
        except Exception as e:
            self.logger.error(f"综合评估生成失败: {e}")
            overall.test_status = "ERROR"
            return overall
    
    def _evaluate_dynamic_results(self, result: DynamicTestResults):
        """评估动态测试结果"""
        try:
            pass_criteria = {}
            recommendations = []
            
            # 评估响应时间
            response_time_pass = result.response_time_ms <= self.response_threshold_ms
            pass_criteria['response_time'] = response_time_pass
            
            if not response_time_pass:
                recommendations.append(f"响应时间过长({result.response_time_ms:.1f}ms)，检查传感器带宽和滤波设置")
            
            # 评估跟踪精度
            tracking_correlations = [v for k, v in result.tracking_accuracy.items() if 'correlation' in k]
            if tracking_correlations:
                avg_correlation = statistics.mean(tracking_correlations)
                tracking_pass = avg_correlation > 0.8  # 80%相关性阈值
                pass_criteria['tracking_accuracy'] = tracking_pass
                
                if not tracking_pass:
                    recommendations.append("跟踪精度不足，检查传感器校准和数据融合算法")
            
            # 评估动态范围
            angle_ranges = [v for k, v in result.dynamic_range.items() if 'angle' in k]
            if angle_ranges:
                max_range = max(angle_ranges)
                range_pass = max_range > 10  # 至少10度动态范围
                pass_criteria['dynamic_range'] = range_pass
                
                if not range_pass:
                    recommendations.append("动态范围不足，测试动作幅度可能不够")
            
            # 确定总体状态
            if all(pass_criteria.values()):
                result.test_status = "PASS"
            elif any(pass_criteria.values()):
                result.test_status = "WARNING"
            else:
                result.test_status = "FAIL"
            
            if not recommendations:
                recommendations.append(f"{result.test_type} 测试指标符合要求")
            
            result.pass_criteria = pass_criteria
            result.recommendations = recommendations
            
        except Exception as e:
            self.logger.error(f"评估动态测试结果失败: {e}")
            result.test_status = "ERROR"
            result.recommendations = [f"评估失败: {str(e)}"] 