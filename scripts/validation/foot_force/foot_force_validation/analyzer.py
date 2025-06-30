#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/analyzer.py
# Generated: 2025-06-27 14:25:00 CST
# Purpose: Unitree Go2 足端力传感器数据分析器

import time
import numpy as np
import scipy.signal as signal
import scipy.stats as stats
from scipy.fft import fft, fftfreq
import logging
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass, field
import json
from pathlib import Path
from datetime import datetime
import pandas as pd

from foot_force_config import FootForceConfig, FootForceReading
from data_collector import FootForceData

@dataclass
class StatisticalAnalysis:
    """统计分析结果"""
    mean: np.ndarray
    std: np.ndarray
    variance: np.ndarray
    min_val: np.ndarray
    max_val: np.ndarray
    median: np.ndarray
    q25: np.ndarray
    q75: np.ndarray
    skewness: np.ndarray
    kurtosis: np.ndarray
    coefficient_of_variation: np.ndarray

@dataclass
class FrequencyAnalysis:
    """频域分析结果"""
    frequencies: np.ndarray
    power_spectrum: np.ndarray
    dominant_frequency: float
    frequency_peaks: List[Tuple[float, float]]  # (频率, 幅值)
    spectral_centroid: float
    bandwidth: float
    signal_to_noise_ratio: float

@dataclass
class AnomalyDetection:
    """异常检测结果"""
    outlier_indices: List[int]
    outlier_scores: np.ndarray
    threshold: float
    anomaly_rate: float
    anomaly_types: Dict[str, List[int]]  # 异常类型: [索引列表]

@dataclass
class CorrelationAnalysis:
    """相关性分析结果"""
    correlation_matrix: np.ndarray
    cross_correlation: Dict[str, Dict[str, float]]
    lag_analysis: Dict[str, Dict[str, int]]  # 滞后分析
    coherence: Dict[str, Dict[str, float]]

@dataclass
class TrendAnalysis:
    """趋势分析结果"""
    linear_trend: np.ndarray  # 线性趋势系数
    trend_significance: np.ndarray  # 趋势显著性
    changepoint_indices: List[int]  # 变点检测
    stability_metrics: Dict[str, float]

class FootForceAnalyzer:
    """足端力传感器数据分析器"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化数据分析器
        
        Args:
            config: 配置字典
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # 分析参数
        self.analysis_config = config.get('analysis', {})
        self.sampling_rate = config.get('foot_force_config', {}).get('sampling_rate', 500.0)
        self.force_threshold = config.get('foot_force_config', {}).get('force_threshold', 5.0)
        
        # 异常检测参数
        self.outlier_threshold = self.analysis_config.get('outlier_threshold', 3.0)
        self.anomaly_window = self.analysis_config.get('anomaly_window_size', 100)
        
        # 频域分析参数
        self.frequency_bands = self.analysis_config.get('frequency_bands', {
            'dc': (0, 1),
            'low': (1, 10),
            'mid': (10, 50),
            'high': (50, 100)
        })
        
        self.logger.info("足端力传感器数据分析器初始化完成")
    
    def perform_statistical_analysis(self, data: List[FootForceData]) -> Dict[str, StatisticalAnalysis]:
        """
        执行统计分析
        
        Args:
            data: 足端力数据列表
            
        Returns:
            Dict[str, StatisticalAnalysis]: 各足端的统计分析结果
        """
        self.logger.info(f"开始统计分析，数据点数: {len(data)}")
        
        try:
            results = {}
            
            # 提取力数据
            forces_array = np.array([d.foot_forces for d in data])  # (samples, 4_feet, 3_axes)
            
            # 对每个足端进行统计分析
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                foot_forces = forces_array[:, i, :]  # (samples, 3_axes)
                
                # 计算各种统计指标
                analysis = StatisticalAnalysis(
                    mean=np.mean(foot_forces, axis=0),
                    std=np.std(foot_forces, axis=0),
                    variance=np.var(foot_forces, axis=0),
                    min_val=np.min(foot_forces, axis=0),
                    max_val=np.max(foot_forces, axis=0),
                    median=np.median(foot_forces, axis=0),
                    q25=np.percentile(foot_forces, 25, axis=0),
                    q75=np.percentile(foot_forces, 75, axis=0),
                    skewness=stats.skew(foot_forces, axis=0),
                    kurtosis=stats.kurtosis(foot_forces, axis=0),
                    coefficient_of_variation=np.std(foot_forces, axis=0) / (np.abs(np.mean(foot_forces, axis=0)) + 1e-8)
                )
                
                results[foot_label] = analysis
            
            # 整体统计分析
            total_forces = np.array([d.total_force for d in data])
            overall_analysis = StatisticalAnalysis(
                mean=np.array([np.mean(total_forces)]),
                std=np.array([np.std(total_forces)]),
                variance=np.array([np.var(total_forces)]),
                min_val=np.array([np.min(total_forces)]),
                max_val=np.array([np.max(total_forces)]),
                median=np.array([np.median(total_forces)]),
                q25=np.array([np.percentile(total_forces, 25)]),
                q75=np.array([np.percentile(total_forces, 75)]),
                skewness=np.array([stats.skew(total_forces)]),
                kurtosis=np.array([stats.kurtosis(total_forces)]),
                coefficient_of_variation=np.array([np.std(total_forces) / (np.abs(np.mean(total_forces)) + 1e-8)])
            )
            results['total'] = overall_analysis
            
            self.logger.info("统计分析完成")
            return results
            
        except Exception as e:
            self.logger.error(f"统计分析失败: {e}")
            return {}
    
    def perform_frequency_analysis(self, data: List[FootForceData]) -> Dict[str, FrequencyAnalysis]:
        """
        执行频域分析
        
        Args:
            data: 足端力数据列表
            
        Returns:
            Dict[str, FrequencyAnalysis]: 各足端各轴的频域分析结果
        """
        self.logger.info(f"开始频域分析，采样率: {self.sampling_rate}Hz")
        
        try:
            results = {}
            
            # 提取力数据
            forces_array = np.array([d.foot_forces for d in data])
            
            # 对每个足端的每个轴进行频域分析
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                foot_results = {}
                
                for axis_idx, axis_name in enumerate(['Fx', 'Fy', 'Fz']):
                    force_signal = forces_array[:, i, axis_idx]
                    
                    # FFT计算
                    n_samples = len(force_signal)
                    fft_values = fft(force_signal)
                    frequencies = fftfreq(n_samples, 1/self.sampling_rate)
                    
                    # 只保留正频率部分
                    positive_freq_idx = frequencies > 0
                    frequencies = frequencies[positive_freq_idx]
                    power_spectrum = np.abs(fft_values[positive_freq_idx])**2
                    
                    # 寻找主频率
                    dominant_freq_idx = np.argmax(power_spectrum)
                    dominant_frequency = frequencies[dominant_freq_idx]
                    
                    # 寻找频率峰值
                    peaks, peak_properties = signal.find_peaks(power_spectrum, height=np.max(power_spectrum)*0.1)
                    frequency_peaks = [(frequencies[peak], power_spectrum[peak]) for peak in peaks]
                    frequency_peaks.sort(key=lambda x: x[1], reverse=True)  # 按幅值排序
                    
                    # 计算频谱质心
                    spectral_centroid = np.sum(frequencies * power_spectrum) / np.sum(power_spectrum)
                    
                    # 计算带宽
                    total_power = np.sum(power_spectrum)
                    cumulative_power = np.cumsum(power_spectrum)
                    freq_80_percent = frequencies[np.where(cumulative_power >= 0.8 * total_power)[0][0]]
                    bandwidth = freq_80_percent
                    
                    # 计算信噪比
                    signal_power = np.mean(power_spectrum)
                    noise_floor = np.percentile(power_spectrum, 25)
                    snr = 10 * np.log10(signal_power / (noise_floor + 1e-12))
                    
                    analysis = FrequencyAnalysis(
                        frequencies=frequencies,
                        power_spectrum=power_spectrum,
                        dominant_frequency=dominant_frequency,
                        frequency_peaks=frequency_peaks[:5],  # 保留前5个峰值
                        spectral_centroid=spectral_centroid,
                        bandwidth=bandwidth,
                        signal_to_noise_ratio=snr
                    )
                    
                    foot_results[axis_name] = analysis
                
                results[foot_label] = foot_results
            
            self.logger.info("频域分析完成")
            return results
            
        except Exception as e:
            self.logger.error(f"频域分析失败: {e}")
            return {}
    
    def detect_anomalies(self, data: List[FootForceData]) -> Dict[str, AnomalyDetection]:
        """
        执行异常检测
        
        Args:
            data: 足端力数据列表
            
        Returns:
            Dict[str, AnomalyDetection]: 各足端的异常检测结果
        """
        self.logger.info(f"开始异常检测，阈值: {self.outlier_threshold}σ")
        
        try:
            results = {}
            
            # 提取力数据
            forces_array = np.array([d.foot_forces for d in data])
            
            # 对每个足端进行异常检测
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                foot_forces = forces_array[:, i, :]  # (samples, 3_axes)
                
                # 多种异常检测方法
                outlier_indices = set()
                anomaly_types = {
                    'statistical': [],
                    'iqr': [],
                    'isolation': [],
                    'sudden_change': []
                }
                
                # 1. 统计异常检测（Z-score）
                for axis_idx in range(3):
                    axis_data = foot_forces[:, axis_idx]
                    z_scores = np.abs(stats.zscore(axis_data))
                    stat_outliers = np.where(z_scores > self.outlier_threshold)[0]
                    outlier_indices.update(stat_outliers)
                    anomaly_types['statistical'].extend(stat_outliers.tolist())
                
                # 2. IQR异常检测
                for axis_idx in range(3):
                    axis_data = foot_forces[:, axis_idx]
                    q25, q75 = np.percentile(axis_data, [25, 75])
                    iqr = q75 - q25
                    lower_bound = q25 - 1.5 * iqr
                    upper_bound = q75 + 1.5 * iqr
                    iqr_outliers = np.where((axis_data < lower_bound) | (axis_data > upper_bound))[0]
                    outlier_indices.update(iqr_outliers)
                    anomaly_types['iqr'].extend(iqr_outliers.tolist())
                
                # 3. 突变检测
                for axis_idx in range(3):
                    axis_data = foot_forces[:, axis_idx]
                    diff = np.diff(axis_data)
                    diff_threshold = np.std(diff) * 3
                    sudden_changes = np.where(np.abs(diff) > diff_threshold)[0] + 1  # +1因为diff少一个元素
                    outlier_indices.update(sudden_changes)
                    anomaly_types['sudden_change'].extend(sudden_changes.tolist())
                
                # 计算异常评分
                outlier_list = sorted(list(outlier_indices))
                n_samples = len(foot_forces)
                outlier_scores = np.zeros(n_samples)
                
                for idx in outlier_list:
                    # 综合异常评分
                    score = 0
                    for axis_idx in range(3):
                        z_score = abs(stats.zscore(foot_forces[:, axis_idx])[idx])
                        score += z_score
                    outlier_scores[idx] = score / 3  # 平均评分
                
                analysis = AnomalyDetection(
                    outlier_indices=outlier_list,
                    outlier_scores=outlier_scores,
                    threshold=self.outlier_threshold,
                    anomaly_rate=len(outlier_list) / n_samples * 100,
                    anomaly_types=anomaly_types
                )
                
                results[foot_label] = analysis
            
            self.logger.info("异常检测完成")
            return results
            
        except Exception as e:
            self.logger.error(f"异常检测失败: {e}")
            return {}
    
    def perform_correlation_analysis(self, data: List[FootForceData]) -> CorrelationAnalysis:
        """
        执行相关性分析
        
        Args:
            data: 足端力数据列表
            
        Returns:
            CorrelationAnalysis: 相关性分析结果
        """
        self.logger.info("开始相关性分析")
        
        try:
            # 提取垂直力数据
            forces_array = np.array([d.foot_forces for d in data])
            vertical_forces = forces_array[:, :, 2]  # 所有足端的垂直力
            
            # 计算相关系数矩阵
            correlation_matrix = np.corrcoef(vertical_forces.T)
            
            # 计算各足端间的交叉相关
            cross_correlation = {}
            lag_analysis = {}
            coherence = {}
            
            for i, foot1 in enumerate(FootForceConfig.FOOT_LABELS):
                cross_correlation[foot1] = {}
                lag_analysis[foot1] = {}
                coherence[foot1] = {}
                
                for j, foot2 in enumerate(FootForceConfig.FOOT_LABELS):
                    if i != j:
                        signal1 = vertical_forces[:, i]
                        signal2 = vertical_forces[:, j]
                        
                        # 交叉相关
                        cross_corr = np.correlate(signal1, signal2, mode='full')
                        max_corr_idx = np.argmax(np.abs(cross_corr))
                        max_correlation = cross_corr[max_corr_idx] / (np.std(signal1) * np.std(signal2) * len(signal1))
                        lag = max_corr_idx - len(signal1) + 1
                        
                        cross_correlation[foot1][foot2] = float(max_correlation)
                        lag_analysis[foot1][foot2] = int(lag)
                        
                        # 相干性分析
                        frequencies, coherence_values = signal.coherence(signal1, signal2, 
                                                                       fs=self.sampling_rate, nperseg=256)
                        mean_coherence = np.mean(coherence_values)
                        coherence[foot1][foot2] = float(mean_coherence)
                    else:
                        cross_correlation[foot1][foot2] = 1.0
                        lag_analysis[foot1][foot2] = 0
                        coherence[foot1][foot2] = 1.0
            
            analysis = CorrelationAnalysis(
                correlation_matrix=correlation_matrix,
                cross_correlation=cross_correlation,
                lag_analysis=lag_analysis,
                coherence=coherence
            )
            
            self.logger.info("相关性分析完成")
            return analysis
            
        except Exception as e:
            self.logger.error(f"相关性分析失败: {e}")
            return CorrelationAnalysis(
                correlation_matrix=np.eye(4),
                cross_correlation={},
                lag_analysis={},
                coherence={}
            )
    
    def analyze_trends(self, data: List[FootForceData]) -> Dict[str, TrendAnalysis]:
        """
        执行趋势分析
        
        Args:
            data: 足端力数据列表
            
        Returns:
            Dict[str, TrendAnalysis]: 各足端的趋势分析结果
        """
        self.logger.info("开始趋势分析")
        
        try:
            results = {}
            
            # 时间序列
            timestamps = np.array([d.timestamp for d in data])
            time_relative = timestamps - timestamps[0]
            
            # 提取力数据
            forces_array = np.array([d.foot_forces for d in data])
            
            # 对每个足端进行趋势分析
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                foot_forces = forces_array[:, i, :]
                
                # 线性趋势分析
                linear_trend = np.zeros(3)
                trend_significance = np.zeros(3)
                
                for axis_idx in range(3):
                    axis_data = foot_forces[:, axis_idx]
                    
                    # 线性拟合
                    slope, intercept, r_value, p_value, std_err = stats.linregress(time_relative, axis_data)
                    linear_trend[axis_idx] = slope
                    trend_significance[axis_idx] = 1 - p_value  # 显著性
                
                # 变点检测（简单版本）
                changepoint_indices = []
                for axis_idx in range(3):
                    axis_data = foot_forces[:, axis_idx]
                    
                    # 滑动窗口变化检测
                    window_size = min(50, len(axis_data) // 10)
                    if window_size > 5:
                        for idx in range(window_size, len(axis_data) - window_size):
                            before_mean = np.mean(axis_data[idx-window_size:idx])
                            after_mean = np.mean(axis_data[idx:idx+window_size])
                            
                            if abs(after_mean - before_mean) > 2 * np.std(axis_data):
                                changepoint_indices.append(idx)
                
                # 去重变点
                changepoint_indices = sorted(list(set(changepoint_indices)))
                
                # 稳定性指标
                stability_metrics = {}
                for axis_idx in range(3):
                    axis_data = foot_forces[:, axis_idx]
                    
                    # 信号稳定性
                    stability_metrics[f'axis_{axis_idx}_stability'] = 1.0 - np.std(axis_data) / (np.abs(np.mean(axis_data)) + 1e-6)
                    
                    # 趋势强度
                    slope = linear_trend[axis_idx]
                    data_range = np.max(axis_data) - np.min(axis_data)
                    trend_strength = abs(slope * (time_relative[-1] - time_relative[0])) / (data_range + 1e-6)
                    stability_metrics[f'axis_{axis_idx}_trend_strength'] = min(trend_strength, 1.0)
                
                # 整体稳定性
                vertical_stability = stability_metrics.get('axis_2_stability', 0)
                overall_stability = np.mean([stability_metrics.get(f'axis_{i}_stability', 0) for i in range(3)])
                stability_metrics['vertical_stability'] = vertical_stability
                stability_metrics['overall_stability'] = overall_stability
                
                analysis = TrendAnalysis(
                    linear_trend=linear_trend,
                    trend_significance=trend_significance,
                    changepoint_indices=changepoint_indices,
                    stability_metrics=stability_metrics
                )
                
                results[foot_label] = analysis
            
            self.logger.info("趋势分析完成")
            return results
            
        except Exception as e:
            self.logger.error(f"趋势分析失败: {e}")
            return {}
    
    def generate_comprehensive_report(self, data: List[FootForceData]) -> Dict[str, Any]:
        """
        生成综合分析报告
        
        Args:
            data: 足端力数据列表
            
        Returns:
            Dict[str, Any]: 综合分析报告
        """
        self.logger.info("开始生成综合分析报告")
        
        try:
            report = {
                'analysis_timestamp': datetime.now().isoformat(),
                'data_summary': {
                    'total_samples': len(data),
                    'duration_seconds': data[-1].timestamp - data[0].timestamp if data else 0,
                    'sampling_rate': self.sampling_rate,
                    'analysis_parameters': self.analysis_config
                }
            }
            
            # 1. 统计分析
            statistical_results = self.perform_statistical_analysis(data)
            report['statistical_analysis'] = {}
            for foot_label, analysis in statistical_results.items():
                report['statistical_analysis'][foot_label] = {
                    'mean': analysis.mean.tolist(),
                    'std': analysis.std.tolist(),
                    'variance': analysis.variance.tolist(),
                    'min': analysis.min_val.tolist(),
                    'max': analysis.max_val.tolist(),
                    'median': analysis.median.tolist(),
                    'q25': analysis.q25.tolist(),
                    'q75': analysis.q75.tolist(),
                    'skewness': analysis.skewness.tolist(),
                    'kurtosis': analysis.kurtosis.tolist(),
                    'cv': analysis.coefficient_of_variation.tolist()
                }
            
            # 2. 频域分析
            frequency_results = self.perform_frequency_analysis(data)
            report['frequency_analysis'] = {}
            for foot_label, foot_analysis in frequency_results.items():
                report['frequency_analysis'][foot_label] = {}
                for axis_name, analysis in foot_analysis.items():
                    report['frequency_analysis'][foot_label][axis_name] = {
                        'dominant_frequency': analysis.dominant_frequency,
                        'frequency_peaks': analysis.frequency_peaks,
                        'spectral_centroid': analysis.spectral_centroid,
                        'bandwidth': analysis.bandwidth,
                        'snr': analysis.signal_to_noise_ratio
                    }
            
            # 3. 异常检测
            anomaly_results = self.detect_anomalies(data)
            report['anomaly_detection'] = {}
            for foot_label, analysis in anomaly_results.items():
                report['anomaly_detection'][foot_label] = {
                    'anomaly_rate': analysis.anomaly_rate,
                    'total_outliers': len(analysis.outlier_indices),
                    'threshold': analysis.threshold,
                    'anomaly_types': {k: len(v) for k, v in analysis.anomaly_types.items()}
                }
            
            # 4. 相关性分析
            correlation_result = self.perform_correlation_analysis(data)
            report['correlation_analysis'] = {
                'correlation_matrix': correlation_result.correlation_matrix.tolist(),
                'cross_correlation': correlation_result.cross_correlation,
                'lag_analysis': correlation_result.lag_analysis,
                'coherence': correlation_result.coherence
            }
            
            # 5. 趋势分析
            trend_results = self.analyze_trends(data)
            report['trend_analysis'] = {}
            for foot_label, analysis in trend_results.items():
                report['trend_analysis'][foot_label] = {
                    'linear_trend': analysis.linear_trend.tolist(),
                    'trend_significance': analysis.trend_significance.tolist(),
                    'changepoints': len(analysis.changepoint_indices),
                    'stability_metrics': analysis.stability_metrics
                }
            
            # 6. 综合评估
            report['overall_assessment'] = self._generate_overall_assessment(
                statistical_results, frequency_results, anomaly_results, 
                correlation_result, trend_results
            )
            
            self.logger.info("综合分析报告生成完成")
            return report
            
        except Exception as e:
            self.logger.error(f"生成综合分析报告失败: {e}")
            return {'error': str(e)}
    
    def _generate_overall_assessment(self, statistical_results, frequency_results, 
                                   anomaly_results, correlation_result, trend_results) -> Dict[str, Any]:
        """生成综合评估"""
        
        assessment = {
            'data_quality_score': 0.0,
            'sensor_consistency_score': 0.0,
            'stability_score': 0.0,
            'overall_score': 0.0,
            'recommendations': [],
            'key_findings': []
        }
        
        try:
            # 数据质量评分
            quality_scores = []
            for foot_label in FootForceConfig.FOOT_LABELS:
                if foot_label in anomaly_results:
                    anomaly_rate = anomaly_results[foot_label].anomaly_rate
                    quality_score = max(0, 100 - anomaly_rate * 2)  # 异常率越低质量越好
                    quality_scores.append(quality_score)
            
            assessment['data_quality_score'] = np.mean(quality_scores) if quality_scores else 0
            
            # 传感器一致性评分
            if correlation_result.correlation_matrix.size > 0:
                # 排除对角线元素计算平均相关系数
                mask = ~np.eye(correlation_result.correlation_matrix.shape[0], dtype=bool)
                avg_correlation = np.mean(np.abs(correlation_result.correlation_matrix[mask]))
                consistency_score = avg_correlation * 100
                assessment['sensor_consistency_score'] = consistency_score
            
            # 稳定性评分
            stability_scores = []
            for foot_label in FootForceConfig.FOOT_LABELS:
                if foot_label in trend_results:
                    stability = trend_results[foot_label].stability_metrics.get('overall_stability', 0)
                    stability_scores.append(stability * 100)
            
            assessment['stability_score'] = np.mean(stability_scores) if stability_scores else 0
            
            # 总体评分
            assessment['overall_score'] = np.mean([
                assessment['data_quality_score'],
                assessment['sensor_consistency_score'], 
                assessment['stability_score']
            ])
            
            # 生成建议
            if assessment['data_quality_score'] < 70:
                assessment['recommendations'].append("数据质量偏低，建议检查传感器连接和环境干扰")
            
            if assessment['sensor_consistency_score'] < 60:
                assessment['recommendations'].append("传感器间一致性较差，建议重新校准传感器系统")
            
            if assessment['stability_score'] < 80:
                assessment['recommendations'].append("信号稳定性有待改善，检查机械安装和固定情况")
            
            if assessment['overall_score'] >= 85:
                assessment['recommendations'].append("传感器系统整体表现优秀，可以投入正式使用")
            
            # 关键发现
            if assessment['data_quality_score'] > 90:
                assessment['key_findings'].append("数据质量优秀，异常率很低")
            
            if assessment['sensor_consistency_score'] > 80:
                assessment['key_findings'].append("四足端传感器响应高度一致")
            
            if assessment['stability_score'] > 85:
                assessment['key_findings'].append("传感器信号稳定性良好")
            
        except Exception as e:
            self.logger.error(f"生成综合评估失败: {e}")
            assessment['recommendations'].append("评估过程出现错误，建议重新分析")
        
        return assessment
    
    def save_analysis_report(self, report: Dict[str, Any], filepath: str) -> bool:
        """
        保存分析报告
        
        Args:
            report: 分析报告字典
            filepath: 保存路径
            
        Returns:
            bool: 是否成功
        """
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            
            self.logger.info(f"分析报告已保存到: {filepath}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存分析报告失败: {e}")
            return False
    
    def export_to_csv(self, data: List[FootForceData], filepath: str) -> bool:
        """
        导出数据到CSV文件
        
        Args:
            data: 足端力数据列表
            filepath: 保存路径
            
        Returns:
            bool: 是否成功
        """
        try:
            # 准备数据
            rows = []
            for d in data:
                row = {
                    'timestamp': d.timestamp,
                    'total_force': d.total_force,
                    'stability_index': d.stability_index,
                    'force_balance': d.force_balance,
                    'cop_x': d.center_of_pressure[0],
                    'cop_y': d.center_of_pressure[1]
                }
                
                # 添加各足端力数据
                for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                    row[f'{foot_label}_fx'] = d.foot_forces[i][0]
                    row[f'{foot_label}_fy'] = d.foot_forces[i][1]
                    row[f'{foot_label}_fz'] = d.foot_forces[i][2]
                    row[f'{foot_label}_magnitude'] = d.force_magnitudes[i]
                    row[f'{foot_label}_contact'] = d.contact_states[i]
                
                rows.append(row)
            
            # 创建DataFrame并保存
            df = pd.DataFrame(rows)
            df.to_csv(filepath, index=False)
            
            self.logger.info(f"数据已导出到CSV文件: {filepath}")
            return True
            
        except Exception as e:
            self.logger.error(f"导出CSV文件失败: {e}")
            return False 