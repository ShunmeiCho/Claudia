#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/static_tester.py
# Generated: 2025-06-27 14:18:30 CST
# Purpose: Unitree Go2 足端力传感器静态验证测试器

import time
import numpy as np
import logging
import threading
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from collections import deque
import json
from pathlib import Path
from datetime import datetime

from foot_force_config import FootForceConfig, FootForceReading
from data_collector import FootForceDataCollector, FootForceData

@dataclass
class StaticTestResult:
    """静态测试结果"""
    test_name: str
    status: str  # PASS, FAIL, WARNING
    score: float  # 0-100分
    measurements: Dict[str, Any]
    analysis: Dict[str, Any]
    recommendations: List[str]
    timestamp: float

@dataclass
class ZeroDriftResult:
    """零点漂移测试结果"""
    initial_baseline: Dict[str, Tuple[float, float, float]]  # 每个足端的初始基线
    final_baseline: Dict[str, Tuple[float, float, float]]    # 最终基线
    drift_values: Dict[str, Tuple[float, float, float]]      # 漂移值
    drift_rates: Dict[str, Tuple[float, float, float]]       # 漂移率(N/min)
    max_drift: float                                          # 最大漂移值
    drift_stability: float                                    # 漂移稳定性评分(0-100)

@dataclass
class ConsistencyResult:
    """四足一致性测试结果"""
    cross_correlation: Dict[str, float]                       # 足端间相关系数
    synchronization_delay: Dict[str, float]                   # 同步延迟(ms)
    response_matching: Dict[str, float]                       # 响应匹配度(0-1)
    calibration_differences: Dict[str, Tuple[float, float, float]]  # 校准差异
    consistency_score: float                                  # 一致性评分(0-100)

@dataclass
class BalanceAnalysis:
    """平衡分析结果"""
    center_of_pressure: Tuple[float, float]                  # 平均压力中心
    cop_std: Tuple[float, float]                             # 压力中心标准差
    cop_range: Tuple[float, float]                           # 压力中心范围
    weight_distribution: Dict[str, float]                    # 重量分布百分比
    stability_metrics: Dict[str, float]                      # 稳定性指标
    balance_quality: float                                    # 平衡质量评分(0-100)

class StaticForceTester:
    """静态力分布验证测试器"""
    
    def __init__(self, config: Dict[str, Any], foot_force_config: FootForceConfig):
        """
        初始化静态测试器
        
        Args:
            config: 配置字典
            foot_force_config: 足端力配置实例
        """
        self.config = config
        self.foot_force_config = foot_force_config
        self.logger = logging.getLogger(__name__)
        
        # 测试参数
        self.static_config = config.get('static_validation', {})
        self.force_threshold = config.get('foot_force_config', {}).get('force_threshold', 5.0)
        self.max_force_per_foot = config.get('foot_force_config', {}).get('max_force_per_foot', 200.0)
        
        # 测试结果存储
        self.test_results: List[StaticTestResult] = []
        self.zero_drift_result: Optional[ZeroDriftResult] = None
        self.consistency_result: Optional[ConsistencyResult] = None
        self.balance_analysis: Optional[BalanceAnalysis] = None
        
        # 数据采集器
        self.data_collector = FootForceDataCollector(config, foot_force_config)
        
        self.logger.info("静态力分布验证测试器初始化完成")
    
    def run_zero_load_test(self, duration: float = 30.0) -> StaticTestResult:
        """
        零负载测试 - 机器人悬空时的零点测试
        
        Args:
            duration: 测试持续时间(秒)
            
        Returns:
            StaticTestResult: 测试结果
        """
        self.logger.info(f"开始零负载测试，持续时间: {duration}秒")
        
        try:
            # 开始数据收集
            if not self.data_collector.start_collection(duration):
                raise Exception("零负载测试数据收集启动失败")
            
            # 等待数据收集完成
            while self.data_collector.is_collecting:
                time.sleep(1.0)
            
            # 获取数据
            collected_data = self.data_collector.get_data()
            
            if len(collected_data) < 10:
                raise Exception("零负载测试数据不足")
            
            # 分析零负载数据
            measurements = self._analyze_zero_load_data(collected_data)
            
            # 评估结果
            score, status, analysis = self._evaluate_zero_load_results(measurements)
            
            # 生成建议
            recommendations = self._generate_zero_load_recommendations(measurements, analysis)
            
            result = StaticTestResult(
                test_name="zero_load_test",
                status=status,
                score=score,
                measurements=measurements,
                analysis=analysis,
                recommendations=recommendations,
                timestamp=time.time()
            )
            
            self.test_results.append(result)
            self.logger.info(f"零负载测试完成，评分: {score:.1f}, 状态: {status}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"零负载测试失败: {e}")
            error_result = StaticTestResult(
                test_name="zero_load_test",
                status="FAIL",
                score=0.0,
                measurements={},
                analysis={"error": str(e)},
                recommendations=["检查机器人悬挂状态", "验证传感器连接"],
                timestamp=time.time()
            )
            self.test_results.append(error_result)
            return error_result
    
    def _analyze_zero_load_data(self, data: List[FootForceData]) -> Dict[str, Any]:
        """分析零负载数据"""
        measurements = {}
        
        # 提取力数据
        forces_array = np.array([d.foot_forces for d in data])  # (samples, 4_feet, 3_axes)
        
        # 分析每个足端
        for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = forces_array[:, i, :]  # (samples, 3_axes)
            
            measurements[f"{foot_label}_stats"] = {
                "mean_fx": float(np.mean(foot_forces[:, 0])),
                "mean_fy": float(np.mean(foot_forces[:, 1])),
                "mean_fz": float(np.mean(foot_forces[:, 2])),
                "std_fx": float(np.std(foot_forces[:, 0])),
                "std_fy": float(np.std(foot_forces[:, 1])),
                "std_fz": float(np.std(foot_forces[:, 2])),
                "max_magnitude": float(np.max([np.linalg.norm(f) for f in foot_forces])),
                "rms_noise": float(np.sqrt(np.mean(np.sum(foot_forces**2, axis=1))))
            }
        
        # 总体统计
        total_forces = np.array([d.total_force for d in data])
        measurements["total_force_stats"] = {
            "mean": float(np.mean(total_forces)),
            "std": float(np.std(total_forces)),
            "max": float(np.max(total_forces)),
            "rms": float(np.sqrt(np.mean(total_forces**2)))
        }
        
        # 压力中心分析
        cops = np.array([d.center_of_pressure for d in data])
        measurements["cop_stats"] = {
            "mean_x": float(np.mean(cops[:, 0])),
            "mean_y": float(np.mean(cops[:, 1])),
            "std_x": float(np.std(cops[:, 0])),
            "std_y": float(np.std(cops[:, 1])),
            "range_x": float(np.max(cops[:, 0]) - np.min(cops[:, 0])),
            "range_y": float(np.max(cops[:, 1]) - np.min(cops[:, 1]))
        }
        
        return measurements
    
    def _evaluate_zero_load_results(self, measurements: Dict[str, Any]) -> Tuple[float, str, Dict[str, Any]]:
        """评估零负载测试结果"""
        analysis = {}
        score_components = []
        
        zero_threshold = self.static_config.get('zero_force_threshold', 2.0)
        
        # 评估每个足端的零点性能
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_stats = measurements[f"{foot_label}_stats"]
            
            # 均值接近零点评分
            mean_magnitude = np.sqrt(foot_stats["mean_fx"]**2 + 
                                   foot_stats["mean_fy"]**2 + 
                                   foot_stats["mean_fz"]**2)
            zero_score = max(0, 100 - (mean_magnitude / zero_threshold) * 100)
            
            # 噪声水平评分
            noise_score = max(0, 100 - (foot_stats["rms_noise"] / zero_threshold) * 100)
            
            # 稳定性评分
            stability_score = max(0, 100 - (foot_stats["std_fz"] / zero_threshold) * 100)
            
            foot_score = (zero_score + noise_score + stability_score) / 3
            score_components.append(foot_score)
            
            analysis[f"{foot_label}_evaluation"] = {
                "zero_score": zero_score,
                "noise_score": noise_score,
                "stability_score": stability_score,
                "overall_score": foot_score,
                "mean_magnitude": mean_magnitude,
                "meets_zero_threshold": mean_magnitude < zero_threshold
            }
        
        # 总体评分
        overall_score = np.mean(score_components)
        
        # 状态判定
        if overall_score >= 90:
            status = "PASS"
        elif overall_score >= 70:
            status = "WARNING"
        else:
            status = "FAIL"
        
        analysis["overall_evaluation"] = {
            "score": overall_score,
            "status": status,
            "zero_threshold": zero_threshold,
            "all_feet_pass": all(measurements[f"{foot}_stats"]["rms_noise"] < zero_threshold 
                               for foot in FootForceConfig.FOOT_LABELS)
        }
        
        return overall_score, status, analysis
    
    def _generate_zero_load_recommendations(self, measurements: Dict[str, Any], 
                                          analysis: Dict[str, Any]) -> List[str]:
        """生成零负载测试建议"""
        recommendations = []
        
        zero_threshold = self.static_config.get('zero_force_threshold', 2.0)
        
        # 检查每个足端
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_stats = measurements[f"{foot_label}_stats"]
            foot_eval = analysis[f"{foot_label}_evaluation"]
            
            if not foot_eval["meets_zero_threshold"]:
                recommendations.append(f"{foot_label}足端零点偏移过大({foot_eval['mean_magnitude']:.2f}N)，建议重新校准")
            
            if foot_stats["rms_noise"] > zero_threshold * 0.5:
                recommendations.append(f"{foot_label}足端噪声水平过高({foot_stats['rms_noise']:.2f}N)，检查传感器连接")
            
            if foot_stats["std_fz"] > zero_threshold * 0.3:
                recommendations.append(f"{foot_label}足端垂直力不稳定，检查机械安装")
        
        # 总体建议
        if analysis["overall_evaluation"]["score"] < 70:
            recommendations.append("整体零点性能不佳，建议完整重新校准系统")
        elif analysis["overall_evaluation"]["score"] < 90:
            recommendations.append("零点性能可接受但有改进空间，建议定期监控")
        
        if not recommendations:
            recommendations.append("零负载测试表现优秀，传感器零点性能正常")
        
        return recommendations
    
    def run_static_standing_test(self, duration: float = 60.0) -> StaticTestResult:
        """
        静态站立测试 - 机器人正常站立时的力分布测试
        
        Args:
            duration: 测试持续时间(秒)
            
        Returns:
            StaticTestResult: 测试结果
        """
        self.logger.info(f"开始静态站立测试，持续时间: {duration}秒")
        
        try:
            # 开始数据收集
            if not self.data_collector.start_collection(duration):
                raise Exception("静态站立测试数据收集启动失败")
            
            # 等待数据收集完成
            while self.data_collector.is_collecting:
                time.sleep(2.0)  # 每2秒检查一次状态
            
            # 获取数据
            collected_data = self.data_collector.get_data()
            
            if len(collected_data) < 30:
                raise Exception("静态站立测试数据不足")
            
            # 分析静态站立数据
            measurements = self._analyze_static_standing_data(collected_data)
            
            # 评估结果
            score, status, analysis = self._evaluate_static_standing_results(measurements)
            
            # 生成建议
            recommendations = self._generate_static_standing_recommendations(measurements, analysis)
            
            result = StaticTestResult(
                test_name="static_standing_test",
                status=status,
                score=score,
                measurements=measurements,
                analysis=analysis,
                recommendations=recommendations,
                timestamp=time.time()
            )
            
            self.test_results.append(result)
            self.logger.info(f"静态站立测试完成，评分: {score:.1f}, 状态: {status}")
            
            return result
            
        except Exception as e:
            self.logger.error(f"静态站立测试失败: {e}")
            error_result = StaticTestResult(
                test_name="static_standing_test",
                status="FAIL",
                score=0.0,
                measurements={},
                analysis={"error": str(e)},
                recommendations=["检查机器人站立姿势", "验证地面水平"],
                timestamp=time.time()
            )
            self.test_results.append(error_result)
            return error_result
    
    def _analyze_static_standing_data(self, data: List[FootForceData]) -> Dict[str, Any]:
        """分析静态站立数据"""
        measurements = {}
        
        # 提取数据数组
        forces_array = np.array([d.foot_forces for d in data])
        total_forces = np.array([d.total_force for d in data])
        cops = np.array([d.center_of_pressure for d in data])
        stability_indices = np.array([d.stability_index for d in data])
        balance_indices = np.array([d.force_balance for d in data])
        
        # 每个足端的详细分析
        for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = forces_array[:, i, :]
            
            measurements[f"{foot_label}_analysis"] = {
                "mean_force": foot_forces.mean(axis=0).tolist(),
                "std_force": foot_forces.std(axis=0).tolist(),
                "mean_vertical": float(np.mean(foot_forces[:, 2])),
                "std_vertical": float(np.std(foot_forces[:, 2])),
                "force_range": float(np.max(foot_forces[:, 2]) - np.min(foot_forces[:, 2])),
                "contact_ratio": float(np.mean(foot_forces[:, 2] > self.force_threshold)),
                "force_stability": float(1.0 - np.std(foot_forces[:, 2]) / (np.mean(foot_forces[:, 2]) + 1e-6))
            }
        
        # 重量分布分析
        vertical_forces = forces_array[:, :, 2]  # 提取垂直力
        mean_vertical_forces = np.mean(vertical_forces, axis=0)
        total_weight = np.sum(mean_vertical_forces)
        
        weight_distribution = {}
        for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            weight_distribution[foot_label] = float(mean_vertical_forces[i] / total_weight * 100) if total_weight > 0 else 0.0
        
        measurements["weight_distribution"] = weight_distribution
        measurements["total_weight"] = float(total_weight)
        
        # 平衡分析
        measurements["balance_analysis"] = {
            "cop_mean": cops.mean(axis=0).tolist(),
            "cop_std": cops.std(axis=0).tolist(),
            "cop_range": (cops.max(axis=0) - cops.min(axis=0)).tolist(),
            "stability_mean": float(np.mean(stability_indices)),
            "stability_std": float(np.std(stability_indices)),
            "balance_mean": float(np.mean(balance_indices)),
            "balance_std": float(np.std(balance_indices))
        }
        
        # 总力分析
        measurements["total_force_analysis"] = {
            "mean": float(np.mean(total_forces)),
            "std": float(np.std(total_forces)),
            "coefficient_of_variation": float(np.std(total_forces) / (np.mean(total_forces) + 1e-6)),
            "force_range": float(np.max(total_forces) - np.min(total_forces))
        }
        
        return measurements
    
    def _evaluate_static_standing_results(self, measurements: Dict[str, Any]) -> Tuple[float, str, Dict[str, Any]]:
        """评估静态站立测试结果"""
        analysis = {}
        score_components = []
        
        expected_total_force = self.static_config.get('expected_total_force', 150.0)  # N
        tolerance = self.static_config.get('tolerance', 10.0)  # N
        balance_threshold = self.static_config.get('balance_threshold', 0.1)
        
        # 1. 总重量准确性评分
        total_weight = measurements["total_weight"]
        weight_error = abs(total_weight - expected_total_force)
        weight_score = max(0, 100 - (weight_error / tolerance) * 100)
        score_components.append(weight_score)
        
        analysis["weight_accuracy"] = {
            "measured_weight": total_weight,
            "expected_weight": expected_total_force,
            "error": weight_error,
            "error_percentage": weight_error / expected_total_force * 100,
            "score": weight_score,
            "within_tolerance": weight_error <= tolerance
        }
        
        # 2. 重量分布平衡性评分
        weight_dist = measurements["weight_distribution"]
        ideal_distribution = 25.0  # 理想情况下每足25%
        distribution_errors = [abs(weight_dist[foot] - ideal_distribution) for foot in FootForceConfig.FOOT_LABELS]
        max_distribution_error = max(distribution_errors)
        distribution_score = max(0, 100 - max_distribution_error * 4)  # 4是缩放因子
        score_components.append(distribution_score)
        
        analysis["weight_distribution_analysis"] = {
            "distribution": weight_dist,
            "distribution_errors": dict(zip(FootForceConfig.FOOT_LABELS, distribution_errors)),
            "max_error": max_distribution_error,
            "score": distribution_score,
            "is_balanced": max_distribution_error <= 10.0  # 10%误差内
        }
        
        # 3. 稳定性评分
        balance_analysis = measurements["balance_analysis"]
        stability_score = balance_analysis["stability_mean"] * 100
        score_components.append(stability_score)
        
        # 4. 压力中心稳定性评分
        cop_std = np.linalg.norm(balance_analysis["cop_std"])
        cop_score = max(0, 100 - cop_std * 1000)  # 假设理想COP标准差 < 0.1m
        score_components.append(cop_score)
        
        analysis["stability_analysis"] = {
            "stability_index": balance_analysis["stability_mean"],
            "balance_index": balance_analysis["balance_mean"],
            "cop_stability": cop_std,
            "stability_score": stability_score,
            "cop_score": cop_score
        }
        
        # 5. 每个足端的一致性评分
        foot_scores = []
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_analysis = measurements[f"{foot_label}_analysis"]
            foot_stability = foot_analysis["force_stability"]
            foot_score = foot_stability * 100
            foot_scores.append(foot_score)
            
            analysis[f"{foot_label}_performance"] = {
                "mean_vertical_force": foot_analysis["mean_vertical"],
                "force_stability": foot_stability,
                "contact_ratio": foot_analysis["contact_ratio"],
                "score": foot_score
            }
        
        consistency_score = np.mean(foot_scores)
        score_components.append(consistency_score)
        
        # 总体评分
        overall_score = np.mean(score_components)
        
        # 状态判定
        if overall_score >= 85:
            status = "PASS"
        elif overall_score >= 70:
            status = "WARNING"
        else:
            status = "FAIL"
        
        analysis["overall_evaluation"] = {
            "score": overall_score,
            "status": status,
            "score_components": {
                "weight_accuracy": weight_score,
                "weight_distribution": distribution_score,
                "stability": stability_score,
                "cop_stability": cop_score,
                "foot_consistency": consistency_score
            }
        }
        
        return overall_score, status, analysis
    
    def _generate_static_standing_recommendations(self, measurements: Dict[str, Any], 
                                                analysis: Dict[str, Any]) -> List[str]:
        """生成静态站立测试建议"""
        recommendations = []
        
        # 重量准确性建议
        weight_analysis = analysis["weight_accuracy"]
        if not weight_analysis["within_tolerance"]:
            recommendations.append(f"总重量误差过大({weight_analysis['error']:.1f}N)，建议检查传感器校准")
        
        # 重量分布建议
        dist_analysis = analysis["weight_distribution_analysis"]
        if not dist_analysis["is_balanced"]:
            max_error_foot = max(dist_analysis["distribution_errors"].items(), key=lambda x: x[1])
            recommendations.append(f"{max_error_foot[0]}足端承重异常({max_error_foot[1]:.1f}%偏差)，检查机器人姿态或地面水平")
        
        # 稳定性建议
        stability_analysis = analysis["stability_analysis"]
        if stability_analysis["stability_score"] < 70:
            recommendations.append("力分布稳定性不佳，建议检查机械结构和传感器安装")
        
        if stability_analysis["cop_score"] < 70:
            recommendations.append("压力中心波动过大，建议检查地面状况和机器人平衡")
        
        # 每个足端建议
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_perf = analysis[f"{foot_label}_performance"]
            if foot_perf["score"] < 70:
                recommendations.append(f"{foot_label}足端性能不佳，建议检查该足端传感器和机械部件")
            
            if foot_perf["contact_ratio"] < 0.95:
                recommendations.append(f"{foot_label}足端接触不稳定，检查地面接触状况")
        
        if not recommendations:
            recommendations.append("静态站立测试表现优秀，力分布和稳定性均正常")
        
        return recommendations
    
    def run_zero_drift_analysis(self, duration: float = 300.0) -> ZeroDriftResult:
        """
        零点漂移分析 - 长时间监控传感器零点稳定性
        
        Args:
            duration: 监控持续时间(秒)
            
        Returns:
            ZeroDriftResult: 漂移分析结果
        """
        self.logger.info(f"开始零点漂移分析，持续时间: {duration}秒")
        
        try:
            # 分段采集数据以分析漂移趋势
            segment_duration = 30.0  # 每段30秒
            num_segments = int(duration / segment_duration)
            
            baseline_data = []
            timestamps = []
            
            for segment in range(num_segments):
                self.logger.info(f"采集第{segment+1}/{num_segments}段数据...")
                
                # 采集本段数据
                if not self.data_collector.start_collection(segment_duration):
                    raise Exception(f"第{segment+1}段数据采集失败")
                
                while self.data_collector.is_collecting:
                    time.sleep(1.0)
                
                segment_data = self.data_collector.get_data()
                
                if len(segment_data) < 10:
                    self.logger.warning(f"第{segment+1}段数据不足，跳过")
                    continue
                
                # 计算本段基线
                forces_array = np.array([d.foot_forces for d in segment_data])
                segment_baseline = {}
                
                for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                    foot_forces = forces_array[:, i, :]
                    segment_baseline[foot_label] = tuple(np.mean(foot_forces, axis=0))
                
                baseline_data.append(segment_baseline)
                timestamps.append(time.time())
                
                # 段间休息
                if segment < num_segments - 1:
                    time.sleep(2.0)
            
            if len(baseline_data) < 3:
                raise Exception("漂移分析数据段数不足")
            
            # 分析漂移趋势
            drift_result = self._analyze_zero_drift(baseline_data, timestamps)
            
            self.zero_drift_result = drift_result
            self.logger.info(f"零点漂移分析完成，最大漂移: {drift_result.max_drift:.3f}N")
            
            return drift_result
            
        except Exception as e:
            self.logger.error(f"零点漂移分析失败: {e}")
            # 返回空的漂移结果
            return ZeroDriftResult(
                initial_baseline={},
                final_baseline={},
                drift_values={},
                drift_rates={},
                max_drift=0.0,
                drift_stability=0.0
            )
    
    def _analyze_zero_drift(self, baseline_data: List[Dict[str, Tuple[float, float, float]]], 
                           timestamps: List[float]) -> ZeroDriftResult:
        """分析零点漂移数据"""
        
        if len(baseline_data) < 2:
            return ZeroDriftResult({}, {}, {}, {}, 0.0, 0.0)
        
        initial_baseline = baseline_data[0]
        final_baseline = baseline_data[-1]
        
        # 计算漂移值和漂移率
        drift_values = {}
        drift_rates = {}
        max_drift = 0.0
        
        duration_minutes = (timestamps[-1] - timestamps[0]) / 60.0
        
        for foot_label in FootForceConfig.FOOT_LABELS:
            if foot_label in initial_baseline and foot_label in final_baseline:
                initial = np.array(initial_baseline[foot_label])
                final = np.array(final_baseline[foot_label])
                
                drift = final - initial
                drift_magnitude = np.linalg.norm(drift)
                drift_rate = drift / duration_minutes if duration_minutes > 0 else np.zeros_like(drift)
                
                drift_values[foot_label] = tuple(drift)
                drift_rates[foot_label] = tuple(drift_rate)
                
                max_drift = max(max_drift, drift_magnitude)
        
        # 计算漂移稳定性评分
        drift_stability = max(0, 100 - max_drift * 50)  # 假设漂移超过2N就很糟糕
        
        return ZeroDriftResult(
            initial_baseline=initial_baseline,
            final_baseline=final_baseline,
            drift_values=drift_values,
            drift_rates=drift_rates,
            max_drift=max_drift,
            drift_stability=drift_stability
        )
    
    def generate_static_validation_report(self) -> Dict[str, Any]:
        """生成静态验证综合报告"""
        report = {
            "test_timestamp": datetime.now().isoformat(),
            "test_results": [result.__dict__ for result in self.test_results],
            "zero_drift_analysis": self.zero_drift_result.__dict__ if self.zero_drift_result else None,
            "consistency_analysis": self.consistency_result.__dict__ if self.consistency_result else None,
            "balance_analysis": self.balance_analysis.__dict__ if self.balance_analysis else None
        }
        
        # 总体评估
        if self.test_results:
            overall_score = np.mean([result.score for result in self.test_results])
            all_pass = all(result.status == "PASS" for result in self.test_results)
            
            report["overall_assessment"] = {
                "overall_score": overall_score,
                "status": "PASS" if all_pass and overall_score >= 80 else "FAIL" if overall_score < 60 else "WARNING",
                "total_tests": len(self.test_results),
                "passed_tests": sum(1 for result in self.test_results if result.status == "PASS"),
                "failed_tests": sum(1 for result in self.test_results if result.status == "FAIL")
            }
        
        return report
    
    def save_static_validation_report(self, filepath: str) -> bool:
        """保存静态验证报告"""
        try:
            report = self.generate_static_validation_report()
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            
            self.logger.info(f"静态验证报告已保存到: {filepath}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存静态验证报告失败: {e}")
            return False 
    
    def run_quick_static_test(self, duration: float = 10.0) -> Dict[str, Any]:
        """
        运行快速静态测试 - 用于完整验证流程
        
        Args:
            duration: 快速测试持续时间(秒)
            
        Returns:
            Dict: 快速测试结果
        """
        self.logger.info(f"开始快速静态测试，持续时间: {duration}秒")
        
        try:
            # 运行零负载测试
            zero_result = self.run_zero_load_test(duration)
            
            # 生成简化的结果
            quick_result = {
                'success': zero_result.status != "FAIL",
                'test_name': 'quick_static_test',
                'status': zero_result.status,
                'final_score': zero_result.score,
                'duration': duration,
                'measurements': zero_result.measurements,
                'recommendations': zero_result.recommendations,
                'timestamp': zero_result.timestamp
            }
            
            self.logger.info(f"快速静态测试完成，评分: {zero_result.score:.1f}")
            return quick_result
            
        except Exception as e:
            self.logger.error(f"快速静态测试失败: {e}")
            return {
                'success': False,
                'test_name': 'quick_static_test',
                'status': 'FAIL',
                'final_score': 0.0,
                'error': str(e),
                'timestamp': time.time()
            }


# 为了兼容性添加别名
StaticFootForceTester = StaticForceTester 