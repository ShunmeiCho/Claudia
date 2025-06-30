#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/dynamic_tester.py
# Generated: 2025-06-26 18:40:00
# Purpose: Unitree Go2 足端力传感器动态响应测试框架

import time
import threading
import logging
import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Callable
from dataclasses import dataclass, field
import json
from datetime import datetime
from pathlib import Path

from foot_force_config import FootForceConfig, FootForceReading
from data_collector import FootForceDataCollector, FootForceData, FootForceCollectionMetrics

@dataclass
class GaitPhase:
    """步态相位数据"""
    phase_name: str  # "stance", "swing", "impact", "lift_off"
    start_time: float
    end_time: float
    foot_id: int
    peak_force: float
    average_force: float
    force_profile: List[float]
    contact_quality: float

@dataclass
class DynamicTestResult:
    """动态测试结果"""
    test_name: str
    start_time: float
    end_time: float
    duration: float
    total_samples: int
    
    # 步态分析结果
    gait_phases: List[GaitPhase] = field(default_factory=list)
    step_frequency: float = 0.0
    stride_length: float = 0.0
    contact_time_avg: float = 0.0
    flight_time_avg: float = 0.0
    
    # 力学特性
    peak_forces: Dict[str, float] = field(default_factory=dict)
    impulse_values: Dict[str, float] = field(default_factory=dict)
    force_rise_times: Dict[str, float] = field(default_factory=dict)
    force_fall_times: Dict[str, float] = field(default_factory=dict)
    
    # 稳定性指标
    stability_variance: float = 0.0
    balance_consistency: float = 0.0
    coordination_index: float = 0.0
    
    # 测试评分
    test_score: float = 0.0
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    
    # 原始数据统计
    raw_data_stats: Dict[str, Any] = field(default_factory=dict)

class DynamicFootForceTester:
    """动态足端力传感器测试器"""
    
    def __init__(self, config: Dict, foot_force_config: FootForceConfig):
        """
        初始化动态测试器
        
        Args:
            config: 配置字典
            foot_force_config: FootForceConfig实例
        """
        self.config = config
        self.foot_force_config = foot_force_config
        self.logger = logging.getLogger(__name__)
        
        # 动态验证配置
        self.dynamic_config = config.get('dynamic_validation', {})
        
        # 数据收集器
        self.data_collector = FootForceDataCollector(config, foot_force_config)
        
        # 测试状态
        self.is_testing = False
        self.current_test = None
        
        # 实时分析
        self.step_detector = StepDetector(self.dynamic_config)
        self.gait_analyzer = GaitAnalyzer(self.dynamic_config)
        
        self.logger.info("动态足端力测试器初始化完成")
    
    def run_dynamic_test_suite(self) -> Dict[str, DynamicTestResult]:
        """运行完整的动态测试套件"""
        test_results = {}
        
        # 从配置获取测试场景
        test_scenarios = self.config.get('test_scenarios', {}).get('dynamic_tests', [])
        
        self.logger.info(f"开始运行动态测试套件，包含 {len(test_scenarios)} 个测试")
        
        for scenario in test_scenarios:
            test_name = scenario['name']
            self.logger.info(f"开始测试: {test_name}")
            
            try:
                # 运行单个测试
                result = self._run_single_dynamic_test(scenario)
                test_results[test_name] = result
                
                # 等待测试间隔
                time.sleep(5.0)
                
            except Exception as e:
                self.logger.error(f"测试 {test_name} 失败: {e}")
                # 创建失败结果
                test_results[test_name] = DynamicTestResult(
                    test_name=test_name,
                    start_time=time.time(),
                    end_time=time.time(),
                    duration=0.0,
                    total_samples=0,
                    test_score=0.0
                )
        
        self.logger.info("动态测试套件完成")
        return test_results
    
    def _run_single_dynamic_test(self, scenario: Dict) -> DynamicTestResult:
        """运行单个动态测试"""
        test_name = scenario['name']
        duration = scenario.get('duration', 60.0)
        
        self.logger.info(f"执行动态测试: {test_name}，持续时间: {duration}秒")
        
        # 初始化测试结果
        result = DynamicTestResult(
            test_name=test_name,
            start_time=time.time(),
            end_time=0.0,
            duration=duration,
            total_samples=0
        )
        
        # 重置分析器
        self.step_detector.reset()
        self.gait_analyzer.reset()
        
        # 设置数据回调
        self.data_collector.add_data_callback(self._dynamic_analysis_callback)
        
        # 开始数据收集
        if not self.data_collector.start_collection(duration):
            raise RuntimeError(f"无法开始数据收集：{test_name}")
        
        # 等待测试完成
        self.is_testing = True
        self.current_test = test_name
        
        # 显示测试提示
        self._display_test_instructions(scenario)
        
        # 等待数据收集完成
        time.sleep(duration + 1.0)
        
        # 停止收集并获取指标
        collection_metrics = self.data_collector.stop_collection()
        self.is_testing = False
        
        # 完成结果统计
        result.end_time = time.time()
        result.total_samples = collection_metrics.total_samples
        
        # 执行动态分析
        collected_data = self.data_collector.get_data()
        result = self._analyze_dynamic_data(result, collected_data, scenario)
        
        # 计算测试评分
        result.test_score = self._calculate_dynamic_test_score(result, scenario)
        
        self.logger.info(f"测试 {test_name} 完成，评分: {result.test_score:.1f}/100")
        
        return result
    
    def _display_test_instructions(self, scenario: Dict):
        """显示测试指令"""
        test_name = scenario['name']
        duration = scenario.get('duration', 60)
        
        print(f"\n{'='*60}")
        print(f"🤖 动态测试: {test_name}")
        print(f"📋 描述: {scenario.get('description', '无描述')}")
        print(f"⏱️  持续时间: {duration}秒")
        print(f"{'='*60}")
        
        if test_name == "slow_walk":
            print("📝 请执行以下操作:")
            print("   1. 让机器人以缓慢速度行走")
            print("   2. 保持稳定的步态节奏")
            print("   3. 避免突然停止或转向")
            
        elif test_name == "normal_walk":
            print("📝 请执行以下操作:")
            print("   1. 让机器人以正常速度行走")
            print("   2. 观察四足协调运动")
            print("   3. 可以包含轻微转向")
            
        elif test_name == "impact_test":
            print("📝 请执行以下操作:")
            print("   1. 让机器人执行跳跃动作")
            print("   2. 观察着地时的冲击力")
            print("   3. 确保安全距离")
            
        print(f"\n⏳ 测试将在 3 秒后开始...")
        for i in range(3, 0, -1):
            print(f"   {i}...")
            time.sleep(1)
        print("🚀 测试开始！")
    
    def _dynamic_analysis_callback(self, data: FootForceData):
        """动态分析数据回调"""
        if not self.is_testing:
            return
        
        try:
            # 步态检测
            self.step_detector.process_data(data)
            
            # 步态分析
            self.gait_analyzer.process_data(data)
            
        except Exception as e:
            self.logger.error(f"动态分析回调错误: {e}")
    
    def _analyze_dynamic_data(self, result: DynamicTestResult, data: List[FootForceData], scenario: Dict) -> DynamicTestResult:
        """分析动态数据"""
        if not data:
            return result
        
        # 获取步态检测结果
        result.gait_phases = self.gait_analyzer.get_gait_phases()
        result.step_frequency = self.step_detector.get_step_frequency()
        
        # 计算接触时间
        contact_times = self._calculate_contact_times(data)
        result.contact_time_avg = np.mean(contact_times) if contact_times else 0.0
        
        # 计算峰值力
        result.peak_forces = self._calculate_peak_forces(data)
        
        # 计算冲量值
        result.impulse_values = self._calculate_impulse_values(data)
        
        # 计算力上升/下降时间
        result.force_rise_times, result.force_fall_times = self._calculate_force_transition_times(data)
        
        # 稳定性分析
        result.stability_variance = self._calculate_stability_variance(data)
        result.balance_consistency = self._calculate_balance_consistency(data)
        result.coordination_index = self._calculate_coordination_index(data)
        
        # 原始数据统计
        result.raw_data_stats = self._calculate_raw_data_statistics(data)
        
        return result
    
    def _calculate_contact_times(self, data: List[FootForceData]) -> List[float]:
        """计算接触时间"""
        contact_times = []
        
        for foot_id in range(4):
            contact_periods = []
            in_contact = False
            contact_start = 0.0
            
            for sample in data:
                if sample.contact_states[foot_id] and not in_contact:
                    # 开始接触
                    in_contact = True
                    contact_start = sample.timestamp
                elif not sample.contact_states[foot_id] and in_contact:
                    # 结束接触
                    in_contact = False
                    contact_duration = sample.timestamp - contact_start
                    contact_periods.append(contact_duration)
            
            contact_times.extend(contact_periods)
        
        return contact_times
    
    def _calculate_peak_forces(self, data: List[FootForceData]) -> Dict[str, float]:
        """计算峰值力"""
        peak_forces = {}
        
        for foot_id, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = [sample.force_magnitude[foot_id] for sample in data]
            peak_forces[foot_label] = float(np.max(foot_forces)) if foot_forces else 0.0
        
        # 总峰值力
        total_forces = [sample.total_force for sample in data]
        peak_forces['total'] = float(np.max(total_forces)) if total_forces else 0.0
        
        return peak_forces
    
    def _calculate_impulse_values(self, data: List[FootForceData]) -> Dict[str, float]:
        """计算冲量值（力乘以时间）"""
        impulse_values = {}
        
        if len(data) < 2:
            return impulse_values
        
        dt = (data[-1].timestamp - data[0].timestamp) / len(data)
        
        for foot_id, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = [sample.force_magnitude[foot_id] for sample in data]
            impulse = float(np.sum(foot_forces) * dt)
            impulse_values[foot_label] = impulse
        
        return impulse_values
    
    def _calculate_force_transition_times(self, data: List[FootForceData]) -> Tuple[Dict[str, float], Dict[str, float]]:
        """计算力上升和下降时间"""
        rise_times = {}
        fall_times = {}
        
        for foot_id, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = [sample.force_magnitude[foot_id] for sample in data]
            
            # 寻找力的上升和下降边缘
            rise_time_samples = []
            fall_time_samples = []
            
            for i in range(1, len(foot_forces)):
                force_change = foot_forces[i] - foot_forces[i-1]
                
                if force_change > 5.0:  # 力快速上升
                    rise_time_samples.append(i)
                elif force_change < -5.0:  # 力快速下降
                    fall_time_samples.append(i)
            
            # 计算平均时间（简化计算）
            rise_times[foot_label] = len(rise_time_samples) * 0.002 if rise_time_samples else 0.0  # 假设500Hz采样率
            fall_times[foot_label] = len(fall_time_samples) * 0.002 if fall_time_samples else 0.0
        
        return rise_times, fall_times
    
    def _calculate_stability_variance(self, data: List[FootForceData]) -> float:
        """计算稳定性方差"""
        stability_indices = [sample.stability_index for sample in data]
        return float(np.var(stability_indices)) if stability_indices else 0.0
    
    def _calculate_balance_consistency(self, data: List[FootForceData]) -> float:
        """计算平衡一致性"""
        balance_indices = [sample.force_balance for sample in data]
        return float(np.mean(balance_indices)) if balance_indices else 0.0
    
    def _calculate_coordination_index(self, data: List[FootForceData]) -> float:
        """计算协调指数"""
        # 分析四足的相位关系
        foot_phases = [[] for _ in range(4)]
        
        for sample in data:
            for foot_id in range(4):
                # 简化的相位计算：基于力的大小
                phase = sample.force_magnitude[foot_id] / (sample.total_force + 1e-6)
                foot_phases[foot_id].append(phase)
        
        # 计算足间相关性
        correlations = []
        for i in range(4):
            for j in range(i+1, 4):
                if foot_phases[i] and foot_phases[j]:
                    corr = np.corrcoef(foot_phases[i], foot_phases[j])[0, 1]
                    if not np.isnan(corr):
                        correlations.append(abs(corr))
        
        return float(np.mean(correlations)) if correlations else 0.0
    
    def _calculate_raw_data_statistics(self, data: List[FootForceData]) -> Dict[str, Any]:
        """计算原始数据统计"""
        if not data:
            return {}
        
        total_forces = [sample.total_force for sample in data]
        cops_x = [sample.center_of_pressure[0] for sample in data]
        cops_y = [sample.center_of_pressure[1] for sample in data]
        
        return {
            'total_force_mean': float(np.mean(total_forces)),
            'total_force_std': float(np.std(total_forces)),
            'total_force_max': float(np.max(total_forces)),
            'cop_x_range': float(np.max(cops_x) - np.min(cops_x)),
            'cop_y_range': float(np.max(cops_y) - np.min(cops_y)),
            'contact_rate_avg': float(np.mean([np.mean(sample.contact_states) for sample in data])),
            'sample_count': len(data)
        }
    
    def _calculate_dynamic_test_score(self, result: DynamicTestResult, scenario: Dict) -> float:
        """计算动态测试评分"""
        score = 0.0
        max_score = 100.0
        
        # 数据质量分 (30%)
        if result.total_samples > 0:
            expected_samples = scenario.get('duration', 60) * 500  # 假设500Hz
            sample_ratio = min(result.total_samples / expected_samples, 1.0)
            score += 30.0 * sample_ratio
        
        # 步态一致性分 (25%)
        if result.coordination_index > 0:
            coordination_score = min(result.coordination_index * 100, 25.0)
            score += coordination_score
        
        # 稳定性分 (20%)
        if result.stability_variance >= 0:
            stability_score = max(0, 20.0 - result.stability_variance * 100)
            score += min(stability_score, 20.0)
        
        # 平衡性分 (15%)
        balance_score = result.balance_consistency * 15.0
        score += min(balance_score, 15.0)
        
        # 特定测试指标 (10%)
        if scenario['name'] == 'impact_test':
            # 冲击测试：检查峰值力是否合理
            max_expected_force = scenario.get('max_expected_force', 250)
            total_peak = result.peak_forces.get('total', 0)
            if 50 <= total_peak <= max_expected_force:
                score += 10.0
        else:
            # 行走测试：检查步频是否合理
            if 0.5 <= result.step_frequency <= 3.0:
                score += 10.0
        
        return min(score, max_score)
    
    def save_dynamic_test_results(self, results: Dict[str, DynamicTestResult], output_dir: str) -> str:
        """保存动态测试结果"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = Path(output_dir) / f"dynamic_test_results_{timestamp}.json"
        
        # 转换为可序列化的格式
        serializable_results = {}
        for test_name, result in results.items():
            serializable_results[test_name] = {
                'test_name': result.test_name,
                'start_time': result.start_time,
                'end_time': result.end_time,
                'duration': result.duration,
                'total_samples': result.total_samples,
                'step_frequency': result.step_frequency,
                'contact_time_avg': result.contact_time_avg,
                'peak_forces': result.peak_forces,
                'impulse_values': result.impulse_values,
                'stability_variance': result.stability_variance,
                'balance_consistency': result.balance_consistency,
                'coordination_index': result.coordination_index,
                'test_score': result.test_score,
                'raw_data_stats': result.raw_data_stats
            }
        
        # 添加汇总信息
        summary = {
            'total_tests': len(results),
            'average_score': np.mean([r.test_score for r in results.values()]) if results else 0.0,
            'test_timestamp': timestamp,
            'config_used': self.dynamic_config
        }
        
        output_data = {
            'summary': summary,
            'test_results': serializable_results
        }
        
        try:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(output_data, f, indent=2, ensure_ascii=False)
            
            self.logger.info(f"动态测试结果已保存到: {output_file}")
            return str(output_file)
            
        except Exception as e:
            self.logger.error(f"保存动态测试结果失败: {e}")
            return ""


class StepDetector:
    """步伐检测器"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.threshold = config.get('step_detection_threshold', 15.0)
        self.step_events = []
        self.last_contact_states = [False] * 4
        
    def reset(self):
        """重置检测器"""
        self.step_events.clear()
        self.last_contact_states = [False] * 4
    
    def process_data(self, data: FootForceData):
        """处理数据并检测步伐"""
        # 检测每个足端的接触变化
        for foot_id in range(4):
            current_contact = data.contact_states[foot_id]
            last_contact = self.last_contact_states[foot_id]
            
            # 检测触地事件
            if current_contact and not last_contact:
                self.step_events.append({
                    'timestamp': data.timestamp,
                    'foot_id': foot_id,
                    'event_type': 'touchdown',
                    'force': data.force_magnitude[foot_id]
                })
            
            # 检测离地事件
            elif not current_contact and last_contact:
                self.step_events.append({
                    'timestamp': data.timestamp,
                    'foot_id': foot_id,
                    'event_type': 'liftoff',
                    'force': data.force_magnitude[foot_id]
                })
        
        self.last_contact_states = data.contact_states.copy()
    
    def get_step_frequency(self) -> float:
        """计算步频"""
        touchdown_events = [e for e in self.step_events if e['event_type'] == 'touchdown']
        
        if len(touchdown_events) < 2:
            return 0.0
        
        total_time = touchdown_events[-1]['timestamp'] - touchdown_events[0]['timestamp']
        step_count = len(touchdown_events)
        
        return (step_count - 1) / total_time if total_time > 0 else 0.0


class GaitAnalyzer:
    """步态分析器"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.gait_phases = []
        self.current_phases = {}  # 跟踪每个足端的当前相位
        
    def reset(self):
        """重置分析器"""
        self.gait_phases.clear()
        self.current_phases.clear()
    
    def process_data(self, data: FootForceData):
        """处理数据并分析步态"""
        # 为每个足端分析当前相位
        for foot_id in range(4):
            self._analyze_foot_phase(foot_id, data)
    
    def _analyze_foot_phase(self, foot_id: int, data: FootForceData):
        """分析单个足端的步态相位"""
        is_contact = data.contact_states[foot_id]
        force = data.force_magnitude[foot_id]
        
        # 获取当前相位状态
        current_phase = self.current_phases.get(foot_id, None)
        
        if is_contact and force > 10.0:
            # 支撑相
            if current_phase is None or current_phase['phase_name'] != 'stance':
                # 开始新的支撑相
                if current_phase:
                    # 结束前一个相位
                    current_phase['end_time'] = data.timestamp
                    self.gait_phases.append(GaitPhase(**current_phase))
                
                # 开始新相位
                self.current_phases[foot_id] = {
                    'phase_name': 'stance',
                    'start_time': data.timestamp,
                    'end_time': 0.0,
                    'foot_id': foot_id,
                    'peak_force': force,
                    'average_force': force,
                    'force_profile': [force],
                    'contact_quality': 1.0 if is_contact else 0.0
                }
            else:
                # 继续支撑相
                current_phase['peak_force'] = max(current_phase['peak_force'], force)
                current_phase['force_profile'].append(force)
                # 更新平均力
                current_phase['average_force'] = np.mean(current_phase['force_profile'])
        
        else:
            # 摆动相
            if current_phase and current_phase['phase_name'] == 'stance':
                # 结束支撑相
                current_phase['end_time'] = data.timestamp
                self.gait_phases.append(GaitPhase(**current_phase))
                
                # 开始摆动相
                self.current_phases[foot_id] = {
                    'phase_name': 'swing',
                    'start_time': data.timestamp,
                    'end_time': 0.0,
                    'foot_id': foot_id,
                    'peak_force': 0.0,
                    'average_force': 0.0,
                    'force_profile': [0.0],
                    'contact_quality': 0.0
                }
            elif current_phase and current_phase['phase_name'] == 'swing':
                # 继续摆动相
                current_phase['force_profile'].append(0.0)
    
    def get_gait_phases(self) -> List[GaitPhase]:
        """获取步态相位列表"""
        # 结束所有正在进行的相位
        current_time = time.time()
        for foot_id, phase in self.current_phases.items():
            if phase and phase['end_time'] == 0.0:
                phase['end_time'] = current_time
                self.gait_phases.append(GaitPhase(**phase))
        
        return self.gait_phases 