#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/data_collector.py
# Generated: 2025-06-27 14:08:45 CST  
# Purpose: Unitree Go2 足端力传感器实时数据采集和处理

import time
import threading
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional, Callable
from dataclasses import dataclass, field, asdict
from collections import deque
import json
import csv
from pathlib import Path
from datetime import datetime

from foot_force_config import FootForceConfig, FootForceReading

@dataclass
class FootForceCollectionMetrics:
    """足端力数据采集指标"""
    start_time: float = 0.0
    end_time: float = 0.0
    total_samples: int = 0
    valid_samples: int = 0
    dropout_count: int = 0
    avg_sampling_rate: float = 0.0
    data_rate_std: float = 0.0
    collection_duration: float = 0.0
    
    # 足端力质量指标
    force_stats: Dict[str, Dict[str, float]] = field(default_factory=dict)  # 每个足端的力统计
    contact_rate: Dict[str, float] = field(default_factory=dict)  # 每个足端的接触率
    total_force_stats: Dict[str, float] = field(default_factory=dict)  # 总力统计
    cop_stats: Dict[str, float] = field(default_factory=dict)  # 压力中心统计
    
    # 实时统计
    real_time_fps: List[float] = field(default_factory=list)
    latency_history: List[float] = field(default_factory=list)
    
    # 数据质量评估
    data_quality_score: float = 0.0
    validation_results: Dict[str, float] = field(default_factory=dict)

@dataclass
class FootForceData:
    """足端力数据结构（用于存储）"""
    timestamp: float
    foot_forces: List[List[float]]  # 4个足端，每个3维力 [[Fx,Fy,Fz], ...]
    contact_states: List[bool]  # 4个足端的接触状态
    total_force: float
    center_of_pressure: List[float]  # [x, y]
    
    # 额外的计算字段
    force_magnitude: List[float] = field(default_factory=list)  # 每个足端的力大小
    stability_index: float = 0.0  # 稳定性指数
    force_balance: float = 0.0  # 力平衡指数

class FootForceDataCollector:
    """足端力传感器数据收集器类"""
    
    def __init__(self, config: Dict, foot_force_config: Optional[FootForceConfig] = None):
        """
        初始化足端力数据收集器
        
        Args:
            config: 配置字典
            foot_force_config: FootForceConfig实例
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.foot_force_config = foot_force_config
        
        # 数据存储
        self.raw_data: List[FootForceData] = []
        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None
        
        # 回调函数
        self.data_callbacks: List[Callable[[FootForceData], None]] = []
        
        # 数据质量监控
        self.quality_window_size = 100  # 数据质量评估窗口大小
        self.recent_validation_scores = deque(maxlen=self.quality_window_size)
        
        # 统计数据
        self.contact_event_counter = {i: 0 for i in range(4)}  # 接触事件计数
        self.force_peak_detector = {i: [] for i in range(4)}  # 力峰值检测
        
        self.logger.info("足端力数据收集器初始化完成")
    
    def add_data_callback(self, callback: Callable[[FootForceData], None]):
        """添加数据回调函数"""
        self.data_callbacks.append(callback)
    
    def _extract_foot_force_data_from_reading(self, reading: FootForceReading) -> FootForceData:
        """从FootForceReading对象中提取和计算数据"""
        
        # 计算每个足端的力大小
        force_magnitude = [np.linalg.norm(force) for force in reading.foot_forces]
        
        # 计算稳定性指数（基于力分布的标准差）
        vertical_forces = [abs(force[2]) for force in reading.foot_forces]
        if sum(vertical_forces) > 0:
            stability_index = 1.0 - (np.std(vertical_forces) / np.mean(vertical_forces))
            stability_index = max(0.0, min(1.0, stability_index))  # 限制在0-1之间
        else:
            stability_index = 0.0
        
        # 计算力平衡指数（前后左右的平衡）
        front_force = vertical_forces[0] + vertical_forces[1]  # 前左 + 前右
        rear_force = vertical_forces[2] + vertical_forces[3]   # 后左 + 后右
        left_force = vertical_forces[0] + vertical_forces[2]   # 前左 + 后左
        right_force = vertical_forces[1] + vertical_forces[3]  # 前右 + 后右
        
        total_vertical = sum(vertical_forces)
        if total_vertical > 0:
            front_rear_balance = 1.0 - abs(front_force - rear_force) / total_vertical
            left_right_balance = 1.0 - abs(left_force - right_force) / total_vertical
            force_balance = (front_rear_balance + left_right_balance) / 2.0
        else:
            force_balance = 0.0
        
        return FootForceData(
            timestamp=reading.timestamp,
            foot_forces=[list(force) for force in reading.foot_forces],
            contact_states=list(reading.contact_states),
            total_force=reading.total_force,
            center_of_pressure=list(reading.center_of_pressure),
            force_magnitude=force_magnitude,
            stability_index=stability_index,
            force_balance=force_balance
        )
    
    def _collection_worker(self):
        """数据收集工作线程"""
        self.logger.info("开始足端力数据收集")
        
        sampling_rate = self.config.get('foot_force_config', {}).get('sampling_rate_hz', 500)
        sleep_interval = 1.0 / sampling_rate
        
        last_contact_states = [False] * 4  # 记录上一次的接触状态
        
        while self.is_collecting:
            try:
                if self.foot_force_config:
                    # 从FootForceConfig获取最新读数
                    reading = self.foot_force_config.get_latest_reading()
                    if reading:
                        # 验证数据质量
                        validation_results = self.foot_force_config.validate_force_data(reading)
                        
                        # 计算验证分数
                        validation_score = sum(validation_results.values()) / len(validation_results)
                        self.recent_validation_scores.append(validation_score)
                        
                        if validation_score >= 0.75:  # 只存储高质量数据
                            foot_force_data = self._extract_foot_force_data_from_reading(reading)
                            
                            # 检测接触事件
                            self._detect_contact_events(foot_force_data.contact_states, last_contact_states)
                            last_contact_states = foot_force_data.contact_states.copy()
                            
                            # 记录力峰值
                            self._record_force_peaks(foot_force_data.force_magnitude)
                            
                            # 存储数据
                            self.raw_data.append(foot_force_data)
                            
                            # 调用回调函数
                            for callback in self.data_callbacks:
                                try:
                                    callback(foot_force_data)
                                except Exception as e:
                                    self.logger.error(f"数据回调错误: {e}")
                        else:
                            self.logger.debug(f"数据质量不足，跳过存储. 分数: {validation_score:.2f}")
                else:
                    self.logger.warning("没有FootForceConfig实例，无法收集数据")
                
                # 控制采样频率
                time.sleep(sleep_interval)
                
            except Exception as e:
                self.logger.error(f"数据收集错误: {e}")
                time.sleep(0.1)
        
        self.logger.info("足端力数据收集结束")
    
    def _detect_contact_events(self, current_contacts: List[bool], last_contacts: List[bool]):
        """检测接触事件（触地和离地）"""
        for i in range(4):
            if current_contacts[i] and not last_contacts[i]:
                # 触地事件
                self.contact_event_counter[i] += 1
                self.logger.debug(f"足端{i}触地事件")
    
    def _record_force_peaks(self, force_magnitudes: List[float]):
        """记录力峰值"""
        for i, magnitude in enumerate(force_magnitudes):
            # 简单的峰值检测：保持最近的峰值
            self.force_peak_detector[i].append(magnitude)
            if len(self.force_peak_detector[i]) > 50:  # 保持最近50个样本
                self.force_peak_detector[i].pop(0)
    
    def start_collection(self, duration_seconds: Optional[float] = None) -> bool:
        """
        开始数据收集
        
        Args:
            duration_seconds: 可选的收集持续时间（秒）
        """
        try:
            if not self.foot_force_config:
                self.logger.error("没有FootForceConfig实例，无法开始数据收集")
                return False
            
            if not self.foot_force_config.is_initialized:
                self.logger.error("FootForceConfig未初始化，无法开始数据收集")
                return False
            
            # 清空数据
            self.raw_data.clear()
            self.recent_validation_scores.clear()
            self.contact_event_counter = {i: 0 for i in range(4)}
            self.force_peak_detector = {i: [] for i in range(4)}
            
            # 启动收集线程
            self.is_collecting = True
            self.collection_thread = threading.Thread(target=self._collection_worker)
            self.collection_thread.daemon = True
            self.collection_thread.start()
            
            if duration_seconds is not None:
                self.logger.info(f"足端力数据收集已启动，持续时间: {duration_seconds}秒")
                
                # 启动定时停止线程
                def auto_stop():
                    time.sleep(duration_seconds)
                    if self.is_collecting:
                        self.stop_collection()
                        self.logger.info(f"自动停止数据收集，已收集 {duration_seconds} 秒")
                
                auto_stop_thread = threading.Thread(target=auto_stop)
                auto_stop_thread.daemon = True
                auto_stop_thread.start()
            else:
                self.logger.info("足端力数据收集已启动，手动停止模式")
            
            return True
            
        except Exception as e:
            self.logger.error(f"启动数据收集失败: {e}")
            return False
    
    def stop_collection(self) -> FootForceCollectionMetrics:
        """停止数据收集并返回采集指标"""
        try:
            # 停止收集线程
            self.is_collecting = False
            if self.collection_thread:
                self.collection_thread.join(timeout=5.0)
            
            # 计算采集指标
            metrics = self._calculate_collection_metrics()
            
            self.logger.info("足端力数据收集已停止")
            return metrics
            
        except Exception as e:
            self.logger.error(f"停止数据收集失败: {e}")
            return FootForceCollectionMetrics()
    
    def _calculate_collection_metrics(self) -> FootForceCollectionMetrics:
        """计算采集指标"""
        metrics = FootForceCollectionMetrics()
        
        if self.raw_data:
            metrics.start_time = self.raw_data[0].timestamp
            metrics.end_time = self.raw_data[-1].timestamp
            metrics.total_samples = len(self.raw_data)
            metrics.valid_samples = len(self.raw_data)
            metrics.collection_duration = metrics.end_time - metrics.start_time
            
            if metrics.collection_duration > 0:
                metrics.avg_sampling_rate = metrics.total_samples / metrics.collection_duration
            
            # 计算足端力统计
            try:
                # 准备数据数组
                all_forces = np.array([data.foot_forces for data in self.raw_data])  # (samples, 4_feet, 3_axes)
                all_contacts = np.array([data.contact_states for data in self.raw_data])  # (samples, 4_feet)
                total_forces = np.array([data.total_force for data in self.raw_data])
                cops = np.array([data.center_of_pressure for data in self.raw_data])  # (samples, 2)
                
                # 每个足端的力统计
                for i in range(4):
                    foot_label = FootForceConfig.FOOT_LABELS[i]
                    foot_forces = all_forces[:, i, :]  # (samples, 3_axes)
                    
                    metrics.force_stats[foot_label] = {
                        'mean_fx': float(np.mean(foot_forces[:, 0])),
                        'mean_fy': float(np.mean(foot_forces[:, 1])),
                        'mean_fz': float(np.mean(foot_forces[:, 2])),
                        'std_fx': float(np.std(foot_forces[:, 0])),
                        'std_fy': float(np.std(foot_forces[:, 1])),
                        'std_fz': float(np.std(foot_forces[:, 2])),
                        'max_magnitude': float(np.max([np.linalg.norm(force) for force in foot_forces])),
                        'contact_events': self.contact_event_counter[i]
                    }
                    
                    # 接触率
                    metrics.contact_rate[foot_label] = float(np.mean(all_contacts[:, i]))
                
                # 总力统计
                metrics.total_force_stats = {
                    'mean': float(np.mean(total_forces)),
                    'std': float(np.std(total_forces)),
                    'min': float(np.min(total_forces)),
                    'max': float(np.max(total_forces))
                }
                
                # 压力中心统计
                metrics.cop_stats = {
                    'mean_x': float(np.mean(cops[:, 0])),
                    'mean_y': float(np.mean(cops[:, 1])),
                    'std_x': float(np.std(cops[:, 0])),
                    'std_y': float(np.std(cops[:, 1])),
                    'range_x': float(np.max(cops[:, 0]) - np.min(cops[:, 0])),
                    'range_y': float(np.max(cops[:, 1]) - np.min(cops[:, 1]))
                }
                
                # 数据质量评分
                if self.recent_validation_scores:
                    metrics.data_quality_score = float(np.mean(self.recent_validation_scores))
                    metrics.validation_results = {
                        'avg_quality': metrics.data_quality_score,
                        'min_quality': float(min(self.recent_validation_scores)),
                        'quality_std': float(np.std(self.recent_validation_scores))
                    }
                
            except Exception as e:
                self.logger.error(f"计算足端力统计失败: {e}")
        
        return metrics
    
    def get_collected_data(self) -> List[FootForceReading]:
        """获取收集的数据并转换为FootForceReading格式"""
        readings = []
        for data in self.raw_data:
            reading = FootForceReading(
                timestamp=data.timestamp,
                foot_forces=tuple(tuple(force) for force in data.foot_forces),
                contact_states=tuple(data.contact_states),
                total_force=data.total_force,
                center_of_pressure=tuple(data.center_of_pressure)
            )
            readings.append(reading)
        
        return readings
    
    def get_real_time_metrics(self) -> Dict[str, Any]:
        """获取实时采集指标"""
        if not self.raw_data:
            return {
                'current_fps': 0.0, 
                'total_samples': 0,
                'data_quality': 0.0,
                'contact_summary': [False] * 4
            }
        
        # 计算当前采样率
        recent_count = min(100, len(self.raw_data))
        if recent_count > 1:
            recent_data = self.raw_data[-recent_count:]
            time_span = recent_data[-1].timestamp - recent_data[0].timestamp
            current_fps = (recent_count - 1) / time_span if time_span > 0 else 0
        else:
            current_fps = 0
        
        # 最新接触状态
        latest_contacts = self.raw_data[-1].contact_states if self.raw_data else [False] * 4
        
        # 最新数据质量
        current_quality = self.recent_validation_scores[-1] if self.recent_validation_scores else 0.0
        
        return {
            'current_fps': current_fps,
            'total_samples': len(self.raw_data),
            'data_quality': current_quality,
            'contact_summary': latest_contacts,
            'last_timestamp': self.raw_data[-1].timestamp if self.raw_data else 0,
            'total_force': self.raw_data[-1].total_force if self.raw_data else 0.0,
            'cop': self.raw_data[-1].center_of_pressure if self.raw_data else [0.0, 0.0],
            'stability_index': self.raw_data[-1].stability_index if self.raw_data else 0.0
        }
    
    def get_data(self) -> List[FootForceData]:
        """获取收集的数据"""
        return self.raw_data.copy()
    
    def get_latest_data(self) -> Optional[FootForceData]:
        """获取最新的足端力数据"""
        if self.raw_data:
            return self.raw_data[-1]
        return None
    
    def save_data(self, filepath: str) -> bool:
        """保存数据到文件"""
        try:
            data_dict = [asdict(data) for data in self.raw_data]
            
            # 添加元数据
            metadata = {
                'total_samples': len(self.raw_data),
                'collection_config': self.config,
                'timestamp': time.time(),
                'duration_seconds': (self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 0,
                'contact_events': self.contact_event_counter,
                'data_quality_avg': float(np.mean(self.recent_validation_scores)) if self.recent_validation_scores else 0.0
            }
            
            with open(filepath, 'w') as f:
                json.dump({
                    'metadata': metadata,
                    'data': data_dict
                }, f, indent=2)
            
            self.logger.info(f"足端力数据已保存到: {filepath}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存数据失败: {e}")
            return False
    
    def save_data_csv(self, filepath: str) -> bool:
        """保存数据到CSV文件"""
        try:
            with open(filepath, 'w', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'total_force', 'cop_x', 'cop_y', 'stability_index', 'force_balance'
                ]
                
                # 添加每个足端的字段
                for i, label in enumerate(FootForceConfig.FOOT_LABELS):
                    fieldnames.extend([
                        f'{label}_fx', f'{label}_fy', f'{label}_fz', 
                        f'{label}_contact', f'{label}_magnitude'
                    ])
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for data in self.raw_data:
                    row = {
                        'timestamp': data.timestamp,
                        'total_force': data.total_force,
                        'cop_x': data.center_of_pressure[0],
                        'cop_y': data.center_of_pressure[1],
                        'stability_index': data.stability_index,
                        'force_balance': data.force_balance
                    }
                    
                    # 添加每个足端的数据
                    for i, label in enumerate(FootForceConfig.FOOT_LABELS):
                        row[f'{label}_fx'] = data.foot_forces[i][0]
                        row[f'{label}_fy'] = data.foot_forces[i][1]
                        row[f'{label}_fz'] = data.foot_forces[i][2]
                        row[f'{label}_contact'] = data.contact_states[i]
                        row[f'{label}_magnitude'] = data.force_magnitude[i]
                    
                    writer.writerow(row)
            
            self.logger.info(f"足端力数据已保存到CSV: {filepath}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存CSV数据失败: {e}")
            return False
    
    def get_statistics(self) -> Dict:
        """获取数据统计信息"""
        if not self.raw_data:
            return {}
        
        # 准备数据数组
        all_forces = np.array([data.foot_forces for data in self.raw_data])
        total_forces = np.array([data.total_force for data in self.raw_data])
        cops = np.array([data.center_of_pressure for data in self.raw_data])
        stability_indices = np.array([data.stability_index for data in self.raw_data])
        force_balances = np.array([data.force_balance for data in self.raw_data])
        
        stats = {
            'total_samples': len(self.raw_data),
            'duration_seconds': (self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 0,
            'avg_sampling_rate': len(self.raw_data) / ((self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 1),
            
            'total_force_stats': {
                'mean': float(np.mean(total_forces)),
                'std': float(np.std(total_forces)),
                'min': float(np.min(total_forces)),
                'max': float(np.max(total_forces))
            },
            
            'cop_stats': {
                'mean_x': float(np.mean(cops[:, 0])),
                'mean_y': float(np.mean(cops[:, 1])),
                'std_x': float(np.std(cops[:, 0])),
                'std_y': float(np.std(cops[:, 1]))
            },
            
            'stability_stats': {
                'mean': float(np.mean(stability_indices)),
                'std': float(np.std(stability_indices)),
                'min': float(np.min(stability_indices)),
                'max': float(np.max(stability_indices))
            },
            
            'balance_stats': {
                'mean': float(np.mean(force_balances)),
                'std': float(np.std(force_balances)),
                'min': float(np.min(force_balances)),
                'max': float(np.max(force_balances))
            },
            
            'contact_events': self.contact_event_counter,
            'data_quality': {
                'avg_score': float(np.mean(self.recent_validation_scores)) if self.recent_validation_scores else 0.0,
                'min_score': float(min(self.recent_validation_scores)) if self.recent_validation_scores else 0.0,
                'score_std': float(np.std(self.recent_validation_scores)) if self.recent_validation_scores else 0.0
            }
        }
        
        # 每个足端的详细统计
        stats['foot_stats'] = {}
        for i in range(4):
            foot_label = FootForceConfig.FOOT_LABELS[i]
            foot_forces = all_forces[:, i, :]
            
            stats['foot_stats'][foot_label] = {
                'force_mean': foot_forces.mean(axis=0).tolist(),
                'force_std': foot_forces.std(axis=0).tolist(),
                'force_min': foot_forces.min(axis=0).tolist(),
                'force_max': foot_forces.max(axis=0).tolist(),
                'contact_rate': float(np.mean([data.contact_states[i] for data in self.raw_data])),
                'contact_events': self.contact_event_counter[i],
                'peak_force': float(max(self.force_peak_detector[i])) if self.force_peak_detector[i] else 0.0
            }
        
        return stats 