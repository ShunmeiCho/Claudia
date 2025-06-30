#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/data_collector.py
# Generated: 2025-06-27 11:54:45 CST  
# Purpose: Unitree Go2 IMU实时数据采集和处理

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

# Unitree SDK2 imports - 使用正确的导入路径
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

from imu_config import IMUConfig, IMUReading

@dataclass
class CollectionMetrics:
    """数据采集指标"""
    start_time: float = 0.0
    end_time: float = 0.0
    total_samples: int = 0
    valid_samples: int = 0
    dropout_count: int = 0
    avg_sampling_rate: float = 0.0
    data_rate_std: float = 0.0
    collection_duration: float = 0.0
    
    # 数据质量指标
    accelerometer_stats: Dict[str, float] = field(default_factory=dict)
    gyroscope_stats: Dict[str, float] = field(default_factory=dict)
    quaternion_stats: Dict[str, float] = field(default_factory=dict)
    
    # 实时统计
    real_time_fps: List[float] = field(default_factory=list)
    latency_history: List[float] = field(default_factory=list)

@dataclass
class IMUData:
    """IMU数据结构"""
    timestamp: float
    quaternion: List[float]  # [w, x, y, z]
    gyroscope: List[float]   # [x, y, z] rad/s
    accelerometer: List[float]  # [x, y, z] m/s²
    temperature: int

class IMUDataCollector:
    """IMU数据收集器类"""
    
    def __init__(self, config: Dict, imu_config=None):
        """
        初始化IMU数据收集器
        
        Args:
            config: 配置字典
            imu_config: IMUConfig实例，如果提供则使用其连接
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.imu_config = imu_config
        
        # 数据存储
        self.raw_data: List[IMUData] = []
        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None
        
        # 回调函数
        self.data_callbacks: List[Callable[[IMUData], None]] = []
        
        self.logger.info("IMU数据收集器初始化完成")
    
    def add_data_callback(self, callback: Callable[[IMUData], None]):
        """添加数据回调函数"""
        self.data_callbacks.append(callback)
    
    def _extract_imu_data_from_reading(self, reading) -> IMUData:
        """从IMUReading对象中提取数据"""
        return IMUData(
            timestamp=reading.timestamp,
            quaternion=list(reading.quaternion),
            gyroscope=list(reading.gyroscope),
            accelerometer=list(reading.accelerometer),
            temperature=getattr(reading, 'temperature', 0)
        )
    
    def _collection_worker(self):
        """数据收集工作线程 - 使用IMUConfig的数据"""
        self.logger.info("开始IMU数据收集")
        
        sampling_rate = self.config.get('imu_config', {}).get('sampling_rate_hz', 100)
        sleep_interval = 1.0 / sampling_rate
        
        while self.is_collecting:
            try:
                if self.imu_config:
                    # 从IMUConfig获取最新读数
                    reading = self.imu_config.get_latest_reading()
                    if reading:
                        imu_data = self._extract_imu_data_from_reading(reading)
                        
                        # 存储数据
                        self.raw_data.append(imu_data)
                        
                        # 调用回调函数
                        for callback in self.data_callbacks:
                            try:
                                callback(imu_data)
                            except Exception as e:
                                self.logger.error(f"数据回调错误: {e}")
                else:
                    self.logger.warning("没有IMUConfig实例，无法收集数据")
                
                # 控制采样频率
                time.sleep(sleep_interval)
                
            except Exception as e:
                self.logger.error(f"数据收集错误: {e}")
                time.sleep(0.1)
        
        self.logger.info("IMU数据收集结束")
    
    def start_collection(self, duration_seconds: Optional[float] = None) -> bool:
        """
        开始数据收集
        
        Args:
            duration_seconds: 可选的收集持续时间（秒），如果不指定则持续收集直到手动停止
        """
        try:
            if not self.imu_config:
                self.logger.error("没有IMUConfig实例，无法开始数据收集")
                return False
            
            if not self.imu_config.is_initialized:
                self.logger.error("IMUConfig未初始化，无法开始数据收集")
                return False
            
            # 清空数据
            self.raw_data.clear()
            
            # 启动收集线程
            self.is_collecting = True
            self.collection_thread = threading.Thread(target=self._collection_worker)
            self.collection_thread.start()
            
            if duration_seconds is not None:
                self.logger.info(f"IMU数据收集已启动，持续时间: {duration_seconds}秒")
                
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
                self.logger.info("IMU数据收集已启动，手动停止模式")
            
            return True
            
        except Exception as e:
            self.logger.error(f"启动数据收集失败: {e}")
            return False
    
    def stop_collection(self) -> CollectionMetrics:
        """停止数据收集并返回采集指标"""
        try:
            # 停止收集线程
            self.is_collecting = False
            if self.collection_thread:
                self.collection_thread.join(timeout=5.0)
            
            # 计算采集指标
            metrics = self._calculate_collection_metrics()
            
            self.logger.info("IMU数据收集已停止")
            return metrics
            
        except Exception as e:
            self.logger.error(f"停止数据收集失败: {e}")
            # 返回空的metrics对象
            return CollectionMetrics()
    
    def _calculate_collection_metrics(self) -> CollectionMetrics:
        """计算采集指标"""
        metrics = CollectionMetrics()
        
        if self.raw_data:
            metrics.start_time = self.raw_data[0].timestamp
            metrics.end_time = self.raw_data[-1].timestamp
            metrics.total_samples = len(self.raw_data)
            metrics.valid_samples = len(self.raw_data)  # 假设所有数据都有效
            metrics.collection_duration = metrics.end_time - metrics.start_time
            
            if metrics.collection_duration > 0:
                metrics.avg_sampling_rate = metrics.total_samples / metrics.collection_duration
            
            # 简化的数据质量统计
            try:
                accelerometers = np.array([data.accelerometer for data in self.raw_data])
                gyroscopes = np.array([data.gyroscope for data in self.raw_data])
                quaternions = np.array([data.quaternion for data in self.raw_data])
                
                metrics.accelerometer_stats = {
                    'mean': accelerometers.mean(axis=0).tolist(),
                    'std': accelerometers.std(axis=0).tolist()
                }
                metrics.gyroscope_stats = {
                    'mean': gyroscopes.mean(axis=0).tolist(),
                    'std': gyroscopes.std(axis=0).tolist()
                }
                metrics.quaternion_stats = {
                    'mean': quaternions.mean(axis=0).tolist(),
                    'std': quaternions.std(axis=0).tolist()
                }
            except Exception as e:
                self.logger.error(f"计算数据质量统计失败: {e}")
        
        return metrics
    
    def get_collected_data(self) -> List[IMUReading]:
        """获取收集的数据并转换为IMUReading格式"""
        from imu_config import IMUReading
        
        readings = []
        for data in self.raw_data:
            # 确保数据长度正确
            accel = data.accelerometer[:3] if len(data.accelerometer) >= 3 else data.accelerometer + [0.0] * (3 - len(data.accelerometer))
            gyro = data.gyroscope[:3] if len(data.gyroscope) >= 3 else data.gyroscope + [0.0] * (3 - len(data.gyroscope))
            quat = data.quaternion[:4] if len(data.quaternion) >= 4 else data.quaternion + [0.0] * (4 - len(data.quaternion))
            
            reading = IMUReading(
                timestamp=data.timestamp,
                accelerometer=(accel[0], accel[1], accel[2]),
                gyroscope=(gyro[0], gyro[1], gyro[2]),
                quaternion=(quat[0], quat[1], quat[2], quat[3]),
                temperature=data.temperature
            )
            readings.append(reading)
        
        return readings
    
    def get_real_time_metrics(self) -> Dict[str, float]:
        """获取实时采集指标"""
        if not self.raw_data:
            return {'current_fps': 0.0, 'total_samples': 0}
        
        # 计算当前采样率（基于最近的样本）
        recent_count = min(100, len(self.raw_data))
        if recent_count > 1:
            recent_data = self.raw_data[-recent_count:]
            time_span = recent_data[-1].timestamp - recent_data[0].timestamp
            current_fps = (recent_count - 1) / time_span if time_span > 0 else 0
        else:
            current_fps = 0
        
        return {
            'current_fps': current_fps,
            'total_samples': len(self.raw_data),
            'last_timestamp': self.raw_data[-1].timestamp if self.raw_data else 0
        }
    
    def get_data(self) -> List[IMUData]:
        """获取收集的数据"""
        return self.raw_data.copy()
    
    def get_latest_data(self) -> Optional[IMUData]:
        """获取最新的IMU数据"""
        if self.raw_data:
            return self.raw_data[-1]
        return None
    
    def save_data(self, filepath: str) -> bool:
        """保存数据到文件"""
        try:
            data_dict = [asdict(data) for data in self.raw_data]
            
            with open(filepath, 'w') as f:
                json.dump({
                    'metadata': {
                        'total_samples': len(self.raw_data),
                        'collection_config': self.config,
                        'timestamp': time.time()
                    },
                    'data': data_dict
                }, f, indent=2)
            
            self.logger.info(f"数据已保存到: {filepath}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存数据失败: {e}")
            return False
    
    def get_statistics(self) -> Dict:
        """获取数据统计信息"""
        if not self.raw_data:
            return {}
        
        # 提取数据数组
        quaternions = np.array([data.quaternion for data in self.raw_data])
        gyroscopes = np.array([data.gyroscope for data in self.raw_data])
        accelerometers = np.array([data.accelerometer for data in self.raw_data])
        temperatures = np.array([data.temperature for data in self.raw_data])
        
        return {
            'total_samples': len(self.raw_data),
            'duration_seconds': (self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 0,
            'quaternion_stats': {
                'mean': quaternions.mean(axis=0).tolist(),
                'std': quaternions.std(axis=0).tolist(),
                'min': quaternions.min(axis=0).tolist(),
                'max': quaternions.max(axis=0).tolist()
            },
            'gyroscope_stats': {
                'mean': gyroscopes.mean(axis=0).tolist(),
                'std': gyroscopes.std(axis=0).tolist(),
                'min': gyroscopes.min(axis=0).tolist(),
                'max': gyroscopes.max(axis=0).tolist()
            },
            'accelerometer_stats': {
                'mean': accelerometers.mean(axis=0).tolist(),
                'std': accelerometers.std(axis=0).tolist(),
                'min': accelerometers.min(axis=0).tolist(),
                'max': accelerometers.max(axis=0).tolist()
            },
            'temperature_stats': {
                'mean': float(temperatures.mean()),
                'std': float(temperatures.std()),
                'min': int(temperatures.min()),
                'max': int(temperatures.max())
            }
        } 