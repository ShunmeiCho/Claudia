#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/imu_config.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU配置和初始化管理

import time
import json
import logging
import numpy as np
import threading
from typing import Dict, Tuple, Optional, Any, List
from dataclasses import dataclass
from collections import deque
import os

@dataclass
class IMUSpec:
    """IMU规格数据类"""
    sampling_rate_hz: float
    accelerometer_range: str
    gyroscope_range: str
    orientation_format: str
    data_format: str
    noise_density: str

@dataclass
class IMUReading:
    """IMU读数数据类"""
    timestamp: float
    quaternion: Tuple[float, float, float, float]  # w, x, y, z
    gyroscope: Tuple[float, float, float]          # rad/s
    accelerometer: Tuple[float, float, float]      # m/s²
    temperature: Optional[float] = None            # °C (if available)

class IMUConfig:
    """IMU配置和管理类"""
    
    def __init__(self, config_input = None):
        """
        初始化IMU配置
        
        Args:
            config_input: 配置文件路径(str)或配置字典(dict)
        """
        self.logger = logging.getLogger(__name__)
        
        # 加载配置
        if isinstance(config_input, dict):
            self.config = config_input
        else:
            self.config = self._load_config(config_input)
        
        # IMU连接状态
        self.is_initialized = False
        self.connection_active = False
        
        # unitree_sdk2py相关
        self.channel_factory = None
        self.lowstate_subscriber = None
        self.latest_reading = None
        
        # 数据缓存
        self.data_buffer = deque(maxlen=self.config.get("imu_config", {}).get("data_buffer_size", 1000))
        self.buffer_lock = threading.Lock()
        
        # IMU规格
        self.target_spec = IMUSpec(
            sampling_rate_hz=self.config.get("imu_config", {}).get("sampling_rate_hz", 100),
            accelerometer_range="±16g",
            gyroscope_range="±2000°/s", 
            orientation_format="quaternion",
            data_format="float32",
            noise_density="typical robotics IMU"
        )
        
        self.actual_spec = None
        
    def _load_config(self, config_path: Optional[str]) -> Dict[str, Any]:
        """加载配置文件"""
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                self.logger.error(f"配置文件加载失败: {e}")
        
        # 返回默认配置
        return {
            "imu_config": {
                "sampling_rate_hz": 100,
                "timeout_seconds": 10,
                "network_interface": "eth0",
                "data_buffer_size": 1000
            }
        }
    
    def initialize_imu(self, method: str = "unitree_sdk2py") -> bool:
        """
        初始化IMU连接
        
        Args:
            method: 初始化方法 ("unitree_sdk2py", "simulation")
            
        Returns:
            bool: 初始化是否成功
        """
        try:
            if method == "unitree_sdk2py":
                return self._initialize_with_unitree_sdk()
            elif method == "simulation":
                return self._initialize_simulation_mode()
            else:
                self.logger.error(f"不支持的初始化方法: {method}")
                return False
                
        except Exception as e:
            self.logger.error(f"IMU初始化失败: {e}")
            # 如果unitree_sdk2py失败，尝试模拟模式
            if method == "unitree_sdk2py":
                self.logger.warning("unitree_sdk2py初始化失败，尝试模拟模式...")
                return self._initialize_simulation_mode()
            return False
    
    def _initialize_with_unitree_sdk(self) -> bool:
        """使用unitree_sdk2py初始化IMU"""
        try:
            # 导入unitree_sdk2py模块
            from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
            
            self.logger.info("导入unitree_sdk2py模块成功")
            
            # 初始化通道工厂
            network_interface = self.config.get("imu_config", {}).get("network_interface", "eth0")
            self.logger.info(f"初始化DDS通道工厂，网卡: {network_interface}")
            
            ChannelFactoryInitialize(0, network_interface)
            self.logger.info("DDS通道工厂初始化成功")
            
            # 创建LowState订阅者
            self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self.lowstate_subscriber.Init(self._lowstate_callback, 10)
            
            self.logger.info("LowState订阅者创建成功")
            
            # 等待首次数据接收
            timeout = self.config.get("imu_config", {}).get("timeout_seconds", 10)
            start_time = time.time()
            
            while not self.latest_reading and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.latest_reading:
                self.is_initialized = True
                self.connection_active = True
                
                # 创建实际规格
                self.actual_spec = IMUSpec(
                    sampling_rate_hz=self._estimate_sampling_rate(),
                    accelerometer_range="±16g (estimated)",
                    gyroscope_range="±2000°/s (estimated)",
                    orientation_format="quaternion",
                    data_format="float32",
                    noise_density="unitree_sdk2py"
                )
                
                self.logger.info("IMU初始化成功")
                return True
            else:
                self.logger.error("IMU初始化超时，未收到数据")
                return False
                
        except ImportError as e:
            self.logger.error(f"unitree_sdk2py模块导入失败: {e}")
            return False
        except Exception as e:
            self.logger.error(f"IMU初始化异常: {e}")
            return False
    
    def _lowstate_callback(self, msg):
        """LowState消息回调函数"""
        try:
            timestamp = time.time()
            
            # 提取IMU数据
            reading = IMUReading(
                timestamp=timestamp,
                quaternion=(
                    msg.imu_state.quaternion[0],  # w
                    msg.imu_state.quaternion[1],  # x
                    msg.imu_state.quaternion[2],  # y
                    msg.imu_state.quaternion[3]   # z
                ),
                gyroscope=(
                    msg.imu_state.gyroscope[0],   # x (rad/s)
                    msg.imu_state.gyroscope[1],   # y (rad/s)
                    msg.imu_state.gyroscope[2]    # z (rad/s)
                ),
                accelerometer=(
                    msg.imu_state.accelerometer[0],  # x (m/s²)
                    msg.imu_state.accelerometer[1],  # y (m/s²)
                    msg.imu_state.accelerometer[2]   # z (m/s²)
                )
            )
            
            # 更新最新读数
            self.latest_reading = reading
            
            # 添加到缓存
            with self.buffer_lock:
                self.data_buffer.append(reading)
                
        except Exception as e:
            self.logger.error(f"IMU数据处理错误: {e}")
    
    def get_latest_reading(self) -> Optional[IMUReading]:
        """获取最新的IMU读数"""
        return self.latest_reading
    
    def get_buffered_data(self, num_samples: int = None) -> List[IMUReading]:
        """
        获取缓存的IMU数据
        
        Args:
            num_samples: 获取的样本数，None表示全部
            
        Returns:
            List[IMUReading]: IMU读数列表
        """
        with self.buffer_lock:
            if num_samples is None:
                return list(self.data_buffer)
            else:
                return list(self.data_buffer)[-num_samples:]
    
    def clear_buffer(self):
        """清空数据缓存"""
        with self.buffer_lock:
            self.data_buffer.clear()
    
    def _estimate_sampling_rate(self) -> float:
        """估算实际采样率"""
        try:
            # 收集一小段时间的数据来估算采样率
            initial_count = len(self.data_buffer)
            time.sleep(2.0)  # 等待2秒
            final_count = len(self.data_buffer)
            
            if final_count > initial_count:
                estimated_rate = (final_count - initial_count) / 2.0
                return estimated_rate
            else:
                return self.target_spec.sampling_rate_hz
                
        except Exception:
            return self.target_spec.sampling_rate_hz
    
    def get_imu_properties(self) -> Dict[str, Any]:
        """获取IMU属性信息"""
        if not self.is_initialized:
            return {"status": "not_initialized"}
        
        latest = self.latest_reading
        if not latest:
            return {"status": "no_data"}
        
        return {
            "status": "active",
            "connection_active": self.connection_active,
            "latest_timestamp": latest.timestamp,
            "buffer_size": len(self.data_buffer),
            "target_spec": self.target_spec.__dict__,
            "actual_spec": self.actual_spec.__dict__ if self.actual_spec else None,
            "latest_reading": {
                "quaternion": latest.quaternion,
                "gyroscope": latest.gyroscope,
                "accelerometer": latest.accelerometer,
                "magnitude_accel": np.linalg.norm(latest.accelerometer),
                "magnitude_gyro": np.linalg.norm(latest.gyroscope)
            }
        }
    
    def check_data_integrity(self) -> Dict[str, Any]:
        """检查数据完整性"""
        if not self.is_initialized or len(self.data_buffer) < 10:
            return {"status": "insufficient_data"}
        
        data = self.get_buffered_data()
        timestamps = [reading.timestamp for reading in data]
        
        # 计算数据率
        if len(timestamps) > 1:
            time_diffs = np.diff(timestamps)
            avg_interval = np.mean(time_diffs)
            actual_rate = 1.0 / avg_interval if avg_interval > 0 else 0
            
            # 检查丢帧
            expected_interval = 1.0 / self.target_spec.sampling_rate_hz
            dropout_count = sum(1 for diff in time_diffs if diff > expected_interval * 1.5)
            dropout_rate = dropout_count / len(time_diffs)
        else:
            actual_rate = 0
            dropout_rate = 0
        
        # 检查数据有效性
        valid_readings = 0
        for reading in data:
            if (all(np.isfinite(reading.quaternion)) and
                all(np.isfinite(reading.gyroscope)) and
                all(np.isfinite(reading.accelerometer))):
                valid_readings += 1
        
        data_validity = valid_readings / len(data) if data else 0
        
        return {
            "status": "analyzed",
            "sample_count": len(data),
            "actual_sampling_rate": actual_rate,
            "target_sampling_rate": self.target_spec.sampling_rate_hz,
            "dropout_rate": dropout_rate,
            "data_validity": data_validity,
            "time_span_seconds": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
        }
    
    def quaternion_to_euler(self, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        """
        四元数转欧拉角 (roll, pitch, yaw)
        
        Args:
            q: 四元数 (w, x, y, z)
            
        Returns:
            Tuple[float, float, float]: 欧拉角 (roll, pitch, yaw) 弧度
        """
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def release(self):
        """释放IMU资源"""
        try:
            self.connection_active = False
            self.is_initialized = False
            
            if self.lowstate_subscriber:
                # unitree_sdk2py的订阅者通常会自动清理
                self.lowstate_subscriber = None
            
            self.logger.info("IMU资源已释放")
            
        except Exception as e:
            self.logger.error(f"释放IMU资源时出错: {e}") 