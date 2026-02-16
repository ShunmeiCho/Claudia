#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia系统状态监控器 - 实时监控Unitree Go2系统状态
监控系统关键状态，为LED控制提供优先级决策依据

Author: Claudia AI System  
Generated: 2025-06-30
Purpose: 子任务6.4 - 系统默认兼容性和安全机制
Version: 0.4.0 (System Compatibility Enhancement)
"""

import sys
import time
import threading
import logging
import json
from typing import Dict, Optional, Callable, Any, List, Tuple
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from collections import deque
import statistics

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String, Float32, Bool
    # Unitree消息类型 (根据实际SDK调整)
    try:
        from unitree_go.msg import LowState, SportModeState, BmsState
        UNITREE_MSGS_AVAILABLE = True
    except ImportError:
        # 备用消息定义
        UNITREE_MSGS_AVAILABLE = False
        # 不打印警告 — 硬件模式使用 SDKStateProvider 获取状态，
        # 不依赖 ROS2 消息类型。仅在实际需要 ROS2 监控时才报错。
        logging.getLogger("SystemStateMonitor").debug(
            "Unitree ROS2 消息类型不可用（LowState/SportModeState/BmsState），"
            "硬件模式将使用 SDK 直连"
        )
    
    ROS2_AVAILABLE = True
except ImportError as e:
    logging.getLogger("SystemStateMonitor").debug("ROS2 导入失败: %s", e)
    ROS2_AVAILABLE = False
    # ROS2 不可用时的占位定义，使 SystemMonitorNode 类定义不报 NameError
    Node = object
    String = Float32 = Bool = object
    UNITREE_MSGS_AVAILABLE = False

class SystemState(IntEnum):
    """系统状态枚举（按优先级排序）"""
    UNKNOWN = 0                    # 未知状态
    NORMAL = 1                     # 正常运行
    IDLE = 2                       # 空闲状态
    ACTIVE = 3                     # 活跃状态
    CALIBRATING = 5                # 校准中
    LOW_BATTERY = 7                # 电池电量低
    ERROR = 8                      # 系统错误
    EMERGENCY = 10                 # 紧急状态

class SystemLEDPriority(IntEnum):
    """系统LED优先级映射"""
    NORMAL = 1                     # 正常状态
    USER_INTERACTION = 5           # 用户交互
    SYSTEM_NOTIFICATION = 7        # 系统通知 
    SAFETY_WARNING = 8             # 安全警告
    CRITICAL_ERROR = 10            # 关键错误

@dataclass 
class SystemStateInfo:
    """系统状态信息"""
    state: SystemState
    priority: SystemLEDPriority
    battery_level: float           # 电池电量 (0-1)
    battery_voltage: float         # 电池电压
    is_charging: bool              # 是否充电中
    error_codes: List[int]         # 错误代码列表
    temperature: float             # 系统温度
    timestamp: float               # 时间戳
    
    # 运动状态
    is_standing: bool = False      # 是否站立
    is_moving: bool = False        # 是否移动中
    current_gait: str = "unknown"  # 当前步态
    
    # 网络和通信
    network_status: str = "unknown"  # 网络状态
    sdk_connection: bool = True      # SDK连接状态
    
    # 元数据
    confidence: float = 1.0        # 状态置信度
    source: str = "system_monitor" # 数据来源

@dataclass
class LEDControlDecision:
    """LED控制决策"""
    allow_custom_control: bool     # 是否允许自定义LED控制
    required_priority: SystemLEDPriority  # 要求的最低优先级
    system_override_active: bool   # 系统是否强制覆盖
    recommended_action: str        # 推荐操作
    reason: str                   # 决策原因
    
class SystemStateMonitor:
    """系统状态监控器"""
    
    def __init__(self, 
                 node_name: str = "claudia_system_monitor",
                 history_size: int = 50,
                 update_rate: float = 10.0):
        """
        初始化系统状态监控器
        
        Args:
            node_name: ROS2节点名称
            history_size: 历史数据缓存大小
            update_rate: 更新频率 (Hz)
        """
        self.logger = logging.getLogger(__name__)
        self.node_name = node_name
        self.history_size = history_size
        self.update_rate = update_rate
        
        # ROS2组件
        self.node = None
        self.executor = None
        self.executor_thread = None
        self.is_ros_initialized = False
        
        # 状态数据
        self.current_state_info = SystemStateInfo(
            state=SystemState.UNKNOWN,
            priority=SystemLEDPriority.NORMAL,
            battery_level=1.0,
            battery_voltage=25.0,
            is_charging=False,
            error_codes=[],
            temperature=30.0,
            timestamp=time.time()
        )
        
        # 历史数据和分析
        self.state_history = deque(maxlen=history_size)
        self.battery_history = deque(maxlen=history_size)
        self.error_history = deque(maxlen=history_size)
        
        # 回调和事件
        self.state_change_callbacks = []
        self.critical_event_callbacks = []
        
        # 监控控制
        self.monitoring_active = False
        self.monitor_thread = None
        self.monitor_lock = threading.Lock()
        
        # 性能统计
        self.update_count = 0
        self.error_count = 0
        self.last_update_time = 0
        self.avg_update_interval = 0.1
        
        # 状态分析配置
        self.low_battery_threshold = 0.15      # 15%低电量阈值
        self.critical_battery_threshold = 0.05  # 5%关键电量阈值
        self.high_temperature_threshold = 70.0  # 70°C高温阈值
        self.network_timeout = 5.0              # 5秒网络超时
        
    def initialize(self) -> bool:
        """初始化监控器"""
        try:
            self.logger.info("🔍 初始化系统状态监控器...")
            
            # 初始化ROS2
            if ROS2_AVAILABLE:
                success = self._initialize_ros2()
                if not success:
                    self.logger.warning("ROS2初始化失败，启用模拟模式")
                    return self._initialize_simulation_mode()
            else:
                self.logger.warning("ROS2不可用，启用模拟模式")
                return self._initialize_simulation_mode()
            
            self.logger.info("✅ 系统状态监控器初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ 系统状态监控器初始化失败: {e}")
            return False
    
    def _initialize_ros2(self) -> bool:
        """初始化ROS2组件"""
        import os
        import contextlib

        try:
            # 抑制ROS2错误输出（用户不应看到底层RMW错误）
            with contextlib.redirect_stderr(open(os.devnull, 'w')):
                if not rclpy.ok():
                    # 设置ROS2日志级别为FATAL（抑制ERROR级别）
                    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = ''
                    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'
                    rclpy.init()

                # 创建节点（可能失败，但错误已被抑制）
                self.node = SystemMonitorNode(
                    node_name=self.node_name,
                    state_callback=self._on_state_update,
                    error_callback=self._on_system_error
                )

            # 创建执行器
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.node)

            # 启动执行器线程
            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self.executor_thread.start()

            self.is_ros_initialized = True
            self.logger.info("✅ ROS2系统状态监控器初始化成功")
            return True

        except Exception as e:
            # ROS2初始化失败时不记录详细错误（会自动fallback到模拟模式）
            return False
    
    def _initialize_simulation_mode(self) -> bool:
        """初始化模拟模式"""
        try:
            self.is_ros_initialized = False
            self.logger.info("🔄 启用系统状态模拟模式")
            
            # 启动模拟监控线程
            self.monitor_thread = threading.Thread(
                target=self._simulation_monitor_worker,
                daemon=True
            )
            self.monitor_thread.start()
            
            return True
            
        except Exception as e:
            self.logger.error(f"❌ 模拟模式初始化失败: {e}")
            return False
    
    def start_monitoring(self) -> bool:
        """开始监控"""
        try:
            with self.monitor_lock:
                if self.monitoring_active:
                    self.logger.warning("系统状态监控已经在运行")
                    return True
                
                self.monitoring_active = True
                
                if not self.is_ros_initialized:
                    # 模拟模式已在initialize中启动
                    pass
                
                self.logger.info("🔍 系统状态监控已启动")
                return True
                
        except Exception as e:
            self.logger.error(f"❌ 启动系统状态监控失败: {e}")
            return False
    
    def stop_monitoring(self) -> None:
        """停止监控"""
        try:
            with self.monitor_lock:
                self.monitoring_active = False
                
                if self.executor:
                    self.executor.shutdown()
                    
                if self.executor_thread and self.executor_thread.is_alive():
                    self.executor_thread.join(timeout=2.0)
                
                self.logger.info("🔍 系统状态监控已停止")
                
        except Exception as e:
            self.logger.error(f"❌ 停止系统状态监控失败: {e}")
    
    def _simulation_monitor_worker(self) -> None:
        """模拟监控工作线程"""
        simulation_cycle = 0
        
        while self.monitoring_active:
            try:
                # 模拟系统状态变化
                simulation_cycle += 1
                current_time = time.time()
                
                # 基础状态模拟
                if simulation_cycle % 100 == 0:  # 每10秒模拟一次低电量
                    battery_level = 0.12  # 模拟低电量
                    state = SystemState.LOW_BATTERY
                    priority = SystemLEDPriority.SAFETY_WARNING
                elif simulation_cycle % 200 == 0:  # 每20秒模拟一次校准
                    battery_level = 0.85
                    state = SystemState.CALIBRATING  
                    priority = SystemLEDPriority.SYSTEM_NOTIFICATION
                else:
                    battery_level = 0.85  # 正常电量
                    state = SystemState.NORMAL
                    priority = SystemLEDPriority.NORMAL
                
                # 创建模拟状态信息
                sim_state_info = SystemStateInfo(
                    state=state,
                    priority=priority,
                    battery_level=battery_level,
                    battery_voltage=24.0 + battery_level * 4.0,
                    is_charging=False,
                    error_codes=[],
                    temperature=35.0,
                    timestamp=current_time,
                    is_standing=True,
                    current_gait="standing",
                    network_status="connected",
                    confidence=0.9,
                    source="simulation"
                )
                
                # 更新状态
                self._on_state_update(sim_state_info)
                
                time.sleep(1.0 / self.update_rate)
                
            except Exception as e:
                self.logger.error(f"模拟监控循环错误: {e}")
                time.sleep(1.0)
    
    def _on_state_update(self, state_info: SystemStateInfo) -> None:
        """处理状态更新"""
        try:
            with self.monitor_lock:
                previous_state = self.current_state_info.state
                previous_priority = self.current_state_info.priority
                
                # 更新当前状态
                self.current_state_info = state_info
                
                # 更新历史数据
                self.state_history.append(state_info)
                self.battery_history.append(state_info.battery_level)
                if state_info.error_codes:
                    self.error_history.extend(state_info.error_codes)
                
                # 更新性能统计
                self.update_count += 1
                current_time = time.time()
                if self.last_update_time > 0:
                    interval = current_time - self.last_update_time
                    self.avg_update_interval = (self.avg_update_interval * 0.9 + interval * 0.1)
                self.last_update_time = current_time
                
                # 检查状态变化
                if (previous_state != state_info.state or 
                    previous_priority != state_info.priority):
                    self._notify_state_change(previous_state, state_info)
                
                # 检查关键事件
                self._check_critical_events(state_info)
                
        except Exception as e:
            self.logger.error(f"状态更新处理失败: {e}")
            self.error_count += 1
    
    def _on_system_error(self, error_code: int, error_message: str) -> None:
        """处理系统错误"""
        try:
            self.logger.warning(f"🚨 系统错误: {error_code} - {error_message}")
            
            # 更新错误历史
            self.error_history.append(error_code)
            
            # 创建错误状态信息
            error_state_info = SystemStateInfo(
                state=SystemState.ERROR,
                priority=SystemLEDPriority.CRITICAL_ERROR,
                battery_level=self.current_state_info.battery_level,
                battery_voltage=self.current_state_info.battery_voltage,
                is_charging=self.current_state_info.is_charging,
                error_codes=[error_code],
                temperature=self.current_state_info.temperature,
                timestamp=time.time(),
                source="error_handler"
            )
            
            # 触发状态更新
            self._on_state_update(error_state_info)
            
        except Exception as e:
            self.logger.error(f"系统错误处理失败: {e}")
    
    def _notify_state_change(self, previous_state: SystemState, new_state_info: SystemStateInfo) -> None:
        """通知状态变化"""
        try:
            self.logger.info(f"🔄 系统状态变化: {previous_state.name} -> {new_state_info.state.name}")
            
            # 调用所有状态变化回调
            for callback in self.state_change_callbacks:
                try:
                    callback(previous_state, new_state_info)
                except Exception as e:
                    self.logger.error(f"状态变化回调失败: {e}")
                    
        except Exception as e:
            self.logger.error(f"状态变化通知失败: {e}")
    
    def _check_critical_events(self, state_info: SystemStateInfo) -> None:
        """检查关键事件"""
        try:
            critical_events = []
            
            # 检查低电量
            if state_info.battery_level <= self.critical_battery_threshold:
                critical_events.append(("critical_battery", f"电池电量极低: {state_info.battery_level*100:.1f}%"))
            elif state_info.battery_level <= self.low_battery_threshold:
                critical_events.append(("low_battery", f"电池电量低: {state_info.battery_level*100:.1f}%"))
            
            # 检查高温
            if state_info.temperature >= self.high_temperature_threshold:
                critical_events.append(("high_temperature", f"系统温度过高: {state_info.temperature:.1f}°C"))
            
            # 检查错误代码
            if state_info.error_codes:
                critical_events.append(("system_errors", f"系统错误: {state_info.error_codes}"))
            
            # 触发关键事件回调
            for event_type, event_message in critical_events:
                self.logger.warning(f"🚨 关键事件: {event_message}")
                for callback in self.critical_event_callbacks:
                    try:
                        callback(event_type, event_message, state_info)
                    except Exception as e:
                        self.logger.error(f"关键事件回调失败: {e}")
                        
        except Exception as e:
            self.logger.error(f"关键事件检查失败: {e}")
    
    def get_current_state(self) -> SystemStateInfo:
        """获取当前系统状态"""
        return self.current_state_info
    
    def get_led_control_decision(self, requested_priority: int = 5) -> LEDControlDecision:
        """
        获取LED控制决策
        
        Args:
            requested_priority: 请求的优先级
            
        Returns:
            LEDControlDecision: LED控制决策
        """
        try:
            current_state = self.current_state_info
            system_priority = current_state.priority.value
            
            # 基本规则：请求优先级必须高于或等于系统状态优先级
            allow_control = requested_priority >= system_priority
            
            # 特殊状态处理
            if current_state.state in [SystemState.EMERGENCY, SystemState.ERROR]:
                # 紧急和错误状态需要更高优先级
                required_priority = SystemLEDPriority.CRITICAL_ERROR
                allow_control = requested_priority >= SystemLEDPriority.CRITICAL_ERROR.value
                system_override = True
                reason = f"系统处于{current_state.state.name}状态，需要高优先级控制权"
                
            elif current_state.state == SystemState.LOW_BATTERY:
                required_priority = SystemLEDPriority.SAFETY_WARNING
                allow_control = requested_priority >= SystemLEDPriority.SAFETY_WARNING.value
                system_override = True
                reason = f"电池电量低({current_state.battery_level*100:.1f}%)，优先显示安全警告"
                
            elif current_state.state == SystemState.CALIBRATING:
                required_priority = SystemLEDPriority.SYSTEM_NOTIFICATION
                allow_control = requested_priority >= SystemLEDPriority.SYSTEM_NOTIFICATION.value
                system_override = False
                reason = "系统校准中，建议避免LED控制冲突"
                
            else:
                # 正常状态
                required_priority = SystemLEDPriority.NORMAL
                system_override = False
                reason = "系统状态正常，允许自定义LED控制"
            
            # 推荐操作
            if allow_control:
                recommended_action = "proceed"
            elif requested_priority < required_priority.value:
                recommended_action = "increase_priority"
            else:
                recommended_action = "wait_for_system_state_change"
            
            return LEDControlDecision(
                allow_custom_control=allow_control,
                required_priority=required_priority,
                system_override_active=system_override,
                recommended_action=recommended_action,
                reason=reason
            )
            
        except Exception as e:
            self.logger.error(f"LED控制决策生成失败: {e}")
            # 安全回退：拒绝控制
            return LEDControlDecision(
                allow_custom_control=False,
                required_priority=SystemLEDPriority.CRITICAL_ERROR,
                system_override_active=True,
                recommended_action="wait",
                reason=f"决策生成失败: {e}"
            )
    
    def register_state_change_callback(self, callback: Callable[[SystemState, SystemStateInfo], None]) -> None:
        """注册状态变化回调"""
        self.state_change_callbacks.append(callback)
    
    def register_critical_event_callback(self, callback: Callable[[str, str, SystemStateInfo], None]) -> None:
        """注册关键事件回调"""
        self.critical_event_callbacks.append(callback)
    
    def get_system_statistics(self) -> Dict[str, Any]:
        """获取系统统计信息"""
        try:
            current_time = time.time()
            uptime = current_time - (self.last_update_time - self.avg_update_interval * self.update_count) if self.update_count > 0 else 0
            
            # 电池统计
            battery_stats = {}
            if self.battery_history:
                battery_stats = {
                    "current": self.current_state_info.battery_level,
                    "average": statistics.mean(self.battery_history),
                    "min": min(self.battery_history),
                    "max": max(self.battery_history),
                    "trend": "stable"  # 简化版，实际可计算趋势
                }
            
            # 状态分布统计
            state_distribution = {}
            if self.state_history:
                for state_info in self.state_history:
                    state_name = state_info.state.name
                    state_distribution[state_name] = state_distribution.get(state_name, 0) + 1
            
            return {
                "uptime_seconds": uptime,
                "update_count": self.update_count,
                "error_count": self.error_count,
                "avg_update_interval": self.avg_update_interval,
                "current_state": self.current_state_info.state.name,
                "current_priority": self.current_state_info.priority.name,
                "battery_stats": battery_stats,
                "state_distribution": state_distribution,
                "error_history_count": len(self.error_history),
                "is_ros_initialized": self.is_ros_initialized,
                "monitoring_active": self.monitoring_active
            }
            
        except Exception as e:
            self.logger.error(f"获取系统统计失败: {e}")
            return {"error": str(e)}
    
    def cleanup(self) -> None:
        """清理资源"""
        try:
            self.stop_monitoring()
            
            if self.node:
                self.node.destroy_node()
            
            if rclpy.ok():
                rclpy.shutdown()
                
            self.logger.info("🔍 系统状态监控器清理完成")
            
        except Exception as e:
            self.logger.error(f"系统状态监控器清理失败: {e}")

class SystemMonitorNode(Node):
    """ROS2系统监控节点"""

    def __init__(self,
                 node_name: str,
                 state_callback: Callable[[SystemStateInfo], None],
                 error_callback: Callable[[int, str], None]):
        """
        初始化ROS2监控节点
        
        Args:
            node_name: 节点名称
            state_callback: 状态更新回调
            error_callback: 错误回调
        """
        super().__init__(node_name)
        self.state_callback = state_callback
        self.error_callback = error_callback
        
        # 创建订阅者
        self._create_subscribers()
        
        # 状态数据
        self.last_low_state = None
        self.last_sport_state = None
        
        self.get_logger().info(f"系统监控节点 {node_name} 已启动")
    
    def _create_subscribers(self) -> None:
        """创建ROS2订阅者"""
        try:
            if UNITREE_MSGS_AVAILABLE:
                # 底层状态订阅
                self.low_state_sub = self.create_subscription(
                    LowState,
                    '/low_state',  # 根据实际话题名调整
                    self._low_state_callback,
                    10
                )
                
                # 运动模式状态订阅  
                self.sport_state_sub = self.create_subscription(
                    SportModeState,
                    '/sportmodestate',  # 根据实际话题名调整
                    self._sport_state_callback,
                    10
                )
                
                # 电池状态订阅
                self.bms_state_sub = self.create_subscription(
                    BmsState,
                    '/bms_state',  # 根据实际话题名调整
                    self._bms_state_callback,
                    10
                )
            else:
                # 模拟订阅 - 使用标准消息类型
                self.sim_state_sub = self.create_subscription(
                    String,
                    '/simulation/system_state',
                    self._simulation_callback,
                    10
                )
                
            self.get_logger().info("✅ ROS2订阅者创建完成")
            
        except Exception as e:
            self.get_logger().error(f"❌ 创建ROS2订阅者失败: {e}")
    
    def _low_state_callback(self, msg) -> None:
        """底层状态消息回调"""
        try:
            self.last_low_state = msg
            
            # 解析系统状态（根据实际消息结构调整）
            battery_level = getattr(msg, 'battery_percentage', 0.85)
            battery_voltage = getattr(msg, 'battery_voltage', 24.0)
            temperature = getattr(msg, 'temperature', 30.0)
            error_codes = getattr(msg, 'error_codes', [])
            
            # 确定系统状态
            if error_codes:
                state = SystemState.ERROR
                priority = SystemLEDPriority.CRITICAL_ERROR
            elif battery_level <= 0.05:
                state = SystemState.EMERGENCY  
                priority = SystemLEDPriority.CRITICAL_ERROR
            elif battery_level <= 0.15:
                state = SystemState.LOW_BATTERY
                priority = SystemLEDPriority.SAFETY_WARNING
            else:
                state = SystemState.NORMAL
                priority = SystemLEDPriority.NORMAL
            
            # 创建状态信息
            state_info = SystemStateInfo(
                state=state,
                priority=priority,
                battery_level=battery_level,
                battery_voltage=battery_voltage,
                is_charging=getattr(msg, 'is_charging', False),
                error_codes=error_codes,
                temperature=temperature,
                timestamp=time.time(),
                source="low_state"
            )
            
            # 调用回调
            self.state_callback(state_info)
            
        except Exception as e:
            self.get_logger().error(f"底层状态回调失败: {e}")
    
    def _sport_state_callback(self, msg) -> None:
        """运动状态消息回调"""
        try:
            self.last_sport_state = msg
            
            # 解析运动状态（根据实际消息结构调整）
            gait_type = getattr(msg, 'gait_type', 0)
            is_standing = getattr(msg, 'mode', 0) == 1  # 假设1为站立模式
            
            # 如果已有状态信息，更新运动相关字段
            if self.last_low_state:
                # 重用最新的底层状态信息并更新运动状态
                pass  # 可根据需要扩展
                
        except Exception as e:
            self.get_logger().error(f"运动状态回调失败: {e}")
    
    def _bms_state_callback(self, msg) -> None:
        """电池管理系统状态回调"""
        try:
            # 处理详细的电池信息
            battery_level = getattr(msg, 'soc', 0.85)  # State of Charge
            is_charging = getattr(msg, 'status', 0) == 2  # 假设2为充电状态
            
            # 可根据需要处理更详细的电池信息
            
        except Exception as e:
            self.get_logger().error(f"电池状态回调失败: {e}")
    
    def _simulation_callback(self, msg: String) -> None:
        """模拟状态回调"""
        try:
            # 解析模拟数据
            data = json.loads(msg.data)
            
            state_info = SystemStateInfo(
                state=SystemState[data.get('state', 'NORMAL')],
                priority=SystemLEDPriority[data.get('priority', 'NORMAL')],
                battery_level=data.get('battery_level', 0.85),
                battery_voltage=data.get('battery_voltage', 24.0),
                is_charging=data.get('is_charging', False),
                error_codes=data.get('error_codes', []),
                temperature=data.get('temperature', 30.0),
                timestamp=time.time(),
                source="simulation"
            )
            
            self.state_callback(state_info)
            
        except Exception as e:
            self.get_logger().error(f"模拟回调失败: {e}")

# 工厂函数
def create_system_state_monitor(
    node_name: str = "claudia_system_monitor",
    history_size: int = 50,
    update_rate: float = 10.0) -> SystemStateMonitor:
    """
    创建系统状态监控器
    
    Args:
        node_name: ROS2节点名称
        history_size: 历史数据缓存大小  
        update_rate: 更新频率 (Hz)
        
    Returns:
        SystemStateMonitor: 系统状态监控器实例
    """
    return SystemStateMonitor(
        node_name=node_name,
        history_size=history_size,
        update_rate=update_rate
    )

# 测试和调试功能
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    print("🔍 系统状态监控器测试")
    
    monitor = create_system_state_monitor()
    
    def on_state_change(prev_state, new_state_info):
        print(f"状态变化: {prev_state.name} -> {new_state_info.state.name}")
    
    def on_critical_event(event_type, message, state_info):
        print(f"关键事件: {event_type} - {message}")
    
    monitor.register_state_change_callback(on_state_change)
    monitor.register_critical_event_callback(on_critical_event)
    
    if monitor.initialize():
        monitor.start_monitoring()
        
        try:
            # 运行5分钟测试
            for i in range(30):
                time.sleep(10)
                decision = monitor.get_led_control_decision(5)
                print(f"LED控制决策: {decision.allow_custom_control} - {decision.reason}")
                
                if i % 3 == 0:
                    stats = monitor.get_system_statistics()
                    print(f"统计信息: {stats}")
                    
        except KeyboardInterrupt:
            print("测试中断")
        finally:
            monitor.cleanup()
    else:
        print("❌ 监控器初始化失败") 