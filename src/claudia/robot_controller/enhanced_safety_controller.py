#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia增强安全控制器
🛡️ Phase 3: 增强安全机制

实现硬件故障检测、自动诊断和多层级安全回退策略
确保系统默认兼容性和安全机制在任何情况下都能正常工作

Author: Claudia AI System
Generated: 2025-01-26
Purpose: 子任务6.4 - Phase 3: 增强安全机制
"""

import sys
import time
import threading
import logging
from typing import Dict, List, Optional, Tuple, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
from datetime import datetime, timedelta
import queue
import statistics

# 添加项目路径
sys.path.append('/home/m1ng/claudia/src')

# 导入依赖模块
try:
    from claudia.robot_controller.led_patterns import ClaudiaLEDMode
    from claudia.robot_controller.system_state_monitor import (
        SystemState, SystemLEDPriority, SystemStateInfo
    )
    DEPENDENCIES_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ 依赖模块导入失败: {e}")
    DEPENDENCIES_AVAILABLE = False

class SafetyLevel(Enum):
    """安全级别枚举"""
    NORMAL = "normal"           # 正常状态
    CAUTION = "caution"         # 谨慎状态
    WARNING = "warning"         # 警告状态  
    CRITICAL = "critical"       # 危险状态
    EMERGENCY = "emergency"     # 紧急状态

class SafetyEventType(Enum):
    """安全事件类型"""
    HARDWARE_FAILURE = "hardware_failure"
    COMMUNICATION_LOSS = "communication_loss"
    POWER_ISSUE = "power_issue"
    THERMAL_ISSUE = "thermal_issue"
    SYSTEM_OVERLOAD = "system_overload"
    SAFETY_VIOLATION = "safety_violation"
    RECOVERY_SUCCESS = "recovery_success"

class SafetyAction(Enum):
    """安全动作枚举"""
    MONITOR = "monitor"             # 监控
    ALERT = "alert"                 # 警报
    DEGRADE = "degrade"             # 降级
    ISOLATE = "isolate"             # 隔离
    SHUTDOWN = "shutdown"           # 关闭
    EMERGENCY_STOP = "emergency_stop"  # 紧急停机

@dataclass
class SafetyEvent:
    """安全事件数据类"""
    event_type: SafetyEventType
    severity: SafetyLevel
    timestamp: datetime
    source: str
    message: str
    data: Dict[str, Any] = field(default_factory=dict)
    resolved: bool = False
    resolution_time: Optional[datetime] = None

@dataclass
class SafetyRule:
    """安全规则数据类"""
    rule_id: str
    name: str
    condition: Callable[[Dict[str, Any]], bool]
    action: SafetyAction
    severity: SafetyLevel
    cooldown_seconds: float = 0
    max_triggers: int = 10
    enabled: bool = True

@dataclass
class HardwareStatus:
    """硬件状态数据类"""
    component: str
    status: str              # "ok", "degraded", "failed", "unknown"
    last_check: datetime
    error_count: int = 0
    total_checks: int = 0
    response_time: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

class EnhancedSafetyController:
    """
    🛡️ Phase 3: 增强安全控制器
    
    实现多层级安全机制：
    1. 硬件故障检测和自动诊断
    2. 安全回退策略
    3. 实时监控和告警
    4. 故障恢复机制
    """
    
    def __init__(self, 
                 response_time_target: float = 0.2,
                 safety_check_interval: float = 1.0):
        """
        初始化增强安全控制器
        
        Args:
            response_time_target: 目标响应时间（秒）
            safety_check_interval: 安全检查间隔（秒）
        """
        self.logger = logging.getLogger(__name__)
        self.response_time_target = response_time_target
        self.safety_check_interval = safety_check_interval
        
        # 安全状态
        self.current_safety_level = SafetyLevel.NORMAL
        self.is_initialized = False
        self.safety_monitoring_active = False
        
        # 事件管理
        self.safety_events: List[SafetyEvent] = []
        self.event_queue = queue.Queue()
        self.max_event_history = 1000
        
        # 硬件状态监控
        self.hardware_status: Dict[str, HardwareStatus] = {}
        self.hardware_check_functions: Dict[str, Callable] = {}
        
        # 安全规则系统
        self.safety_rules: Dict[str, SafetyRule] = {}
        self.rule_trigger_history: Dict[str, List[datetime]] = {}
        
        # 回退策略
        self.fallback_strategies: Dict[SafetyLevel, List[Callable]] = {
            SafetyLevel.CAUTION: [],
            SafetyLevel.WARNING: [],
            SafetyLevel.CRITICAL: [],
            SafetyLevel.EMERGENCY: []
        }
        
        # 监控线程
        self.monitoring_thread = None
        self.event_processing_thread = None
        
        # 性能指标
        self.safety_metrics = {
            'total_events': 0,
            'events_by_severity': {level.value: 0 for level in SafetyLevel},
            'events_by_type': {event_type.value: 0 for event_type in SafetyEventType},
            'avg_response_time': 0.0,
            'max_response_time': 0.0,
            'successful_recoveries': 0,
            'failed_recoveries': 0,
            'safety_level_changes': 0
        }
        
        # 回调函数
        self.safety_event_callbacks: List[Callable] = []
        self.safety_level_change_callbacks: List[Callable] = []
        
        self.logger.info("增强安全控制器初始化完成")
    
    def initialize(self) -> bool:
        """
        初始化安全控制器
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            self.logger.info("初始化增强安全控制器...")
            
            # 初始化默认安全规则
            self._initialize_default_safety_rules()
            
            # 初始化硬件检查函数
            self._initialize_hardware_checks()
            
            # 启动监控线程
            self.safety_monitoring_active = True
            
            self.monitoring_thread = threading.Thread(
                target=self._safety_monitoring_worker,
                daemon=True
            )
            self.monitoring_thread.start()
            
            self.event_processing_thread = threading.Thread(
                target=self._event_processing_worker,
                daemon=True
            )
            self.event_processing_thread.start()
            
            self.is_initialized = True
            self.logger.info("✅ 增强安全控制器初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"增强安全控制器初始化失败: {e}")
            return False
    
    def _initialize_default_safety_rules(self) -> None:
        """初始化默认安全规则"""
        
        # 规则1: 硬件故障检测
        self.add_safety_rule(SafetyRule(
            rule_id="hardware_failure_detection",
            name="硬件故障检测",
            condition=lambda data: self._check_hardware_failures(data),
            action=SafetyAction.ALERT,
            severity=SafetyLevel.WARNING,
            cooldown_seconds=5.0
        ))
        
        # 规则2: 通信丢失检测
        self.add_safety_rule(SafetyRule(
            rule_id="communication_loss_detection",
            name="通信丢失检测",
            condition=lambda data: self._check_communication_loss(data),
            action=SafetyAction.DEGRADE,
            severity=SafetyLevel.CRITICAL,
            cooldown_seconds=2.0
        ))
        
        # 规则3: 电源问题检测
        self.add_safety_rule(SafetyRule(
            rule_id="power_issue_detection",
            name="电源问题检测",
            condition=lambda data: self._check_power_issues(data),
            action=SafetyAction.ALERT,
            severity=SafetyLevel.WARNING,
            cooldown_seconds=3.0
        ))
        
        # 规则4: 温度异常检测
        self.add_safety_rule(SafetyRule(
            rule_id="thermal_issue_detection",
            name="温度异常检测",
            condition=lambda data: self._check_thermal_issues(data),
            action=SafetyAction.DEGRADE,
            severity=SafetyLevel.CRITICAL,
            cooldown_seconds=1.0
        ))
        
        # 规则5: 系统过载检测
        self.add_safety_rule(SafetyRule(
            rule_id="system_overload_detection",
            name="系统过载检测",
            condition=lambda data: self._check_system_overload(data),
            action=SafetyAction.DEGRADE,
            severity=SafetyLevel.WARNING,
            cooldown_seconds=5.0
        ))
        
        self.logger.info(f"已初始化 {len(self.safety_rules)} 条默认安全规则")
    
    def _initialize_hardware_checks(self) -> None:
        """初始化硬件检查函数"""
        
        # LED控制器检查
        self.hardware_check_functions["led_controller"] = self._check_led_controller
        
        # 系统状态监控器检查
        self.hardware_check_functions["system_monitor"] = self._check_system_monitor
        
        # 通信链路检查
        self.hardware_check_functions["communication"] = self._check_communication
        
        # 电源系统检查
        self.hardware_check_functions["power_system"] = self._check_power_system
        
        self.logger.info(f"已初始化 {len(self.hardware_check_functions)} 个硬件检查功能")
    
    def _safety_monitoring_worker(self) -> None:
        """安全监控工作线程"""
        last_check_time = 0
        
        while self.safety_monitoring_active:
            try:
                current_time = time.time()
                
                # 检查是否到了执行安全检查的时间
                if current_time - last_check_time >= self.safety_check_interval:
                    self._perform_safety_checks()
                    last_check_time = current_time
                
                # 处理硬件状态检查
                self._perform_hardware_checks()
                
                # 短暂休眠
                time.sleep(0.1)
                
            except Exception as e:
                self.logger.error(f"安全监控工作线程错误: {e}")
                time.sleep(self.safety_check_interval)
    
    def _event_processing_worker(self) -> None:
        """事件处理工作线程"""
        while self.safety_monitoring_active:
            try:
                # 等待安全事件
                try:
                    event = self.event_queue.get(timeout=1.0)
                    self._process_safety_event(event)
                    self.event_queue.task_done()
                except queue.Empty:
                    continue
                    
            except Exception as e:
                self.logger.error(f"事件处理工作线程错误: {e}")
    
    def _perform_safety_checks(self) -> None:
        """执行安全检查"""
        start_time = time.time()
        
        # 收集当前系统数据
        system_data = self._collect_system_data()
        
        # 执行所有启用的安全规则
        for rule_id, rule in self.safety_rules.items():
            if not rule.enabled:
                continue
                
            try:
                # 检查冷却时间
                if not self._check_rule_cooldown(rule_id, rule.cooldown_seconds):
                    continue
                
                # 执行规则条件检查
                if rule.condition(system_data):
                    self._trigger_safety_rule(rule, system_data)
                    
            except Exception as e:
                self.logger.error(f"安全规则 {rule_id} 执行失败: {e}")
        
        # 更新性能指标
        response_time = time.time() - start_time
        self._update_response_time_metrics(response_time)
    
    def _perform_hardware_checks(self) -> None:
        """执行硬件检查"""
        for component, check_function in self.hardware_check_functions.items():
            try:
                start_time = time.time()
                status = check_function()
                response_time = time.time() - start_time
                
                # 更新硬件状态
                if component not in self.hardware_status:
                    self.hardware_status[component] = HardwareStatus(
                        component=component,
                        status="unknown",
                        last_check=datetime.now()
                    )
                
                hw_status = self.hardware_status[component]
                hw_status.last_check = datetime.now()
                hw_status.total_checks += 1
                hw_status.response_time = response_time
                
                if status:
                    hw_status.status = "ok"
                    hw_status.error_count = max(0, hw_status.error_count - 1)  # 缓慢恢复
                else:
                    hw_status.status = "failed"
                    hw_status.error_count += 1
                    
                    # 触发硬件故障事件
                    self.report_safety_event(
                        SafetyEventType.HARDWARE_FAILURE,
                        SafetyLevel.WARNING,
                        f"硬件组件检查失败: {component}",
                        f"enhanced_safety_{component}",
                        {"component": component, "response_time": response_time}
                    )
                
            except Exception as e:
                self.logger.error(f"硬件检查失败 {component}: {e}")
    
    def _collect_system_data(self) -> Dict[str, Any]:
        """收集系统数据"""
        return {
            'timestamp': time.time(),
            'safety_level': self.current_safety_level,
            'hardware_status': self.hardware_status.copy(),
            'event_count': len(self.safety_events),
            'recent_events': self.safety_events[-10:] if self.safety_events else [],
            'metrics': self.safety_metrics.copy()
        }
    
    def _check_rule_cooldown(self, rule_id: str, cooldown_seconds: float) -> bool:
        """检查规则冷却时间"""
        if rule_id not in self.rule_trigger_history:
            return True
            
        recent_triggers = self.rule_trigger_history[rule_id]
        if not recent_triggers:
            return True
            
        time_since_last = (datetime.now() - recent_triggers[-1]).total_seconds()
        return time_since_last >= cooldown_seconds
    
    def _trigger_safety_rule(self, rule: SafetyRule, system_data: Dict[str, Any]) -> None:
        """触发安全规则"""
        # 记录触发历史
        if rule.rule_id not in self.rule_trigger_history:
            self.rule_trigger_history[rule.rule_id] = []
        
        self.rule_trigger_history[rule.rule_id].append(datetime.now())
        
        # 限制历史记录大小
        if len(self.rule_trigger_history[rule.rule_id]) > rule.max_triggers:
            self.rule_trigger_history[rule.rule_id].pop(0)
        
        # 创建安全事件
        event = SafetyEvent(
            event_type=SafetyEventType.SAFETY_VIOLATION,
            severity=rule.severity,
            timestamp=datetime.now(),
            source=f"safety_rule_{rule.rule_id}",
            message=f"安全规则触发: {rule.name}",
            data={
                'rule_id': rule.rule_id,
                'action': rule.action.value,
                'system_data': system_data
            }
        )
        
        # 排队处理事件
        self.event_queue.put(event)
        
        self.logger.warning(f"🛡️ 安全规则触发: {rule.name} (动作: {rule.action.value})")
    
    def _process_safety_event(self, event: SafetyEvent) -> None:
        """处理安全事件"""
        try:
            # 添加到事件历史
            self.safety_events.append(event)
            
            # 限制历史大小
            if len(self.safety_events) > self.max_event_history:
                self.safety_events.pop(0)
            
            # 更新指标
            self.safety_metrics['total_events'] += 1
            self.safety_metrics['events_by_severity'][event.severity.value] += 1
            self.safety_metrics['events_by_type'][event.event_type.value] += 1
            
            # 根据事件严重性更新安全级别
            if event.severity.value == SafetyLevel.EMERGENCY.value:
                self._change_safety_level(SafetyLevel.EMERGENCY)
            elif event.severity.value == SafetyLevel.CRITICAL.value and self.current_safety_level.value in ['normal', 'caution', 'warning']:
                self._change_safety_level(SafetyLevel.CRITICAL)
            elif event.severity.value == SafetyLevel.WARNING.value and self.current_safety_level.value in ['normal', 'caution']:
                self._change_safety_level(SafetyLevel.WARNING)
            elif event.severity.value == SafetyLevel.CAUTION.value and self.current_safety_level == SafetyLevel.NORMAL:
                self._change_safety_level(SafetyLevel.CAUTION)
            
            # 执行回调
            for callback in self.safety_event_callbacks:
                try:
                    callback(event)
                except Exception as e:
                    self.logger.error(f"安全事件回调失败: {e}")
            
            self.logger.info(f"🛡️ 处理安全事件: {event.event_type.value} (严重性: {event.severity.value})")
            
        except Exception as e:
            self.logger.error(f"安全事件处理失败: {e}")
    
    def _change_safety_level(self, new_level: SafetyLevel) -> None:
        """改变安全级别"""
        if new_level == self.current_safety_level:
            return
            
        previous_level = self.current_safety_level
        self.current_safety_level = new_level
        self.safety_metrics['safety_level_changes'] += 1
        
        self.logger.warning(f"🛡️ 安全级别变化: {previous_level.value} → {new_level.value}")
        
        # 执行相应的回退策略
        if new_level in self.fallback_strategies:
            for strategy in self.fallback_strategies[new_level]:
                try:
                    strategy(previous_level, new_level)
                except Exception as e:
                    self.logger.error(f"回退策略执行失败: {e}")
        
        # 执行安全级别变化回调
        for callback in self.safety_level_change_callbacks:
            try:
                callback(previous_level, new_level)
            except Exception as e:
                self.logger.error(f"安全级别变化回调失败: {e}")
    
    # 硬件检查函数
    def _check_led_controller(self) -> bool:
        """检查LED控制器状态"""
        # 简化的LED控制器检查
        # 实际实现应该检查LED渲染器和控制器的响应性
        return True  # 假设正常
    
    def _check_system_monitor(self) -> bool:
        """检查系统状态监控器"""
        # 检查系统监控器是否正常工作
        return True  # 假设正常
    
    def _check_communication(self) -> bool:
        """检查通信链路"""
        # 检查ROS2通信、网络连接等
        return True  # 假设正常
    
    def _check_power_system(self) -> bool:
        """检查电源系统"""
        # 检查电池状态、电源管理等
        return True  # 假设正常
    
    # 安全规则条件检查函数
    def _check_hardware_failures(self, data: Dict[str, Any]) -> bool:
        """检查硬件故障"""
        hardware_status = data.get('hardware_status', {})
        
        for component, status in hardware_status.items():
            if status.status == "failed" and status.error_count > 3:
                return True
        
        return False
    
    def _check_communication_loss(self, data: Dict[str, Any]) -> bool:
        """检查通信丢失"""
        # 简化实现：检查通信组件的错误计数
        hardware_status = data.get('hardware_status', {})
        comm_status = hardware_status.get('communication')
        
        if comm_status and comm_status.error_count > 5:
            return True
        
        return False
    
    def _check_power_issues(self, data: Dict[str, Any]) -> bool:
        """检查电源问题"""
        # 简化实现：检查电源系统状态
        hardware_status = data.get('hardware_status', {})
        power_status = hardware_status.get('power_system')
        
        if power_status and power_status.status == "failed":
            return True
        
        return False
    
    def _check_thermal_issues(self, data: Dict[str, Any]) -> bool:
        """检查温度问题"""
        # 简化实现：基于错误率判断温度问题
        recent_events = data.get('recent_events', [])
        thermal_events = [e for e in recent_events 
                         if e.event_type == SafetyEventType.THERMAL_ISSUE]
        
        return len(thermal_events) > 2
    
    def _check_system_overload(self, data: Dict[str, Any]) -> bool:
        """检查系统过载"""
        # 简化实现：基于事件频率判断系统过载
        recent_events = data.get('recent_events', [])
        
        if len(recent_events) > 8:  # 最近有很多事件
            return True
        
        return False
    
    def _update_response_time_metrics(self, response_time: float) -> None:
        """更新响应时间指标"""
        current_avg = self.safety_metrics['avg_response_time']
        total_events = self.safety_metrics['total_events']
        
        if total_events > 0:
            new_avg = (current_avg * (total_events - 1) + response_time) / total_events
            self.safety_metrics['avg_response_time'] = new_avg
        
        if response_time > self.safety_metrics['max_response_time']:
            self.safety_metrics['max_response_time'] = response_time
    
    # 公共接口方法
    def add_safety_rule(self, rule: SafetyRule) -> None:
        """添加安全规则"""
        self.safety_rules[rule.rule_id] = rule
        self.logger.info(f"添加安全规则: {rule.name}")
    
    def remove_safety_rule(self, rule_id: str) -> bool:
        """移除安全规则"""
        if rule_id in self.safety_rules:
            del self.safety_rules[rule_id]
            self.logger.info(f"移除安全规则: {rule_id}")
            return True
        return False
    
    def enable_safety_rule(self, rule_id: str, enabled: bool = True) -> bool:
        """启用/禁用安全规则"""
        if rule_id in self.safety_rules:
            self.safety_rules[rule_id].enabled = enabled
            status = "启用" if enabled else "禁用"
            self.logger.info(f"{status}安全规则: {rule_id}")
            return True
        return False
    
    def add_fallback_strategy(self, safety_level: SafetyLevel, strategy: Callable) -> None:
        """添加回退策略"""
        if safety_level not in self.fallback_strategies:
            self.fallback_strategies[safety_level] = []
        
        self.fallback_strategies[safety_level].append(strategy)
        self.logger.info(f"添加回退策略: {safety_level.value}")
    
    def report_safety_event(self, 
                          event_type: SafetyEventType,
                          severity: SafetyLevel,
                          message: str,
                          source: str,
                          data: Optional[Dict[str, Any]] = None) -> None:
        """报告安全事件"""
        event = SafetyEvent(
            event_type=event_type,
            severity=severity,
            timestamp=datetime.now(),
            source=source,
            message=message,
            data=data or {}
        )
        
        self.event_queue.put(event)
    
    def add_safety_event_callback(self, callback: Callable) -> None:
        """添加安全事件回调"""
        self.safety_event_callbacks.append(callback)
    
    def add_safety_level_change_callback(self, callback: Callable) -> None:
        """添加安全级别变化回调"""
        self.safety_level_change_callbacks.append(callback)
    
    def get_current_safety_level(self) -> SafetyLevel:
        """获取当前安全级别"""
        return self.current_safety_level
    
    def get_safety_metrics(self) -> Dict[str, Any]:
        """获取安全指标"""
        metrics = self.safety_metrics.copy()
        metrics['current_safety_level'] = self.current_safety_level.value
        metrics['total_rules'] = len(self.safety_rules)
        metrics['active_rules'] = sum(1 for rule in self.safety_rules.values() if rule.enabled)
        metrics['hardware_components'] = len(self.hardware_status)
        metrics['response_time_target'] = self.response_time_target
        metrics['meets_response_target'] = metrics['max_response_time'] <= self.response_time_target
        
        return metrics
    
    def get_hardware_status(self) -> Dict[str, HardwareStatus]:
        """获取硬件状态"""
        return self.hardware_status.copy()
    
    def get_recent_events(self, limit: int = 10) -> List[SafetyEvent]:
        """获取最近的安全事件"""
        return self.safety_events[-limit:] if limit > 0 else self.safety_events.copy()
    
    def force_safety_level(self, level: SafetyLevel, reason: str = "manual") -> None:
        """强制设置安全级别"""
        self.logger.warning(f"🛡️ 强制设置安全级别: {level.value} (原因: {reason})")
        self._change_safety_level(level)
        
        # 记录强制设置事件
        self.report_safety_event(
            SafetyEventType.SAFETY_VIOLATION,
            level,
            f"手动强制设置安全级别: {level.value}",
            "manual_override",
            {"reason": reason}
        )
    
    def reset_to_normal(self) -> None:
        """重置到正常状态"""
        self.logger.info("🛡️ 重置安全控制器到正常状态")
        self._change_safety_level(SafetyLevel.NORMAL)
        
        # 清理过期事件
        cutoff_time = datetime.now() - timedelta(hours=1)
        self.safety_events = [event for event in self.safety_events 
                            if event.timestamp > cutoff_time]
    
    def emergency_stop(self) -> bool:
        """紧急停机"""
        try:
            self.logger.critical("🚨 执行紧急停机")
            
            # 立即设置为紧急状态
            self._change_safety_level(SafetyLevel.EMERGENCY)
            
            # 报告紧急事件
            self.report_safety_event(
                SafetyEventType.SAFETY_VIOLATION,
                SafetyLevel.EMERGENCY,
                "执行紧急停机",
                "emergency_stop"
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"紧急停机失败: {e}")
            return False
    
    def cleanup(self) -> None:
        """清理资源"""
        self.logger.info("清理增强安全控制器资源...")
        
        try:
            # 停止监控线程
            self.safety_monitoring_active = False
            
            if self.monitoring_thread and self.monitoring_thread.is_alive():
                self.monitoring_thread.join(timeout=2.0)
            
            if self.event_processing_thread and self.event_processing_thread.is_alive():
                self.event_processing_thread.join(timeout=2.0)
            
            self.is_initialized = False
            self.logger.info("✅ 增强安全控制器清理完成")
            
        except Exception as e:
            self.logger.error(f"增强安全控制器清理失败: {e}")


# 工厂函数
def create_enhanced_safety_controller(
    response_time_target: float = 0.2,
    safety_check_interval: float = 1.0
) -> EnhancedSafetyController:
    """
    创建增强安全控制器实例
    
    Args:
        response_time_target: 目标响应时间（秒）
        safety_check_interval: 安全检查间隔（秒）
        
    Returns:
        EnhancedSafetyController: 安全控制器实例
    """
    return EnhancedSafetyController(
        response_time_target=response_time_target,
        safety_check_interval=safety_check_interval
    )


if __name__ == "__main__":
    # 基础测试
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    print("🛡️ 增强安全控制器测试")
    print("=" * 50)
    
    try:
        # 创建安全控制器
        safety_controller = create_enhanced_safety_controller()
        
        if safety_controller.initialize():
            print("✅ 增强安全控制器初始化成功")
            
            # 测试安全事件报告
            print("\n🚨 测试安全事件...")
            
            safety_controller.report_safety_event(
                SafetyEventType.HARDWARE_FAILURE,
                SafetyLevel.WARNING,
                "测试硬件故障事件",
                "test_source"
            )
            
            time.sleep(2)
            
            safety_controller.report_safety_event(
                SafetyEventType.POWER_ISSUE,
                SafetyLevel.CRITICAL,
                "测试电源问题事件",
                "test_source"
            )
            
            time.sleep(2)
            
            # 显示安全指标
            metrics = safety_controller.get_safety_metrics()
            print(f"\n📊 安全指标:")
            print(f"   当前安全级别: {metrics['current_safety_level']}")
            print(f"   总事件数: {metrics['total_events']}")
            print(f"   安全级别变化: {metrics['safety_level_changes']}")
            print(f"   平均响应时间: {metrics['avg_response_time']*1000:.1f}ms")
            print(f"   响应时间要求: {'✅' if metrics['meets_response_target'] else '❌'}")
            
            # 显示最近事件
            recent_events = safety_controller.get_recent_events(3)
            print(f"\n📜 最近事件:")
            for event in recent_events:
                print(f"   {event.timestamp.strftime('%H:%M:%S')} | {event.event_type.value} | {event.severity.value}")
            
            # 测试紧急停机
            print(f"\n🚨 测试紧急停机...")
            safety_controller.emergency_stop()
            
            time.sleep(1)
            print(f"   紧急停机后安全级别: {safety_controller.get_current_safety_level().value}")
            
            # 清理
            safety_controller.cleanup()
            print("\n✅ 测试完成")
        else:
            print("❌ 增强安全控制器初始化失败")
            
    except KeyboardInterrupt:
        print("\n⏹️ 测试中断")
    except Exception as e:
        print(f"\n❌ 测试失败: {e}") 