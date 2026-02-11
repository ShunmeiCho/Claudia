#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia LED状态机模块
实现LED模式的优先级管理、状态切换和冲突解决

Author: Claudia AI System
Generated: 2025-06-30
Purpose: 子任务6.2 - LED模式定义与状态机实现
"""

import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import queue
from datetime import datetime, timedelta

# 添加项目路径
sys.path.append('/home/m1ng/claudia/src')

# 导入LED模式定义
try:
    from claudia.robot_controller.led_patterns import (
        ClaudiaLEDMode, LEDPattern, ClaudiaLEDModeDefinitions, 
        LEDModeRenderer, create_led_mode_renderer
    )
    # 🧠 Phase 2: 导入系统状态监控器相关类
    from claudia.robot_controller.system_state_monitor import (
        SystemState, SystemLEDPriority, SystemStateInfo, LEDControlDecision
    )
    LED_PATTERNS_AVAILABLE = True
    SYSTEM_STATE_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ LED模式定义导入失败: {e}")
    LED_PATTERNS_AVAILABLE = False
    SYSTEM_STATE_AVAILABLE = False

@dataclass
class LEDStateRequest:
    """LED状态请求"""
    mode: 'ClaudiaLEDMode'
    priority: int
    duration: Optional[float] = None      # 持续时间覆盖
    timestamp: datetime = field(default_factory=datetime.now)
    source: str = "unknown"               # 请求来源
    auto_revert: bool = True              # 是否自动回退到前一状态
    interrupt_lower: bool = True          # 是否可以中断低优先级状态

@dataclass 
class LEDStateHistory:
    """LED状态历史记录"""
    mode: 'ClaudiaLEDMode'
    start_time: datetime
    end_time: Optional[datetime] = None
    duration: Optional[float] = None
    source: str = "unknown"
    interrupted: bool = False

class DynamicPriorityManager:
    """
    🧠 Phase 2: 动态优先级管理器
    
    根据系统状态动态调整LED控制优先级，实现智能决策
    """
    
    def __init__(self):
        """初始化动态优先级管理器"""
        self.logger = logging.getLogger(__name__)
        self.current_system_state: Optional[SystemStateInfo] = None
        self.base_priority_mapping = {
            # Claudia专用模式的基础优先级
            ClaudiaLEDMode.OFF: 1,
            ClaudiaLEDMode.WAKE_CONFIRM: 7,
            ClaudiaLEDMode.PROCESSING_VOICE: 6,
            ClaudiaLEDMode.EXECUTING_ACTION: 8,
            ClaudiaLEDMode.ACTION_COMPLETE: 9,
            ClaudiaLEDMode.ERROR_STATE: 10,
            # 系统兼容性模式
            ClaudiaLEDMode.SYSTEM_BOOT: 3,
            ClaudiaLEDMode.SYSTEM_CALIBRATION: 4,
            ClaudiaLEDMode.LOW_BATTERY: 5,
            ClaudiaLEDMode.SEARCH_LIGHT: 2
        }
        
        # 系统状态调节表
        self.system_state_adjustments = {
            SystemState.NORMAL: 0,           # 正常状态无调节
            SystemState.IDLE: 0,             # 空闲状态无调节
            SystemState.ACTIVE: 0,           # 活跃状态无调节
            SystemState.CALIBRATING: +2,     # 校准时提升系统相关模式优先级
            SystemState.LOW_BATTERY: +3,     # 低电量时提升安全相关模式优先级
            SystemState.ERROR: +5,           # 错误时大幅提升错误相关模式优先级
            SystemState.EMERGENCY: +7,       # 紧急状态时最大幅度提升关键模式优先级
            SystemState.UNKNOWN: 0          # 未知状态无调节
        }
        
        # 优先级动态调节历史
        self.adjustment_history: List[Tuple[datetime, SystemState, int]] = []
        self.max_history_size = 50
        
    def update_system_state(self, system_state_info: SystemStateInfo) -> None:
        """
        更新系统状态信息
        
        Args:
            system_state_info: 系统状态信息
        """
        previous_state = self.current_system_state.state if self.current_system_state else SystemState.UNKNOWN
        self.current_system_state = system_state_info
        
        # 记录状态变化调节
        if previous_state != system_state_info.state:
            adjustment = self.system_state_adjustments.get(system_state_info.state, 0)
            self.adjustment_history.append((datetime.now(), system_state_info.state, adjustment))
            
            # 限制历史大小
            if len(self.adjustment_history) > self.max_history_size:
                self.adjustment_history.pop(0)
            
            self.logger.info(f"🧠 系统状态变化影响优先级: {previous_state.name} → {system_state_info.state.name} "
                           f"(调节: {adjustment:+d})")
    
    def calculate_dynamic_priority(self, mode: ClaudiaLEDMode, base_priority: Optional[int] = None) -> int:
        """
        计算动态调整后的优先级
        
        Args:
            mode: LED模式
            base_priority: 基础优先级（如果为None，使用默认映射）
            
        Returns:
            int: 动态调整后的优先级 (1-10)
        """
        # 获取基础优先级
        if base_priority is None:
            base_priority = self.base_priority_mapping.get(mode, 5)
        
        # 如果没有系统状态信息，返回基础优先级
        if not self.current_system_state:
            return max(1, min(10, base_priority))
        
        # 计算系统状态调节
        system_adjustment = self.system_state_adjustments.get(
            self.current_system_state.state, 0
        )
        
        # 计算模式特定调节
        mode_adjustment = self._calculate_mode_specific_adjustment(mode, self.current_system_state)
        
        # 应用动态调节
        dynamic_priority = base_priority + system_adjustment + mode_adjustment
        
        # 限制在有效范围内
        final_priority = max(1, min(10, dynamic_priority))
        
        self.logger.debug(f"动态优先级计算: {mode.value} | 基础={base_priority} + 系统调节={system_adjustment} + "
                         f"模式调节={mode_adjustment} = {final_priority}")
        
        return final_priority
    
    def _calculate_mode_specific_adjustment(self, mode: ClaudiaLEDMode, system_state: SystemStateInfo) -> int:
        """
        计算模式特定的优先级调节
        
        Args:
            mode: LED模式
            system_state: 系统状态信息
            
        Returns:
            int: 模式特定调节值
        """
        adjustment = 0
        
        # 错误状态相关调节
        if mode == ClaudiaLEDMode.ERROR_STATE:
            if system_state.state in [SystemState.ERROR, SystemState.EMERGENCY]:
                adjustment += 2  # 系统错误时进一步提升错误LED优先级
            elif len(system_state.error_codes) > 0:
                adjustment += 1  # 有错误代码时适度提升
        
        # 低电量相关调节
        elif mode == ClaudiaLEDMode.LOW_BATTERY:
            if system_state.battery_level <= 0.05:  # 极低电量
                adjustment += 3
            elif system_state.battery_level <= 0.15:  # 低电量
                adjustment += 2
        
        # 校准相关调节
        elif mode == ClaudiaLEDMode.SYSTEM_CALIBRATION:
            if system_state.state == SystemState.CALIBRATING:
                adjustment += 2  # 校准期间提升校准LED优先级
        
        # 用户交互模式在系统忙碌时的调节
        elif mode in [ClaudiaLEDMode.WAKE_CONFIRM, ClaudiaLEDMode.PROCESSING_VOICE, ClaudiaLEDMode.EXECUTING_ACTION]:
            if system_state.state in [SystemState.LOW_BATTERY, SystemState.ERROR]:
                adjustment -= 1  # 系统问题时降低用户交互优先级
            elif system_state.state == SystemState.EMERGENCY:
                adjustment -= 2  # 紧急状态时大幅降低
        
        return adjustment
    
    def get_led_control_decision(self, mode: ClaudiaLEDMode, requested_priority: int) -> LEDControlDecision:
        """
        获取LED控制决策
        
        Args:
            mode: 请求的LED模式
            requested_priority: 请求的优先级
            
        Returns:
            LEDControlDecision: 控制决策
        """
        if not self.current_system_state:
            # 没有系统状态信息，允许控制
            return LEDControlDecision(
                allow_custom_control=True,
                required_priority=SystemLEDPriority.NORMAL,
                system_override_active=False,
                recommended_action="proceed",
                reason="无系统状态信息，允许控制"
            )
        
        # 计算动态优先级要求
        dynamic_priority = self.calculate_dynamic_priority(mode)
        system_priority = self.current_system_state.priority.value
        
        # 判断是否允许控制
        allow_control = requested_priority >= max(dynamic_priority, system_priority)
        
        # 确定推荐优先级
        required_priority = SystemLEDPriority(min(10, max(1, max(dynamic_priority, system_priority))))
        
        # 检查系统强制覆盖
        system_override = self.current_system_state.state in [
            SystemState.EMERGENCY, SystemState.ERROR, SystemState.LOW_BATTERY
        ]
        
        # 生成决策原因
        if allow_control:
            reason = f"系统状态: {self.current_system_state.state.name}, 动态优先级: {dynamic_priority}"
        else:
            reason = f"优先级不足 (需要: {required_priority.value}, 请求: {requested_priority})"
        
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
    
    def should_auto_switch_mode(self, current_mode: ClaudiaLEDMode) -> Optional[ClaudiaLEDMode]:
        """
        判断是否应该自动切换LED模式
        
        Args:
            current_mode: 当前LED模式
            
        Returns:
            Optional[ClaudiaLEDMode]: 建议切换的模式，None表示不需要切换
        """
        if not self.current_system_state:
            return None
        
        system_state = self.current_system_state.state
        
        # 紧急状态自动切换
        if system_state == SystemState.EMERGENCY:
            if current_mode != ClaudiaLEDMode.ERROR_STATE:
                return ClaudiaLEDMode.ERROR_STATE
        
        # 错误状态自动切换
        elif system_state == SystemState.ERROR:
            if current_mode not in [ClaudiaLEDMode.ERROR_STATE]:
                return ClaudiaLEDMode.ERROR_STATE
        
        # 极低电量自动切换
        elif (system_state == SystemState.LOW_BATTERY and 
              self.current_system_state.battery_level <= 0.05):
            if current_mode != ClaudiaLEDMode.ERROR_STATE:
                return ClaudiaLEDMode.ERROR_STATE  # 极低电量用错误模式
        
        # 低电量自动切换
        elif system_state == SystemState.LOW_BATTERY:
            if current_mode != ClaudiaLEDMode.LOW_BATTERY:
                return ClaudiaLEDMode.LOW_BATTERY
        
        # 校准状态自动切换
        elif system_state == SystemState.CALIBRATING:
            if current_mode != ClaudiaLEDMode.SYSTEM_CALIBRATION:
                return ClaudiaLEDMode.SYSTEM_CALIBRATION
        
        return None
    
    def get_adjustment_statistics(self) -> Dict[str, Any]:
        """获取优先级调节统计信息"""
        if not self.adjustment_history:
            return {"total_adjustments": 0}
        
        # 统计各种系统状态的调节次数
        state_counts = {}
        total_adjustments = len(self.adjustment_history)
        
        for _, state, adjustment in self.adjustment_history:
            state_name = state.name
            if state_name not in state_counts:
                state_counts[state_name] = {"count": 0, "total_adjustment": 0}
            
            state_counts[state_name]["count"] += 1
            state_counts[state_name]["total_adjustment"] += adjustment
        
        # 计算平均调节
        for state_info in state_counts.values():
            if state_info["count"] > 0:
                state_info["average_adjustment"] = state_info["total_adjustment"] / state_info["count"]
        
        return {
            "total_adjustments": total_adjustments,
            "state_adjustment_stats": state_counts,
            "current_system_state": self.current_system_state.state.name if self.current_system_state else None,
            "history_size": len(self.adjustment_history)
        }

class LEDStateMachine:
    """
    Claudia LED状态机
    
    负责管理LED模式的优先级、状态切换和冲突解决
    确保系统兼容性并避免干扰默认LED状态
    🧠 Phase 2: 集成动态优先级管理和系统状态感知
    """
    
    def __init__(self, response_time_target: float = 0.2):
        """
        初始化LED状态机
        
        Args:
            response_time_target: 目标响应时间（秒）
        """
        self.logger = logging.getLogger(__name__)
        self.response_time_target = response_time_target
        
        # 核心组件
        self.renderer = None
        self.is_initialized = False
        
        # 🧠 Phase 2: 动态优先级管理器
        self.dynamic_priority_manager = DynamicPriorityManager() if SYSTEM_STATE_AVAILABLE else None
        self.current_system_state = None
        
        # 状态管理
        self.current_state = ClaudiaLEDMode.OFF
        self.current_priority = 1
        self.state_lock = threading.Lock()
        
        # 请求队列和处理
        self.request_queue = queue.PriorityQueue()
        self.processing_thread = None
        self.processing_active = False
        
        # 状态历史和回退
        self.state_history: List[LEDStateHistory] = []
        self.previous_state_stack: List[Tuple[ClaudiaLEDMode, int]] = []
        self.max_history_size = 100
        
        # 系统兼容性管理
        self.system_override_enabled = True
        self.protected_system_modes = {
            ClaudiaLEDMode.SYSTEM_BOOT,
            ClaudiaLEDMode.SYSTEM_CALIBRATION, 
            ClaudiaLEDMode.LOW_BATTERY
        }
        
        # 🧠 Phase 2: 自动模式切换
        self.auto_mode_switching_enabled = True
        self.last_auto_switch_check = 0
        self.auto_switch_check_interval = 1.0  # 1秒检查一次
        
        # 性能监控
        self.performance_metrics = {
            'state_changes': 0,
            'average_response_time': 0.0,
            'max_response_time': 0.0,
            'queue_overflows': 0,
            'priority_conflicts': 0,
            'dynamic_priority_adjustments': 0,  # 新增：动态优先级调整次数
            'auto_mode_switches': 0            # 新增：自动模式切换次数
        }
        
        self.logger.info("LED状态机初始化完成 (Phase 2: 智能决策版本)")
    
    def initialize(self) -> bool:
        """
        初始化LED状态机
        
        Returns:
            bool: 初始化是否成功
        """
        if not LED_PATTERNS_AVAILABLE:
            self.logger.error("LED模式定义不可用")
            return False
            
        try:
            self.logger.info("初始化LED状态机...")
            
            # 创建LED模式渲染器
            self.renderer = create_led_mode_renderer()
            if not self.renderer.initialize_vui():
                self.logger.error("LED渲染器初始化失败")
                return False
                
            # 启动请求处理线程
            self.processing_active = True
            self.processing_thread = threading.Thread(
                target=self._request_processing_worker,
                daemon=True
            )
            self.processing_thread.start()
            
            # 设置初始状态
            self._record_state_change(ClaudiaLEDMode.OFF, "system", 0.0)
            
            self.is_initialized = True
            self.logger.info("✅ LED状态机初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"LED状态机初始化失败: {e}")
            return False
    
    def request_state(self, 
                     mode: ClaudiaLEDMode, 
                     source: str = "user",
                     duration: Optional[float] = None,
                     priority_override: Optional[int] = None) -> bool:
        """
        请求LED状态变更
        🧠 Phase 2: 集成动态优先级计算和智能决策
        
        Args:
            mode: 目标LED模式
            source: 请求来源标识
            duration: 可选的持续时间覆盖
            priority_override: 可选的优先级覆盖
            
        Returns:
            bool: 请求是否被接受
        """
        if not self.is_initialized:
            self.logger.error("LED状态机未初始化")
            return False
        
        # 获取模式参数
        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        if not ClaudiaLEDModeDefinitions.validate_pattern(pattern):
            self.logger.error(f"无效的LED模式: {mode}")
            return False
        
        # 🧠 Phase 2: 使用动态优先级计算
        if priority_override is not None:
            effective_priority = priority_override
        elif self.dynamic_priority_manager:
            # 使用动态优先级管理器计算优先级
            effective_priority = self.dynamic_priority_manager.calculate_dynamic_priority(mode, pattern.priority)
            self.logger.debug(f"🧠 动态优先级计算: {mode.value} 基础={pattern.priority} → 动态={effective_priority}")
        else:
            effective_priority = pattern.priority
        
        # 🧠 Phase 2: 检查LED控制决策
        if self.dynamic_priority_manager:
            control_decision = self.dynamic_priority_manager.get_led_control_decision(mode, effective_priority)
            
            if not control_decision.allow_custom_control:
                self.logger.warning(f"🛡️ LED控制决策拒绝请求: {control_decision.reason}")
                self.logger.info(f"💡 建议操作: {control_decision.recommended_action}")
                return False
            
            # 如果有更高的推荐优先级，使用它
            if control_decision.required_priority.value > effective_priority:
                effective_priority = control_decision.required_priority.value
                self.logger.debug(f"🔝 提升优先级至推荐值: {effective_priority}")
        
        # 系统兼容性检查（增强版）
        if not self._check_system_compatibility(mode, effective_priority):
            self.logger.warning(f"系统兼容性检查失败，拒绝状态请求: {mode}")
            return False
        
        # 创建状态请求
        request = LEDStateRequest(
            mode=mode,
            priority=effective_priority,
            duration=duration,
            source=source,
            auto_revert=True,
            interrupt_lower=True
        )
        
        try:
            # 使用负优先级确保高优先级请求先处理
            self.request_queue.put((-effective_priority, time.time(), request), timeout=1.0)
            self.logger.debug(f"LED状态请求已排队: {mode.value} (动态优先级={effective_priority}, 来源={source})")
            return True
            
        except queue.Full:
            self.logger.error("LED状态请求队列已满")
            self.performance_metrics['queue_overflows'] += 1
            return False
    
    def _request_processing_worker(self) -> None:
        """LED状态请求处理工作线程"""
        while self.processing_active:
            try:
                # 等待请求
                try:
                    neg_priority, timestamp, request = self.request_queue.get(timeout=1.0)
                    actual_priority = -neg_priority
                except queue.Empty:
                    continue
                
                # 处理请求
                self._process_state_request(request, timestamp)
                self.request_queue.task_done()
                
            except Exception as e:
                self.logger.error(f"状态请求处理失败: {e}")
    
    def _process_state_request(self, request: LEDStateRequest, request_timestamp: float) -> None:
        """
        处理单个LED状态请求
        
        Args:
            request: LED状态请求
            request_timestamp: 请求时间戳
        """
        process_start = time.time()
        
        with self.state_lock:
            # 检查优先级和冲突
            can_interrupt = self._can_interrupt_current_state(request.priority, request.interrupt_lower)
            
            if not can_interrupt:
                self.logger.debug(f"优先级不足，忽略状态请求: {request.mode.value} (请求优先级={request.priority}, 当前优先级={self.current_priority})")
                self.performance_metrics['priority_conflicts'] += 1
                return
            
            # 保存当前状态到回退栈
            if self.current_state != ClaudiaLEDMode.OFF and request.auto_revert:
                self.previous_state_stack.append((self.current_state, self.current_priority))
                # 限制栈大小
                if len(self.previous_state_stack) > 10:
                    self.previous_state_stack.pop(0)
            
            # 执行状态切换
            self._execute_state_change(request, process_start)
            
            # 计算和记录响应时间
            response_time = time.time() - request_timestamp
            self._update_performance_metrics(response_time)
            
            if response_time > self.response_time_target:
                self.logger.warning(f"LED状态切换响应时间超标: {response_time*1000:.1f}ms > {self.response_time_target*1000}ms")
    
    def _execute_state_change(self, request: LEDStateRequest, start_time: float) -> None:
        """
        执行LED状态变更
        
        Args:
            request: LED状态请求
            start_time: 开始时间
        """
        try:
            # 结束当前状态记录
            self._end_current_state_record()
            
            # 更新当前状态
            self.current_state = request.mode
            self.current_priority = request.priority
            
            # 启动LED渲染
            if self.renderer:
                success = self.renderer.render_mode(request.mode, request.duration)
                if success:
                    self.logger.info(f"LED状态切换成功: {request.mode.value} (优先级={request.priority}, 来源={request.source})")
                else:
                    self.logger.error(f"LED渲染失败: {request.mode.value}")
            else:
                self.logger.error("LED渲染器不可用")
                
            # 记录状态变更
            self._record_state_change(request.mode, request.source, time.time() - start_time)
            
            # 如果有持续时间，安排自动回退
            if request.duration and request.duration > 0 and request.auto_revert:
                self._schedule_auto_revert(request.duration)
                
        except Exception as e:
            self.logger.error(f"状态变更执行失败: {e}")
    
    def _can_interrupt_current_state(self, new_priority: int, interrupt_lower: bool) -> bool:
        """
        检查是否可以中断当前状态
        
        Args:
            new_priority: 新请求的优先级
            interrupt_lower: 是否允许中断低优先级状态
            
        Returns:
            bool: 是否可以中断
        """
        # 系统模式保护
        if self.current_state in self.protected_system_modes and self.system_override_enabled:
            # 只有更高优先级的系统模式可以中断
            return new_priority > self.current_priority and new_priority >= 8
        
        # 普通优先级比较
        if interrupt_lower:
            return new_priority >= self.current_priority
        else:
            return new_priority > self.current_priority
    
    def _check_system_compatibility(self, mode: ClaudiaLEDMode, priority: int) -> bool:
        """
        检查系统兼容性
        
        Args:
            mode: LED模式
            priority: 请求优先级
            
        Returns:
            bool: 是否兼容
        """
        # 检查是否与保护的系统模式冲突
        if self.current_state in self.protected_system_modes:
            # 低优先级请求不能中断系统模式
            if priority < self.current_priority:
                return False
        
        # 检查是否是被保护的系统模式
        if mode in self.protected_system_modes and not self.system_override_enabled:
            return False
            
        return True
    
    def _schedule_auto_revert(self, delay: float) -> None:
        """
        安排自动回退到前一状态
        
        Args:
            delay: 延迟时间（秒）
        """
        def auto_revert_worker():
            time.sleep(delay)
            
            with self.state_lock:
                # 检查是否还需要回退
                if self.previous_state_stack:
                    prev_mode, prev_priority = self.previous_state_stack.pop()
                    
                    # 创建回退请求
                    revert_request = LEDStateRequest(
                        mode=prev_mode,
                        priority=prev_priority,
                        source="auto_revert",
                        auto_revert=False,
                        interrupt_lower=False
                    )
                    
                    self._process_state_request(revert_request, time.time())
                    self.logger.debug(f"自动回退到前一状态: {prev_mode.value}")
                else:
                    # 没有前一状态，回退到OFF
                    off_request = LEDStateRequest(
                        mode=ClaudiaLEDMode.OFF,
                        priority=1,
                        source="auto_revert",
                        auto_revert=False
                    )
                    self._process_state_request(off_request, time.time())
        
        # 启动回退线程
        revert_thread = threading.Thread(target=auto_revert_worker, daemon=True)
        revert_thread.start()
    
    def _record_state_change(self, mode: ClaudiaLEDMode, source: str, response_time: float) -> None:
        """
        记录状态变更到历史
        
        Args:
            mode: LED模式
            source: 来源
            response_time: 响应时间
        """
        # 创建历史记录
        history_entry = LEDStateHistory(
            mode=mode,
            start_time=datetime.now(),
            source=source
        )
        
        self.state_history.append(history_entry)
        
        # 限制历史大小
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)
        
        # 更新性能指标
        self.performance_metrics['state_changes'] += 1
    
    def _end_current_state_record(self) -> None:
        """结束当前状态记录"""
        if self.state_history:
            current_record = self.state_history[-1]
            if current_record.end_time is None:
                current_record.end_time = datetime.now()
                duration = (current_record.end_time - current_record.start_time).total_seconds()
                current_record.duration = duration
    
    def _update_performance_metrics(self, response_time: float) -> None:
        """
        更新性能指标
        
        Args:
            response_time: 响应时间
        """
        # 更新平均响应时间
        total_changes = self.performance_metrics['state_changes']
        if total_changes > 0:
            current_avg = self.performance_metrics['average_response_time']
            new_avg = (current_avg * (total_changes - 1) + response_time) / total_changes
            self.performance_metrics['average_response_time'] = new_avg
        
        # 更新最大响应时间
        if response_time > self.performance_metrics['max_response_time']:
            self.performance_metrics['max_response_time'] = response_time
    
    def get_current_state(self) -> Tuple[ClaudiaLEDMode, int]:
        """
        获取当前LED状态
        
        Returns:
            Tuple[ClaudiaLEDMode, int]: (当前模式, 当前优先级)
        """
        with self.state_lock:
            return self.current_state, self.current_priority
    
    def get_state_history(self, limit: int = 10) -> List[LEDStateHistory]:
        """
        获取状态历史记录
        
        Args:
            limit: 返回记录数量限制
            
        Returns:
            List[LEDStateHistory]: 历史记录列表
        """
        with self.state_lock:
            return self.state_history[-limit:] if limit > 0 else self.state_history.copy()
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        获取性能指标
        
        Returns:
            Dict[str, Any]: 性能指标字典
        """
        with self.state_lock:
            metrics = self.performance_metrics.copy()
            metrics['meets_response_requirement'] = metrics['max_response_time'] <= self.response_time_target
            metrics['average_response_time_ms'] = metrics['average_response_time'] * 1000
            metrics['max_response_time_ms'] = metrics['max_response_time'] * 1000
            return metrics
    
    def force_state(self, mode: ClaudiaLEDMode, source: str = "force") -> bool:
        """
        强制设置LED状态（忽略优先级）
        
        Args:
            mode: 目标LED模式
            source: 来源标识
            
        Returns:
            bool: 是否成功
        """
        if not self.is_initialized:
            self.logger.error("LED状态机未初始化")
            return False
        
        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        
        # 创建高优先级强制请求
        request = LEDStateRequest(
            mode=mode,
            priority=10,  # 最高优先级
            source=source,
            auto_revert=False,
            interrupt_lower=True
        )
        
        # 直接处理，绕过队列
        with self.state_lock:
            self._execute_state_change(request, time.time())
            
        self.logger.info(f"强制设置LED状态: {mode.value}")
        return True
    
    def emergency_stop(self) -> bool:
        """
        紧急停止所有LED活动
        
        Returns:
            bool: 是否成功
        """
        self.logger.warning("紧急停止LED状态机")
        
        try:
            # 停止渲染器
            if self.renderer:
                self.renderer.stop_all_rendering()
            
            # 清空请求队列
            while not self.request_queue.empty():
                try:
                    self.request_queue.get_nowait()
                    self.request_queue.task_done()
                except queue.Empty:
                    break
            
            # 重置状态
            with self.state_lock:
                self.current_state = ClaudiaLEDMode.OFF
                self.current_priority = 1
                self.previous_state_stack.clear()
                
            return True
            
        except Exception as e:
            self.logger.error(f"紧急停止失败: {e}")
            return False
    
    def set_system_override(self, enabled: bool) -> None:
        """
        设置系统覆盖模式
        
        Args:
            enabled: 是否启用系统模式保护
        """
        self.system_override_enabled = enabled
        self.logger.info(f"系统覆盖模式: {'启用' if enabled else '禁用'}")
    
    def cleanup(self) -> None:
        """清理资源"""
        self.logger.info("清理LED状态机资源...")
        
        try:
            # 停止处理线程
            self.processing_active = False
            if self.processing_thread and self.processing_thread.is_alive():
                self.processing_thread.join(timeout=2.0)
            
            # 清理渲染器
            if self.renderer:
                self.renderer.cleanup()
                
            # 最后的状态记录
            self._end_current_state_record()
            
            self.is_initialized = False
            self.logger.info("✅ LED状态机清理完成")
            
        except Exception as e:
            self.logger.error(f"LED状态机清理失败: {e}")

    def update_system_state(self, system_state_info: SystemStateInfo) -> None:
        """
        🧠 Phase 2: 更新系统状态并触发相关处理
        
        Args:
            system_state_info: 系统状态信息
        """
        if not self.dynamic_priority_manager:
            self.logger.warning("动态优先级管理器不可用，无法更新系统状态")
            return
        
        previous_state = self.current_system_state
        self.current_system_state = system_state_info
        
        # 更新动态优先级管理器
        self.dynamic_priority_manager.update_system_state(system_state_info)
        
        self.logger.info(f"🧠 系统状态更新: {system_state_info.state.name}")
        
        # 检查是否需要自动切换模式
        if self.auto_mode_switching_enabled:
            self._check_auto_mode_switch()
        
        # 重新评估当前状态的优先级
        self._reevaluate_current_priority()
    
    def _check_auto_mode_switch(self) -> None:
        """
        🧠 Phase 2: 检查是否需要自动切换LED模式
        """
        current_time = time.time()
        if current_time - self.last_auto_switch_check < self.auto_switch_check_interval:
            return
            
        self.last_auto_switch_check = current_time
        
        if not self.dynamic_priority_manager:
            return
        
        # 检查是否建议自动切换
        suggested_mode = self.dynamic_priority_manager.should_auto_switch_mode(self.current_state)
        
        if suggested_mode and suggested_mode != self.current_state:
            self.logger.info(f"🔄 系统建议自动切换模式: {self.current_state.value} → {suggested_mode.value}")
            
            # 获取建议模式的动态优先级
            dynamic_priority = self.dynamic_priority_manager.calculate_dynamic_priority(suggested_mode)
            
            # 只有当建议模式的优先级高于当前时才切换
            if dynamic_priority > self.current_priority:
                success = self.request_state(
                    mode=suggested_mode,
                    source="auto_switch",
                    priority_override=dynamic_priority
                )
                
                if success:
                    self.performance_metrics['auto_mode_switches'] += 1
                    self.logger.info(f"✅ 自动模式切换成功: {suggested_mode.value}")
                else:
                    self.logger.warning(f"❌ 自动模式切换失败: {suggested_mode.value}")
    
    def _reevaluate_current_priority(self) -> None:
        """
        🧠 Phase 2: 重新评估当前状态的优先级
        """
        if not self.dynamic_priority_manager or self.current_state == ClaudiaLEDMode.OFF:
            return
        
        # 计算当前模式的新动态优先级
        new_priority = self.dynamic_priority_manager.calculate_dynamic_priority(
            self.current_state, self.current_priority
        )
        
        if new_priority != self.current_priority:
            self.logger.debug(f"🔄 重新评估优先级: {self.current_state.value} "
                            f"{self.current_priority} → {new_priority}")
            
            with self.state_lock:
                self.current_priority = new_priority
            
            self.performance_metrics['dynamic_priority_adjustments'] += 1
    
    def get_led_control_decision(self, mode: ClaudiaLEDMode, requested_priority: int) -> Optional['LEDControlDecision']:
        """
        🧠 Phase 2: 获取LED控制决策
        
        Args:
            mode: 请求的LED模式
            requested_priority: 请求的优先级
            
        Returns:
            Optional[LEDControlDecision]: 控制决策（如果动态优先级管理器可用）
        """
        if not self.dynamic_priority_manager:
            return None
        
        return self.dynamic_priority_manager.get_led_control_decision(mode, requested_priority)
    
    def set_auto_mode_switching(self, enabled: bool) -> None:
        """
        🧠 Phase 2: 设置自动模式切换
        
        Args:
            enabled: 是否启用自动模式切换
        """
        self.auto_mode_switching_enabled = enabled
        self.logger.info(f"自动模式切换: {'启用' if enabled else '禁用'}")
    
    def get_dynamic_priority_statistics(self) -> Optional[Dict[str, Any]]:
        """
        🧠 Phase 2: 获取动态优先级统计信息
        
        Returns:
            Optional[Dict[str, Any]]: 统计信息（如果可用）
        """
        if not self.dynamic_priority_manager:
            return None
        
        return self.dynamic_priority_manager.get_adjustment_statistics()


# 工厂函数
def create_led_state_machine() -> LEDStateMachine:
    """
    创建LED状态机实例
    
    Returns:
        LEDStateMachine: 状态机实例
    """
    return LEDStateMachine()


if __name__ == "__main__":
    # 基础测试
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    print("🧪 LED状态机测试")
    print("=" * 50)
    
    try:
        # 创建状态机
        state_machine = create_led_state_machine()
        
        if state_machine.initialize():
            print("✅ LED状态机初始化成功")
            
            # 测试状态切换序列
            print("\n🔄 测试状态切换序列...")
            
            # 1. 唤醒确认
            print("1. 🟢 唤醒确认 (2秒)")
            state_machine.request_state(ClaudiaLEDMode.WAKE_CONFIRM, "test")
            time.sleep(3)
            
            # 2. 处理语音
            print("2. 🔵 处理语音 (5秒)")
            state_machine.request_state(ClaudiaLEDMode.PROCESSING_VOICE, "test", duration=5.0)
            time.sleep(2)
            
            # 3. 执行动作（高优先级，应该中断处理语音）
            print("3. 🟠 执行动作 (3秒)")
            state_machine.request_state(ClaudiaLEDMode.EXECUTING_ACTION, "test", duration=3.0)
            time.sleep(4)
            
            # 4. 动作完成
            print("4. ⚪ 动作完成")
            state_machine.request_state(ClaudiaLEDMode.ACTION_COMPLETE, "test")
            time.sleep(2)
            
            # 5. 错误状态（最高优先级）
            print("5. 🔴 错误状态")
            state_machine.request_state(ClaudiaLEDMode.ERROR_STATE, "test")
            time.sleep(3)
            
            # 显示性能指标
            metrics = state_machine.get_performance_metrics()
            print(f"\n📊 性能指标:")
            print(f"   状态变更次数: {metrics['state_changes']}")
            print(f"   平均响应时间: {metrics['average_response_time_ms']:.1f}ms")
            print(f"   最大响应时间: {metrics['max_response_time_ms']:.1f}ms")
            print(f"   响应时间要求: {'✅' if metrics['meets_response_requirement'] else '❌'}")
            print(f"   队列溢出: {metrics['queue_overflows']}")
            print(f"   优先级冲突: {metrics['priority_conflicts']}")
            
            # 显示状态历史
            history = state_machine.get_state_history(5)
            print(f"\n📜 最近状态历史:")
            for i, record in enumerate(history[-3:]):
                duration_str = f"{record.duration:.1f}s" if record.duration else "进行中"
                print(f"   {i+1}. {record.mode.value} ({record.source}) - {duration_str}")
                
        else:
            print("❌ LED状态机初始化失败")
            
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
    finally:
        state_machine.cleanup() 