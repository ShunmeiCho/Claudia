#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia统一LED控制器 - 深化环境自适应版本
集成VUI客户端和LowCmd两种控制方式，提供高级环境自适应功能

Author: Claudia AI System
Generated: 2025-06-30 
Purpose: 子任务6.3 - 深化环境自适应功能（基于子任务6.2修改）
Version: 0.3.0 (Enhanced Environmental Adaptation)
"""

import sys
import time
import threading
import logging
import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict, Any, Union
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from collections import deque
import statistics
import math

# 添加项目路径
sys.path.append('/home/m1ng/claudia/src')

# 导入LED组件
try:
    from claudia.robot_controller.led_patterns import (
        ClaudiaLEDMode, LEDPattern, ClaudiaLEDModeDefinitions,
        LEDModeRenderer, create_led_mode_renderer
    )
    from claudia.robot_controller.led_state_machine import (
        LEDStateMachine, create_led_state_machine
    )
    from claudia.robot_controller.led_controller import (
        ClaudiaLEDController, create_led_controller
    )
    # 🎯 Phase 1.3: 导入系统状态监控器
    from claudia.robot_controller.system_state_monitor import (
        SystemStateMonitor, SystemStateInfo, SystemState, SystemLEDPriority,
        LEDControlDecision, create_system_state_monitor
    )
    LED_COMPONENTS_AVAILABLE = True
    SYSTEM_MONITOR_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ LED组件导入失败: {e}")
    LED_COMPONENTS_AVAILABLE = False
    SYSTEM_MONITOR_AVAILABLE = False

class LEDControlMethod(Enum):
    """LED控制方法枚举"""
    VUI_CLIENT = "vui_client"        # VUI客户端控制（推荐）
    LOW_CMD = "low_cmd"              # LowCmd消息控制（备用）
    AUTO_SELECT = "auto_select"      # 自动选择最佳方法

class EnvironmentalLightCategory(Enum):
    """环境光线分类枚举"""
    VERY_DARK = "very_dark"          # 极暗 (<5%)
    DARK = "dark"                    # 暗 (5-20%)
    DIM = "dim"                      # 微暗 (20-40%)
    NORMAL = "normal"                # 正常 (40-70%)
    BRIGHT = "bright"                # 亮 (70-85%)
    VERY_BRIGHT = "very_bright"      # 极亮 (>85%)

@dataclass
class AdvancedEnvironmentalLightInfo:
    """高级环境光线信息"""
    # 基础光线数据
    brightness_level: float          # 整体亮度等级 (0-1)
    brightness_category: EnvironmentalLightCategory  # 亮度分类
    suggested_led_factor: float      # 建议LED调节因子 (0.1-3.0)
    timestamp: float                 # 检测时间戳
    detection_confidence: float      # 检测置信度 (0-1)
    
    # 高级分析数据
    brightness_std: float            # 亮度标准差 (均匀性指标)
    contrast_ratio: float            # 对比度比率
    histogram_peaks: List[int]       # 直方图峰值位置
    dominant_brightness_range: Tuple[float, float]  # 主导亮度范围
    temporal_stability: float       # 时间稳定性 (0-1)
    
    # 环境分析
    light_source_type: str           # 光源类型推测 (natural/artificial/mixed)
    uniformity_score: float         # 光线均匀性评分 (0-1)
    flicker_detected: bool           # 是否检测到闪烁
    recommended_adaptation_speed: float  # 推荐适应速度 (秒)
    
    # 元数据
    analysis_method: str = "advanced_v2"  # 分析方法版本
    frame_quality: float = 1.0       # 帧质量评分

@dataclass
class EnvironmentalAdaptationProfile:
    """环境自适应配置文件"""
    # 基础适应参数
    min_led_factor: float = 0.1      # 最小LED因子
    max_led_factor: float = 3.0      # 最大LED因子
    adaptation_sensitivity: float = 1.0  # 适应敏感度
    temporal_smoothing: float = 0.7  # 时间平滑因子
    
    # 高级适应策略
    contrast_compensation: bool = True  # 对比度补偿
    flicker_protection: bool = True     # 闪烁保护
    source_type_optimization: bool = True  # 光源类型优化
    uniformity_adjustment: bool = True  # 均匀性调整
    
    # 性能优化
    fast_adaptation_threshold: float = 0.3  # 快速适应阈值
    stability_requirement: float = 0.8      # 稳定性要求
    confidence_threshold: float = 0.6       # 置信度阈值

class AdvancedEnvironmentalAnalyzer:
    """高级环境光线分析器"""
    
    def __init__(self, history_size: int = 10):
        """
        初始化高级环境分析器
        
        Args:
            history_size: 历史数据缓存大小
        """
        self.logger = logging.getLogger(__name__)
        self.history_size = history_size
        
        # 历史数据缓存
        self.brightness_history = deque(maxlen=history_size)
        self.analysis_history = deque(maxlen=history_size)
        
        # 分析配置
        self.histogram_bins = 256
        self.flicker_detection_frames = 5
        self.flicker_threshold = 0.15  # 15%变化认为是闪烁
        
    def analyze_environmental_light(self, frame: np.ndarray) -> Optional[AdvancedEnvironmentalLightInfo]:
        """
        高级环境光线分析
        
        Args:
            frame: 输入图像帧
            
        Returns:
            Optional[AdvancedEnvironmentalLightInfo]: 分析结果
        """
        try:
            if frame is None or frame.size == 0:
                return None
            
            # 转换为灰度图
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame.copy()
            
            # 基础亮度分析
            mean_brightness = np.mean(gray) / 255.0
            brightness_std = np.std(gray) / 255.0
            
            # 直方图分析
            histogram = cv2.calcHist([gray], [0], None, [self.histogram_bins], [0, 256])
            histogram = histogram.flatten() / histogram.sum()  # 归一化
            
            # 查找峰值
            peaks = self._find_histogram_peaks(histogram)
            
            # 对比度分析
            contrast_ratio = self._calculate_contrast_ratio(gray)
            
            # 主导亮度范围
            dominant_range = self._find_dominant_brightness_range(histogram)
            
            # 光源类型推测
            light_source_type = self._estimate_light_source_type(histogram, mean_brightness, brightness_std)
            
            # 均匀性评分
            uniformity_score = self._calculate_uniformity_score(gray, brightness_std)
            
            # 时间稳定性分析
            temporal_stability = self._calculate_temporal_stability(mean_brightness)
            
            # 闪烁检测
            flicker_detected = self._detect_flicker(mean_brightness)
            
            # 亮度分类
            brightness_category = self._classify_brightness(mean_brightness, histogram)
            
            # 计算检测置信度
            detection_confidence = self._calculate_detection_confidence(
                brightness_std, temporal_stability, len(self.brightness_history)
            )
            
            # 计算LED调节因子（高级算法）
            led_factor = self._calculate_advanced_led_factor(
                mean_brightness, brightness_category, contrast_ratio, 
                light_source_type, uniformity_score, flicker_detected
            )
            
            # 推荐适应速度
            adaptation_speed = self._calculate_adaptation_speed(
                temporal_stability, flicker_detected, brightness_category
            )
            
            # 帧质量评估
            frame_quality = self._assess_frame_quality(gray, brightness_std)
            
            # 创建分析结果
            analysis_result = AdvancedEnvironmentalLightInfo(
                brightness_level=mean_brightness,
                brightness_category=brightness_category,
                suggested_led_factor=led_factor,
                timestamp=time.time(),
                detection_confidence=detection_confidence,
                brightness_std=brightness_std,
                contrast_ratio=contrast_ratio,
                histogram_peaks=peaks,
                dominant_brightness_range=dominant_range,
                temporal_stability=temporal_stability,
                light_source_type=light_source_type,
                uniformity_score=uniformity_score,
                flicker_detected=flicker_detected,
                recommended_adaptation_speed=adaptation_speed,
                frame_quality=frame_quality
            )
            
            # 更新历史数据
            self.brightness_history.append(mean_brightness)
            self.analysis_history.append(analysis_result)
            
            return analysis_result
            
        except Exception as e:
            self.logger.error(f"高级环境光线分析失败: {e}")
            return None
    
    def _find_histogram_peaks(self, histogram: np.ndarray) -> List[int]:
        """查找直方图峰值"""
        peaks = []
        threshold = np.max(histogram) * 0.1  # 10%阈值
        
        for i in range(1, len(histogram) - 1):
            if (histogram[i] > histogram[i-1] and 
                histogram[i] > histogram[i+1] and 
                histogram[i] > threshold):
                peaks.append(i)
        
        return peaks[:5]  # 最多返回5个峰值
    
    def _calculate_contrast_ratio(self, gray: np.ndarray) -> float:
        """计算对比度比率"""
        min_val = np.min(gray)
        max_val = np.max(gray)
        
        if min_val == 0:
            return float('inf') if max_val > 0 else 1.0
        
        return max_val / min_val
    
    def _find_dominant_brightness_range(self, histogram: np.ndarray) -> Tuple[float, float]:
        """查找主导亮度范围（包含80%像素的最小范围）"""
        cumsum = np.cumsum(histogram)
        
        # 查找10%和90%位置
        low_idx = np.searchsorted(cumsum, 0.1)
        high_idx = np.searchsorted(cumsum, 0.9)
        
        return (low_idx / 255.0, high_idx / 255.0)
    
    def _estimate_light_source_type(self, histogram: np.ndarray, mean_brightness: float, brightness_std: float) -> str:
        """估计光源类型"""
        # 人工光源通常有明显的峰值，自然光更加平滑
        peaks = self._find_histogram_peaks(histogram)
        num_peaks = len(peaks)
        
        # 特征分析
        if num_peaks <= 1 and brightness_std > 0.3:
            return "natural"  # 自然光：单峰且变化较大
        elif num_peaks >= 3:
            return "mixed"    # 混合光源：多峰
        elif mean_brightness > 0.7 and brightness_std < 0.2:
            return "artificial_bright"  # 人工强光：高亮度低变化
        elif mean_brightness < 0.3 and brightness_std < 0.2:
            return "artificial_dim"     # 人工弱光：低亮度低变化
        else:
            return "artificial"  # 一般人工光源
    
    def _calculate_uniformity_score(self, gray: np.ndarray, brightness_std: float) -> float:
        """计算光线均匀性评分"""
        # 使用标准差和局部变化来评估均匀性
        height, width = gray.shape
        
        # 计算局部方差
        kernel_size = min(height, width) // 10
        if kernel_size < 3:
            kernel_size = 3
        
        # 高斯滤波后计算局部标准差
        blurred = cv2.GaussianBlur(gray.astype('float32'), (kernel_size, kernel_size), 0)
        local_variance = np.var(blurred) / (255.0 ** 2)
        
        # 结合全局和局部变化
        global_uniformity = 1.0 - min(1.0, brightness_std * 2)
        local_uniformity = 1.0 - min(1.0, local_variance * 5)
        
        return (global_uniformity + local_uniformity) / 2.0
    
    def _calculate_temporal_stability(self, current_brightness: float) -> float:
        """计算时间稳定性"""
        if len(self.brightness_history) < 3:
            return 0.5  # 历史数据不足
        
        # 计算亮度变化的标准差
        recent_history = list(self.brightness_history)[-5:]  # 最近5次
        if len(recent_history) < 2:
            return 0.5
        
        brightness_variation = statistics.stdev(recent_history)
        stability = 1.0 - min(1.0, brightness_variation * 10)  # 10倍敏感度
        
        return max(0.0, min(1.0, stability))
    
    def _detect_flicker(self, current_brightness: float) -> bool:
        """检测闪烁"""
        if len(self.brightness_history) < self.flicker_detection_frames:
            return False
        
        # 检查最近几帧的亮度变化
        recent_history = list(self.brightness_history)[-(self.flicker_detection_frames-1):]
        recent_history.append(current_brightness)
        
        # 计算相邻帧间的变化率
        changes = []
        for i in range(1, len(recent_history)):
            if recent_history[i-1] > 0:
                change_rate = abs(recent_history[i] - recent_history[i-1]) / recent_history[i-1]
                changes.append(change_rate)
        
        if not changes:
            return False
        
        # 如果有多个大变化，认为是闪烁
        large_changes = [c for c in changes if c > self.flicker_threshold]
        return len(large_changes) >= 2
    
    def _classify_brightness(self, mean_brightness: float, histogram: np.ndarray) -> EnvironmentalLightCategory:
        """亮度分类（高级算法）"""
        # 结合均值和直方图分布进行分类
        dominant_range = self._find_dominant_brightness_range(histogram)
        
        if mean_brightness < 0.05:
            return EnvironmentalLightCategory.VERY_DARK
        elif mean_brightness < 0.2:
            return EnvironmentalLightCategory.DARK
        elif mean_brightness < 0.4:
            return EnvironmentalLightCategory.DIM
        elif mean_brightness < 0.7:
            return EnvironmentalLightCategory.NORMAL
        elif mean_brightness < 0.85:
            return EnvironmentalLightCategory.BRIGHT
        else:
            return EnvironmentalLightCategory.VERY_BRIGHT
    
    def _calculate_detection_confidence(self, brightness_std: float, temporal_stability: float, history_length: int) -> float:
        """计算检测置信度"""
        # 基于多个因素计算置信度
        std_confidence = min(1.0, brightness_std * 3)  # 标准差越大置信度越高
        stability_confidence = temporal_stability
        history_confidence = min(1.0, history_length / 5.0)  # 历史数据越多置信度越高
        
        return (std_confidence + stability_confidence + history_confidence) / 3.0
    
    def _calculate_advanced_led_factor(self, mean_brightness: float, category: EnvironmentalLightCategory,
                                     contrast_ratio: float, light_source_type: str, 
                                     uniformity_score: float, flicker_detected: bool) -> float:
        """计算高级LED调节因子"""
        # 基础因子（根据亮度分类）
        base_factors = {
            EnvironmentalLightCategory.VERY_DARK: 2.5,
            EnvironmentalLightCategory.DARK: 2.0,
            EnvironmentalLightCategory.DIM: 1.5,
            EnvironmentalLightCategory.NORMAL: 1.0,
            EnvironmentalLightCategory.BRIGHT: 0.7,
            EnvironmentalLightCategory.VERY_BRIGHT: 0.4
        }
        
        base_factor = base_factors.get(category, 1.0)
        
        # 对比度调整
        if contrast_ratio > 50:  # 高对比度环境
            contrast_adjustment = 1.2
        elif contrast_ratio < 5:  # 低对比度环境
            contrast_adjustment = 0.8
        else:
            contrast_adjustment = 1.0
        
        # 光源类型调整
        source_adjustments = {
            "natural": 1.0,
            "artificial": 0.9,      # 人工光源下稍微降低
            "artificial_bright": 0.8,
            "artificial_dim": 1.1,
            "mixed": 1.05
        }
        source_adjustment = source_adjustments.get(light_source_type, 1.0)
        
        # 均匀性调整
        uniformity_adjustment = 1.0 + (1.0 - uniformity_score) * 0.2  # 不均匀时稍微提高
        
        # 闪烁保护
        flicker_adjustment = 0.8 if flicker_detected else 1.0
        
        # 综合计算
        final_factor = base_factor * contrast_adjustment * source_adjustment * uniformity_adjustment * flicker_adjustment
        
        return max(0.1, min(3.0, final_factor))
    
    def _calculate_adaptation_speed(self, temporal_stability: float, flicker_detected: bool, 
                                  category: EnvironmentalLightCategory) -> float:
        """计算推荐适应速度"""
        # 基础速度
        if flicker_detected:
            base_speed = 0.5  # 闪烁时快速适应
        elif temporal_stability > 0.8:
            base_speed = 3.0  # 稳定时慢速适应
        else:
            base_speed = 1.5  # 中等速度
        
        # 根据亮度分类调整
        if category in [EnvironmentalLightCategory.VERY_DARK, EnvironmentalLightCategory.VERY_BRIGHT]:
            speed_adjustment = 0.7  # 极端条件下更谨慎
        else:
            speed_adjustment = 1.0
        
        return base_speed * speed_adjustment
    
    def _assess_frame_quality(self, gray: np.ndarray, brightness_std: float) -> float:
        """评估帧质量"""
        # 基于清晰度和信息量评估
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        clarity_score = min(1.0, laplacian_var / 1000.0)  # 归一化清晰度
        
        # 信息量评分（基于标准差）
        information_score = min(1.0, brightness_std * 5)
        
        return (clarity_score + information_score) / 2.0

class UnifiedLEDController:
    """
    Claudia统一LED控制器 - 深化环境自适应版本
    
    集成高级环境光线分析和智能自适应功能
    """
    
    def __init__(self, 
                 preferred_method: LEDControlMethod = LEDControlMethod.VUI_CLIENT,
                 enable_environmental_adaptation: bool = True,
                 camera_device_id: int = 0,
                 adaptation_profile: Optional[EnvironmentalAdaptationProfile] = None):
        """
        初始化统一LED控制器
        
        Args:
            preferred_method: 首选控制方法
            enable_environmental_adaptation: 是否启用环境自适应
            camera_device_id: 前置摄像头设备ID
            adaptation_profile: 环境自适应配置文件
        """
        self.logger = logging.getLogger(__name__)
        self.preferred_method = preferred_method
        self.enable_environmental_adaptation = enable_environmental_adaptation
        self.camera_device_id = camera_device_id
        self.adaptation_profile = adaptation_profile or EnvironmentalAdaptationProfile()
        
        # 核心组件
        self.state_machine = None
        self.vui_renderer = None 
        self.lowcmd_controller = None
        self.is_initialized = False
        
        # 🎯 Phase 1.3: 系统状态监控器集成
        self.system_state_monitor = None
        self.system_monitoring_active = False
        self.current_system_state = None
        self.last_led_control_decision = None
        
        # 高级环境分析
        self.camera = None
        self.environment_analyzer = AdvancedEnvironmentalAnalyzer(history_size=15)
        self.environment_monitor_thread = None
        self.environment_monitoring_active = False
        self.current_environmental_info = None
        self.environment_update_interval = 2.0  # 2秒更新一次（提高响应性）
        
        # 自适应状态
        self.adaptive_led_factor = 1.0
        self.adaptive_factor_history = deque(maxlen=10)
        self.last_adaptation_time = 0
        self.adaptation_smoothing_active = True
        
        # 控制策略
        self.active_control_method = None
        self.fallback_available = False
        self.control_lock = threading.Lock()
        
        # 性能和状态
        self.control_attempts = 0
        self.control_successes = 0
        self.method_performance = {
            LEDControlMethod.VUI_CLIENT: {'attempts': 0, 'successes': 0},
            LEDControlMethod.LOW_CMD: {'attempts': 0, 'successes': 0}
        }
        
        # 高级监控指标
        self.adaptation_statistics = {
            'total_adaptations': 0,
            'flicker_detections': 0,
            'light_source_changes': 0,
            'rapid_adaptations': 0,
            'average_confidence': 0.0
        }
        
        self.logger.info("统一LED控制器初始化完成 (Enhanced Environmental Adaptation v0.3.0)")
    
    def initialize(self) -> bool:
        """
        初始化统一LED控制器
        
        Returns:
            bool: 初始化是否成功
        """
        if not LED_COMPONENTS_AVAILABLE:
            self.logger.error("LED组件不可用")
            return False
            
        try:
            self.logger.info("初始化统一LED控制器...")
            
            # 初始化LED状态机
            self.state_machine = create_led_state_machine()
            if not self.state_machine.initialize():
                self.logger.error("LED状态机初始化失败")
                return False
            
            # 初始化控制方法
            success = self._initialize_control_methods()
            if not success:
                self.logger.error("LED控制方法初始化失败")
                return False
            
            # 初始化环境监控
            if self.enable_environmental_adaptation:
                self._initialize_environmental_monitoring()
            
            # 🎯 Phase 1.3: 初始化系统状态监控器
            if SYSTEM_MONITOR_AVAILABLE:
                self._initialize_system_state_monitoring()
            
            self.is_initialized = True
            self.logger.info(f"✅ 统一LED控制器初始化成功 - 活跃方法: {self.active_control_method.value}")
            return True
            
        except Exception as e:
            self.logger.error(f"统一LED控制器初始化失败: {e}")
            return False
    
    def _initialize_control_methods(self) -> bool:
        """
        初始化LED控制方法
        
        Returns:
            bool: 初始化是否成功
        """
        vui_success = False
        lowcmd_success = False
        
        # 尝试初始化VUI控制
        try:
            self.vui_renderer = create_led_mode_renderer()
            vui_success = self.vui_renderer.initialize_vui()
            if vui_success:
                self.logger.info("✅ VUI LED控制初始化成功")
            else:
                self.logger.warning("❌ VUI LED控制初始化失败")
        except Exception as e:
            self.logger.warning(f"VUI LED控制初始化异常: {e}")
        
        # 尝试初始化LowCmd控制
        try:
            self.lowcmd_controller = create_led_controller()
            lowcmd_success = self.lowcmd_controller.initialize()
            if lowcmd_success:
                self.logger.info("✅ LowCmd LED控制初始化成功")
            else:
                self.logger.warning("❌ LowCmd LED控制初始化失败")
        except Exception as e:
            self.logger.warning(f"LowCmd LED控制初始化异常: {e}")
        
        # 确定活跃控制方法
        if self.preferred_method == LEDControlMethod.VUI_CLIENT and vui_success:
            self.active_control_method = LEDControlMethod.VUI_CLIENT
            self.fallback_available = lowcmd_success
        elif self.preferred_method == LEDControlMethod.LOW_CMD and lowcmd_success:
            self.active_control_method = LEDControlMethod.LOW_CMD  
            self.fallback_available = vui_success
        elif self.preferred_method == LEDControlMethod.AUTO_SELECT:
            if vui_success:
                self.active_control_method = LEDControlMethod.VUI_CLIENT
                self.fallback_available = lowcmd_success
            elif lowcmd_success:
                self.active_control_method = LEDControlMethod.LOW_CMD
                self.fallback_available = False
            else:
                self.logger.error("没有可用的LED控制方法")
                return False
        else:
            # 首选方法不可用，尝试备用方法
            if vui_success:
                self.active_control_method = LEDControlMethod.VUI_CLIENT
                self.fallback_available = lowcmd_success
            elif lowcmd_success:
                self.active_control_method = LEDControlMethod.LOW_CMD
                self.fallback_available = False
            else:
                self.logger.error("没有可用的LED控制方法")
                return False
        
        if self.fallback_available:
            self.logger.info(f"备用控制方法可用: {self._get_fallback_method().value}")
        
        return True
    
    def _get_fallback_method(self) -> Optional[LEDControlMethod]:
        """获取备用控制方法"""
        if not self.fallback_available:
            return None
        
        if self.active_control_method == LEDControlMethod.VUI_CLIENT:
            return LEDControlMethod.LOW_CMD
        else:
            return LEDControlMethod.VUI_CLIENT
    
    def _initialize_environmental_monitoring(self) -> bool:
        """
        初始化环境光线监控
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            self.logger.info("初始化环境光线监控...")
            
            # 尝试打开前置摄像头
            self.camera = cv2.VideoCapture(self.camera_device_id)
            if not self.camera.isOpened():
                self.logger.warning(f"无法打开摄像头 {self.camera_device_id}")
                return False
            
            # 设置摄像头参数
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            # 启动监控线程
            self.environment_monitoring_active = True
            self.environment_monitor_thread = threading.Thread(
                target=self._environment_monitoring_worker,
                daemon=True
            )
            self.environment_monitor_thread.start()
            
            self.logger.info("✅ 环境光线监控初始化成功")
            return True
            
        except Exception as e:
            self.logger.warning(f"环境光线监控初始化失败: {e}")
            return False
    
    def _initialize_system_state_monitoring(self) -> bool:
        """
        🎯 Phase 1.3: 初始化系统状态监控器
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            self.logger.info("🔍 初始化系统状态监控器...")
            
            # 创建系统状态监控器
            self.system_state_monitor = create_system_state_monitor(
                node_name="claudia_unified_led_system_monitor",
                history_size=50,
                update_rate=10.0
            )
            
            # 注册状态变化回调
            self.system_state_monitor.register_state_change_callback(
                self._on_system_state_change
            )
            
            # 注册关键事件回调
            self.system_state_monitor.register_critical_event_callback(
                self._on_system_critical_event
            )
            
            # 初始化监控器
            if self.system_state_monitor.initialize():
                # 启动监控
                if self.system_state_monitor.start_monitoring():
                    self.system_monitoring_active = True
                    self.logger.info("✅ 系统状态监控器初始化成功")
                    return True
                else:
                    self.logger.warning("❌ 系统状态监控器启动失败")
                    return False
            else:
                self.logger.warning("❌ 系统状态监控器初始化失败")
                return False
            
        except Exception as e:
            self.logger.warning(f"系统状态监控器初始化失败: {e}")
            return False
    
    def _on_system_state_change(self, previous_state: SystemState, new_state_info: SystemStateInfo) -> None:
        """
        🎯 Phase 1.3 & 🧠 Phase 2: 系统状态变化回调
        
        Args:
            previous_state: 前一个系统状态
            new_state_info: 新的系统状态信息
        """
        try:
            self.current_system_state = new_state_info
            
            self.logger.info(f"🔄 系统状态变化: {previous_state.name} → {new_state_info.state.name} "
                           f"(优先级: {new_state_info.priority.name})")
            
            # 🧠 Phase 2: 更新LED状态机的系统状态和动态优先级
            if self.state_machine and hasattr(self.state_machine, 'update_system_state'):
                self.state_machine.update_system_state(new_state_info)
            
            # 根据系统状态自动调整LED显示
            if new_state_info.state in [SystemState.LOW_BATTERY, SystemState.ERROR, SystemState.EMERGENCY]:
                # 高优先级系统状态 - 强制显示相应LED模式
                self._handle_high_priority_system_state(new_state_info)
            elif new_state_info.state == SystemState.CALIBRATING:
                # 校准状态 - 显示校准指示
                self._handle_calibration_state(new_state_info)
            
            # 获取并记录动态优先级统计信息
            if (self.state_machine and 
                hasattr(self.state_machine, 'get_dynamic_priority_statistics')):
                stats = self.state_machine.get_dynamic_priority_statistics()
                if stats:
                    self.logger.debug(f"🧠 动态优先级统计: {stats}")
                
        except Exception as e:
            self.logger.error(f"系统状态变化回调失败: {e}")
    
    def _on_system_critical_event(self, event_type: str, event_message: str, state_info: SystemStateInfo) -> None:
        """
        🎯 Phase 1.3 & 🧠 Phase 2: 系统关键事件回调
        
        Args:
            event_type: 事件类型
            event_message: 事件消息
            state_info: 系统状态信息
        """
        try:
            self.logger.warning(f"🚨 关键系统事件: {event_type} - {event_message}")
            
            # 🧠 Phase 2: 确保LED状态机了解当前系统状态
            if self.state_machine and hasattr(self.state_machine, 'update_system_state'):
                self.state_machine.update_system_state(state_info)
            
            # 🧠 Phase 2: 检查LED控制决策
            led_control_decision = None
            if (self.state_machine and 
                hasattr(self.state_machine, 'get_led_control_decision')):
                
                # 根据事件类型确定目标LED模式
                target_mode = ClaudiaLEDMode.ERROR_STATE
                target_priority = 9
                
                if event_type == "critical_battery":
                    target_mode = ClaudiaLEDMode.ERROR_STATE
                    target_priority = 10  # 最高优先级
                elif event_type == "low_battery":
                    target_mode = ClaudiaLEDMode.LOW_BATTERY
                    target_priority = 8
                elif event_type in ["high_temperature", "system_errors"]:
                    target_mode = ClaudiaLEDMode.ERROR_STATE
                    target_priority = 9
                
                led_control_decision = self.state_machine.get_led_control_decision(target_mode, target_priority)
                
                if led_control_decision:
                    self.logger.info(f"🧠 LED控制决策: {led_control_decision.recommended_action} - {led_control_decision.reason}")
            
            # 根据事件类型执行相应动作（使用智能决策）
            if event_type == "critical_battery":
                # 极低电量 - 强制显示红色警告
                self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_critical_battery", force=True)
            elif event_type == "low_battery":
                # 低电量 - 显示黄色警告（使用动态优先级）
                priority = (led_control_decision.required_priority.value 
                           if led_control_decision else 8)
                self.set_mode(ClaudiaLEDMode.LOW_BATTERY, "system_low_battery", priority_override=priority)
            elif event_type == "high_temperature":
                # 高温 - 显示温度警告（使用动态优先级）
                priority = (led_control_decision.required_priority.value 
                           if led_control_decision else 9)
                self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_high_temperature", priority_override=priority)
            elif event_type == "system_errors":
                # 系统错误 - 显示错误状态（使用动态优先级）
                priority = (led_control_decision.required_priority.value 
                           if led_control_decision else 10)
                self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_error", priority_override=priority)
                
        except Exception as e:
            self.logger.error(f"系统关键事件回调失败: {e}")
    
    def _handle_high_priority_system_state(self, state_info: SystemStateInfo) -> None:
        """
        🧠 Phase 2: 处理高优先级系统状态（集成动态优先级）
        """
        try:
            # 获取LED控制决策
            led_decision = None
            if (self.state_machine and 
                hasattr(self.state_machine, 'get_led_control_decision')):
                
                if state_info.state == SystemState.LOW_BATTERY:
                    # 低电量状态
                    battery_pct = state_info.battery_level * 100
                    if battery_pct <= 5:
                        # 极低电量 - 红色闪烁
                        led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.ERROR_STATE, 10)
                        self.set_mode(ClaudiaLEDMode.ERROR_STATE, "critical_battery", force=True)
                    else:
                        # 低电量 - 黄色提示
                        led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.LOW_BATTERY, 8)
                        priority = led_decision.required_priority.value if led_decision else 8
                        self.set_mode(ClaudiaLEDMode.LOW_BATTERY, "low_battery", priority_override=priority)
                        
                elif state_info.state == SystemState.ERROR:
                    # 错误状态 - 红色闪烁
                    led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.ERROR_STATE, 9)
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_error", force=True)
                    
                elif state_info.state == SystemState.EMERGENCY:
                    # 紧急状态 - 快速红色闪烁
                    led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.ERROR_STATE, 10)
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "emergency", force=True)
            
            else:
                # 备用逻辑（没有动态优先级管理器时）
                if state_info.state == SystemState.LOW_BATTERY:
                    battery_pct = state_info.battery_level * 100
                    if battery_pct <= 5:
                        self.set_mode(ClaudiaLEDMode.ERROR_STATE, "critical_battery", force=True)
                    else:
                        self.set_mode(ClaudiaLEDMode.LOW_BATTERY, "low_battery", priority_override=8)
                elif state_info.state == SystemState.ERROR:
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_error", force=True)
                elif state_info.state == SystemState.EMERGENCY:
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "emergency", force=True)
            
            if led_decision:
                self.logger.debug(f"🧠 高优先级状态LED决策: {led_decision.recommended_action}")
                
        except Exception as e:
            self.logger.error(f"高优先级系统状态处理失败: {e}")
    
    def _handle_calibration_state(self, state_info: SystemStateInfo) -> None:
        """
        🧠 Phase 2: 处理校准状态（集成动态优先级）
        """
        try:
            # 获取校准LED控制决策
            led_decision = None
            if (self.state_machine and 
                hasattr(self.state_machine, 'get_led_control_decision')):
                led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.SYSTEM_CALIBRATION, 7)
                
                if led_decision and led_decision.allow_custom_control:
                    priority = led_decision.required_priority.value
                    self.set_mode(ClaudiaLEDMode.SYSTEM_CALIBRATION, "system_calibrating", priority_override=priority)
                    self.logger.debug(f"🧠 校准状态LED决策: {led_decision.recommended_action}")
                else:
                    self.logger.warning(f"🛡️ 校准LED被系统阻止: {led_decision.reason if led_decision else '未知原因'}")
            else:
                # 备用逻辑
                self.set_mode(ClaudiaLEDMode.SYSTEM_CALIBRATION, "system_calibrating", priority_override=7)
            
        except Exception as e:
            self.logger.error(f"校准状态处理失败: {e}")
    
    def _environment_monitoring_worker(self) -> None:
        """高级环境光线监控工作线程"""
        last_light_source_type = None
        confidence_history = deque(maxlen=5)
        
        while self.environment_monitoring_active:
            try:
                # 检测环境光线
                env_info = self._detect_environmental_light()
                if env_info:
                    self.current_environmental_info = env_info
                    confidence_history.append(env_info.detection_confidence)
                    
                    # 应用智能自适应策略
                    self._apply_intelligent_adaptation(env_info)
                    
                    # 检测光源类型变化
                    if (last_light_source_type is not None and 
                        last_light_source_type != env_info.light_source_type):
                        self.adaptation_statistics['light_source_changes'] += 1
                        self.logger.info(f"检测到光源类型变化: {last_light_source_type} → {env_info.light_source_type}")
                    
                    last_light_source_type = env_info.light_source_type
                    
                    # 更新统计信息
                    self.adaptation_statistics['total_adaptations'] += 1
                    if env_info.flicker_detected:
                        self.adaptation_statistics['flicker_detections'] += 1
                    
                    if confidence_history:
                        self.adaptation_statistics['average_confidence'] = statistics.mean(confidence_history)
                    
                    # 动态调整更新间隔
                    self._adjust_monitoring_interval(env_info)
                
                # 等待下次更新
                time.sleep(self.environment_update_interval)
                
            except Exception as e:
                self.logger.error(f"环境光线监控错误: {e}")
                time.sleep(self.environment_update_interval)
    
    def _apply_intelligent_adaptation(self, env_info: AdvancedEnvironmentalLightInfo) -> None:
        """
        应用智能自适应策略
        
        Args:
            env_info: 环境光线信息
        """
        try:
            current_time = time.time()
            
            # 检查置信度阈值
            if env_info.detection_confidence < self.adaptation_profile.confidence_threshold:
                self.logger.debug(f"检测置信度过低，跳过自适应: {env_info.detection_confidence:.2f}")
                return
            
            # 应用时间平滑
            if self.adaptation_smoothing_active:
                new_factor = self._apply_temporal_smoothing(env_info.suggested_led_factor)
            else:
                new_factor = env_info.suggested_led_factor
            
            # 检查是否需要快速适应
            rapid_adaptation_needed = (
                env_info.flicker_detected or
                env_info.temporal_stability < self.adaptation_profile.fast_adaptation_threshold or
                abs(new_factor - self.adaptive_led_factor) > 0.5
            )
            
            if rapid_adaptation_needed:
                self.adaptation_statistics['rapid_adaptations'] += 1
                self.logger.debug("启用快速适应模式")
            
            # 应用新的调节因子
            if abs(new_factor - self.adaptive_led_factor) > 0.05:  # 5%变化阈值
                self.adaptive_led_factor = new_factor
                self.adaptive_factor_history.append(new_factor)
                self.last_adaptation_time = current_time
                
                # 更新LED渲染器
                if self.vui_renderer:
                    self.vui_renderer.set_environmental_brightness_factor(new_factor)
                
                self.logger.debug(f"自适应LED因子更新: {new_factor:.2f} (置信度={env_info.detection_confidence:.2f})")
            
        except Exception as e:
            self.logger.error(f"智能自适应应用失败: {e}")
    
    def _apply_temporal_smoothing(self, new_factor: float) -> float:
        """
        应用时间平滑
        
        Args:
            new_factor: 新的调节因子
            
        Returns:
            float: 平滑后的调节因子
        """
        if not self.adaptive_factor_history:
            return new_factor
        
        # 计算历史平均值
        recent_factors = list(self.adaptive_factor_history)[-3:]  # 最近3次
        if recent_factors:
            historical_average = statistics.mean(recent_factors)
            
            # 应用平滑因子
            smoothing = self.adaptation_profile.temporal_smoothing
            smoothed_factor = smoothing * historical_average + (1.0 - smoothing) * new_factor
            
            return max(self.adaptation_profile.min_led_factor, 
                      min(self.adaptation_profile.max_led_factor, smoothed_factor))
        
        return new_factor
    
    def _adjust_monitoring_interval(self, env_info: AdvancedEnvironmentalLightInfo) -> None:
        """
        动态调整监控间隔
        
        Args:
            env_info: 环境光线信息
        """
        # 根据环境稳定性和推荐适应速度调整间隔
        base_interval = 2.0
        
        if env_info.flicker_detected:
            # 闪烁时提高监控频率
            self.environment_update_interval = max(0.5, base_interval * 0.25)
        elif env_info.temporal_stability > 0.9:
            # 环境稳定时降低监控频率
            self.environment_update_interval = min(5.0, base_interval * 2.0)
        else:
            # 使用推荐的适应速度
            speed_factor = env_info.recommended_adaptation_speed / 2.0
            self.environment_update_interval = max(0.5, min(5.0, speed_factor))
    
    def _detect_environmental_light(self) -> Optional[AdvancedEnvironmentalLightInfo]:
        """
        检测环境光线条件
        
        Returns:
            Optional[AdvancedEnvironmentalLightInfo]: 环境光线信息
        """
        if not self.camera or not self.camera.isOpened():
            return None
            
        try:
            # 捕获帧
            ret, frame = self.camera.read()
            if not ret or frame is None:
                return None
            
            # 分析环境光线
            env_info = self.environment_analyzer.analyze_environmental_light(frame)
            if env_info:
                self.current_environmental_info = env_info
                
                # 更新LED渲染器的环境调节因子
                if self.vui_renderer:
                    self.vui_renderer.set_environmental_brightness_factor(env_info.suggested_led_factor)
                
                self.logger.debug(f"环境光线分析成功: {env_info.brightness_category} (因子={env_info.suggested_led_factor:.2f})")
                
                return env_info
            
        except Exception as e:
            self.logger.error(f"环境光线检测失败: {e}")
            return None
    
    def set_mode(self, 
                mode: ClaudiaLEDMode, 
                source: str = "unified_controller",
                duration: Optional[float] = None,
                priority_override: Optional[int] = None,
                force: bool = False) -> bool:
        """
        设置LED模式（主要接口）
        
        Args:
            mode: LED模式
            source: 来源标识
            duration: 可选的持续时间覆盖
            priority_override: 可选的优先级覆盖
            force: 是否强制执行（忽略优先级）
            
        Returns:
            bool: 设置是否成功
        """
        if not self.is_initialized:
            self.logger.error("统一LED控制器未初始化")
            return False
        
        self.control_attempts += 1
        
        try:
            with self.control_lock:
                if force:
                    # 强制模式
                    success = self.state_machine.force_state(mode, source)
                else:
                    # 正常模式
                    success = self.state_machine.request_state(
                        mode=mode,
                        source=source,
                        duration=duration,
                        priority_override=priority_override
                    )
                
                if success:
                    self.control_successes += 1
                    self._update_method_performance(self.active_control_method, True)
                    self.logger.info(f"LED模式设置成功: {mode.value} (方法={self.active_control_method.value})")
                else:
                    self._update_method_performance(self.active_control_method, False)
                    
                    # 尝试使用备用方法
                    if self.fallback_available:
                        self.logger.info("尝试使用备用控制方法...")
                        success = self._try_fallback_method(mode, source, duration, priority_override, force)
                        
                        if success:
                            self.logger.info(f"备用方法成功: {mode.value}")
                            self.control_successes += 1
                
                return success
                
        except Exception as e:
            self.logger.error(f"LED模式设置失败: {e}")
            self._update_method_performance(self.active_control_method, False)
            return False
    
    def _try_fallback_method(self, mode: ClaudiaLEDMode, source: str, duration: Optional[float], 
                           priority_override: Optional[int], force: bool) -> bool:
        """
        尝试使用备用控制方法
        
        Returns:
            bool: 是否成功
        """
        fallback_method = self._get_fallback_method()
        if not fallback_method:
            return False
            
        try:
            # 临时切换到备用方法
            original_method = self.active_control_method
            self.active_control_method = fallback_method
            
            # 使用备用方法执行控制
            if fallback_method == LEDControlMethod.VUI_CLIENT and self.vui_renderer:
                pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
                success = self.vui_renderer.render_mode(mode, duration)
            elif fallback_method == LEDControlMethod.LOW_CMD and self.lowcmd_controller:
                # 使用LowCmd方法（简化实现）
                pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
                r, g, b = pattern.color
                led_data = [r//21, g//21, b//21] * 4  # 简单映射到12字节
                success = self.lowcmd_controller.set_led_direct(led_data)
            else:
                success = False
            
            self._update_method_performance(fallback_method, success)
            
            if success:
                self.logger.info(f"备用方法执行成功，切换活跃方法: {original_method.value} → {fallback_method.value}")
                # 如果备用方法成功，考虑将其作为新的活跃方法
                self.fallback_available = True  # 原方法变成备用
            else:
                # 备用方法也失败，恢复原方法
                self.active_control_method = original_method
                
            return success
            
        except Exception as e:
            self.logger.error(f"备用方法执行失败: {e}")
            return False
    
    def _update_method_performance(self, method: LEDControlMethod, success: bool) -> None:
        """
        更新控制方法性能统计
        
        Args:
            method: 控制方法
            success: 是否成功
        """
        if method in self.method_performance:
            self.method_performance[method]['attempts'] += 1
            if success:
                self.method_performance[method]['successes'] += 1
    
    # Claudia专用快捷方法
    def wake_confirm(self, source: str = "claudia") -> bool:
        """🟢 唤醒确认：绿色双闪"""
        return self.set_mode(ClaudiaLEDMode.WAKE_CONFIRM, source)
    
    def processing_voice(self, source: str = "claudia") -> bool:
        """🔵 处理语音：蓝色常亮"""
        return self.set_mode(ClaudiaLEDMode.PROCESSING_VOICE, source)
    
    def executing_action(self, source: str = "claudia") -> bool:
        """🟠 执行动作：橙色常亮"""
        return self.set_mode(ClaudiaLEDMode.EXECUTING_ACTION, source)
    
    def action_complete(self, source: str = "claudia") -> bool:
        """⚪ 动作完成：白色短闪3次"""
        return self.set_mode(ClaudiaLEDMode.ACTION_COMPLETE, source)
    
    def error_state(self, source: str = "claudia") -> bool:
        """🔴 错误状态：红色三闪"""
        return self.set_mode(ClaudiaLEDMode.ERROR_STATE, source)
    
    def turn_off(self, source: str = "claudia") -> bool:
        """关闭LED"""
        return self.set_mode(ClaudiaLEDMode.OFF, source)
    
    # 状态查询方法
    def get_current_mode(self) -> Tuple[ClaudiaLEDMode, int]:
        """获取当前LED模式和优先级"""
        if self.state_machine:
            return self.state_machine.get_current_state()
        return ClaudiaLEDMode.OFF, 1
    
    def get_environmental_info(self) -> Optional[AdvancedEnvironmentalLightInfo]:
        """获取当前环境光线信息"""
        return self.current_environmental_info
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """获取高级性能指标"""
        overall_success_rate = self.control_successes / max(1, self.control_attempts)
        
        metrics = {
            'overall_success_rate': overall_success_rate,
            'total_attempts': self.control_attempts,
            'total_successes': self.control_successes,
            'active_method': self.active_control_method.value if self.active_control_method else None,
            'fallback_available': self.fallback_available,
            'environmental_adaptation': self.enable_environmental_adaptation,
            'method_performance': {},
            'adaptation_statistics': self.adaptation_statistics.copy(),
            'current_adaptive_factor': self.adaptive_led_factor,
            'adaptation_profile': {
                'min_led_factor': self.adaptation_profile.min_led_factor,
                'max_led_factor': self.adaptation_profile.max_led_factor,
                'confidence_threshold': self.adaptation_profile.confidence_threshold,
                'temporal_smoothing': self.adaptation_profile.temporal_smoothing
            }
        }
        
        # 各方法的性能统计
        for method, stats in self.method_performance.items():
            if stats['attempts'] > 0:
                success_rate = stats['successes'] / stats['attempts']
                metrics['method_performance'][method.value] = {
                    'success_rate': success_rate,
                    'attempts': stats['attempts'],
                    'successes': stats['successes']
                }
        
        # 环境分析器统计
        if hasattr(self.environment_analyzer, 'brightness_history'):
            history_length = len(self.environment_analyzer.brightness_history)
            if history_length > 0:
                recent_brightness = list(self.environment_analyzer.brightness_history)
                metrics['environmental_analysis'] = {
                    'history_length': history_length,
                    'current_brightness': recent_brightness[-1] if recent_brightness else 0,
                    'brightness_stability': statistics.stdev(recent_brightness) if len(recent_brightness) > 1 else 0,
                    'update_interval': self.environment_update_interval
                }
        
        # 状态机性能指标
        if self.state_machine:
            state_metrics = self.state_machine.get_performance_metrics()
            metrics.update(state_metrics)
            
            # 🧠 Phase 2: 添加动态优先级统计
            if hasattr(self.state_machine, 'get_dynamic_priority_statistics'):
                dynamic_stats = self.state_machine.get_dynamic_priority_statistics()
                if dynamic_stats:
                    metrics['dynamic_priority_stats'] = dynamic_stats
        
        return metrics
    
    # 🧠 Phase 2: 新增智能决策功能方法
    def get_current_system_state(self) -> Optional[SystemStateInfo]:
        """
        获取当前系统状态信息
        
        Returns:
            Optional[SystemStateInfo]: 当前系统状态（如果可用）
        """
        return self.current_system_state
    
    def get_led_control_decision(self, mode: ClaudiaLEDMode, priority: int) -> Optional['LEDControlDecision']:
        """
        🧠 Phase 2: 获取LED控制决策
        
        Args:
            mode: LED模式
            priority: 请求优先级
            
        Returns:
            Optional[LEDControlDecision]: 控制决策（如果可用）
        """
        if (self.state_machine and 
            hasattr(self.state_machine, 'get_led_control_decision')):
            return self.state_machine.get_led_control_decision(mode, priority)
        return None
    
    def set_auto_mode_switching(self, enabled: bool) -> None:
        """
        🧠 Phase 2: 设置自动模式切换
        
        Args:
            enabled: 是否启用自动模式切换
        """
        if (self.state_machine and 
            hasattr(self.state_machine, 'set_auto_mode_switching')):
            self.state_machine.set_auto_mode_switching(enabled)
            self.logger.info(f"自动模式切换已{'启用' if enabled else '禁用'}")
        else:
            self.logger.warning("LED状态机不支持自动模式切换功能")
    
    def get_dynamic_priority_statistics(self) -> Optional[Dict[str, Any]]:
        """
        🧠 Phase 2: 获取动态优先级统计信息
        
        Returns:
            Optional[Dict[str, Any]]: 统计信息（如果可用）
        """
        if (self.state_machine and 
            hasattr(self.state_machine, 'get_dynamic_priority_statistics')):
            return self.state_machine.get_dynamic_priority_statistics()
        return None
    
    def simulate_system_state_change(self, state_info: SystemStateInfo) -> None:
        """
        🧠 Phase 2: 模拟系统状态变化（用于测试）
        
        Args:
            state_info: 要模拟的系统状态信息
        """
        self.logger.info(f"🧪 模拟系统状态变化: {state_info.state.name}")
        
        # 直接调用状态变化回调
        previous_state = self.current_system_state.state if self.current_system_state else SystemState.UNKNOWN
        self._on_system_state_change(previous_state, state_info)
    
    def force_dynamic_priority_recalculation(self) -> bool:
        """
        🧠 Phase 2: 强制重新计算动态优先级
        
        Returns:
            bool: 是否成功
        """
        try:
            if (self.state_machine and 
                hasattr(self.state_machine, '_reevaluate_current_priority')):
                self.state_machine._reevaluate_current_priority()
                self.logger.info("🔄 强制重新计算动态优先级完成")
                return True
            else:
                self.logger.warning("LED状态机不支持动态优先级重新计算")
                return False
        except Exception as e:
            self.logger.error(f"强制动态优先级重新计算失败: {e}")
            return False
    
    # 高级环境自适应控制方法
    def set_adaptation_profile(self, profile: EnvironmentalAdaptationProfile) -> None:
        """
        设置环境自适应配置文件
        
        Args:
            profile: 新的适应配置文件
        """
        self.adaptation_profile = profile
        self.logger.info(f"环境自适应配置已更新: 置信度阈值={profile.confidence_threshold}, 平滑因子={profile.temporal_smoothing}")
    
    def enable_adaptation_smoothing(self, enable: bool) -> None:
        """
        启用或禁用自适应平滑
        
        Args:
            enable: 是否启用平滑
        """
        self.adaptation_smoothing_active = enable
        self.logger.info(f"自适应平滑: {'启用' if enable else '禁用'}")
    
    def get_adaptation_statistics(self) -> Dict[str, Any]:
        """获取详细的自适应统计信息"""
        stats = self.adaptation_statistics.copy()
        
        # 添加实时信息
        if self.current_environmental_info:
            env_info = self.current_environmental_info
            stats['current_environment'] = {
                'brightness_category': env_info.brightness_category.value,
                'suggested_led_factor': env_info.suggested_led_factor,
                'detection_confidence': env_info.detection_confidence,
                'light_source_type': env_info.light_source_type,
                'temporal_stability': env_info.temporal_stability,
                'flicker_detected': env_info.flicker_detected,
                'uniformity_score': env_info.uniformity_score,
                'contrast_ratio': env_info.contrast_ratio
            }
        
        # 历史因子统计
        if self.adaptive_factor_history:
            factor_history = list(self.adaptive_factor_history)
            stats['factor_statistics'] = {
                'current_factor': self.adaptive_led_factor,
                'average_factor': statistics.mean(factor_history),
                'factor_range': (min(factor_history), max(factor_history)),
                'factor_stability': statistics.stdev(factor_history) if len(factor_history) > 1 else 0
            }
        
        return stats
    
    def force_adaptation_update(self) -> bool:
        """
        强制执行一次环境检测和自适应更新
        
        Returns:
            bool: 更新是否成功
        """
        try:
            env_info = self._detect_environmental_light()
            if env_info:
                self._apply_intelligent_adaptation(env_info)
                self.logger.info(f"强制自适应更新完成: 因子={self.adaptive_led_factor:.2f}")
                return True
            else:
                self.logger.warning("强制自适应更新失败：无法获取环境信息")
                return False
        except Exception as e:
            self.logger.error(f"强制自适应更新异常: {e}")
            return False
    
    def reset_adaptation_statistics(self) -> None:
        """重置自适应统计信息"""
        self.adaptation_statistics = {
            'total_adaptations': 0,
            'flicker_detections': 0,
            'light_source_changes': 0,
            'rapid_adaptations': 0,
            'average_confidence': 0.0
        }
        self.adaptive_factor_history.clear()
        self.logger.info("自适应统计信息已重置")
    
    def emergency_stop(self) -> bool:
        """紧急停止所有LED活动"""
        self.logger.warning("统一LED控制器紧急停止")
        
        success = True
        
        # 停止状态机
        if self.state_machine:
            success &= self.state_machine.emergency_stop()
        
        # 停止VUI渲染器
        if self.vui_renderer:
            self.vui_renderer.stop_all_rendering()
        
        # 停止LowCmd控制器
        if self.lowcmd_controller:
            self.lowcmd_controller.set_led_direct([0] * 12)
        
        return success
    
    def cleanup(self) -> None:
        """清理资源"""
        self.logger.info("清理统一LED控制器资源...")
        
        try:
            # 停止环境监控
            self.environment_monitoring_active = False
            if self.environment_monitor_thread and self.environment_monitor_thread.is_alive():
                self.environment_monitor_thread.join(timeout=2.0)
            
            # 🎯 Phase 1.3: 停止系统状态监控器
            if self.system_state_monitor and self.system_monitoring_active:
                self.system_monitoring_active = False
                self.system_state_monitor.stop_monitoring()
                self.system_state_monitor.cleanup()
                self.logger.info("✅ 系统状态监控器已清理")
            
            # 关闭摄像头
            if self.camera:
                self.camera.release()
            
            # 清理LED组件
            if self.state_machine:
                self.state_machine.cleanup()
            
            if self.vui_renderer:
                self.vui_renderer.cleanup()
                
            if self.lowcmd_controller:
                self.lowcmd_controller.cleanup()
            
            self.is_initialized = False
            self.logger.info("✅ 统一LED控制器清理完成")
            
        except Exception as e:
            self.logger.error(f"统一LED控制器清理失败: {e}")


# 工厂函数
def create_unified_led_controller(
    preferred_method: LEDControlMethod = LEDControlMethod.VUI_CLIENT,
    enable_environmental_adaptation: bool = True,
    adaptation_profile: Optional[EnvironmentalAdaptationProfile] = None) -> UnifiedLEDController:
    """
    创建统一LED控制器实例（高级环境自适应版本）
    
    Args:
        preferred_method: 首选控制方法
        enable_environmental_adaptation: 是否启用环境自适应
        adaptation_profile: 环境自适应配置文件
        
    Returns:
        UnifiedLEDController: 控制器实例
    """
    return UnifiedLEDController(
        preferred_method=preferred_method,
        enable_environmental_adaptation=enable_environmental_adaptation,
        adaptation_profile=adaptation_profile
    )

def create_enhanced_led_controller(
    camera_device_id: int = 0,
    adaptation_sensitivity: float = 1.0,
    temporal_smoothing: float = 0.7,
    confidence_threshold: float = 0.6) -> UnifiedLEDController:
    """
    创建增强环境自适应LED控制器
    
    Args:
        camera_device_id: 摄像头设备ID
        adaptation_sensitivity: 适应敏感度
        temporal_smoothing: 时间平滑因子
        confidence_threshold: 置信度阈值
        
    Returns:
        UnifiedLEDController: 增强版控制器实例
    """
    enhanced_profile = EnvironmentalAdaptationProfile(
        adaptation_sensitivity=adaptation_sensitivity,
        temporal_smoothing=temporal_smoothing,
        confidence_threshold=confidence_threshold,
        contrast_compensation=True,
        flicker_protection=True,
        source_type_optimization=True,
        uniformity_adjustment=True
    )
    
    return UnifiedLEDController(
        preferred_method=LEDControlMethod.VUI_CLIENT,
        enable_environmental_adaptation=True,
        camera_device_id=camera_device_id,
        adaptation_profile=enhanced_profile
    )


if __name__ == "__main__":
    # 基础测试
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    print("🧪 统一LED控制器测试")
    print("=" * 50)
    
    try:
        # 创建控制器
        controller = create_unified_led_controller()
        
        if controller.initialize():
            print("✅ 统一LED控制器初始化成功")
            
            # 测试Claudia专用LED序列
            print("\n🎭 测试Claudia LED交互序列...")
            
            print("1. 🟢 用户唤醒Claudia")
            controller.wake_confirm("test_sequence")
            time.sleep(3)
            
            print("2. 🔵 Claudia开始处理语音")
            controller.processing_voice("test_sequence")
            time.sleep(2)
            
            print("3. 🟠 Claudia执行用户指令")
            controller.executing_action("test_sequence")
            time.sleep(3)
            
            print("4. ⚪ 指令执行完成")
            controller.action_complete("test_sequence")
            time.sleep(2)
            
            print("5. 🔴 模拟错误情况")
            controller.error_state("test_sequence")
            time.sleep(3)
            
            print("6. 关闭LED")
            controller.turn_off("test_sequence")
            
            # 显示性能指标
            metrics = controller.get_performance_metrics()
            print(f"\n📊 性能报告:")
            print(f"   整体成功率: {metrics['overall_success_rate']*100:.1f}%")
            print(f"   总操作次数: {metrics['total_attempts']}")
            print(f"   活跃控制方法: {metrics['active_method']}")
            print(f"   备用方法可用: {'✅' if metrics['fallback_available'] else '❌'}")
            print(f"   环境自适应: {'✅' if metrics['environmental_adaptation'] else '❌'}")
            
            # 环境信息
            env_info = controller.get_environmental_info()
            if env_info:
                print(f"\n🌅 环境光线信息:")
                print(f"   亮度类别: {env_info.brightness_category}")
                print(f"   亮度等级: {env_info.brightness_level:.2f}")
                print(f"   LED调节因子: {env_info.suggested_led_factor:.2f}")
                print(f"   检测置信度: {env_info.detection_confidence:.2f}")
                
        else:
            print("❌ 统一LED控制器初始化失败")
            
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
    finally:
        controller.cleanup() 