#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia LED模式定义模块
基于VUI客户端实现5种专用LED状态指示器

Author: Claudia AI System
Generated: 2025-06-30
Purpose: 子任务6.2 - LED模式定义与状态机实现
"""

import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any, Callable
from dataclasses import dataclass
from enum import Enum
import math

# 添加项目路径
sys.path.append('/home/m1ng/claudia/src')

# Unitree SDK2 VUI imports
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.vui.vui_client import VuiClient
    VUI_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ VUI客户端导入失败: {e}")
    VUI_AVAILABLE = False

class ClaudiaLEDMode(Enum):
    """
    Claudia专用LED模式枚举
    
    每种模式对应特定的交互状态，用于向用户传达机器人当前状态
    """
    # 系统状态模式
    OFF = "off"                          # 关闭状态
    
    # Claudia专用状态指示器
    WAKE_CONFIRM = "wake_confirm"        # 🟢 绿色双闪 (唤醒确认)
    PROCESSING_VOICE = "processing"      # 🔵 蓝色常亮 (处理语音)
    EXECUTING_ACTION = "executing"       # 🟠 橙色常亮 (执行动作)
    ACTION_COMPLETE = "action_complete"  # ⚪ 白色短闪3次 (动作完成)
    ERROR_STATE = "error"                # 🔴 红色三闪 (错误/无法理解)
    
    # 系统兼容性模式（避免干扰默认状态）
    SYSTEM_BOOT = "system_boot"          # 🟢 绿色常亮 (开机) - 保留兼容
    SYSTEM_CALIBRATION = "calibration"   # 🔵 蓝色闪烁 (校准) - 保留兼容
    LOW_BATTERY = "low_battery"          # 🟡 黄色闪烁 (低电量) - 保留兼容
    SEARCH_LIGHT = "search_light"        # ⚪ 白色常亮 (搜索灯) - 保留兼容

@dataclass
class LEDPattern:
    """LED模式参数"""
    color: Tuple[int, int, int]          # RGB颜色 (0-255)
    brightness: int                      # 亮度 (0-10, VUI标准)
    flash_count: int                     # 闪烁次数 (0=常亮)
    flash_interval: float                # 闪烁间隔 (秒)
    duration: float                      # 模式持续时间 (秒, 0=无限)
    priority: int                        # 优先级 (1-10, 10最高)

class ClaudiaLEDModeDefinitions:
    """
    Claudia LED模式定义类
    
    定义所有LED模式的视觉参数和行为逻辑
    """
    
    # Claudia专用LED模式定义
    PATTERNS = {
        ClaudiaLEDMode.OFF: LEDPattern(
            color=(0, 0, 0),
            brightness=0,
            flash_count=0,
            flash_interval=0.0,
            duration=0.0,
            priority=1
        ),
        
        # 🟢 绿色双闪 (唤醒确认)
        ClaudiaLEDMode.WAKE_CONFIRM: LEDPattern(
            color=(0, 255, 0),               # 鲜绿色
            brightness=8,                    # 较高亮度确保可见
            flash_count=2,                   # 双闪
            flash_interval=0.3,              # 300ms间隔
            duration=2.0,                    # 2秒后自动结束
            priority=7                       # 高优先级
        ),
        
        # 🔵 蓝色常亮 (处理语音)
        ClaudiaLEDMode.PROCESSING_VOICE: LEDPattern(
            color=(0, 100, 255),             # 柔和蓝色
            brightness=6,                    # 中等亮度避免刺眼
            flash_count=0,                   # 常亮
            flash_interval=0.0,
            duration=0.0,                    # 无限持续直到状态改变
            priority=6                       # 中高优先级
        ),
        
        # 🟠 橙色常亮 (执行动作)
        ClaudiaLEDMode.EXECUTING_ACTION: LEDPattern(
            color=(255, 165, 0),             # 标准橙色
            brightness=7,                    # 较高亮度表示活跃状态
            flash_count=0,                   # 常亮
            flash_interval=0.0,
            duration=0.0,                    # 无限持续直到动作完成
            priority=8                       # 高优先级
        ),
        
        # ⚪ 白色短闪3次 (动作完成)
        ClaudiaLEDMode.ACTION_COMPLETE: LEDPattern(
            color=(255, 255, 255),           # 纯白色
            brightness=9,                    # 高亮度确保注意
            flash_count=3,                   # 三闪
            flash_interval=0.2,              # 200ms快速闪烁
            duration=1.5,                    # 1.5秒完成
            priority=9                       # 很高优先级
        ),
        
        # 🔴 红色三闪 (错误/无法理解)
        ClaudiaLEDMode.ERROR_STATE: LEDPattern(
            color=(255, 0, 0),               # 鲜红色
            brightness=10,                   # 最高亮度警示
            flash_count=3,                   # 三闪
            flash_interval=0.4,              # 400ms较慢表示错误
            duration=2.5,                    # 2.5秒确保用户注意
            priority=10                      # 最高优先级
        ),
        
        # 系统兼容性模式（保留但不主动使用）
        ClaudiaLEDMode.SYSTEM_BOOT: LEDPattern(
            color=(0, 255, 0),
            brightness=5,
            flash_count=0,
            flash_interval=0.0,
            duration=0.0,
            priority=3
        ),
        
        ClaudiaLEDMode.SYSTEM_CALIBRATION: LEDPattern(
            color=(0, 0, 255),
            brightness=5,
            flash_count=10,                  # 持续闪烁
            flash_interval=0.5,
            duration=0.0,
            priority=4
        ),
        
        ClaudiaLEDMode.LOW_BATTERY: LEDPattern(
            color=(255, 255, 0),             # 黄色
            brightness=6,
            flash_count=10,                  # 持续闪烁
            flash_interval=1.0,              # 慢闪警告
            duration=0.0,
            priority=5
        ),
        
        ClaudiaLEDMode.SEARCH_LIGHT: LEDPattern(
            color=(255, 255, 255),
            brightness=10,                   # 最高亮度
            flash_count=0,
            flash_interval=0.0,
            duration=0.0,
            priority=2
        )
    }
    
    @classmethod
    def get_pattern(cls, mode: ClaudiaLEDMode) -> LEDPattern:
        """
        获取指定模式的LED模式参数
        
        Args:
            mode: LED模式
            
        Returns:
            LEDPattern: LED模式参数
        """
        return cls.PATTERNS.get(mode, cls.PATTERNS[ClaudiaLEDMode.OFF])
    
    @classmethod
    def get_all_modes(cls) -> List[ClaudiaLEDMode]:
        """获取所有可用的LED模式"""
        return list(cls.PATTERNS.keys())
    
    @classmethod
    def get_claudia_modes(cls) -> List[ClaudiaLEDMode]:
        """获取Claudia专用的LED模式（排除系统兼容模式）"""
        claudia_modes = [
            ClaudiaLEDMode.WAKE_CONFIRM,
            ClaudiaLEDMode.PROCESSING_VOICE,
            ClaudiaLEDMode.EXECUTING_ACTION,
            ClaudiaLEDMode.ACTION_COMPLETE,
            ClaudiaLEDMode.ERROR_STATE
        ]
        return claudia_modes
    
    @classmethod
    def validate_pattern(cls, pattern: LEDPattern) -> bool:
        """
        验证LED模式参数的有效性
        
        Args:
            pattern: LED模式参数
            
        Returns:
            bool: 参数是否有效
        """
        # 验证颜色范围
        r, g, b = pattern.color
        if not all(0 <= c <= 255 for c in [r, g, b]):
            return False
        
        # 验证亮度范围（VUI标准：0-10）
        if not (0 <= pattern.brightness <= 10):
            return False
        
        # 验证其他参数
        if pattern.flash_count < 0 or pattern.flash_interval < 0 or pattern.duration < 0:
            return False
        
        if not (1 <= pattern.priority <= 10):
            return False
            
        return True

class LEDModeRenderer:
    """
    LED模式渲染器
    
    负责将LED模式转换为具体的VUI控制指令
    """
    
    def __init__(self):
        """初始化渲染器"""
        self.logger = logging.getLogger(__name__)
        self.vui_client = None
        self.is_initialized = False
        
        # 渲染状态
        self.current_mode = ClaudiaLEDMode.OFF
        self.current_pattern = None
        self.render_thread = None
        self.render_active = False
        self.render_lock = threading.Lock()
        
        # 环境自适应参数
        self.environmental_brightness_factor = 1.0  # 环境亮度调节因子
        self.auto_brightness_enabled = True
        
    def initialize_vui(self) -> bool:
        """
        初始化VUI客户端
        
        Returns:
            bool: 初始化是否成功
        """
        if not VUI_AVAILABLE:
            self.logger.error("VUI客户端不可用")
            return False
            
        try:
            self.logger.info("初始化VUI客户端...")
            
            # 初始化通道
            ChannelFactoryInitialize(0)
            
            # 创建VUI客户端
            self.vui_client = VuiClient()
            self.vui_client.SetTimeout(3.0)
            self.vui_client.Init()
            
            # 测试连接
            code, current_brightness = self.vui_client.GetBrightness()
            if code == 0:
                self.logger.info(f"✅ VUI客户端初始化成功，当前LED亮度: {current_brightness}")
                self.is_initialized = True
                return True
            else:
                self.logger.error(f"VUI客户端连接测试失败，错误码: {code}")
                return False
                
        except Exception as e:
            self.logger.error(f"VUI客户端初始化失败: {e}")
            return False
    
    def render_mode(self, mode: ClaudiaLEDMode, duration_override: Optional[float] = None) -> bool:
        """
        渲染指定的LED模式
        
        Args:
            mode: 要渲染的LED模式
            duration_override: 可选的持续时间覆盖
            
        Returns:
            bool: 渲染是否成功开始
        """
        if not self.is_initialized:
            self.logger.error("VUI客户端未初始化")
            return False
            
        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        if not ClaudiaLEDModeDefinitions.validate_pattern(pattern):
            self.logger.error(f"无效的LED模式参数: {mode}")
            return False
        
        with self.render_lock:
            # 停止当前渲染
            self._stop_current_render()
            
            # 设置新模式
            self.current_mode = mode
            self.current_pattern = pattern
            
            # 应用持续时间覆盖
            if duration_override is not None:
                pattern = LEDPattern(
                    color=pattern.color,
                    brightness=pattern.brightness,
                    flash_count=pattern.flash_count,
                    flash_interval=pattern.flash_interval,
                    duration=duration_override,
                    priority=pattern.priority
                )
                self.current_pattern = pattern
            
            # 启动渲染线程
            self.render_active = True
            self.render_thread = threading.Thread(
                target=self._render_pattern_worker,
                args=(pattern,),
                daemon=True
            )
            self.render_thread.start()
            
            self.logger.info(f"开始渲染LED模式: {mode.value}")
            return True
    
    def _render_pattern_worker(self, pattern: LEDPattern) -> None:
        """
        LED模式渲染工作线程
        
        Args:
            pattern: 要渲染的LED模式
        """
        try:
            start_time = time.time()
            r, g, b = pattern.color
            
            # 应用环境自适应亮度
            effective_brightness = self._calculate_effective_brightness(pattern.brightness)
            
            if pattern.flash_count == 0:
                # 常亮模式
                self._set_led_color_brightness(r, g, b, effective_brightness)
                
                # 如果有持续时间限制，等待后关闭
                if pattern.duration > 0:
                    elapsed = 0
                    while elapsed < pattern.duration and self.render_active:
                        time.sleep(0.1)
                        elapsed = time.time() - start_time
                    
                    # 时间到，关闭LED
                    if self.render_active:
                        self._set_led_color_brightness(0, 0, 0, 0)
                        
            else:
                # 闪烁模式
                for flash_num in range(pattern.flash_count):
                    if not self.render_active:
                        break
                    
                    # 亮
                    self._set_led_color_brightness(r, g, b, effective_brightness)
                    time.sleep(pattern.flash_interval / 2)
                    
                    if not self.render_active:
                        break
                    
                    # 灭
                    self._set_led_color_brightness(0, 0, 0, 0)
                    
                    # 如果不是最后一次闪烁，等待间隔
                    if flash_num < pattern.flash_count - 1:
                        time.sleep(pattern.flash_interval / 2)
                
                # 闪烁完成后，根据持续时间决定是否保持状态
                if pattern.duration > 0:
                    elapsed = time.time() - start_time
                    remaining = pattern.duration - elapsed
                    if remaining > 0 and self.render_active:
                        time.sleep(remaining)
                
                # 最终关闭LED
                if self.render_active:
                    self._set_led_color_brightness(0, 0, 0, 0)
            
        except Exception as e:
            self.logger.error(f"LED模式渲染失败: {e}")
        finally:
            with self.render_lock:
                self.render_active = False
                self.current_mode = ClaudiaLEDMode.OFF
    
    def _set_led_color_brightness(self, r: int, g: int, b: int, brightness: int) -> bool:
        """
        设置LED颜色和亮度
        
        Args:
            r, g, b: RGB颜色值 (0-255)
            brightness: 亮度值 (0-10)
            
        Returns:
            bool: 设置是否成功
        """
        try:
            if self.vui_client is None:
                return False
            
            # 设置亮度
            brightness_code = self.vui_client.SetBrightness(brightness)
            
            # 注意：VUI客户端可能没有直接的RGB控制方法
            # 需要根据实际SDK能力调整
            # 这里假设使用AudioClient的LedControl方法
            
            if brightness_code != 0:
                self.logger.warning(f"设置亮度失败，错误码: {brightness_code}")
                return False
            
            # TODO: 实现RGB颜色控制
            # 可能需要使用AudioClient或其他接口
            
            return True
            
        except Exception as e:
            self.logger.error(f"设置LED颜色亮度失败: {e}")
            return False
    
    def _calculate_effective_brightness(self, target_brightness: int) -> int:
        """
        计算环境自适应后的有效亮度
        
        Args:
            target_brightness: 目标亮度 (0-10)
            
        Returns:
            int: 有效亮度 (0-10)
        """
        if not self.auto_brightness_enabled:
            return target_brightness
        
        # 应用环境亮度调节因子
        effective = int(target_brightness * self.environmental_brightness_factor)
        return max(0, min(10, effective))
    
    def _stop_current_render(self) -> None:
        """停止当前渲染"""
        if self.render_thread and self.render_thread.is_alive():
            self.render_active = False
            self.render_thread.join(timeout=1.0)
    
    def stop_all_rendering(self) -> None:
        """停止所有LED渲染"""
        with self.render_lock:
            self._stop_current_render()
            # 关闭所有LED
            if self.is_initialized:
                self._set_led_color_brightness(0, 0, 0, 0)
            self.current_mode = ClaudiaLEDMode.OFF
    
    def get_current_mode(self) -> ClaudiaLEDMode:
        """获取当前LED模式"""
        with self.render_lock:
            return self.current_mode
    
    def set_environmental_brightness_factor(self, factor: float) -> None:
        """
        设置环境亮度调节因子
        
        Args:
            factor: 调节因子 (0.1-2.0, 1.0为正常)
        """
        self.environmental_brightness_factor = max(0.1, min(2.0, factor))
        self.logger.info(f"环境亮度调节因子设置为: {self.environmental_brightness_factor}")
    
    def cleanup(self) -> None:
        """清理资源"""
        self.logger.info("清理LED模式渲染器...")
        self.stop_all_rendering()
        self.is_initialized = False


# 工厂函数
def create_led_mode_renderer() -> LEDModeRenderer:
    """
    创建LED模式渲染器实例
    
    Returns:
        LEDModeRenderer: 渲染器实例
    """
    return LEDModeRenderer()


if __name__ == "__main__":
    # 基础测试
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    print("🧪 LED模式定义测试")
    print("=" * 50)
    
    # 测试模式定义
    print("\n📋 Claudia专用LED模式:")
    for mode in ClaudiaLEDModeDefinitions.get_claudia_modes():
        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        print(f"   {mode.value}: RGB{pattern.color} 亮度={pattern.brightness} 优先级={pattern.priority}")
    
    # 测试渲染器（需要真实硬件）
    renderer = create_led_mode_renderer()
    
    try:
        if renderer.initialize_vui():
            print("✅ VUI客户端初始化成功")
            
            # 测试唤醒确认模式
            print("\n🟢 测试唤醒确认模式...")
            renderer.render_mode(ClaudiaLEDMode.WAKE_CONFIRM)
            time.sleep(3)
            
            print("测试完成！")
        else:
            print("❌ VUI客户端初始化失败（可能未连接机器人）")
            
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
    finally:
        renderer.cleanup() 