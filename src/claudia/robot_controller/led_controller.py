#!/usr/bin/env python3
"""
Claudia机器人LED控制器
基于Unitree Go2 VUI客户端的LED亮度控制

唯一可用的LED控制方式：
- VuiClient.SetBrightness(0-10) - LED亮度控制（无需特殊权限）

已废除的方案（权限不足）：
- AudioClient.LedControl(r,g,b) - 需要系统级权限
- LowCmd uint8[12] led字段 - 需要机器人控制权限

Author: Claudia AI System
Generated: 2025-07-04
"""

import logging
from typing import Optional, Dict, Any
from enum import Enum

class LEDBrightnessLevel(Enum):
    """LED亮度级别枚举"""
    OFF = 0          # 关闭
    VERY_LOW = 1     # 很暗
    LOW = 2          # 暗
    DIM = 3          # 较暗
    MEDIUM_LOW = 4   # 中等偏暗
    MEDIUM = 5       # 中等
    MEDIUM_HIGH = 6  # 中等偏亮
    BRIGHT = 7       # 亮
    HIGH = 8         # 很亮
    VERY_HIGH = 9    # 极亮
    MAX = 10         # 最亮

class ClaudiaLEDMode(Enum):
    """Claudia LED模式（通过亮度变化实现）"""
    IDLE = "idle"                    # 空闲：低亮度常亮
    WAKE_CONFIRM = "wake_confirm"    # 唤醒确认：中等亮度
    PROCESSING = "processing"        # 处理中：闪烁模式
    EXECUTING = "executing"          # 执行中：高亮度常亮
    ACTION_COMPLETE = "complete"     # 完成：渐变模式
    ERROR = "error"                  # 错误：快速闪烁

class LEDController:
    """LED控制器类"""
    
    def __init__(self):
        """初始化LED控制器"""
        self.logger = logging.getLogger(__name__)
        self.vui_client = None
        self.current_brightness = 0
        self.current_mode = ClaudiaLEDMode.IDLE
        self.is_initialized = False
        
    def initialize(self) -> bool:
        """
        初始化LED控制器
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            # 尝试导入Unitree SDK2
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.vui.vui_client import VuiClient
            
            # 初始化通道
            ChannelFactoryInitialize(0)
            
            # 创建VUI客户端
            self.vui_client = VuiClient()
            self.vui_client.SetTimeout(3.0)
            self.vui_client.Init()
            
            # 获取当前亮度
            code, brightness = self.vui_client.GetBrightness()
            if code == 0:
                self.current_brightness = brightness
                self.is_initialized = True
                self.logger.info(f"✅ LED控制器初始化成功，当前亮度: {brightness}")
                return True
            else:
                self.logger.error(f"❌ 获取LED亮度失败，错误码: {code}")
                return False
                
        except ImportError as e:
            self.logger.error(f"❌ Unitree SDK2 导入失败: {e}")
            return False
        except Exception as e:
            self.logger.error(f"❌ LED控制器初始化失败: {e}")
            return False
    
    def set_brightness(self, brightness: int) -> bool:
        """
        设置LED亮度
        
        Args:
            brightness: 亮度级别 (0-10)
            
        Returns:
            bool: 设置是否成功
        """
        if not self.is_initialized:
            self.logger.error("LED控制器未初始化")
            return False
        
        if not (0 <= brightness <= 10):
            self.logger.error(f"亮度级别必须在0-10之间，当前值: {brightness}")
            return False
        
        try:
            code = self.vui_client.SetBrightness(brightness)
            if code == 0:
                self.current_brightness = brightness
                self.logger.debug(f"✅ LED亮度设置为: {brightness}")
                return True
            else:
                self.logger.error(f"❌ LED亮度设置失败，错误码: {code}")
                return False
        except Exception as e:
            self.logger.error(f"❌ LED亮度设置异常: {e}")
            return False
    
    def get_brightness(self) -> Optional[int]:
        """
        获取当前LED亮度
        
        Returns:
            Optional[int]: 当前亮度级别，失败时返回None
        """
        if not self.is_initialized:
            return None
        
        try:
            code, brightness = self.vui_client.GetBrightness()
            if code == 0:
                self.current_brightness = brightness
                return brightness
            else:
                self.logger.error(f"❌ 获取LED亮度失败，错误码: {code}")
                return None
        except Exception as e:
            self.logger.error(f"❌ 获取LED亮度异常: {e}")
            return None
    
    def set_mode(self, mode: ClaudiaLEDMode) -> bool:
        """
        设置Claudia LED模式
        
        Args:
            mode: LED模式
            
        Returns:
            bool: 设置是否成功
        """
        brightness_map = {
            ClaudiaLEDMode.IDLE: LEDBrightnessLevel.LOW.value,
            ClaudiaLEDMode.WAKE_CONFIRM: LEDBrightnessLevel.MEDIUM.value,
            ClaudiaLEDMode.PROCESSING: LEDBrightnessLevel.MEDIUM_HIGH.value,
            ClaudiaLEDMode.EXECUTING: LEDBrightnessLevel.HIGH.value,
            ClaudiaLEDMode.ACTION_COMPLETE: LEDBrightnessLevel.BRIGHT.value,
            ClaudiaLEDMode.ERROR: LEDBrightnessLevel.MAX.value
        }
        
        brightness = brightness_map.get(mode, LEDBrightnessLevel.MEDIUM.value)
        
        if self.set_brightness(brightness):
            self.current_mode = mode
            self.logger.info(f"✅ LED模式设置为: {mode.value} (亮度: {brightness})")
            return True
        else:
            self.logger.error(f"❌ LED模式设置失败: {mode.value}")
            return False
    
    def turn_off(self) -> bool:
        """关闭LED"""
        return self.set_brightness(LEDBrightnessLevel.OFF.value)
    
    def turn_on(self, brightness: int = 5) -> bool:
        """打开LED到指定亮度"""
        return self.set_brightness(brightness)
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取LED控制器状态
        
        Returns:
            Dict[str, Any]: 控制器状态信息
        """
        return {
            'initialized': self.is_initialized,
            'current_brightness': self.current_brightness,
            'current_mode': self.current_mode.value if self.current_mode else None,
            'available_brightness_levels': list(range(11)),  # 0-10
            'supported_modes': [mode.value for mode in ClaudiaLEDMode]
        }
    
    def cleanup(self):
        """清理LED控制器资源"""
        if self.is_initialized:
            # 关闭LED
            self.turn_off()
            self.logger.info("🧹 LED控制器清理完成")

# 便捷函数
def create_led_controller() -> LEDController:
    """创建LED控制器实例"""
    return LEDController()

# 全局实例（可选）
_global_led_controller = None

def get_led_controller() -> LEDController:
    """获取全局LED控制器实例"""
    global _global_led_controller
    if _global_led_controller is None:
        _global_led_controller = create_led_controller()
    return _global_led_controller