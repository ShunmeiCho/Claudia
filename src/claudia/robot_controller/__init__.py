# src/claudia/robot_controller/__init__.py
"""
Claudia机器人控制器模块

提供与Unitree Go2机器人的硬件控制接口
包括LED控制、传感器数据读取等功能

Author: Claudia AI System
"""

# 原有LED控制器 (LowCmd基础设施)
from .led_controller import ClaudiaLEDController, create_led_controller, LEDControlMode

# 新增LED模式定义和渲染器
from .led_patterns import (
    ClaudiaLEDMode, LEDPattern, ClaudiaLEDModeDefinitions,
    LEDModeRenderer, create_led_mode_renderer
)

# LED状态机
from .led_state_machine import (
    LEDStateMachine, LEDStateRequest, LEDStateHistory,
    create_led_state_machine
)

# 统一LED控制器 (推荐使用)
from .unified_led_controller import (
    UnifiedLEDController, LEDControlMethod, AdvancedEnvironmentalLightInfo,
    EnvironmentalAdaptationProfile, create_unified_led_controller
)

__all__ = [
    # 原有LED控制器
    'ClaudiaLEDController',
    'create_led_controller', 
    'LEDControlMode',
    
    # LED模式定义
    'ClaudiaLEDMode',
    'LEDPattern',
    'ClaudiaLEDModeDefinitions',
    'LEDModeRenderer',
    'create_led_mode_renderer',
    
    # LED状态机
    'LEDStateMachine',
    'LEDStateRequest', 
    'LEDStateHistory',
    'create_led_state_machine',
    
    # 统一LED控制器 (推荐)
    'UnifiedLEDController',
    'LEDControlMethod',
    'AdvancedEnvironmentalLightInfo',
    'EnvironmentalAdaptationProfile',
    'create_unified_led_controller'
]

__version__ = "0.2.0"  # 升级版本号，增加了LED模式定义和状态机功能

# 快捷创建函数，为上层应用提供便利
def create_claudia_led_system(preferred_method: str = "vui", 
                             enable_environmental_adaptation: bool = True):
    """
    创建完整的Claudia LED控制系统
    
    这是推荐的方式来创建LED控制系统，集成了所有功能：
    - VUI/LowCmd双重控制方法
    - 5种专用LED状态指示器
    - 优先级管理和状态机
    - 环境自适应亮度调节
    - 系统兼容性保护
    
    Args:
        preferred_method: 首选控制方法 ("vui", "lowcmd", "auto")
        enable_environmental_adaptation: 是否启用环境自适应
        
    Returns:
        UnifiedLEDController: 统一LED控制器实例
        
    Example:
        >>> from claudia.robot_controller import create_claudia_led_system
        >>> led_system = create_claudia_led_system()
        >>> led_system.initialize()
        >>> led_system.wake_confirm()  # 🟢 绿色双闪
        >>> led_system.processing_voice()  # 🔵 蓝色常亮
        >>> led_system.cleanup()
    """
    method_map = {
        "vui": LEDControlMethod.VUI_CLIENT,
        "lowcmd": LEDControlMethod.LOW_CMD,
        "auto": LEDControlMethod.AUTO_SELECT
    }
    
    control_method = method_map.get(preferred_method, LEDControlMethod.VUI_CLIENT)
    
    return create_unified_led_controller(
        preferred_method=control_method,
        enable_environmental_adaptation=enable_environmental_adaptation
    )