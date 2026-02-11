#!/usr/bin/env python3
"""
LED控制系统测试框架
任务6.5: 全面测试、验证和性能优化

本模块提供了完整的LED控制系统测试能力，包括：
- 功能验证测试
- 性能基准测试  
- 压力和稳定性测试
- 视觉验证工具
- 自动化测试报告
"""

__version__ = "1.0.0"

# 导出主要测试类和工具
from .led_test_base import LEDTestBase
from .test_config import LEDTestConfig
from .data_collector import LEDTestDataCollector

__all__ = [
    'LEDTestBase',
    'LEDTestConfig', 
    'LEDTestDataCollector',
] 