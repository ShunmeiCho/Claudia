#!/usr/bin/env python3
"""
LED测试配置管理器
管理测试参数、环境设置和测试选项
"""

import os
import json
from dataclasses import dataclass, asdict, field
from typing import Dict, Any, List, Optional
from pathlib import Path

@dataclass
class PerformanceConfig:
    """性能测试配置"""
    max_response_time_ms: float = 200.0  # 最大响应时间（毫秒）
    stress_test_iterations: int = 100    # 压力测试迭代次数
    stress_test_duration: float = 10.0   # 压力测试最大持续时间（秒）
    performance_samples: int = 50        # 性能采样数量
    baseline_cpu_threshold: float = 80.0 # CPU使用率阈值（%）
    baseline_memory_mb: float = 100.0    # 内存使用基线（MB）

@dataclass
class LEDModeConfig:
    """LED模式测试配置"""
    wake_confirm_duration: float = 2.0      # 唤醒确认持续时间
    processing_timeout: float = 30.0        # 处理超时时间
    action_complete_duration: float = 1.5   # 动作完成持续时间
    error_state_duration: float = 2.5       # 错误状态持续时间
    mode_transition_delay: float = 0.1      # 模式转换延迟
    
@dataclass  
class EnvironmentConfig:
    """环境测试配置"""
    light_levels: List[str] = field(default_factory=lambda: ["extremely_dark", "dark", "dim", "normal", "bright", "extremely_bright"])
    brightness_levels: List[int] = field(default_factory=lambda: [0, 25, 50, 75, 100])
    adaptation_timeout: float = 5.0         # 适应超时时间
    camera_warmup_time: float = 2.0         # 摄像头预热时间

@dataclass
class HardwareConfig:
    """硬件测试配置"""
    unitree_ip: str = "192.168.123.161"     # Unitree机器人IP
    connection_timeout: float = 5.0          # 连接超时时间
    retry_attempts: int = 3                  # 重试次数
    hardware_required: bool = False          # 是否需要硬件
    mock_hardware: bool = True               # 是否使用模拟硬件

@dataclass
class StabilityConfig:
    """稳定性测试配置"""
    long_duration_hours: float = 1.0        # 长时间测试持续时间（小时）
    memory_leak_threshold_mb: float = 50.0  # 内存泄漏阈值（MB）
    error_rate_threshold: float = 1.0       # 错误率阈值（%）
    recovery_test_cycles: int = 10           # 恢复测试循环次数

class LEDTestConfig:
    """LED测试配置管理器"""
    
    def __init__(self, config_file: Optional[str] = None):
        """初始化配置管理器"""
        self.config_file = config_file or "test/led_system/config.json"
        
        # 默认配置
        self.performance = PerformanceConfig()
        self.led_modes = LEDModeConfig()
        self.environment = EnvironmentConfig()
        self.hardware = HardwareConfig()
        self.stability = StabilityConfig()
        
        # 加载配置文件（如果存在）
        self.load_config()
        
        # 环境变量覆盖
        self._apply_environment_overrides()
    
    def load_config(self):
        """从文件加载配置"""
        config_path = Path(self.config_file)
        if config_path.exists():
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                
                # 更新配置
                if 'performance' in config_data:
                    self._update_dataclass(self.performance, config_data['performance'])
                if 'led_modes' in config_data:
                    self._update_dataclass(self.led_modes, config_data['led_modes'])
                if 'environment' in config_data:
                    self._update_dataclass(self.environment, config_data['environment'])
                if 'hardware' in config_data:
                    self._update_dataclass(self.hardware, config_data['hardware'])
                if 'stability' in config_data:
                    self._update_dataclass(self.stability, config_data['stability'])
                    
                print(f"✅ 配置已从 {config_path} 加载")
                
            except Exception as e:
                print(f"⚠️ 配置文件加载失败: {e}")
    
    def save_config(self):
        """保存配置到文件"""
        config_data = {
            'performance': asdict(self.performance),
            'led_modes': asdict(self.led_modes),
            'environment': asdict(self.environment),
            'hardware': asdict(self.hardware),
            'stability': asdict(self.stability)
        }
        
        config_path = Path(self.config_file)
        config_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
            print(f"✅ 配置已保存到 {config_path}")
        except Exception as e:
            print(f"❌ 配置保存失败: {e}")
    
    def _update_dataclass(self, dataclass_instance, update_dict: Dict[str, Any]):
        """更新数据类实例"""
        for key, value in update_dict.items():
            if hasattr(dataclass_instance, key):
                setattr(dataclass_instance, key, value)
    
    def _apply_environment_overrides(self):
        """应用环境变量覆盖"""
        # 性能配置环境变量
        max_response_time = os.getenv('LED_TEST_MAX_RESPONSE_TIME')
        if max_response_time is not None:
            self.performance.max_response_time_ms = float(max_response_time)
        
        stress_iterations = os.getenv('LED_TEST_STRESS_ITERATIONS')
        if stress_iterations is not None:
            self.performance.stress_test_iterations = int(stress_iterations)
        
        # 硬件配置环境变量
        unitree_ip = os.getenv('UNITREE_IP')
        if unitree_ip is not None:
            self.hardware.unitree_ip = unitree_ip
        
        hardware_required = os.getenv('LED_TEST_HARDWARE_REQUIRED')
        if hardware_required is not None:
            self.hardware.hardware_required = hardware_required.lower() == 'true'
        
        mock_hardware = os.getenv('LED_TEST_MOCK_HARDWARE')
        if mock_hardware is not None:
            self.hardware.mock_hardware = mock_hardware.lower() == 'true'
    
    def get_test_mode(self) -> str:
        """获取测试模式"""
        if self.hardware.hardware_required and not self.hardware.mock_hardware:
            return "hardware"
        elif self.hardware.mock_hardware:
            return "simulation"
        else:
            return "mixed"
    
    def is_performance_test_enabled(self) -> bool:
        """是否启用性能测试"""
        return os.getenv('LED_TEST_SKIP_PERFORMANCE', 'false').lower() != 'true'
    
    def is_stress_test_enabled(self) -> bool:
        """是否启用压力测试"""
        return os.getenv('LED_TEST_SKIP_STRESS', 'false').lower() != 'true'
    
    def is_long_duration_test_enabled(self) -> bool:
        """是否启用长时间测试"""
        return os.getenv('LED_TEST_LONG_DURATION', 'false').lower() == 'true'
    
    def get_test_output_dir(self) -> Path:
        """获取测试输出目录"""
        output_dir = os.getenv('LED_TEST_OUTPUT_DIR', 'logs/led_tests')
        return Path(output_dir)
    
    def print_config_summary(self):
        """打印配置摘要"""
        print("🔧 LED测试配置摘要:")
        print(f"   测试模式: {self.get_test_mode()}")
        print(f"   最大响应时间: {self.performance.max_response_time_ms}ms")
        print(f"   压力测试迭代: {self.performance.stress_test_iterations}")
        print(f"   硬件IP: {self.hardware.unitree_ip}")
        print(f"   性能测试: {'启用' if self.is_performance_test_enabled() else '禁用'}")
        print(f"   压力测试: {'启用' if self.is_stress_test_enabled() else '禁用'}")
        print(f"   长时间测试: {'启用' if self.is_long_duration_test_enabled() else '禁用'}")
        print(f"   输出目录: {self.get_test_output_dir()}")

# 全局配置实例
_global_config = None

def get_led_test_config() -> LEDTestConfig:
    """获取全局LED测试配置实例"""
    global _global_config
    if _global_config is None:
        _global_config = LEDTestConfig()
    return _global_config

def reset_led_test_config():
    """重置全局配置（用于测试）"""
    global _global_config
    _global_config = None

if __name__ == "__main__":
    # 配置演示
    config = LEDTestConfig()
    config.print_config_summary()
    
    # 保存默认配置
    config.save_config()
    print("✅ 默认配置已生成") 