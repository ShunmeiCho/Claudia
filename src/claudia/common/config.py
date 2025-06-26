# src/claudia/common/config.py
"""
配置管理模块

统一管理Claudia机器人系统的所有配置参数，
包括网络设置、传感器参数、AI模型配置等。
"""

import os
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, field


@dataclass
class NetworkConfig:
    """网络配置"""
    robot_ip: str = "192.168.123.161"  # Go2机器人IP
    host_ip: str = "192.168.123.162"   # 主机IP
    dds_domain_id: int = 0
    ros2_discovery_timeout: float = 5.0


@dataclass
class SensorConfig:
    """传感器配置"""
    lidar_enabled: bool = True
    camera_enabled: bool = True
    realsense_enabled: bool = True
    imu_enabled: bool = True
    force_sensor_enabled: bool = True
    
    # 相机配置
    main_camera_resolution: tuple = (1280, 720)
    main_camera_fps: int = 30
    realsense_resolution: tuple = (640, 480)
    realsense_fps: int = 30

@dataclass
class AIConfig:
    """AI组件配置"""
    # LLM配置
    llm_model: str = "Qwen2.5-7B-Instruct"
    llm_quantization: str = "int4"
    llm_max_tokens: int = 512
    llm_temperature: float = 0.7
    
    # ASR配置
    asr_model: str = "nvidia/parakeet-tdt_ctc-0.6b-ja"
    asr_language: str = "ja"
    asr_sample_rate: int = 16000
    
    # TTS配置
    tts_model: str = "esnya/japanese_speecht5_tts"
    tts_language: str = "ja"
    tts_speaker_id: int = 0
    
    # 唤醒词配置
    wake_words: list = field(default_factory=lambda: ["ねえ、くら", "くらこい"])
    wake_word_sensitivity: float = 0.5
    wake_word_timeout: float = 3.0
@dataclass
class RobotConfig:
    """机器人配置"""
    model: str = "Unitree Go2 R&D Plus"
    control_frequency: int = 100  # Hz
    max_linear_velocity: float = 1.5  # m/s
    max_angular_velocity: float = 1.0  # rad/s
    
    # 预设动作列表
    available_actions: list = field(default_factory=lambda: [
        'stand_up', 'sit_down', 'hello', 'dance', 'stretch',
        'move_forward', 'move_backward', 'turn_left', 'turn_right',
        'bow', 'shake_hand', 'wave', 'nod', 'trot', 'walk'
    ])


class Config:
    """主配置类 - 统一管理所有配置"""
    
    def __init__(self, config_file: Optional[str] = None):
        self.project_root = Path(__file__).parent.parent.parent.parent
        self.config_dir = self.project_root / "config"
        
        # 默认配置
        self.network = NetworkConfig()
        self.sensors = SensorConfig()
        self.ai = AIConfig()
        self.robot = RobotConfig()
        
        # 加载配置文件
        if config_file:
            self.load_config(config_file)
        else:
            self.load_default_config()    
    def load_config(self, config_file: str):
        """从YAML文件加载配置"""
        config_path = Path(config_file)
        if not config_path.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_file}")
            
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
            
        # 更新配置
        if 'network' in config_data:
            self._update_dataclass(self.network, config_data['network'])
        if 'sensors' in config_data:
            self._update_dataclass(self.sensors, config_data['sensors'])
        if 'ai' in config_data:
            self._update_dataclass(self.ai, config_data['ai'])
        if 'robot' in config_data:
            self._update_dataclass(self.robot, config_data['robot'])
    
    def load_default_config(self):
        """加载默认配置文件"""
        default_config = self.config_dir / "default.yaml"
        if default_config.exists():
            self.load_config(str(default_config))
    
    def _update_dataclass(self, obj, data: Dict[str, Any]):
        """更新dataclass对象"""
        for key, value in data.items():
            if hasattr(obj, key):
                setattr(obj, key, value)
    
    def save_config(self, config_file: str):
        """保存配置到YAML文件"""
        config_data = {
            'network': self.network.__dict__,
            'sensors': self.sensors.__dict__,
            'ai': self.ai.__dict__,
            'robot': self.robot.__dict__,
        }
        
        config_path = Path(config_file)
        config_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(config_data, f, default_flow_style=False, 
                     allow_unicode=True, indent=2)