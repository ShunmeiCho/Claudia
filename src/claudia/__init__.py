# src/claudia/__init__.py
"""
Claudia智能四足机器人系统

基于Unitree Go2 R&D Plus平台的智能四足机器人系统，
通过深度集成大语言模型技术实现高级自然语言交互功能。

主要功能模块：
- robot_controller: 机器人控制和动作执行
- ai_components: AI组件（LLM, ASR, TTS, 唤醒词检测）
- sensors: 传感器数据处理（LiDAR, Camera, IMU, Force）
- vision: 视觉处理和目标检测
- navigation: SLAM和路径规划
- audio: 音频处理和语音交互
- common: 通用工具和实用函数
"""

__version__ = "0.1.0"
__author__ = "Claudia Development Team"
__description__ = "Claudia智能四足机器人系统"

# 版本信息
VERSION = __version__
AUTHOR = __author__
DESCRIPTION = __description__

# 项目配置
PROJECT_NAME = "claudia"
ROBOT_MODEL = "Unitree Go2 R&D Plus"
SUPPORTED_ROS_VERSION = "ROS2 Foxy"
TARGET_PLATFORM = "NVIDIA Jetson Orin NX"