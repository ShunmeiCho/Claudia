# src/claudia/sensors/__init__.py
"""
传感器模块

处理所有传感器数据，包括：
- 4D LiDAR L1 - 环境点云数据
- Go2主摄像头 - 1280x720@120° 视觉数据
- Intel RealSense D435i - RGB-D深度数据
- IMU - 惯性测量单元数据
- 足端力传感器 - 接触力数据

对应TaskMaster任务:
- Task 4: Hardware & Sensor System Validation
- Task 5: Intel RealSense D435i Integration
- Task 15: Multi-modal Sensor Fusion Framework
"""

# 导入传感器处理类（待实现）
# from .lidar import LidarProcessor
# from .camera import CameraManager
# from .imu import IMUProcessor
# from .force import ForceProcessor
# from .fusion import SensorFusion

__all__ = [
    # 'LidarProcessor',
    # 'CameraManager',
    # 'IMUProcessor',
    # 'ForceProcessor',
    # 'SensorFusion',
]