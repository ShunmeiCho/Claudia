#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/camera_config.py
# Generated: 2025-06-27
# Purpose: Unitree Go2前置摄像头配置和初始化管理

import cv2
import time
import json
import logging
import numpy as np
from typing import Dict, Tuple, Optional, Any
from dataclasses import dataclass
import os

@dataclass
class CameraSpec:
    """摄像头规格数据类"""
    resolution: Tuple[int, int]
    fps: float
    latency_ms: float
    color_format: str
    bit_depth: int

class CameraConfig:
    """前置摄像头配置管理器"""
    
    def __init__(self, config_path: str = None):
        """
        初始化摄像头配置
        
        Args:
            config_path: 配置文件路径
        """
        self.logger = logging.getLogger(__name__)
        self.config = self._load_config(config_path)
        self.camera = None
        self.is_initialized = False
        self.actual_spec = None
        
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        if config_path:
            config_file = config_path
        else:
            # 使用相对于当前脚本的路径
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_file = os.path.join(script_dir, "validation_config.json")
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            self.logger.error(f"配置文件未找到: {config_file}")
            return self._get_default_config()
        except json.JSONDecodeError as e:
            self.logger.error(f"配置文件JSON解析错误: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            "camera_config": {
                "target_resolution": [1280, 720],
                "fallback_resolution": [480, 1280],
                "target_fps": 30,
                "camera_id": 0,
                "timeout_seconds": 10
            }
        }
    
    def initialize_camera(self, method: str = "opencv") -> bool:
        """
        初始化摄像头
        
        Args:
            method: 初始化方法 ("opencv" 或 "unitree_sdk")
            
        Returns:
            bool: 初始化是否成功
        """
        try:
            if method == "opencv":
                return self._initialize_opencv()
            elif method == "unitree_sdk":
                return self._initialize_unitree_sdk()
            else:
                self.logger.error(f"不支持的初始化方法: {method}")
                return False
        except Exception as e:
            self.logger.error(f"摄像头初始化失败: {e}")
            return False
    
    def _initialize_opencv(self) -> bool:
        """使用OpenCV初始化摄像头"""
        self.logger.info("使用OpenCV初始化前置摄像头...")
        
        camera_config = self.config.get("camera_config", {})
        camera_id = camera_config.get("camera_id", 0)
        
        # 尝试不同的后端
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        
        for backend in backends:
            try:
                self.camera = cv2.VideoCapture(camera_id, backend)
                if self.camera.isOpened():
                    self.logger.info(f"成功打开摄像头 (ID: {camera_id}, Backend: {backend})")
                    break
                else:
                    self.camera.release()
                    self.camera = None
            except Exception as e:
                self.logger.warning(f"Backend {backend} 初始化失败: {e}")
                continue
        
        if not self.camera or not self.camera.isOpened():
            self.logger.error("所有后端都无法打开摄像头")
            return False
        
        # 配置摄像头参数
        if not self._configure_camera_parameters():
            self.logger.error("摄像头参数配置失败")
            return False
        
        # 验证配置
        if not self._verify_camera_configuration():
            self.logger.error("摄像头配置验证失败")
            return False
        
        self.is_initialized = True
        self.logger.info("OpenCV摄像头初始化完成")
        return True
    
    def _initialize_unitree_sdk(self) -> bool:
        """使用Unitree SDK初始化摄像头"""
        self.logger.info("使用Unitree SDK初始化前置摄像头...")
        
        try:
            # 这里应该集成Unitree SDK的摄像头初始化代码
            # 目前使用模拟实现
            self.logger.warning("Unitree SDK集成尚未实现，使用OpenCV回退")
            return self._initialize_opencv()
        except Exception as e:
            self.logger.error(f"Unitree SDK初始化失败: {e}")
            return False
    
    def _configure_camera_parameters(self) -> bool:
        """配置摄像头参数"""
        camera_config = self.config.get("camera_config", {})
        
        # 尝试设置目标分辨率
        target_resolution = camera_config.get("target_resolution", [1280, 720])
        fallback_resolution = camera_config.get("fallback_resolution", [480, 1280])
        target_fps = camera_config.get("target_fps", 30)
        
        # 设置分辨率
        resolution_set = False
        for resolution in [target_resolution, fallback_resolution]:
            width, height = resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # 验证分辨率是否设置成功
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            if actual_width == width and actual_height == height:
                self.logger.info(f"成功设置分辨率: {width}x{height}")
                resolution_set = True
                break
            else:
                self.logger.warning(f"分辨率设置失败: 目标{width}x{height}, 实际{actual_width}x{actual_height}")
        
        if not resolution_set:
            self.logger.warning("使用摄像头默认分辨率")
        
        # 设置帧率
        self.camera.set(cv2.CAP_PROP_FPS, target_fps)
        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        self.logger.info(f"设置帧率: 目标{target_fps}fps, 实际{actual_fps}fps")
        
        # 设置其他参数
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲延迟
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 自动曝光
        
        return True
    
    def _verify_camera_configuration(self) -> bool:
        """验证摄像头配置"""
        try:
            # 测试捕获几帧
            for i in range(5):
                ret, frame = self.camera.read()
                if not ret:
                    self.logger.error(f"测试帧捕获失败 (第{i+1}帧)")
                    return False
                time.sleep(0.1)
            
            # 记录实际规格
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.actual_spec = CameraSpec(
                resolution=(actual_width, actual_height),
                fps=actual_fps,
                latency_ms=0,  # 将在性能测试中测量
                color_format="BGR",
                bit_depth=8
            )
            
            self.logger.info(f"摄像头配置验证成功: {actual_width}x{actual_height}@{actual_fps}fps")
            return True
            
        except Exception as e:
            self.logger.error(f"摄像头配置验证失败: {e}")
            return False
    
    def capture_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        捕获单帧图像
        
        Returns:
            Tuple[bool, Optional[np.ndarray]]: (成功标志, 图像数据)
        """
        if not self.is_initialized or not self.camera:
            return False, None
        
        try:
            ret, frame = self.camera.read()
            return ret, frame
        except Exception as e:
            self.logger.error(f"图像捕获失败: {e}")
            return False, None
    
    def capture_frame_with_timestamp(self) -> Tuple[bool, Optional[np.ndarray], float]:
        """
        捕获带时间戳的单帧图像
        
        Returns:
            Tuple[bool, Optional[np.ndarray], float]: (成功标志, 图像数据, 时间戳)
        """
        timestamp = time.time()
        ret, frame = self.capture_frame()
        return ret, frame, timestamp
    
    def get_camera_properties(self) -> Dict[str, Any]:
        """获取摄像头属性"""
        if not self.is_initialized or not self.camera:
            return {}
        
        properties = {
            "width": int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": self.camera.get(cv2.CAP_PROP_FPS),
            "format": int(self.camera.get(cv2.CAP_PROP_FORMAT)),
            "brightness": self.camera.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": self.camera.get(cv2.CAP_PROP_CONTRAST),
            "saturation": self.camera.get(cv2.CAP_PROP_SATURATION),
            "auto_exposure": self.camera.get(cv2.CAP_PROP_AUTO_EXPOSURE),
            "exposure": self.camera.get(cv2.CAP_PROP_EXPOSURE),
            "buffer_size": int(self.camera.get(cv2.CAP_PROP_BUFFERSIZE))
        }
        
        return properties
    
    def adjust_camera_settings(self, **kwargs) -> bool:
        """
        调整摄像头设置
        
        Args:
            **kwargs: 摄像头属性键值对
            
        Returns:
            bool: 调整是否成功
        """
        if not self.is_initialized or not self.camera:
            return False
        
        property_map = {
            'brightness': cv2.CAP_PROP_BRIGHTNESS,
            'contrast': cv2.CAP_PROP_CONTRAST,
            'saturation': cv2.CAP_PROP_SATURATION,
            'exposure': cv2.CAP_PROP_EXPOSURE,
            'auto_exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
            'fps': cv2.CAP_PROP_FPS
        }
        
        for prop_name, prop_value in kwargs.items():
            if prop_name in property_map:
                cv_prop = property_map[prop_name]
                self.camera.set(cv_prop, prop_value)
                self.logger.info(f"设置 {prop_name} = {prop_value}")
        
        return True
    
    def release(self):
        """释放摄像头资源"""
        if self.camera:
            self.camera.release()
            self.camera = None
            self.is_initialized = False
            self.logger.info("摄像头资源已释放")
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.release()

# 测试函数
def test_camera_config():
    """测试摄像头配置功能"""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    with CameraConfig() as camera_config:
        # 测试初始化
        if camera_config.initialize_camera("opencv"):
            print("摄像头初始化成功")
            
            # 测试属性获取
            properties = camera_config.get_camera_properties()
            print(f"摄像头属性: {properties}")
            
            # 测试图像捕获
            ret, frame = camera_config.capture_frame()
            if ret:
                print(f"图像捕获成功: {frame.shape}")
            else:
                print("图像捕获失败")
        else:
            print("摄像头初始化失败")

if __name__ == "__main__":
    test_camera_config() 