#!/usr/bin/env python3
"""
ROS2管理器模块 - Claudia机器人系统
集成ROS2 Foxy环境与Claudia项目结构
"""

import os
import sys
import subprocess
import logging
from pathlib import Path
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

# 设置日志
logger = logging.getLogger(__name__)

class ROS2Manager:
    """ROS2环境和节点管理器"""
    
    def __init__(self, project_root: str = None):
        """
        初始化ROS2管理器
        
        Args:
            project_root: 项目根目录路径，默认自动检测
        """
        self.project_root = Path(project_root or self._find_project_root())
        self.cyclonedds_ws = self.project_root / "cyclonedds_ws"
        self.is_initialized = False
        self.executor: Optional[MultiThreadedExecutor] = None
        self.nodes = {}
        self._executor_thread: Optional[threading.Thread] = None
        
    def _find_project_root(self) -> str:
        """自动查找项目根目录"""
        current_path = Path(__file__).parent
        while current_path.parent != current_path:
            if (current_path / "src" / "claudia").exists():
                return str(current_path)
            current_path = current_path.parent
        return str(Path.cwd())
    
    def setup_environment(self) -> bool:
        """设置ROS2环境变量和路径"""
        try:
            # 设置基础ROS2环境
            ros2_setup = "/opt/ros/foxy/setup.bash"
            if not Path(ros2_setup).exists():
                logger.error(f"ROS2 Foxy setup file not found: {ros2_setup}")
                return False
            
            # 设置工作空间环境
            workspace_setup = self.cyclonedds_ws / "install" / "setup.bash"
            if not workspace_setup.exists():
                logger.warning(f"Workspace setup not found: {workspace_setup}")
                logger.info("尝试构建工作空间...")
                if not self._build_workspace():
                    return False
            
            # 设置环境变量
            self._set_ros2_env_vars()
            
            # 验证环境
            return self._verify_environment()
            
        except Exception as e:
            logger.error(f"Environment setup failed: {e}")
            return False
    
    def _set_ros2_env_vars(self):
        """设置ROS2相关环境变量"""
        env_vars = {
            'ROS_VERSION': '2',
            'ROS_DISTRO': 'foxy',
            'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
            'CYCLONEDDS_URI': '<不限制通信域>',
            'ROS_DOMAIN_ID': '0',  # 默认域ID
        }
        
        for key, value in env_vars.items():
            os.environ[key] = value
            
        # 添加工作空间到Python路径
        install_path = self.cyclonedds_ws / "install"
        if install_path.exists():
            python_path = str(install_path / "lib" / "python3.8" / "site-packages")
            if python_path not in sys.path:
                sys.path.insert(0, python_path)
    
    def _build_workspace(self) -> bool:
        """构建ROS2工作空间"""
        try:
            logger.info("正在构建cyclonedds_ws工作空间...")
            
            # 切换到工作空间目录
            os.chdir(self.cyclonedds_ws)
            
            # 源ROS2环境并构建
            cmd = [
                'bash', '-c', 
                'source /opt/ros/foxy/setup.bash && colcon build --symlink-install'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
            
            if result.returncode == 0:
                logger.info("工作空间构建成功")
                return True
            else:
                logger.error(f"工作空间构建失败: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("工作空间构建超时")
            return False
        except Exception as e:
            logger.error(f"工作空间构建异常: {e}")
            return False
        finally:
            # 返回项目根目录
            os.chdir(self.project_root)
    
    def _verify_environment(self) -> bool:
        """验证ROS2环境设置"""
        try:
            # 检查ROS2命令
            result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
            if result.returncode != 0:
                logger.error("ros2 command not available")
                return False
            
            # 检查Python包导入
            try:
                import rclpy
                import std_msgs
                import geometry_msgs
                logger.info("ROS2 Python packages imported successfully")
            except ImportError as e:
                logger.error(f"Failed to import ROS2 packages: {e}")
                return False
            
            # 检查Unitree包
            try:
                import unitree_go.msg
                import unitree_api.msg
                logger.info("Unitree ROS2 packages imported successfully")
            except ImportError as e:
                logger.warning(f"Unitree packages not available: {e}")
                # 这不是致命错误，继续
            
            return True
            
        except Exception as e:
            logger.error(f"Environment verification failed: {e}")
            return False
    
    def initialize_rclpy(self) -> bool:
        """初始化rclpy"""
        try:
            if not rclpy.ok():
                rclpy.init()
                logger.info("rclpy initialized successfully")
            
            # 创建执行器
            self.executor = MultiThreadedExecutor()
            
            # 在单独线程中运行执行器
            self._executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self._executor_thread.start()
            
            self.is_initialized = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize rclpy: {e}")
            return False
    
    def create_node(self, node_name: str, node_class: type = None) -> Optional[Node]:
        """
        创建ROS2节点
        
        Args:
            node_name: 节点名称
            node_class: 节点类，默认为基础Node类
            
        Returns:
            创建的节点实例
        """
        try:
            if not self.is_initialized:
                logger.error("ROS2 manager not initialized")
                return None
            
            # 使用提供的类或默认Node类
            node_cls = node_class or Node
            node = node_cls(node_name)
            
            # 添加到执行器
            self.executor.add_node(node)
            self.nodes[node_name] = node
            
            logger.info(f"Created ROS2 node: {node_name}")
            return node
            
        except Exception as e:
            logger.error(f"Failed to create node {node_name}: {e}")
            return None
    
    def get_node(self, node_name: str) -> Optional[Node]:
        """获取已创建的节点"""
        return self.nodes.get(node_name)
    
    def remove_node(self, node_name: str) -> bool:
        """移除节点"""
        try:
            if node_name in self.nodes:
                node = self.nodes[node_name]
                self.executor.remove_node(node)
                node.destroy_node()
                del self.nodes[node_name]
                logger.info(f"Removed ROS2 node: {node_name}")
                return True
            return False
        except Exception as e:
            logger.error(f"Failed to remove node {node_name}: {e}")
            return False
    
    def shutdown(self):
        """关闭ROS2管理器"""
        try:
            # 移除所有节点
            for node_name in list(self.nodes.keys()):
                self.remove_node(node_name)
            
            # 关闭执行器
            if self.executor:
                self.executor.shutdown()
            
            # 等待执行器线程结束
            if self._executor_thread and self._executor_thread.is_alive():
                self._executor_thread.join(timeout=5.0)
            
            # 关闭rclpy
            if rclpy.ok():
                rclpy.shutdown()
            
            self.is_initialized = False
            logger.info("ROS2 manager shutdown completed")
            
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """获取ROS2管理器状态"""
        return {
            'initialized': self.is_initialized,
            'rclpy_ok': rclpy.ok() if self.is_initialized else False,
            'executor_running': self._executor_thread.is_alive() if self._executor_thread else False,
            'active_nodes': list(self.nodes.keys()),
            'workspace_path': str(self.cyclonedds_ws),
            'project_root': str(self.project_root),
        }

# 全局实例
_ros2_manager = None

def get_ros2_manager() -> ROS2Manager:
    """获取ROS2管理器单例"""
    global _ros2_manager
    if _ros2_manager is None:
        _ros2_manager = ROS2Manager()
    return _ros2_manager

def initialize_ros2_integration() -> bool:
    """初始化ROS2集成"""
    manager = get_ros2_manager()
    
    # 设置环境
    if not manager.setup_environment():
        logger.error("Failed to setup ROS2 environment")
        return False
    
    # 初始化rclpy
    if not manager.initialize_rclpy():
        logger.error("Failed to initialize rclpy")
        return False
    
    logger.info("ROS2 integration initialized successfully")
    return True 