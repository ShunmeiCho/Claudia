"""
通用工具函数模块 - Claudia机器人系统

提供项目范围内的通用工具函数和辅助功能。
"""

import os
import sys
import time
import json
import yaml
import subprocess
from pathlib import Path
from typing import Dict, Any, Optional, Union, List
from datetime import datetime

def get_project_root() -> Path:
    """
    获取项目根目录
    
    Returns:
        项目根目录路径
    """
    current_path = Path(__file__).parent
    while current_path.parent != current_path:
        if (current_path / "src" / "claudia").exists():
            return current_path
        current_path = current_path.parent
    return Path.cwd()

def load_config(config_file: str = "config/default.yaml") -> Dict[str, Any]:
    """
    加载配置文件
    
    Args:
        config_file: 配置文件路径（相对于项目根目录）
        
    Returns:
        配置字典
    """
    project_root = get_project_root()
    config_path = project_root / config_file
    
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件未找到: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def save_config(config: Dict[str, Any], config_file: str = "config/default.yaml") -> bool:
    """
    保存配置到文件
    
    Args:
        config: 配置字典
        config_file: 配置文件路径（相对于项目根目录）
        
    Returns:
        是否保存成功
    """
    try:
        project_root = get_project_root()
        config_path = project_root / config_file
        
        # 确保目录存在
        config_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
        return True
    except Exception:
        return False

def get_timestamp(format_str: str = "%Y-%m-%d %H:%M:%S") -> str:
    """
    获取当前时间戳
    
    Args:
        format_str: 时间格式字符串
        
    Returns:
        格式化的时间字符串
    """
    return datetime.now().strftime(format_str)

def run_command(cmd: Union[str, List[str]], 
                cwd: Optional[str] = None, 
                timeout: int = 30,
                capture_output: bool = True) -> tuple:
    """
    运行系统命令
    
    Args:
        cmd: 命令字符串或命令列表
        cwd: 工作目录
        timeout: 超时时间（秒）
        capture_output: 是否捕获输出
        
    Returns:
        (返回码, 标准输出, 标准错误)
    """
    try:
        if isinstance(cmd, str):
            cmd = cmd.split()
            
        result = subprocess.run(
            cmd,
            cwd=cwd,
            timeout=timeout,
            capture_output=capture_output,
            text=True
        )
        
        return result.returncode, result.stdout, result.stderr
        
    except subprocess.TimeoutExpired:
        return -1, "", "Command timeout"
    except Exception as e:
        return -1, "", str(e)

def ensure_directory(path: Union[str, Path]) -> bool:
    """
    确保目录存在，不存在则创建
    
    Args:
        path: 目录路径
        
    Returns:
        是否成功
    """
    try:
        Path(path).mkdir(parents=True, exist_ok=True)
        return True
    except Exception:
        return False

def file_exists(path: Union[str, Path]) -> bool:
    """
    检查文件是否存在
    
    Args:
        path: 文件路径
        
    Returns:
        文件是否存在
    """
    return Path(path).exists()

def read_json(file_path: Union[str, Path]) -> Optional[Dict[str, Any]]:
    """
    读取JSON文件
    
    Args:
        file_path: JSON文件路径
        
    Returns:
        JSON数据字典，失败返回None
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return None

def write_json(data: Dict[str, Any], file_path: Union[str, Path]) -> bool:
    """
    写入JSON文件
    
    Args:
        data: 要写入的数据
        file_path: JSON文件路径
        
    Returns:
        是否写入成功
    """
    try:
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception:
        return False

def get_system_info() -> Dict[str, str]:
    """
    获取系统信息
    
    Returns:
        系统信息字典
    """
    info = {
        'platform': sys.platform,
        'python_version': sys.version.split()[0],
        'architecture': os.uname().machine if hasattr(os, 'uname') else 'unknown',
    }
    
    # 获取Ubuntu版本
    try:
        with open('/etc/lsb-release', 'r') as f:
            for line in f:
                if line.startswith('DISTRIB_RELEASE='):
                    info['ubuntu_version'] = line.split('=')[1].strip()
                    break
    except:
        pass
    
    # 获取Tegra版本
    try:
        with open('/etc/nv_tegra_release', 'r') as f:
            content = f.read().strip()
            if content:
                info['tegra_version'] = content
    except:
        pass
    
    return info

def format_size(size_bytes: int) -> str:
    """
    格式化文件大小
    
    Args:
        size_bytes: 字节数
        
    Returns:
        格式化的大小字符串
    """
    if size_bytes == 0:
        return "0B"
    
    size_names = ["B", "KB", "MB", "GB", "TB"]
    i = 0
    while size_bytes >= 1024.0 and i < len(size_names) - 1:
        size_bytes /= 1024.0
        i += 1
    
    return f"{size_bytes:.1f}{size_names[i]}"

def retry_on_failure(func, max_attempts: int = 3, delay: float = 1.0):
    """
    失败重试装饰器
    
    Args:
        func: 要重试的函数
        max_attempts: 最大尝试次数
        delay: 重试间隔（秒）
        
    Returns:
        装饰后的函数
    """
    def wrapper(*args, **kwargs):
        last_exception = None
        
        for attempt in range(max_attempts):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                last_exception = e
                if attempt < max_attempts - 1:
                    time.sleep(delay)
                continue
        
        # 所有尝试都失败了，抛出最后的异常
        raise last_exception
    
    return wrapper

def check_ros2_environment() -> bool:
    """
    检查ROS2环境是否可用
    
    Returns:
        ROS2环境是否可用
    """
    # 检查环境变量
    ros_version = os.environ.get('ROS_VERSION')
    ros_distro = os.environ.get('ROS_DISTRO')
    
    if ros_version != '2' or ros_distro != 'foxy':
        return False
    
    # 检查ros2命令
    returncode, _, _ = run_command(['ros2', '--version'], timeout=10)
    return returncode == 0

def source_ros2_environment():
    """
    在当前进程中源ROS2环境
    注意：这只对子进程有效，不能改变当前Python进程的环境
    """
    ros2_setup = "/opt/ros/foxy/setup.bash"
    if file_exists(ros2_setup):
        # 设置基础环境变量
        os.environ['ROS_VERSION'] = '2'
        os.environ['ROS_DISTRO'] = 'foxy'
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        
        # 添加工作空间路径
        project_root = get_project_root()
        workspace_install = project_root / "cyclonedds_ws" / "install"
        
        if workspace_install.exists():
            python_path = workspace_install / "lib" / "python3.8" / "site-packages"
            if python_path.exists():
                current_path = os.environ.get('PYTHONPATH', '')
                os.environ['PYTHONPATH'] = f"{python_path}:{current_path}"

def get_available_memory() -> Dict[str, int]:
    """
    获取系统内存信息
    
    Returns:
        内存信息字典（单位：bytes）
    """
    try:
        with open('/proc/meminfo', 'r') as f:
            meminfo = {}
            for line in f:
                key, value = line.split(':')
                meminfo[key.strip()] = int(value.split()[0]) * 1024  # 转换为bytes
            
            return {
                'total': meminfo.get('MemTotal', 0),
                'available': meminfo.get('MemAvailable', 0),
                'free': meminfo.get('MemFree', 0),
                'used': meminfo.get('MemTotal', 0) - meminfo.get('MemAvailable', 0)
            }
    except:
        return {'total': 0, 'available': 0, 'free': 0, 'used': 0}

def get_disk_usage(path: Union[str, Path] = ".") -> Dict[str, int]:
    """
    获取磁盘使用情况
    
    Args:
        path: 检查的路径
        
    Returns:
        磁盘使用信息字典（单位：bytes）
    """
    try:
        statvfs = os.statvfs(path)
        total = statvfs.f_frsize * statvfs.f_blocks
        free = statvfs.f_frsize * statvfs.f_available
        used = total - free
        
        return {
            'total': total,
            'used': used,
            'free': free,
            'percentage': (used / total * 100) if total > 0 else 0
        }
    except:
        return {'total': 0, 'used': 0, 'free': 0, 'percentage': 0}

# 导出所有公共函数
__all__ = [
    'get_project_root',
    'load_config',
    'save_config', 
    'get_timestamp',
    'run_command',
    'ensure_directory',
    'file_exists',
    'read_json',
    'write_json',
    'get_system_info',
    'format_size',
    'retry_on_failure',
    'check_ros2_environment',
    'source_ros2_environment',
    'get_available_memory',
    'get_disk_usage',
] 