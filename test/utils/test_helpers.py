"""
Claudia机器人项目测试辅助工具

提供测试中常用的辅助函数和工具类。
"""

import os
import sys
import time
import tempfile
import shutil
from pathlib import Path
from typing import Any, Dict, Optional, List
from contextlib import contextmanager

# 添加项目根目录到Python路径
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

class TestEnvironment:
    """测试环境管理"""
    
    def __init__(self, test_name: str):
        self.test_name = test_name
        self.start_time = time.time()
        self.temp_dirs = []
        self.cleanup_callbacks = []
        
    def create_temp_dir(self, prefix: str = "claudia_test_") -> Path:
        """创建临时目录"""
        temp_dir = Path(tempfile.mkdtemp(prefix=f"{prefix}{self.test_name}_"))
        self.temp_dirs.append(temp_dir)
        return temp_dir
    
    def add_cleanup(self, callback):
        """添加清理回调函数"""
        self.cleanup_callbacks.append(callback)
    
    def cleanup(self):
        """清理测试环境"""
        # 执行清理回调
        for callback in self.cleanup_callbacks:
            try:
                callback()
            except Exception as e:
                print(f"清理回调执行失败: {e}")
        
        # 清理临时目录
        for temp_dir in self.temp_dirs:
            if temp_dir.exists():
                try:
                    shutil.rmtree(temp_dir)
                except Exception as e:
                    print(f"清理临时目录失败 {temp_dir}: {e}")
        
        # 打印测试时间
        duration = time.time() - self.start_time
        print(f"测试 {self.test_name} 运行时间: {duration:.2f}秒")

def setup_test_environment(test_name: str = "unknown") -> TestEnvironment:
    """设置测试环境"""
    return TestEnvironment(test_name)

@contextmanager
def mock_environment_variables(**env_vars):
    """临时设置环境变量"""
    old_env = {}
    
    # 保存原始环境变量
    for key, value in env_vars.items():
        old_env[key] = os.environ.get(key)
        os.environ[key] = str(value)
    
    try:
        yield
    finally:
        # 恢复原始环境变量
        for key, old_value in old_env.items():
            if old_value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old_value

def wait_for_condition(condition_func, timeout: float = 10.0, 
                      interval: float = 0.1, description: str = "条件满足") -> bool:
    """等待条件满足"""
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        try:
            if condition_func():
                return True
        except Exception as e:
            # 条件检查函数出错，继续等待
            pass
        
        time.sleep(interval)
    
    print(f"等待超时: {description} (超时时间: {timeout}秒)")
    return False

def create_mock_config(config_data: Dict[str, Any], 
                      config_file: Optional[Path] = None) -> Path:
    """创建模拟配置文件"""
    if config_file is None:
        config_file = Path(tempfile.mktemp(suffix=".yaml"))
    
    import yaml
    with open(config_file, 'w', encoding='utf-8') as f:
        yaml.dump(config_data, f, default_flow_style=False, allow_unicode=True)
    
    return config_file

def simulate_robot_response(topic: str, message_type: str, 
                          data: Dict[str, Any]) -> Dict[str, Any]:
    """模拟机器人响应消息"""
    return {
        'topic': topic,
        'message_type': message_type,
        'timestamp': time.time(),
        'data': data,
        'simulated': True
    }

class MockRobotConnection:
    """模拟机器人连接"""
    
    def __init__(self, simulation_mode: bool = True):
        self.simulation_mode = simulation_mode
        self.connected = False
        self.mock_data = {}
        
    def connect(self) -> bool:
        """模拟连接"""
        if self.simulation_mode:
            self.connected = True
            return True
        else:
            # 实际硬件连接逻辑
            return False
    
    def disconnect(self):
        """断开连接"""
        self.connected = False
    
    def set_mock_data(self, topic: str, data: Any):
        """设置模拟数据"""
        self.mock_data[topic] = data
    
    def get_data(self, topic: str) -> Optional[Any]:
        """获取数据"""
        if self.simulation_mode:
            return self.mock_data.get(topic)
        else:
            # 实际硬件数据获取逻辑
            return None

def validate_ros2_environment() -> bool:
    """验证ROS2环境是否正确设置"""
    required_env_vars = [
        'ROS_VERSION',
        'ROS_DISTRO',
        'RMW_IMPLEMENTATION'
    ]
    
    missing_vars = []
    for var in required_env_vars:
        if var not in os.environ:
            missing_vars.append(var)
    
    if missing_vars:
        print(f"缺少ROS2环境变量: {missing_vars}")
        return False
    
    # 检查ROS2版本
    if os.environ.get('ROS_VERSION') != '2':
        print(f"需要ROS2，当前版本: {os.environ.get('ROS_VERSION')}")
        return False
    
    return True

def check_network_connectivity(host: str = "8.8.8.8", port: int = 53, timeout: float = 3.0) -> bool:
    """检查网络连通性"""
    import socket
    
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except socket.error:
        return False

def get_available_network_interfaces() -> List[str]:
    """获取可用的网络接口"""
    import subprocess
    
    try:
        # Linux/macOS
        result = subprocess.run(['ip', 'link', 'show'], 
                              capture_output=True, text=True)
        interfaces = []
        for line in result.stdout.split('\n'):
            if ': ' in line and 'state UP' in line:
                interface = line.split(':')[1].strip().split('@')[0]
                interfaces.append(interface)
        return interfaces
    except:
        try:
            # 备用方法
            result = subprocess.run(['ifconfig'], 
                                  capture_output=True, text=True)
            interfaces = []
            for line in result.stdout.split('\n'):
                if line and not line.startswith(' ') and ':' in line:
                    interface = line.split(':')[0]
                    interfaces.append(interface)
            return interfaces
        except:
            return ['eth0', 'enp2s0', 'wlan0']  # 默认接口名

def create_test_log_file(test_name: str, content: str) -> Path:
    """创建测试日志文件"""
    log_dir = PROJECT_ROOT / "logs" / "tests"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    log_file = log_dir / f"{timestamp}_{test_name}.log"
    
    with open(log_file, 'w', encoding='utf-8') as f:
        f.write(f"测试: {test_name}\n")
        f.write(f"时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("-" * 50 + "\n")
        f.write(content)
    
    return log_file

# 测试装饰器
def hardware_test(require_robot: bool = True):
    """硬件测试装饰器"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if require_robot:
                print(f"⚠️ 硬件测试 {func.__name__} 需要真实机器人连接")
            return func(*args, **kwargs)
        return wrapper
    return decorator

def integration_test(dependencies: Optional[List[str]] = None):
    """集成测试装饰器"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if dependencies:
                print(f"🔗 集成测试 {func.__name__} 依赖: {', '.join(dependencies)}")
            return func(*args, **kwargs)
        return wrapper
    return decorator 