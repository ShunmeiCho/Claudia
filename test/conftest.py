"""
Claudia机器人项目pytest配置文件

全局pytest配置和fixture定义
"""

import pytest
import sys
from pathlib import Path

# 添加项目根目录到Python路径
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

@pytest.fixture(scope="session")
def project_root():
    """项目根目录fixture"""
    return PROJECT_ROOT

@pytest.fixture(scope="function")
def test_environment():
    """测试环境fixture"""
    from test.utils.test_helpers import setup_test_environment
    import inspect
    
    # 获取测试函数名称
    frame = inspect.currentframe()
    test_name = frame.f_back.f_code.co_name if frame.f_back else "unknown"
    
    env = setup_test_environment(test_name)
    yield env
    env.cleanup()

@pytest.fixture(scope="function")
def mock_robot():
    """模拟机器人连接fixture"""
    from test.utils.test_helpers import MockRobotConnection
    
    robot = MockRobotConnection(simulation_mode=True)
    robot.connect()
    yield robot
    robot.disconnect()
