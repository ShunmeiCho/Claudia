"""
Claudia Robot Project pytest Configuration File

Global pytest configuration and fixture definitions
"""

import pytest
import sys
from pathlib import Path

# Add project root and src/ directory to Python path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "src"))

@pytest.fixture(scope="session")
def project_root():
    """Project root directory fixture"""
    return PROJECT_ROOT

@pytest.fixture(scope="function")
def test_environment():
    """Test environment fixture"""
    from test.utils.test_helpers import setup_test_environment
    import inspect

    # Get the test function name
    frame = inspect.currentframe()
    test_name = frame.f_back.f_code.co_name if frame.f_back else "unknown"

    env = setup_test_environment(test_name)
    yield env
    env.cleanup()

@pytest.fixture(scope="function")
def mock_robot():
    """Mock robot connection fixture"""
    from test.utils.test_helpers import MockRobotConnection

    robot = MockRobotConnection(simulation_mode=True)
    robot.connect()
    yield robot
    robot.disconnect()
