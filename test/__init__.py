"""
Claudia机器人项目测试套件

此包包含Claudia智能四足机器人系统的所有测试代码：
- unit/: 单元测试
- integration/: 集成测试  
- hardware/: 硬件相关测试
- utils/: 测试工具函数
"""

__version__ = "1.0.0"
__author__ = "Claudia Project Team"

# 导入常用测试工具
import sys
import os
from pathlib import Path

# 添加项目根目录到Python路径
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT)) 