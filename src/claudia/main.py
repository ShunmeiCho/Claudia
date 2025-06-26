# src/claudia/main.py
"""
Claudia智能四足机器人系统主程序

系统入口点，负责初始化所有组件和启动主要服务。
"""

import sys
import asyncio
import signal
from pathlib import Path
from typing import Optional

from .common.config import Config
from .common.logger import get_logger, setup_project_logging


class ClaudiaSystem:
    """Claudia机器人系统主类"""
    
    def __init__(self, config_file: Optional[str] = None):
        """初始化系统"""
        # 设置项目日志
        setup_project_logging(level="INFO")
        self.logger = get_logger(__name__)
        
        # 加载配置
        self.config = Config(config_file)
        self.logger.info("Claudia系统初始化完成")
        
        # 初始化组件
        self.components = {}
        self.running = False    
    async def initialize_components(self):
        """初始化所有系统组件"""
        self.logger.info("开始初始化系统组件...")
        
        try:
            # TODO: 根据TaskMaster任务逐步添加组件初始化
            # Task 1-3: 环境和SDK初始化
            # Task 4: 传感器系统初始化
            # Task 6: LED控制系统初始化
            # Task 7-9: AI组件初始化
            # Task 10-11: LLM和动作映射初始化
            
            self.logger.info("所有组件初始化完成")
            
        except Exception as e:
            self.logger.error(f"组件初始化失败: {e}")
            raise
    
    async def start(self):
        """启动系统"""
        self.logger.info("启动Claudia机器人系统...")
        
        await self.initialize_components()
        self.running = True
        
        # 主运行循环
        while self.running:
            try:
                # TODO: 主要系统循环逻辑
                await asyncio.sleep(0.1)
                
            except KeyboardInterrupt:
                self.logger.info("收到中断信号，开始关闭系统...")
                break
            except Exception as e:
                self.logger.error(f"系统运行错误: {e}")
                await asyncio.sleep(1.0)
        
        await self.shutdown()
    
    async def shutdown(self):
        """关闭系统"""
        self.logger.info("关闭Claudia机器人系统...")
        self.running = False
        
        # TODO: 清理所有组件
        
        self.logger.info("系统已关闭")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Claudia智能四足机器人系统")
    parser.add_argument("--config", "-c", help="配置文件路径")
    parser.add_argument("--log-level", default="INFO", 
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                       help="日志级别")
    
    args = parser.parse_args()
    
    try:
        # 创建并启动系统
        system = ClaudiaSystem(args.config)
        asyncio.run(system.start())
        
    except KeyboardInterrupt:
        print("系统被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"系统启动失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()