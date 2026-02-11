#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Commander - 生产环境交互式命令器
使用修复后的LLM大脑架构进行实机测试
"""

import asyncio
import time
import sys
import os
from datetime import datetime
from typing import Optional

# 添加项目路径
sys.path.append('/home/m1ng/claudia')
sys.path.append('/home/m1ng/claudia/src')

from src.claudia.brain.production_brain import ProductionBrain, BrainOutput


class ProductionCommander:
    """生产环境命令器"""
    
    def __init__(self, use_real_hardware: bool = False):
        """初始化命令器
        
        Args:
            use_real_hardware: 是否使用真实硬件（默认False为模拟模式）
        """
        self.brain = ProductionBrain(use_real_hardware=use_real_hardware)
        self.running = True
        self.command_history = []
        self.session_start = datetime.now()
        
    def print_header(self):
        """打印界面头部"""
        print("\n" + "="*60)
        print("🤖 Claudia Production Commander - LLM大脑实机测试")
        print("="*60)
        print(f"⚙️  模式: {'真实硬件' if self.brain.use_real_hardware else '模拟执行'}")
        print(f"🧠 模型: {self.brain.model_7b}")
        print(f"⏰ 会话开始: {self.session_start.strftime('%Y-%m-%d %H:%M:%S')}")
        print("-"*60)
        print("💡 提示: 输入日语/中文/英文命令，输入 /help 查看帮助")
        print("💡 示例: お手, 坐下, dance, 座ってから挨拶")
        print("-"*60 + "\n")
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "="*40)
        print("📖 帮助信息")
        print("="*40)
        print("\n基本命令:")
        print("  お手, おすわり, タッテ, ハート, ダンス")
        print("  坐下, 站立, 比心, 握手, 跳舞")
        print("  sit, stand, heart, dance, hello")
        print("\n复杂命令:")
        print("  座ってから挨拶 - 坐下然后打招呼")
        print("  運動して - 做运动")
        print("  表演一套 - 表演一套动作")
        print("\n系统命令:")
        print("  /help    - 显示帮助")
        print("  /stats   - 显示统计")
        print("  /history - 显示历史")
        print("  /clear   - 清屏")
        print("  /exit    - 退出")
        print("="*40 + "\n")
    
    def print_stats(self):
        """打印统计信息"""
        stats = self.brain.get_statistics()
        print("\n" + "="*40)
        print("📊 统计信息")
        print("="*40)
        print(f"🧠 模型: {stats['model']}")
        print(f"⚡ 缓存大小: {stats['cache_size']} 条")
        print(f"🤖 硬件模式: {'真实' if stats['hardware_mode'] else '模拟'}")
        print(f"🔌 SportClient: {'已连接' if stats['sport_client'] else '未连接'}")
        print(f"📝 历史命令: {len(self.command_history)} 条")
        runtime = datetime.now() - self.session_start
        print(f"⏱️ 运行时间: {runtime.total_seconds():.0f} 秒")
        print("="*40 + "\n")
    
    def print_history(self):
        """打印历史记录"""
        print("\n" + "="*40)
        print("📜 命令历史")
        print("="*40)
        if not self.command_history:
            print("(暂无历史记录)")
        else:
            for i, (timestamp, cmd, response) in enumerate(self.command_history[-10:], 1):
                print(f"{i}. [{timestamp}] {cmd}")
                print(f"   → {response}")
        print("="*40 + "\n")
    
    async def process_command(self, command: str):
        """处理单个命令"""
        if command.startswith("/"):
            # 系统命令
            if command == "/help":
                self.print_help()
            elif command == "/stats":
                self.print_stats()
            elif command == "/history":
                self.print_history()
            elif command == "/clear":
                os.system('clear' if os.name == 'posix' else 'cls')
                self.print_header()
            elif command == "/exit":
                self.running = False
                print("\n👋 再见！感谢使用Claudia Production Commander\n")
            else:
                print(f"❌ 未知命令: {command}")
        else:
            # 用户指令
            print(f"\n🎯 处理指令: '{command}'")
            print("-"*40)
            
            # 调用大脑处理
            start_time = time.time()
            brain_output = await self.brain.process_command(command)
            process_time = (time.time() - start_time) * 1000
            
            # 显示结果
            print(f"💬 回复: {brain_output.response}")
            
            if brain_output.api_code:
                print(f"🔧 API: {brain_output.api_code}")
            
            if brain_output.sequence:
                print(f"📋 序列: {brain_output.sequence}")
            
            print(f"⏱️ 处理时间: {process_time:.0f}ms")
            
            # 执行动作
            if brain_output.api_code or brain_output.sequence:
                print("-"*40)
                print("🚀 执行动作...")
                result = await self.brain.execute_action(brain_output)
                if result is True:
                    print("✅ 执行成功")
                elif result == "unknown":
                    print("⚠️ 动作超时（机器人可达，可能仍在执行）")
                else:
                    print("❌ 执行失败")
            
            # 记录历史
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.command_history.append((
                timestamp,
                command,
                brain_output.response
            ))
            
            print("-"*40 + "\n")
    
    async def run(self):
        """运行主循环"""
        self.print_header()
        
        # 预热模型
        print("🔄 预热模型中...")
        await self.brain.process_command("hello")
        print("✅ 模型就绪\n")
        
        # 主循环
        while self.running:
            try:
                # 获取用户输入
                command = input("くら> ").strip()
                
                if command:
                    await self.process_command(command)
                    
            except KeyboardInterrupt:
                print("\n\n⚠️ 检测到Ctrl+C，正在退出...")
                self.running = False
            except Exception as e:
                print(f"\n❌ 错误: {e}\n")
        
        # 清理
        print("\n🧹 清理资源...")
        print("✅ 会话结束\n")


async def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Claudia Production Commander")
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="使用真实硬件模式（默认为模拟模式）"
    )
    
    args = parser.parse_args()
    
    # 创建并运行命令器
    commander = ProductionCommander(use_real_hardware=args.hardware)
    await commander.run()


if __name__ == "__main__":
    # 设置事件循环
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    
    # 运行主程序
    asyncio.run(main())
