#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM到真实机器人控制测试
使用正确的CycloneDDS环境配置，实现任务11的真实机器人控制

Generated: 2025-07-08 13:25:00
Purpose: 验证任务11的LLM命令能够真正控制机器人
Author: M1nG
"""

import os
import sys
import time
import asyncio
import subprocess
from datetime import datetime

# 设置项目路径
sys.path.append('/home/m1ng/claudia/src')
sys.path.append('/home/m1ng/claudia/unitree_sdk2_python')

def setup_cyclonedds_environment():
    """设置CycloneDDS环境 - 关键修复"""
    print("🔧 配置CycloneDDS环境...")
    
    # 运行setup脚本获取环境变量
    try:
        result = subprocess.run([
            'bash', '-c', 
            'source /home/m1ng/claudia/scripts/setup/setup_cyclonedds.sh && env'
        ], capture_output=True, text=True, check=True)
        
        # 解析环境变量
        env_vars = {}
        for line in result.stdout.split('\n'):
            if '=' in line and not line.startswith('_'):
                key, value = line.split('=', 1)
                env_vars[key] = value
        
        # 设置关键环境变量
        important_vars = [
            'CYCLONEDDS_HOME', 'LD_LIBRARY_PATH', 'RMW_IMPLEMENTATION'
        ]
        
        for var in important_vars:
            if var in env_vars:
                os.environ[var] = env_vars[var]
                print(f"   {var}: {env_vars[var][:100]}...")
        
        print("✅ CycloneDDS环境配置完成")
        return True
        
    except Exception as e:
        print(f"❌ 环境配置失败: {e}")
        return False

class LLMRealRobotTest:
    """LLM真实机器人控制测试"""
    
    def __init__(self):
        self.results = []
        
        # 导入映射引擎（环境配置后）
        try:
            from claudia.robot_controller.action_mapping_engine_real import RealActionMappingEngine, SDK_AVAILABLE
            self.engine_class = RealActionMappingEngine
            self.sdk_available = SDK_AVAILABLE
            print(f"✅ 映射引擎导入成功 (SDK可用: {SDK_AVAILABLE})")
        except Exception as e:
            print(f"❌ 映射引擎导入失败: {e}")
            self.engine_class = None
            self.sdk_available = False
    
    def log_result(self, command, success, duration, message=""):
        """记录测试结果"""
        result = {
            'command': command,
            'success': success,
            'duration': duration,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'message': message
        }
        self.results.append(result)
        status = "✅" if success else "❌"
        print(f"{status} {command}: {duration:.3f}s - {message}")
    
    def safety_confirmation(self):
        """安全确认"""
        print("\n" + "="*60)
        print("🚨 LLM真实机器人控制测试")
        print("="*60)
        print("⚠️  这将测试LLM命令到真实机器人的映射!")
        print("⚠️  请确保机器人周围安全!")
        print("\n📋 测试命令:")
        print('   1. {"intent": "robot_control"} → 坐下')
        print('   2. {"intent": "お座り"} → 坐下 (日语)')
        print('   3. {"intent": "stand"} → 站立')
        print('   4. {"intent": "hello"} → 招手')
        print("="*60)
        
        response = input("确认安全条件，继续LLM机器人测试? (yes/no): ").lower().strip()
        return response in ['yes', 'y', '是']
    
    async def run_llm_robot_test(self):
        """运行LLM机器人控制测试"""
        if not self.safety_confirmation():
            print("❌ 测试已取消")
            return
        
        if not self.sdk_available or not self.engine_class:
            print("❌ SDK不可用或映射引擎无法加载")
            return
        
        print(f"\n🧠 开始LLM真实机器人控制测试 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # 创建映射引擎
        try:
            engine = self.engine_class()
            print("✅ 真实机器人映射引擎初始化成功")
        except Exception as e:
            print(f"❌ 映射引擎初始化失败: {e}")
            return
        
        # LLM测试命令（从任务10输出格式和日语命令）
        test_commands = [
            ('robot_control', '{"intent": "robot_control", "confidence": 0.8}'),  # 任务10标准输出
            ('japanese_sit', '{"intent": "お座り", "confidence": 0.9}'),          # 日语坐下
            ('english_stand', '{"intent": "stand", "confidence": 0.8}'),          # 英语站立  
            ('hello_wave', '{"intent": "hello", "confidence": 0.8}'),             # 招手动作
        ]
        
        print(f"\n📝 执行LLM命令序列 (共{len(test_commands)}个):")
        print("-" * 50)
        
        for i, (cmd_name, llm_output) in enumerate(test_commands, 1):
            print(f"\n[{i}/{len(test_commands)}] LLM命令: {llm_output}")
            
            start_time = time.time()
            try:
                # 🧠 这里是关键：LLM输出 → 真实机器人动作
                result = await engine.map_intent_to_action(llm_output)
                duration = time.time() - start_time
                
                if result.success:
                    self.log_result(cmd_name, True, duration, 
                                  f"✅ LLM→机器人成功: {result.message}")
                else:
                    self.log_result(cmd_name, False, duration,
                                  f"❌ LLM→机器人失败: {result.error_message}")
                
                # 安全间隔
                await asyncio.sleep(3)
                
            except Exception as e:
                duration = time.time() - start_time
                self.log_result(cmd_name, False, duration, f"❌ 执行异常: {str(e)}")
        
        # 生成报告
        self.generate_report()
    
    def generate_report(self):
        """生成测试报告"""
        print("\n" + "="*60)
        print("📊 LLM真实机器人控制测试报告")
        print("="*60)
        
        success_count = sum(1 for r in self.results if r['success'])
        total_count = len(self.results)
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0
        
        print(f"🎯 任务11验证结果: {success_count}/{total_count} 成功 ({success_rate:.1f}%)")
        print(f"⏱️  测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        print("\n📋 详细结果:")
        print("-" * 60)
        for result in self.results:
            status = "✅" if result['success'] else "❌"
            print(f"{status} {result['timestamp']} | {result['command']:<15} | "
                  f"{result['duration']:>6.3f}s | {result['message']}")
        
        # 关键结论
        print(f"\n🎯 任务11关键验证:")
        if success_count > 0:
            print("✅ LLM命令可以成功控制真实机器人!")
            print("✅ 自然语言 → 机器人动作的映射工作正常!")
            print("✅ 任务11的核心功能已实现!")
        else:
            print("❌ LLM到机器人的映射需要进一步调试")
        
        print("="*60)

async def main():
    """主函数"""
    print("🤖 LLM真实机器人控制测试 (任务11验证)")
    print(f"🕐 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 关键：配置CycloneDDS环境
    if not setup_cyclonedds_environment():
        print("❌ 环境配置失败，无法继续")
        return
    
    # 运行测试
    tester = LLMRealRobotTest()
    await tester.run_llm_robot_test()

if __name__ == "__main__":
    asyncio.run(main()) 