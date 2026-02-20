#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态感知LLM机器人控制测试
避免重复状态命令冲突，验证任务11的状态管理

Generated: 2025-07-08 13:30:00
Purpose: 解决状态冲突问题，优化LLM命令序列
Author: M1nG
"""

import os
import sys
import time
import asyncio
import subprocess
from datetime import datetime

# 设置项目路径
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'src'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

def setup_cyclonedds_environment():
    """设置CycloneDDS环境"""
    print("🔧 配置CycloneDDS环境...")
    
    try:
        result = subprocess.run([
            'bash', '-c', 
            'source ~/claudia/scripts/setup/setup_cyclonedds.sh && env'
        ], capture_output=True, text=True, check=True)
        
        env_vars = {}
        for line in result.stdout.split('\n'):
            if '=' in line and not line.startswith('_'):
                key, value = line.split('=', 1)
                env_vars[key] = value
        
        important_vars = ['CYCLONEDDS_HOME', 'LD_LIBRARY_PATH', 'RMW_IMPLEMENTATION']
        
        for var in important_vars:
            if var in env_vars:
                os.environ[var] = env_vars[var]
                print(f"   {var}: {env_vars[var][:100]}...")
        
        print("✅ CycloneDDS环境配置完成")
        return True
        
    except Exception as e:
        print(f"❌ 环境配置失败: {e}")
        return False

class StateAwareLLMTest:
    """状态感知的LLM机器人测试"""
    
    def __init__(self):
        self.results = []
        self.current_robot_state = "unknown"  # 跟踪机器人状态
        
        try:
            from claudia.robot_controller.action_mapping_engine_real import RealActionMappingEngine, SDK_AVAILABLE
            self.engine_class = RealActionMappingEngine
            self.sdk_available = SDK_AVAILABLE
            print(f"✅ 映射引擎导入成功 (SDK可用: {SDK_AVAILABLE})")
        except Exception as e:
            print(f"❌ 映射引擎导入失败: {e}")
            self.engine_class = None
            self.sdk_available = False
    
    def log_result(self, command, success, duration, message="", robot_state=""):
        """记录测试结果和状态"""
        result = {
            'command': command,
            'success': success,
            'duration': duration,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'message': message,
            'robot_state': robot_state
        }
        self.results.append(result)
        status = "✅" if success else "❌"
        state_info = f" [状态: {robot_state}]" if robot_state else ""
        print(f"{status} {command}: {duration:.3f}s - {message}{state_info}")
    
    def update_robot_state(self, action_api, success):
        """根据执行结果更新机器人状态"""
        if not success:
            return
            
        state_mapping = {
            1009: "sitting",      # Sit
            1004: "standing",     # StandUp
            1005: "lying",        # StandDown
            1016: "standing",     # Hello (保持站立)
            1017: "standing",     # Stretch (保持站立)
            1006: "standing",     # RecoveryStand
        }
        
        if action_api in state_mapping:
            old_state = self.current_robot_state
            self.current_robot_state = state_mapping[action_api]
            print(f"🤖 机器人状态更新: {old_state} → {self.current_robot_state}")
    
    def predict_command_conflict(self, intent, target_api):
        """预测命令是否会与当前状态冲突"""
        conflict_rules = {
            # API代码: [会冲突的当前状态列表]
            1009: ["sitting"],       # Sit命令在已坐下时冲突
            1004: ["standing"],      # StandUp命令在已站立时可能冲突
            1005: ["lying"],         # StandDown命令在已趴下时冲突
        }
        
        if target_api in conflict_rules:
            if self.current_robot_state in conflict_rules[target_api]:
                return True, f"状态冲突: 机器人已处于{self.current_robot_state}状态"
        
        return False, ""
    
    def safety_confirmation(self):
        """安全确认"""
        print("\n" + "="*60)
        print("🚨 状态感知LLM机器人控制测试")
        print("="*60)
        print("🧠 这将测试状态冲突检测和解决方案!")
        print("⚠️  请确保机器人周围安全!")
        print("\n📋 优化的测试序列 (避免状态冲突):")
        print('   1. {"intent": "robot_control"} → 坐下')
        print('   2. {"intent": "stand"} → 站立 (不同状态)')
        print('   3. {"intent": "hello"} → 招手 (保持站立)')
        print('   4. {"intent": "お座り"} → 坐下 (日语)')
        print('   5. 验证状态冲突检测')
        print("="*60)
        
        response = input("确认继续状态感知测试? (yes/no): ").lower().strip()
        return response in ['yes', 'y', '是']
    
    async def run_state_aware_test(self):
        """运行状态感知测试"""
        if not self.safety_confirmation():
            print("❌ 测试已取消")
            return
        
        if not self.sdk_available or not self.engine_class:
            print("❌ SDK不可用或映射引擎无法加载")
            return
        
        print(f"\n🧠 开始状态感知LLM机器人控制测试 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        engine = self.engine_class()
        print("✅ 真实机器人映射引擎初始化成功")
        
        # 🎯 优化的测试序列 - 避免连续相同状态命令
        test_commands = [
            ('robot_control', '{"intent": "robot_control", "confidence": 0.8}', 1009),  # 坐下
            ('english_stand', '{"intent": "stand", "confidence": 0.8}', 1004),          # 站立
            ('hello_wave', '{"intent": "hello", "confidence": 0.8}', 1016),             # 招手
            ('japanese_sit', '{"intent": "お座り", "confidence": 0.9}', 1009),          # 日语坐下
            ('conflict_test', '{"intent": "robot_control", "confidence": 0.8}', 1009),  # 故意重复测试
        ]
        
        print(f"\n📝 执行状态感知测试序列 (共{len(test_commands)}个):")
        print("-" * 60)
        
        for i, (cmd_name, llm_output, expected_api) in enumerate(test_commands, 1):
            print(f"\n[{i}/{len(test_commands)}] 当前状态: {self.current_robot_state}")
            print(f"LLM命令: {llm_output}")
            
            # 🧠 预测冲突
            will_conflict, conflict_reason = self.predict_command_conflict(llm_output, expected_api)
            if will_conflict:
                print(f"⚠️  预测冲突: {conflict_reason}")
            
            start_time = time.time()
            try:
                result = await engine.map_intent_to_action(llm_output)
                duration = time.time() - start_time
                
                # 🤖 更新状态
                if result.success:
                    self.update_robot_state(result.action_code, True)
                    self.log_result(cmd_name, True, duration, 
                                  f"✅ LLM→机器人成功: {result.message}", 
                                  self.current_robot_state)
                else:
                    conflict_detected = "状态冲突" in str(result.error_message) or result.robot_response == -1
                    conflict_msg = " (检测到状态冲突)" if conflict_detected else ""
                    
                    self.log_result(cmd_name, False, duration,
                                  f"❌ LLM→机器人失败: {result.error_message}{conflict_msg}",
                                  self.current_robot_state)
                
                # 验证预测准确性
                if will_conflict and not result.success:
                    print("🎯 状态冲突预测准确!")
                elif will_conflict and result.success:
                    print("🤔 预测冲突但执行成功，可能状态判断有误")
                
                # 安全间隔
                await asyncio.sleep(2)
                
            except Exception as e:
                duration = time.time() - start_time
                self.log_result(cmd_name, False, duration, f"❌ 执行异常: {str(e)}")
        
        self.generate_report()
    
    def generate_report(self):
        """生成详细报告"""
        print("\n" + "="*60)
        print("📊 状态感知LLM机器人控制测试报告")
        print("="*60)
        
        success_count = sum(1 for r in self.results if r['success'])
        total_count = len(self.results)
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0
        
        print(f"🎯 总体结果: {success_count}/{total_count} 成功 ({success_rate:.1f}%)")
        print(f"⏱️  测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        print(f"\n🤖 最终机器人状态: {self.current_robot_state}")
        
        print("\n📋 详细结果:")
        print("-" * 60)
        for result in self.results:
            status = "✅" if result['success'] else "❌"
            state_info = f" → {result['robot_state']}" if result['robot_state'] else ""
            print(f"{status} {result['timestamp']} | {result['command']:<15} | "
                  f"{result['duration']:>6.3f}s | {result['message']}{state_info}")
        
        # 状态冲突分析
        conflict_failures = [r for r in self.results if not r['success'] and "状态冲突" in r['message']]
        
        print(f"\n🎯 状态管理分析:")
        print(f"   状态冲突失败: {len(conflict_failures)}次")
        print(f"   状态跟踪准确性: 基于执行结果自动更新")
        
        if len(conflict_failures) > 0:
            print("✅ 成功检测并处理状态冲突!")
            print("✅ 状态感知机制工作正常!")
        
        print("="*60)

async def main():
    """主函数"""
    print("🤖 状态感知LLM机器人控制测试")
    print(f"🕐 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    if not setup_cyclonedds_environment():
        print("❌ 环境配置失败，无法继续")
        return
    
    tester = StateAwareLLMTest()
    await tester.run_state_aware_test()

if __name__ == "__main__":
    asyncio.run(main()) 