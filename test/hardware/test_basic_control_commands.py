#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unitree Go2 基础控制命令测试
Generated: 2024-12-26 17:56:16
Safety: 包含安全确认提示，避免机器人意外跌倒
Updated: 2025-06-26 18:17:00 - 修正为官方示例标准初始化方式
"""

import time
import os
import sys
from datetime import datetime

# 添加SDK路径
sys.path.append('/home/m1ng/claudia/unitree_sdk2_python')

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    print("✅ 成功导入所有必需的模块")
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    print("Please ensure unitree_sdk2py is properly installed")
    sys.exit(1)

class BasicControlTest:
    def __init__(self):
        self.results = []
        self.start_time = None
        self.network_interface = "eth0"  # Go2默认网络接口
        
    def log_result(self, command, success, duration, notes=""):
        """记录测试结果"""
        result = {
            'command': command,
            'success': success,
            'duration': duration,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'notes': notes
        }
        self.results.append(result)
        status = "✅" if success else "❌"
        print(f"{status} {command}: {duration:.3f}s - {notes}")
        
    def safety_confirmation(self):
        """安全确认提示"""
        print("\n" + "="*60)
        print("🚨 安全提示 - Unitree Go2 基础控制命令测试")
        print("="*60)
        print("⚠️  请确保:")
        print("   1. 机器人周围有足够的安全空间")
        print("   2. 没有人员或障碍物在机器人活动范围内")
        print("   3. 您已准备好在紧急情况下按下遥控器的急停按钮")
        print("   4. 机器人电池电量充足")
        print("\n📋 测试将执行以下安全序列:")
        print("   1. Sit() - 安全坐下状态")
        print("   2. StandUp() - 站立动作")
        print("   3. StandDown() - 趴下动作") 
        print("   4. RecoveryStand() - 恢复站立")
        print("   5. 保持站立状态 - 便于后续操作")
        print("\n" + "="*60)
        
        response = input("确认安全条件已满足，继续测试? (yes/no): ").lower().strip()
        if response not in ['yes', 'y', '是']:
            print("❌ 测试已取消")
            return False
        return True
        
    def run_test(self):
        """执行基础控制命令测试（按官方示例方式）"""
        if not self.safety_confirmation():
            return
            
        print(f"\n🤖 开始基础控制命令测试 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"📡 网络接口: {self.network_interface}")
        
        try:
            # 按照官方示例初始化DDS通道
            print("📡 初始化DDS通道工厂...")
            ChannelFactoryInitialize(0, self.network_interface)
            print("✅ DDS通道工厂初始化成功")
            
            # 创建SportClient（按官方示例方式）
            print("🔌 创建SportClient...")
            client = SportClient()
            client.SetTimeout(10.0)
            
            # 按照官方示例，直接调用Init()不检查返回值
            print("🚀 初始化SportClient...")
            client.Init()
            print("✅ SportClient初始化完成（按官方示例风格）")
            
            # 等待连接稳定
            time.sleep(2)
            
            # 测试序列 - 安全优化版本
            test_commands = [
                ("Sit", lambda: client.Sit(), "安全坐下状态"),
                ("StandUp", lambda: client.StandUp(), "站立动作"),
                ("StandDown", lambda: client.StandDown(), "趴下动作"),
                ("RecoveryStand", lambda: client.RecoveryStand(), "恢复站立"),
            ]
            
            print(f"\n📝 执行测试序列 (共{len(test_commands)}个命令):")
            print("-" * 50)
            
            for i, (cmd_name, cmd_func, description) in enumerate(test_commands, 1):
                print(f"\n[{i}/{len(test_commands)}] 执行 {cmd_name}() - {description}")
                
                start_time = time.time()
                try:
                    result = cmd_func()
                    duration = time.time() - start_time
                    
                    if result == 0:  # 0表示成功
                        self.log_result(cmd_name, True, duration, f"{description} - 命令成功发送")
                        
                        # 等待动作完成
                        if cmd_name in ["StandUp", "StandDown", "RecoveryStand"]:
                            print(f"   ⏳ 等待{description}完成...")
                            time.sleep(5)  # 给机器人足够时间完成动作
                        else:
                            time.sleep(2)  # 其他命令等待较短时间
                            
                    else:
                        self.log_result(cmd_name, False, duration, f"命令失败，返回码: {result}")
                        
                except Exception as e:
                    duration = time.time() - start_time
                    self.log_result(cmd_name, False, duration, f"执行异常: {str(e)}")
                    
            print("\n🏁 测试完成，机器人保持站立状态便于后续操作")
            
        except Exception as e:
            print(f"❌ 测试过程异常: {e}")
            print("请确认机器人网络连接和SDK配置正确")
            import traceback
            traceback.print_exc()
            return
            
        # 生成测试报告
        self.generate_report()
        
    def generate_report(self):
        """生成详细测试报告"""
        print("\n" + "="*60)
        print("📊 基础控制命令测试报告")
        print("="*60)
        
        success_count = sum(1 for r in self.results if r['success'])
        total_count = len(self.results)
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0
        
        print(f"📈 总体结果: {success_count}/{total_count} 成功 ({success_rate:.1f}%)")
        print(f"⏱️  测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        print("\n📋 详细结果:")
        print("-" * 60)
        for result in self.results:
            status = "✅" if result['success'] else "❌"
            print(f"{status} {result['timestamp']} | {result['command']:<12} | "
                  f"{result['duration']:>6.3f}s | {result['notes']}")
        
        # 性能统计
        if self.results:
            durations = [r['duration'] for r in self.results if r['success']]
            if durations:
                avg_duration = sum(durations) / len(durations)
                max_duration = max(durations)
                min_duration = min(durations)
                
                print(f"\n⚡ 性能指标:")
                print(f"   平均响应时间: {avg_duration:.3f}s")
                print(f"   最快响应时间: {min_duration:.3f}s") 
                print(f"   最慢响应时间: {max_duration:.3f}s")
        
        print("\n✅ 基础控制功能验证完成")
        print("🤖 机器人现在处于站立状态，准备进行后续操作")
        print("="*60)

def main():
    """主函数"""
    # 设置正确的环境变量（修正）
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    
    print("🚀 Unitree Go2 基础控制命令测试（官方示例风格）")
    print(f"🕐 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("🔧 初始化方式: 官方标准 (ChannelFactoryInitialize + 直接Init)")
    
    tester = BasicControlTest()
    tester.run_test()

if __name__ == "__main__":
    main() 