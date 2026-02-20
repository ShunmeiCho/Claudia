#!/usr/bin/env python3
"""
基于官方示例风格的控制命令测试
按照unitree_sdk2_python官方示例的方式进行测试
"""

import time
import sys
import os
from datetime import datetime

# 设置环境变量
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

# 添加SDK路径
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    print("✅ 成功导入unitree_sdk2py模块")
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    sys.exit(1)

def test_official_style():
    """按照官方示例风格测试SportClient"""
    print(f"🤖 官方风格SportClient测试 - {datetime.now().strftime('%H:%M:%S')}")
    
    # 安全提示
    print("\n⚠️ 安全警告：请确保机器人周围没有障碍物")
    print("本测试将按照官方示例方式进行")
    response = input("按回车键继续...")
    
    try:
        # 按照官方示例初始化DDS通道
        print("📡 初始化DDS通道工厂 (eth0)...")
        ChannelFactoryInitialize(0, "eth0")
        
        # 创建SportClient（按官方示例方式）
        print("🏃 创建SportClient...")
        sport_client = SportClient()
        sport_client.SetTimeout(10.0)
        
        # 按照官方示例，直接调用Init()不检查返回值
        print("🔌 初始化SportClient...")
        sport_client.Init()
        print("✅ SportClient初始化完成（按官方示例风格）")
        
        # 测试序列
        test_commands = [
            ("BalanceStand", lambda: sport_client.BalanceStand(), "平衡站立"),
            ("StandUp", lambda: sport_client.StandUp(), "站立"),
            ("StandDown", lambda: sport_client.StandDown(), "趴下"),
            ("RecoveryStand", lambda: sport_client.RecoveryStand(), "恢复站立"),
        ]
        
        print(f"\n📝 开始测试序列 (共{len(test_commands)}个命令):")
        print("-" * 50)
        
        for i, (cmd_name, cmd_func, description) in enumerate(test_commands, 1):
            print(f"\n[{i}/{len(test_commands)}] 执行 {cmd_name}() - {description}")
            
            try:
                result = cmd_func()
                print(f"✅ {cmd_name}: 命令执行完成, 返回值: {result}")
                
                # 等待动作完成
                if cmd_name in ["StandUp", "StandDown", "RecoveryStand"]:
                    print(f"   ⏳ 等待{description}完成...")
                    time.sleep(5)
                else:
                    time.sleep(2)
                    
            except Exception as e:
                print(f"❌ {cmd_name}: 执行异常: {e}")
                
        print("\n🏁 测试序列完成")
        
        # 最终状态检查
        print("\n📊 测试总结:")
        print("✅ 基于官方示例风格的测试完成")
        print("🤖 机器人应该处于站立状态")
        
        return True
        
    except Exception as e:
        print(f"❌ 测试过程异常: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("🚀 Unitree Go2 官方风格控制测试")
    print(f"🕐 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
    
    success = test_official_style()
    
    print("\n" + "=" * 60)
    if success:
        print("✅ 官方风格测试成功完成")
    else:
        print("❌ 官方风格测试失败")
    print("=" * 60)

if __name__ == "__main__":
    main() 