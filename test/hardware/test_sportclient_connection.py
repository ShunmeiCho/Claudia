#!/usr/bin/env python3
"""
SportClient连接测试
用于诊断机器人连接问题
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

def test_sportclient_connection():
    """测试SportClient连接"""
    print(f"🔍 SportClient连接诊断 - {datetime.now().strftime('%H:%M:%S')}")
    
    try:
        # 尝试不同的网络配置
        network_configs = [
            ("eth0", "以太网接口"),
            ("", "默认接口"),
        ]
        
        for interface, description in network_configs:
            print(f"\n📡 尝试网络接口: {interface} ({description})")
            
            try:
                # 初始化DDS通道
                if interface:
                    print(f"   初始化DDS通道工厂: {interface}")
                    ChannelFactoryInitialize(0, interface)
                else:
                    print("   使用默认DDS通道配置")
                    
                # 创建SportClient
                print("   创建SportClient...")
                client = SportClient()
                client.SetTimeout(5.0)  # 5秒超时
                
                print("   初始化SportClient...")
                result = client.Init()
                
                if result is not None:
                    print(f"✅ SportClient初始化成功! 接口: {interface}")
                    
                    # 尝试一个简单的状态查询
                    print("   测试简单命令...")
                    time.sleep(1)
                    print("✅ 连接测试成功!")
                    return True
                else:
                    print(f"❌ SportClient初始化失败 (返回None)")
                    
            except Exception as e:
                print(f"❌ 接口 {interface} 连接失败: {e}")
                continue
                
        print("❌ 所有网络接口测试都失败")
        return False
        
    except Exception as e:
        print(f"❌ 连接测试异常: {e}")
        return False

def main():
    print("🤖 Unitree Go2 SportClient连接诊断")
    print("=" * 50)
    
    success = test_sportclient_connection()
    
    print("\n" + "=" * 50)
    if success:
        print("✅ 连接诊断成功 - SportClient可以正常工作")
    else:
        print("❌ 连接诊断失败 - 需要检查机器人状态和网络配置")
    print("=" * 50)

if __name__ == "__main__":
    main() 