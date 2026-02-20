#!/usr/bin/env python3
"""
Unitree Dance命令返回码测试
验证Dance1/Dance2的返回码含义

测试目的：确定3104等返回码是否为正常完成状态码
"""

import sys
import time
import os

# 设置环境和路径
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

def test_dance_return_codes():
    """专门测试舞蹈动作的返回码含义"""
    
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        
        print("🕺 Dance返回码专项测试")
        print("=" * 50)
        
        # 初始化
        ChannelFactoryInitialize(0, "eth0")
        client = SportClient()
        client.SetTimeout(10.0)
        client.Init()
        
        print("✅ SportClient初始化成功\n")
        
        # 测试多种命令的返回码
        test_commands = [
            ("Sit", client.Sit, "坐下"),
            ("StandUp", client.StandUp, "站立"),
            ("Hello", client.Hello, "招手"),
            ("Dance1", client.Dance1, "舞蹈1"),
            ("Dance2", client.Dance2, "舞蹈2"),
            ("Stretch", client.Stretch, "伸展"),
        ]
        
        return_codes = {}
        
        for cmd_name, cmd_func, description in test_commands:
            print(f"🧪 测试 {cmd_name}() - {description}")
            
            try:
                start_time = time.time()
                result = cmd_func()
                duration = time.time() - start_time
                
                return_codes[cmd_name] = result
                
                print(f"   返回码: {result}")
                print(f"   执行时间: {duration:.3f}秒")
                
                # 根据命令类型等待不同时间
                if "Dance" in cmd_name:
                    print(f"   等待舞蹈动作完成...")
                    time.sleep(8)  # 舞蹈动作需要更长时间
                elif cmd_name in ["StandUp", "Hello", "Stretch"]:
                    print(f"   等待动作完成...")
                    time.sleep(4)
                else:
                    time.sleep(2)
                    
                print(f"   状态: {'✅ 传统成功' if result == 0 else '🔍 需要验证'}\n")
                
            except Exception as e:
                print(f"   ❌ 执行异常: {e}\n")
                return_codes[cmd_name] = f"异常: {e}"
        
        # 分析返回码模式
        print("📊 返回码分析报告")
        print("=" * 50)
        
        print(f"{'命令':<12} {'返回码':<8} {'可能含义'}")
        print("-" * 40)
        
        for cmd_name, code in return_codes.items():
            if isinstance(code, int):
                if code == 0:
                    meaning = "标准成功"
                elif code in [3104, 3105, 3106]:  # 常见的完成状态码
                    meaning = "可能是完成状态码"
                elif code > 3000:
                    meaning = "高值状态码（可能正常）"
                else:
                    meaning = "需要进一步验证"
                    
                print(f"{cmd_name:<12} {code:<8} {meaning}")
            else:
                print(f"{cmd_name:<12} {'ERROR':<8} {code}")
        
        # 提出建议
        print("\n💡 建议修改的成功判断逻辑:")
        successful_codes = [0]  # 默认成功
        
        # 分析哪些高返回码可能是成功状态
        for cmd_name, code in return_codes.items():
            if isinstance(code, int) and code > 0:
                if "Dance" in cmd_name and code == 3104:
                    successful_codes.append(3104)
                    print(f"   - 3104: Dance1完成状态码（基于用户观察）")
                elif code in [3105, 3106, 3107]:  # 其他可能的完成码
                    print(f"   - {code}: {cmd_name}可能的完成状态码")
        
        print(f"\n🔧 建议的成功返回码列表: {sorted(set(successful_codes))}")
        
        return return_codes
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    print("⚠️ 请确认机器人已连接并处于安全环境")
    input("按Enter键开始测试...")
    
    results = test_dance_return_codes()
    
    if results:
        print("\n🎯 测试完成！请观察机器人实际动作执行情况")
        print("如果Dance1执行了正确动作但返回码非0，则说明该返回码是正常完成状态") 