#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整测试Unitree Go2 SDK所有声明的动作方法
确认哪些动作在Go2硬件上真正可用
"""

import os
import sys
import time
import json

# 添加路径
sys.path.append('/home/m1ng/claudia')
sys.path.append('/home/m1ng/claudia/unitree_sdk2_python')

# 设置环境变量
os.environ['CYCLONEDDS_HOME'] = '/home/m1ng/claudia/cyclonedds/install'
ld_path = os.environ.get('LD_LIBRARY_PATH', '')
cyclone_lib = '/home/m1ng/claudia/cyclonedds/install/lib'
unitree_lib = '/home/m1ng/claudia/cyclonedds_ws/install/unitree_sdk2/lib'
if cyclone_lib not in ld_path:
    os.environ['LD_LIBRARY_PATH'] = f"{cyclone_lib}:{unitree_lib}:{ld_path}"
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = '''<CycloneDDS><Domain><General><Interfaces>
                        <NetworkInterface name="eth0" priority="default" multicast="default" />
                    </Interfaces></General></Domain></CycloneDDS>'''

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# 定义SDK中声明的所有动作方法
ALL_ACTIONS = [
    # 基础控制动作（无参数）
    (1001, "Damp", None, "阻尼"),
    (1002, "BalanceStand", None, "平衡站立"),
    (1003, "StopMove", None, "停止移动"),
    (1004, "StandUp", None, "站立"),
    (1005, "StandDown", None, "趴下"),
    (1006, "RecoveryStand", None, "恢复站立"),
    (1009, "Sit", None, "坐下"),
    (1010, "RiseSit", None, "起坐"),
    (1012, "Trigger", None, "触发器"),
    
    # 表演动作
    (1016, "Hello", None, "打招呼"),
    (1017, "Stretch", None, "伸懒腰"),
    (1021, "Wallow", None, "打滚/比心？"),
    (1022, "Dance1", None, "舞蹈1"),
    (1023, "Dance2", None, "舞蹈2"),
    
    # 高级动作  
    (1029, "Scrape", None, "刮擦"),
    (1030, "FrontFlip", None, "前空翻"),
    (1031, "FrontJump", None, "前跳"),
    (1032, "FrontPounce", None, "前扑"),
    (1033, "WiggleHips", None, "扭腰"),
    (1036, "Heart", None, "比心"),
    
    # 高级动作（可能不支持）
    (1042, "LeftFlip", None, "左翻"),
    (1044, "BackFlip", None, "后空翻"),
    
    # 带参数的动作（需要参数）
    (1007, "Euler", (0.0, 0.0, 0.0), "姿态角度"),
    (1008, "Move", (0.0, 0.0, 0.0), "移动"),
    (1011, "SwitchGait", (0,), "步态切换"),
    (1013, "BodyHeight", (0.0,), "身体高度"),
    (1014, "FootRaiseHeight", (0.0,), "抬脚高度"),
    (1015, "SpeedLevel", (0,), "速度等级"),
    (1019, "ContinuousGait", (1,), "连续步态"),
    (1027, "SwitchJoystick", (True,), "切换摇杆"),
    (1028, "Pose", (True,), "摆姿势"),
    (1035, "EconomicGait", (True,), "经济步态"),
    (1045, "FreeWalk", (True,), "自由行走"),
    (1046, "FreeBound", (True,), "自由跳跃"),
    (1047, "FreeJump", (True,), "自由跳"),
    (1048, "FreeAvoid", (True,), "自由避障"),
    (1049, "WalkStair", (True,), "爬楼梯"),
    (1050, "WalkUpright", (True,), "直立行走"),
    (1051, "CrossStep", (True,), "交叉步"),
]

def test_all_actions():
    """测试所有SDK动作"""
    
    print("="*80)
    print("🔬 Unitree Go2 SDK动作完整测试")
    print("="*80)
    
    # 初始化
    print("\n📡 初始化DDS通道...")
    ChannelFactoryInitialize(0, "eth0")
    
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()
    
    # 测试连接
    print("🔗 测试连接...")
    test_result = client.RecoveryStand()
    if test_result == 0:
        print("✅ 连接成功")
    elif test_result == 3103:
        print("❌ APP占用，请关闭APP并重启机器人")
        return
    else:
        print(f"⚠️ 连接测试返回码: {test_result}")
    
    time.sleep(1)
    
    # 统计
    results = {
        "supported": [],
        "unsupported_3203": [],
        "unsupported_3104": [],
        "error_other": [],
        "not_found": []
    }
    
    print("\n" + "="*80)
    print("📊 开始测试所有动作...")
    print("="*80)
    
    for api_id, method_name, params, description in ALL_ACTIONS:
        print(f"\n测试 {api_id:4d} | {method_name:20s} | {description:20s}", end=" ")
        
        # 检查方法是否存在
        if not hasattr(client, method_name):
            print(f"❌ 方法不存在")
            results["not_found"].append((api_id, method_name, description))
            continue
        
        # 获取方法
        method = getattr(client, method_name)
        
        try:
            # 调用方法（根据是否需要参数）
            if params is None:
                result = method()
            else:
                result = method(*params)
            
            # 分析返回码
            if result == 0:
                print(f"✅ 成功 (0)")
                results["supported"].append((api_id, method_name, description))
            elif result == -1:
                print(f"✅ 已在状态 (-1)")
                results["supported"].append((api_id, method_name, description))
            elif result == 3203:
                print(f"❌ 未实现 (3203)")
                results["unsupported_3203"].append((api_id, method_name, description))
            elif result == 3104:
                print(f"⚠️ 特殊返回 (3104)")
                results["unsupported_3104"].append((api_id, method_name, description))
            else:
                print(f"❓ 未知返回码 ({result})")
                results["error_other"].append((api_id, method_name, description, result))
                
        except Exception as e:
            print(f"💥 异常: {e}")
            results["error_other"].append((api_id, method_name, description, str(e)))
        
        # 短暂延迟避免命令冲突
        time.sleep(0.5)
    
    # 打印总结
    print("\n" + "="*80)
    print("📊 测试结果总结")
    print("="*80)
    
    print(f"\n✅ **支持的动作** ({len(results['supported'])}个):")
    for api_id, method, desc in results['supported']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")
    
    print(f"\n❌ **未实现(3203)** ({len(results['unsupported_3203'])}个):")
    for api_id, method, desc in results['unsupported_3203']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")
    
    print(f"\n⚠️ **特殊返回(3104)** ({len(results['unsupported_3104'])}个):")
    for api_id, method, desc in results['unsupported_3104']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")
    
    print(f"\n❌ **方法不存在** ({len(results['not_found'])}个):")
    for api_id, method, desc in results['not_found']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")
    
    if results['error_other']:
        print(f"\n💥 **其他错误** ({len(results['error_other'])}个):")
        for item in results['error_other']:
            print(f"   {item}")
    
    # 保存结果
    result_file = f"/home/m1ng/claudia/test_results_{int(time.time())}.json"
    with open(result_file, 'w', encoding='utf-8') as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "results": {
                "supported": [(a, m, d) for a, m, d in results['supported']],
                "unsupported_3203": [(a, m, d) for a, m, d in results['unsupported_3203']],
                "unsupported_3104": [(a, m, d) for a, m, d in results['unsupported_3104']],
                "not_found": [(a, m, d) for a, m, d in results['not_found']],
                "error_other": results['error_other']
            },
            "summary": {
                "total_tested": len(ALL_ACTIONS),
                "supported": len(results['supported']),
                "unsupported": len(results['unsupported_3203']) + len(results['unsupported_3104']),
                "not_found": len(results['not_found'])
            }
        }, f, indent=2, ensure_ascii=False)
    
    print(f"\n💾 结果已保存到: {result_file}")
    
    # 最终统计
    print("\n" + "="*80)
    print("🎯 最终统计")
    print("="*80)
    print(f"测试动作总数: {len(ALL_ACTIONS)}")
    print(f"✅ 支持: {len(results['supported'])}")
    print(f"❌ 不支持(3203): {len(results['unsupported_3203'])}")
    print(f"⚠️ 特殊(3104): {len(results['unsupported_3104'])}")
    print(f"❌ 方法不存在: {len(results['not_found'])}")
    print("="*80)

if __name__ == "__main__":
    test_all_actions()
