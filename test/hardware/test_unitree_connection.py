#!/usr/bin/env python3
"""
Unitree Go2机器人基础通信测试
测试与机器人的连接和基本功能
基于官方示例代码的正确API使用方法
"""

import time
import sys

def test_robot_connection():
    """测试与Go2机器人的基础连接"""
    print("🤖 开始测试Unitree Go2机器人连接...")
    
    try:
        # 使用正确的导入方式（基于官方示例）
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
        
        print("✅ 成功导入unitree_sdk2py模块")
        
        print("初始化DDS通道工厂...")
        # 初始化通道工厂，参数：domain_id=0, interface="eth0"
        ChannelFactoryInitialize(0, "eth0")
        print("✅ DDS通道工厂初始化成功")
        
        # 创建状态数据容器
        received_data = {'lowstate': False, 'sportstate': False}
        
        def low_state_handler(msg: LowState_):
            print(f"✅ 收到LowState数据!")
            print(f"   IMU: x={msg.imu_state.quaternion[0]:.3f}, y={msg.imu_state.quaternion[1]:.3f}")
            print(f"   电池: 电压={msg.power_v:.1f}V, 电流={msg.power_a:.1f}A")
            print(f"   电机状态 (FR_0): 位置={msg.motor_state[0].q:.3f}, 速度={msg.motor_state[0].dq:.3f}")
            received_data['lowstate'] = True
        
        def sport_state_handler(msg: SportModeState_):
            print(f"✅ 收到SportModeState数据!")
            print(f"   模式: {msg.mode}")
            print(f"   进度: {msg.progress}")
            if hasattr(msg, 'velocity') and len(msg.velocity) >= 3:
                print(f"   速度: x={msg.velocity[0]:.3f}, y={msg.velocity[1]:.3f}, yaw={msg.velocity[2]:.3f}")
            if hasattr(msg, 'position') and len(msg.position) >= 3:
                print(f"   位置: x={msg.position[0]:.3f}, y={msg.position[1]:.3f}, z={msg.position[2]:.3f}")
            received_data['sportstate'] = True
        
        # 创建订阅者（使用官方API）
        print("创建低级状态订阅者...")
        lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_subscriber.Init(low_state_handler, 10)
        print("✅ 低级状态订阅者创建成功")
        
        print("创建运动状态订阅者...")
        sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        sportstate_subscriber.Init(sport_state_handler, 10)
        print("✅ 运动状态订阅者创建成功")
        
        print("等待机器人状态数据...")
        print("（注意：如果机器人未连接，将在10秒后超时）")
        
        start_time = time.time()
        timeout = 10  # 10秒超时
        
        while time.time() - start_time < timeout:
            # 使用官方示例的读取方式
            try:
                # 尝试读取低级状态
                lowstate_msg = lowstate_subscriber.Read()
                if lowstate_msg is not None:
                    low_state_handler(lowstate_msg)
                    
                # 尝试读取运动状态
                sportstate_msg = sportstate_subscriber.Read()
                if sportstate_msg is not None:
                    sport_state_handler(sportstate_msg)
                
                # 如果收到任一数据，认为连接成功
                if received_data['lowstate'] or received_data['sportstate']:
                    return True
                    
            except Exception as e:
                print(f"读取数据时出错: {e}")
            
            time.sleep(0.1)
        
        print("⚠️ 超时：未接收到机器人状态数据")
        print("   可能原因：")
        print("   1. 机器人未连接或未开机")
        print("   2. 网络配置不正确（当前网卡：eth0）")
        print("   3. 机器人不在同一网段")
        print("   4. DDS域配置不匹配")
        print("   5. 防火墙阻止了DDS通信")
        return False
        
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        print("请确认unitree_sdk2py已正确安装")
        return False
    except Exception as e:
        print(f"❌ 连接测试失败: {e}")
        print(f"   错误类型: {type(e).__name__}")
        import traceback
        traceback.print_exc()
        return False

def test_environment_setup():
    """测试环境配置"""
    print("🔧 测试环境配置...")
    
    import os
    rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'not set')
    print(f"   RMW_IMPLEMENTATION: {rmw_impl}")
    
    if rmw_impl != 'rmw_cyclonedds_cpp':
        print("⚠️  警告：RMW_IMPLEMENTATION未设置为rmw_cyclonedds_cpp")
        return False
    
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        print("✅ 成功导入核心通道模块")
        return True
    except Exception as e:
        print(f"❌ 环境配置测试失败: {e}")
        return False

def main():
    """主函数"""
    print("=" * 60)
    print("Unitree Go2 机器人通信测试")
    print("基于官方API的正确实现")
    print("=" * 60)
    
    # 显示系统信息
    print(f"Python版本: {sys.version}")
    print(f"测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 环境配置测试
    env_success = test_environment_setup()
    print()
    
    if not env_success:
        print("❌ 环境配置不正确，无法继续测试")
        return 1
    
    # 执行连接测试
    print("开始执行机器人连接测试...")
    success = test_robot_connection()
    
    print()
    print("=" * 60)
    if success:
        print("🎉 测试完成：机器人通信正常!")
        print("✅ 成功建立与Unitree Go2的DDS通信")
        print("下一步建议：")
        print("  - 测试基本控制命令（站立/趴下）")
        print("  - 验证传感器数据质量")
        print("  - 测试实时控制响应")
    else:
        print("❌ 测试失败：无法与机器人通信")
        print("故障排除步骤：")
        print("1. 确认机器人开机并处于正常状态")
        print("2. 检查网络连接：ping 192.168.123.xxx（机器人IP）")
        print("3. 确认在同一网段：机器人和开发机")
        print("4. 检查防火墙设置，允许DDS通信")
        print("5. 确认环境变量：export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    print("=" * 60)
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
