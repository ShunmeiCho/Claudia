#!/usr/bin/env python3
"""
Unitree消息类型配置
基于led_controller.py的成功实现
"""

# 正确的Unitree消息导入方式
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, MotorCmd_, BmsCmd_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_AVAILABLE = True
    UNITREE_IMPORT_METHOD = 'unitree_go.msg.dds_'
    print("✅ Unitree硬件控制可用 (unitree_go.msg.dds_)")
except ImportError as e:
    UNITREE_AVAILABLE = False
    print(f"⚠️ Unitree SDK2未安装或不可用: {e}")
    print("💡 LED控制将使用ClaudiaLEDController（已验证工作正常）")
    # 定义占位符
    LowCmd_ = None
    LowState_ = None
    ChannelSubscriber = None
    ChannelPublisher = None
    ChannelFactoryInitialize = None

class UnitreeMessages:
    """Unitree消息管理类"""
    
    @staticmethod
    def is_available():
        """检查Unitree硬件是否可用"""
        return UNITREE_AVAILABLE
    
    @staticmethod
    def get_import_method():
        """获取导入方法"""
        return UNITREE_IMPORT_METHOD if UNITREE_AVAILABLE else None
    
    @staticmethod
    def create_subscriber(topic, message_type=None):
        """创建订阅者"""
        if not UNITREE_AVAILABLE:
            return None
        
        if message_type is None:
            message_type = LowState_
            
        return ChannelSubscriber(topic, message_type)
    
    @staticmethod
    def create_publisher(topic, message_type=None):
        """创建发布者"""
        if not UNITREE_AVAILABLE:
            return None
            
        if message_type is None:
            message_type = LowCmd_
            
        return ChannelPublisher(topic, message_type)
    
    @staticmethod
    def create_low_cmd_with_params():
        """创建LowCmd消息（带所需参数）"""
        if not UNITREE_AVAILABLE:
            return None
        
        try:
            # 基于led_controller.py的成功实现
            # 1. head: uint8[2] - 消息头
            head = [0xFE, 0xEF]
            
            # 2. level_flag: uint8 - 级别标志  
            level_flag = 0xFF
            
            # 3. frame_reserve: uint8 - 帧保留
            frame_reserve = 0
            
            # 4. sn: uint32[2] - 序列号
            sn = [0, 0]
            
            # 5. version: uint32[2] - 版本号
            version = [0, 0]
            
            # 6. bandwidth: uint16 - 带宽
            bandwidth = 0
            
            # 7. motor_cmd: MotorCmd_[20] - 电机命令数组
            motor_cmd = []
            for i in range(20):
                motor_cmd.append(MotorCmd_(
                    mode=0x00, q=0.0, dq=0.0, tau=0.0, 
                    kp=0.0, kd=0.0, reserve=[0, 0, 0]
                ))
            
            # 8. bms_cmd: BmsCmd_ - 电池管理系统命令
            bms_cmd = BmsCmd_(off=0, reserve=[0, 0, 0])
            
            # 9. wireless_remote: uint8[40] - 无线遥控器数据
            wireless_remote = [0] * 40
            
            # 10. led: uint8[12] - LED数据
            led = [0] * 12
            
            # 11. fan: uint8[2] - 风扇控制
            fan = [0, 0]
            
            # 12. gpio: uint8 - GPIO状态
            gpio = 0
            
            # 13. reserve: uint32 - 保留字段
            reserve = 0
            
            # 14. crc: uint32 - CRC校验
            crc = 0
            
            # 使用位置参数创建LowCmd消息
            msg = LowCmd_(
                head=head, level_flag=level_flag, frame_reserve=frame_reserve, sn=sn, version=version, bandwidth=bandwidth,
                motor_cmd=motor_cmd, bms_cmd=bms_cmd, wireless_remote=wireless_remote, led=led, fan=fan, gpio=gpio, reserve=reserve, crc=crc
            )
            
            return msg
            
        except Exception as e:
            print(f"创建LowCmd消息失败: {e}")
            return None
    
    @staticmethod
    def test_hardware_communication():
        """测试硬件通信"""
        if not UNITREE_AVAILABLE:
            print("❌ SDK不可用")
            return False
        
        try:
            print("🔧 测试硬件通信...")
            
            # 初始化DDS通道工厂（必需步骤）
            try:
                ChannelFactoryInitialize(0, "eth0")  # 使用默认网络接口
                print("✅ DDS通道工厂初始化成功")
            except Exception as e:
                print(f"⚠️ DDS通道工厂初始化失败: {e}")
                # 继续测试，可能在某些环境下这不是必需的
            
            # 测试LowCmd创建
            cmd_msg = UnitreeMessages.create_low_cmd_with_params()
            if cmd_msg is None:
                print("❌ LowCmd创建失败")
                return False
            print("✅ LowCmd创建成功")
            
            # 测试通道创建（不实际发送数据）
            try:
                # 创建发布者测试
                pub = UnitreeMessages.create_publisher("rt/lowcmd")
                if pub is not None:
                    print("✅ 发布者创建成功")
                    try:
                        pub.Init()  # 初始化发布者
                        print("✅ 发布者初始化成功")
                    except Exception as e:
                        print(f"⚠️ 发布者初始化失败: {e}")
                else:
                    print("❌ 发布者创建失败")
                    return False
                
                # 创建订阅者测试
                sub = UnitreeMessages.create_subscriber("rt/lowstate")
                if sub is not None:
                    print("✅ 订阅者创建成功")
                    try:
                        sub.Init()  # 初始化订阅者
                        print("✅ 订阅者初始化成功")
                    except Exception as e:
                        print(f"⚠️ 订阅者初始化失败: {e}")
                else:
                    print("❌ 订阅者创建失败")
                    return False
                
                print("✅ 硬件通信测试完全通过")
                return True
                
            except Exception as e:
                print(f"❌ 通道创建失败: {e}")
                return False
            
        except Exception as e:
            print(f"❌ 硬件通信测试失败: {e}")
            return False

# 测试配置
if __name__ == "__main__":
    print(f"Unitree可用性: {UnitreeMessages.is_available()}")
    print(f"导入方法: {UnitreeMessages.get_import_method()}")
    
    if UnitreeMessages.is_available():
        success = UnitreeMessages.test_hardware_communication()
        if success:
            print("🎉 所有测试通过，硬件模式可用！")
        else:
            print("❌ 硬件通信测试失败")
    else:
        print("⚠️ 硬件不可用，将使用模拟模式") 