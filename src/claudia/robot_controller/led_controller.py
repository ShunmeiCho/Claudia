#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unitree Go2 LED控制器 - 核心模块
实现LowCmd消息发布和LED基础控制
增强版：集成基于亮度控制的闪灯系统

Author: Claudia AI System
Generated: 2025-06-30
Enhanced: 2025-07-01 - 添加闪灯模式支持
Purpose: 子任务6.1-6.4 - LED控制系统完整实现
"""

import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass
from enum import Enum
import struct

# 添加项目路径
sys.path.append('/home/m1ng/claudia/src')

# Unitree SDK2 imports
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, MotorCmd_, BmsCmd_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Unitree SDK2导入失败: {e}")
    UNITREE_SDK_AVAILABLE = False

# VUI LED brightness control import
try:
    from unitree_sdk2py.go2.vui.vui_client import VuiClient
    VUI_CLIENT_AVAILABLE = True
except ImportError:
    VUI_CLIENT_AVAILABLE = False

@dataclass
class LEDState:
    """LED状态数据类"""
    timestamp: float
    led_data: List[int]  # uint8[12] LED数据
    brightness: int      # 亮度级别 (0-255)
    is_active: bool     # 是否激活
    
class LEDControlMode(Enum):
    """LED控制模式枚举"""
    OFF = 0
    SOLID = 1           # 常亮
    FLASH = 2           # 闪烁
    DOUBLE_FLASH = 3    # 双闪
    TRIPLE_FLASH = 4    # 三闪
    PULSE = 5           # 脉冲
    # 新增闪灯模式
    NORMAL = 10         # 正常运行-常亮
    WAITING = 11        # 等待处理-单闪1Hz
    WARNING = 12        # 警告状态-双闪2Hz  
    ERROR = 13          # 故障状态-快闪4Hz
    SPECIAL = 14        # 特殊状态-呼吸灯

@dataclass
class FlashModeConfig:
    """闪灯模式配置"""
    mode: LEDControlMode
    type: str          # "steady", "single_flash", "double_flash", "fast_flash", "breathing"
    brightness: int    # 亮度级别 (0-10 for VUI, 0-255 for RGB)
    freq: float        # 频率 (Hz)
    brightness_range: Optional[Tuple[int, int]] = None  # 呼吸灯亮度范围
    description: str = ""
    
class ClaudiaLEDController:
    """
    Claudia机器人LED控制器
    
    负责通过LowCmd消息控制Unitree Go2的前置LED阵列
    支持多种LED模式和环境自适应亮度调节
    增强版：支持基于亮度控制的闪灯系统
    """
    
    def __init__(self, network_interface: str = "eth0"):
        """
        初始化LED控制器
        
        Args:
            network_interface: 网络接口名称
        """
        self.logger = logging.getLogger(__name__)
        self.network_interface = network_interface
        
        # LED控制状态
        self.current_state = LEDState(
            timestamp=time.time(),
            led_data=[0] * 12,
            brightness=128,  # 默认50%亮度
            is_active=False
        )
        
        # 控制参数
        self.control_frequency = 50  # 50Hz控制频率
        self.control_dt = 1.0 / self.control_frequency
        self.max_response_time = 0.2  # 200ms最大响应时间要求
        
        # SDK组件
        self.publisher = None
        self.crc_calculator = None
        self.vui_client = None
        self.is_initialized = False
        self.control_thread = None
        self.control_active = False
        
        # 闪灯系统状态
        self.flash_thread = None
        self.flash_stop_event = threading.Event()
        self.current_flash_mode = LEDControlMode.OFF
        self.use_vui_brightness = True  # 优先使用VUI亮度控制
        
        # 线程安全
        self.state_lock = threading.Lock()
        
        # 闪灯模式配置
        self.flash_mode_configs = {
            LEDControlMode.NORMAL: FlashModeConfig(
                mode=LEDControlMode.NORMAL,
                type="steady",
                brightness=8,
                freq=0.0,
                description="正常运行-常亮"
            ),
            LEDControlMode.WAITING: FlashModeConfig(
                mode=LEDControlMode.WAITING,
                type="single_flash", 
                brightness=6,
                freq=1.0,
                description="等待处理-单闪1Hz"
            ),
            LEDControlMode.WARNING: FlashModeConfig(
                mode=LEDControlMode.WARNING,
                type="double_flash",
                brightness=8,
                freq=2.0,
                description="警告状态-双闪2Hz"
            ),
            LEDControlMode.ERROR: FlashModeConfig(
                mode=LEDControlMode.ERROR,
                type="fast_flash",
                brightness=10,
                freq=4.0,
                description="故障状态-快闪4Hz"
            ),
            LEDControlMode.SPECIAL: FlashModeConfig(
                mode=LEDControlMode.SPECIAL,
                type="breathing",
                brightness=6,
                freq=0.5,
                brightness_range=(2, 10),
                description="特殊状态-呼吸灯"
            ),
            LEDControlMode.OFF: FlashModeConfig(
                mode=LEDControlMode.OFF,
                type="steady",
                brightness=0,
                freq=0.0,
                description="关闭状态"
            )
        }
        
        # 性能监控
        self.performance_metrics = {
            'messages_sent': 0,
            'last_send_time': 0.0,
            'average_latency': 0.0,
            'max_latency': 0.0,
            'flash_mode_switches': 0
        }
        
        self.logger.info("LED控制器初始化完成 (含闪灯系统)")
        
    def initialize_communication(self) -> bool:
        """
        初始化与机器人的通信连接
        
        Returns:
            bool: 初始化是否成功
        """
        success_count = 0
        total_methods = 2
        
        # 尝试初始化VUI客户端（优先方法）
        if self._initialize_vui_client():
            success_count += 1
            self.use_vui_brightness = True
            self.logger.info("✅ VUI LED亮度控制可用")
        else:
            self.logger.warning("⚠️ VUI LED亮度控制不可用")
            
        # 尝试初始化LowCmd控制（备用方法）
        if self._initialize_lowcmd_control():
            success_count += 1
            self.logger.info("✅ LowCmd LED控制可用")
        else:
            self.logger.warning("⚠️ LowCmd LED控制不可用")
        
        # 至少有一种方法可用即可
        if success_count > 0:
            self.is_initialized = True
            self.logger.info(f"🎉 LED控制器通信初始化完成 ({success_count}/{total_methods}种方法可用)")
            return True
        else:
            self.logger.error("❌ 所有LED控制方法均不可用")
            return False
    
    def _initialize_vui_client(self) -> bool:
        """初始化VUI客户端用于亮度控制"""
        if not VUI_CLIENT_AVAILABLE:
            return False
            
        try:
            ChannelFactoryInitialize(0, self.network_interface)
            self.vui_client = VuiClient()
            self.vui_client.SetTimeout(3.0)
            self.vui_client.Init()
            
            # 测试VUI亮度控制
            self.vui_client.SetBrightness(0)  # 安全测试
            self.logger.info("✅ VUI客户端初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"VUI客户端初始化失败: {e}")
            return False
    
    def _initialize_lowcmd_control(self) -> bool:
        """初始化LowCmd控制（原有方法）"""
        if not UNITREE_SDK_AVAILABLE:
            return False
            
        try:
            # 初始化DDS通道工厂（如果VUI未初始化）
            if self.vui_client is None:
                ChannelFactoryInitialize(0, self.network_interface)
            
            # 创建LowCmd发布者
            lowcmd_topic = "rt/lowcmd"
            self.publisher = ChannelPublisher(lowcmd_topic, LowCmd_)
            self.publisher.Init()
            
            # 初始化CRC校验器
            self.crc_calculator = CRC()
            
            # 测试基础通信
            return self._test_basic_communication()
            
        except Exception as e:
            self.logger.error(f"LowCmd控制初始化失败: {e}")
            return False
        
    def set_vui_brightness(self, level: int) -> bool:
        """
        使用VUI客户端设置LED亮度
        
        Args:
            level: 亮度级别 (0-10)
            
        Returns:
            bool: 设置是否成功
        """
        if not self.vui_client:
            return False
            
        level = max(0, min(10, level))  # 限制范围
        
        try:
            start_time = time.time()
            self.vui_client.SetBrightness(level)
            
            # 性能监控
            send_duration = time.time() - start_time
            self._update_performance_metrics(send_duration)
            
            self.logger.debug(f"VUI亮度设置成功: {level}, 耗时: {send_duration*1000:.1f}ms")
            return True
            
        except Exception as e:
            self.logger.error(f"VUI亮度设置失败: {e}")
            return False
    
    def start_flash_mode(self, mode: LEDControlMode) -> bool:
        """
        启动指定的闪灯模式
        
        Args:
            mode: 闪灯模式
            
        Returns:
            bool: 启动是否成功
        """
        if mode not in self.flash_mode_configs:
            self.logger.error(f"未知闪灯模式: {mode}")
            return False
        
        if not self.is_initialized:
            self.logger.error("LED控制器未初始化")
            return False
        
        # 停止当前闪灯模式
        self.stop_flash_mode()
        
        config = self.flash_mode_configs[mode]
        self.current_flash_mode = mode
        
        self.logger.info(f"🔄 启动闪灯模式: {config.description}")
        
        with self.state_lock:
            self.performance_metrics['flash_mode_switches'] += 1
        
        if config.type == "steady":
            # 常亮模式（包括关闭）
            if self.use_vui_brightness and self.vui_client:
                return self.set_vui_brightness(config.brightness)
            else:
                # 使用RGB方法设置白色LED
                brightness_255 = int(config.brightness * 255 / 10)
                return self.set_led_color_simple(255, 255, 255, brightness_255)
        else:
            # 闪烁模式 - 启动独立线程
            self.flash_stop_event.clear()
            self.flash_thread = threading.Thread(
                target=self._flash_worker,
                args=(config,),
                daemon=True
            )
            self.flash_thread.start()
            return True
    
    def stop_flash_mode(self) -> None:
        """停止当前闪灯模式"""
        if self.flash_thread and self.flash_thread.is_alive():
            self.flash_stop_event.set()
            self.flash_thread.join(timeout=2.0)
        self.current_flash_mode = LEDControlMode.OFF
    
    def _flash_worker(self, config: FlashModeConfig) -> None:
        """闪灯工作线程"""
        flash_type = config.type
        
        try:
            if flash_type == "single_flash":
                self._single_flash(config)
            elif flash_type == "double_flash":
                self._double_flash(config)
            elif flash_type == "fast_flash":
                self._fast_flash(config)
            elif flash_type == "breathing":
                self._breathing_flash(config)
        except Exception as e:
            self.logger.error(f"闪灯工作线程异常: {e}")
    
    def _single_flash(self, config: FlashModeConfig) -> None:
        """单闪模式实现"""
        period = 1.0 / config.freq
        on_time = period * 0.5
        off_time = period * 0.5
        
        while not self.flash_stop_event.is_set():
            # 亮
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(config.brightness)
            else:
                brightness_255 = int(config.brightness * 255 / 10)
                self.set_led_color_simple(255, 255, 255, brightness_255)
                
            if self.flash_stop_event.wait(on_time):
                break
                
            # 灭
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(0)
            else:
                self.set_led_color_simple(0, 0, 0, 0)
                
            if self.flash_stop_event.wait(off_time):
                break
    
    def _double_flash(self, config: FlashModeConfig) -> None:
        """双闪模式实现"""
        period = 1.0 / config.freq
        flash_time = period * 0.2
        gap_time = period * 0.1
        pause_time = period * 0.5
        
        while not self.flash_stop_event.is_set():
            # 第一次闪烁
            self._flash_once(config, flash_time)
            if self.flash_stop_event.wait(gap_time):
                break
                
            # 第二次闪烁
            self._flash_once(config, flash_time)
            if self.flash_stop_event.wait(pause_time):
                break
    
    def _fast_flash(self, config: FlashModeConfig) -> None:
        """快闪模式实现"""
        period = 1.0 / config.freq
        on_time = period * 0.5
        off_time = period * 0.5
        
        while not self.flash_stop_event.is_set():
            # 亮
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(config.brightness)
            else:
                brightness_255 = int(config.brightness * 255 / 10)
                self.set_led_color_simple(255, 255, 255, brightness_255)
                
            if self.flash_stop_event.wait(on_time):
                break
                
            # 灭
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(0)
            else:
                self.set_led_color_simple(0, 0, 0, 0)
                
            if self.flash_stop_event.wait(off_time):
                break
    
    def _breathing_flash(self, config: FlashModeConfig) -> None:
        """呼吸灯模式实现"""
        if not config.brightness_range:
            return
            
        min_brightness, max_brightness = config.brightness_range
        period = 1.0 / config.freq
        steps = 20
        step_time = period / (2 * steps)
        
        while not self.flash_stop_event.is_set():
            # 渐亮
            for i in range(steps + 1):
                if self.flash_stop_event.is_set():
                    break
                brightness = min_brightness + (max_brightness - min_brightness) * (i / steps)
                
                if self.use_vui_brightness and self.vui_client:
                    self.set_vui_brightness(int(brightness))
                else:
                    brightness_255 = int(brightness * 255 / 10)
                    self.set_led_color_simple(255, 255, 255, brightness_255)
                
                if self.flash_stop_event.wait(step_time):
                    break
                    
            # 渐暗
            for i in range(steps + 1):
                if self.flash_stop_event.is_set():
                    break
                brightness = max_brightness - (max_brightness - min_brightness) * (i / steps)
                
                if self.use_vui_brightness and self.vui_client:
                    self.set_vui_brightness(int(brightness))
                else:
                    brightness_255 = int(brightness * 255 / 10)
                    self.set_led_color_simple(255, 255, 255, brightness_255)
                
                if self.flash_stop_event.wait(step_time):
                    break
    
    def _flash_once(self, config: FlashModeConfig, duration: float) -> None:
        """执行一次闪烁"""
        # 亮
        if self.use_vui_brightness and self.vui_client:
            self.set_vui_brightness(config.brightness)
        else:
            brightness_255 = int(config.brightness * 255 / 10)
            self.set_led_color_simple(255, 255, 255, brightness_255)
            
        self.flash_stop_event.wait(duration / 2)
        
        # 灭
        if self.use_vui_brightness and self.vui_client:
            self.set_vui_brightness(0)
        else:
            self.set_led_color_simple(0, 0, 0, 0)
            
        self.flash_stop_event.wait(duration / 2)
    
    def get_available_flash_modes(self) -> Dict[LEDControlMode, str]:
        """
        获取可用的闪灯模式
        
        Returns:
            Dict: 模式ID到描述的映射
        """
        return {mode: config.description for mode, config in self.flash_mode_configs.items()}
    
    def get_current_flash_mode(self) -> LEDControlMode:
        """获取当前闪灯模式"""
        return self.current_flash_mode
    
    def _test_basic_communication(self) -> bool:
        """
        测试基础通信功能
        
        Returns:
            bool: 通信测试是否成功
        """
        try:
            self.logger.info("🔧 测试LED控制器基础通信...")
            
            if self.publisher is None:
                self.logger.error("发布者未初始化")
                return False
            
            # 创建测试LowCmd消息
            test_msg = self._create_lowcmd_message()
            
            # 设置安全的测试LED数据（全部关闭）
            test_msg.led = [0] * 12
            
            # 计算并设置CRC
            self._set_message_crc(test_msg)
            
            # 发送测试消息
            start_time = time.time()
            self.publisher.Write(test_msg)
            send_duration = time.time() - start_time
            
            self.logger.info(f"✅ 测试消息发送成功，耗时: {send_duration*1000:.1f}ms")
            
            # 验证响应时间要求
            if send_duration > self.max_response_time:
                self.logger.warning(f"⚠️ 发送耗时超过要求: {send_duration*1000:.1f}ms > {self.max_response_time*1000}ms")
                
            return True
            
        except Exception as e:
            self.logger.error(f"通信测试失败: {e}")
            return False
    
    def _create_lowcmd_message(self) -> 'LowCmd_':
        """
        创建标准的LowCmd消息结构
        根据调试结果，使用位置参数方法创建
        
        Returns:
            LowCmd_: 初始化的LowCmd消息
        """
        try:
            # 根据调试结果，LowCmd_需要按顺序提供14个参数
            
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
            
            # 7. motor_cmd: MotorCmd_[20] - 电机命令数组（20个电机）
            motor_cmd = []
            for i in range(20):  # Go2有20个电机
                # 创建安全的电机命令（停止模式）
                motor_cmd.append(MotorCmd_(
                    mode=0x00,      # 停止模式
                    q=0.0,          # 位置
                    dq=0.0,         # 速度  
                    tau=0.0,        # 力矩
                    kp=0.0,         # 位置增益
                    kd=0.0,         # 速度增益
                    reserve=[0, 0, 0]  # 保留字段 uint32[3]
                ))
            
            # 8. bms_cmd: BmsCmd_ - 电池管理系统命令
            bms_cmd = BmsCmd_(
                off=0,          # 关闭标志
                reserve=[0, 0, 0] # 保留字段 uint8[3]
            )
            
            # 9. wireless_remote: uint8[40] - 无线遥控器数据
            wireless_remote = [0] * 40
            
            # 10. led: uint8[12] - LED数据（我们要控制的字段！）
            led = [0] * 12
            
            # 11. fan: uint8[2] - 风扇控制
            fan = [0, 0]
            
            # 12. gpio: uint8 - GPIO状态
            gpio = 0
            
            # 13. reserve: uint32 - 保留字段
            reserve = 0
            
            # 14. crc: uint32 - CRC校验（稍后计算）
            crc = 0
            
            # 使用位置参数创建LowCmd消息（调试已确认这种方法有效）
            msg = LowCmd_(
                head, level_flag, frame_reserve, sn, version, bandwidth,
                motor_cmd, bms_cmd, wireless_remote, led, fan, gpio, reserve, crc
            )
            
            self.logger.debug("✅ LowCmd消息创建成功（位置参数方法）")
            self.logger.debug(f"   LED字段长度: {len(msg.led)}")
            return msg
            
        except Exception as e:
            self.logger.error(f"创建LowCmd消息失败: {e}")
            # 提供更多调试信息
            try:
                self.logger.debug(f"LowCmd_类型: {type(LowCmd_)}")
                if hasattr(LowCmd_, '__doc__'):
                    self.logger.debug(f"LowCmd_文档: {LowCmd_.__doc__}")
            except:
                pass
            raise
    
    def _set_message_crc(self, msg: 'LowCmd_') -> None:
        """
        计算并设置消息的CRC校验值
        
        Args:
            msg: LowCmd消息
        """
        try:
            # 基于文档示例的CRC计算方法
            # crc = crc32_core((uint32_t *)&lowcmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1)
            
            if self.crc_calculator is None:
                self.logger.warning("CRC校验器未初始化，跳过CRC计算")
                msg.crc = 0
                return
                
            # 使用Unitree SDK提供的CRC计算方法
            # 注意：实际的CRC计算可能需要根据SDK文档进行调整
            crc_value = self.crc_calculator.Crc(msg)
            msg.crc = crc_value
            
            self.logger.debug(f"CRC计算完成: 0x{crc_value:08X}")
            
        except Exception as e:
            self.logger.error(f"CRC计算失败: {e}")
            msg.crc = 0  # 设置为0作为安全回退
    
    def set_led_direct(self, led_data: List[int]) -> bool:
        """
        直接设置LED数据
        
        Args:
            led_data: LED数据数组，长度必须为12
            
        Returns:
            bool: 设置是否成功
        """
        if not self.is_initialized:
            self.logger.error("LED控制器未初始化")
            return False
            
        if self.publisher is None:
            self.logger.error("发布者未初始化")
            return False
            
        if len(led_data) != 12:
            self.logger.error(f"LED数据长度错误: {len(led_data)} != 12")
            return False
            
        # 验证数据范围
        for i, value in enumerate(led_data):
            if not (0 <= value <= 255):
                self.logger.error(f"LED数据[{i}]超出范围: {value} (应为0-255)")
                return False
        
        try:
            start_time = time.time()
            
            # 更新内部状态
            with self.state_lock:
                self.current_state.led_data = led_data.copy()
                self.current_state.timestamp = start_time
                self.current_state.is_active = any(x > 0 for x in led_data)
            
            # 创建并发送LowCmd消息
            msg = self._create_lowcmd_message()
            msg.led = led_data
            self._set_message_crc(msg)
            
            # 发送消息
            self.publisher.Write(msg)
            
            # 性能监控
            send_duration = time.time() - start_time
            self._update_performance_metrics(send_duration)
            
            self.logger.debug(f"LED直接设置成功，耗时: {send_duration*1000:.1f}ms")
            
            return True
            
        except Exception as e:
            self.logger.error(f"LED直接设置失败: {e}")
            return False
    
    def set_led_color_simple(self, r: int, g: int, b: int, brightness: int = 255) -> bool:
        """
        设置简单的RGB颜色（假设前4个LED为RGB控制）
        
        Args:
            r: 红色分量 (0-255)
            g: 绿色分量 (0-255)  
            b: 蓝色分量 (0-255)
            brightness: 亮度 (0-255)
            
        Returns:
            bool: 设置是否成功
        """
        # 应用亮度缩放，确保结果为整数
        r_scaled = int((r * brightness) // 255)
        g_scaled = int((g * brightness) // 255)
        b_scaled = int((b * brightness) // 255)
        
        # 创建LED数据 - 假设格式1: [R1,G1,B1,R2,G2,B2,R3,G3,B3,R4,G4,B4]
        # 这个假设需要通过实验验证
        led_data = [r_scaled, g_scaled, b_scaled] * 4  # 复制到4个LED
        
        self.logger.info(f"设置LED颜色: RGB({r_scaled},{g_scaled},{b_scaled}) 亮度={brightness}")
        return self.set_led_direct(led_data)
    
    def turn_off_all_leds(self) -> bool:
        """
        关闭所有LED
        
        Returns:
            bool: 操作是否成功
        """
        self.logger.info("关闭所有LED")
        return self.set_led_direct([0] * 12)
    
    def _update_performance_metrics(self, send_duration: float) -> None:
        """
        更新性能监控指标
        
        Args:
            send_duration: 发送耗时（秒）
        """
        self.performance_metrics['messages_sent'] += 1
        self.performance_metrics['last_send_time'] = send_duration
        
        # 计算平均延迟
        count = self.performance_metrics['messages_sent']
        if count == 1:
            self.performance_metrics['average_latency'] = send_duration
        else:
            # 移动平均
            alpha = 0.1  # 平滑因子
            current_avg = self.performance_metrics['average_latency']
            self.performance_metrics['average_latency'] = (1 - alpha) * current_avg + alpha * send_duration
        
        # 更新最大延迟
        if send_duration > self.performance_metrics['max_latency']:
            self.performance_metrics['max_latency'] = send_duration
        
        # 检查性能要求
        if send_duration > self.max_response_time:
            self.logger.warning(f"⚠️ LED响应时间超标: {send_duration*1000:.1f}ms > {self.max_response_time*1000}ms")
    
    def get_performance_info(self) -> Dict[str, Any]:
        """
        获取性能信息
        
        Returns:
            Dict: 性能监控数据
        """
        with self.state_lock:
            return {
                'messages_sent': self.performance_metrics['messages_sent'],
                'last_send_time_ms': self.performance_metrics['last_send_time'] * 1000,
                'average_latency_ms': self.performance_metrics['average_latency'] * 1000,
                'max_latency_ms': self.performance_metrics['max_latency'] * 1000,
                'meets_requirement': self.performance_metrics['max_latency'] <= self.max_response_time,
                'flash_mode_switches': self.performance_metrics['flash_mode_switches'],
                'current_flash_mode': self.current_flash_mode.name,
                'vui_brightness_available': self.vui_client is not None,
                'lowcmd_control_available': self.publisher is not None,
                'current_state': self.current_state
            }
    
    def get_current_state(self) -> LEDState:
        """
        获取当前LED状态
        
        Returns:
            LEDState: 当前LED状态
        """
        with self.state_lock:
            return LEDState(
                timestamp=self.current_state.timestamp,
                led_data=self.current_state.led_data.copy(),
                brightness=self.current_state.brightness,
                is_active=self.current_state.is_active
            )
    
    def cleanup(self) -> None:
        """清理资源"""
        self.logger.info("清理LED控制器资源...")
        
        try:
            # 停止闪灯模式
            self.stop_flash_mode()
            
            # 关闭所有LED
            if self.is_initialized:
                if self.use_vui_brightness and self.vui_client:
                    self.set_vui_brightness(0)
                else:
                    self.turn_off_all_leds()
                time.sleep(0.1)  # 确保最后一条消息发送完成
                
            # 停止控制线程
            if self.control_thread and self.control_thread.is_alive():
                self.control_active = False
                self.control_thread.join(timeout=1.0)
                
            self.is_initialized = False
            self.logger.info("✅ LED控制器资源清理完成")
            
        except Exception as e:
            self.logger.error(f"LED控制器清理失败: {e}")
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.cleanup()

# 工厂函数
def create_led_controller(network_interface: str = "eth0") -> ClaudiaLEDController:
    """
    创建LED控制器实例
    
    Args:
        network_interface: 网络接口名称
        
    Returns:
        ClaudiaLEDController: LED控制器实例
    """
    return ClaudiaLEDController(network_interface)

if __name__ == "__main__":
    # 基础测试
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    print("🧪 LED控制器基础测试")
    print("=" * 50)
    
    try:
        with create_led_controller() as controller:
            if controller.initialize_communication():
                print("✅ LED控制器初始化成功")
                
                # 测试基础LED控制
                print("\n🔴 测试红色LED...")
                controller.set_led_color_simple(255, 0, 0, 128)
                time.sleep(2)
                
                print("🟢 测试绿色LED...")
                controller.set_led_color_simple(0, 255, 0, 128)
                time.sleep(2)
                
                print("🔵 测试蓝色LED...")
                controller.set_led_color_simple(0, 0, 255, 128)
                time.sleep(2)
                
                print("⚫ 关闭所有LED...")
                controller.turn_off_all_leds()
                
                # 显示性能信息
                perf_info = controller.get_performance_info()
                print(f"\n📊 性能信息:")
                print(f"   发送消息数: {perf_info['messages_sent']}")
                print(f"   平均延迟: {perf_info['average_latency_ms']:.1f}ms")
                print(f"   最大延迟: {perf_info['max_latency_ms']:.1f}ms") 
                print(f"   符合要求: {'✅' if perf_info['meets_requirement'] else '❌'}")
                
            else:
                print("❌ LED控制器初始化失败")
                
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
    except Exception as e:
        print(f"❌ 测试过程出错: {e}")
        import traceback
        traceback.print_exc() 