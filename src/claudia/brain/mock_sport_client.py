#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mock SportClient - 模拟机器人控制器
用于在没有真实机器人的情况下测试系统
"""

import logging
import time
import random
from typing import Optional

class MockSportClient:
    """模拟的SportClient，返回逼真的响应"""
    
    def __init__(self):
        self.logger = logging.getLogger("MockSportClient")
        self.timeout = 10.0
        self.is_connected = False
        self.current_state = "standing"  # 当前状态
        
    def SetTimeout(self, timeout: float):
        """设置超时时间"""
        self.timeout = timeout
        self.logger.info(f"设置超时: {timeout}秒")
        
    def Init(self):
        """初始化（模拟）"""
        time.sleep(0.5)  # 模拟初始化延迟
        self.is_connected = True
        self.logger.info("✅ MockSportClient初始化成功")
        
    def _simulate_action(self, action_name: str, duration: float = 1.0) -> int:
        """模拟动作执行"""
        self.logger.info(f"🎭 [模拟] 执行动作: {action_name}")
        time.sleep(duration)
        
        # 模拟不同的返回码
        if action_name in ["Dance1", "Dance2"]:
            return random.choice([0, 3104, 3105])  # 舞蹈特殊返回码
        elif action_name == "Wallow":
            return 3203  # 比心特殊返回码
        else:
            return 0  # 正常返回码
    
    # ========== 基础控制动作 ==========
    
    def Damp(self) -> int:
        """阻尼/紧急停止"""
        self.current_state = "damped"
        return self._simulate_action("Damp", 0.5)
    
    def StopMove(self) -> int:
        """停止移动"""
        return self._simulate_action("StopMove", 0.3)
    
    def BalanceStand(self) -> int:
        """平衡站立"""
        self.current_state = "standing"
        return self._simulate_action("BalanceStand", 1.0)
    
    def StandUp(self) -> int:
        """站立"""
        self.current_state = "standing"
        return self._simulate_action("StandUp", 1.5)
    
    def StandDown(self) -> int:
        """趴下"""
        self.current_state = "lying"
        return self._simulate_action("StandDown", 1.5)
    
    def Sit(self) -> int:
        """坐下"""
        self.current_state = "sitting"
        return self._simulate_action("Sit", 1.0)
    
    # ========== 表演动作 ==========
    
    def Hello(self) -> int:
        """招手打招呼"""
        return self._simulate_action("Hello", 2.0)
    
    def Stretch(self) -> int:
        """伸展"""
        return self._simulate_action("Stretch", 2.5)
    
    def Wallow(self) -> int:
        """比心"""
        return self._simulate_action("Wallow", 2.0)
    
    def Dance1(self) -> int:
        """舞蹈1"""
        return self._simulate_action("Dance1", 5.0)
    
    def Dance2(self) -> int:
        """舞蹈2"""
        return self._simulate_action("Dance2", 5.0)
    
    def ShakeHands(self) -> int:
        """握手"""
        return self._simulate_action("ShakeHands", 2.0)
    
    def Cheer(self) -> int:
        """庆祝/拜年"""
        return self._simulate_action("Cheer", 2.0)
    
    def Bow(self) -> int:
        """鞠躬"""
        return self._simulate_action("Bow", 1.5)
    
    # ========== 高级动作 ==========
    
    def Jump(self) -> int:
        """跳跃"""
        return self._simulate_action("Jump", 2.0)
    
    def Pounce(self) -> int:
        """扑击"""
        return self._simulate_action("Pounce", 2.0)
    
    def FrontFlip(self) -> int:
        """前空翻"""
        return self._simulate_action("FrontFlip", 3.0)
    
    def BackFlip(self) -> int:
        """后空翻"""
        return self._simulate_action("BackFlip", 3.0)
    
    def Punch(self) -> int:
        """出拳"""
        return self._simulate_action("Punch", 1.5)
    
    # ========== 运动控制（需要参数的） ==========
    
    def Move(self, x: float = 0, y: float = 0, z: float = 0) -> int:
        """移动"""
        return self._simulate_action(f"Move({x},{y},{z})", 1.0)
    
    def Euler(self, roll: float = 0, pitch: float = 0, yaw: float = 0) -> int:
        """姿态角度"""
        return self._simulate_action(f"Euler({roll},{pitch},{yaw})", 1.0)
    
    def BodyHeight(self, height: float = 0.3) -> int:
        """身体高度"""
        return self._simulate_action(f"BodyHeight({height})", 1.0)
    
    def FootRaiseHeight(self, height: float = 0.1) -> int:
        """抬脚高度"""
        return self._simulate_action(f"FootRaiseHeight({height})", 1.0)
    
    def SpeedLevel(self, level: int = 1) -> int:
        """速度等级"""
        return self._simulate_action(f"SpeedLevel({level})", 0.5)
    
    def SwitchGait(self, gait: int = 0) -> int:
        """步态切换"""
        return self._simulate_action(f"SwitchGait({gait})", 1.0)
    
    def Trigger(self) -> int:
        """触发器"""
        return self._simulate_action("Trigger", 0.5)
    
    def GetState(self) -> str:
        """获取当前状态"""
        return self.current_state
