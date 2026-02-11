#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mock SportClient — 模拟机器人控制器 (V2)

用于在没有真实机器人的情况下测试系统。
返回值确定性（P0-6 要求），与真实 SDK 返回码语义一致：
  0    = 成功
  -1   = 已处于目标状态
  3104 = RPC_ERR_CLIENT_API_TIMEOUT（异步动作"已触发但未确认完成"）
  3103 = APP 占用
  3203 = 不支持
"""

import logging
import time


class MockSportClient:
    """模拟的 SportClient，返回确定性响应

    所有无参数动作与 ACTION_REGISTRY 中的 METHOD_MAP 对齐。
    """

    def __init__(self):
        self.logger = logging.getLogger("MockSportClient")
        self.timeout = 10.0
        self.is_connected = False
        self.current_state = "standing"

    def SetTimeout(self, timeout):
        # type: (float) -> None
        """设置超时时间"""
        self.timeout = timeout

    def Init(self):
        """初始化（模拟）"""
        time.sleep(0.1)  # 缩短模拟延迟（测试友好）
        self.is_connected = True
        self.logger.info("MockSportClient 初始化成功")

    def _simulate_action(self, action_name, duration=0.1):
        # type: (str, float) -> int
        """模拟动作执行（确定性返回）"""
        self.logger.info("[模拟] 执行动作: {}".format(action_name))
        time.sleep(duration)
        return 0

    # === 基础姿态 ===

    def Damp(self):
        # type: () -> int
        """阻尼模式 (1001)"""
        self.current_state = "damped"
        return self._simulate_action("Damp", 0.1)

    def BalanceStand(self):
        # type: () -> int
        """平衡站立 (1002)"""
        self.current_state = "standing"
        return self._simulate_action("BalanceStand", 0.1)

    def StopMove(self):
        # type: () -> int
        """停止移动 (1003)"""
        return self._simulate_action("StopMove", 0.05)

    def StandUp(self):
        # type: () -> int
        """站立 (1004)"""
        if self.current_state == "standing":
            return -1  # 已经站立
        self.current_state = "standing"
        return self._simulate_action("StandUp", 0.2)

    def StandDown(self):
        # type: () -> int
        """伏せ (1005)"""
        self.current_state = "lying"
        return self._simulate_action("StandDown", 0.2)

    def RecoveryStand(self):
        # type: () -> int
        """回復スタンド (1006)"""
        self.current_state = "standing"
        return self._simulate_action("RecoveryStand", 0.2)

    def Sit(self):
        # type: () -> int
        """座る (1009)"""
        self.current_state = "sitting"
        return self._simulate_action("Sit", 0.2)

    def RiseSit(self):
        # type: () -> int
        """起き上がる (1010)"""
        self.current_state = "standing"
        return self._simulate_action("RiseSit", 0.2)

    # === 表演动作 ===

    def Hello(self):
        # type: () -> int
        """挨拶 (1016)"""
        return self._simulate_action("Hello", 0.3)

    def Stretch(self):
        # type: () -> int
        """伸び (1017)"""
        return self._simulate_action("Stretch", 0.3)

    def Wallow(self):
        # type: () -> int
        """転がる (1021)"""
        return self._simulate_action("Wallow", 0.3)

    def Dance1(self):
        # type: () -> int
        """ダンス1 (1022) — 确定性返回 3104（异步动作触发码）"""
        self.logger.info("[模拟] 执行动作: Dance1")
        time.sleep(0.3)
        return 3104

    def Dance2(self):
        # type: () -> int
        """ダンス2 (1023) — 确定性返回 3104（异步动作触发码）"""
        self.logger.info("[模拟] 执行动作: Dance2")
        time.sleep(0.3)
        return 3104

    def Scrape(self):
        # type: () -> int
        """刮る (1029)"""
        return self._simulate_action("Scrape", 0.3)

    def WiggleHips(self):
        # type: () -> int
        """腰を振る (1033)"""
        return self._simulate_action("WiggleHips", 0.3)

    def Heart(self):
        # type: () -> int
        """ハート (1036)"""
        return self._simulate_action("Heart", 0.3)

    # === 高风险动作 ===

    def FrontFlip(self):
        # type: () -> int
        """前転 (1030)"""
        return self._simulate_action("FrontFlip", 0.5)

    def FrontJump(self):
        # type: () -> int
        """ジャンプ (1031)"""
        return self._simulate_action("FrontJump", 0.5)

    def FrontPounce(self):
        # type: () -> int
        """飛びかかる (1032)"""
        return self._simulate_action("FrontPounce", 0.5)

    # === 参数化动作 ===

    def Euler(self, roll=0, pitch=0, yaw=0):
        # type: (float, float, float) -> int
        """姿勢制御 (1007)"""
        return self._simulate_action("Euler({},{},{})".format(roll, pitch, yaw), 0.1)

    def Move(self, x=0, y=0, z=0):
        # type: (float, float, float) -> int
        """移動 (1008)"""
        return self._simulate_action("Move({},{},{})".format(x, y, z), 0.1)

    def SpeedLevel(self, level=1):
        # type: (int,) -> int
        """速度レベル (1015)"""
        return self._simulate_action("SpeedLevel({})".format(level), 0.05)

    def ContinuousGait(self, flag=1):
        # type: (int,) -> int
        """歩行モード (1019)"""
        return self._simulate_action("ContinuousGait({})".format(flag), 0.05)

    def SwitchJoystick(self, enable=True):
        # type: (bool,) -> int
        """操作切替 (1027)"""
        return self._simulate_action("SwitchJoystick({})".format(enable), 0.05)

    def Pose(self, enable=True):
        # type: (bool,) -> int
        """ポーズ (1028)"""
        return self._simulate_action("Pose({})".format(enable), 0.1)

    # === 状态查询 ===

    def GetState(self, keys=None):
        # type: (list) -> tuple
        """获取当前状态（兼容 SDK GetState 签名）

        Returns:
            (return_code, state_dict) 元组，与真实 SDK 返回格式一致。
            P0-5 连通性检查依赖此签名：GetState(["mode"])
        """
        state = {"mode": self.current_state}
        if keys:
            state = {k: state.get(k) for k in keys}
        return 0, state
