#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全验证器 - 代码层强制安全规则（不依赖LLM判断）
Track A核心模块，修复REVIEW问题
"""

import logging
from typing import Tuple, Optional, List
from dataclasses import dataclass

# 导入状态监控器类型
try:
    from claudia.robot_controller.system_state_monitor import SystemStateInfo, SystemState
    STATE_MONITOR_AVAILABLE = True
except ImportError:
    STATE_MONITOR_AVAILABLE = False
    SystemStateInfo = None
    SystemState = None


@dataclass
class SafetyCheckResult:
    """安全检查结果"""
    is_safe: bool
    reason: str
    modified_sequence: Optional[List[int]] = None  # 自动补全的前置动作
    should_use_sequence_only: bool = False  # 是否只使用sequence（避免双轨执行）


class SafetyValidator:
    """安全规则验证器（Track A版本）"""

    # 动作分类
    HIGH_ENERGY_ACTIONS = [1030, 1031, 1032]  # FrontFlip, FrontJump, FrontPounce
    PERFORMANCE_ACTIONS = [1016, 1017, 1022, 1023, 1036, 1029]  # Hello, Stretch, Dance, Heart, Scrape
    SAFE_ACTIONS = [1001, 1002, 1003, 1004, 1005, 1006, 1009, 1010]  # Damp, Balance, Stop, Stand, Down, Recovery, Sit, Rise

    # 需要站立姿态
    REQUIRE_STANDING = [1016, 1017, 1022, 1023, 1029, 1030, 1031, 1032, 1036]

    def __init__(self, enable_high_risk_actions: bool = False):
        """
        初始化安全验证器

        Args:
            enable_high_risk_actions: 是否允许高风险动作（Flip/Jump/Pounce）
        """
        self.logger = logging.getLogger("SafetyValidator")
        self.enable_high_risk_actions = enable_high_risk_actions

        # 安全阈值（统一为0-1比例）
        self.low_battery_threshold = 0.20  # 20%
        self.critical_battery_threshold = 0.05  # 5%

        if not enable_high_risk_actions:
            self.logger.info("🛡️ 高风险动作已禁用（Flip/Jump/Pounce）")

    def validate_action(self,
                       api_code: Optional[int],
                       state: Optional[SystemStateInfo] = None) -> SafetyCheckResult:
        """
        验证单个动作的安全性

        Args:
            api_code: API代码
            state: 当前机器人状态（电量为0-1比例）

        Returns:
            SafetyCheckResult: 安全检查结果
        """
        # 无API code（纯对话）
        if api_code is None or api_code == 0:
            return SafetyCheckResult(is_safe=True, reason="OK")

        # Track A初期：高风险动作默认禁用
        if not self.enable_high_risk_actions and api_code in self.HIGH_ENERGY_ACTIONS:
            return SafetyCheckResult(
                is_safe=False,
                reason="高リスク動作は現在無効化されています"
            )

        # 如果没有状态信息，进行基础检查
        if state is None:
            self.logger.warning("⚠️ 状态信息不可用，仅执行基础安全检查")
            return self._basic_safety_check(api_code)

        # 确保电量为0-1比例（修复REVIEW问题）
        battery_level = state.battery_level
        if battery_level > 1.0:
            self.logger.warning(f"⚠️ 电量单位异常({battery_level})，假设为百分比并转换")
            battery_level = battery_level / 100.0

        # 规则1: 紧急状态
        if STATE_MONITOR_AVAILABLE and state.state == SystemState.EMERGENCY:
            if api_code != 1003:  # 非Stop
                return SafetyCheckResult(
                    is_safe=False,
                    reason="緊急状態のため、停止のみ可能です"
                )

        # 规则2: 关键低电量
        if battery_level <= self.critical_battery_threshold:
            if api_code not in [1003, 1009]:  # 仅允许Stop和Sit
                return SafetyCheckResult(
                    is_safe=False,
                    reason=f"バッテリーが極めて低い({battery_level*100:.0f}%)です。充電してください"
                )

        # 规则3: 低电量限制高能动作
        if battery_level <= self.low_battery_threshold:
            if api_code in self.HIGH_ENERGY_ACTIONS:
                return SafetyCheckResult(
                    is_safe=False,
                    reason=f"バッテリーが低い({battery_level*100:.0f}%)ため、高エネルギー動作は禁止です"
                )

        # 规则4: 姿态检查（修复：避免双轨执行）
        if api_code in self.REQUIRE_STANDING:
            if not state.is_standing:
                self.logger.info(f"⚡ 动作{api_code}需要站立，自动添加前置动作")
                return SafetyCheckResult(
                    is_safe=True,
                    reason="自動的に立ち上がります",
                    modified_sequence=[1004, api_code],  # StandUp + 目标动作
                    should_use_sequence_only=True  # 只执行sequence，不执行单独的api_code
                )

        # 规则5: 错误状态
        if STATE_MONITOR_AVAILABLE and state.state == SystemState.ERROR:
            if state.error_codes:
                return SafetyCheckResult(
                    is_safe=False,
                    reason=f"システムエラー({state.error_codes})のため、動作を実行できません"
                )

        # 通过所有检查
        return SafetyCheckResult(is_safe=True, reason="OK")

    def _basic_safety_check(self, api_code: int) -> SafetyCheckResult:
        """基础安全检查（无状态信息时）"""
        # 高风险动作警告
        if api_code in self.HIGH_ENERGY_ACTIONS:
            if not self.enable_high_risk_actions:
                return SafetyCheckResult(
                    is_safe=False,
                    reason="高リスク動作は無効化されています"
                )
            return SafetyCheckResult(
                is_safe=True,
                reason="⚠️ 高リスク動作：安全な環境を確認してください"
            )

        return SafetyCheckResult(is_safe=True, reason="OK")

    def validate_sequence(self,
                         sequence: List[int],
                         state: Optional[SystemStateInfo] = None) -> SafetyCheckResult:
        """验证动作序列"""
        if not sequence:
            return SafetyCheckResult(is_safe=True, reason="OK")

        for i, api_code in enumerate(sequence):
            result = self.validate_action(api_code, state)
            if not result.is_safe:
                return SafetyCheckResult(
                    is_safe=False,
                    reason=f"シーケンス内の動作{i+1}({api_code})が安全でない: {result.reason}"
                )

        return SafetyCheckResult(is_safe=True, reason="OK")


# 全局单例
_safety_validator_instance = None

def get_safety_validator(enable_high_risk: bool = False) -> SafetyValidator:
    """获取安全验证器单例"""
    global _safety_validator_instance
    if _safety_validator_instance is None:
        _safety_validator_instance = SafetyValidator(enable_high_risk_actions=enable_high_risk)
    return _safety_validator_instance
