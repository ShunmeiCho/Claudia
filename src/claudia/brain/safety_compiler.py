#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全编译器 — 所有安全检查的唯一入口

输入: 候选动作（单个或序列）+ 机器人状态
输出: SafetyVerdict（可执行序列 / 拒绝 / 降级）

pipeline:
  1. 白名单校验 — 非法 api_code 直接拒绝
  2. 高风险策略门控 — allow_high_risk=False 时无条件拒绝 HIGH_ENERGY_ACTIONS
  3. 电量门控 — <=0.10: 仅安全动作, <=0.20: 禁高能, <=0.30: 高能降级为 Dance
  4. 站立前置 — 需站立但未站立 → 按需插入 StandUp(1004)（支持序列中间插入）
  5. 序列截断 — 序列中首拒=整段拒，中拒=截断，降级=替换

注意:
  - 电量值全程使用 0.0-1.0 归一化格式，不使用 0-100 百分比
  - 时钟域: snapshot_timestamp 使用 time.monotonic()（不受 NTP 跳变影响）
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional

from .action_registry import (
    EXECUTABLE_API_CODES, REQUIRE_STANDING, HIGH_ENERGY_ACTIONS,
)


@dataclass
class SafetyVerdict:
    """安全编译结果"""
    executable_sequence: List[int] = field(default_factory=list)
    auto_prepend: List[int] = field(default_factory=list)  # 自动前插的动作（如 StandUp）
    rejected: List[int] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    is_blocked: bool = False           # 整个请求被拒绝
    block_reason: str = ""
    response_override: Optional[str] = None  # 被拒时的安全响应


class SafetyCompiler:
    """统一安全管线 — 所有安全检查的唯一入口

    取代旧的 SafetyValidator + _quick_safety_precheck + _final_safety_gate 三处分散逻辑。
    """

    # 安全动作白名单（极低电量时允许）
    SAFE_ACTIONS = frozenset([1001, 1002, 1003, 1004, 1005, 1006, 1009, 1010])
    # 会改变姿态状态的动作（用于序列内虚拟姿态推进）
    # True=执行后站立, False=执行后非站立
    POSTURE_TRANSITIONS = {
        1004: True,   # StandUp
        1006: True,   # RecoveryStand
        1010: True,   # RiseSit
        1005: False,  # StandDown
        1009: False,  # Sit
    }

    def __init__(self, downgrade_target=1023, snapshot_max_age=5.0,
                 allow_high_risk=False):
        # type: (int, float, bool) -> None
        """初始化安全编译器

        Args:
            downgrade_target: 高能动作降级目标（默认 Dance2=1023，可调优）
            snapshot_max_age: 状态快照最大年龄（秒），超过则对高风险动作 fail-safe
            allow_high_risk: 是否允许高风险动作 (FrontFlip/FrontJump/FrontPounce)。
                默认 False，与旧 SafetyValidator(enable_high_risk_actions=False) 行为一致。
                通过 SAFETY_ALLOW_HIGH_RISK=1 环境变量可启用。
        """
        self.downgrade_target = downgrade_target
        self.snapshot_max_age = snapshot_max_age
        self.allow_high_risk = allow_high_risk

    def compile(
        self,
        actions,           # type: List[int]
        battery_level,     # type: float
        is_standing,       # type: bool
        snapshot_timestamp=None,  # type: Optional[float]
    ):
        # type: (...) -> SafetyVerdict
        """编译动作序列为安全可执行版本

        Args:
            actions: 候选动作列表（单动作也包装为 [code]）
            battery_level: 当前电量 (0.0-1.0 归一化)
            is_standing: 当前是否站立
            snapshot_timestamp: 状态快照时间戳 (time.monotonic())，None = 跳过新鲜度检查
        """
        verdict = SafetyVerdict()

        # === Step 0a: 输入契约 — 电量硬性校验 ===
        if battery_level is None or not isinstance(battery_level, (int, float)):
            verdict.is_blocked = True
            verdict.block_reason = "battery_level 缺失或类型异常"
            verdict.response_override = "状態データが不正です。安全のため動作を停止しました"
            verdict.warnings.append("battery_level={!r} 不合法".format(battery_level))
            return verdict

        if battery_level > 1.0:
            # 不做自动 /100 修正 — 自动修正会掩盖上游归一化 bug
            verdict.is_blocked = True
            verdict.block_reason = (
                "battery_level={} > 1.0，超出合法范围 [0.0, 1.0]".format(battery_level)
            )
            verdict.response_override = "バッテリーデータが異常です。安全のため動作を停止しました"
            verdict.warnings.append(
                "battery_level={} 不在 [0.0, 1.0] 范围，"
                "检查 _normalize_battery() 或 state_monitor 输出".format(battery_level)
            )
            return verdict

        if battery_level < 0.0:
            battery_level = 0.0
            verdict.warnings.append("battery_level < 0，修正为 0.0")

        # === Step 0b: 状态新鲜度检查 ===
        if snapshot_timestamp is not None:
            age = time.monotonic() - snapshot_timestamp
            if age > self.snapshot_max_age:
                verdict.warnings.append(
                    "状態快照过期 ({:.1f}s > {:.1f}s)".format(age, self.snapshot_max_age)
                )
                safe_only = [a for a in actions if a in self.SAFE_ACTIONS]
                if safe_only != actions:
                    verdict.warnings.append("过期状态下仅允许安全动作")
                    actions = safe_only
                    if not actions:
                        verdict.is_blocked = True
                        verdict.block_reason = "状态数据过期，高风险动作被拒绝"
                        verdict.response_override = (
                            "状態データが古いため、安全な動作のみ実行できます"
                        )
                        return verdict

        # === Step 1-5: 逐项编译 ===
        # 关键: 序列内姿态会变化，不能仅用输入 is_standing 一次判断。
        # virtual_standing 在每个动作后根据 POSTURE_TRANSITIONS 推进。
        virtual_standing = bool(is_standing)
        for api_code in actions:
            # Step 1: 白名单
            if api_code not in EXECUTABLE_API_CODES:
                verdict.rejected.append(api_code)
                verdict.warnings.append("API {} 不在白名单中".format(api_code))
                if not verdict.executable_sequence:
                    # 首动作非法 → 整个请求被拒
                    verdict.is_blocked = True
                    verdict.block_reason = "首动作 {} 非法".format(api_code)
                    verdict.response_override = "安全のため動作を停止しました"
                    return verdict
                # 中间非法 → 截断
                verdict.warnings.append(
                    "序列截断: 保留 {}".format(verdict.executable_sequence)
                )
                break

            # Step 2: 高风险策略门控（与旧 SafetyValidator 行为一致）
            if not self.allow_high_risk and api_code in HIGH_ENERGY_ACTIONS:
                verdict.rejected.append(api_code)
                verdict.warnings.append(
                    "高風險動作 {} 被策略禁止（allow_high_risk=False）".format(api_code)
                )
                if not verdict.executable_sequence:
                    verdict.is_blocked = True
                    verdict.block_reason = (
                        "高リスク動作は現在無効化されています (api_code={})".format(
                            api_code
                        )
                    )
                    verdict.response_override = (
                        "高リスク動作は現在無効化されています"
                    )
                    return verdict
                # 序列中间命中 → 截断
                verdict.warnings.append(
                    "序列截断: 高风险动作 {} 被策略拒绝".format(api_code)
                )
                break

            # Step 3: 电量门控
            gate_result = self._battery_gate(api_code, battery_level)
            if gate_result == "reject":
                verdict.rejected.append(api_code)
                if not verdict.executable_sequence:
                    verdict.is_blocked = True
                    verdict.block_reason = (
                        "电量 {:.0%} 不足，{} 被拒绝".format(battery_level, api_code)
                    )
                    verdict.response_override = (
                        "バッテリーが低いため、安全な動作のみ実行できます"
                    )
                    return verdict
                verdict.warnings.append(
                    "序列截断: {} 被电量门控拒绝".format(api_code)
                )
                break
            elif gate_result == "downgrade":
                verdict.warnings.append(
                    "API {} 降级为 {}（电量 {:.0%}）".format(
                        api_code, self.downgrade_target, battery_level
                    )
                )
                api_code = self.downgrade_target

            # Step 4: 站立前置（序列内按需插入）
            if api_code in REQUIRE_STANDING and not virtual_standing:
                # 避免紧邻重复插入 StandUp
                if not verdict.executable_sequence or verdict.executable_sequence[-1] != 1004:
                    insert_at_head = len(verdict.executable_sequence) == 0
                    verdict.executable_sequence.append(1004)
                    # 兼容旧字段：只有头部插入计入 auto_prepend
                    if insert_at_head and 1004 not in verdict.auto_prepend:
                        verdict.auto_prepend.append(1004)
                        verdict.warnings.append(
                            "自动前插 StandUp(1004): {} 需要站立".format(api_code)
                        )
                    elif not insert_at_head:
                        verdict.warnings.append(
                            "序列中插入 StandUp(1004): {} 需要站立".format(api_code)
                        )
                virtual_standing = True

            verdict.executable_sequence.append(api_code)

            # 序列姿态推进：让后续动作使用“执行后”姿态判断
            if api_code in self.POSTURE_TRANSITIONS:
                virtual_standing = self.POSTURE_TRANSITIONS[api_code]

        return verdict

    def _battery_gate(self, api_code, battery_level):
        # type: (int, float) -> str
        """电量门控

        Args:
            api_code: 待检查的动作码
            battery_level: 0.0-1.0 归一化电量

        Returns: "pass" | "reject" | "downgrade"
        """
        if battery_level <= 0.10:
            if api_code not in self.SAFE_ACTIONS:
                return "reject"
        elif battery_level <= 0.20:
            if api_code in HIGH_ENERGY_ACTIONS:
                return "reject"
        elif battery_level <= 0.30:
            if api_code in HIGH_ENERGY_ACTIONS:
                return "downgrade"
        return "pass"
