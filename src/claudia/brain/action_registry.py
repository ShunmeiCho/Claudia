#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动作注册表 — 机器人动作定义的唯一真源

所有动作属性定义在此，下游集合（白名单、站立需求、方法映射等）自动派生。
新增/删除/修改动作只需编辑 _ACTIONS 列表。
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, Optional, Tuple


@dataclass(frozen=True)
class ActionDef:
    """机器人动作定义 — 所有属性的唯一真源"""
    api_code: int
    method: str             # SportClient 方法名
    name_ja: str            # 日语显示名（TTS/响应模板用）
    requires_standing: bool = False
    risk_level: str = "safe"   # "safe" | "medium" | "high"
    has_params: bool = False   # True = 需要参数，不接入 LLM 白名单
    enabled: bool = True       # False = 全局禁用（SDK 未实现等）
    safe_default_params: Optional[Tuple] = None  # 安全默认参数（仅热缓存/直执行用）


# === 动作注册表（唯一定义点）===
_ACTIONS = [
    # --- 基础姿态（无参数）---
    ActionDef(1001, "Damp",          "ダンプモード"),
    ActionDef(1002, "BalanceStand",   "バランスします"),
    ActionDef(1003, "StopMove",       "止まります"),
    ActionDef(1004, "StandUp",        "立ちます"),
    ActionDef(1005, "StandDown",      "伏せます"),
    ActionDef(1006, "RecoveryStand",  "回復します"),
    ActionDef(1009, "Sit",            "座ります"),
    ActionDef(1010, "RiseSit",        "起き上がります"),

    # --- 表演动作（无参数，需站立的标注）---
    ActionDef(1016, "Hello",      "挨拶します",     requires_standing=True),
    ActionDef(1017, "Stretch",    "伸びをします",   requires_standing=True),
    ActionDef(1021, "Wallow",     "転がります"),     # 地面动作，不需站立
    ActionDef(1022, "Dance1",     "ダンス1します",   requires_standing=True),
    ActionDef(1023, "Dance2",     "ダンス2します",   requires_standing=True),
    ActionDef(1029, "Scrape",     "刮ります",       requires_standing=True),
    ActionDef(1033, "WiggleHips", "腰を振ります",   requires_standing=True),
    ActionDef(1036, "Heart",      "ハートします",   requires_standing=True),

    # --- 高风险表演（无参数，高能耗）---
    ActionDef(1030, "FrontFlip",   "前転します",       requires_standing=True, risk_level="high"),
    ActionDef(1031, "FrontJump",   "ジャンプします",   requires_standing=True, risk_level="high"),
    ActionDef(1032, "FrontPounce", "飛びかかります",   requires_standing=True, risk_level="high"),

    # --- 参数化动作（enabled=True 但 has_params=True 排除出 LLM 白名单）---
    ActionDef(1007, "Euler",           "姿勢を調整します", has_params=True),
    ActionDef(1008, "Move",            "歩きます",         has_params=True, risk_level="medium"),
    ActionDef(1015, "SpeedLevel",      "速度を変えます",   has_params=True),
    ActionDef(1019, "ContinuousGait",  "歩行モード変更",   has_params=True),
    ActionDef(1027, "SwitchJoystick",  "操作切り替え",     has_params=True),
    ActionDef(1028, "Pose",            "ポーズします",     has_params=True, safe_default_params=(True,)),

    # --- 高风险/未验证（禁用）---
    ActionDef(1042, "LeftFlip",  "横回転します", risk_level="high", enabled=False),
    ActionDef(1044, "BackFlip",  "バク転します", risk_level="high", enabled=False),
]

# === 索引 ===
ACTION_REGISTRY: Dict[int, ActionDef] = {a.api_code: a for a in _ACTIONS}

# === 自动派生集合（下游代码只用这些，不直接查 _ACTIONS）===

# LLM 可决策的动作白名单（无参数 + 已启用）
# 用途：LLM 输出解析层过滤，阻止模型输出参数化动作
VALID_API_CODES: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS if a.enabled and not a.has_params
)

# 可执行白名单（包含有安全默认参数的参数化动作）
# 用途：SafetyCompiler 白名单检查、热缓存条目校验
# 规则：无参数动作 + 有 safe_default_params 的参数化动作
EXECUTABLE_API_CODES: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS
    if a.enabled and (not a.has_params or a.safe_default_params is not None)
)

# 参数化动作的安全默认参数（仅限 safe_default_params 已声明的）
# 执行器调用时使用：method(*SAFE_DEFAULT_PARAMS[api_code])
SAFE_DEFAULT_PARAMS: Dict[int, Tuple] = {
    a.api_code: a.safe_default_params
    for a in _ACTIONS
    if a.enabled and a.has_params and a.safe_default_params is not None
}

# 需要站立前置的动作
REQUIRE_STANDING: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS if a.requires_standing
)

# 高能耗动作（电量门控用）
HIGH_ENERGY_ACTIONS: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS if a.risk_level == "high"
)

# api_code → SportClient 方法名
METHOD_MAP: Dict[int, str] = {
    a.api_code: a.method for a in _ACTIONS if a.enabled
}

# api_code → 日语响应模板
ACTION_RESPONSES: Dict[int, str] = {
    a.api_code: a.name_ja for a in _ACTIONS if a.enabled
}


def get_response_for_action(api_code: int) -> str:
    """获取动作的日语响应文本"""
    return ACTION_RESPONSES.get(api_code, "はい、わかりました")


def get_response_for_sequence(sequence: list) -> str:
    """获取序列动作的组合响应"""
    responses = [ACTION_RESPONSES.get(c, "動作") for c in sequence]
    return "、".join(responses)


# === JSON Schema（Action 通道结构化输出用）===
ACTION_SCHEMA = {
    "type": "object",
    "oneOf": [
        {"properties": {"a": {"type": ["integer", "null"]}},
         "required": ["a"], "additionalProperties": False},
        {"properties": {"s": {"type": "array", "items": {"type": "integer"}, "minItems": 1}},
         "required": ["s"], "additionalProperties": False}
    ]
}


def _remove_suffix(s: str, suffix: str) -> str:
    """Python 3.8 兼容的 removesuffix（3.9+ 才有 str.removesuffix）"""
    if s.endswith(suffix):
        return s[:-len(suffix)]
    return s


def generate_modelfile_action_block() -> str:
    """从 registry 自动生成 Modelfile 中的动作列表段

    用于: Modelfile 自动生成 / 一致性校验
    确保 Modelfile 动作列表与 VALID_API_CODES 始终一致
    """
    lines = []
    for a in sorted(_ACTIONS, key=lambda x: x.api_code):
        if a.enabled and not a.has_params:
            short_name = _remove_suffix(a.name_ja, "します")
            lines.append(f"{a.api_code}={short_name}")
    return " ".join(lines)
