#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
审计日志 route 字段的规范名称 — 一次定稿，禁止随意新增

所有 _log_audit() 调用的 route= 参数必须使用本模块的常量，
运行时 assert + AST 级 CI 测试双重保证。
"""

# === PR1 路由 ===
ROUTE_EMERGENCY = "emergency"                      # 紧急停止
ROUTE_HOTPATH = "hotpath"                          # 热缓存命中
ROUTE_HOTPATH_REJECTED = "hotpath_safety_rejected"  # 热缓存安全拒绝
ROUTE_SEQUENCE = "sequence_predefined"             # 序列预定义
ROUTE_DANCE = "dance"                              # 舞蹈随机分支
ROUTE_CONVERSATIONAL = "conversational"            # 对话检测
ROUTE_PRECHECK_REJECTED = "precheck_rejected"      # 预检拒绝
ROUTE_LLM_7B = "7B"                               # Legacy 7B 单通道

# === PR2 路由（BRAIN_ROUTER_MODE != legacy 时）===
ROUTE_ACTION_CHANNEL = "action_channel"            # 双通道 action channel 正式执行
ROUTE_VOICE_CHANNEL = "voice_channel"              # 双通道 voice channel（纯文本）
ROUTE_SHADOW = "shadow"                            # Shadow 模式记录
ROUTE_ACTION_FALLBACK = "action_fallback"          # Action channel 失败回退 legacy

# === 所有合法 route 值（Go/No-Go 统计脚本用）===
ALL_ROUTES = frozenset([
    ROUTE_EMERGENCY, ROUTE_HOTPATH, ROUTE_HOTPATH_REJECTED,
    ROUTE_SEQUENCE, ROUTE_DANCE, ROUTE_CONVERSATIONAL,
    ROUTE_PRECHECK_REJECTED, ROUTE_LLM_7B,
    ROUTE_ACTION_CHANNEL, ROUTE_VOICE_CHANNEL,
    ROUTE_SHADOW, ROUTE_ACTION_FALLBACK,
])
