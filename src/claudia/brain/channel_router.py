#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ChannelRouter — 双通道 LLM 决策路由器（PR2-B）

决策层：根据 BRAIN_ROUTER_MODE 环境变量决定使用哪个 LLM 通道。
**不执行动作，不调用 SafetyCompiler** — 只产出候选决策 (api_code/sequence/response)，
交由 process_command() 安全编译后再执行。

三种模式:
  - LEGACY: 完全透传现有 7B LLM 路径（零行为变更）
  - DUAL:   Action 通道优先，a=null 时回退 Voice 通道
  - SHADOW: Legacy 为主，Dual 并行观测 + 日志对比
"""

import os
import time
import uuid
import asyncio
import logging
from enum import Enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from claudia.brain.production_brain import ProductionBrain

from claudia.brain.audit_routes import (
    ROUTE_LLM_7B, ROUTE_ACTION_CHANNEL, ROUTE_VOICE_CHANNEL,
    ROUTE_SHADOW, ROUTE_ACTION_FALLBACK,
)
from claudia.brain.action_registry import (
    VALID_API_CODES, HIGH_ENERGY_ACTIONS,
    get_response_for_action, get_response_for_sequence,
)


class RouterMode(Enum):
    """路由模式（BRAIN_ROUTER_MODE 环境变量）"""
    LEGACY = "legacy"
    DUAL = "dual"
    SHADOW = "shadow"


@dataclass
class RouterResult:
    """路由决策结果 — process_command() 映射到 BrainOutput 前的中间表示

    所有路由方法统一返回此类型（Fix #2: 无 tuple 返回值）。
    shadow_comparison 仅在 SHADOW 模式下填充。
    """
    api_code: Optional[int]
    sequence: Optional[List[int]]
    response: str
    route: str                                # audit_routes 常量
    action_latency_ms: float = 0.0
    voice_latency_ms: float = 0.0
    request_id: str = ""                      # Invariant 4: 全链路追踪
    raw_llm_output: Optional[str] = None      # 审计用: 原始 LLM JSON
    shadow_comparison: Optional[Dict[str, Any]] = None  # Shadow 模式专用

    @property
    def has_action(self):
        # type: () -> bool
        """是否包含可执行动作（api_code 或 sequence）"""
        return self.api_code is not None or (self.sequence is not None and len(self.sequence) > 0)


# 序列校验常量（Fix #4）
MAX_SEQUENCE_LENGTH = 4


class ChannelRouter:
    """决策路由器 — 只决策，不执行，不调用 SafetyCompiler

    产出候选 (api_code, sequence, response)，process_command() 负责:
    1. SafetyCompiler.compile() 安全编译
    2. execute_action() 实际执行
    """

    def __init__(self, brain, mode):
        # type: (ProductionBrain, RouterMode) -> None
        self._brain = brain
        self._mode = mode
        self._logger = brain.logger
        self._action_model = os.getenv("BRAIN_MODEL_ACTION", "claudia-action-v1")

    @property
    def mode(self):
        # type: () -> RouterMode
        return self._mode

    async def route(self, command, state_snapshot=None, start_time=None):
        # type: (str, Any, Optional[float]) -> RouterResult
        """主入口: 根据当前模式分派到对应路由方法"""
        request_id = uuid.uuid4().hex[:8]
        if self._mode == RouterMode.LEGACY:
            return await self._legacy_route(command, request_id,
                                            state_snapshot=state_snapshot,
                                            start_time=start_time)
        elif self._mode == RouterMode.DUAL:
            return await self._dual_route(command, request_id,
                                          state_snapshot=state_snapshot,
                                          start_time=start_time)
        else:
            return await self._shadow_route(command, request_id,
                                            state_snapshot=state_snapshot,
                                            start_time=start_time)

    # ------------------------------------------------------------------
    # Legacy: 完全透传现有 7B LLM
    # ------------------------------------------------------------------

    async def _legacy_route(self, command, request_id,
                            state_snapshot=None, start_time=None,
                            route=ROUTE_LLM_7B):
        # type: (str, str, Any, Optional[float], str) -> RouterResult
        """Legacy 模式: 调用现有 7B 模型，返回 RouterResult

        透传: 行为与 PR1 完全一致，只是包装为 RouterResult。
        """
        t0 = time.monotonic()
        result = await self._brain._call_ollama_v2(
            self._brain.model_7b,
            command,
            timeout=25,
        )
        latency = (time.monotonic() - t0) * 1000

        if result is None:
            return RouterResult(
                api_code=None, sequence=None,
                response="すみません、理解できませんでした",
                route=route,
                action_latency_ms=latency,
                request_id=request_id,
            )

        # 提取字段（兼容完整和缩写字段名）
        raw_response = result.get("response") or result.get("r", "実行します")
        response = self._brain._sanitize_response(raw_response)
        api_code = result.get("api_code") or result.get("a")
        sequence = result.get("sequence") or result.get("s")

        return RouterResult(
            api_code=api_code,
            sequence=sequence,
            response=response,
            route=route,
            action_latency_ms=latency,
            request_id=request_id,
            raw_llm_output=str(result)[:200],
        )

    # ------------------------------------------------------------------
    # Dual: Action 通道优先，a=null → Voice 回退
    # ------------------------------------------------------------------

    async def _dual_route(self, command, request_id,
                          state_snapshot=None, start_time=None):
        # type: (str, str, Any, Optional[float]) -> RouterResult
        """Dual 模式: Action 通道决策 → a=null 时回退 Voice"""
        # Step 1: Action 通道（短 prompt，~30 tokens）
        action_result = await self._action_channel(command, request_id)

        # Step 2: a=null → 需要完整文本响应（Invariant 2）
        # Fix #3: 如果 _action_channel 已经 fallback 到 legacy（route=ACTION_FALLBACK），
        # 直接返回该 legacy 结果，不再重复调用 _voice_fallback()
        if not action_result.has_action:
            if action_result.route == ROUTE_ACTION_FALLBACK:
                # 已经走过 legacy 了，直接返回（避免双重 LLM 调用）
                return action_result
            voice_result = await self._voice_fallback(command, request_id,
                                                      state_snapshot=state_snapshot,
                                                      start_time=start_time)
            return voice_result

        # Step 3: 有动作 → 使用模板响应（不需要 LLM 生成文本）
        action_result.response = self._generate_template_response(action_result)
        return action_result

    async def _action_channel(self, command, request_id, allow_fallback=True):
        # type: (str, str, bool) -> RouterResult
        """Action 通道: 调用 action model，只输出 {a:N} 或 {s:[...]} 或 {a:null}

        Fix #4: 校验 api_code 和 sequence 合法性
        allow_fallback: True=失败时回退 legacy（Dual 模式），False=直接返回失败结果（Shadow 观测用）
        """
        t0 = time.monotonic()
        raw = await self._brain._call_ollama_v2(
            model=self._action_model,
            command=command,
            timeout=5,          # 动作通道紧凑超时
            num_predict=30,     # ~30 tokens 足够输出 {"a":1009}
            num_ctx=1024,       # 缩小上下文窗口，降低推理开销
        )
        latency = (time.monotonic() - t0) * 1000

        if raw is None:
            if allow_fallback:
                # Dual 模式: 解析/超时失败 → 回退 legacy
                self._logger.warning("Action 通道超时/失败，回退 legacy")
                fallback = await self._legacy_route(
                    command, request_id, route=ROUTE_ACTION_FALLBACK)
                return fallback
            else:
                # Shadow 观测: 不回退，直接记录失败（保持对照纯净性）
                self._logger.warning("Action 通道超时/失败 (shadow 观测，不回退)")
                return RouterResult(
                    api_code=None, sequence=None,
                    response="",
                    route=ROUTE_ACTION_CHANNEL,
                    action_latency_ms=latency,
                    request_id=request_id,
                    raw_llm_output="timeout/error",
                )

        api_code = raw.get("a")
        sequence = raw.get("s")

        # --- Fix: a+s 同时出现时归一化 ---
        # s 更具体（序列动作），优先于 a；清除 a 防止模板响应与执行不一致
        if api_code is not None and sequence is not None:
            self._logger.info(
                "Action 通道 a={} 与 s={} 同时出现，s 优先".format(api_code, sequence))
            api_code = None

        # --- 单动作校验 ---
        if api_code is not None and api_code not in VALID_API_CODES:
            self._logger.warning(
                "Action 通道非法 api_code={}".format(api_code))
            if allow_fallback:
                fallback = await self._legacy_route(
                    command, request_id, route=ROUTE_ACTION_FALLBACK)
                return fallback
            # Shadow 观测: 记录非法结果，不回退
            api_code = None

        # --- 序列校验（Fix #4）---
        if sequence is not None:
            if not isinstance(sequence, list) or len(sequence) == 0:
                self._logger.warning(
                    "Action 通道序列类型错误或为空")
                if allow_fallback:
                    fallback = await self._legacy_route(
                        command, request_id, route=ROUTE_ACTION_FALLBACK)
                    return fallback
                sequence = None
            else:
                # 过滤: 保留合法码，记录非法项
                valid_seq = [c for c in sequence
                             if isinstance(c, int) and c in VALID_API_CODES]
                invalid_items = [c for c in sequence
                                 if not (isinstance(c, int) and c in VALID_API_CODES)]
                if invalid_items:
                    self._logger.warning(
                        "Action 通道过滤非法序列项: {}".format(invalid_items))

                # 长度限制
                if len(valid_seq) > MAX_SEQUENCE_LENGTH:
                    self._logger.warning(
                        "Action 通道序列过长 ({}), 截断至 {}".format(
                            len(valid_seq), MAX_SEQUENCE_LENGTH))
                    valid_seq = valid_seq[:MAX_SEQUENCE_LENGTH]

                if not valid_seq:
                    # 全部非法
                    self._logger.warning("Action 通道序列全部非法")
                    if allow_fallback:
                        fallback = await self._legacy_route(
                            command, request_id, route=ROUTE_ACTION_FALLBACK)
                        return fallback
                    sequence = None
                else:
                    sequence = valid_seq

        return RouterResult(
            api_code=api_code,
            sequence=sequence,
            response="",  # 由 _generate_template_response 或 _voice_fallback 填充
            route=ROUTE_ACTION_CHANNEL,
            action_latency_ms=latency,
            request_id=request_id,
            raw_llm_output=str(raw)[:200],
        )

    async def _voice_fallback(self, command, request_id,
                              state_snapshot=None, start_time=None):
        # type: (str, str, Any, Optional[float]) -> RouterResult
        """a=null 时的 Voice 回退: 调用 legacy LLM 获取完整文本响应

        Invariant 2: 绝不返回空响应给用户
        """
        t0 = time.monotonic()
        legacy = await self._legacy_route(command, request_id,
                                          state_snapshot=state_snapshot,
                                          start_time=start_time)
        legacy.route = ROUTE_VOICE_CHANNEL
        legacy.voice_latency_ms = (time.monotonic() - t0) * 1000
        return legacy

    def _generate_template_response(self, result):
        # type: (RouterResult) -> str
        """根据 api_code/sequence 生成模板日语响应"""
        if result.api_code is not None:
            return get_response_for_action(result.api_code)
        elif result.sequence:
            return get_response_for_sequence(result.sequence)
        return ""

    # ------------------------------------------------------------------
    # Shadow: Legacy 为主，Dual 并行观测
    # ------------------------------------------------------------------

    async def _shadow_route(self, command, request_id,
                            state_snapshot=None, start_time=None):
        # type: (str, str, Any, Optional[float]) -> RouterResult
        """Shadow 模式: Legacy 为主（返回给用户），Dual 并行观测 + 对比日志

        Invariant 4: request_id 全链路追踪，dual 任务 5s 超时，
        超时/错误显式记录（不静默丢弃）

        Fix #2: 统一返回 RouterResult（shadow_comparison 为 Optional 字段）
        Fix #3: 对比 api_code + sequence，检测 high_risk_divergence
        """
        # 主路径: Legacy（胜出，返回给调用方）
        legacy_task = asyncio.ensure_future(
            self._legacy_route(command, request_id,
                               state_snapshot=state_snapshot,
                               start_time=start_time))
        # 观测路径: Action 通道（有限超时，不回退 legacy — 保持对照纯净性）
        dual_task = asyncio.ensure_future(
            self._action_channel(command, request_id, allow_fallback=False))

        # 等待 legacy（主路径）
        legacy_result = await legacy_task

        # 等待 dual（观测路径，带超时 — Invariant 4）
        shadow = self._build_shadow_comparison(
            legacy_result, dual_task)
        shadow = await shadow

        legacy_result.route = ROUTE_SHADOW
        legacy_result.shadow_comparison = shadow
        return legacy_result

    async def _build_shadow_comparison(self, legacy_result, dual_task):
        # type: (RouterResult, asyncio.Task) -> Dict[str, Any]
        """构建 shadow 对比数据（Fix #3: 完整对比 + high_risk_divergence）"""
        try:
            dual_result = await asyncio.wait_for(
                asyncio.shield(dual_task), timeout=5.0)

            # 原始决策对比（SafetyCompiler 前）
            raw_agreement = (
                legacy_result.api_code == dual_result.api_code
                and legacy_result.sequence == dual_result.sequence
            )

            # 高风险分歧检测
            legacy_codes = self._extract_action_codes(legacy_result)
            dual_codes = self._extract_action_codes(dual_result)
            high_risk_divergence = bool(
                (legacy_codes & HIGH_ENERGY_ACTIONS) != (dual_codes & HIGH_ENERGY_ACTIONS)
            )

            return {
                "legacy_api_code": legacy_result.api_code,
                "legacy_sequence": legacy_result.sequence,
                "dual_api_code": dual_result.api_code,
                "dual_sequence": dual_result.sequence,
                "raw_agreement": raw_agreement,
                "high_risk_divergence": high_risk_divergence,
                "legacy_ms": legacy_result.action_latency_ms,
                "dual_ms": dual_result.action_latency_ms,
            }

        except asyncio.TimeoutError:
            self._logger.warning(
                "Shadow dual 超时 (>5s), request_id={}".format(
                    legacy_result.request_id))
            if not dual_task.done():
                dual_task.cancel()
            return {
                "legacy_api_code": legacy_result.api_code,
                "legacy_sequence": legacy_result.sequence,
                "dual_api_code": "timeout",
                "dual_sequence": None,
                "raw_agreement": False,
                "high_risk_divergence": False,
                "legacy_ms": legacy_result.action_latency_ms,
                "dual_ms": 5000.0,
            }

        except Exception as e:
            self._logger.error(
                "Shadow dual 异常: {}, request_id={}".format(
                    e, legacy_result.request_id))
            return {
                "legacy_api_code": legacy_result.api_code,
                "legacy_sequence": legacy_result.sequence,
                "dual_api_code": "error",
                "dual_sequence": None,
                "raw_agreement": False,
                "high_risk_divergence": False,
                "error": str(e),
            }

    @staticmethod
    def _extract_action_codes(result):
        # type: (RouterResult) -> set
        """从 RouterResult 提取动作码集合（用于分歧检测）"""
        codes = set()  # type: set
        if result.api_code is not None:
            codes.add(result.api_code)
        if result.sequence:
            codes.update(result.sequence)
        return codes
