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
  - SHADOW: Legacy 为主，Action 顺序观测 + 日志对比（单 GPU 优化）
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
    VALID_API_CODES, HIGH_ENERGY_ACTIONS, ACTION_SCHEMA,
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
    # action channel 状态（shadow 对比用，区分 a=null vs 超时 vs 非法输出）
    # "ok" | "timeout" | "error" | "invalid_output"
    _action_status: str = "ok"

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
                            route=ROUTE_LLM_7B, ollama_timeout=25):
        # type: (str, str, Any, Optional[float], str, int) -> RouterResult
        """Legacy 模式: 调用现有 7B 模型，返回 RouterResult

        透传: 行为与 PR1 完全一致，只是包装为 RouterResult。
        ollama_timeout: Ollama 调用超时（Legacy/Dual=25s，Shadow=45s 含 VRAM 换入）
        """
        t0 = time.monotonic()
        result = await self._brain._call_ollama_v2(
            self._brain.model_7b,
            command,
            timeout=ollama_timeout,
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

    async def _action_channel(self, command, request_id,
                              allow_fallback=True, ollama_timeout=5):
        # type: (str, str, bool, int) -> RouterResult
        """Action 通道: 调用 action model，只输出 {a:N} 或 {s:[...]} 或 {a:null}

        Fix #4: 校验 api_code 和 sequence 合法性
        allow_fallback: True=失败时回退 legacy（Dual 模式），False=直接返回失败结果（Shadow 观测用）
        ollama_timeout: Ollama 调用超时（Dual=5s 紧凑，Shadow=15s 含 VRAM 换入）
        """
        t0 = time.monotonic()
        raw = await self._brain._call_ollama_v2(
            model=self._action_model,
            command=command,
            timeout=ollama_timeout,
            num_predict=30,     # ~30 tokens 足够输出 {"a":1009}
            num_ctx=1024,       # 缩小上下文窗口，降低推理开销
            output_format=ACTION_SCHEMA,  # 结构化输出: 强制 {"a":N} | {"s":[...]}
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
                    raw_llm_output="timeout",
                    _action_status="timeout",
                )

        api_code = raw.get("a")
        sequence = raw.get("s")

        # --- Fix: a+s 同时出现时归一化 ---
        # s 更具体（序列动作），优先于 a；清除 a 防止模板响应与执行不一致
        if api_code is not None and sequence is not None:
            self._logger.info(
                "Action 通道 a={} 与 s={} 同时出现，s 优先".format(api_code, sequence))
            api_code = None

        # 校验状态追踪（shadow 对比用）
        action_status = "ok"  # 会被下游校验降级

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
            action_status = "invalid_output"

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
                action_status = "invalid_output"
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
                    action_status = "invalid_output"
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
            _action_status=action_status,
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
    # Shadow: Legacy 为主，Action 顺序观测
    # ------------------------------------------------------------------

    async def _shadow_route(self, command, request_id,
                            state_snapshot=None, start_time=None):
        # type: (str, str, Any, Optional[float]) -> RouterResult
        """Shadow 模式: Legacy 为主（返回给用户），Action 通道顺序观测 + 对比日志

        单 GPU 设计: Ollama 无法并行处理不同模型（8GB VRAM 只容纳一个 4.7GB 模型）。
        并行 ensure_future 会被 Ollama 串行处理，但超时从请求提交时计算，
        导致排在后面的请求必然超时。因此改为顺序执行:

        1. Action 通道先跑（观测用，VRAM 换入 action 模型）
        2. Legacy 7B 后跑（主路径，VRAM 换入 7B — 留在 VRAM 为下条命令准备）

        Invariant 4: request_id 全链路追踪，action 超时/错误显式记录
        Fix #2: 统一返回 RouterResult（shadow_comparison 为 Optional 字段）
        Fix #3: 对比 api_code + sequence，检测 high_risk_divergence
        """
        # Step 1: Action 通道观测（allow_fallback=False 保持对照纯净性）
        # Jetson 8GB 统一内存: 模型切换 ~15-25s（Ollama 内部 30s + 外层 45s 兜底）
        dual_result = await self._action_channel_shadow(
            command, request_id, timeout=45)

        # Step 2: Legacy 7B 主路径（7B 模型留在 VRAM，为下条命令优化）
        # VRAM 换入 ~15-25s + 推理 ~5-10s → ollama_timeout=45s
        legacy_result = await self._legacy_route(
            command, request_id,
            state_snapshot=state_snapshot,
            start_time=start_time,
            ollama_timeout=45)

        # Step 3: 构建对比
        shadow = self._build_shadow_comparison(legacy_result, dual_result)
        legacy_result.route = ROUTE_SHADOW
        legacy_result.shadow_comparison = shadow
        return legacy_result

    async def _action_channel_shadow(self, command, request_id, timeout=20):
        # type: (str, str, int) -> RouterResult
        """Shadow 专用 action 通道: 捕获超时/异常，返回带状态的 RouterResult

        超时放宽: Ollama 内部 30s（Jetson VRAM 换入 ~15-25s + 推理 ~3-5s），
        外层 45s（兜底）。
        """
        try:
            result = await asyncio.wait_for(
                self._action_channel(
                    command, request_id,
                    allow_fallback=False, ollama_timeout=30),
                timeout=timeout,
            )
            return result
        except asyncio.TimeoutError:
            self._logger.warning(
                "Shadow action 通道超时 (>{}s), request_id={}".format(
                    timeout, request_id))
            return RouterResult(
                api_code=None, sequence=None,
                response="",
                route=ROUTE_ACTION_CHANNEL,
                action_latency_ms=timeout * 1000,
                request_id=request_id,
                _action_status="timeout",
            )
        except Exception as e:
            self._logger.error(
                "Shadow action 通道异常: {}, request_id={}".format(
                    e, request_id))
            return RouterResult(
                api_code=None, sequence=None,
                response="",
                route=ROUTE_ACTION_CHANNEL,
                action_latency_ms=0.0,
                request_id=request_id,
                _action_status="error",
            )

    def _build_shadow_comparison(self, legacy_result, dual_result):
        # type: (RouterResult, RouterResult) -> Dict[str, Any]
        """构建 shadow 对比数据（同步版，双结果均已完成）

        dual_status 语义:
          "ok"             — action channel 正常返回（含 a=null）
          "timeout"        — Ollama 调用超时
          "error"          — 异常（代码错误、网络断等）
          "invalid_output" — 模型输出非法 api_code/sequence，校验后清零
        """
        dual_status = getattr(dual_result, '_action_status', 'ok')

        if dual_status != "ok":
            return {
                "legacy_api_code": legacy_result.api_code,
                "legacy_sequence": legacy_result.sequence,
                "dual_api_code": dual_result.api_code,
                "dual_sequence": dual_result.sequence,
                "dual_status": dual_status,
                "raw_agreement": False,
                "high_risk_divergence": False,
                "legacy_ms": legacy_result.action_latency_ms,
                "dual_ms": dual_result.action_latency_ms,
            }

        raw_agreement = (
            legacy_result.api_code == dual_result.api_code
            and legacy_result.sequence == dual_result.sequence
        )

        legacy_codes = self._extract_action_codes(legacy_result)
        dual_codes = self._extract_action_codes(dual_result)
        high_risk_divergence = bool(
            (legacy_codes & HIGH_ENERGY_ACTIONS)
            != (dual_codes & HIGH_ENERGY_ACTIONS)
        )

        return {
            "legacy_api_code": legacy_result.api_code,
            "legacy_sequence": legacy_result.sequence,
            "dual_api_code": dual_result.api_code,
            "dual_sequence": dual_result.sequence,
            "dual_status": "ok",
            "raw_agreement": raw_agreement,
            "high_risk_divergence": high_risk_divergence,
            "legacy_ms": legacy_result.action_latency_ms,
            "dual_ms": dual_result.action_latency_ms,
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
