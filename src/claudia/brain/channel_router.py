#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ChannelRouter -- Dual-Channel LLM Decision Router (PR2-B)

Decision layer: Determines which LLM channel to use based on the BRAIN_ROUTER_MODE environment variable.
**Does not execute actions, does not call SafetyCompiler** -- only produces candidate decisions
(api_code/sequence/response), which are passed to process_command() for safety compilation before execution.

Three modes:
  - LEGACY: Full passthrough of existing 7B LLM path (zero behavior change)
  - DUAL:   Action channel priority, falls back to Voice channel when a=null
  - SHADOW: Legacy as primary, Action sequentially observed + logged for comparison (single GPU optimized)
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
    """Router mode (BRAIN_ROUTER_MODE environment variable)"""
    LEGACY = "legacy"
    DUAL = "dual"
    SHADOW = "shadow"


@dataclass
class RouterResult:
    """Router decision result -- intermediate representation before process_command() maps to BrainOutput

    All routing methods return this type uniformly (Fix #2: no tuple return values).
    shadow_comparison is only populated in SHADOW mode.
    """
    api_code: Optional[int]
    sequence: Optional[List[int]]
    response: str
    route: str                                # audit_routes constant
    action_latency_ms: float = 0.0
    voice_latency_ms: float = 0.0
    request_id: str = ""                      # Invariant 4: end-to-end tracing
    raw_llm_output: Optional[str] = None      # For audit: raw LLM JSON
    shadow_comparison: Optional[Dict[str, Any]] = None  # Shadow mode only
    # Action channel status (for shadow comparison, distinguishes a=null vs timeout vs invalid output)
    # "ok" | "timeout" | "error" | "invalid_output"
    _action_status: str = "ok"

    @property
    def has_action(self):
        # type: () -> bool
        """Whether it contains an executable action (api_code or sequence)"""
        return self.api_code is not None or (self.sequence is not None and len(self.sequence) > 0)


# Sequence validation constants (Fix #4)
MAX_SEQUENCE_LENGTH = 4


class ChannelRouter:
    """Decision router -- decides only, does not execute, does not call SafetyCompiler

    Produces candidate (api_code, sequence, response); process_command() is responsible for:
    1. SafetyCompiler.compile() safety compilation
    2. execute_action() actual execution
    """

    def __init__(self, brain, mode):
        # type: (ProductionBrain, RouterMode) -> None
        self._brain = brain
        self._mode = mode
        self._logger = brain.logger
        self._action_model = os.getenv("BRAIN_MODEL_ACTION", "claudia-action-v3")

    @property
    def mode(self):
        # type: () -> RouterMode
        return self._mode

    async def route(self, command, state_snapshot=None, start_time=None):
        # type: (str, Any, Optional[float]) -> RouterResult
        """Main entry: dispatch to corresponding routing method based on current mode"""
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
    # Legacy: Full passthrough of existing 7B LLM
    # ------------------------------------------------------------------

    async def _legacy_route(self, command, request_id,
                            state_snapshot=None, start_time=None,
                            route=ROUTE_LLM_7B, ollama_timeout=30):
        # type: (str, str, Any, Optional[float], str, int) -> RouterResult
        """Legacy mode: Call existing 7B model, return RouterResult

        Passthrough: behavior is fully identical to PR1, just wrapped as RouterResult.
        ollama_timeout: Ollama call timeout (Legacy/Dual=30s, Shadow=45s including VRAM swap-in)
        """
        await self._brain._ensure_model_loaded(self._brain.model_7b, num_ctx=2048)
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

        # Extract fields (compatible with full and abbreviated field names)
        raw_response = result.get("response") or result.get("r", "実行します")
        response = self._brain._sanitize_response(raw_response)
        api_code = result.get("api_code") or result.get("a")
        sequence = result.get("sequence") or result.get("s")

        # Normalize api_code to int (LLM may return string)
        if api_code is not None and not isinstance(api_code, int):
            try:
                api_code = int(api_code)
            except (ValueError, TypeError):
                api_code = None

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
    # Dual: Action channel priority, a=null -> Voice fallback
    # ------------------------------------------------------------------

    async def _dual_route(self, command, request_id,
                          state_snapshot=None, start_time=None):
        # type: (str, str, Any, Optional[float]) -> RouterResult
        """Dual mode: Action channel decision, template response (no 7B model dependency)

        a=null -> Use brain._generate_conversational_response() template response
        Has action -> Use ACTION_RESPONSES template response
        Only uses Action model throughout, no need for 7B in VRAM.
        """
        # Step 1: Action channel (short prompt, ~30 tokens)
        action_result = await self._action_channel(command, request_id)

        # Step 2: a=null -> Template conversational response (no 7B call needed)
        if not action_result.has_action:
            if action_result.route == ROUTE_ACTION_FALLBACK:
                return action_result
            # Use brain's conversational template generator (keyword matching, no LLM call)
            action_result.response = self._brain._generate_conversational_response(command)
            action_result.route = ROUTE_VOICE_CHANNEL
            return action_result

        # Step 3: Has action -> Use template response (no LLM text generation needed)
        action_result.response = self._generate_template_response(action_result)
        return action_result

    async def _action_channel(self, command, request_id,
                              allow_fallback=True, ollama_timeout=10):
        # type: (str, str, bool, int) -> RouterResult
        """Action channel: Call action model, outputs only {a:N} or {s:[...]} or {a:null}

        Fix #4: Validates api_code and sequence legality
        allow_fallback: True=fallback to legacy on failure (Dual mode), False=return failure result directly (Shadow observation)
        ollama_timeout: Ollama call timeout (Dual=10s Jetson GPU inference margin, Shadow=15s including VRAM swap-in)
        """
        await self._brain._ensure_model_loaded(self._action_model, num_ctx=1024)
        t0 = time.monotonic()
        raw = await self._brain._call_ollama_v2(
            model=self._action_model,
            command=command,
            timeout=ollama_timeout,
            num_predict=30,     # ~30 tokens sufficient for {"a":1009}
            num_ctx=1024,       # Reduced context window to lower inference overhead
            output_format=ACTION_SCHEMA,  # Structured output: forces {"a":N} | {"s":[...]}
        )
        latency = (time.monotonic() - t0) * 1000

        if raw is None:
            if allow_fallback:
                # Dual mode: timeout/failure -> template response (no 7B call)
                self._logger.warning("Action channel timeout/failure, using template response")
                return self._template_fallback(command, request_id, latency)
            else:
                # Shadow observation: no fallback, directly record failure (maintain control purity)
                self._logger.warning("Action channel timeout/failure (shadow observation, no fallback)")
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

        # Normalize api_code to int (LLM may return string)
        if api_code is not None and not isinstance(api_code, int):
            try:
                api_code = int(api_code)
            except (ValueError, TypeError):
                api_code = None

        # --- Fix: Normalize when both a+s are present ---
        # s is more specific (sequence action), takes priority over a; clear a to prevent
        # inconsistency between template response and execution
        if api_code is not None and sequence is not None:
            self._logger.info(
                "Action channel a={} and s={} both present, s takes priority".format(api_code, sequence))
            api_code = None

        # Validation status tracking (for shadow comparison)
        action_status = "ok"  # May be downgraded by downstream validation

        # --- Single action validation ---
        if api_code is not None and api_code not in VALID_API_CODES:
            self._logger.warning(
                "Action channel illegal api_code={}".format(api_code))
            if allow_fallback:
                return self._template_fallback(command, request_id, latency)
            # Shadow observation: record illegal result, no fallback
            api_code = None
            action_status = "invalid_output"

        # --- Sequence validation (Fix #4) ---
        if sequence is not None:
            if not isinstance(sequence, list) or len(sequence) == 0:
                self._logger.warning(
                    "Action channel sequence type error or empty")
                if allow_fallback:
                    return self._template_fallback(command, request_id, latency)
                sequence = None
                action_status = "invalid_output"
            else:
                # Filter: keep valid codes, log invalid items
                valid_seq = [c for c in sequence
                             if isinstance(c, int) and c in VALID_API_CODES]
                invalid_items = [c for c in sequence
                                 if not (isinstance(c, int) and c in VALID_API_CODES)]
                if invalid_items:
                    self._logger.warning(
                        "Action channel filtered invalid sequence items: {}".format(invalid_items))

                # Length limit
                if len(valid_seq) > MAX_SEQUENCE_LENGTH:
                    self._logger.warning(
                        "Action channel sequence too long ({}), truncating to {}".format(
                            len(valid_seq), MAX_SEQUENCE_LENGTH))
                    valid_seq = valid_seq[:MAX_SEQUENCE_LENGTH]

                if not valid_seq:
                    # All invalid
                    self._logger.warning("Action channel sequence all invalid")
                    if allow_fallback:
                        return self._template_fallback(command, request_id, latency)
                    sequence = None
                    action_status = "invalid_output"
                else:
                    sequence = valid_seq

        return RouterResult(
            api_code=api_code,
            sequence=sequence,
            response="",  # Filled by _generate_template_response
            route=ROUTE_ACTION_CHANNEL,
            action_latency_ms=latency,
            request_id=request_id,
            raw_llm_output=str(raw)[:200],
            _action_status=action_status,
        )

    def _generate_template_response(self, result):
        # type: (RouterResult) -> str
        """Generate template Japanese response from api_code/sequence"""
        if result.api_code is not None:
            return get_response_for_action(result.api_code)
        elif result.sequence:
            return get_response_for_sequence(result.sequence)
        return ""

    def _template_fallback(self, command, request_id, latency):
        # type: (str, str, float) -> RouterResult
        """Template fallback when action channel fails (no 7B call)

        Uses brain._generate_conversational_response() keyword matching to generate response,
        zero LLM inference overhead, avoids VRAM model switching.
        """
        response = self._brain._generate_conversational_response(command)
        return RouterResult(
            api_code=None,
            sequence=None,
            response=response,
            route=ROUTE_ACTION_FALLBACK,
            action_latency_ms=latency,
            request_id=request_id,
        )

    # ------------------------------------------------------------------
    # Shadow: Legacy as primary, Action sequentially observed
    # ------------------------------------------------------------------

    async def _shadow_route(self, command, request_id,
                            state_snapshot=None, start_time=None):
        # type: (str, str, Any, Optional[float]) -> RouterResult
        """Shadow mode: Legacy as primary (returned to user), Action channel sequentially observed + comparison logged

        Single GPU design: Ollama cannot process different models in parallel (8GB VRAM holds only one 4.7GB model).
        Parallel ensure_future would be serialized by Ollama, but timeout is calculated from request submission,
        causing the queued request to inevitably timeout. Therefore, sequential execution:

        1. Action channel runs first (for observation, swaps action model into VRAM)
        2. Legacy 7B runs second (main path, swaps 7B into VRAM -- stays in VRAM for next command)

        Invariant 4: request_id end-to-end tracing, action timeout/error explicitly recorded
        Fix #2: Unified RouterResult return (shadow_comparison is Optional field)
        Fix #3: Compare api_code + sequence, detect high_risk_divergence
        """
        # Step 1: Action channel observation (allow_fallback=False to maintain control purity)
        # Jetson 8GB unified memory: model swap ~15-25s (Ollama internal 30s + outer 45s safety net)
        dual_result = await self._action_channel_shadow(
            command, request_id, timeout=45)

        # Step 2: Legacy 7B main path (7B model stays in VRAM, optimized for next command)
        # VRAM swap-in ~15-25s + inference ~5-10s -> ollama_timeout=45s
        legacy_result = await self._legacy_route(
            command, request_id,
            state_snapshot=state_snapshot,
            start_time=start_time,
            ollama_timeout=45)

        # Step 3: Build comparison
        shadow = self._build_shadow_comparison(legacy_result, dual_result)
        legacy_result.route = ROUTE_SHADOW
        legacy_result.shadow_comparison = shadow
        return legacy_result

    async def _action_channel_shadow(self, command, request_id, timeout=20):
        # type: (str, str, int) -> RouterResult
        """Shadow-specific action channel: captures timeout/exception, returns RouterResult with status

        Timeout relaxed: Ollama internal 30s (Jetson VRAM swap-in ~15-25s + inference ~3-5s),
        outer 45s (safety net).
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
                "Shadow action channel timeout (>{}s), request_id={}".format(
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
                "Shadow action channel exception: {}, request_id={}".format(
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
        """Build shadow comparison data (synchronous, both results already complete)

        dual_status semantics:
          "ok"             -- action channel returned normally (including a=null)
          "timeout"        -- Ollama call timeout
          "error"          -- Exception (code error, network disconnect, etc.)
          "invalid_output" -- Model output illegal api_code/sequence, cleared after validation
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
        """Extract action code set from RouterResult (for divergence detection)"""
        codes = set()  # type: set
        if result.api_code is not None:
            codes.add(result.api_code)
        if result.sequence:
            codes.update(result.sequence)
        return codes
