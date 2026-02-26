#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Brain Fixed - Fixed SportClient initialization and prompt issues
"""

import contextvars
import copy
import json
import re
import time
import asyncio
import logging
import random
import threading
from typing import Dict, List, Optional, Tuple, Any, Union
from dataclasses import dataclass, field
from enum import Enum

# PR2: Coroutine-safe process_and_execute context marker
# contextvars ensures each asyncio.Task has independent count, no concurrent cross-talk
_pae_depth = contextvars.ContextVar('_pae_depth', default=0)  # type: contextvars.ContextVar[int]

from claudia.brain.action_registry import (
    ACTION_REGISTRY, VALID_API_CODES, EXECUTABLE_API_CODES,
    REQUIRE_STANDING, HIGH_ENERGY_ACTIONS,
    METHOD_MAP, ACTION_RESPONSES, SAFE_DEFAULT_PARAMS,
    get_response_for_action, get_response_for_sequence,
)
from claudia.brain.safety_compiler import SafetyCompiler, SafetyVerdict
from claudia.brain.channel_router import ChannelRouter, RouterMode, RouterResult
from claudia.brain.audit_routes import (
    ROUTE_EMERGENCY, ROUTE_HOTPATH, ROUTE_HOTPATH_REJECTED,
    ROUTE_SEQUENCE, ROUTE_DANCE, ROUTE_CONVERSATIONAL,
    ROUTE_PRECHECK_REJECTED, ROUTE_LLM_7B,
    ALL_ROUTES,
)

# Optional dependency imports
try:
    import ollama  # Python ollama library
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

try:
    from claudia.robot_controller.system_state_monitor import (
        create_system_state_monitor,
        SystemStateInfo,
        SystemState
    )
    STATE_MONITOR_AVAILABLE = True
except ImportError:
    STATE_MONITOR_AVAILABLE = False

try:
    from claudia.brain.sdk_state_provider import SDKStateProvider
    SDK_STATE_PROVIDER_AVAILABLE = True
except ImportError:
    SDK_STATE_PROVIDER_AVAILABLE = False

try:
    from claudia.brain.audit_logger import get_audit_logger, AuditEntry, sanitize_audit_input
    AUDIT_LOGGER_AVAILABLE = True
except ImportError:
    AUDIT_LOGGER_AVAILABLE = False

# Go2 firmware GetState RPC requires full-field query (single-key query returns empty response body)
# Reference: unitree_sdk2py/test/client/sport_client_example.py:101
GETSTATE_FULL_KEYS = ["state", "bodyHeight", "footRaiseHeight", "speedLevel", "gait"]

@dataclass
class BrainOutput:
    """Brain output format"""
    response: str           # Japanese TTS response
    api_code: Optional[int] = None  # Single action API
    sequence: Optional[List[int]] = None  # Action sequence
    confidence: float = 1.0
    reasoning: str = ""     # Reasoning process/route marker (for audit and debugging)
    success: bool = True    # Backward compatible (gradually deprecated, use execution_status instead)
    execution_status: Optional[str] = None  # "success" | "unknown" | "failed" | None
    raw_decision: Optional[List[int]] = None  # For Shadow: raw LLM decision before safety compilation

    def to_dict(self):
        # type: () -> Dict
        """Convert to dictionary"""
        result = {
            "response": self.response,
            "api_code": self.api_code,
            "success": self.success,
        }
        if self.sequence:
            result["sequence"] = self.sequence
        if self.reasoning:
            result["reasoning"] = self.reasoning
        if self.execution_status is not None:
            result["execution_status"] = self.execution_status
        return result

class ProductionBrain:
    """Production Brain - Using fixed model"""

    MAX_COMMAND_LENGTH = 500  # Japanese commands typically 2-20 chars; 500 is generous

    def __init__(self, use_real_hardware: bool = False):
        self.logger = self._setup_logger()
        self.use_real_hardware = use_real_hardware

        # Unified 7B model (supports environment variable override)
        import os
        self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-7b:v2.0")

        _mode = os.getenv("BRAIN_ROUTER_MODE", "dual")
        if _mode != "dual":
            self.logger.info("🧠 7B model: {}".format(self.model_7b))

        # Streamlined action cache (only culture-specific words and core commands LLM often gets wrong)
        self.hot_cache = {
            # === Culture-specific words (must keep) ===
            "ちんちん": {"response": "お辞儀します", "api_code": 1029},
            "ちんちんして": {"response": "お辞儀します", "api_code": 1029},
            "チンチン": {"response": "お辞儀します", "api_code": 1029},
            "拜年": {"response": "お辞儀します", "api_code": 1029},

            # === Multilingual emergency stop (safety critical) ===
            "止まって": {"response": "止まります", "api_code": 1003},
            "止まれ": {"response": "止まります", "api_code": 1003},
            "停止": {"response": "止まります", "api_code": 1003},
            "停下": {"response": "止まります", "api_code": 1003},
            "stop": {"response": "止まります", "api_code": 1003},
            "halt": {"response": "止まります", "api_code": 1003},
            "ダンプ": {"response": "ダンプモード", "api_code": 1001},  # Emergency damp
            "damp": {"response": "ダンプモード", "api_code": 1001},
            "阻尼": {"response": "ダンプモード", "api_code": 1001},
            "バランス": {"response": "バランスします", "api_code": 1002},  # Emergency balance
            "balance": {"response": "バランスします", "api_code": 1002},
            "平衡": {"response": "バランスします", "api_code": 1002},

            # === Core basic commands ===
            "座って": {"response": "座ります", "api_code": 1009},
            "おすわり": {"response": "お座りします", "api_code": 1009},
            "立って": {"response": "立ちます", "api_code": 1004},
            "タッテ": {"response": "立ちます", "api_code": 1004},
            "立ってください": {"response": "立ちます", "api_code": 1004},
            "伏せて": {"response": "伏せます", "api_code": 1005},
            "横になって": {"response": "横になります", "api_code": 1005},
            "横になってください": {"response": "横になります", "api_code": 1005},

            # === Core performance actions ===
            "お手": {"response": "こんにちは", "api_code": 1016},
            "挨拶": {"response": "挨拶します", "api_code": 1016},
            "挨拶して": {"response": "挨拶します", "api_code": 1016},
            "こんにちは": {"response": "こんにちは", "api_code": 1016},
            "hello": {"response": "挨拶します", "api_code": 1016},
            "ストレッチ": {"response": "伸びをします", "api_code": 1017},
            "伸び": {"response": "伸びをします", "api_code": 1017},
            "ダンス": {"response": "踊ります", "api_code": 1022},
            "踊って": {"response": "踊ります", "api_code": 1022},
            "ハート": {"response": "ハートします", "api_code": 1036},
            "比心": {"response": "ハートします", "api_code": 1036},

            # === Friendly greetings -> Hello(1016) ===
            "おはよう": {"response": "おはようございます！挨拶します", "api_code": 1016},
            "おはようございます": {"response": "おはようございます！挨拶します", "api_code": 1016},
            "こんばんは": {"response": "こんばんは！挨拶します", "api_code": 1016},
            "こんばんわ": {"response": "こんばんは！挨拶します", "api_code": 1016},
            "さようなら": {"response": "さようなら！またね。", "api_code": 1016},
            "おやすみ": {"response": "おやすみなさい！", "api_code": 1016},
            "おやすみなさい": {"response": "おやすみなさい！", "api_code": 1016},
            "good morning": {"response": "おはようございます！挨拶します", "api_code": 1016},
            "good evening": {"response": "こんばんは！挨拶します", "api_code": 1016},
            "good night": {"response": "おやすみなさい！挨拶します", "api_code": 1016},
            "goodbye": {"response": "さようなら！またね。", "api_code": 1016},
            "bye": {"response": "さようなら！またね。", "api_code": 1016},
            "早上好": {"response": "おはようございます！挨拶します", "api_code": 1016},
            "晚上好": {"response": "こんばんは！挨拶します", "api_code": 1016},
            "晚安": {"response": "おやすみなさい！", "api_code": 1016},
            "再见": {"response": "さようなら！またね。", "api_code": 1016},

            # === Compliments -> Heart(1036) ===
            "かわいい": {"response": "ありがとう！ハートします", "api_code": 1036},
            "可愛い": {"response": "ありがとう！ハートします", "api_code": 1036},
            "すごい": {"response": "ありがとう！ハートします", "api_code": 1036},
            "凄い": {"response": "ありがとう！ハートします", "api_code": 1036},
            "いい子": {"response": "ありがとう！ハートします", "api_code": 1036},
            "可爱": {"response": "ありがとう！ハートします", "api_code": 1036},
            "cute": {"response": "ありがとう！ハートします", "api_code": 1036},

            # === Special words (easily misunderstood) ===
            "お辞儀": {"response": "お辞儀します", "api_code": 1029},  # Bow action -> Scrape (front paw bow)
            "礼": {"response": "お辞儀します", "api_code": 1029},
            "ジャンプ": {"response": "前跳します", "api_code": 1031},
            # === Dual-layer whitelist: Pose(1028) intentional design difference ===
            # LLM path: VALID_API_CODES (has_params=True -> excluded)
            #   -> Blocks risk of LLM hallucinating parameter values
            # hot_cache path: EXECUTABLE_API_CODES (safe_default_params=(True,) -> allowed)
            #   -> User direct commands execute with safe default values
            # This dual standard confirmed in security audit R3 (accepted risk).
            # Verification tests: test_action_registry.py::test_pose_1028_in_executable_not_valid
            #           test_safety_regression.py::test_pose_1028_intentional_whitelist_difference
            "ポーズ": {"response": "ポーズします", "api_code": 1028},
        }

        # Auto-add ASR kana variants to hot_cache (KANA_ALIASES sole reference)
        # Only adds when kanji key exists and kana key is not yet registered
        for kana, kanji in self.KANA_ALIASES.items():
            if kanji in self.hot_cache and kana not in self.hot_cache:
                self.hot_cache[kana] = self.hot_cache[kanji]

        # Startup validation: ensure all hot_cache api_code values are in EXECUTABLE_API_CODES
        for _hc_key, _hc_entry in self.hot_cache.items():
            _hc_api = _hc_entry.get("api_code")
            if _hc_api is not None and _hc_api not in EXECUTABLE_API_CODES:
                raise ValueError(
                    "hot_cache key '{}' has api_code={} not in EXECUTABLE_API_CODES".format(
                        _hc_key, _hc_api
                    )
                )

        # Complex sequence detection keywords - extended Japanese conjunctions
        self.sequence_keywords = [
            # Chinese conjunctions
            "然后", "接着", "一套", "表演",

            # Japanese conjunctions (focus: extended)
            "てから", "その後", "それから",    # Then, after that
            "したら", "すれば", "なら",        # If...then...
            "次に", "つぎに", "それで",        # Next
            "してから", "したあと",           # After doing...

            # Combined action keywords
            "連続", "れんぞく",               # Continuous
            "パフォーマンス", "芸", "技",      # Performance, skill
            "一緒に", "同時に",               # Together, simultaneously
            "順番に", "順序",                 # In order, sequence
        ]

        # SportClient connection (if real hardware)
        self.sport_client = None
        if use_real_hardware:
            self._init_sport_client()

        # Robot state management
        self.robot_state = "unknown"  # unknown, standing, sitting, lying
        # Standing prerequisite list migrated to action_registry.REQUIRE_STANDING,
        # SafetyCompiler handles it automatically in compile().

        # State monitor
        # Hardware mode: Use SDKStateProvider (queries via SportClient RPC, avoids DDS domain conflict)
        # Simulation mode: Use ROS2 state_monitor (no domain conflict risk)
        self.state_monitor = None
        if use_real_hardware and self.sport_client is not None and SDK_STATE_PROVIDER_AVAILABLE:
            # Hardware mode + SDK available: skip ROS2 monitor to avoid rmw_create_node domain conflict
            try:
                self.state_monitor = SDKStateProvider(
                    rpc_call_fn=self._rpc_call,
                    logger=self.logger,
                )
                self.state_monitor.start_polling(interval=2.0)
                self.logger.info("SDK state provider started (RPC polling, interval 2.0s)")
            except Exception as e:
                self.logger.warning(f"SDK state provider startup failed: {e}")
                self.state_monitor = None
        elif not use_real_hardware and STATE_MONITOR_AVAILABLE:
            # Simulation mode: can try ROS2 monitor
            try:
                self.state_monitor = create_system_state_monitor(
                    node_name="claudia_brain_monitor",
                    update_rate=5.0  # 5Hz update
                )
                if self.state_monitor.initialize():
                    self.state_monitor.start_monitoring()
                    self.logger.info("✅ ROS2 state monitor started")
                else:
                    self.logger.warning("⚠️ ROS2 state monitor initialization failed, using default state")
            except Exception as e:
                self.logger.warning(f"⚠️ ROS2 state monitor unavailable: {e}")
                self.state_monitor = None
        else:
            reason = "SDK unavailable" if use_real_hardware else "state monitor module unavailable"
            self.logger.warning(f"⚠️ State monitor not started: {reason}")

        # Safety compiler (unified safety pipeline)
        allow_high_risk = os.getenv("SAFETY_ALLOW_HIGH_RISK", "0") == "1"
        self.safety_compiler = SafetyCompiler(allow_high_risk=allow_high_risk)
        if allow_high_risk:
            self.logger.warning("!! SAFETY_ALLOW_HIGH_RISK=1: high-risk actions enabled !!")
        else:
            self.logger.info("SafetyCompiler loaded (high-risk actions disabled)")

        # RPC lock (SportClient is not thread-safe, all RPC calls must go through _rpc_call)
        self._rpc_lock = threading.RLock()
        self._current_timeout = 10.0  # Track current SDK timeout value

        # Command-level serial lock (PR1 introduced framework, PR2 forced migration of all callers)
        self._command_lock = asyncio.Lock()

        # Audit logger
        if AUDIT_LOGGER_AVAILABLE:
            self.audit_logger = get_audit_logger()
            self.logger.info("✅ Audit logger started (logs/audit/)")
        else:
            self.audit_logger = None
            self.logger.warning("⚠️ Audit logger unavailable")

        # Posture tracking (for simulation mode state accuracy)
        self.last_posture_standing = False  # Initially assume sitting posture
        self.last_executed_api = None       # Last executed API code

        # PR2: Dual-channel router (controlled by BRAIN_ROUTER_MODE env var)
        router_mode_str = os.getenv("BRAIN_ROUTER_MODE", "dual")
        try:
            self._router_mode = RouterMode(router_mode_str)
        except ValueError:
            self.logger.warning(
                "Invalid BRAIN_ROUTER_MODE='{}', downgrading to legacy".format(router_mode_str))
            self._router_mode = RouterMode.LEGACY
        self._channel_router = ChannelRouter(self, self._router_mode)

        # Non-legacy mode: verify action model exists
        if self._router_mode != RouterMode.LEGACY:
            if not self._verify_action_model():
                self.logger.warning(
                    "Action model unavailable, downgrading BRAIN_ROUTER_MODE -> legacy")
                self._router_mode = RouterMode.LEGACY
                self._channel_router = ChannelRouter(self, self._router_mode)

        self.logger.info("🧠 ProductionBrain initialization complete")
        self.logger.info(f"   Hardware: {'real robot' if use_real_hardware else 'simulation'}")
        self.logger.info(f"   Routing: {self._router_mode.value}")

    def _setup_logger(self) -> logging.Logger:
        """Set up logger"""
        logger = logging.getLogger("ProductionBrain")
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('🧠 %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        # Complete with own handler, prevent duplicate output with root handler
        logger.propagate = False
        return logger

    def _kana_to_kanji(self, text):
        """Normalize ASR kana output to kanji (KANA_ALIASES sole reference)

        Applied before SEQUENCE_HOTPATH substring match.
        hot_cache is auto-expanded in __init__, so not needed there.
        """
        for kana, kanji in self.KANA_ALIASES.items():
            text = text.replace(kana, kanji)
        return text

    def _init_sport_client(self):
        """Fixed SportClient initialization - includes correct network configuration"""
        try:
            import sys
            import os

            # Add correct paths (derived from project root, avoiding hardcoded paths)
            _project_root = os.path.abspath(os.path.join(
                os.path.dirname(__file__), '..', '..', '..'))
            sys.path.append(_project_root)
            _sdk_path = os.path.join(_project_root, 'unitree_sdk2_python')
            if os.path.isdir(_sdk_path):
                sys.path.append(_sdk_path)

            # CycloneDDS path unification: prefer env var, fallback to project directory
            # Resolves path inconsistency between start_production_brain.sh and setup_cyclonedds.sh
            cyclone_home = os.environ.get('CYCLONEDDS_HOME', '')
            if not cyclone_home or not os.path.isdir(cyclone_home):
                # Try two known paths in priority order
                candidates = [
                    os.path.join(_project_root, 'cyclonedds', 'install'),
                    os.path.expanduser('~/cyclonedds/install'),
                ]
                for candidate in candidates:
                    if os.path.isdir(candidate):
                        cyclone_home = candidate
                        break
                else:
                    cyclone_home = candidates[0]  # Final fallback
            os.environ['CYCLONEDDS_HOME'] = cyclone_home

            # Set LD_LIBRARY_PATH
            ld_path = os.environ.get('LD_LIBRARY_PATH', '')
            cyclone_lib = os.path.join(cyclone_home, 'lib')
            unitree_lib = os.path.join(_project_root, 'cyclonedds_ws', 'install', 'unitree_sdk2', 'lib')

            if cyclone_lib not in ld_path:
                os.environ['LD_LIBRARY_PATH'] = f"{cyclone_lib}:{unitree_lib}:{ld_path}"

            # Set RMW implementation
            os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

            # Set network configuration - using official recommended inline config!
            os.environ['CYCLONEDDS_URI'] = '''<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'''

            # Import necessary modules
            from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient

            # Initialize DDS channel factory - this is the critical step!
            self.logger.info("📡 DDS channel factory initialization (eth0)...")
            ChannelFactoryInitialize(0, "eth0")

            # Create SportClient instance
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

            # Test connection - use read-only API, with retries (DDS needs time to establish connection)
            import time

            # P0-5 + retry: GetState probe, 3 retries, incremental wait
            # Go2 firmware requires full-field query (single-key query returns empty response causing JSON parse failure)
            test_result = None
            probe_ok = False
            MAX_PROBE_RETRIES = 3
            for attempt in range(MAX_PROBE_RETRIES):
                wait_sec = 1.0 + attempt * 1.0  # 1s, 2s, 3s
                time.sleep(wait_sec)
                try:
                    test_result, probe_data = self.sport_client.GetState(GETSTATE_FULL_KEYS)
                    if self._is_valid_getstate_probe(test_result, probe_data):
                        probe_ok = True
                        self.logger.info(
                            "   GetState probe succeeded (attempt {}/{})".format(
                                attempt + 1, MAX_PROBE_RETRIES
                            )
                        )
                        break  # Return code + data both valid, exit retry
                    else:
                        # RPC returned result but not valid (code!=0 or data empty)
                        if attempt < MAX_PROBE_RETRIES - 1:
                            self.logger.info(
                                "   GetState probe #{}: code={}, data={}, retrying in {}s...".format(
                                    attempt + 1, test_result,
                                    'empty' if not probe_data else type(probe_data).__name__,
                                    1.0 + (attempt + 1) * 1.0
                                )
                            )
                        else:
                            self.logger.warning(
                                "   GetState probe: {} retries all returned invalid results (code={})".format(
                                    MAX_PROBE_RETRIES, test_result
                                )
                            )
                except (json.JSONDecodeError, ValueError):
                    # RPC response empty -- DDS ready but sport service not fully initialized
                    if attempt < MAX_PROBE_RETRIES - 1:
                        self.logger.info(
                            "   GetState probe #{}: response empty, retrying in {}s...".format(
                                attempt + 1, 1.0 + (attempt + 1) * 1.0
                            )
                        )
                    else:
                        self.logger.warning("   GetState probe: all {} retries failed (JSON parse error)".format(
                            MAX_PROBE_RETRIES
                        ))
                        test_result = -1
                except Exception as e:
                    if attempt < MAX_PROBE_RETRIES - 1:
                        self.logger.info(
                            "   GetState probe #{} failed: {}, retrying in {}s...".format(
                                attempt + 1, e, 1.0 + (attempt + 1) * 1.0
                            )
                        )
                    else:
                        self.logger.warning("   GetState probe: all {} retries failed: {}".format(
                            MAX_PROBE_RETRIES, e
                        ))
                        test_result = -1

            # Prevent "code=0 + empty/invalid data" from being misjudged as connection success
            if not probe_ok and test_result == 0:
                self.logger.warning("   GetState probe: code=0 but data invalid, treating as failure")
                test_result = -1

            try:
                # Analyze return code
                if test_result == 0:
                    self.logger.info("✅ SportClient initialization succeeded — robot connected")
                    self.logger.info(f"   Network: eth0")
                    self.logger.info(f"   Local IP: 192.168.123.18")
                    self.logger.info(f"   Robot IP: 192.168.123.161")
                    self.logger.info(f"   Test response code: {test_result}")

                elif test_result == 3103:
                    # APP occupation issue - this is the most common issue
                    self.logger.error("="*60)
                    self.logger.error("❌ APP is occupying sport_mode (error 3103)")
                    self.logger.error("")
                    self.logger.error("Cause: SDK and APP cannot control the robot simultaneously")
                    self.logger.error("This is Unitree's safety design, not a malfunction")
                    self.logger.error("")
                    self.logger.error("Resolution steps:")
                    self.logger.error("1. Close the Unitree Go APP on your phone")
                    self.logger.error("2. Long-press the robot power button to restart")
                    self.logger.error("3. Wait 30 seconds then re-run the program")
                    self.logger.error("")
                    self.logger.error("または: ./start_sdk_exclusive.sh")
                    self.logger.error("="*60)
                    self.logger.warning("Switching to simulation mode...")
                    self._init_mock_client()
                    return  # Use mock client

                elif test_result == 3203:
                    self.logger.warning("⚠️ API not implemented (3203) — some actions may be unsupported")
                    self.logger.info("   SportClient created, continuing...")

                else:
                    self.logger.warning(f"⚠️ Connection test response code: {test_result}")
                    self.logger.info("   SportClient created, continuing...")

            except Exception as e:
                self.logger.warning(f"⚠️ Connection test exception: {e}")
                self.logger.info("   SportClient created, continuing...")

        except ImportError as e:
            self.logger.error(f"❌ Import error: {e}")
            self.logger.info("   Simulating with MockSportClient")
            self._init_mock_client()

        except Exception as e:
            self.logger.error(f"❌ SportClient initialization failed: {e}")
            self.logger.info("   Hint: robot may not be connected")
            self.logger.info("   Simulating with MockSportClient")
            self._init_mock_client()

    def _is_valid_getstate_probe(self, code: Any, data: Any) -> bool:
        """GetState connectivity probe validity check.

        Valid conditions:
          - code == 0
          - data is a non-empty dict (Go2 firmware returns structured fields)
        """
        if code != 0:
            return False
        if not isinstance(data, dict):
            return False
        return len(data) > 0

    def _init_mock_client(self):
        """Initialize mock client"""
        try:
            from claudia.brain.mock_sport_client import MockSportClient
            self.sport_client = MockSportClient()
            self.sport_client.Init()
            self.logger.info("🎭 MockSportClient initialization complete (simulation)")
            # Keep hardware mode flag, but use mock client
            # This way the user knows the system tried hardware control, just using mock instead
        except Exception as e:
            self.logger.error(f"❌ MockSportClient initialization failed: {e}")
            self.sport_client = None
            self.use_real_hardware = False

    def _rpc_call(self, method_name, *args, **kwargs):
        """Unified RPC wrapper -- all SportClient calls must go through this method

        Features:
          - RLock guarantees thread safety (supports nested calls from same thread)
          - Stack-based timeout save/restore (timeout_override doesn't pollute global state)
          - Exception safe: even if SetTimeout fails, tracking value is restored

        Args:
            method_name: SportClient method name (e.g., "StandUp", "Dance1")
            *args: Method arguments
            **kwargs: timeout_override=float can temporarily override timeout
        """
        timeout_override = kwargs.pop("timeout_override", None)
        with self._rpc_lock:
            previous_timeout = self._current_timeout
            timeout_changed = False
            if timeout_override is not None:
                try:
                    self.sport_client.SetTimeout(timeout_override)
                    self._current_timeout = timeout_override
                    timeout_changed = True
                except Exception:
                    pass  # Keep original timeout if SetTimeout fails
            try:
                method = getattr(self.sport_client, method_name)
                return method(*args)
            finally:
                if timeout_changed:
                    try:
                        self.sport_client.SetTimeout(previous_timeout)
                        self._current_timeout = previous_timeout
                    except Exception:
                        # SDK restore failed, at least keep tracking value consistent
                        self._current_timeout = previous_timeout

    # === Input normalization: trailing punctuation strip characters (unified definition) ===
    _TRAILING_PUNCTUATION = "！!。.．、，,？?…~"

    # === ASR kana alias table (sole definition point) ===
    # ASR speech recognition may output kana (hiragana) instead of kanji.
    # This mapping normalizes input text to match hot_cache / SEQUENCE_HOTPATH /
    # dance_commands keys.
    # Edit only here when adding new ASR kana overrides.
    # key = kana form ASR may output, value = master dictionary key form (containing kanji)
    KANA_ALIASES = {
        # Basic actions
        "すわって": "座って",
        "たって": "立って",
        "ふせて": "伏せて",
        "よこになって": "横になって",
        # Performance actions
        "あいさつ": "挨拶",
        "のび": "伸び",
        "おどって": "踊って",
        "おどる": "踊る",
        # Special cases
        "おじぎ": "お辞儀",
        "れい": "礼",
        "ひしん": "比心",
        # Dialogue patterns (for _generate_conversational_response)
        "なまえ": "名前",
        "だれ": "誰",
        "きみ": "君",
        # ASR polite form variants ("~kudasai" "~shite" -> normalize to base form)
        "たってください": "立って",
        "すわってください": "座って",
        "おどってください": "踊って",
        "よこになってください": "横になって",
        "おてして": "お手",
        "あいさつして": "挨拶して",
        "はーとして": "ハート",
    }

    # === Emergency stop commands (single source of truth) ===
    # Referenced by both process_and_execute / process_command.
    # key = command text (matched after strip().lower()), value = Japanese response.
    # All keys uniformly map to StopMove(1003), executed by _handle_emergency().
    # Includes ASR kana variants (とまれ/とめて/ていし/きんきゅうていし).
    EMERGENCY_COMMANDS = {
        # Japanese (kanji)
        "止まれ": "止まります",
        "止めて": "止まります",
        "止まって": "止まります",
        "緊急停止": "緊急停止しました",
        "やめて": "止まります",
        # Japanese (ASR kana variants)
        "とまれ": "止まります",
        "とめて": "止まります",
        "とまって": "止まります",
        "きんきゅうていし": "緊急停止しました",
        # Katakana
        "ストップ": "止まります",
        # English
        "stop": "止まります",
        "halt": "止まります",
        "emergency": "緊急停止しました",
        # Chinese
        "停止": "止まります",
        "停下": "止まります",
    }

    # === Sequence predefinitions (avoid LLM calls for common action combinations) ===
    # Referenced by process_command on each call, no longer rebuilt each time.
    SEQUENCE_HOTPATH = {
        # Stand + action series
        '立ってから挨拶': [1004, 1016],
        '立って挨拶': [1004, 1016],
        '立ってそして挨拶': [1004, 1016],
        '立ってこんにちは': [1004, 1016],
        '立ってからハート': [1004, 1036],
        '立ってハート': [1004, 1036],
        '立ってダンス': [1004, 1023],
        '立ってから踊る': [1004, 1023],
        # Sit + action series
        '座ってから挨拶': [1009, 1016],
        '座って挨拶': [1009, 1016],
        '座ってこんにちは': [1009, 1016],
        # English
        'stand and hello': [1004, 1016],
        'stand then hello': [1004, 1016],
        'sit and hello': [1009, 1016],
        # Chinese
        '站立然后问好': [1004, 1016],
        '坐下然后问好': [1009, 1016],
    }

    async def process_and_execute(self, command):
        # type: (str) -> BrainOutput
        """Atomic command processing + execution entry point (PR1 introduced framework, PR2 forced all entries to use this)

        Emergency commands bypass the lock and execute directly; normal commands are serialized within the lock.
        execution_status semantics:
          - "success": Action executed successfully
          - "unknown": RPC timeout (robot may still be executing)
          - "failed": Action execution failed
          - "skipped": Text-only response, no action executed
        """
        # Input length validation (reject excessively long input before any processing)
        if len(command.strip()) > self.MAX_COMMAND_LENGTH:
            self.logger.warning("Command too long (%d chars), rejected", len(command.strip()))
            return BrainOutput(
                response="コマンドが長すぎます。短くしてください。",
                api_code=None,
                execution_status="skipped",
            )

        # contextvars marker: coroutine-safe, no concurrent cross-talk
        token = _pae_depth.set(_pae_depth.get(0) + 1)
        try:
            cmd_lower = command.strip().lower().rstrip(self._TRAILING_PUNCTUATION)
            if cmd_lower in self.EMERGENCY_COMMANDS:
                # === Emergency bypass: does not acquire _command_lock (intentional design) ===
                # Reason: Emergency stop must execute with minimum delay.
                #   Waiting for _command_lock could block for several seconds.
                # Risk: _rpc_lock is shared, so if another RPC is executing,
                #   StopMove may be delayed by ~1s (waiting for fast action RPC to complete).
                # Acceptance: In single-user Jetson environment, concurrent RPCs are rare,
                #   and ~1s delay is acceptable. Confirmed in security audit R4.
                return await self._handle_emergency(command)

            async with self._command_lock:
                brain_output = await self.process_command(command)
                if brain_output.api_code or brain_output.sequence:
                    result = await self.execute_action(brain_output)
                    if result is True:
                        brain_output.execution_status = "success"
                    elif result == "unknown":
                        brain_output.execution_status = "unknown"
                    else:
                        brain_output.execution_status = "failed"
                else:
                    brain_output.execution_status = "skipped"
                return brain_output
        finally:
            _pae_depth.reset(token)

    async def _handle_emergency(self, command):
        # type: (str) -> BrainOutput
        """Emergency stop handling -- does not acquire lock, directly calls StopMove

        Return code semantics:
          - sport_client does not exist (simulation mode) -> success (no physical stop needed)
          - RPC returns 0 or -1 (already stopped) -> success
          - RPC returns other value -> failed
          - RPC exception -> failed
        """
        self.logger.warning("!! Emergency stop: {} !!".format(command))
        exec_status = "success"  # Default: no physical stop needed in simulation mode
        response = "緊急停止しました"
        if self.sport_client:
            try:
                result = self._rpc_call("StopMove")
                if isinstance(result, tuple):
                    result = result[0]
                if result == 0 or result == -1:
                    exec_status = "success"
                else:
                    exec_status = "failed"
                    response = "緊急停止を試みましたが、エラーが発生しました（コード:{}）".format(result)
                    self.logger.error("Emergency stop abnormal response: {}".format(result))
            except Exception as e:
                exec_status = "failed"
                response = "緊急停止に失敗しました"
                self.logger.error("Emergency stop RPC failed: {}".format(e))
        output = BrainOutput(
            response=response,
            api_code=1003,
            reasoning="emergency",
            execution_status=exec_status,
        )
        self._log_audit(
            command, output,
            route=ROUTE_EMERGENCY,
            elapsed_ms=0.0,
            cache_hit=False,
            model_used="bypass",
            current_state=None,
            llm_output=None,
            safety_verdict="emergency_bypass",
        )
        return output

    def _is_complex_command(self, command: str) -> bool:
        """Determine whether the command is complex"""
        return any(keyword in command for keyword in self.sequence_keywords)

    def _normalize_battery(self, level):
        # type: (Optional[float]) -> Optional[float]
        """Battery normalization: clamp sensor precision boundary values to 1.0

        Values >1.0 (e.g., 1.01) may come from sensor precision errors. Passing through
        directly would cause SafetyCompiler fail-safe to reject all actions, affecting usability.
        Clamp to 1.0 here with a warning, balancing safety and usability.
        Obviously anomalous values (>1.5) still log error for upstream bug investigation.
        """
        if level is None:
            return None
        if level > 1.0:
            if level > 1.5:
                self.logger.error(
                    "battery_level={} >> 1.0, upstream normalization anomaly! "
                    "Clamped to 1.0 but data source needs investigation".format(level)
                )
            else:
                self.logger.warning(
                    "battery_level={} > 1.0 (sensor precision), clamped to 1.0".format(level)
                )
            return 1.0
        return level

    def _sanitize_response(self, r: str) -> str:
        """
        Sanitize LLM output response field to prevent meaningless or non-Japanese output

        Fixes edge case issues:
        - "今日はいい天気ですね" -> " godee" (bad)
        - "ちんちん" -> " pong" (bad)

        Args:
            r: LLM output response field

        Returns:
            Sanitized response, returns default response if invalid
        """
        if not r or not r.strip():
            return "すみません、よく分かりません"

        r = r.strip()

        # Check for Japanese characters (hiragana, katakana, kanji)
        has_hiragana = any('\u3040' <= ch <= '\u309f' for ch in r)
        has_katakana = any('\u30a0' <= ch <= '\u30ff' for ch in r)
        has_kanji = any('\u4e00' <= ch <= '\u9faf' for ch in r)
        has_japanese = has_hiragana or has_katakana or has_kanji

        # If no Japanese characters, return default response
        if not has_japanese:
            self.logger.warning(f"⚠️ No Japanese in LLM output: '{r}' -> default response")
            return "すみません、よく分かりません"

        # Check for nonsense words (godee, pong, etc.)
        # Use \b word boundary matching to avoid 'ok' false-matching 'tokyo' and similar valid substrings
        nonsense_patterns = [r'\bgodee\b', r'\bpong\b', r'\bhi\b', r'\bhello\b',
                             r'\bok\b', r'\byes\b', r'\bno\b']
        r_lower = r.lower()
        if any(re.search(pat, r_lower) for pat in nonsense_patterns):
            self.logger.warning(f"⚠️ Nonsense word detected in LLM output: '{r}' -> default response")
            return "すみません、よく分かりません"

        return r

    def _quick_safety_precheck(self, command, state):
        # type: (str, Optional[Any]) -> Optional[str]
        """DEPRECATED in V2: Use SafetyCompiler.compile() instead.
        Code retained for reference, no longer called by process_command.

        Quick safety pre-check: executed before LLM (millisecond-level)

        Args:
            command: User command
            state: Current state (already normalized)

        Returns:
            If unsafe, returns rejection reason; otherwise returns None (allow to continue)
        """
        if not state or state.battery_level is None:
            return None

        b = state.battery_level  # Already normalized to 0.0-1.0
        cmd = command.lower()

        # Critically low battery (<=10%): only allow sit/stop/stand keywords
        if b <= 0.10:
            safe_kw = ('sit', 'stop', 'stand', '座', '立', '止', 'やめ', 'とまれ')
            if not any(k in cmd for k in safe_kw):
                return f"Battery level is critically low ({b*100:.0f}%). Only Sit/Stand/Stop commands are available."

        # Low battery (<=20%): reject obvious high-energy keywords
        if b <= 0.20:
            high_kw = ('flip', '転', 'jump', '跳', 'pounce', '飛', 'かっこいい')
            if any(k in cmd for k in high_kw):
                return f"Battery level is low ({b*100:.0f}%). High-energy actions are prohibited."

        return None  # Allow to continue

    def _final_safety_gate(self, api_code, state):
        # type: (Optional[int], Optional[Any]) -> Tuple[Optional[int], str]
        """DEPRECATED in V2: Use SafetyCompiler.compile() instead.
        Code retained for reference, no longer called by process_command.

        Final safety gate: hard check before execution (does not depend on LLM/SafetyValidator)

        Args:
            api_code: Action code returned by LLM
            state: Current state (already normalized)

        Returns:
            (safe_api_code, reason) - If rejected: (None, reason); If downgraded: (new_code, reason)
        """
        if api_code is None or not state or state.battery_level is None:
            return api_code, "ok"

        b = state.battery_level  # Already normalized to 0.0-1.0
        HIGH = (1030, 1031, 1032)  # Flip, Jump, Pounce

        # Critically low battery (<=10%): only allow 1003/1009/1004
        if b <= 0.10:
            if api_code not in (1003, 1009, 1004, None):
                return None, f"Final gate: Battery {b*100:.0f}% too low for action {api_code}"

        # Low battery (<=20%): prohibit high-energy actions
        elif b <= 0.20:
            if api_code in HIGH:
                return None, f"Final gate: Battery {b*100:.0f}% insufficient for high-energy action {api_code}"

        # Medium battery (<=30%): downgrade high-energy actions to Dance
        elif b <= 0.30:
            if api_code in HIGH:
                return 1023, f"Final gate: Downgraded {api_code}→Dance at {b*100:.0f}%"

        return api_code, "ok"

    def _is_conversational_query(self, command: str) -> bool:
        """
        Detect whether the command is a conversational query (should not return action API)

        Args:
            command: User command

        Returns:
            True for conversational query, False for action command
        """
        cmd = command.strip().lower()
        # ASR kana normalization: "おなまえは" -> "お名前は" -> matches '名前'
        cmd = self._kana_to_kanji(cmd)

        # Conversational keyword patterns
        CONVERSATIONAL_PATTERNS = [
            # Japanese (compliments moved to hot_cache: かわいい/すごい -> Heart(1036))
            # Friendly greetings also moved to hot_cache: おはよう/こんばんは etc. -> Hello(1016)
            'あなた', '君', 'きみ', '名前', 'なまえ', '誰', 'だれ',
            '何', 'なに', 'どう', 'なぜ', 'いつ', 'どこ',
            'ありがとう', 'ごめん',
            # English (cute moved to hot_cache -> Heart, greetings moved to hot_cache -> Hello)
            'who are you', 'what is your name', 'your name',
            'who', 'what', 'why', 'when', 'where', 'how',
            'you are', "you're", 'thank you', 'thanks', 'sorry',
            'cool', 'awesome', 'nice',
            # Chinese (可爱 moved to hot_cache -> Heart, greetings moved to hot_cache -> Hello)
            '你是', '你叫', '你的名字', '谁', '什么', '为什么',
            '怎么', '哪里', '什么时候',
            '厉害', '谢谢', '对不起',
        ]

        # Check for conversational keywords
        for pattern in CONVERSATIONAL_PATTERNS:
            if pattern in cmd:
                return True

        return False

    def _generate_conversational_response(self, command: str) -> str:
        """
        Generate conversational response (no action executed)

        Args:
            command: User command

        Returns:
            Friendly conversational response
        """
        cmd = command.strip().lower()
        # ASR kana normalization: "おなまえは" -> "お名前は" -> matches '名前'
        cmd = self._kana_to_kanji(cmd)

        # Name/identity related
        if any(k in cmd for k in ['あなた', '誰', '名前', 'who', 'your name', '你是', '你叫']):
            return "私はClaudiaです。Unitree Go2のAIアシスタントです。"

        # Compliments -> handled by hot_cache (Heart 1036)
        # Greetings -> handled by hot_cache (Hello 1016)

        # Thanks related (remains in CONVERSATIONAL_PATTERNS, not in hot_cache)
        if any(k in cmd for k in ['ありがとう', 'thank', '谢谢']):
            return "どういたしまして！"

        # Default conversational response
        return "はい、何でしょうか？"

    def _compile_safety(self, candidate, state_snapshot, snapshot_monotonic_ts):
        # type: (List[int], Optional[Any], Optional[float]) -> SafetyVerdict
        """SafetyCompiler unified call wrapper -- extracts parameters from state_snapshot

        fail-closed policy: state_snapshot=None -> battery=0.0, is_standing=False,
        only SAFE_ACTIONS can pass.
        """
        _batt = state_snapshot.battery_level if state_snapshot else 0.0
        _stand = state_snapshot.is_standing if state_snapshot else False
        _ts = snapshot_monotonic_ts if state_snapshot else None
        if not state_snapshot:
            self.logger.warning("No state monitor: fail-safe compile (battery=0.0)")
        return self.safety_compiler.compile(
            candidate, _batt, _stand, snapshot_timestamp=_ts,
        )

    def _verify_action_model(self):
        # type: () -> bool
        """Verify Action model is available (one-time check at startup)"""
        if not OLLAMA_AVAILABLE:
            return False
        try:
            ollama.show(self._channel_router._action_model)
            self.logger.info("Action model verified: {}".format(
                self._channel_router._action_model))
            return True
        except Exception as e:
            self.logger.warning("Action model unavailable: {}".format(e))
            return False

    async def _ensure_model_loaded(self, model, num_ctx=2048):
        # type: (str, int) -> bool
        """Pre-inference check: ensure target model is loaded in GPU VRAM

        Checks if ollama.ps() contains the target model. If not in VRAM, sends a
        lightweight num_predict=1 request to trigger model loading (waits up to 60s).
        This way subsequent inference timeout only needs to cover pure inference time,
        not model swapping.

        Returns:
            True=model ready, False=loading failed (caller can still attempt inference)
        """
        if not OLLAMA_AVAILABLE:
            return True  # Cannot check, pass optimistically

        try:
            ps_result = ollama.ps()
            loaded_names = [m.model for m in (ps_result.models or [])]
            # ollama.ps() returns full name with tag (e.g., "model:latest")
            # Input model may not have tag, need base name comparison
            loaded_base = [n.split(':')[0] for n in loaded_names]
            model_base = model.split(':')[0]
            if model in loaded_names or model_base in loaded_base:
                return True  # Already in VRAM

            # Model not in VRAM -> trigger loading
            self.logger.warning(
                "Model {} not in GPU VRAM (current: {}), triggering preload..."
                .format(model, loaded_names or "none")
            )

            _num_ctx = num_ctx

            def _sync_preload():
                ollama.chat(
                    model=model,
                    messages=[{'role': 'user', 'content': 'hi'}],
                    format='json',
                    options={'num_predict': 1, 'num_ctx': _num_ctx},
                    keep_alive='30m',
                )

            loop = asyncio.get_running_loop()
            start = time.monotonic()
            await asyncio.wait_for(
                loop.run_in_executor(None, _sync_preload),
                timeout=60,
            )
            elapsed_ms = (time.monotonic() - start) * 1000
            self.logger.info("Model {} preload complete ({:.0f}ms)".format(model, elapsed_ms))
            return True

        except asyncio.TimeoutError:
            self.logger.error("Model {} preload timeout (60s)".format(model))
            return False
        except (ConnectionError, OSError) as e:
            # Ollama process unreachable, subsequent inference will definitely fail, fast-fail
            self.logger.error("Model preload connection failed (Ollama not running?): {}".format(e))
            return False
        except Exception as e:
            self.logger.warning("Model preload check exception: {}".format(e))
            return True  # Non-connection exceptions pass optimistically, let inference handle it

    async def _call_ollama_v2(self, model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
        # type: (str, str, int, int, int, Any) -> Optional[Dict]
        """Call Ollama LLM inference

        Args:
            model: Ollama model name
            command: User input
            timeout: Async timeout in seconds
            num_predict: Max generated token count (Action channel passes 30, Legacy defaults to 100)
            num_ctx: Context window size (Action channel passes 1024, Legacy defaults to 2048)
            output_format: Output format constraint. 'json' = any valid JSON (for 7B),
                          dict = JSON Schema structured output (Action channel uses ACTION_SCHEMA)
        """
        if not OLLAMA_AVAILABLE:
            self.logger.error("ollama Python package unavailable. Install: pip install ollama")
            return None

        # Closure capture: bind parameters to local variables for _sync_ollama_call
        _num_predict = num_predict
        _num_ctx = num_ctx
        _output_format = output_format

        try:
            def _sync_ollama_call():
                response = ollama.chat(
                    model=model,
                    messages=[{'role': 'user', 'content': command}],
                    format=_output_format,
                    options={
                        'temperature': 0.0,
                        'num_predict': _num_predict,
                        'num_ctx': _num_ctx,
                        'top_p': 0.9,
                    }
                )

                content = response['message']['content']
                return json.loads(content)

            # Use run_in_executor to avoid blocking (Python 3.7+ get_running_loop)
            loop = asyncio.get_running_loop()
            result = await asyncio.wait_for(
                loop.run_in_executor(None, _sync_ollama_call),
                timeout=timeout
            )
            return result

        except asyncio.TimeoutError:
            self.logger.warning(f"Model timeout ({timeout}s): {model}")
            return None
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON parse failed: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Ollama call error: {e}")
            return None

    def _apply_safety_to_router_result(self, command, router_result,
                                        state_snapshot, snapshot_monotonic_ts,
                                        start_time):
        # type: (str, RouterResult, Any, Optional[float], float) -> BrainOutput
        """RouterResult -> SafetyCompiler -> BrainOutput (Invariant 1: safety compilation never skipped)

        Dual/Shadow path exclusive. Legacy path does not go through this method.
        """
        api_code = router_result.api_code
        sequence = router_result.sequence
        response = router_result.response
        route = router_result.route
        raw_llm_output = router_result.raw_llm_output

        # Save raw decision (for Shadow comparison)
        raw_decision = None
        if sequence:
            raw_decision = list(sequence)
        elif api_code is not None:
            raw_decision = [api_code]

        # Build candidate action list
        candidate = sequence if sequence else ([api_code] if api_code else [])

        if candidate:
            verdict = self._compile_safety(
                candidate, state_snapshot, snapshot_monotonic_ts)
            if verdict.is_blocked:
                self.logger.warning("Router path safety rejection: {}".format(verdict.block_reason))
                elapsed = (time.monotonic() - start_time) * 1000
                rejected_output = BrainOutput(
                    response=verdict.response_override or "Action stopped for safety.",
                    api_code=None, confidence=1.0,
                    reasoning="router_safety_rejected",
                    raw_decision=raw_decision,
                )
                self._log_audit(
                    command, rejected_output, route=route,
                    elapsed_ms=elapsed, cache_hit=False,
                    model_used=self._router_mode.value,
                    current_state=state_snapshot,
                    llm_output=raw_llm_output,
                    safety_verdict="rejected:{}".format(verdict.block_reason),
                    request_id=router_result.request_id,
                    router_mode=self._router_mode.value,
                    shadow_comparison=router_result.shadow_comparison,
                    action_latency_ms=router_result.action_latency_ms,
                    voice_latency_ms=router_result.voice_latency_ms,
                )
                return rejected_output

            exec_seq = verdict.executable_sequence
            if len(exec_seq) == 1:
                final_api = exec_seq[0]
                final_sequence = None
            else:
                final_api = None
                final_sequence = exec_seq

            if verdict.warnings:
                for w in verdict.warnings:
                    self.logger.info("SafetyCompiler: {}".format(w))

            # If SafetyCompiler downgraded/auto-prepended, regenerate response template
            # (fixes case where original response is "前転します" but actually downgraded to Dance2)
            if exec_seq != candidate:
                if final_sequence:
                    response = get_response_for_sequence(final_sequence)
                elif final_api is not None:
                    response = get_response_for_action(final_api)
        else:
            final_api = api_code
            final_sequence = sequence

        elapsed = (time.monotonic() - start_time) * 1000
        output = BrainOutput(
            response=response,
            api_code=final_api,
            sequence=final_sequence,
            raw_decision=raw_decision,
        )
        self._log_audit(
            command, output, route=route,
            elapsed_ms=elapsed, cache_hit=False,
            model_used=self._router_mode.value,
            current_state=state_snapshot,
            llm_output=raw_llm_output,
            safety_verdict="ok",
            request_id=router_result.request_id,
            router_mode=self._router_mode.value,
            shadow_comparison=router_result.shadow_comparison,
            action_latency_ms=router_result.action_latency_ms,
            voice_latency_ms=router_result.voice_latency_ms,
        )
        return output

    def _log_audit(self, command, output, route,
                   elapsed_ms, cache_hit, model_used,
                   current_state,
                   llm_output, safety_verdict,
                   safety_reason=None,
                   request_id=None, router_mode=None,
                   shadow_comparison=None,
                   action_latency_ms=None, voice_latency_ms=None):
        # type: (str, BrainOutput, str, float, bool, str, Optional[Any], Optional[str], str, Optional[str], Optional[str], Optional[str], Optional[Dict], Optional[float], Optional[float]) -> None
        """Record complete audit log (route must use audit_routes.py constants)"""
        if route not in ALL_ROUTES:
            raise ValueError(
                "Illegal route='{}', must use constants from audit_routes.py. "
                "Valid values: {}".format(route, ALL_ROUTES)
            )
        if not self.audit_logger:
            return

        from datetime import datetime
        try:
            entry = AuditEntry(
                timestamp=datetime.now().isoformat(),
                model_name=model_used,
                input_command=sanitize_audit_input(command),
                state_battery=current_state.battery_level if current_state else None,
                state_standing=current_state.is_standing if current_state else None,
                state_emergency=(
                    hasattr(current_state, 'state')
                    and current_state.state is not None
                    and getattr(current_state.state, 'name', '') == "EMERGENCY"
                ) if current_state else None,
                llm_output=llm_output,
                api_code=output.api_code,
                sequence=output.sequence,
                safety_verdict=safety_verdict,
                safety_reason=safety_reason,
                elapsed_ms=elapsed_ms,
                cache_hit=cache_hit,
                route=route,
                # success = pipeline completed normally (including dialogue/safety rejection), not "whether there was an action"
                # Use safety_verdict and api_code/sequence for fine-grained analysis
                success=not safety_verdict.startswith("error"),
                # PR2 extension fields
                request_id=request_id,
                router_mode=router_mode,
                shadow_comparison=shadow_comparison,
                action_latency_ms=action_latency_ms,
                voice_latency_ms=voice_latency_ms,
            )
            if not self.audit_logger.log_entry(entry):
                self.logger.warning("⚠️ Audit log write failed (route={})".format(route))
        except Exception as e:
            self.logger.warning(f"⚠️ Audit log recording failed: {e}")

    async def process_command(self, command: str) -> BrainOutput:
        """Process user command (state snapshot + hot path + safety gate optimized version)"""
        if _pae_depth.get(0) == 0:
            self.logger.warning(
                "process_command() called outside process_and_execute() "
                "— please migrate to process_and_execute() atomic entry point"
            )
        start_time = time.monotonic()
        self.logger.info(f"📥 Command received: '{command}'")

        # ===== 1) One-time snapshot with unified normalization =====
        state_snapshot = self.state_monitor.get_current_state() if self.state_monitor else None
        snapshot_monotonic_ts = time.monotonic()  # For SafetyCompiler freshness check

        if state_snapshot:
            # Shallow copy: do not modify the original object cached in state_monitor
            state_snapshot = copy.copy(state_snapshot)
            raw_batt = state_snapshot.battery_level
            state_snapshot.battery_level = self._normalize_battery(raw_batt)

            # State source check: trust level by source
            state_source = getattr(state_snapshot, 'source', 'unknown')
            if state_source == 'simulation':
                # Simulation data completely untrustworthy: battery=0.85/is_standing=True are fake values
                # fail-safe: is_standing=False, let SafetyCompiler auto-prepend StandUp
                state_snapshot.is_standing = False
                state_snapshot.battery_level = 0.50  # Conservative value, limits high-energy actions
                self.logger.warning(
                    "State snapshot: source=simulation (untrusted), battery unknown (safe default 50%), posture=not standing (fail-safe)"
                )
            elif state_source == 'sdk':
                # SDK real data: directly trust mode->is_standing and battery
                # Do not go through ros_initialized override branch (SDK is real hardware data)
                self.logger.info(
                    "State snapshot: source=sdk, battery {:.0f}%, posture={}".format(
                        state_snapshot.battery_level * 100 if state_snapshot.battery_level else 0,
                        'standing' if state_snapshot.is_standing else 'not standing'
                    )
                )
            elif state_source == 'sdk_partial':
                # SDK partial data: trust at state_ok/battery_ok granularity
                has_state = getattr(state_snapshot, 'state_ok', False)
                has_battery = getattr(state_snapshot, 'battery_ok', False)
                if not has_state:
                    # Posture unavailable -> fail-safe: assume not standing, let SafetyCompiler prepend StandUp
                    state_snapshot.is_standing = False
                # If battery_ok=False, keep SDKStateSnapshot default value 0.5
                battery_desc = (
                    "battery {:.0f}%".format(
                        state_snapshot.battery_level * 100 if state_snapshot.battery_level else 0
                    )
                    if has_battery else "battery unknown (safe default 50%)"
                )
                self.logger.info(
                    "State snapshot: source=sdk_partial (state={}, battery={}), {}, posture={}{}".format(
                        'ok' if has_state else 'fail',
                        'ok' if has_battery else 'fail',
                        battery_desc,
                        'standing' if state_snapshot.is_standing else 'not standing',
                        '(fail-safe)' if not has_state else '',
                    )
                )
            elif state_source == 'sdk_fallback':
                # SDK all failed: posture and battery both use conservative values
                # fail-safe: is_standing=False, let SafetyCompiler auto-prepend StandUp
                state_snapshot.is_standing = False
                self.logger.info(
                    "State snapshot: source=sdk_fallback, battery unknown (safe default 50%), posture=not standing (fail-safe)"
                )
            else:
                # ROS2 state_monitor or unknown
                ros_initialized = (
                    self.state_monitor
                    and hasattr(self.state_monitor, 'is_ros_initialized')
                    and self.state_monitor.is_ros_initialized
                )
                if not ros_initialized:
                    state_snapshot.is_standing = self.last_posture_standing
                self.logger.info(
                    "State snapshot: source={}, battery {:.0f}%, posture={}".format(
                        state_source,
                        state_snapshot.battery_level * 100 if state_snapshot.battery_level else 0,
                        'standing' if state_snapshot.is_standing else 'not standing'
                    )
                )

        # 0. Emergency command fast path -- references EMERGENCY_COMMANDS single source of truth
        # Note: process_and_execute() already intercepts emergency commands upstream and calls _handle_emergency.
        # This is a defensive check to prevent missing emergency handling when process_command is called directly.
        cmd_emergency = command.strip().lower().rstrip(self._TRAILING_PUNCTUATION)
        if cmd_emergency in self.EMERGENCY_COMMANDS:
            self.logger.warning(
                "Emergency command reached process_command directly -- "
                "recommend using process_and_execute() entry point"
            )
            return await self._handle_emergency(command)

        # ===== 2) Safety pre-check -- DEPRECATED (SafetyCompiler handles uniformly) =====
        # _quick_safety_precheck replaced by SafetyCompiler.
        # SafetyCompiler executes on every action-producing path, covering all old pre-check scenarios.
        # Old pre-check was text-keyword based; SafetyCompiler is api_code based, more precise.

        # ===== 3) Hot cache check -> SafetyCompiler unified safety compilation =====
        # Four-layer normalization:
        #   1) strip() exact match
        #   2) Strip trailing common punctuation (!?. etc.)
        #   3) lower() fallback match (English/mixed input)
        #   4) Japanese grammar suffix stripping (です/ます/ね/よ/ください/なさい)
        #      ASR often appends polite forms, but hot_cache keys are base forms
        cmd_stripped = command.strip()
        cmd_normalized = cmd_stripped.rstrip(self._TRAILING_PUNCTUATION)
        cmd_lower = cmd_normalized.lower()

        # Japanese grammar suffix stripping (longest to shortest, avoid "ください" being mis-stripped by "い")
        cmd_desuffixed = cmd_lower
        for suffix in ('ください', 'なさい', 'です', 'ます', 'ね', 'よ'):
            if cmd_desuffixed.endswith(suffix) and len(cmd_desuffixed) > len(suffix):
                cmd_desuffixed = cmd_desuffixed[:-len(suffix)]
                break

        cached = (
            self.hot_cache.get(cmd_stripped)
            or self.hot_cache.get(cmd_normalized)
            or self.hot_cache.get(cmd_lower)
            or self.hot_cache.get(cmd_desuffixed)
        )
        if cached:
            self.logger.info("Hot cache hit: {}".format(command))

            api_code = cached.get("api_code")
            sequence = cached.get("sequence")
            candidate = sequence if sequence else ([api_code] if api_code else [])

            if candidate:
                verdict = self._compile_safety(
                    candidate, state_snapshot, snapshot_monotonic_ts)
                if verdict.is_blocked:
                    self.logger.warning("Hot path safety rejection: {}".format(verdict.block_reason))
                    elapsed = (time.monotonic() - start_time) * 1000
                    rejected_output = BrainOutput(
                        response=verdict.response_override or "Action stopped for safety.",
                        api_code=None, confidence=1.0,
                        reasoning="hotpath_safety_rejected", success=False,
                    )
                    self._log_audit(
                        command, rejected_output, route=ROUTE_HOTPATH_REJECTED,
                        elapsed_ms=elapsed, cache_hit=True, model_used="hotpath",
                        current_state=state_snapshot, llm_output=None,
                        safety_verdict="rejected:{}".format(verdict.block_reason),
                    )
                    return rejected_output

                # verdict.executable_sequence already contains auto StandUp + downgrade
                exec_seq = verdict.executable_sequence
                if len(exec_seq) == 1:
                    final_api = exec_seq[0]
                    final_sequence = None
                else:
                    final_api = None
                    final_sequence = exec_seq
            else:
                final_api = api_code
                final_sequence = sequence

            brain_output = BrainOutput(
                response=cached.get("response", "実行します"),
                api_code=final_api,
                sequence=final_sequence,
                confidence=1.0,
                reasoning="hotpath_executed",
                success=True,
            )

            elapsed = (time.monotonic() - start_time) * 1000
            self.logger.info("Hot path processing complete ({:.0f}ms)".format(elapsed))
            self._log_audit(
                command, brain_output, route=ROUTE_HOTPATH,
                elapsed_ms=elapsed, cache_hit=True, model_used="hotpath",
                current_state=state_snapshot, llm_output=None, safety_verdict="ok",
            )
            return brain_output

        # Hot path miss, log it
        self.logger.info(f"🔍 Hot path miss, checking sequence definitions...")

        # ===== 3.3) Common sequence predefinitions (avoid LLM calls) =====
        cmd_lower = command.strip().lower()
        # ASR kana normalization: "たってからあいさつ" -> "立ってから挨拶"
        cmd_normalized = self._kana_to_kanji(cmd_lower)
        for key, seq in self.SEQUENCE_HOTPATH.items():
            if key in cmd_normalized:
                self.logger.info("Sequence definition hit: {} -> {}".format(key, seq))

                # P0-9: Sequence path must go through SafetyCompiler (old version had no safety check)
                verdict = self._compile_safety(
                    seq, state_snapshot, snapshot_monotonic_ts)
                if verdict.is_blocked:
                    self.logger.warning("Sequence safety rejection: {}".format(verdict.block_reason))
                    elapsed = (time.monotonic() - start_time) * 1000
                    rejected_output = BrainOutput(
                        response=verdict.response_override or "Action stopped for safety.",
                        api_code=None, reasoning="sequence_safety_rejected",
                    )
                    self._log_audit(
                        command, rejected_output, route=ROUTE_SEQUENCE,
                        elapsed_ms=elapsed, cache_hit=False, model_used="sequence_hotpath",
                        current_state=state_snapshot, llm_output=None,
                        safety_verdict="rejected:{}".format(verdict.block_reason),
                    )
                    return rejected_output
                exec_seq = verdict.executable_sequence

                seq_output = BrainOutput(
                    response=get_response_for_sequence(exec_seq),
                    sequence=exec_seq,
                    confidence=1.0,
                    reasoning="sequence_predefined",
                    success=True,
                )

                elapsed = (time.monotonic() - start_time) * 1000
                self._log_audit(
                    command, seq_output, route=ROUTE_SEQUENCE,
                    elapsed_ms=elapsed, cache_hit=False, model_used="sequence_hotpath",
                    current_state=state_snapshot, llm_output=None, safety_verdict="ok",
                )
                return seq_output

        self.logger.info("Sequence definition miss, checking conversational queries...")

        # ===== 3.5) Conversational query detection (avoid LLM misinterpreting dialogue as action) =====
        if self._is_conversational_query(command):
            conversational_response = self._generate_conversational_response(command)
            elapsed = (time.monotonic() - start_time) * 1000
            self.logger.info(f"💬 Conversational query detected ({elapsed:.0f}ms)")

            dialog_output = BrainOutput(
                response=conversational_response,
                api_code=None,  # Dialogue does not execute actions
                sequence=None,
                confidence=1.0,
                reasoning="conversational_query",
                success=True
            )

            # Audit log
            self._log_audit(command, dialog_output,
                          route=ROUTE_CONVERSATIONAL, elapsed_ms=elapsed, cache_hit=False,
                          model_used="dialog_detector", current_state=state_snapshot,
                          llm_output=None, safety_verdict="dialog")

            return dialog_output

        # 0.5. Special command handling - dance random selection -> SafetyCompiler
        dance_commands = ["dance", "ダンス", "跳舞", "舞蹈", "踊る", "踊って", "おどる", "おどって"]
        if command.lower() in dance_commands:
            dance_choice = random.choice([1022, 1023])
            dance_name = "1" if dance_choice == 1022 else "2"

            verdict = self._compile_safety(
                [dance_choice], state_snapshot, snapshot_monotonic_ts)
            if verdict.is_blocked:
                self.logger.warning("Dance safety rejection: {}".format(verdict.block_reason))
                elapsed = (time.monotonic() - start_time) * 1000
                rejected_output = BrainOutput(
                    response=verdict.response_override or "Action stopped for safety.",
                    api_code=None, reasoning="dance_safety_rejected",
                )
                self._log_audit(
                    command, rejected_output, route=ROUTE_DANCE,
                    elapsed_ms=elapsed, cache_hit=False, model_used="dance_random",
                    current_state=state_snapshot, llm_output=None,
                    safety_verdict="rejected:{}".format(verdict.block_reason),
                )
                return rejected_output

            exec_seq = verdict.executable_sequence
            if len(exec_seq) == 1:
                final_api = exec_seq[0]
                final_sequence = None
            else:
                final_api = None
                final_sequence = exec_seq

            elapsed = (time.monotonic() - start_time) * 1000
            self.logger.info("Random dance {} ({:.0f}ms)".format(dance_name, elapsed))
            dance_output = BrainOutput(
                response="踊ります{}".format(dance_name),
                api_code=final_api,
                sequence=final_sequence,
            )
            self._log_audit(
                command, dance_output, route=ROUTE_DANCE,
                elapsed_ms=elapsed, cache_hit=False, model_used="dance_random",
                current_state=state_snapshot, llm_output=None, safety_verdict="ok",
            )
            return dance_output

        # 2. LLM inference -> SafetyCompiler unified safety compilation
        # PR2: Dispatch to different channels based on BRAIN_ROUTER_MODE
        if self._router_mode == RouterMode.LEGACY:
            # --- Legacy direct path (zero behavior change) ---
            self.logger.info("7B model inference in progress...")
            await self._ensure_model_loaded(self.model_7b, num_ctx=2048)
            result = await self._call_ollama_v2(
                self.model_7b,
                command,
                timeout=30,
            )

            if result:
                elapsed = (time.monotonic() - start_time) * 1000
                self.logger.info("7B model response ({:.0f}ms)".format(elapsed))

                raw_response = result.get("response") or result.get("r", "実行します")
                response = self._sanitize_response(raw_response)
                api_code = result.get("api_code") or result.get("a")
                sequence = result.get("sequence") or result.get("s")

                if api_code is not None and api_code not in VALID_API_CODES:
                    self.logger.warning("LLM invalid api_code={}, demoting to text-only".format(api_code))
                    api_code = None
                if sequence:
                    valid_seq = [c for c in sequence if c in VALID_API_CODES]
                    if len(valid_seq) != len(sequence):
                        dropped = [c for c in sequence if c not in VALID_API_CODES]
                        self.logger.warning("LLM sequence has invalid codes {}, after filter: {}".format(dropped, valid_seq))
                        sequence = valid_seq if valid_seq else None

                candidate = sequence if sequence else ([api_code] if api_code else [])

                if candidate:
                    verdict = self._compile_safety(
                        candidate, state_snapshot, snapshot_monotonic_ts)
                    if verdict.is_blocked:
                        self.logger.warning("LLM path safety rejection: {}".format(verdict.block_reason))
                        rejected_output = BrainOutput(
                            response=verdict.response_override or "Action stopped for safety.",
                            api_code=None, confidence=1.0,
                            reasoning="llm_safety_rejected",
                        )
                        self._log_audit(
                            command, rejected_output, route=ROUTE_LLM_7B,
                            elapsed_ms=elapsed, cache_hit=False, model_used="7B",
                            current_state=state_snapshot,
                            llm_output=str(result)[:200],
                            safety_verdict="rejected:{}".format(verdict.block_reason),
                        )
                        return rejected_output

                    exec_seq = verdict.executable_sequence
                    if len(exec_seq) == 1:
                        final_api = exec_seq[0]
                        final_sequence = None
                    else:
                        final_api = None
                        final_sequence = exec_seq

                    if verdict.warnings:
                        for w in verdict.warnings:
                            self.logger.info("SafetyCompiler: {}".format(w))
                else:
                    final_api = api_code
                    final_sequence = sequence

                llm_output = BrainOutput(
                    response=response,
                    api_code=final_api,
                    sequence=final_sequence,
                )
                self._log_audit(
                    command, llm_output, route=ROUTE_LLM_7B,
                    elapsed_ms=elapsed, cache_hit=False, model_used="7B",
                    current_state=state_snapshot,
                    llm_output=str(result)[:200],
                    safety_verdict="ok",
                )
                return llm_output

            # Legacy no-response fallback
            elapsed = (time.monotonic() - start_time) * 1000
            self.logger.warning("Model no response, using default ({:.0f}ms)".format(elapsed))
            return BrainOutput(
                response="すみません、理解できませんでした",
                api_code=None,
            )

        # --- Dual/Shadow router path ---
        self.logger.info("Router inference (mode={})...".format(self._router_mode.value))
        router_result = await self._channel_router.route(
            command, state_snapshot=state_snapshot, start_time=start_time)
        return self._apply_safety_to_router_result(
            command, router_result, state_snapshot,
            snapshot_monotonic_ts, start_time)

    async def execute_action(self, brain_output: BrainOutput) -> Union[bool, str]:
        """Execute action

        Returns:
            True -- success
            "unknown" -- timeout but robot reachable (action may still be executing)
            False -- failure
        """
        if _pae_depth.get(0) == 0:
            self.logger.warning(
                "execute_action() called outside process_and_execute() "
                "— please migrate to process_and_execute() atomic entry point"
            )
        # Check hardware mode and SportClient status
        if self.use_real_hardware and self.sport_client:
            self.logger.info("🤖 Executing on real hardware")
            return await self._execute_real(brain_output)
        else:
            if self.use_real_hardware:
                self.logger.warning("⚠️ Real hardware mode but SportClient not initialized, using simulation")
            return await self._execute_mock(brain_output)

    async def _execute_mock(self, brain_output: BrainOutput) -> bool:
        """Mock execution (mirrors _execute_real posture tracking)"""
        if brain_output.api_code:
            self.logger.info(f"🎭 [Sim] API execution: {brain_output.api_code}")
            await asyncio.sleep(0.5)
            self._update_posture_tracking(brain_output.api_code)
            return True

        if brain_output.sequence:
            self.logger.info(f"🎭 [Sim] Sequence execution: {brain_output.sequence}")
            for api in brain_output.sequence:
                self.logger.info(f"   → API: {api}")
                await asyncio.sleep(0.3)
                self._update_posture_tracking(api)
            return True

        return False

    async def _verify_standing_after_unknown(self, max_retries=3, interval=1.0):
        """Verify standing state via GetState short-polling after StandUp returns unknown(3104)

        Go2 StandUp animation usually takes 2-3s; after 3104 timeout, a short delay + query can confirm.
        Used when StandUp is a prerequisite in sequence execution: must confirm standing before subsequent actions.

        Returns:
            True -- GetState confirms mode is in STANDING_MODES
            False -- Retries exhausted without confirming standing
        """
        # Consistent with SDKStateProvider.STANDING_MODES
        STANDING_MODES = {1, 2, 3, 4, 5, 6, 7, 8, 9}
        for attempt in range(max_retries):
            await asyncio.sleep(interval)
            try:
                result = self._rpc_call(
                    "GetState", GETSTATE_FULL_KEYS, timeout_override=3.0
                )
                if isinstance(result, tuple) and len(result) >= 2:
                    code, data = result[0], result[1]
                    if code == 0 and isinstance(data, dict):
                        mode = data.get("state", data.get("mode", -1))
                        if isinstance(mode, (int, float)):
                            mode = int(mode)
                            if mode in STANDING_MODES:
                                self.logger.info(
                                    "   GetState standing confirmed (mode={}, attempt {}/{})".format(
                                        mode, attempt + 1, max_retries
                                    )
                                )
                                return True
                            else:
                                self.logger.info(
                                    "   GetState not standing (mode={}, attempt {}/{})".format(
                                        mode, attempt + 1, max_retries
                                    )
                                )
            except Exception as e:
                self.logger.warning(
                    "   GetState query failed (attempt {}/{}): {}".format(
                        attempt + 1, max_retries, e
                    )
                )
        self.logger.warning("   StandUp confirmation timeout: not standing after {} retries".format(max_retries))
        return False

    def _update_posture_tracking(self, api_code):
        """Update internal posture tracking -- only called after action confirmed successful

        This method is only called in _execute_real() when result==0 or result==-1,
        ensuring unknown(3104) or failure does not pollute last_posture_standing state.
        3104 = RPC timeout (action may still be executing), cannot be treated as completed.
        """
        if api_code == 1004:  # StandUp
            self.robot_state = "standing"
            self.last_posture_standing = True
        elif api_code == 1006:  # RecoveryStand -> standing
            self.robot_state = "standing"
            self.last_posture_standing = True
        elif api_code == 1010:  # RiseSit -> standing
            self.robot_state = "standing"
            self.last_posture_standing = True
        elif api_code == 1009:  # Sit
            self.robot_state = "sitting"
            self.last_posture_standing = False
        elif api_code == 1005:  # StandDown
            self.robot_state = "lying"
            self.last_posture_standing = False

    async def _execute_real(self, brain_output: BrainOutput) -> Union[bool, str]:
        """Real execution (using _rpc_call + registry METHOD_MAP)

        Returns:
            True -- success
            "unknown" -- 3104 timeout but robot reachable (action may still be executing)
            False -- failure
        """
        try:
            # P0-8: Abort sequence on intermediate failure (no longer silently continue)
            if brain_output.sequence:
                self.logger.info("Executing sequence: {}".format(brain_output.sequence))
                for i, api in enumerate(brain_output.sequence):
                    single = BrainOutput("", api)
                    success = await self._execute_real(single)
                    if not success and success != "unknown":
                        self.logger.error(
                            "Sequence aborted: API {} (step {}/{}) execution failed".format(
                                api, i + 1, len(brain_output.sequence)
                            )
                        )
                        return False
                    # When StandUp(1004) returns unknown: subsequent actions may need standing,
                    # must confirm standing state via GetState before continuing sequence
                    if success == "unknown" and api == 1004:
                        has_subsequent = i + 1 < len(brain_output.sequence)
                        if has_subsequent:
                            standing_ok = await self._verify_standing_after_unknown()
                            if not standing_ok:
                                self.logger.error(
                                    "Sequence aborted: cannot confirm standing after StandUp(1004) unknown, "
                                    "subsequent actions {} require standing state".format(
                                        brain_output.sequence[i + 1:]
                                    )
                                )
                                return False
                            # Standing confirmed -> update posture tracking
                            self._update_posture_tracking(1004)
                    await asyncio.sleep(1)
                return True

            if not brain_output.api_code:
                return False

            # Look up method name from registry (replaces inline method_map)
            method_name = METHOD_MAP.get(brain_output.api_code)
            if not method_name:
                self.logger.error("Unregistered API: {}".format(brain_output.api_code))
                return False

            # SafetyCompiler already handles standing prerequisites (auto-prepends StandUp),
            # no need to duplicate check for actions_need_standing here.

            # Use _rpc_call for unified invocation (thread-safe + timeout management)
            self.logger.info("Executing: {} (API:{})".format(method_name, brain_output.api_code))

            # Long-running actions: increase RPC timeout (Dance/Scrape/Heart animations ~10-20s)
            LONG_RUNNING_ACTIONS = {1022, 1023, 1029, 1036}  # Dance1, Dance2, Scrape, Heart
            timeout_kw = {}
            if brain_output.api_code in LONG_RUNNING_ACTIONS:
                timeout_kw["timeout_override"] = 25.0

            # Parameterized actions use SAFE_DEFAULT_PARAMS
            if brain_output.api_code in SAFE_DEFAULT_PARAMS:
                params = SAFE_DEFAULT_PARAMS[brain_output.api_code]
                result = self._rpc_call(method_name, *params, **timeout_kw)
            else:
                result = self._rpc_call(method_name, **timeout_kw)

            # Handle tuple return values (e.g., GetState returns (code, data))
            if isinstance(result, tuple):
                result = result[0]

            self.logger.info("   Response code: {}".format(result))

            self.last_executed_api = brain_output.api_code

            # P0-1: Fix 3104 misjudgment (timeout != success)
            # Posture tracking only updated after confirmed success, to avoid unknown/failure polluting internal state
            if result == 0:
                self._update_posture_tracking(brain_output.api_code)
                return True
            elif result == -1:  # Already in target state
                self._update_posture_tracking(brain_output.api_code)
                return True
            elif result == 3104:  # RPC_ERR_CLIENT_API_TIMEOUT
                self.logger.warning("   Action response timeout (3104)")
                # Long-running actions (Dance/FrontFlip etc.) often trigger 3104:
                # Action already sent to robot and executing, just RPC response timed out.
                # Connectivity check: use correct key "state" (not "mode")
                try:
                    state_code, _ = self._rpc_call(
                        "GetState", GETSTATE_FULL_KEYS, timeout_override=3.0
                    )
                    if state_code == 0:
                        self.logger.info("   Connection check OK, action still executing")
                        return "unknown"
                    else:
                        self.logger.warning("   Connection anomaly ({}), action may have been executed".format(state_code))
                        return "unknown"  # 3104 itself means command was sent, should not be judged as failure
                except (json.JSONDecodeError, ValueError):
                    # GetState RPC may also timeout (robot busy executing action)
                    self.logger.info("   Connection probe timeout (robot may be executing action)")
                    return "unknown"
                except Exception as e:
                    self.logger.warning("   Connection check exception: {}".format(e))
                    return "unknown"  # 3104 means command was sent, conservative judgment as unknown
            else:
                # P0-2: Fix 3103 comments and logging
                if result == 3103:
                    self.logger.error("   Control conflict (3103): APP may be occupying sport_mode")
                    self.logger.error("      Close the APP and restart the robot, or confirm Init() succeeded")
                elif result == 3203:
                    self.logger.warning("   Action not supported (3203): this API is not implemented in Go2 firmware")
                else:
                    self.logger.warning("   Unknown error: {}".format(result))
                return False

        except Exception as e:
            self.logger.error("Execution error: {}".format(e))
            return False

    def get_statistics(self) -> Dict:
        """Get statistics"""
        return {
            "model": self.model_7b,
            "cache_size": len(self.hot_cache),
            "hardware_mode": self.use_real_hardware,
            "sport_client": self.sport_client is not None
        }


# Export
__all__ = ['ProductionBrain', 'BrainOutput']
