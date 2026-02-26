#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SDK State Provider -- Retrieve real robot state via Unitree SDK2

Replaces ROS2 state_monitor to avoid in-process DDS domain conflicts.

Data sources:
  - Posture (mode): SportClient.GetState(all 5 keys) RPC call, extract "state" field
  - Battery (soc):  ChannelSubscriber("rt/lowstate") DDS subscription -> LowState_.bms_state.soc

Note: Go2 firmware GetState requires full-field query (single-key returns empty response body).
      Must use ["state","bodyHeight","footRaiseHeight","speedLevel","gait"].
      Battery can only be obtained via LowState_ subscription; GetState has no "battery" key.

Interface compatibility: Returns SDKStateSnapshot object with fields matching SystemStateInfo.
"""

import time
import logging
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Any

# Go2 firmware requires full-field query (single-key query returns empty response body causing JSONDecodeError)
GETSTATE_FULL_KEYS = ["state", "bodyHeight", "footRaiseHeight", "speedLevel", "gait"]


@dataclass
class SDKStateSnapshot:
    """SDK state snapshot -- SystemStateInfo compatible subset

    Contains only fields actually used by SafetyCompiler and production_brain,
    without referencing ROS2-related SystemState/SystemLEDPriority enums.
    """
    battery_level: float = 0.5       # 0.0-1.0 normalized
    battery_voltage: float = 25.0
    is_charging: bool = False
    is_standing: bool = False
    is_moving: bool = False
    temperature: float = 40.0
    timestamp: float = 0.0
    source: str = "sdk"              # "sdk" | "sdk_partial" | "sdk_fallback"
    confidence: float = 1.0
    # Fine-grained data availability flags (resolves sdk_partial ambiguity)
    state_ok: bool = False           # Whether GetState RPC succeeded (posture data trustworthy)
    battery_ok: bool = False         # Whether LowState DDS has data (battery data trustworthy)
    # Compatibility fields (state_monitor has these, brain/audit may read)
    error_codes: list = field(default_factory=list)
    current_gait: str = "unknown"
    network_status: str = "connected"
    sdk_connection: bool = True
    _mono_ts: float = 0.0  # Monotonic timestamp for cache freshness (not affected by NTP jumps)


class SDKStateProvider:
    """Retrieve robot state via Unitree SDK2 (no ROS2 dependency)

    Posture data: SportClient.GetState(GETSTATE_FULL_KEYS) RPC
    Battery data: ChannelSubscriber("rt/lowstate") DDS subscription

    Design constraints:
      - Does not create ROS2 nodes, does not trigger rclpy.init()
      - RPC calls via rpc_call_fn callback, reusing brain's _rpc_call lock
      - DDS subscription uses Unitree SDK's ChannelSubscriber (same domain, no conflict)
      - Returns conservative fallback on query failure
      - Thread-safe: _cached_state protected by Lock

    source semantics:
      - "sdk":          Both state and battery available (complete data)
      - "sdk_partial":  Only state or only battery (partial data)
      - "sdk_fallback": All failed (conservative safe values)
    """

    # State value to posture mapping (Unitree Go2 SDK: "state" field from GetState)
    # 0=idle/damped, 1=standing(balancestand), 2=walking, ...
    STANDING_MODES = {1, 2, 3, 4, 5, 6, 7, 8, 9}  # Standing/moving modes
    MOVING_MODES = {2, 3, 4, 5, 6, 7, 8, 9}        # Moving modes

    def __init__(self, rpc_call_fn: Callable, logger: Optional[logging.Logger] = None):
        """
        Args:
            rpc_call_fn: brain._rpc_call callback function
            logger: Logger instance
        """
        self.rpc_call_fn = rpc_call_fn
        self.logger = logger or logging.getLogger("SDKStateProvider")

        self._lock = threading.Lock()
        self._cached_state = self._make_conservative_fallback()
        self._last_success_time = 0.0
        self._consecutive_failures = 0

        # Polling control
        self._stop_event = threading.Event()
        self._poll_thread = None
        self._poll_interval = 2.0

        # Compatibility field: state_monitor interface
        self.is_ros_initialized = False  # Always False -- does not use ROS2

        # LowState DDS subscription (battery source)
        self._lowstate_sub = None
        self._battery_soc = None       # 0-100 integer, None=not yet received
        self._battery_voltage = None   # Float voltage
        self._battery_lock = threading.Lock()
        self._init_lowstate_subscriber()

    def _init_lowstate_subscriber(self):
        """Initialize LowState DDS subscription (battery data source)

        Uses Unitree SDK's ChannelSubscriber, reusing existing DDS domain.
        No rclpy needed, no domain conflict.
        """
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

            self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._lowstate_sub.Init(self._on_lowstate, 10)
            self.logger.info("LowState DDS subscription started (battery/voltage)")
        except ImportError as e:
            self.logger.warning("LowState subscription unavailable (SDK import failed): {}".format(e))
            self._lowstate_sub = None
        except Exception as e:
            self.logger.warning("LowState subscription initialization failed: {}".format(e))
            self._lowstate_sub = None

    def _on_lowstate(self, msg):
        """LowState DDS callback -- extract BMS battery and voltage"""
        try:
            with self._battery_lock:
                # bms_state.soc: 0-100 integer (battery percentage)
                if hasattr(msg, 'bms_state') and hasattr(msg.bms_state, 'soc'):
                    self._battery_soc = int(msg.bms_state.soc)
                # power_v: float voltage
                if hasattr(msg, 'power_v'):
                    self._battery_voltage = float(msg.power_v)
        except Exception as e:
            self.logger.debug("LowState callback error: %s", e)

    def _make_conservative_fallback(self):
        # type: () -> SDKStateSnapshot
        """Conservative fallback -- safe default values when query fails

        battery=0.50 (moderate restriction), is_standing=False (conservative).
        """
        return SDKStateSnapshot(
            battery_level=0.50,
            is_standing=False,
            temperature=40.0,
            timestamp=time.time(),
            source="sdk_fallback",
            confidence=0.3,
            _mono_ts=time.monotonic(),
        )

    def query_state(self):
        # type: () -> SDKStateSnapshot
        """Actively query robot state once

        Gets posture via SportClient.GetState(GETSTATE_FULL_KEYS),
        gets battery via LowState DDS subscription.
        """
        snapshot = SDKStateSnapshot(timestamp=time.time(), _mono_ts=time.monotonic())
        state_ok = False
        battery_ok = False

        # 1. Query motion mode: GetState(all 5 keys) -- Go2 firmware doesn't support single-key query
        try:
            result = self.rpc_call_fn("GetState", GETSTATE_FULL_KEYS)
            if isinstance(result, tuple) and len(result) >= 2:
                code, data = result[0], result[1]
                if code == 0 and isinstance(data, dict):
                    mode = data.get("state", data.get("mode", -1))
                    if isinstance(mode, (int, float)):
                        mode = int(mode)
                        snapshot.is_standing = mode in self.STANDING_MODES
                        snapshot.is_moving = mode in self.MOVING_MODES
                        snapshot.current_gait = "mode_{}".format(mode)
                        state_ok = True
                elif code == 0 and isinstance(data, (list, tuple)) and len(data) > 0:
                    mode = data[0] if isinstance(data[0], (int, float)) else -1
                    mode = int(mode)
                    snapshot.is_standing = mode in self.STANDING_MODES
                    snapshot.is_moving = mode in self.MOVING_MODES
                    state_ok = True
                else:
                    self.logger.debug("GetState(state) response code: {}".format(code))
            else:
                self.logger.debug("GetState(state) unexpected response format: {}".format(type(result)))
        except Exception as e:
            self.logger.debug("GetState(state) query failed: {}".format(e))

        # 2. Get battery from LowState DDS subscription
        with self._battery_lock:
            soc = self._battery_soc
            voltage = self._battery_voltage

        if soc is not None and 0 <= soc <= 100:
            snapshot.battery_level = soc / 100.0
            battery_ok = True
        if voltage is not None and voltage > 0:
            snapshot.battery_voltage = voltage

        # 3. Set fine-grained flags + source
        snapshot.state_ok = state_ok
        snapshot.battery_ok = battery_ok
        if state_ok and battery_ok:
            snapshot.source = "sdk"
            snapshot.confidence = 1.0
        elif state_ok or battery_ok:
            snapshot.source = "sdk_partial"
            snapshot.confidence = 0.5
        else:
            # All failed -> conservative fallback
            fallback = self._make_conservative_fallback()
            with self._lock:
                self._cached_state = fallback
                self._consecutive_failures += 1
                if self._consecutive_failures >= 5:
                    self.logger.warning(
                        "{} consecutive queries all failed, maintaining conservative fallback".format(
                            self._consecutive_failures
                        )
                    )
            return fallback

        # Update cache
        with self._lock:
            self._cached_state = snapshot
            self._last_success_time = time.monotonic()
            self._consecutive_failures = 0

        return snapshot

    def get_current_state(self):
        """Get latest state (compatible with state_monitor interface)"""
        with self._lock:
            cached = self._cached_state

        # Cache expired (>10s) and no background polling -> trigger one query
        _polling_active = self._poll_thread is not None and self._poll_thread.is_alive()
        if time.monotonic() - cached._mono_ts > 10.0 and not _polling_active:
            return self.query_state()

        return cached

    def start_polling(self, interval=2.0):
        # type: (float) -> None
        """Start background polling"""
        if self._poll_thread and self._poll_thread.is_alive():
            return

        self._poll_interval = interval
        self._stop_event.clear()
        self._poll_thread = threading.Thread(
            target=self._poll_worker,
            daemon=True,
            name="sdk-state-poll"
        )
        self._poll_thread.start()
        self.logger.info("SDK state polling started (interval {:.1f}s)".format(interval))

    def stop_polling(self):
        """Stop background polling"""
        self._stop_event.set()
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=3.0)
        self.logger.info("SDK state polling stopped")

    def _poll_worker(self):
        """Background polling thread"""
        while not self._stop_event.is_set():
            try:
                self.query_state()
            except Exception as e:
                self.logger.warning("SDK state polling exception: {}".format(e))
                with self._lock:
                    self._consecutive_failures += 1
                    if self._consecutive_failures >= 5:
                        self._cached_state = self._make_conservative_fallback()
                        self.logger.warning(
                            "{} consecutive polling failures, switching to conservative fallback".format(
                                self._consecutive_failures
                            )
                        )
            self._stop_event.wait(timeout=self._poll_interval)
