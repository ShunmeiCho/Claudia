#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia LED State Machine Module
Implements LED mode priority management, state switching, and conflict resolution

Author: Claudia AI System
Generated: 2025-06-30
Purpose: Subtask 6.2 - LED pattern definition and state machine implementation
"""

import os
import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import queue
from datetime import datetime, timedelta

# Add project path (derived from module location, avoiding hardcoded paths)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# Import LED pattern definitions
try:
    from claudia.robot_controller.led_patterns import (
        ClaudiaLEDMode, LEDPattern, ClaudiaLEDModeDefinitions,
        LEDModeRenderer, create_led_mode_renderer
    )
    # Phase 2: Import system state monitor related classes
    from claudia.robot_controller.system_state_monitor import (
        SystemState, SystemLEDPriority, SystemStateInfo, LEDControlDecision
    )
    LED_PATTERNS_AVAILABLE = True
    SYSTEM_STATE_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: LED pattern definition import failed: {e}")
    LED_PATTERNS_AVAILABLE = False
    SYSTEM_STATE_AVAILABLE = False

@dataclass
class LEDStateRequest:
    """LED state request"""
    mode: 'ClaudiaLEDMode'
    priority: int
    duration: Optional[float] = None      # Duration override
    timestamp: datetime = field(default_factory=datetime.now)
    source: str = "unknown"               # Request source
    auto_revert: bool = True              # Whether to auto-revert to previous state
    interrupt_lower: bool = True          # Whether it can interrupt lower priority states

    def __lt__(self, other):
        """Comparison for PriorityQueue tiebreaking when priority and timestamp are equal"""
        return self.timestamp < other.timestamp

@dataclass
class LEDStateHistory:
    """LED state history record"""
    mode: 'ClaudiaLEDMode'
    start_time: datetime
    end_time: Optional[datetime] = None
    duration: Optional[float] = None
    source: str = "unknown"
    interrupted: bool = False

class DynamicPriorityManager:
    """
    Phase 2: Dynamic Priority Manager

    Dynamically adjusts LED control priorities based on system state for intelligent decision-making
    """

    def __init__(self):
        """Initialize dynamic priority manager"""
        self.logger = logging.getLogger(__name__)
        self.current_system_state: Optional[SystemStateInfo] = None
        self.base_priority_mapping = {
            # Base priorities for Claudia dedicated modes
            ClaudiaLEDMode.OFF: 1,
            ClaudiaLEDMode.WAKE_CONFIRM: 7,
            ClaudiaLEDMode.PROCESSING_VOICE: 6,
            ClaudiaLEDMode.EXECUTING_ACTION: 8,
            ClaudiaLEDMode.ACTION_COMPLETE: 9,
            ClaudiaLEDMode.ERROR_STATE: 10,
            # System compatibility modes
            ClaudiaLEDMode.SYSTEM_BOOT: 3,
            ClaudiaLEDMode.SYSTEM_CALIBRATION: 4,
            ClaudiaLEDMode.LOW_BATTERY: 5,
            ClaudiaLEDMode.SEARCH_LIGHT: 2
        }

        # System state adjustment table
        self.system_state_adjustments = {
            SystemState.NORMAL: 0,           # Normal state: no adjustment
            SystemState.IDLE: 0,             # Idle state: no adjustment
            SystemState.ACTIVE: 0,           # Active state: no adjustment
            SystemState.CALIBRATING: +2,     # Calibration: raise system-related mode priority
            SystemState.LOW_BATTERY: +3,     # Low battery: raise safety-related mode priority
            SystemState.ERROR: +5,           # Error: significantly raise error-related mode priority
            SystemState.EMERGENCY: +7,       # Emergency: maximum raise for critical mode priority
            SystemState.UNKNOWN: 0          # Unknown state: no adjustment
        }

        # Priority dynamic adjustment history
        self.adjustment_history: List[Tuple[datetime, SystemState, int]] = []
        self.max_history_size = 50

    def update_system_state(self, system_state_info: SystemStateInfo) -> None:
        """
        Update system state information

        Args:
            system_state_info: System state information
        """
        previous_state = self.current_system_state.state if self.current_system_state else SystemState.UNKNOWN
        self.current_system_state = system_state_info

        # Record state change adjustment
        if previous_state != system_state_info.state:
            adjustment = self.system_state_adjustments.get(system_state_info.state, 0)
            self.adjustment_history.append((datetime.now(), system_state_info.state, adjustment))

            # Limit history size
            if len(self.adjustment_history) > self.max_history_size:
                self.adjustment_history.pop(0)

            self.logger.info(f"System state change affecting priority: {previous_state.name} -> {system_state_info.state.name} "
                           f"(adjustment: {adjustment:+d})")

    def calculate_dynamic_priority(self, mode: ClaudiaLEDMode, base_priority: Optional[int] = None) -> int:
        """
        Calculate dynamically adjusted priority

        Args:
            mode: LED mode
            base_priority: Base priority (if None, use default mapping)

        Returns:
            int: Dynamically adjusted priority (1-10)
        """
        # Get base priority
        if base_priority is None:
            base_priority = self.base_priority_mapping.get(mode, 5)

        # If no system state info, return base priority
        if not self.current_system_state:
            return max(1, min(10, base_priority))

        # Calculate system state adjustment
        system_adjustment = self.system_state_adjustments.get(
            self.current_system_state.state, 0
        )

        # Calculate mode-specific adjustment
        mode_adjustment = self._calculate_mode_specific_adjustment(mode, self.current_system_state)

        # Apply dynamic adjustment
        dynamic_priority = base_priority + system_adjustment + mode_adjustment

        # Clamp to valid range
        final_priority = max(1, min(10, dynamic_priority))

        self.logger.debug(f"Dynamic priority calculation: {mode.value} | base={base_priority} + system_adj={system_adjustment} + "
                         f"mode_adj={mode_adjustment} = {final_priority}")

        return final_priority

    def _calculate_mode_specific_adjustment(self, mode: ClaudiaLEDMode, system_state: SystemStateInfo) -> int:
        """
        Calculate mode-specific priority adjustment

        Args:
            mode: LED mode
            system_state: System state information

        Returns:
            int: Mode-specific adjustment value
        """
        adjustment = 0

        # Error state related adjustment
        if mode == ClaudiaLEDMode.ERROR_STATE:
            if system_state.state in [SystemState.ERROR, SystemState.EMERGENCY]:
                adjustment += 2  # Further raise error LED priority during system error
            elif len(system_state.error_codes) > 0:
                adjustment += 1  # Moderately raise when error codes present

        # Low battery related adjustment
        elif mode == ClaudiaLEDMode.LOW_BATTERY:
            if system_state.battery_level <= 0.05:  # Critically low battery
                adjustment += 3
            elif system_state.battery_level <= 0.15:  # Low battery
                adjustment += 2

        # Calibration related adjustment
        elif mode == ClaudiaLEDMode.SYSTEM_CALIBRATION:
            if system_state.state == SystemState.CALIBRATING:
                adjustment += 2  # Raise calibration LED priority during calibration

        # User interaction mode adjustment when system is busy
        elif mode in [ClaudiaLEDMode.WAKE_CONFIRM, ClaudiaLEDMode.PROCESSING_VOICE, ClaudiaLEDMode.EXECUTING_ACTION]:
            if system_state.state in [SystemState.LOW_BATTERY, SystemState.ERROR]:
                adjustment -= 1  # Lower user interaction priority during system issues
            elif system_state.state == SystemState.EMERGENCY:
                adjustment -= 2  # Significantly lower during emergency

        return adjustment

    def get_led_control_decision(self, mode: ClaudiaLEDMode, requested_priority: int) -> LEDControlDecision:
        """
        Get LED control decision

        Args:
            mode: Requested LED mode
            requested_priority: Requested priority

        Returns:
            LEDControlDecision: Control decision
        """
        if not self.current_system_state:
            # No system state info, allow control
            return LEDControlDecision(
                allow_custom_control=True,
                required_priority=SystemLEDPriority.NORMAL,
                system_override_active=False,
                recommended_action="proceed",
                reason="No system state info, allowing control"
            )

        # Calculate dynamic priority requirement
        dynamic_priority = self.calculate_dynamic_priority(mode)
        system_priority = self.current_system_state.priority.value

        # Determine whether control is allowed
        allow_control = requested_priority >= max(dynamic_priority, system_priority)

        # Determine recommended priority
        required_priority = SystemLEDPriority(min(10, max(1, max(dynamic_priority, system_priority))))

        # Check system forced override
        system_override = self.current_system_state.state in [
            SystemState.EMERGENCY, SystemState.ERROR, SystemState.LOW_BATTERY
        ]

        # Generate decision reason
        if allow_control:
            reason = f"System state: {self.current_system_state.state.name}, dynamic priority: {dynamic_priority}"
        else:
            reason = f"Insufficient priority (required: {required_priority.value}, requested: {requested_priority})"

        # Recommended action
        if allow_control:
            recommended_action = "proceed"
        elif requested_priority < required_priority.value:
            recommended_action = "increase_priority"
        else:
            recommended_action = "wait_for_system_state_change"

        return LEDControlDecision(
            allow_custom_control=allow_control,
            required_priority=required_priority,
            system_override_active=system_override,
            recommended_action=recommended_action,
            reason=reason
        )

    def should_auto_switch_mode(self, current_mode: ClaudiaLEDMode) -> Optional[ClaudiaLEDMode]:
        """
        Determine whether LED mode should be automatically switched

        Args:
            current_mode: Current LED mode

        Returns:
            Optional[ClaudiaLEDMode]: Suggested mode to switch to, None if no switch needed
        """
        if not self.current_system_state:
            return None

        system_state = self.current_system_state.state

        # Emergency state auto-switch
        if system_state == SystemState.EMERGENCY:
            if current_mode != ClaudiaLEDMode.ERROR_STATE:
                return ClaudiaLEDMode.ERROR_STATE

        # Error state auto-switch
        elif system_state == SystemState.ERROR:
            if current_mode not in [ClaudiaLEDMode.ERROR_STATE]:
                return ClaudiaLEDMode.ERROR_STATE

        # Critically low battery auto-switch
        elif (system_state == SystemState.LOW_BATTERY and
              self.current_system_state.battery_level <= 0.05):
            if current_mode != ClaudiaLEDMode.ERROR_STATE:
                return ClaudiaLEDMode.ERROR_STATE  # Use error mode for critically low battery

        # Low battery auto-switch
        elif system_state == SystemState.LOW_BATTERY:
            if current_mode != ClaudiaLEDMode.LOW_BATTERY:
                return ClaudiaLEDMode.LOW_BATTERY

        # Calibration state auto-switch
        elif system_state == SystemState.CALIBRATING:
            if current_mode != ClaudiaLEDMode.SYSTEM_CALIBRATION:
                return ClaudiaLEDMode.SYSTEM_CALIBRATION

        return None

    def get_adjustment_statistics(self) -> Dict[str, Any]:
        """Get priority adjustment statistics"""
        if not self.adjustment_history:
            return {"total_adjustments": 0}

        # Count adjustments per system state
        state_counts = {}
        total_adjustments = len(self.adjustment_history)

        for _, state, adjustment in self.adjustment_history:
            state_name = state.name
            if state_name not in state_counts:
                state_counts[state_name] = {"count": 0, "total_adjustment": 0}

            state_counts[state_name]["count"] += 1
            state_counts[state_name]["total_adjustment"] += adjustment

        # Calculate average adjustment
        for state_info in state_counts.values():
            if state_info["count"] > 0:
                state_info["average_adjustment"] = state_info["total_adjustment"] / state_info["count"]

        return {
            "total_adjustments": total_adjustments,
            "state_adjustment_stats": state_counts,
            "current_system_state": self.current_system_state.state.name if self.current_system_state else None,
            "history_size": len(self.adjustment_history)
        }

class LEDStateMachine:
    """
    Claudia LED State Machine

    Responsible for managing LED mode priorities, state switching, and conflict resolution.
    Ensures system compatibility and avoids interfering with default LED states.
    Phase 2: Integrated dynamic priority management and system state awareness.
    """

    def __init__(self, response_time_target: float = 0.2):
        """
        Initialize LED state machine

        Args:
            response_time_target: Target response time (seconds)
        """
        self.logger = logging.getLogger(__name__)
        self.response_time_target = response_time_target

        # Core components
        self.renderer = None
        self.is_initialized = False

        # Phase 2: Dynamic priority manager
        self.dynamic_priority_manager = DynamicPriorityManager() if SYSTEM_STATE_AVAILABLE else None
        self.current_system_state = None

        # State management
        self.current_state = ClaudiaLEDMode.OFF
        self.current_priority = 1
        self.state_lock = threading.RLock()

        # Request queue and processing
        self.request_queue = queue.PriorityQueue()
        self.processing_thread = None
        self.processing_active = False

        # State history and revert
        self.state_history: List[LEDStateHistory] = []
        self.previous_state_stack: List[Tuple[ClaudiaLEDMode, int]] = []
        self.max_history_size = 100

        # System compatibility management
        self.system_override_enabled = True
        self.protected_system_modes = {
            ClaudiaLEDMode.SYSTEM_BOOT,
            ClaudiaLEDMode.SYSTEM_CALIBRATION,
            ClaudiaLEDMode.LOW_BATTERY
        }

        # Phase 2: Auto mode switching
        self.auto_mode_switching_enabled = True
        self.last_auto_switch_check = 0
        self.auto_switch_check_interval = 1.0  # Check every 1 second

        # Performance monitoring
        self.performance_metrics = {
            'state_changes': 0,
            'average_response_time': 0.0,
            'max_response_time': 0.0,
            'queue_overflows': 0,
            'priority_conflicts': 0,
            'dynamic_priority_adjustments': 0,  # New: dynamic priority adjustment count
            'auto_mode_switches': 0            # New: auto mode switch count
        }

        self.logger.info("LED state machine initialized (Phase 2: intelligent decision version)")

    def initialize(self) -> bool:
        """
        Initialize LED state machine

        Returns:
            bool: Whether initialization succeeded
        """
        if not LED_PATTERNS_AVAILABLE:
            self.logger.error("LED pattern definitions not available")
            return False

        try:
            self.logger.info("Initializing LED state machine...")

            # Create LED mode renderer
            self.renderer = create_led_mode_renderer()
            if not self.renderer.initialize_vui():
                self.logger.error("LED renderer initialization failed")
                return False

            # Start request processing thread
            self.processing_active = True
            self.processing_thread = threading.Thread(
                target=self._request_processing_worker,
                daemon=True
            )
            self.processing_thread.start()

            # Set initial state
            self._record_state_change(ClaudiaLEDMode.OFF, "system", 0.0)

            self.is_initialized = True
            self.logger.info("LED state machine initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"LED state machine initialization failed: {e}")
            return False

    def request_state(self,
                     mode: ClaudiaLEDMode,
                     source: str = "user",
                     duration: Optional[float] = None,
                     priority_override: Optional[int] = None) -> bool:
        """
        Request LED state change
        Phase 2: Integrated dynamic priority calculation and intelligent decision-making

        Args:
            mode: Target LED mode
            source: Request source identifier
            duration: Optional duration override
            priority_override: Optional priority override

        Returns:
            bool: Whether the request was accepted
        """
        if not self.is_initialized:
            self.logger.error("LED state machine not initialized")
            return False

        # Get mode parameters
        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        if not ClaudiaLEDModeDefinitions.validate_pattern(pattern):
            self.logger.error(f"Invalid LED mode: {mode}")
            return False

        # Phase 2: Use dynamic priority calculation
        if priority_override is not None:
            effective_priority = priority_override
        elif self.dynamic_priority_manager:
            # Use dynamic priority manager to calculate priority
            effective_priority = self.dynamic_priority_manager.calculate_dynamic_priority(mode, pattern.priority)
            self.logger.debug(f"Dynamic priority calculation: {mode.value} base={pattern.priority} -> dynamic={effective_priority}")
        else:
            effective_priority = pattern.priority

        # Phase 2: Check LED control decision
        if self.dynamic_priority_manager:
            control_decision = self.dynamic_priority_manager.get_led_control_decision(mode, effective_priority)

            if not control_decision.allow_custom_control:
                self.logger.warning(f"LED control decision rejected request: {control_decision.reason}")
                self.logger.info(f"Suggested action: {control_decision.recommended_action}")
                return False

            # If there is a higher recommended priority, use it
            if control_decision.required_priority.value > effective_priority:
                effective_priority = control_decision.required_priority.value
                self.logger.debug(f"Raising priority to recommended value: {effective_priority}")

        # System compatibility check (enhanced version)
        if not self._check_system_compatibility(mode, effective_priority):
            self.logger.warning(f"System compatibility check failed, rejecting state request: {mode}")
            return False

        # Create state request
        request = LEDStateRequest(
            mode=mode,
            priority=effective_priority,
            duration=duration,
            source=source,
            auto_revert=True,
            interrupt_lower=True
        )

        try:
            # Use negative priority to ensure higher priority requests are processed first
            self.request_queue.put((-effective_priority, time.time(), request), timeout=1.0)
            self.logger.debug(f"LED state request queued: {mode.value} (dynamic_priority={effective_priority}, source={source})")
            return True

        except queue.Full:
            self.logger.error("LED state request queue is full")
            self.performance_metrics['queue_overflows'] += 1
            return False

    def _request_processing_worker(self) -> None:
        """LED state request processing worker thread"""
        while self.processing_active:
            try:
                # Wait for request
                try:
                    neg_priority, timestamp, request = self.request_queue.get(timeout=1.0)
                    actual_priority = -neg_priority
                except queue.Empty:
                    continue

                # Process request
                self._process_state_request(request, timestamp)
                self.request_queue.task_done()

            except Exception as e:
                self.logger.error(f"State request processing failed: {e}")

    def _process_state_request(self, request: LEDStateRequest, request_timestamp: float) -> None:
        """
        Process a single LED state request

        Args:
            request: LED state request
            request_timestamp: Request timestamp
        """
        process_start = time.time()

        with self.state_lock:
            # Check priority and conflicts
            can_interrupt = self._can_interrupt_current_state(request.priority, request.interrupt_lower)

            if not can_interrupt:
                self.logger.debug(f"Insufficient priority, ignoring state request: {request.mode.value} (requested_priority={request.priority}, current_priority={self.current_priority})")
                self.performance_metrics['priority_conflicts'] += 1
                return

            # Save current state to revert stack
            if self.current_state != ClaudiaLEDMode.OFF and request.auto_revert:
                self.previous_state_stack.append((self.current_state, self.current_priority))
                # Limit stack size
                if len(self.previous_state_stack) > 10:
                    self.previous_state_stack.pop(0)

            # Execute state change
            self._execute_state_change(request, process_start)

            # Calculate and record response time
            response_time = time.time() - request_timestamp
            self._update_performance_metrics(response_time)

            if response_time > self.response_time_target:
                self.logger.warning(f"LED state switch response time exceeded target: {response_time*1000:.1f}ms > {self.response_time_target*1000}ms")

    def _execute_state_change(self, request: LEDStateRequest, start_time: float) -> None:
        """
        Execute LED state change

        Args:
            request: LED state request
            start_time: Start time
        """
        try:
            # End current state record
            self._end_current_state_record()

            # Update current state
            self.current_state = request.mode
            self.current_priority = request.priority

            # Start LED rendering
            if self.renderer:
                success = self.renderer.render_mode(request.mode, request.duration)
                if success:
                    self.logger.info(f"LED state switch successful: {request.mode.value} (priority={request.priority}, source={request.source})")
                else:
                    self.logger.error(f"LED rendering failed: {request.mode.value}")
            else:
                self.logger.error("LED renderer not available")

            # Record state change
            self._record_state_change(request.mode, request.source, time.time() - start_time)

            # If there is a duration, schedule auto-revert
            if request.duration and request.duration > 0 and request.auto_revert:
                self._schedule_auto_revert(request.duration)

        except Exception as e:
            self.logger.error(f"State change execution failed: {e}")

    def _can_interrupt_current_state(self, new_priority: int, interrupt_lower: bool) -> bool:
        """
        Check whether the current state can be interrupted

        Args:
            new_priority: New request priority
            interrupt_lower: Whether interrupting lower priority states is allowed

        Returns:
            bool: Whether interruption is possible
        """
        # System mode protection
        if self.current_state in self.protected_system_modes and self.system_override_enabled:
            # Only higher priority system modes can interrupt
            return new_priority > self.current_priority and new_priority >= 8

        # Normal priority comparison
        if interrupt_lower:
            return new_priority >= self.current_priority
        else:
            return new_priority > self.current_priority

    def _check_system_compatibility(self, mode: ClaudiaLEDMode, priority: int) -> bool:
        """
        Check system compatibility

        Args:
            mode: LED mode
            priority: Request priority

        Returns:
            bool: Whether compatible
        """
        # Check for conflict with protected system modes
        if self.current_state in self.protected_system_modes:
            # Low priority requests cannot interrupt system modes
            if priority < self.current_priority:
                return False

        # Check if the mode is a protected system mode
        if mode in self.protected_system_modes and not self.system_override_enabled:
            return False

        return True

    def _schedule_auto_revert(self, delay: float) -> None:
        """
        Schedule auto-revert to previous state

        Args:
            delay: Delay time (seconds)
        """
        def auto_revert_worker():
            time.sleep(delay)

            with self.state_lock:
                # Check if revert is still needed
                if self.previous_state_stack:
                    prev_mode, prev_priority = self.previous_state_stack.pop()

                    # Create revert request
                    revert_request = LEDStateRequest(
                        mode=prev_mode,
                        priority=prev_priority,
                        source="auto_revert",
                        auto_revert=False,
                        interrupt_lower=False
                    )

                    self._process_state_request(revert_request, time.time())
                    self.logger.debug(f"Auto-reverted to previous state: {prev_mode.value}")
                else:
                    # No previous state, revert to OFF
                    off_request = LEDStateRequest(
                        mode=ClaudiaLEDMode.OFF,
                        priority=1,
                        source="auto_revert",
                        auto_revert=False
                    )
                    self._process_state_request(off_request, time.time())

        # Start revert thread
        revert_thread = threading.Thread(target=auto_revert_worker, daemon=True)
        revert_thread.start()

    def _record_state_change(self, mode: ClaudiaLEDMode, source: str, response_time: float) -> None:
        """
        Record state change to history

        Args:
            mode: LED mode
            source: Source
            response_time: Response time
        """
        # Create history record
        history_entry = LEDStateHistory(
            mode=mode,
            start_time=datetime.now(),
            source=source
        )

        self.state_history.append(history_entry)

        # Limit history size
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)

        # Update performance metrics
        self.performance_metrics['state_changes'] += 1

    def _end_current_state_record(self) -> None:
        """End current state record"""
        if self.state_history:
            current_record = self.state_history[-1]
            if current_record.end_time is None:
                current_record.end_time = datetime.now()
                duration = (current_record.end_time - current_record.start_time).total_seconds()
                current_record.duration = duration

    def _update_performance_metrics(self, response_time: float) -> None:
        """
        Update performance metrics

        Args:
            response_time: Response time
        """
        # Update average response time
        total_changes = self.performance_metrics['state_changes']
        if total_changes > 0:
            current_avg = self.performance_metrics['average_response_time']
            new_avg = (current_avg * (total_changes - 1) + response_time) / total_changes
            self.performance_metrics['average_response_time'] = new_avg

        # Update maximum response time
        if response_time > self.performance_metrics['max_response_time']:
            self.performance_metrics['max_response_time'] = response_time

    def get_current_state(self) -> Tuple[ClaudiaLEDMode, int]:
        """
        Get current LED state

        Returns:
            Tuple[ClaudiaLEDMode, int]: (current mode, current priority)
        """
        with self.state_lock:
            return self.current_state, self.current_priority

    def get_state_history(self, limit: int = 10) -> List[LEDStateHistory]:
        """
        Get state history records

        Args:
            limit: Maximum number of records to return

        Returns:
            List[LEDStateHistory]: History record list
        """
        with self.state_lock:
            return self.state_history[-limit:] if limit > 0 else self.state_history.copy()

    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get performance metrics

        Returns:
            Dict[str, Any]: Performance metrics dictionary
        """
        with self.state_lock:
            metrics = self.performance_metrics.copy()
            metrics['meets_response_requirement'] = metrics['max_response_time'] <= self.response_time_target
            metrics['average_response_time_ms'] = metrics['average_response_time'] * 1000
            metrics['max_response_time_ms'] = metrics['max_response_time'] * 1000
            return metrics

    def force_state(self, mode: ClaudiaLEDMode, source: str = "force") -> bool:
        """
        Force set LED state (ignoring priority)

        Args:
            mode: Target LED mode
            source: Source identifier

        Returns:
            bool: Whether successful
        """
        if not self.is_initialized:
            self.logger.error("LED state machine not initialized")
            return False

        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)

        # Create high priority forced request
        request = LEDStateRequest(
            mode=mode,
            priority=10,  # Highest priority
            source=source,
            auto_revert=False,
            interrupt_lower=True
        )

        # Process directly, bypassing queue
        with self.state_lock:
            self._execute_state_change(request, time.time())

        self.logger.info(f"Forced LED state set: {mode.value}")
        return True

    def emergency_stop(self) -> bool:
        """
        Emergency stop all LED activity

        Returns:
            bool: Whether successful
        """
        self.logger.warning("Emergency stopping LED state machine")

        try:
            # Stop renderer
            if self.renderer:
                self.renderer.stop_all_rendering()

            # Clear request queue
            while not self.request_queue.empty():
                try:
                    self.request_queue.get_nowait()
                    self.request_queue.task_done()
                except queue.Empty:
                    break

            # Reset state
            with self.state_lock:
                self.current_state = ClaudiaLEDMode.OFF
                self.current_priority = 1
                self.previous_state_stack.clear()

            return True

        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")
            return False

    def set_system_override(self, enabled: bool) -> None:
        """
        Set system override mode

        Args:
            enabled: Whether to enable system mode protection
        """
        self.system_override_enabled = enabled
        self.logger.info(f"System override mode: {'enabled' if enabled else 'disabled'}")

    def cleanup(self) -> None:
        """Clean up resources"""
        self.logger.info("Cleaning up LED state machine resources...")

        try:
            # Stop processing thread
            self.processing_active = False
            if self.processing_thread and self.processing_thread.is_alive():
                self.processing_thread.join(timeout=2.0)

            # Clean up renderer
            if self.renderer:
                self.renderer.cleanup()

            # Final state record
            self._end_current_state_record()

            self.is_initialized = False
            self.logger.info("LED state machine cleanup complete")

        except Exception as e:
            self.logger.error(f"LED state machine cleanup failed: {e}")

    def update_system_state(self, system_state_info: SystemStateInfo) -> None:
        """
        Phase 2: Update system state and trigger related processing

        Args:
            system_state_info: System state information
        """
        if not self.dynamic_priority_manager:
            self.logger.warning("Dynamic priority manager not available, cannot update system state")
            return

        previous_state = self.current_system_state
        self.current_system_state = system_state_info

        # Update dynamic priority manager
        self.dynamic_priority_manager.update_system_state(system_state_info)

        self.logger.info(f"System state updated: {system_state_info.state.name}")

        # Check if auto mode switch is needed
        if self.auto_mode_switching_enabled:
            self._check_auto_mode_switch()

        # Re-evaluate current state priority
        self._reevaluate_current_priority()

    def _check_auto_mode_switch(self) -> None:
        """
        Phase 2: Check if automatic LED mode switch is needed
        """
        current_time = time.time()
        if current_time - self.last_auto_switch_check < self.auto_switch_check_interval:
            return

        self.last_auto_switch_check = current_time

        if not self.dynamic_priority_manager:
            return

        # Check if auto-switch is suggested
        suggested_mode = self.dynamic_priority_manager.should_auto_switch_mode(self.current_state)

        if suggested_mode and suggested_mode != self.current_state:
            self.logger.info(f"System suggests auto mode switch: {self.current_state.value} -> {suggested_mode.value}")

            # Get dynamic priority for suggested mode
            dynamic_priority = self.dynamic_priority_manager.calculate_dynamic_priority(suggested_mode)

            # Only switch when suggested mode's priority is higher than current
            if dynamic_priority > self.current_priority:
                success = self.request_state(
                    mode=suggested_mode,
                    source="auto_switch",
                    priority_override=dynamic_priority
                )

                if success:
                    self.performance_metrics['auto_mode_switches'] += 1
                    self.logger.info(f"Auto mode switch successful: {suggested_mode.value}")
                else:
                    self.logger.warning(f"Auto mode switch failed: {suggested_mode.value}")

    def _reevaluate_current_priority(self) -> None:
        """
        Phase 2: Re-evaluate current state priority
        """
        if not self.dynamic_priority_manager or self.current_state == ClaudiaLEDMode.OFF:
            return

        # Calculate new dynamic priority for current mode
        new_priority = self.dynamic_priority_manager.calculate_dynamic_priority(
            self.current_state, self.current_priority
        )

        if new_priority != self.current_priority:
            self.logger.debug(f"Re-evaluating priority: {self.current_state.value} "
                            f"{self.current_priority} -> {new_priority}")

            with self.state_lock:
                self.current_priority = new_priority

            self.performance_metrics['dynamic_priority_adjustments'] += 1

    def get_led_control_decision(self, mode: ClaudiaLEDMode, requested_priority: int) -> Optional['LEDControlDecision']:
        """
        Phase 2: Get LED control decision

        Args:
            mode: Requested LED mode
            requested_priority: Requested priority

        Returns:
            Optional[LEDControlDecision]: Control decision (if dynamic priority manager is available)
        """
        if not self.dynamic_priority_manager:
            return None

        return self.dynamic_priority_manager.get_led_control_decision(mode, requested_priority)

    def set_auto_mode_switching(self, enabled: bool) -> None:
        """
        Phase 2: Set auto mode switching

        Args:
            enabled: Whether to enable auto mode switching
        """
        self.auto_mode_switching_enabled = enabled
        self.logger.info(f"Auto mode switching: {'enabled' if enabled else 'disabled'}")

    def get_dynamic_priority_statistics(self) -> Optional[Dict[str, Any]]:
        """
        Phase 2: Get dynamic priority statistics

        Returns:
            Optional[Dict[str, Any]]: Statistics (if available)
        """
        if not self.dynamic_priority_manager:
            return None

        return self.dynamic_priority_manager.get_adjustment_statistics()


# Factory function
def create_led_state_machine() -> LEDStateMachine:
    """
    Create LED state machine instance

    Returns:
        LEDStateMachine: State machine instance
    """
    return LEDStateMachine()


if __name__ == "__main__":
    # Basic test
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    print("LED State Machine Test")
    print("=" * 50)

    try:
        # Create state machine
        state_machine = create_led_state_machine()

        if state_machine.initialize():
            print("LED state machine initialized successfully")

            # Test state switching sequence
            print("\nTesting state switching sequence...")

            # 1. Wake confirmation
            print("1. Wake confirmation (2 seconds)")
            state_machine.request_state(ClaudiaLEDMode.WAKE_CONFIRM, "test")
            time.sleep(3)

            # 2. Processing voice
            print("2. Processing voice (5 seconds)")
            state_machine.request_state(ClaudiaLEDMode.PROCESSING_VOICE, "test", duration=5.0)
            time.sleep(2)

            # 3. Executing action (high priority, should interrupt processing voice)
            print("3. Executing action (3 seconds)")
            state_machine.request_state(ClaudiaLEDMode.EXECUTING_ACTION, "test", duration=3.0)
            time.sleep(4)

            # 4. Action complete
            print("4. Action complete")
            state_machine.request_state(ClaudiaLEDMode.ACTION_COMPLETE, "test")
            time.sleep(2)

            # 5. Error state (highest priority)
            print("5. Error state")
            state_machine.request_state(ClaudiaLEDMode.ERROR_STATE, "test")
            time.sleep(3)

            # Display performance metrics
            metrics = state_machine.get_performance_metrics()
            print(f"\nPerformance metrics:")
            print(f"   State changes: {metrics['state_changes']}")
            print(f"   Average response time: {metrics['average_response_time_ms']:.1f}ms")
            print(f"   Max response time: {metrics['max_response_time_ms']:.1f}ms")
            print(f"   Response time requirement: {'PASS' if metrics['meets_response_requirement'] else 'FAIL'}")
            print(f"   Queue overflows: {metrics['queue_overflows']}")
            print(f"   Priority conflicts: {metrics['priority_conflicts']}")

            # Display state history
            history = state_machine.get_state_history(5)
            print(f"\nRecent state history:")
            for i, record in enumerate(history[-3:]):
                duration_str = f"{record.duration:.1f}s" if record.duration else "in progress"
                print(f"   {i+1}. {record.mode.value} ({record.source}) - {duration_str}")

        else:
            print("LED state machine initialization failed")

    except KeyboardInterrupt:
        print("\nUser interrupted test")
    finally:
        state_machine.cleanup()
