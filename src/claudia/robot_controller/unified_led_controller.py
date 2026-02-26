#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia Unified LED Controller - Enhanced Environmental Adaptation Version
Integrates VUI client and LowCmd control methods, providing advanced environmental adaptation

Author: Claudia AI System
Generated: 2025-06-30
Purpose: Subtask 6.3 - Enhanced environmental adaptation (based on subtask 6.2 modifications)
Version: 0.3.0 (Enhanced Environmental Adaptation)
"""

import os
import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any, Union

# cv2/numpy: optional for environments where OpenCV is not installed
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from collections import deque
import statistics
import math

# Add project path (derived from module location, avoiding hardcoded paths)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# Import LED components
try:
    from claudia.robot_controller.led_patterns import (
        ClaudiaLEDMode, LEDPattern, ClaudiaLEDModeDefinitions,
        LEDModeRenderer, create_led_mode_renderer
    )
    from claudia.robot_controller.led_state_machine import (
        LEDStateMachine, create_led_state_machine
    )
    from claudia.robot_controller.led_controller import (
        ClaudiaLEDController, create_led_controller
    )
    # Phase 1.3: Import system state monitor
    from claudia.robot_controller.system_state_monitor import (
        SystemStateMonitor, SystemStateInfo, SystemState, SystemLEDPriority,
        LEDControlDecision, create_system_state_monitor
    )
    LED_COMPONENTS_AVAILABLE = True
    SYSTEM_MONITOR_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: LED component import failed: {e}")
    LED_COMPONENTS_AVAILABLE = False
    SYSTEM_MONITOR_AVAILABLE = False

class LEDControlMethod(Enum):
    """LED control method enumeration"""
    VUI_CLIENT = "vui_client"        # VUI client control (recommended)
    LOW_CMD = "low_cmd"              # LowCmd message control (fallback)
    AUTO_SELECT = "auto_select"      # Auto-select best method

class EnvironmentalLightCategory(Enum):
    """Environmental light category enumeration"""
    VERY_DARK = "very_dark"          # Very dark (<5%)
    DARK = "dark"                    # Dark (5-20%)
    DIM = "dim"                      # Dim (20-40%)
    NORMAL = "normal"                # Normal (40-70%)
    BRIGHT = "bright"                # Bright (70-85%)
    VERY_BRIGHT = "very_bright"      # Very bright (>85%)

@dataclass
class AdvancedEnvironmentalLightInfo:
    """Advanced environmental light information"""
    # Basic light data
    brightness_level: float          # Overall brightness level (0-1)
    brightness_category: EnvironmentalLightCategory  # Brightness category
    suggested_led_factor: float      # Suggested LED adjustment factor (0.1-3.0)
    timestamp: float                 # Detection timestamp
    detection_confidence: float      # Detection confidence (0-1)

    # Advanced analysis data
    brightness_std: float            # Brightness standard deviation (uniformity indicator)
    contrast_ratio: float            # Contrast ratio
    histogram_peaks: List[int]       # Histogram peak positions
    dominant_brightness_range: Tuple[float, float]  # Dominant brightness range
    temporal_stability: float       # Temporal stability (0-1)

    # Environmental analysis
    light_source_type: str           # Estimated light source type (natural/artificial/mixed)
    uniformity_score: float         # Light uniformity score (0-1)
    flicker_detected: bool           # Whether flicker detected
    recommended_adaptation_speed: float  # Recommended adaptation speed (seconds)

    # Metadata
    analysis_method: str = "advanced_v2"  # Analysis method version
    frame_quality: float = 1.0       # Frame quality score

@dataclass
class EnvironmentalAdaptationProfile:
    """Environmental adaptation configuration profile"""
    # Basic adaptation parameters
    min_led_factor: float = 0.1      # Minimum LED factor
    max_led_factor: float = 3.0      # Maximum LED factor
    adaptation_sensitivity: float = 1.0  # Adaptation sensitivity
    temporal_smoothing: float = 0.7  # Temporal smoothing factor

    # Advanced adaptation strategies
    contrast_compensation: bool = True  # Contrast compensation
    flicker_protection: bool = True     # Flicker protection
    source_type_optimization: bool = True  # Light source type optimization
    uniformity_adjustment: bool = True  # Uniformity adjustment

    # Performance optimization
    fast_adaptation_threshold: float = 0.3  # Fast adaptation threshold
    stability_requirement: float = 0.8      # Stability requirement
    confidence_threshold: float = 0.6       # Confidence threshold

class AdvancedEnvironmentalAnalyzer:
    """Advanced environmental light analyzer"""

    def __init__(self, history_size: int = 10):
        """
        Initialize advanced environmental analyzer

        Args:
            history_size: History data cache size
        """
        self.logger = logging.getLogger(__name__)
        self.history_size = history_size

        # History data cache
        self.brightness_history = deque(maxlen=history_size)
        self.analysis_history = deque(maxlen=history_size)

        # Analysis configuration
        self.histogram_bins = 256
        self.flicker_detection_frames = 5
        self.flicker_threshold = 0.15  # 15% change considered as flicker

    def analyze_environmental_light(self, frame: np.ndarray) -> Optional[AdvancedEnvironmentalLightInfo]:
        """
        Advanced environmental light analysis

        Args:
            frame: Input image frame

        Returns:
            Optional[AdvancedEnvironmentalLightInfo]: Analysis result
        """
        try:
            if frame is None or frame.size == 0:
                return None

            # Convert to grayscale
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame.copy()

            # Basic brightness analysis
            mean_brightness = np.mean(gray) / 255.0
            brightness_std = np.std(gray) / 255.0

            # Histogram analysis
            histogram = cv2.calcHist([gray], [0], None, [self.histogram_bins], [0, 256])
            histogram = histogram.flatten() / histogram.sum()  # Normalize

            # Find peaks
            peaks = self._find_histogram_peaks(histogram)

            # Contrast analysis
            contrast_ratio = self._calculate_contrast_ratio(gray)

            # Dominant brightness range
            dominant_range = self._find_dominant_brightness_range(histogram)

            # Estimate light source type
            light_source_type = self._estimate_light_source_type(histogram, mean_brightness, brightness_std)

            # Uniformity score
            uniformity_score = self._calculate_uniformity_score(gray, brightness_std)

            # Temporal stability analysis
            temporal_stability = self._calculate_temporal_stability(mean_brightness)

            # Flicker detection
            flicker_detected = self._detect_flicker(mean_brightness)

            # Brightness classification
            brightness_category = self._classify_brightness(mean_brightness, histogram)

            # Calculate detection confidence
            detection_confidence = self._calculate_detection_confidence(
                brightness_std, temporal_stability, len(self.brightness_history)
            )

            # Calculate LED adjustment factor (advanced algorithm)
            led_factor = self._calculate_advanced_led_factor(
                mean_brightness, brightness_category, contrast_ratio,
                light_source_type, uniformity_score, flicker_detected
            )

            # Recommended adaptation speed
            adaptation_speed = self._calculate_adaptation_speed(
                temporal_stability, flicker_detected, brightness_category
            )

            # Frame quality assessment
            frame_quality = self._assess_frame_quality(gray, brightness_std)

            # Create analysis result
            analysis_result = AdvancedEnvironmentalLightInfo(
                brightness_level=mean_brightness,
                brightness_category=brightness_category,
                suggested_led_factor=led_factor,
                timestamp=time.time(),
                detection_confidence=detection_confidence,
                brightness_std=brightness_std,
                contrast_ratio=contrast_ratio,
                histogram_peaks=peaks,
                dominant_brightness_range=dominant_range,
                temporal_stability=temporal_stability,
                light_source_type=light_source_type,
                uniformity_score=uniformity_score,
                flicker_detected=flicker_detected,
                recommended_adaptation_speed=adaptation_speed,
                frame_quality=frame_quality
            )

            # Update history data
            self.brightness_history.append(mean_brightness)
            self.analysis_history.append(analysis_result)

            return analysis_result

        except Exception as e:
            self.logger.error(f"Advanced environmental light analysis failed: {e}")
            return None

    def _find_histogram_peaks(self, histogram: np.ndarray) -> List[int]:
        """Find histogram peaks"""
        peaks = []
        threshold = np.max(histogram) * 0.1  # 10% threshold

        for i in range(1, len(histogram) - 1):
            if (histogram[i] > histogram[i-1] and
                histogram[i] > histogram[i+1] and
                histogram[i] > threshold):
                peaks.append(i)

        return peaks[:5]  # Return at most 5 peaks

    def _calculate_contrast_ratio(self, gray: np.ndarray) -> float:
        """Calculate contrast ratio"""
        min_val = np.min(gray)
        max_val = np.max(gray)

        if min_val == 0:
            return float('inf') if max_val > 0 else 1.0

        return max_val / min_val

    def _find_dominant_brightness_range(self, histogram: np.ndarray) -> Tuple[float, float]:
        """Find dominant brightness range (minimum range containing 80% of pixels)"""
        cumsum = np.cumsum(histogram)

        # Find 10% and 90% positions
        low_idx = np.searchsorted(cumsum, 0.1)
        high_idx = np.searchsorted(cumsum, 0.9)

        return (low_idx / 255.0, high_idx / 255.0)

    def _estimate_light_source_type(self, histogram: np.ndarray, mean_brightness: float, brightness_std: float) -> str:
        """Estimate light source type"""
        # Artificial light typically has distinct peaks, natural light is smoother
        peaks = self._find_histogram_peaks(histogram)
        num_peaks = len(peaks)

        # Feature analysis
        if num_peaks <= 1 and brightness_std > 0.3:
            return "natural"  # Natural light: single peak with high variation
        elif num_peaks >= 3:
            return "mixed"    # Mixed light sources: multiple peaks
        elif mean_brightness > 0.7 and brightness_std < 0.2:
            return "artificial_bright"  # Bright artificial light: high brightness, low variation
        elif mean_brightness < 0.3 and brightness_std < 0.2:
            return "artificial_dim"     # Dim artificial light: low brightness, low variation
        else:
            return "artificial"  # General artificial light

    def _calculate_uniformity_score(self, gray: np.ndarray, brightness_std: float) -> float:
        """Calculate light uniformity score"""
        # Use standard deviation and local variation to evaluate uniformity
        height, width = gray.shape

        # Calculate local variance
        kernel_size = min(height, width) // 10
        if kernel_size < 3:
            kernel_size = 3

        # Calculate local standard deviation after Gaussian blur
        blurred = cv2.GaussianBlur(gray.astype('float32'), (kernel_size, kernel_size), 0)
        local_variance = np.var(blurred) / (255.0 ** 2)

        # Combine global and local variation
        global_uniformity = 1.0 - min(1.0, brightness_std * 2)
        local_uniformity = 1.0 - min(1.0, local_variance * 5)

        return (global_uniformity + local_uniformity) / 2.0

    def _calculate_temporal_stability(self, current_brightness: float) -> float:
        """Calculate temporal stability"""
        if len(self.brightness_history) < 3:
            return 0.5  # Insufficient history data

        # Calculate standard deviation of brightness changes
        recent_history = list(self.brightness_history)[-5:]  # Last 5 entries
        if len(recent_history) < 2:
            return 0.5

        brightness_variation = statistics.stdev(recent_history)
        stability = 1.0 - min(1.0, brightness_variation * 10)  # 10x sensitivity

        return max(0.0, min(1.0, stability))

    def _detect_flicker(self, current_brightness: float) -> bool:
        """Detect flicker"""
        if len(self.brightness_history) < self.flicker_detection_frames:
            return False

        # Check brightness changes in recent frames
        recent_history = list(self.brightness_history)[-(self.flicker_detection_frames-1):]
        recent_history.append(current_brightness)

        # Calculate change rate between adjacent frames
        changes = []
        for i in range(1, len(recent_history)):
            if recent_history[i-1] > 0:
                change_rate = abs(recent_history[i] - recent_history[i-1]) / recent_history[i-1]
                changes.append(change_rate)

        if not changes:
            return False

        # If multiple large changes detected, consider it flicker
        large_changes = [c for c in changes if c > self.flicker_threshold]
        return len(large_changes) >= 2

    def _classify_brightness(self, mean_brightness: float, histogram: np.ndarray) -> EnvironmentalLightCategory:
        """Brightness classification (advanced algorithm)"""
        # Classify using both mean and histogram distribution
        dominant_range = self._find_dominant_brightness_range(histogram)

        if mean_brightness < 0.05:
            return EnvironmentalLightCategory.VERY_DARK
        elif mean_brightness < 0.2:
            return EnvironmentalLightCategory.DARK
        elif mean_brightness < 0.4:
            return EnvironmentalLightCategory.DIM
        elif mean_brightness < 0.7:
            return EnvironmentalLightCategory.NORMAL
        elif mean_brightness < 0.85:
            return EnvironmentalLightCategory.BRIGHT
        else:
            return EnvironmentalLightCategory.VERY_BRIGHT

    def _calculate_detection_confidence(self, brightness_std: float, temporal_stability: float, history_length: int) -> float:
        """Calculate detection confidence"""
        # Calculate confidence based on multiple factors
        std_confidence = min(1.0, brightness_std * 3)  # Higher std = higher confidence
        stability_confidence = temporal_stability
        history_confidence = min(1.0, history_length / 5.0)  # More history = higher confidence

        return (std_confidence + stability_confidence + history_confidence) / 3.0

    def _calculate_advanced_led_factor(self, mean_brightness: float, category: EnvironmentalLightCategory,
                                     contrast_ratio: float, light_source_type: str,
                                     uniformity_score: float, flicker_detected: bool) -> float:
        """Calculate advanced LED adjustment factor"""
        # Base factor (by brightness category)
        base_factors = {
            EnvironmentalLightCategory.VERY_DARK: 2.5,
            EnvironmentalLightCategory.DARK: 2.0,
            EnvironmentalLightCategory.DIM: 1.5,
            EnvironmentalLightCategory.NORMAL: 1.0,
            EnvironmentalLightCategory.BRIGHT: 0.7,
            EnvironmentalLightCategory.VERY_BRIGHT: 0.4
        }

        base_factor = base_factors.get(category, 1.0)

        # Contrast adjustment
        if contrast_ratio > 50:  # High contrast environment
            contrast_adjustment = 1.2
        elif contrast_ratio < 5:  # Low contrast environment
            contrast_adjustment = 0.8
        else:
            contrast_adjustment = 1.0

        # Light source type adjustment
        source_adjustments = {
            "natural": 1.0,
            "artificial": 0.9,      # Slightly lower under artificial light
            "artificial_bright": 0.8,
            "artificial_dim": 1.1,
            "mixed": 1.05
        }
        source_adjustment = source_adjustments.get(light_source_type, 1.0)

        # Uniformity adjustment
        uniformity_adjustment = 1.0 + (1.0 - uniformity_score) * 0.2  # Slightly increase when non-uniform

        # Flicker protection
        flicker_adjustment = 0.8 if flicker_detected else 1.0

        # Combined calculation
        final_factor = base_factor * contrast_adjustment * source_adjustment * uniformity_adjustment * flicker_adjustment

        return max(0.1, min(3.0, final_factor))

    def _calculate_adaptation_speed(self, temporal_stability: float, flicker_detected: bool,
                                  category: EnvironmentalLightCategory) -> float:
        """Calculate recommended adaptation speed"""
        # Base speed
        if flicker_detected:
            base_speed = 0.5  # Fast adaptation during flicker
        elif temporal_stability > 0.8:
            base_speed = 3.0  # Slow adaptation when stable
        else:
            base_speed = 1.5  # Medium speed

        # Adjust by brightness category
        if category in [EnvironmentalLightCategory.VERY_DARK, EnvironmentalLightCategory.VERY_BRIGHT]:
            speed_adjustment = 0.7  # More cautious under extreme conditions
        else:
            speed_adjustment = 1.0

        return base_speed * speed_adjustment

    def _assess_frame_quality(self, gray: np.ndarray, brightness_std: float) -> float:
        """Assess frame quality"""
        # Evaluate based on sharpness and information content
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        clarity_score = min(1.0, laplacian_var / 1000.0)  # Normalized clarity

        # Information content score (based on standard deviation)
        information_score = min(1.0, brightness_std * 5)

        return (clarity_score + information_score) / 2.0

class UnifiedLEDController:
    """
    Claudia Unified LED Controller - Enhanced Environmental Adaptation Version

    Integrates advanced environmental light analysis and intelligent adaptation
    """

    def __init__(self,
                 preferred_method: LEDControlMethod = LEDControlMethod.VUI_CLIENT,
                 enable_environmental_adaptation: bool = True,
                 camera_device_id: int = 0,
                 adaptation_profile: Optional[EnvironmentalAdaptationProfile] = None):
        """
        Initialize unified LED controller

        Args:
            preferred_method: Preferred control method
            enable_environmental_adaptation: Whether to enable environmental adaptation
            camera_device_id: Front camera device ID
            adaptation_profile: Environmental adaptation configuration profile
        """
        self.logger = logging.getLogger(__name__)
        self.preferred_method = preferred_method
        self.enable_environmental_adaptation = enable_environmental_adaptation
        self.camera_device_id = camera_device_id
        self.adaptation_profile = adaptation_profile or EnvironmentalAdaptationProfile()

        # Core components
        self.state_machine = None
        self.vui_renderer = None
        self.lowcmd_controller = None
        self.is_initialized = False

        # Phase 1.3: System state monitor integration
        self.system_state_monitor = None
        self.system_monitoring_active = False
        self.current_system_state = None
        self.last_led_control_decision = None

        # Advanced environmental analysis
        self.camera = None
        self.environment_analyzer = AdvancedEnvironmentalAnalyzer(history_size=15)
        self.environment_monitor_thread = None
        self.environment_monitoring_active = False
        self.current_environmental_info = None
        self.environment_update_interval = 2.0  # Update every 2 seconds (improved responsiveness)

        # Adaptation state
        self.adaptive_led_factor = 1.0
        self.adaptive_factor_history = deque(maxlen=10)
        self.last_adaptation_time = 0
        self.adaptation_smoothing_active = True

        # Control strategy
        self.active_control_method = None
        self.fallback_available = False
        self.control_lock = threading.Lock()

        # Performance and state
        self.control_attempts = 0
        self.control_successes = 0
        self.method_performance = {
            LEDControlMethod.VUI_CLIENT: {'attempts': 0, 'successes': 0},
            LEDControlMethod.LOW_CMD: {'attempts': 0, 'successes': 0}
        }

        # Advanced monitoring metrics
        self.adaptation_statistics = {
            'total_adaptations': 0,
            'flicker_detections': 0,
            'light_source_changes': 0,
            'rapid_adaptations': 0,
            'average_confidence': 0.0
        }

        self.logger.info("Unified LED controller initialized (Enhanced Environmental Adaptation v0.3.0)")

    def initialize(self) -> bool:
        """
        Initialize unified LED controller

        Returns:
            bool: Whether initialization succeeded
        """
        if not LED_COMPONENTS_AVAILABLE:
            self.logger.error("LED components not available")
            return False

        try:
            self.logger.info("Initializing unified LED controller...")

            # Initialize LED state machine
            self.state_machine = create_led_state_machine()
            if not self.state_machine.initialize():
                self.logger.error("LED state machine initialization failed")
                return False

            # Initialize control methods
            success = self._initialize_control_methods()
            if not success:
                self.logger.error("LED control method initialization failed")
                return False

            # Initialize environmental monitoring
            if self.enable_environmental_adaptation:
                self._initialize_environmental_monitoring()

            # Phase 1.3: Initialize system state monitor
            if SYSTEM_MONITOR_AVAILABLE:
                self._initialize_system_state_monitoring()

            self.is_initialized = True
            self.logger.info(f"Unified LED controller initialized successfully - active method: {self.active_control_method.value}")
            return True

        except Exception as e:
            self.logger.error(f"Unified LED controller initialization failed: {e}")
            return False

    def _initialize_control_methods(self) -> bool:
        """
        Initialize LED control methods

        Returns:
            bool: Whether initialization succeeded
        """
        vui_success = False
        lowcmd_success = False

        # Try initializing VUI control
        try:
            self.vui_renderer = create_led_mode_renderer()
            vui_success = self.vui_renderer.initialize_vui()
            if vui_success:
                self.logger.info("VUI LED control initialized successfully")
            else:
                self.logger.warning("VUI LED control initialization failed")
        except Exception as e:
            self.logger.warning(f"VUI LED control initialization exception: {e}")

        # Try initializing LowCmd control
        try:
            self.lowcmd_controller = create_led_controller()
            lowcmd_success = self.lowcmd_controller.initialize()
            if lowcmd_success:
                self.logger.info("LowCmd LED control initialized successfully")
            else:
                self.logger.warning("LowCmd LED control initialization failed")
        except Exception as e:
            self.logger.warning(f"LowCmd LED control initialization exception: {e}")

        # Determine active control method
        if self.preferred_method == LEDControlMethod.VUI_CLIENT and vui_success:
            self.active_control_method = LEDControlMethod.VUI_CLIENT
            self.fallback_available = lowcmd_success
        elif self.preferred_method == LEDControlMethod.LOW_CMD and lowcmd_success:
            self.active_control_method = LEDControlMethod.LOW_CMD
            self.fallback_available = vui_success
        elif self.preferred_method == LEDControlMethod.AUTO_SELECT:
            if vui_success:
                self.active_control_method = LEDControlMethod.VUI_CLIENT
                self.fallback_available = lowcmd_success
            elif lowcmd_success:
                self.active_control_method = LEDControlMethod.LOW_CMD
                self.fallback_available = False
            else:
                self.logger.error("No available LED control methods")
                return False
        else:
            # Preferred method not available, try fallback
            if vui_success:
                self.active_control_method = LEDControlMethod.VUI_CLIENT
                self.fallback_available = lowcmd_success
            elif lowcmd_success:
                self.active_control_method = LEDControlMethod.LOW_CMD
                self.fallback_available = False
            else:
                self.logger.error("No available LED control methods")
                return False

        if self.fallback_available:
            self.logger.info(f"Fallback control method available: {self._get_fallback_method().value}")

        return True

    def _get_fallback_method(self) -> Optional[LEDControlMethod]:
        """Get fallback control method"""
        if not self.fallback_available:
            return None

        if self.active_control_method == LEDControlMethod.VUI_CLIENT:
            return LEDControlMethod.LOW_CMD
        else:
            return LEDControlMethod.VUI_CLIENT

    def _initialize_environmental_monitoring(self) -> bool:
        """
        Initialize environmental light monitoring

        Returns:
            bool: Whether initialization succeeded
        """
        try:
            self.logger.info("Initializing environmental light monitoring...")

            # Try opening front camera
            self.camera = cv2.VideoCapture(self.camera_device_id)
            if not self.camera.isOpened():
                self.logger.warning(f"Cannot open camera {self.camera_device_id}")
                return False

            # Set camera parameters
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)

            # Start monitoring thread
            self.environment_monitoring_active = True
            self.environment_monitor_thread = threading.Thread(
                target=self._environment_monitoring_worker,
                daemon=True
            )
            self.environment_monitor_thread.start()

            self.logger.info("Environmental light monitoring initialized successfully")
            return True

        except Exception as e:
            self.logger.warning(f"Environmental light monitoring initialization failed: {e}")
            return False

    def _initialize_system_state_monitoring(self) -> bool:
        """
        Phase 1.3: Initialize system state monitor

        Returns:
            bool: Whether initialization succeeded
        """
        try:
            self.logger.info("Initializing system state monitor...")

            # Create system state monitor
            self.system_state_monitor = create_system_state_monitor(
                node_name="claudia_unified_led_system_monitor",
                history_size=50,
                update_rate=10.0
            )

            # Register state change callback
            self.system_state_monitor.register_state_change_callback(
                self._on_system_state_change
            )

            # Register critical event callback
            self.system_state_monitor.register_critical_event_callback(
                self._on_system_critical_event
            )

            # Initialize monitor
            if self.system_state_monitor.initialize():
                # Start monitoring
                if self.system_state_monitor.start_monitoring():
                    self.system_monitoring_active = True
                    self.logger.info("System state monitor initialized successfully")
                    return True
                else:
                    self.logger.warning("System state monitor start failed")
                    return False
            else:
                self.logger.warning("System state monitor initialization failed")
                return False

        except Exception as e:
            self.logger.warning(f"System state monitor initialization failed: {e}")
            return False

    def _on_system_state_change(self, previous_state: SystemState, new_state_info: SystemStateInfo) -> None:
        """
        Phase 1.3 & Phase 2: System state change callback

        Args:
            previous_state: Previous system state
            new_state_info: New system state information
        """
        try:
            self.current_system_state = new_state_info

            self.logger.info(f"System state change: {previous_state.name} -> {new_state_info.state.name} "
                           f"(priority: {new_state_info.priority.name})")

            # Phase 2: Update LED state machine's system state and dynamic priority
            if self.state_machine and hasattr(self.state_machine, 'update_system_state'):
                self.state_machine.update_system_state(new_state_info)

            # Automatically adjust LED display based on system state
            if new_state_info.state in [SystemState.LOW_BATTERY, SystemState.ERROR, SystemState.EMERGENCY]:
                # High priority system state - force display corresponding LED mode
                self._handle_high_priority_system_state(new_state_info)
            elif new_state_info.state == SystemState.CALIBRATING:
                # Calibration state - display calibration indicator
                self._handle_calibration_state(new_state_info)

            # Get and log dynamic priority statistics
            if (self.state_machine and
                hasattr(self.state_machine, 'get_dynamic_priority_statistics')):
                stats = self.state_machine.get_dynamic_priority_statistics()
                if stats:
                    self.logger.debug(f"Dynamic priority statistics: {stats}")

        except Exception as e:
            self.logger.error(f"System state change callback failed: {e}")

    def _on_system_critical_event(self, event_type: str, event_message: str, state_info: SystemStateInfo) -> None:
        """
        Phase 1.3 & Phase 2: System critical event callback

        Args:
            event_type: Event type
            event_message: Event message
            state_info: System state information
        """
        try:
            self.logger.warning(f"Critical system event: {event_type} - {event_message}")

            # Phase 2: Ensure LED state machine knows current system state
            if self.state_machine and hasattr(self.state_machine, 'update_system_state'):
                self.state_machine.update_system_state(state_info)

            # Phase 2: Check LED control decision
            led_control_decision = None
            if (self.state_machine and
                hasattr(self.state_machine, 'get_led_control_decision')):

                # Determine target LED mode based on event type
                target_mode = ClaudiaLEDMode.ERROR_STATE
                target_priority = 9

                if event_type == "critical_battery":
                    target_mode = ClaudiaLEDMode.ERROR_STATE
                    target_priority = 10  # Highest priority
                elif event_type == "low_battery":
                    target_mode = ClaudiaLEDMode.LOW_BATTERY
                    target_priority = 8
                elif event_type in ["high_temperature", "system_errors"]:
                    target_mode = ClaudiaLEDMode.ERROR_STATE
                    target_priority = 9

                led_control_decision = self.state_machine.get_led_control_decision(target_mode, target_priority)

                if led_control_decision:
                    self.logger.info(f"LED control decision: {led_control_decision.recommended_action} - {led_control_decision.reason}")

            # Execute corresponding action based on event type (using intelligent decision)
            if event_type == "critical_battery":
                # Critically low battery - force display red warning
                self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_critical_battery", force=True)
            elif event_type == "low_battery":
                # Low battery - display yellow warning (using dynamic priority)
                priority = (led_control_decision.required_priority.value
                           if led_control_decision else 8)
                self.set_mode(ClaudiaLEDMode.LOW_BATTERY, "system_low_battery", priority_override=priority)
            elif event_type == "high_temperature":
                # High temperature - display temperature warning (using dynamic priority)
                priority = (led_control_decision.required_priority.value
                           if led_control_decision else 9)
                self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_high_temperature", priority_override=priority)
            elif event_type == "system_errors":
                # System error - display error state (using dynamic priority)
                priority = (led_control_decision.required_priority.value
                           if led_control_decision else 10)
                self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_error", priority_override=priority)

        except Exception as e:
            self.logger.error(f"System critical event callback failed: {e}")

    def _handle_high_priority_system_state(self, state_info: SystemStateInfo) -> None:
        """
        Phase 2: Handle high priority system state (integrated with dynamic priority)
        """
        try:
            # Get LED control decision
            led_decision = None
            if (self.state_machine and
                hasattr(self.state_machine, 'get_led_control_decision')):

                if state_info.state == SystemState.LOW_BATTERY:
                    # Low battery state
                    battery_pct = state_info.battery_level * 100
                    if battery_pct <= 5:
                        # Critically low battery - red flash
                        led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.ERROR_STATE, 10)
                        self.set_mode(ClaudiaLEDMode.ERROR_STATE, "critical_battery", force=True)
                    else:
                        # Low battery - yellow indicator
                        led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.LOW_BATTERY, 8)
                        priority = led_decision.required_priority.value if led_decision else 8
                        self.set_mode(ClaudiaLEDMode.LOW_BATTERY, "low_battery", priority_override=priority)

                elif state_info.state == SystemState.ERROR:
                    # Error state - red flash
                    led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.ERROR_STATE, 9)
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_error", force=True)

                elif state_info.state == SystemState.EMERGENCY:
                    # Emergency state - rapid red flash
                    led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.ERROR_STATE, 10)
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "emergency", force=True)

            else:
                # Fallback logic (when no dynamic priority manager)
                if state_info.state == SystemState.LOW_BATTERY:
                    battery_pct = state_info.battery_level * 100
                    if battery_pct <= 5:
                        self.set_mode(ClaudiaLEDMode.ERROR_STATE, "critical_battery", force=True)
                    else:
                        self.set_mode(ClaudiaLEDMode.LOW_BATTERY, "low_battery", priority_override=8)
                elif state_info.state == SystemState.ERROR:
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "system_error", force=True)
                elif state_info.state == SystemState.EMERGENCY:
                    self.set_mode(ClaudiaLEDMode.ERROR_STATE, "emergency", force=True)

            if led_decision:
                self.logger.debug(f"High priority state LED decision: {led_decision.recommended_action}")

        except Exception as e:
            self.logger.error(f"High priority system state handling failed: {e}")

    def _handle_calibration_state(self, state_info: SystemStateInfo) -> None:
        """
        Phase 2: Handle calibration state (integrated with dynamic priority)
        """
        try:
            # Get calibration LED control decision
            led_decision = None
            if (self.state_machine and
                hasattr(self.state_machine, 'get_led_control_decision')):
                led_decision = self.state_machine.get_led_control_decision(ClaudiaLEDMode.SYSTEM_CALIBRATION, 7)

                if led_decision and led_decision.allow_custom_control:
                    priority = led_decision.required_priority.value
                    self.set_mode(ClaudiaLEDMode.SYSTEM_CALIBRATION, "system_calibrating", priority_override=priority)
                    self.logger.debug(f"Calibration state LED decision: {led_decision.recommended_action}")
                else:
                    self.logger.warning(f"Calibration LED blocked by system: {led_decision.reason if led_decision else 'unknown reason'}")
            else:
                # Fallback logic
                self.set_mode(ClaudiaLEDMode.SYSTEM_CALIBRATION, "system_calibrating", priority_override=7)

        except Exception as e:
            self.logger.error(f"Calibration state handling failed: {e}")

    def _environment_monitoring_worker(self) -> None:
        """Advanced environmental light monitoring worker thread"""
        last_light_source_type = None
        confidence_history = deque(maxlen=5)

        while self.environment_monitoring_active:
            try:
                # Detect environmental light
                env_info = self._detect_environmental_light()
                if env_info:
                    self.current_environmental_info = env_info
                    confidence_history.append(env_info.detection_confidence)

                    # Apply intelligent adaptation strategy
                    self._apply_intelligent_adaptation(env_info)

                    # Detect light source type change
                    if (last_light_source_type is not None and
                        last_light_source_type != env_info.light_source_type):
                        self.adaptation_statistics['light_source_changes'] += 1
                        self.logger.info(f"Light source type change detected: {last_light_source_type} -> {env_info.light_source_type}")

                    last_light_source_type = env_info.light_source_type

                    # Update statistics
                    self.adaptation_statistics['total_adaptations'] += 1
                    if env_info.flicker_detected:
                        self.adaptation_statistics['flicker_detections'] += 1

                    if confidence_history:
                        self.adaptation_statistics['average_confidence'] = statistics.mean(confidence_history)

                    # Dynamically adjust update interval
                    self._adjust_monitoring_interval(env_info)

                # Wait for next update
                time.sleep(self.environment_update_interval)

            except Exception as e:
                self.logger.error(f"Environmental light monitoring error: {e}")
                time.sleep(self.environment_update_interval)

    def _apply_intelligent_adaptation(self, env_info: AdvancedEnvironmentalLightInfo) -> None:
        """
        Apply intelligent adaptation strategy

        Args:
            env_info: Environmental light information
        """
        try:
            current_time = time.time()

            # Check confidence threshold
            if env_info.detection_confidence < self.adaptation_profile.confidence_threshold:
                self.logger.debug(f"Detection confidence too low, skipping adaptation: {env_info.detection_confidence:.2f}")
                return

            # Apply temporal smoothing
            if self.adaptation_smoothing_active:
                new_factor = self._apply_temporal_smoothing(env_info.suggested_led_factor)
            else:
                new_factor = env_info.suggested_led_factor

            # Check if rapid adaptation is needed
            rapid_adaptation_needed = (
                env_info.flicker_detected or
                env_info.temporal_stability < self.adaptation_profile.fast_adaptation_threshold or
                abs(new_factor - self.adaptive_led_factor) > 0.5
            )

            if rapid_adaptation_needed:
                self.adaptation_statistics['rapid_adaptations'] += 1
                self.logger.debug("Enabling rapid adaptation mode")

            # Apply new adjustment factor
            if abs(new_factor - self.adaptive_led_factor) > 0.05:  # 5% change threshold
                self.adaptive_led_factor = new_factor
                self.adaptive_factor_history.append(new_factor)
                self.last_adaptation_time = current_time

                # Update LED renderer
                if self.vui_renderer:
                    self.vui_renderer.set_environmental_brightness_factor(new_factor)

                self.logger.debug(f"Adaptive LED factor updated: {new_factor:.2f} (confidence={env_info.detection_confidence:.2f})")

        except Exception as e:
            self.logger.error(f"Intelligent adaptation application failed: {e}")

    def _apply_temporal_smoothing(self, new_factor: float) -> float:
        """
        Apply temporal smoothing

        Args:
            new_factor: New adjustment factor

        Returns:
            float: Smoothed adjustment factor
        """
        if not self.adaptive_factor_history:
            return new_factor

        # Calculate historical average
        recent_factors = list(self.adaptive_factor_history)[-3:]  # Last 3 entries
        if recent_factors:
            historical_average = statistics.mean(recent_factors)

            # Apply smoothing factor
            smoothing = self.adaptation_profile.temporal_smoothing
            smoothed_factor = smoothing * historical_average + (1.0 - smoothing) * new_factor

            return max(self.adaptation_profile.min_led_factor,
                      min(self.adaptation_profile.max_led_factor, smoothed_factor))

        return new_factor

    def _adjust_monitoring_interval(self, env_info: AdvancedEnvironmentalLightInfo) -> None:
        """
        Dynamically adjust monitoring interval

        Args:
            env_info: Environmental light information
        """
        # Adjust interval based on environmental stability and recommended adaptation speed
        base_interval = 2.0

        if env_info.flicker_detected:
            # Increase monitoring frequency during flicker
            self.environment_update_interval = max(0.5, base_interval * 0.25)
        elif env_info.temporal_stability > 0.9:
            # Decrease monitoring frequency when environment is stable
            self.environment_update_interval = min(5.0, base_interval * 2.0)
        else:
            # Use recommended adaptation speed
            speed_factor = env_info.recommended_adaptation_speed / 2.0
            self.environment_update_interval = max(0.5, min(5.0, speed_factor))

    def _detect_environmental_light(self) -> Optional[AdvancedEnvironmentalLightInfo]:
        """
        Detect environmental light conditions

        Returns:
            Optional[AdvancedEnvironmentalLightInfo]: Environmental light information
        """
        if not self.camera or not self.camera.isOpened():
            return None

        try:
            # Capture frame
            ret, frame = self.camera.read()
            if not ret or frame is None:
                return None

            # Analyze environmental light
            env_info = self.environment_analyzer.analyze_environmental_light(frame)
            if env_info:
                self.current_environmental_info = env_info

                # Update LED renderer environmental adjustment factor
                if self.vui_renderer:
                    self.vui_renderer.set_environmental_brightness_factor(env_info.suggested_led_factor)

                self.logger.debug(f"Environmental light analysis successful: {env_info.brightness_category} (factor={env_info.suggested_led_factor:.2f})")

                return env_info

        except Exception as e:
            self.logger.error(f"Environmental light detection failed: {e}")
            return None

    def set_mode(self,
                mode: ClaudiaLEDMode,
                source: str = "unified_controller",
                duration: Optional[float] = None,
                priority_override: Optional[int] = None,
                force: bool = False) -> bool:
        """
        Set LED mode (main interface)

        Args:
            mode: LED mode
            source: Source identifier
            duration: Optional duration override
            priority_override: Optional priority override
            force: Whether to force execution (ignoring priority)

        Returns:
            bool: Whether setting succeeded
        """
        if not self.is_initialized:
            self.logger.error("Unified LED controller not initialized")
            return False

        self.control_attempts += 1

        try:
            with self.control_lock:
                if force:
                    # Force mode
                    success = self.state_machine.force_state(mode, source)
                else:
                    # Normal mode
                    success = self.state_machine.request_state(
                        mode=mode,
                        source=source,
                        duration=duration,
                        priority_override=priority_override
                    )

                if success:
                    self.control_successes += 1
                    self._update_method_performance(self.active_control_method, True)
                    self.logger.info(f"LED mode set successfully: {mode.value} (method={self.active_control_method.value})")
                else:
                    self._update_method_performance(self.active_control_method, False)

                    # Try using fallback method
                    if self.fallback_available:
                        self.logger.info("Trying fallback control method...")
                        success = self._try_fallback_method(mode, source, duration, priority_override, force)

                        if success:
                            self.logger.info(f"Fallback method succeeded: {mode.value}")
                            self.control_successes += 1

                return success

        except Exception as e:
            self.logger.error(f"LED mode setting failed: {e}")
            self._update_method_performance(self.active_control_method, False)
            return False

    def _try_fallback_method(self, mode: ClaudiaLEDMode, source: str, duration: Optional[float],
                           priority_override: Optional[int], force: bool) -> bool:
        """
        Try using fallback control method

        Returns:
            bool: Whether successful
        """
        fallback_method = self._get_fallback_method()
        if not fallback_method:
            return False

        try:
            # Temporarily switch to fallback method
            original_method = self.active_control_method
            self.active_control_method = fallback_method

            # Execute control using fallback method
            if fallback_method == LEDControlMethod.VUI_CLIENT and self.vui_renderer:
                pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
                success = self.vui_renderer.render_mode(mode, duration)
            elif fallback_method == LEDControlMethod.LOW_CMD and self.lowcmd_controller:
                # Use LowCmd method (simplified implementation)
                pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
                r, g, b = pattern.color
                led_data = [r//21, g//21, b//21] * 4  # Simple mapping to 12 bytes
                success = self.lowcmd_controller.set_led_direct(led_data)
            else:
                success = False

            self._update_method_performance(fallback_method, success)

            if success:
                self.logger.info(f"Fallback method succeeded, switching active method: {original_method.value} -> {fallback_method.value}")
                # If fallback method succeeds, consider making it the new active method
                self.fallback_available = True  # Original method becomes fallback
            else:
                # Fallback method also failed, restore original method
                self.active_control_method = original_method

            return success

        except Exception as e:
            self.logger.error(f"Fallback method execution failed: {e}")
            return False

    def _update_method_performance(self, method: LEDControlMethod, success: bool) -> None:
        """
        Update control method performance statistics

        Args:
            method: Control method
            success: Whether successful
        """
        if method in self.method_performance:
            self.method_performance[method]['attempts'] += 1
            if success:
                self.method_performance[method]['successes'] += 1

    # Claudia dedicated shortcut methods
    def wake_confirm(self, source: str = "claudia") -> bool:
        """Wake confirmation: green double flash"""
        return self.set_mode(ClaudiaLEDMode.WAKE_CONFIRM, source)

    def processing_voice(self, source: str = "claudia") -> bool:
        """Processing voice: blue steady"""
        return self.set_mode(ClaudiaLEDMode.PROCESSING_VOICE, source)

    def executing_action(self, source: str = "claudia") -> bool:
        """Executing action: orange steady"""
        return self.set_mode(ClaudiaLEDMode.EXECUTING_ACTION, source)

    def action_complete(self, source: str = "claudia") -> bool:
        """Action complete: white short flash x3"""
        return self.set_mode(ClaudiaLEDMode.ACTION_COMPLETE, source)

    def error_state(self, source: str = "claudia") -> bool:
        """Error state: red triple flash"""
        return self.set_mode(ClaudiaLEDMode.ERROR_STATE, source)

    def turn_off(self, source: str = "claudia") -> bool:
        """Turn off LED"""
        return self.set_mode(ClaudiaLEDMode.OFF, source)

    # State query methods
    def get_current_mode(self) -> Tuple[ClaudiaLEDMode, int]:
        """Get current LED mode and priority"""
        if self.state_machine:
            return self.state_machine.get_current_state()
        return ClaudiaLEDMode.OFF, 1

    def get_environmental_info(self) -> Optional[AdvancedEnvironmentalLightInfo]:
        """Get current environmental light information"""
        return self.current_environmental_info

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get advanced performance metrics"""
        overall_success_rate = self.control_successes / max(1, self.control_attempts)

        metrics = {
            'overall_success_rate': overall_success_rate,
            'total_attempts': self.control_attempts,
            'total_successes': self.control_successes,
            'active_method': self.active_control_method.value if self.active_control_method else None,
            'fallback_available': self.fallback_available,
            'environmental_adaptation': self.enable_environmental_adaptation,
            'method_performance': {},
            'adaptation_statistics': self.adaptation_statistics.copy(),
            'current_adaptive_factor': self.adaptive_led_factor,
            'adaptation_profile': {
                'min_led_factor': self.adaptation_profile.min_led_factor,
                'max_led_factor': self.adaptation_profile.max_led_factor,
                'confidence_threshold': self.adaptation_profile.confidence_threshold,
                'temporal_smoothing': self.adaptation_profile.temporal_smoothing
            }
        }

        # Per-method performance statistics
        for method, stats in self.method_performance.items():
            if stats['attempts'] > 0:
                success_rate = stats['successes'] / stats['attempts']
                metrics['method_performance'][method.value] = {
                    'success_rate': success_rate,
                    'attempts': stats['attempts'],
                    'successes': stats['successes']
                }

        # Environmental analyzer statistics
        if hasattr(self.environment_analyzer, 'brightness_history'):
            history_length = len(self.environment_analyzer.brightness_history)
            if history_length > 0:
                recent_brightness = list(self.environment_analyzer.brightness_history)
                metrics['environmental_analysis'] = {
                    'history_length': history_length,
                    'current_brightness': recent_brightness[-1] if recent_brightness else 0,
                    'brightness_stability': statistics.stdev(recent_brightness) if len(recent_brightness) > 1 else 0,
                    'update_interval': self.environment_update_interval
                }

        # State machine performance metrics
        if self.state_machine:
            state_metrics = self.state_machine.get_performance_metrics()
            metrics.update(state_metrics)

            # Phase 2: Add dynamic priority statistics
            if hasattr(self.state_machine, 'get_dynamic_priority_statistics'):
                dynamic_stats = self.state_machine.get_dynamic_priority_statistics()
                if dynamic_stats:
                    metrics['dynamic_priority_stats'] = dynamic_stats

        return metrics

    # Phase 2: New intelligent decision methods
    def get_current_system_state(self) -> Optional[SystemStateInfo]:
        """
        Get current system state information

        Returns:
            Optional[SystemStateInfo]: Current system state (if available)
        """
        return self.current_system_state

    def get_led_control_decision(self, mode: ClaudiaLEDMode, priority: int) -> Optional['LEDControlDecision']:
        """
        Phase 2: Get LED control decision

        Args:
            mode: LED mode
            priority: Request priority

        Returns:
            Optional[LEDControlDecision]: Control decision (if available)
        """
        if (self.state_machine and
            hasattr(self.state_machine, 'get_led_control_decision')):
            return self.state_machine.get_led_control_decision(mode, priority)
        return None

    def set_auto_mode_switching(self, enabled: bool) -> None:
        """
        Phase 2: Set auto mode switching

        Args:
            enabled: Whether to enable auto mode switching
        """
        if (self.state_machine and
            hasattr(self.state_machine, 'set_auto_mode_switching')):
            self.state_machine.set_auto_mode_switching(enabled)
            self.logger.info(f"Auto mode switching {'enabled' if enabled else 'disabled'}")
        else:
            self.logger.warning("LED state machine does not support auto mode switching")

    def get_dynamic_priority_statistics(self) -> Optional[Dict[str, Any]]:
        """
        Phase 2: Get dynamic priority statistics

        Returns:
            Optional[Dict[str, Any]]: Statistics (if available)
        """
        if (self.state_machine and
            hasattr(self.state_machine, 'get_dynamic_priority_statistics')):
            return self.state_machine.get_dynamic_priority_statistics()
        return None

    def simulate_system_state_change(self, state_info: SystemStateInfo) -> None:
        """
        Phase 2: Simulate system state change (for testing)

        Args:
            state_info: System state information to simulate
        """
        self.logger.info(f"Simulating system state change: {state_info.state.name}")

        # Directly call state change callback
        previous_state = self.current_system_state.state if self.current_system_state else SystemState.UNKNOWN
        self._on_system_state_change(previous_state, state_info)

    def force_dynamic_priority_recalculation(self) -> bool:
        """
        Phase 2: Force recalculate dynamic priority

        Returns:
            bool: Whether successful
        """
        try:
            if (self.state_machine and
                hasattr(self.state_machine, '_reevaluate_current_priority')):
                self.state_machine._reevaluate_current_priority()
                self.logger.info("Forced dynamic priority recalculation complete")
                return True
            else:
                self.logger.warning("LED state machine does not support dynamic priority recalculation")
                return False
        except Exception as e:
            self.logger.error(f"Forced dynamic priority recalculation failed: {e}")
            return False

    # Advanced environmental adaptation control methods
    def set_adaptation_profile(self, profile: EnvironmentalAdaptationProfile) -> None:
        """
        Set environmental adaptation configuration profile

        Args:
            profile: New adaptation profile
        """
        self.adaptation_profile = profile
        self.logger.info(f"Environmental adaptation profile updated: confidence_threshold={profile.confidence_threshold}, smoothing_factor={profile.temporal_smoothing}")

    def enable_adaptation_smoothing(self, enable: bool) -> None:
        """
        Enable or disable adaptation smoothing

        Args:
            enable: Whether to enable smoothing
        """
        self.adaptation_smoothing_active = enable
        self.logger.info(f"Adaptation smoothing: {'enabled' if enable else 'disabled'}")

    def get_adaptation_statistics(self) -> Dict[str, Any]:
        """Get detailed adaptation statistics"""
        stats = self.adaptation_statistics.copy()

        # Add real-time information
        if self.current_environmental_info:
            env_info = self.current_environmental_info
            stats['current_environment'] = {
                'brightness_category': env_info.brightness_category.value,
                'suggested_led_factor': env_info.suggested_led_factor,
                'detection_confidence': env_info.detection_confidence,
                'light_source_type': env_info.light_source_type,
                'temporal_stability': env_info.temporal_stability,
                'flicker_detected': env_info.flicker_detected,
                'uniformity_score': env_info.uniformity_score,
                'contrast_ratio': env_info.contrast_ratio
            }

        # Historical factor statistics
        if self.adaptive_factor_history:
            factor_history = list(self.adaptive_factor_history)
            stats['factor_statistics'] = {
                'current_factor': self.adaptive_led_factor,
                'average_factor': statistics.mean(factor_history),
                'factor_range': (min(factor_history), max(factor_history)),
                'factor_stability': statistics.stdev(factor_history) if len(factor_history) > 1 else 0
            }

        return stats

    def force_adaptation_update(self) -> bool:
        """
        Force execute one environmental detection and adaptation update

        Returns:
            bool: Whether update succeeded
        """
        try:
            env_info = self._detect_environmental_light()
            if env_info:
                self._apply_intelligent_adaptation(env_info)
                self.logger.info(f"Forced adaptation update complete: factor={self.adaptive_led_factor:.2f}")
                return True
            else:
                self.logger.warning("Forced adaptation update failed: cannot obtain environmental info")
                return False
        except Exception as e:
            self.logger.error(f"Forced adaptation update exception: {e}")
            return False

    def reset_adaptation_statistics(self) -> None:
        """Reset adaptation statistics"""
        self.adaptation_statistics = {
            'total_adaptations': 0,
            'flicker_detections': 0,
            'light_source_changes': 0,
            'rapid_adaptations': 0,
            'average_confidence': 0.0
        }
        self.adaptive_factor_history.clear()
        self.logger.info("Adaptation statistics reset")

    def emergency_stop(self) -> bool:
        """Emergency stop all LED activity"""
        self.logger.warning("Unified LED controller emergency stop")

        success = True

        # Stop state machine
        if self.state_machine:
            success &= self.state_machine.emergency_stop()

        # Stop VUI renderer
        if self.vui_renderer:
            self.vui_renderer.stop_all_rendering()

        # Stop LowCmd controller
        if self.lowcmd_controller:
            self.lowcmd_controller.set_led_direct([0] * 12)

        return success

    def cleanup(self) -> None:
        """Clean up resources"""
        self.logger.info("Cleaning up unified LED controller resources...")

        try:
            # Stop environmental monitoring
            self.environment_monitoring_active = False
            if self.environment_monitor_thread and self.environment_monitor_thread.is_alive():
                self.environment_monitor_thread.join(timeout=2.0)

            # Phase 1.3: Stop system state monitor
            if self.system_state_monitor and self.system_monitoring_active:
                self.system_monitoring_active = False
                self.system_state_monitor.stop_monitoring()
                self.system_state_monitor.cleanup()
                self.logger.info("System state monitor cleaned up")

            # Release camera
            if self.camera:
                self.camera.release()

            # Clean up LED components
            if self.state_machine:
                self.state_machine.cleanup()

            if self.vui_renderer:
                self.vui_renderer.cleanup()

            if self.lowcmd_controller:
                self.lowcmd_controller.cleanup()

            self.is_initialized = False
            self.logger.info("Unified LED controller cleanup complete")

        except Exception as e:
            self.logger.error(f"Unified LED controller cleanup failed: {e}")


# Factory functions
def create_unified_led_controller(
    preferred_method: LEDControlMethod = LEDControlMethod.VUI_CLIENT,
    enable_environmental_adaptation: bool = True,
    adaptation_profile: Optional[EnvironmentalAdaptationProfile] = None) -> UnifiedLEDController:
    """
    Create unified LED controller instance (advanced environmental adaptation version)

    Args:
        preferred_method: Preferred control method
        enable_environmental_adaptation: Whether to enable environmental adaptation
        adaptation_profile: Environmental adaptation configuration profile

    Returns:
        UnifiedLEDController: Controller instance
    """
    return UnifiedLEDController(
        preferred_method=preferred_method,
        enable_environmental_adaptation=enable_environmental_adaptation,
        adaptation_profile=adaptation_profile
    )

def create_enhanced_led_controller(
    camera_device_id: int = 0,
    adaptation_sensitivity: float = 1.0,
    temporal_smoothing: float = 0.7,
    confidence_threshold: float = 0.6) -> UnifiedLEDController:
    """
    Create enhanced environmental adaptation LED controller

    Args:
        camera_device_id: Camera device ID
        adaptation_sensitivity: Adaptation sensitivity
        temporal_smoothing: Temporal smoothing factor
        confidence_threshold: Confidence threshold

    Returns:
        UnifiedLEDController: Enhanced controller instance
    """
    enhanced_profile = EnvironmentalAdaptationProfile(
        adaptation_sensitivity=adaptation_sensitivity,
        temporal_smoothing=temporal_smoothing,
        confidence_threshold=confidence_threshold,
        contrast_compensation=True,
        flicker_protection=True,
        source_type_optimization=True,
        uniformity_adjustment=True
    )

    return UnifiedLEDController(
        preferred_method=LEDControlMethod.VUI_CLIENT,
        enable_environmental_adaptation=True,
        camera_device_id=camera_device_id,
        adaptation_profile=enhanced_profile
    )


if __name__ == "__main__":
    # Basic test
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    print("Unified LED Controller Test")
    print("=" * 50)

    try:
        # Create controller
        controller = create_unified_led_controller()

        if controller.initialize():
            print("Unified LED controller initialized successfully")

            # Test Claudia dedicated LED sequence
            print("\nTesting Claudia LED interaction sequence...")

            print("1. User wakes up Claudia")
            controller.wake_confirm("test_sequence")
            time.sleep(3)

            print("2. Claudia starts processing voice")
            controller.processing_voice("test_sequence")
            time.sleep(2)

            print("3. Claudia executing user command")
            controller.executing_action("test_sequence")
            time.sleep(3)

            print("4. Command execution complete")
            controller.action_complete("test_sequence")
            time.sleep(2)

            print("5. Simulating error condition")
            controller.error_state("test_sequence")
            time.sleep(3)

            print("6. Turning off LED")
            controller.turn_off("test_sequence")

            # Display performance metrics
            metrics = controller.get_performance_metrics()
            print(f"\nPerformance report:")
            print(f"   Overall success rate: {metrics['overall_success_rate']*100:.1f}%")
            print(f"   Total operations: {metrics['total_attempts']}")
            print(f"   Active control method: {metrics['active_method']}")
            print(f"   Fallback available: {'yes' if metrics['fallback_available'] else 'no'}")
            print(f"   Environmental adaptation: {'yes' if metrics['environmental_adaptation'] else 'no'}")

            # Environmental info
            env_info = controller.get_environmental_info()
            if env_info:
                print(f"\nEnvironmental light info:")
                print(f"   Brightness category: {env_info.brightness_category}")
                print(f"   Brightness level: {env_info.brightness_level:.2f}")
                print(f"   LED adjustment factor: {env_info.suggested_led_factor:.2f}")
                print(f"   Detection confidence: {env_info.detection_confidence:.2f}")

        else:
            print("Unified LED controller initialization failed")

    except KeyboardInterrupt:
        print("\nUser interrupted test")
    finally:
        controller.cleanup()
