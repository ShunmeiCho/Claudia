#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_safety_compiler.py — SafetyCompiler unit tests

Validates all paths of the unified safety pipeline:
  - Step 0a: Input contract (battery validation)
  - Step 0b: Snapshot freshness
  - Step 1: Whitelist
  - Step 2: High-risk policy gate
  - Step 3: Battery gate (3-tier: 0.10/0.20/0.30)
  - Step 4: Standing prerequisite
  - Step 5: Sequence truncation (first-reject/mid-reject/downgrade)
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.safety_compiler import SafetyCompiler, SafetyVerdict
from claudia.brain.action_registry import (
    EXECUTABLE_API_CODES, VALID_API_CODES,
    HIGH_ENERGY_ACTIONS, REQUIRE_STANDING,
)


def make_compiler(**kwargs):
    """Create SafetyCompiler instance (default allow_high_risk=False)"""
    defaults = dict(downgrade_target=1023, snapshot_max_age=5.0, allow_high_risk=False)
    defaults.update(kwargs)
    return SafetyCompiler(**defaults)


# === Step 0a: Input contract ===

class TestInputContract:
    """Battery input validation — fail-safe rejection"""

    def test_battery_none_blocked(self):
        """battery_level=None -> is_blocked"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=None, is_standing=True)
        assert v.is_blocked
        assert "missing" in v.block_reason.lower()

    def test_battery_string_blocked(self):
        """battery_level='high' -> is_blocked"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level="high", is_standing=True)
        assert v.is_blocked

    def test_battery_over_1_blocked(self):
        """battery_level=80.0 -> is_blocked (no auto /100 correction)"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=80.0, is_standing=True)
        assert v.is_blocked
        assert "1.0" in v.block_reason
        assert v.response_override is not None

    def test_battery_exactly_1_passes(self):
        """battery_level=1.0 -> not blocked"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=1.0, is_standing=True)
        assert not v.is_blocked
        assert 1004 in v.executable_sequence

    def test_battery_negative_clamped(self):
        """battery_level=-0.1 -> clamped to 0.0, with warning"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=-0.1, is_standing=True)
        assert not v.is_blocked
        assert any("< 0" in w for w in v.warnings)
        # 1004 is a SAFE_ACTION, passes even at battery=0.0
        assert 1004 in v.executable_sequence

    def test_battery_1_01_blocked(self):
        """battery_level=1.01 -> is_blocked (boundary)"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=1.01, is_standing=True)
        assert v.is_blocked


# === Step 0b: Snapshot freshness ===

class TestSnapshotFreshness:
    """Snapshot freshness check"""

    def test_fresh_snapshot_passes(self):
        """Fresh snapshot (1s ago) does not affect result"""
        sc = make_compiler()
        ts = time.monotonic() - 1.0  # 1 second ago
        v = sc.compile([1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=ts)
        assert not v.is_blocked
        assert 1016 in v.executable_sequence

    def test_stale_snapshot_filters_to_safe(self):
        """Stale snapshot -> non-safe actions filtered, safe actions retained"""
        sc = make_compiler()
        ts = time.monotonic() - 10.0  # 10 seconds ago (> 5s threshold)
        # 1004 is a safe action, 1016(Hello) requires standing (non-safe category)
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=ts)
        assert not v.is_blocked
        assert 1004 in v.executable_sequence
        # 1016 is not in SAFE_ACTIONS, should be filtered out
        assert 1016 not in v.executable_sequence
        assert any("expired" in w.lower() or "Expired" in w for w in v.warnings)

    def test_stale_snapshot_all_unsafe_blocked(self):
        """Stale snapshot + all non-safe actions -> is_blocked"""
        sc = make_compiler()
        ts = time.monotonic() - 10.0
        v = sc.compile([1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=ts)
        assert v.is_blocked
        assert "expired" in v.block_reason.lower() or "Expired" in v.block_reason

    def test_no_timestamp_skips_check(self):
        """snapshot_timestamp=None -> skip freshness check"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=True,
                        snapshot_timestamp=None)
        assert not v.is_blocked
        assert 1016 in v.executable_sequence


# === Step 1: Whitelist ===

class TestWhitelist:
    """Whitelist validation"""

    def test_valid_action_passes(self):
        """Normal action passes whitelist"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]

    def test_invalid_first_action_blocked(self):
        """First action not in whitelist -> entire request rejected"""
        sc = make_compiler()
        v = sc.compile([9999], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert 9999 in v.rejected
        assert "illegal" in v.block_reason.lower() or "First action" in v.block_reason

    def test_invalid_mid_action_truncated(self):
        """Invalid action mid-sequence -> truncated, keeping passed portion"""
        sc = make_compiler()
        v = sc.compile([1004, 9999, 1005], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 9999 in v.rejected

    def test_disabled_action_blocked(self):
        """Disabled action (1042 LeftFlip) is not in whitelist"""
        sc = make_compiler()
        assert 1042 not in EXECUTABLE_API_CODES
        v = sc.compile([1042], battery_level=0.80, is_standing=True)
        assert v.is_blocked

    def test_parameterized_with_safe_defaults_passes(self):
        """Pose(1028) has safe_default_params -> in EXECUTABLE_API_CODES"""
        sc = make_compiler()
        assert 1028 in EXECUTABLE_API_CODES
        assert 1028 not in VALID_API_CODES  # But not in LLM whitelist
        v = sc.compile([1028], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert 1028 in v.executable_sequence

    def test_parameterized_without_defaults_blocked(self):
        """Move(1008) has no safe_default_params -> not in EXECUTABLE_API_CODES"""
        sc = make_compiler()
        assert 1008 not in EXECUTABLE_API_CODES
        v = sc.compile([1008], battery_level=0.80, is_standing=True)
        assert v.is_blocked


# === Step 2: High-risk policy gate ===

class TestHighRiskGate:
    """High-risk action policy gate"""

    def test_high_risk_blocked_by_default(self):
        """allow_high_risk=False -> FrontFlip(1030) rejected"""
        sc = make_compiler(allow_high_risk=False)
        v = sc.compile([1030], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert 1030 in v.rejected
        assert "high-risk" in v.block_reason.lower() or "disabled" in v.block_reason.lower() or "High-risk" in v.block_reason

    def test_high_risk_allowed_when_enabled(self):
        """allow_high_risk=True -> FrontFlip(1030) passes"""
        sc = make_compiler(allow_high_risk=True)
        v = sc.compile([1030], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert 1030 in v.executable_sequence

    def test_all_high_energy_blocked(self):
        """All HIGH_ENERGY_ACTIONS rejected by default policy"""
        sc = make_compiler(allow_high_risk=False)
        for code in HIGH_ENERGY_ACTIONS:
            if code in EXECUTABLE_API_CODES:  # Only test enabled ones
                v = sc.compile([code], battery_level=0.80, is_standing=True)
                assert v.is_blocked, "HIGH_ENERGY {} should be rejected".format(code)

    def test_high_risk_mid_sequence_truncated(self):
        """High-risk action mid-sequence -> truncated"""
        sc = make_compiler(allow_high_risk=False)
        v = sc.compile([1004, 1030], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 1030 in v.rejected


# === Step 3: Battery gate ===

class TestBatteryGate:
    """Battery gate three-tier system"""

    # --- Tier 1: <=0.10 safe actions only ---

    def test_critical_battery_safe_action_passes(self):
        """battery=0.08, StandUp(1004) -> passes (safe action)"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.08, is_standing=False)
        assert not v.is_blocked
        assert 1004 in v.executable_sequence

    def test_critical_battery_unsafe_action_blocked(self):
        """battery=0.08, Hello(1016) -> rejected (non-safe action)"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.08, is_standing=True)
        assert v.is_blocked
        assert "battery" in v.block_reason.lower() or "insufficient" in v.block_reason.lower()

    def test_critical_battery_dance_blocked(self):
        """battery=0.05, Dance1(1022) -> rejected"""
        sc = make_compiler()
        v = sc.compile([1022], battery_level=0.05, is_standing=True)
        assert v.is_blocked

    # --- Tier 2: <=0.20 no high-energy ---

    def test_low_battery_normal_action_passes(self):
        """battery=0.15, Hello(1016) -> passes (non high-energy)"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.15, is_standing=True)
        assert not v.is_blocked
        assert 1016 in v.executable_sequence

    def test_low_battery_high_energy_blocked(self):
        """battery=0.15, FrontFlip(1030) -> rejected (dual gate: high-energy + low battery)"""
        sc = make_compiler(allow_high_risk=True)  # Allow high-risk to reach battery gate
        v = sc.compile([1030], battery_level=0.15, is_standing=True)
        assert v.is_blocked

    # --- Tier 3: <=0.30 high-energy downgrade ---

    def test_moderate_battery_high_energy_downgraded(self):
        """battery=0.25, FrontFlip(1030) -> downgraded to Dance2(1023)"""
        sc = make_compiler(allow_high_risk=True)  # Pass high-risk gate first
        v = sc.compile([1030], battery_level=0.25, is_standing=True)
        assert not v.is_blocked
        assert 1023 in v.executable_sequence  # downgrade_target
        assert 1030 not in v.executable_sequence
        assert any("downgraded" in w.lower() or "Downgraded" in w for w in v.warnings)

    def test_custom_downgrade_target(self):
        """Custom downgrade target: 1022(Dance1)"""
        sc = make_compiler(allow_high_risk=True, downgrade_target=1022)
        v = sc.compile([1030], battery_level=0.25, is_standing=True)
        assert 1022 in v.executable_sequence

    # --- Sufficient battery ---

    def test_full_battery_all_passes(self):
        """battery=0.80 -> all normal actions pass"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]

    # --- Boundary values ---

    def test_boundary_0_10_safe_passes(self):
        """battery=0.10, safe action passes"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.10, is_standing=False)
        assert not v.is_blocked

    def test_boundary_0_10_unsafe_blocked(self):
        """battery=0.10, non-safe action rejected (<=0.10)"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.10, is_standing=True)
        assert v.is_blocked

    def test_boundary_0_11_unsafe_passes(self):
        """battery=0.11, non-safe non-high-energy action passes (>0.10 and <=0.20 tier only blocks high-energy)"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.11, is_standing=True)
        assert not v.is_blocked


# === Step 4: Standing prerequisite ===

class TestStandingPrerequisite:
    """Standing prerequisite auto-insertion"""

    def test_standing_action_auto_prepend(self):
        """Not standing + requires-standing action -> auto-prepend StandUp(1004)"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=False)
        assert not v.is_blocked
        assert v.executable_sequence[0] == 1004  # auto StandUp
        assert v.executable_sequence[1] == 1016
        assert 1004 in v.auto_prepend

    def test_already_standing_no_prepend(self):
        """Already standing -> no prepend"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=True)
        assert v.executable_sequence == [1016]
        assert not v.auto_prepend

    def test_non_standing_action_no_prepend(self):
        """Action not requiring standing -> no prepend (e.g., RecoveryStand)"""
        sc = make_compiler()
        assert 1006 not in REQUIRE_STANDING  # RecoveryStand does not require standing
        v = sc.compile([1006], battery_level=0.80, is_standing=False)
        assert v.executable_sequence == [1006]
        assert not v.auto_prepend

    def test_sequence_single_prepend(self):
        """Multiple requires-standing actions in sequence -> only one StandUp prepended"""
        sc = make_compiler()
        v = sc.compile([1016, 1017], battery_level=0.80, is_standing=False)
        assert v.executable_sequence.count(1004) == 1
        assert v.executable_sequence == [1004, 1016, 1017]

    def test_standdown_requires_standing_prepend(self):
        """StandDown(1005) requires standing -> auto-prepend StandUp when not standing"""
        sc = make_compiler()
        assert 1005 in REQUIRE_STANDING
        v = sc.compile([1005], battery_level=0.80, is_standing=False)
        assert v.executable_sequence == [1004, 1005]
        assert 1004 in v.auto_prepend

    def test_standdown_already_standing_no_prepend(self):
        """StandDown(1005) already standing -> no prepend"""
        sc = make_compiler()
        v = sc.compile([1005], battery_level=0.80, is_standing=True)
        assert v.executable_sequence == [1005]
        assert not v.auto_prepend


# === Step 5: Sequence compilation ===

class TestSequenceCompilation:
    """Sequence compilation (truncation/downgrade/merge)"""

    def test_normal_sequence(self):
        """Normal sequence all pass"""
        sc = make_compiler()
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]

    def test_first_rejected_blocks_all(self):
        """First action rejected -> entire sequence rejected"""
        sc = make_compiler()
        v = sc.compile([9999, 1004], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert v.executable_sequence == []

    def test_mid_rejected_truncates(self):
        """Mid-sequence rejection -> truncated, keeping preceding actions"""
        sc = make_compiler()
        v = sc.compile([1004, 9999, 1005], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]

    def test_mid_battery_rejected_truncates(self):
        """Mid-sequence battery rejection -> truncated"""
        sc = make_compiler()
        # battery=0.08 -> 1004(safe) passes, 1016(non-safe) rejected
        v = sc.compile([1004, 1016], battery_level=0.08, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 1016 in v.rejected

    def test_mid_downgrade_replaces(self):
        """Mid-sequence downgrade -> replaced and continues"""
        sc = make_compiler(allow_high_risk=True)
        # battery=0.25 -> 1004 passes, 1030(high-energy) downgraded to 1023
        v = sc.compile([1004, 1030], battery_level=0.25, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1023]  # 1030 downgraded to 1023

    def test_mid_posture_transition_inserts_standup_for_following_action(self):
        """Mid-sequence posture transition: Sit followed by Hello -> insert StandUp in between"""
        sc = make_compiler()
        # Initially standing:
        # 1009(Sit) changes to non-standing after execution, subsequent 1016(Hello) requires_standing
        # Should insert 1004 in between, not just prepend once at the beginning.
        v = sc.compile([1009, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1009, 1004, 1016]

    def test_mid_posture_transition_after_standdown_inserts_standup(self):
        """Mid-sequence posture transition: StandDown followed by Heart -> insert StandUp in between"""
        sc = make_compiler()
        # 1005(StandDown) changes to non-standing after execution, 1036(Heart) requires_standing
        v = sc.compile([1005, 1036], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1005, 1004, 1036]

    def test_empty_actions(self):
        """Empty action list -> empty result, not blocked"""
        sc = make_compiler()
        v = sc.compile([], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == []

    def test_sequence_with_prepend_and_downgrade(self):
        """Combined scenario: standing prepend + downgrade"""
        sc = make_compiler(allow_high_risk=True)
        # Not standing, battery=0.25, [1030(high-energy requires standing)] ->
        # Downgraded to 1023(Dance2, also requires standing) -> prepend StandUp
        v = sc.compile([1030], battery_level=0.25, is_standing=False)
        assert not v.is_blocked
        assert v.executable_sequence[0] == 1004  # auto StandUp
        assert 1023 in v.executable_sequence      # downgraded


# === SafetyVerdict data integrity ===

class TestVerdictIntegrity:
    """SafetyVerdict field correctness"""

    def test_blocked_has_reason_and_override(self):
        """Blocked -> has block_reason + response_override"""
        sc = make_compiler()
        v = sc.compile([9999], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert len(v.block_reason) > 0
        assert v.response_override is not None
        assert len(v.response_override) > 0

    def test_passed_has_no_override(self):
        """Passed -> response_override=None"""
        sc = make_compiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.response_override is None

    def test_warnings_accumulate(self):
        """Multiple trigger conditions -> warnings accumulate"""
        sc = make_compiler()
        v = sc.compile([1016], battery_level=-0.5, is_standing=False)
        # battery<0 correction + battery<=0.10 rejection
        assert len(v.warnings) >= 1

    def test_default_verdict_clean(self):
        """Default SafetyVerdict has all fields clean"""
        v = SafetyVerdict()
        assert v.executable_sequence == []
        assert v.auto_prepend == []
        assert v.rejected == []
        assert v.warnings == []
        assert not v.is_blocked
        assert v.block_reason == ""
        assert v.response_override is None
