#!/usr/bin/env python3
"""
Production Brain Integration Test -- End-to-end Pipeline Verification

Verifies the complete processing flow of ProductionBrain:
  User input -> emergency/cache/LLM -> SafetyCompiler -> execution/simulation
Does not depend on Ollama, ROS2, CycloneDDS, or real hardware (all mocked).
"""

import asyncio
import os
import sys
import unittest
from unittest.mock import MagicMock

# Ensure src/ is in sys.path for direct execution (subprocess via run_tests.py)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

# Mock heavy dependencies before importing ProductionBrain to avoid OOM
# conftest.py performs the same mocking, but conftest may not load first when running directly
for _mod in [
    'ollama',
    'cyclonedds', 'cyclonedds.core', 'cyclonedds.domain',
    'cyclonedds.idl', 'cyclonedds.pub', 'cyclonedds.sub',
    'cyclonedds._clayer',
    'rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.qos',
    'rclpy.callback_groups', 'rclpy.parameter',
    'unitree_sdk2py', 'unitree_sdk2py.go2',
    'unitree_sdk2py.go2.sport', 'unitree_sdk2py.go2.sport.sport_client',
    'unitree_sdk2py.idl', 'unitree_sdk2py.rpc',
]:
    sys.modules.setdefault(_mod, MagicMock())


class TestProductionBrainIntegration(unittest.TestCase):
    """ProductionBrain integration test -- simulation mode"""

    @classmethod
    def setUpClass(cls):
        os.environ["BRAIN_ROUTER_MODE"] = "legacy"
        os.environ["BRAIN_MODEL_7B"] = "test-model-7b"

        from claudia.brain.production_brain import ProductionBrain
        cls._brain_cls = ProductionBrain

    def setUp(self):
        self.brain = self._brain_cls(use_real_hardware=False)

        async def _mock_ensure(model, **kwargs):
            return True
        self.brain._ensure_model_loaded = _mock_ensure

    def tearDown(self):
        # Clean up state_monitor to prevent resource accumulation (especially in non-mock environments)
        if hasattr(self.brain, 'state_monitor') and self.brain.state_monitor is not None:
            if hasattr(self.brain.state_monitor, 'stop_monitoring'):
                try:
                    self.brain.state_monitor.stop_monitoring()
                except Exception:
                    pass
            if hasattr(self.brain.state_monitor, 'stop_polling'):
                try:
                    self.brain.state_monitor.stop_polling()
                except Exception:
                    pass
            self.brain.state_monitor = None

    def _run(self, coro):
        """Synchronously execute an async coroutine"""
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    # ------------------------------------------------------------------
    # Emergency bypass
    # ------------------------------------------------------------------

    def test_emergency_stop(self):
        r = self._run(self.brain.process_and_execute("止まれ"))
        self.assertEqual(r.api_code, 1003)
        self.assertEqual(r.execution_status, "success")

    def test_emergency_with_punctuation(self):
        r = self._run(self.brain.process_and_execute("止まって！"))
        self.assertEqual(r.api_code, 1003)

    def test_emergency_kana(self):
        r = self._run(self.brain.process_and_execute("とまれ"))
        self.assertEqual(r.api_code, 1003)

    def test_emergency_english(self):
        r = self._run(self.brain.process_and_execute("stop"))
        self.assertEqual(r.api_code, 1003)

    # ------------------------------------------------------------------
    # Hot cache
    # ------------------------------------------------------------------

    def test_cache_sit(self):
        r = self._run(self.brain.process_and_execute("座って"))
        # Simulation mode: is_standing=False -> SafetyCompiler inserts StandUp(1004) first
        # Result: api_code=None, sequence=[1004, 1009]
        self.assertIn(1009, r.sequence or [r.api_code])

    def test_cache_stand(self):
        r = self._run(self.brain.process_and_execute("立って"))
        self.assertEqual(r.api_code, 1004)

    def test_cache_kana_alias(self):
        r = self._run(self.brain.process_and_execute("たって"))
        self.assertEqual(r.api_code, 1004)

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def test_safety_blocks_high_risk(self):
        async def mock_llm(model, command, timeout=30, **kw):
            return {"r": "前転します", "a": 1030}
        self.brain._call_ollama_v2 = mock_llm

        r = self._run(self.brain.process_and_execute("前転して"))
        self.assertNotEqual(r.api_code, 1030)


if __name__ == "__main__":
    unittest.main()
