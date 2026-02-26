"""Integration tests configuration.

These tests import ProductionBrain which may trigger CycloneDDS C library.
On Jetson (16GB unified memory), high swap usage can cause bad_alloc/OOM.
Pre-mock heavy native deps to reduce memory footprint.
"""
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock

# Ensure src/ is in sys.path for direct execution and subprocess invocations
_SRC_DIR = str(Path(__file__).parent.parent.parent / "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

# Pre-mock heavy native deps before any claudia import
# CycloneDDS: C library that allocates network buffers → bad_alloc under memory pressure
# ollama: avoids loading LLM runtime
# unitree_sdk2py: depends on cyclonedds
_HEAVY_MODS = [
    'ollama',
    # CycloneDDS: C layer bad_alloc
    'cyclonedds', 'cyclonedds.core', 'cyclonedds.domain',
    'cyclonedds.idl', 'cyclonedds.pub', 'cyclonedds.sub',
    'cyclonedds._clayer',
    # ROS2: rclpy node initialization allocates large amounts of memory
    'rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.qos',
    'rclpy.callback_groups', 'rclpy.parameter',
    # unitree SDK: depends on cyclonedds
    'unitree_sdk2py', 'unitree_sdk2py.go2',
    'unitree_sdk2py.go2.sport', 'unitree_sdk2py.go2.sport.sport_client',
    'unitree_sdk2py.idl', 'unitree_sdk2py.rpc',
]
for mod in _HEAVY_MODS:
    sys.modules.setdefault(mod, MagicMock())

collect_ignore_glob = []

try:
    from claudia.brain.production_brain import ProductionBrain  # noqa: F401
except (ImportError, ModuleNotFoundError, OSError):
    # OSError: catches bad_alloc propagated as OSError on some builds
    collect_ignore_glob = ["test_*.py"]
