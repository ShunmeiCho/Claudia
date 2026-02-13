"""Hardware tests require unitree_sdk2py (native DDS libraries).

Skip collection entirely when the SDK is not importable (e.g., CI, non-robot hosts).
"""
import pytest

collect_ignore_glob = []

try:
    import unitree_sdk2py  # noqa: F401
except (ImportError, OSError):
    collect_ignore_glob = ["test_*.py"]
