"""Integration tests may depend on modules not available in all environments.

Skip collection when required modules are missing.
"""
collect_ignore_glob = []

try:
    from src.claudia.interactive_japanese_commander import JapaneseCommandInterface  # noqa: F401
except (ImportError, ModuleNotFoundError):
    collect_ignore_glob = ["test_*.py"]
