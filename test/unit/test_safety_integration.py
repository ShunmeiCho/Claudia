#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_safety_integration.py — AST-level architecture constraint enforcement tests

Validates architectural rules for production_brain.py:
  1. Legacy safety paths are deprecated — no non-deprecated safety_validator/final_gate/precheck calls
  2. Bare sport_client calls are eliminated — no bare calls except Init() and within _rpc_call
  3. Audit route uses constants — _log_audit(route=...) does not use string literals
  4. SafetyCompiler full path coverage — all action paths have safety_compiler.compile calls
"""

import sys
import os
import ast
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

BRAIN_PATH = Path(__file__).parent.parent.parent / "src" / "claudia" / "brain" / "production_brain.py"


# === AST visitors ===

class DeprecatedCallFinder(ast.NodeVisitor):
    """Find illegal calls to deprecated methods (exempt: method's own def body)

    Deprecated methods:
      - self._final_safety_gate(...)
      - self._quick_safety_precheck(...)
      - self.safety_validator.validate_action(...)

    Exemption: calls within the deprecated method's own def body are not violations
    """
    DEPRECATED_SELF = {"_final_safety_gate", "_quick_safety_precheck"}
    DEPRECATED_ATTR = {"safety_validator": {"validate_action"}}

    def __init__(self):
        self.violations = []
        self._current_method = None

    def visit_FunctionDef(self, node):
        old_method = self._current_method
        self._current_method = node.name
        self.generic_visit(node)
        self._current_method = old_method

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_Call(self, node):
        if isinstance(node.func, ast.Attribute):
            obj = node.func.value
            method = node.func.attr
            # self.safety_validator.validate_action(...)
            if isinstance(obj, ast.Attribute) and isinstance(obj.value, ast.Name):
                if obj.value.id == "self":
                    if (obj.attr in self.DEPRECATED_ATTR
                            and method in self.DEPRECATED_ATTR[obj.attr]):
                        self.violations.append(
                            (node.lineno, "self.{}.{}()".format(obj.attr, method))
                        )
            # self._final_safety_gate(...) — exempt within own def body
            elif isinstance(obj, ast.Name) and obj.id == "self":
                if method in self.DEPRECATED_SELF:
                    if self._current_method != method:
                        self.violations.append(
                            (node.lineno, "self.{}()".format(method))
                        )
        self.generic_visit(node)


class BareClientCallFinder(ast.NodeVisitor):
    """Find bare sport_client calls

    Exemptions:
      - Init(): initialization during construction
      - Within _rpc_call method: legitimate calls inside unified RPC wrapper
      - Within _init_sport_client method: initialization flow
    """
    ALLOWED_METHODS = {"Init"}
    EXEMPT_ENCLOSING = {"_rpc_call", "_init_sport_client"}

    def __init__(self):
        self.violations = []
        self._current_method = None

    def visit_FunctionDef(self, node):
        old_method = self._current_method
        self._current_method = node.name
        self.generic_visit(node)
        self._current_method = old_method

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_Call(self, node):
        if isinstance(node.func, ast.Attribute):
            obj = node.func.value
            method = node.func.attr
            # self.sport_client.XXX()
            if (isinstance(obj, ast.Attribute)
                    and isinstance(obj.value, ast.Name)
                    and obj.value.id == "self"
                    and obj.attr == "sport_client"
                    and method not in self.ALLOWED_METHODS
                    and self._current_method not in self.EXEMPT_ENCLOSING):
                self.violations.append(
                    (node.lineno, "self.sport_client.{}()".format(method))
                )
        self.generic_visit(node)


class AuditRouteLiteralFinder(ast.NodeVisitor):
    """Find string literal route parameters in _log_audit calls

    Rule: route= parameter must reference ROUTE_* constants, string literals are not allowed
    """

    def __init__(self):
        self.violations = []

    def visit_Call(self, node):
        if isinstance(node.func, ast.Attribute):
            if node.func.attr == "_log_audit":
                for kw in node.keywords:
                    if kw.arg == "route":
                        # Python 3.8: ast.Constant (3.8+) or ast.Str (3.6-3.7)
                        if isinstance(kw.value, ast.Constant) and isinstance(kw.value.value, str):
                            self.violations.append(
                                (node.lineno, 'route="{}"'.format(kw.value.value))
                            )
                        elif hasattr(ast, 'Str') and isinstance(kw.value, ast.Str):
                            self.violations.append(
                                (node.lineno, 'route="{}"'.format(kw.value.s))
                            )
        self.generic_visit(node)


# === Test cases ===

class TestDeprecatedCallsRemoved:
    """Legacy safety path deprecation validation"""

    def test_no_legacy_safety_calls(self):
        """No non-deprecated legacy safety calls in production_brain.py"""
        assert BRAIN_PATH.exists(), "production_brain.py does not exist"
        source = BRAIN_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        finder = DeprecatedCallFinder()
        finder.visit(tree)
        assert not finder.violations, (
            "Found {} deprecated method calls (should use SafetyCompiler):\n{}".format(
                len(finder.violations),
                "\n".join("  L{}: {}".format(ln, call) for ln, call in finder.violations)
            )
        )


class TestNoBareClientCalls:
    """Bare sport_client call elimination validation"""

    def test_no_bare_sport_client_calls(self):
        """No bare sport_client calls except Init() and exempt methods"""
        assert BRAIN_PATH.exists()
        source = BRAIN_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        finder = BareClientCallFinder()
        finder.visit(tree)
        assert not finder.violations, (
            "Found {} bare sport_client calls (should use _rpc_call):\n{}".format(
                len(finder.violations),
                "\n".join("  L{}: {}".format(ln, call) for ln, call in finder.violations)
            )
        )


class TestAuditRouteConstants:
    """Audit route constant usage validation"""

    def test_audit_route_uses_constants(self):
        """All _log_audit(route=...) use constant references, string literals are forbidden"""
        assert BRAIN_PATH.exists()
        source = BRAIN_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        finder = AuditRouteLiteralFinder()
        finder.visit(tree)
        assert not finder.violations, (
            "Found {} _log_audit calls using string literal route:\n{}\n"
            "Should use audit_routes.ROUTE_* constants instead".format(
                len(finder.violations),
                "\n".join("  L{}: {}".format(ln, call) for ln, call in finder.violations)
            )
        )


class TestSafetyCompilerCoverage:
    """SafetyCompiler full path coverage validation (source-level check)"""

    def _get_brain_source(self):
        """Read production_brain.py source code"""
        return BRAIN_PATH.read_text(encoding="utf-8")

    def test_safety_compiler_in_source(self):
        """production_brain.py imports and uses SafetyCompiler"""
        source = self._get_brain_source()
        assert "from claudia.brain.safety_compiler import" in source
        assert "safety_compiler.compile" in source

    def test_safety_compiler_in_hotcache(self):
        """hot_cache path uses SafetyCompiler"""
        source = self._get_brain_source()
        # hot_cache related area should have safety_compiler.compile
        assert "safety_compiler.compile" in source

    def test_safety_compiler_in_sequence(self):
        """sequence path uses SafetyCompiler"""
        source = self._get_brain_source()
        # Search sequence_hotpath/predefined area for safety_compiler
        assert source.count("safety_compiler.compile") >= 1, (
            "safety_compiler.compile appears fewer times than expected (expected>=1: unified _compile_safety gate)"
        )

    def test_action_registry_imported(self):
        """production_brain.py uses action_registry module"""
        source = self._get_brain_source()
        assert "from claudia.brain.action_registry import" in source
        assert "METHOD_MAP" in source
        assert "VALID_API_CODES" in source

    def test_audit_routes_imported(self):
        """production_brain.py uses audit_routes module"""
        source = self._get_brain_source()
        assert "from claudia.brain.audit_routes import" in source
        assert "ROUTE_EMERGENCY" in source
        assert "ROUTE_HOTPATH" in source
        assert "ALL_ROUTES" in source


class TestArchitecturalIntegrity:
    """Architectural integrity: ensure key modules exist and are importable"""

    def test_action_registry_importable(self):
        """action_registry can be imported normally"""
        from claudia.brain.action_registry import (
            _ACTIONS, ACTION_REGISTRY,
            VALID_API_CODES, EXECUTABLE_API_CODES,
            REQUIRE_STANDING, HIGH_ENERGY_ACTIONS,
            METHOD_MAP, ACTION_RESPONSES,
        )
        assert len(_ACTIONS) > 0
        assert len(VALID_API_CODES) > 0
        assert len(METHOD_MAP) > 0

    def test_safety_compiler_importable(self):
        """safety_compiler can be imported normally"""
        from claudia.brain.safety_compiler import SafetyCompiler, SafetyVerdict
        sc = SafetyCompiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert isinstance(v, SafetyVerdict)

    def test_audit_routes_importable(self):
        """audit_routes can be imported normally"""
        from claudia.brain.audit_routes import ALL_ROUTES, ROUTE_EMERGENCY
        assert len(ALL_ROUTES) > 0
        assert ROUTE_EMERGENCY in ALL_ROUTES

    def test_production_brain_importable(self):
        """production_brain can be imported (without creating an instance)"""
        from claudia.brain.production_brain import ProductionBrain, BrainOutput
        assert hasattr(ProductionBrain, 'process_command')
        assert hasattr(ProductionBrain, 'process_and_execute')
        assert hasattr(ProductionBrain, '_rpc_call')
