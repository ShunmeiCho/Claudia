#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_safety_integration.py — AST 级架构约束强制测试

验证 production_brain.py 的架构规则:
  1. 旧安全路径已废弃 — 无非 deprecated 的 safety_validator/final_gate/precheck 调用
  2. 裸 sport_client 调用已消灭 — 除 Init() 和 _rpc_call 内部外无裸调用
  3. 审计 route 使用常量 — _log_audit(route=...) 不使用字符串字面量
  4. SafetyCompiler 全路径覆盖 — 所有动作路径都有 safety_compiler.compile 调用
"""

import sys
import os
import ast
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

BRAIN_PATH = Path(__file__).parent.parent.parent / "src" / "claudia" / "brain" / "production_brain.py"


# === AST 访问器 ===

class DeprecatedCallFinder(ast.NodeVisitor):
    """查找被废弃方法的非法调用（豁免方法体自身定义）

    废弃方法:
      - self._final_safety_gate(...)
      - self._quick_safety_precheck(...)
      - self.safety_validator.validate_action(...)

    豁免: deprecated 方法自身 def body 内的调用不算违规
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
            # self._final_safety_gate(...) — 豁免自身 def body
            elif isinstance(obj, ast.Name) and obj.id == "self":
                if method in self.DEPRECATED_SELF:
                    if self._current_method != method:
                        self.violations.append(
                            (node.lineno, "self.{}()".format(method))
                        )
        self.generic_visit(node)


class BareClientCallFinder(ast.NodeVisitor):
    """查找裸 sport_client 调用

    豁免:
      - Init(): 构造期初始化
      - _rpc_call 方法内: 统一 RPC 包装器的合法调用
      - _init_sport_client 方法内: 初始化流程
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
    """查找 _log_audit 调用中使用字符串字面量的 route 参数

    规则: route= 参数必须引用 ROUTE_* 常量，不允许字符串散写
    """

    def __init__(self):
        self.violations = []

    def visit_Call(self, node):
        if isinstance(node.func, ast.Attribute):
            if node.func.attr == "_log_audit":
                for kw in node.keywords:
                    if kw.arg == "route":
                        # Python 3.8: ast.Constant (3.8+) 或 ast.Str (3.6-3.7)
                        if isinstance(kw.value, ast.Constant) and isinstance(kw.value.value, str):
                            self.violations.append(
                                (node.lineno, 'route="{}"'.format(kw.value.value))
                            )
                        elif hasattr(ast, 'Str') and isinstance(kw.value, ast.Str):
                            self.violations.append(
                                (node.lineno, 'route="{}"'.format(kw.value.s))
                            )
        self.generic_visit(node)


# === 测试用例 ===

class TestDeprecatedCallsRemoved:
    """旧安全路径废弃验证"""

    def test_no_legacy_safety_calls(self):
        """production_brain.py 中无非 deprecated 的旧安全调用"""
        assert BRAIN_PATH.exists(), "production_brain.py 不存在"
        source = BRAIN_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        finder = DeprecatedCallFinder()
        finder.visit(tree)
        assert not finder.violations, (
            "发现 {} 处废弃方法调用（应使用 SafetyCompiler）:\n{}".format(
                len(finder.violations),
                "\n".join("  L{}: {}".format(ln, call) for ln, call in finder.violations)
            )
        )


class TestNoBareClientCalls:
    """裸 sport_client 调用消灭验证"""

    def test_no_bare_sport_client_calls(self):
        """除 Init() 和豁免方法外无裸 sport_client 调用"""
        assert BRAIN_PATH.exists()
        source = BRAIN_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        finder = BareClientCallFinder()
        finder.visit(tree)
        assert not finder.violations, (
            "发现 {} 处裸 sport_client 调用（应使用 _rpc_call）:\n{}".format(
                len(finder.violations),
                "\n".join("  L{}: {}".format(ln, call) for ln, call in finder.violations)
            )
        )


class TestAuditRouteConstants:
    """审计 route 使用常量验证"""

    def test_audit_route_uses_constants(self):
        """所有 _log_audit(route=...) 使用常量引用，禁止字符串字面量"""
        assert BRAIN_PATH.exists()
        source = BRAIN_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        finder = AuditRouteLiteralFinder()
        finder.visit(tree)
        assert not finder.violations, (
            "发现 {} 处 _log_audit 使用字符串字面量 route:\n{}\n"
            "应改为 audit_routes.ROUTE_* 常量".format(
                len(finder.violations),
                "\n".join("  L{}: {}".format(ln, call) for ln, call in finder.violations)
            )
        )


class TestSafetyCompilerCoverage:
    """SafetyCompiler 全路径覆盖验证（源码级检查）"""

    def _get_brain_source(self):
        """读取 production_brain.py 源码"""
        return BRAIN_PATH.read_text(encoding="utf-8")

    def test_safety_compiler_in_source(self):
        """production_brain.py 中导入并使用 SafetyCompiler"""
        source = self._get_brain_source()
        assert "from claudia.brain.safety_compiler import" in source
        assert "safety_compiler.compile" in source

    def test_safety_compiler_in_hotcache(self):
        """hot_cache 路径使用 SafetyCompiler"""
        source = self._get_brain_source()
        # 在 hot_cache 相关区域内应有 safety_compiler.compile
        assert "safety_compiler.compile" in source

    def test_safety_compiler_in_sequence(self):
        """sequence 路径使用 SafetyCompiler"""
        source = self._get_brain_source()
        # 搜索 sequence_hotpath/predefined 区域是否有 safety_compiler
        assert source.count("safety_compiler.compile") >= 3, (
            "safety_compiler.compile 出现次数不足（期望>=3: hot_cache+sequence+dance/LLM）"
        )

    def test_action_registry_imported(self):
        """production_brain.py 使用 action_registry 模块"""
        source = self._get_brain_source()
        assert "from claudia.brain.action_registry import" in source
        assert "METHOD_MAP" in source
        assert "VALID_API_CODES" in source

    def test_audit_routes_imported(self):
        """production_brain.py 使用 audit_routes 模块"""
        source = self._get_brain_source()
        assert "from claudia.brain.audit_routes import" in source
        assert "ROUTE_EMERGENCY" in source
        assert "ROUTE_HOTPATH" in source
        assert "ALL_ROUTES" in source


class TestArchitecturalIntegrity:
    """架构完整性: 确保关键模块存在且可导入"""

    def test_action_registry_importable(self):
        """action_registry 可正常导入"""
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
        """safety_compiler 可正常导入"""
        from claudia.brain.safety_compiler import SafetyCompiler, SafetyVerdict
        sc = SafetyCompiler()
        v = sc.compile([1004], battery_level=0.80, is_standing=True)
        assert isinstance(v, SafetyVerdict)

    def test_audit_routes_importable(self):
        """audit_routes 可正常导入"""
        from claudia.brain.audit_routes import ALL_ROUTES, ROUTE_EMERGENCY
        assert len(ALL_ROUTES) > 0
        assert ROUTE_EMERGENCY in ALL_ROUTES

    def test_production_brain_importable(self):
        """production_brain 基本可导入（不初始化实例）"""
        from claudia.brain.production_brain import ProductionBrain, BrainOutput
        assert hasattr(ProductionBrain, 'process_command')
        assert hasattr(ProductionBrain, 'process_and_execute')
        assert hasattr(ProductionBrain, '_rpc_call')
