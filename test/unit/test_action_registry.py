#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_action_registry.py — action_registry 单元测试

验证:
  - 派生集合与 ActionDef 属性一致
  - VALID_API_CODES 不含参数化动作
  - EXECUTABLE_API_CODES 包含 safe_default_params 动作
  - METHOD_MAP 只含 enabled 动作
  - generate_modelfile_action_block() 输出与 VALID_API_CODES 一致
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.action_registry import (
    _ACTIONS, ACTION_REGISTRY,
    VALID_API_CODES, EXECUTABLE_API_CODES, SAFE_DEFAULT_PARAMS,
    REQUIRE_STANDING, HIGH_ENERGY_ACTIONS, ACTION_MODEL_API_CODES,
    METHOD_MAP, ACTION_RESPONSES,
    get_response_for_action, get_response_for_sequence,
    generate_modelfile_action_block,
)


class TestDerivedSets:
    """派生集合一致性"""

    def test_valid_api_codes_no_params(self):
        """VALID_API_CODES 不包含 has_params 动作"""
        for a in _ACTIONS:
            if a.has_params:
                assert a.api_code not in VALID_API_CODES, (
                    "has_params=True 的 {} 不应在 VALID_API_CODES 中".format(a.api_code)
                )

    def test_valid_api_codes_only_enabled(self):
        """VALID_API_CODES 只包含 enabled 动作"""
        for a in _ACTIONS:
            if not a.enabled:
                assert a.api_code not in VALID_API_CODES, (
                    "enabled=False 的 {} 不应在 VALID_API_CODES 中".format(a.api_code)
                )

    def test_valid_api_codes_completeness(self):
        """所有无参数+已启用的动作都在 VALID_API_CODES 中"""
        expected = {a.api_code for a in _ACTIONS if a.enabled and not a.has_params}
        assert VALID_API_CODES == expected

    def test_executable_includes_safe_default_params(self):
        """EXECUTABLE_API_CODES 包含有 safe_default_params 的参数化动作"""
        for a in _ACTIONS:
            if a.enabled and a.has_params and a.safe_default_params is not None:
                assert a.api_code in EXECUTABLE_API_CODES, (
                    "safe_default_params 的 {} 应在 EXECUTABLE_API_CODES 中".format(a.api_code)
                )

    def test_executable_excludes_params_without_defaults(self):
        """EXECUTABLE_API_CODES 不包含无 safe_default_params 的参数化动作"""
        for a in _ACTIONS:
            if a.enabled and a.has_params and a.safe_default_params is None:
                assert a.api_code not in EXECUTABLE_API_CODES, (
                    "无 safe_default_params 的 {} 不应在 EXECUTABLE_API_CODES 中".format(a.api_code)
                )

    def test_executable_superset_of_valid(self):
        """EXECUTABLE_API_CODES 是 VALID_API_CODES 的超集"""
        assert VALID_API_CODES.issubset(EXECUTABLE_API_CODES)

    def test_pose_1028_in_executable_not_valid(self):
        """Pose(1028) 在 EXECUTABLE 中但不在 VALID 中（双层白名单核心验证）"""
        assert 1028 in EXECUTABLE_API_CODES
        assert 1028 not in VALID_API_CODES

    def test_safe_default_params_dict(self):
        """SAFE_DEFAULT_PARAMS 只含有 safe_default_params 的已启用参数化动作"""
        for api_code, params in SAFE_DEFAULT_PARAMS.items():
            a = ACTION_REGISTRY[api_code]
            assert a.enabled
            assert a.has_params
            assert a.safe_default_params is not None
            assert params == a.safe_default_params

    def test_require_standing_matches(self):
        """REQUIRE_STANDING 与 requires_standing=True 一致"""
        expected = {a.api_code for a in _ACTIONS if a.requires_standing}
        assert REQUIRE_STANDING == expected

    def test_sit_1009_explicitly_requires_standing(self):
        """策略约束: Sit(1009) 明确要求站立前置"""
        assert 1009 in REQUIRE_STANDING, (
            "Sit(1009) 应在 REQUIRE_STANDING 中，"
            "用于不可信状态下触发 SafetyCompiler 自动前插 StandUp"
        )

    def test_high_energy_matches(self):
        """HIGH_ENERGY_ACTIONS 与 risk_level='high' 一致"""
        expected = {a.api_code for a in _ACTIONS if a.risk_level == "high"}
        assert HIGH_ENERGY_ACTIONS == expected

    def test_method_map_only_enabled(self):
        """METHOD_MAP 只包含 enabled 动作"""
        for a in _ACTIONS:
            if a.enabled:
                assert a.api_code in METHOD_MAP
                assert METHOD_MAP[a.api_code] == a.method
            else:
                assert a.api_code not in METHOD_MAP

    def test_action_responses_only_enabled(self):
        """ACTION_RESPONSES 只包含 enabled 动作"""
        for a in _ACTIONS:
            if a.enabled:
                assert a.api_code in ACTION_RESPONSES
            else:
                assert a.api_code not in ACTION_RESPONSES


class TestResponseHelpers:
    """响应辅助函数"""

    def test_get_response_known(self):
        assert get_response_for_action(1004) == "立ちます"

    def test_get_response_unknown(self):
        assert get_response_for_action(9999) == "はい、わかりました"

    def test_get_response_for_sequence(self):
        result = get_response_for_sequence([1004, 1016])
        assert "立ちます" in result
        assert "挨拶します" in result


class TestModelfileGeneration:
    """Modelfile 生成"""

    def test_generate_nonempty(self):
        block = generate_modelfile_action_block()
        assert len(block) > 0

    def test_generate_contains_all_valid(self):
        """生成的动作列表包含所有 VALID_API_CODES"""
        block = generate_modelfile_action_block()
        for code in VALID_API_CODES:
            assert str(code) in block, (
                "VALID_API_CODES 中的 {} 未出现在生成的 Modelfile 动作列表中".format(code)
            )

    def test_generate_no_params_actions(self):
        """生成的动作列表不包含参数化动作"""
        block = generate_modelfile_action_block()
        for a in _ACTIONS:
            if a.has_params:
                # 参数化动作的 api_code 不应作为独立条目出现
                entry = "{}=".format(a.api_code)
                assert entry not in block, (
                    "参数化动作 {} 不应出现在 Modelfile 生成中".format(a.api_code)
                )


class TestRegistryIntegrity:
    """注册表完整性"""

    def test_no_duplicate_api_codes(self):
        codes = [a.api_code for a in _ACTIONS]
        assert len(codes) == len(set(codes)), "存在重复的 api_code"

    def test_no_duplicate_methods(self):
        methods = [a.method for a in _ACTIONS]
        assert len(methods) == len(set(methods)), "存在重复的 method"

    def test_all_enabled_have_method_name(self):
        for a in _ACTIONS:
            if a.enabled:
                assert a.method, "enabled 动作 {} 缺少 method".format(a.api_code)

    def test_all_have_name_ja(self):
        for a in _ACTIONS:
            assert a.name_ja, "动作 {} 缺少 name_ja".format(a.api_code)

    def test_disabled_not_in_executable(self):
        """禁用动作不在任何可执行白名单中"""
        for a in _ACTIONS:
            if not a.enabled:
                assert a.api_code not in VALID_API_CODES
                assert a.api_code not in EXECUTABLE_API_CODES
                assert a.api_code not in METHOD_MAP


class TestActionModelApiCodes:
    """PR2: ACTION_MODEL_API_CODES 一致性"""

    def test_subset_of_valid(self):
        """ACTION_MODEL_API_CODES 是 VALID_API_CODES 的子集"""
        assert ACTION_MODEL_API_CODES <= VALID_API_CODES

    def test_no_high_risk(self):
        """ACTION_MODEL_API_CODES 不含高风险动作"""
        overlap = ACTION_MODEL_API_CODES & HIGH_ENERGY_ACTIONS
        assert not overlap, (
            "ACTION_MODEL_API_CODES 不应包含高风险动作: {}".format(overlap))

    def test_excludes_exactly_high_risk(self):
        """VALID - ACTION_MODEL = 仅高风险动作"""
        diff = VALID_API_CODES - ACTION_MODEL_API_CODES
        expected_high = frozenset(
            a.api_code for a in _ACTIONS
            if a.enabled and not a.has_params and a.risk_level == "high"
        )
        assert diff == expected_high, (
            "VALID - ACTION_MODEL 应恰好等于高风险动作集合，"
            "差异: {}".format(diff ^ expected_high))

    def test_generate_no_high_risk(self):
        """generate_modelfile_action_block(include_high_risk=False) 不含高风险码"""
        block = generate_modelfile_action_block(include_high_risk=False)
        for code in HIGH_ENERGY_ACTIONS:
            # 确保高风险码不作为独立条目出现
            # 注意: 1030 不应出现为 "1030=" 的形式
            assert "{}=".format(code) not in block, (
                "高风险动作 {} 不应出现在 Action 模型列表中".format(code))

    def test_generate_includes_safe_actions(self):
        """generate_modelfile_action_block(include_high_risk=False) 包含安全动作"""
        block = generate_modelfile_action_block(include_high_risk=False)
        for code in ACTION_MODEL_API_CODES:
            assert str(code) in block, (
                "安全动作 {} 未出现在 Action 模型列表中".format(code))

    def test_modelfile_action_list_consistent(self):
        """ClaudiaAction modelfile 动作列表与 ACTION_MODEL_API_CODES 一致"""
        import re
        modelfile_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'models',
            'ClaudiaAction_v1.0')
        with open(modelfile_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 只在 SYSTEM 区域内匹配 "NNNN=" 格式的动作定义
        # 避免误匹配 PARAMETER 值（如 num_ctx=1024）
        codes_in_file = set()
        for match in re.finditer(r'(\d{4})=', content):
            code = int(match.group(1))
            if 1000 <= code <= 1099:
                codes_in_file.add(code)

        # modelfile 中的码应是 ACTION_MODEL_API_CODES 的子集
        unexpected = codes_in_file - ACTION_MODEL_API_CODES
        assert not unexpected, (
            "Modelfile 包含不在 ACTION_MODEL_API_CODES 中的码: {}。"
            "如果这些是高风险动作，请从 modelfile 中移除。".format(unexpected))

        # ACTION_MODEL_API_CODES 中的码应出现在 modelfile 中
        missing = ACTION_MODEL_API_CODES - codes_in_file
        assert not missing, (
            "ACTION_MODEL_API_CODES 中的码 {} 未出现在 modelfile 中。"
            "请更新 modelfile 或 registry。".format(missing))

    def test_7b_modelfile_action_list_consistent(self):
        """ClaudiaIntelligent 7B modelfile 动作列表与 VALID_API_CODES 一致

        7B 模型可输出任何 VALID_API_CODES 中的码（含高风险）。
        Modelfile 中不应包含参数化动作码（会被运行时过滤，浪费推理 token）。
        """
        import re
        modelfile_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'models',
            'ClaudiaIntelligent_7B_v2.0')
        with open(modelfile_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 提取 SYSTEM 块中 "NNNN=" 格式的动作定义（排除 PARAMETER 行的值）
        codes_in_file = set()
        for match in re.finditer(r'(\d{4})=', content):
            code = int(match.group(1))
            if 1000 <= code <= 1099:
                codes_in_file.add(code)

        # Modelfile 中的码应是 VALID_API_CODES 的子集（不应含参数化动作）
        unexpected = codes_in_file - VALID_API_CODES
        assert not unexpected, (
            "7B Modelfile 包含不在 VALID_API_CODES 中的码: {}。"
            "参数化动作（1008/1015/1028 等）会被运行时过滤，不应出现在 modelfile 中。"
            .format(unexpected))

        # VALID_API_CODES 中的码应出现在 Modelfile 中（确保模型能学到所有合法动作）
        missing = VALID_API_CODES - codes_in_file
        assert not missing, (
            "VALID_API_CODES 中的码 {} 未出现在 7B modelfile 中。"
            "请更新 modelfile 使模型能学到这些动作。".format(missing))
