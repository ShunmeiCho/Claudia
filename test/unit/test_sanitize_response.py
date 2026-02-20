#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sanitize_response.py — _sanitize_response() 单元测试

验证 ProductionBrain._sanitize_response() 的过滤逻辑:
  - 空/None 输入 → 默认日语回复
  - 纯英文/非日语 → 默认日语回复
  - 含无意义词（godee/pong 等）→ 默认日语回复
  - 合法日语输出 → 原样返回
  - 含日语的混合文本 → 原样返回
  - 单词边界匹配不误伤合法子串
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import MagicMock, patch


# 默认回复常量（与 production_brain.py 中一致）
DEFAULT_RESPONSE = "すみません、よく分かりません"


def _make_brain_for_sanitize():
    """创建最小化的 ProductionBrain 实例，仅用于测试 _sanitize_response

    跳过 __init__ 中的重量级初始化（Ollama、DDS、ROS2），
    直接构造一个拥有 logger 和 _sanitize_response 方法的对象。
    """
    from claudia.brain.production_brain import ProductionBrain

    # 绕过 __init__，直接创建实例
    brain = object.__new__(ProductionBrain)
    brain.logger = MagicMock()
    return brain


class TestSanitizeResponseEmpty(unittest.TestCase):
    """空值和边界输入"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_none_input(self):
        """None → 默认回复"""
        result = self.brain._sanitize_response(None)
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_empty_string(self):
        """空字符串 → 默认回复"""
        result = self.brain._sanitize_response("")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_whitespace_only(self):
        """纯空白 → 默认回复"""
        result = self.brain._sanitize_response("   \n\t  ")
        self.assertEqual(result, DEFAULT_RESPONSE)


class TestSanitizeResponseNonJapanese(unittest.TestCase):
    """非日语输出过滤"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_pure_english(self):
        """纯英文 → 默认回复"""
        result = self.brain._sanitize_response("Hello world")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_pure_numbers(self):
        """纯数字 → 默认回复"""
        result = self.brain._sanitize_response("12345")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_pure_punctuation(self):
        """纯标点 → 默认回复"""
        result = self.brain._sanitize_response("!!??...")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_korean_text(self):
        """韩文（非日语）→ 默认回复"""
        result = self.brain._sanitize_response("안녕하세요")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_chinese_only(self):
        """纯汉字（CJK 统一表意文字范围内）→ 原样返回
        注: 汉字在 \\u4e00-\\u9faf 范围，has_kanji = True"""
        result = self.brain._sanitize_response("你好世界")
        self.assertEqual(result, "你好世界")


class TestSanitizeResponseNonsense(unittest.TestCase):
    """无意义词过滤（word-boundary matching）"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_godee_mixed(self):
        """含 godee 的混合文本 → 默认回复"""
        result = self.brain._sanitize_response("今日は godee ですね")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_pong_mixed(self):
        """含 pong 的混合文本 → 默认回复"""
        result = self.brain._sanitize_response("ちんちん pong")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_hi_word_boundary(self):
        """独立 'hi' → 默认回复"""
        result = self.brain._sanitize_response("こんにちは hi")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_hello_word_boundary(self):
        """独立 'hello' → 默认回复"""
        result = self.brain._sanitize_response("元気です hello")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_ok_word_boundary(self):
        """独立 'ok' → 默认回复"""
        result = self.brain._sanitize_response("はい ok")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_yes_word_boundary(self):
        """独立 'yes' → 默认回复"""
        result = self.brain._sanitize_response("分かった yes")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_no_word_boundary(self):
        """独立 'no' → 默认回复"""
        result = self.brain._sanitize_response("いいえ no")
        self.assertEqual(result, DEFAULT_RESPONSE)


class TestSanitizeResponseWordBoundary(unittest.TestCase):
    """单词边界匹配不误伤合法子串"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_tokyo_not_matched_by_ok(self):
        """'tokyo' 含 'ok' 子串但不应被误伤"""
        result = self.brain._sanitize_response("東京はtokyoです")
        self.assertEqual(result, "東京はtokyoです")

    def test_hi_in_longer_word(self):
        """'high' 含 'hi' 子串但不应被误伤"""
        result = self.brain._sanitize_response("テンションがhighです")
        self.assertEqual(result, "テンションがhighです")

    def test_pong_in_longer_word(self):
        """'pingpong' 含 'pong' 子串但不应被误伤"""
        result = self.brain._sanitize_response("ピンポン pingpong")
        self.assertEqual(result, "ピンポン pingpong")

    def test_nobody_not_matched(self):
        """'nobody' 含 'no' 子串但不应被误伤"""
        result = self.brain._sanitize_response("誰もnobodyいない")
        self.assertEqual(result, "誰もnobodyいない")


class TestSanitizeResponseValidJapanese(unittest.TestCase):
    """合法日语输出原样返回"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_hiragana(self):
        """纯平假名 → 原样返回"""
        result = self.brain._sanitize_response("こんにちは")
        self.assertEqual(result, "こんにちは")

    def test_katakana(self):
        """纯片假名 → 原样返回"""
        result = self.brain._sanitize_response("ロボット")
        self.assertEqual(result, "ロボット")

    def test_mixed_japanese(self):
        """混合日语 → 原样返回"""
        result = self.brain._sanitize_response("お散歩しましょう！ワンワン！")
        self.assertEqual(result, "お散歩しましょう！ワンワン！")

    def test_japanese_with_numbers(self):
        """日语+数字 → 原样返回"""
        result = self.brain._sanitize_response("バッテリーは80%です")
        self.assertEqual(result, "バッテリーは80%です")

    def test_whitespace_stripped(self):
        """前後空白は除去される"""
        result = self.brain._sanitize_response("  こんにちは  ")
        self.assertEqual(result, "こんにちは")

    def test_action_response_template(self):
        """动作模板响应 → 原样返回"""
        result = self.brain._sanitize_response("立ちます！")
        self.assertEqual(result, "立ちます！")

    def test_long_japanese_response(self):
        """长日语响应 → 原样返回"""
        long_resp = "今日はとても良い天気ですね。お散歩に行きましょうか？"
        result = self.brain._sanitize_response(long_resp)
        self.assertEqual(result, long_resp)


if __name__ == "__main__":
    unittest.main()
