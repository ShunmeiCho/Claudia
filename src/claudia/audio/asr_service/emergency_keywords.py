#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
紧急停止关键词匹配模块

与 production_brain.py EMERGENCY_COMMANDS (lines 611-632) 保持同步。
纯字符串操作，无模型依赖，匹配速度 < 1ms。
宁误停不漏停 — 所有 emergency 事件无论 confidence 都执行。
"""

import re
import unicodedata
from typing import Optional, Set, Tuple

# === 紧急关键词列表 ===
# 与 ProductionBrain.EMERGENCY_COMMANDS 的 key 完全同步
# 包含：漢字、ASR かな変体、カタカナ、英语、中文
EMERGENCY_KEYWORDS_TEXT = [
    # 日语（漢字）
    "止まれ",
    "止めて",
    "止まって",
    "緊急停止",
    "やめて",
    # 日语（ASR かな変体）
    "とまれ",
    "とめて",
    "とまって",
    "きんきゅうていし",
    # カタカナ
    "ストップ",
    # 英语
    "stop",
    "halt",
    "emergency",
    # 中文
    "停止",
    "停下",
]

# 预编译: 归一化后的关键词集合（用于精确匹配）
_NORMALIZED_KEYWORDS: Set[str] = set()

# 预编译: 用于标点剥离的正则
_PUNCTUATION_RE = re.compile(
    r"[\s\u3000\u3001\u3002\uff01\uff0c\uff0e\uff1f"
    r"\uff1a\uff1b!?,.:;\-\u300c\u300d\u300e\u300f"
    r"\u3010\u3011\u2018\u2019\u201c\u201d()（）]"
)


def normalize_for_emergency(text: str) -> str:
    """紧急匹配用归一化: 去空白 → 小写 → NFKC → 去标点

    与 brain 的 _kana_to_kanji 类似但更简化，
    专注于让 ASR 输出的变体能命中关键词列表。
    """
    # strip + lowercase
    text = text.strip().lower()
    # Unicode NFKC 正規化 (半角→全角統一、合成文字分解 等)
    text = unicodedata.normalize("NFKC", text)
    # 标点・空白除去
    text = _PUNCTUATION_RE.sub("", text)
    return text


def _init_normalized_keywords() -> None:
    """启动时预归一化所有关键词"""
    global _NORMALIZED_KEYWORDS
    _NORMALIZED_KEYWORDS = {
        normalize_for_emergency(kw) for kw in EMERGENCY_KEYWORDS_TEXT
    }


# 模块加载时初始化
_init_normalized_keywords()


def match_emergency(text: str) -> Optional[Tuple[str, float]]:
    """紧急关键词匹配

    Args:
        text: ASR 转写文本（原始或已归一化均可）

    Returns:
        (matched_keyword, confidence_estimate) or None
        confidence: 1.0 = 完全一致, 0.7-0.9 = 部分/子串匹配
    """
    normalized = normalize_for_emergency(text)
    if not normalized:
        return None

    # Layer 1: 完全一致匹配 (confidence = 1.0)
    if normalized in _NORMALIZED_KEYWORDS:
        return (normalized, 1.0)

    # Layer 2: 关键词が入力テキストの部分文字列 (confidence = 0.85)
    # 例: "今すぐ止まれ！" → "止まれ" を含む
    for kw in _NORMALIZED_KEYWORDS:
        if kw in normalized:
            return (kw, 0.85)

    # Layer 3: 入力テキストが関键词の部分文字列 (confidence = 0.7)
    # 例: ASR が "とま" だけ拾った → "とまれ" の先頭部分
    # 最低 2 文字以上の一致を要求（1 文字だと誤爆が多い）
    if len(normalized) >= 2:
        for kw in _NORMALIZED_KEYWORDS:
            if normalized in kw:
                return (kw, 0.7)

    return None
