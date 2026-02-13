#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audit_baseline.py — PR2 审计日志基线分析

读取 logs/audit/*.jsonl，输出:
  - 路由分布（emergency/hotpath/7B/action_channel/...）
  - 动作成功率 / unknown 率 / 安全拒绝率
  - P50/P95 延迟（按路由分组）
  - PR2 扩展: 按 router_mode 分组、shadow 一致率

用法:
  python3 scripts/audit_baseline.py              # 默认 logs/audit/
  python3 scripts/audit_baseline.py --dir /path   # 指定目录
  python3 scripts/audit_baseline.py --min-n 100   # 最小样本量
"""

import argparse
import json
import sys
from pathlib import Path
from collections import Counter, defaultdict


def load_entries(audit_dir):
    """加载所有 JSONL 文件"""
    entries = []
    audit_path = Path(audit_dir)
    if not audit_path.exists():
        print("错误: 审计目录不存在: {}".format(audit_dir))
        sys.exit(1)

    for f in sorted(audit_path.glob("audit_*.jsonl")):
        with f.open('r', encoding='utf-8') as fh:
            for line_num, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError:
                    print("警告: {}:{} JSON 解析失败".format(f.name, line_num))
    return entries


def percentile(sorted_values, p):
    """计算百分位数"""
    if not sorted_values:
        return 0.0
    idx = int(len(sorted_values) * p / 100)
    idx = min(idx, len(sorted_values) - 1)
    return sorted_values[idx]


def analyze(entries, min_n):
    """分析审计日志"""
    if len(entries) < min_n:
        print("样本不足: {} 条 (最小要求 {})".format(len(entries), min_n))
        print("继续分析，但结果可能不具统计显著性。\n")

    print("=" * 60)
    print("审计日志基线分析")
    print("总条目: {}".format(len(entries)))
    print("=" * 60)

    # --- 路由分布 ---
    route_counts = Counter(e.get("route", "unknown") for e in entries)
    print("\n--- 路由分布 ---")
    for route, count in route_counts.most_common():
        pct = count / len(entries) * 100
        print("  {:30s} {:5d} ({:5.1f}%)".format(route, count, pct))

    # --- 按路由的延迟统计 ---
    print("\n--- 延迟统计 (ms) ---")
    route_latencies = defaultdict(list)
    for e in entries:
        route = e.get("route", "unknown")
        ms = e.get("elapsed_ms", 0)
        if ms > 0:
            route_latencies[route].append(ms)

    print("  {:30s} {:>8s} {:>8s} {:>8s} {:>6s}".format(
        "Route", "P50", "P95", "P99", "N"))
    for route in sorted(route_latencies.keys()):
        vals = sorted(route_latencies[route])
        p50 = percentile(vals, 50)
        p95 = percentile(vals, 95)
        p99 = percentile(vals, 99)
        print("  {:30s} {:8.1f} {:8.1f} {:8.1f} {:6d}".format(
            route, p50, p95, p99, len(vals)))

    # --- 安全裁决 ---
    verdicts = Counter(e.get("safety_verdict", "unknown") for e in entries)
    print("\n--- 安全裁决 ---")
    for v, count in verdicts.most_common():
        pct = count / len(entries) * 100
        print("  {:30s} {:5d} ({:5.1f}%)".format(v, count, pct))

    # --- 流水线成功率 + 动作率 ---
    pipeline_ok = sum(1 for e in entries if e.get("success", False))
    action_entries = [e for e in entries
                      if e.get("api_code") is not None or e.get("sequence")]
    conversational = len(entries) - len(action_entries)
    rejected = sum(1 for e in entries
                   if str(e.get("safety_verdict", "")).startswith("rejected"))
    print("\n--- 流水线概况 ---")
    print("  总条目:     {}".format(len(entries)))
    print("  流水线成功: {} ({:.1f}%)".format(
        pipeline_ok, pipeline_ok / len(entries) * 100 if entries else 0))
    print("  有动作:     {} ({:.1f}%)".format(
        len(action_entries),
        len(action_entries) / len(entries) * 100 if entries else 0))
    print("  纯对话:     {} ({:.1f}%)".format(
        conversational,
        conversational / len(entries) * 100 if entries else 0))
    print("  安全拒绝:   {} ({:.1f}%)".format(
        rejected, rejected / len(entries) * 100 if entries else 0))

    # --- PR2: router_mode 分组 ---
    mode_counts = Counter(e.get("router_mode", "none") for e in entries)
    has_pr2 = any(e.get("router_mode") is not None for e in entries)
    if has_pr2:
        print("\n--- PR2: 路由模式分布 ---")
        for mode, count in mode_counts.most_common():
            pct = count / len(entries) * 100
            print("  {:30s} {:5d} ({:5.1f}%)".format(mode, count, pct))

    # --- PR2: Shadow 一致率 ---
    shadow_entries = [e for e in entries if e.get("shadow_comparison")]
    if shadow_entries:
        agreements = sum(
            1 for e in shadow_entries
            if e["shadow_comparison"].get("raw_agreement", False))
        high_risk = sum(
            1 for e in shadow_entries
            if e["shadow_comparison"].get("high_risk_divergence", False))
        # dual_status 语义: "ok"=正常, "timeout"=超时, "error"=异常, "invalid_output"=非法输出
        # 兼容旧日志: 无 dual_status 时回退检查 dual_api_code
        status_counts = Counter()
        for e in shadow_entries:
            sc = e["shadow_comparison"]
            ds = sc.get("dual_status")
            if ds is None:
                # 旧日志兼容: 用 dual_api_code 推断
                dac = sc.get("dual_api_code")
                if dac == "timeout":
                    ds = "timeout"
                elif dac == "error":
                    ds = "error"
                else:
                    ds = "ok"
            status_counts[ds] += 1

        print("\n--- PR2: Shadow 对比 ---")
        print("  Shadow 条目: {}".format(len(shadow_entries)))
        print("  原始一致率: {:.1f}%".format(
            agreements / len(shadow_entries) * 100))
        print("  高风险分歧: {}".format(high_risk))
        print("  Dual 状态分布:")
        for st, cnt in status_counts.most_common():
            print("    {:20s} {:5d} ({:5.1f}%)".format(
                st, cnt, cnt / len(shadow_entries) * 100))

        # PR2 Action 通道延迟（仅 dual_status=ok 的样本）
        action_ms = [
            e["shadow_comparison"].get("dual_ms", 0)
            for e in shadow_entries
            if isinstance(e["shadow_comparison"].get("dual_ms"), (int, float))
            and e["shadow_comparison"].get("dual_status", "ok") == "ok"
        ]
        if action_ms:
            action_ms.sort()
            print("\n  Action 通道延迟 (dual_status=ok):")
            print("    P50: {:.1f}ms  P95: {:.1f}ms  N={}".format(
                percentile(action_ms, 50),
                percentile(action_ms, 95),
                len(action_ms)))

    print("\n" + "=" * 60)


def main():
    parser = argparse.ArgumentParser(description="审计日志基线分析")
    parser.add_argument("--dir", default="logs/audit",
                        help="审计日志目录")
    parser.add_argument("--min-n", type=int, default=100,
                        help="统计显著性最小样本量")
    args = parser.parse_args()

    entries = load_entries(args.dir)
    if not entries:
        print("无审计日志条目")
        sys.exit(0)

    analyze(entries, args.min_n)


if __name__ == "__main__":
    main()
