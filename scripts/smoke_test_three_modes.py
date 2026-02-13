#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
smoke_test_three_modes.py — PR2 三模式冒烟测试

在 legacy/dual/shadow 三种路由模式下运行一组测试命令，
验证 hot_cache 映射、LLM 路由、安全编译器等关键路径。

修复:
  - LLM 超时判定为 FAIL（不是 OK）
  - 使用 process_and_execute()（原子入口，不触发弃用警告）
  - Shadow 对比数据从 audit log 读取（不是 BrainOutput）
  - 踊って 归类为 hot_cache（不是 LLM）

用法:
  python3 scripts/smoke_test_three_modes.py
"""

import os
import sys
import asyncio
import time
import json
import glob as glob_mod
from pathlib import Path

# 确保项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# 禁用 ROS2 状态监控（避免 Jetson OOM / DDS 初始化）
import claudia.brain.production_brain as pb_mod
pb_mod.STATE_MONITOR_AVAILABLE = False

# LLM 超时判定阈值（超过此值视为超时 FAIL）
LLM_TIMEOUT_THRESHOLD_MS = 28000


def create_brain(mode):
    """创建指定路由模式的 brain 实例"""
    os.environ["BRAIN_ROUTER_MODE"] = mode
    from claudia.brain.production_brain import ProductionBrain
    brain = ProductionBrain(use_real_hardware=False)

    # Finding #4 修复: 使用具名属性（非 MagicMock），避免日志 source=<MagicMock...>
    from types import SimpleNamespace
    mock_state = SimpleNamespace(
        battery_level=0.80,
        is_standing=True,
        is_moving=False,
        temperature=40.0,
        timestamp=0.0,
        source="smoke_test",       # 明确标识数据来源
        confidence=1.0,
        current_gait="unknown",
        network_status="unknown",
        sdk_connection=False,
    )
    mock_monitor = SimpleNamespace(
        get_current_state=lambda: mock_state,
        is_ros_initialized=True,
    )
    brain.state_monitor = mock_monitor
    return brain


def run_async(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


# === 测试用例定义 ===
# (command, expected_api_code, description, is_hotcache)
# expected_api_code: 精确期望值（hot_cache），或 "any_action" / "conversational" / "any"（LLM）
TEST_CASES = [
    # --- hot_cache 鞠躬词组 (全部 → 1029) ---
    ("ちんちん", 1029, "ちんちん → Scrape", True),
    ("チンチン", 1029, "チンチン(片假名) → Scrape", True),
    ("拜年", 1029, "拜年(中文) → Scrape", True),
    ("お辞儀", 1029, "お辞儀 → Scrape", True),
    ("礼", 1029, "礼 → Scrape", True),
    ("ちんちん！", 1029, "ちんちん！(标点) → Scrape", True),

    # --- hot_cache 基础命令 ---
    ("座って", 1009, "座って → Sit", True),
    ("立って", 1004, "立って → StandUp", True),
    ("止まれ", 1003, "止まれ → Stop", True),
    ("かわいい", 1036, "かわいい → Heart", True),
    ("踊って", 1022, "踊って → Dance1", True),  # 踊って 在 hot_cache，不是 LLM

    # --- LLM 路由命令（依赖 Ollama 推理）---
    # expected: "any_action"=期望有 api_code, "conversational"=期望无动作, "any"=宽松
    ("座りなさい", "any_action", "座りなさい → LLM (expect Sit)", False),
    ("ストレッチして", "any_action", "ストレッチして → LLM (expect Stretch)", False),
    ("元気を出して", "any", "元気を出して → LLM (expect action/conv)", False),
    ("今日の天気は？", "conversational", "今日の天気は？ → LLM (expect conv)", False),
]


def format_result(output, elapsed_ms):
    """格式化单条结果"""
    api = output.api_code
    seq = getattr(output, 'sequence', None)
    route = getattr(output, 'reasoning', '?')
    resp = output.response[:30] if output.response else ""
    exec_status = getattr(output, 'execution_status', None)
    if seq:
        return "seq={} route={} exec={} resp='{}' {:.0f}ms".format(
            seq, route, exec_status, resp, elapsed_ms)
    return "api={} route={} exec={} resp='{}' {:.0f}ms".format(
        api, route, exec_status, resp, elapsed_ms)


def judge_llm_result(output, expected, elapsed_ms):
    """判定 LLM 路由命令结果

    返回 (passed, status_label)
    """
    # 超时检测: 超过阈值 = FAIL
    if elapsed_ms > LLM_TIMEOUT_THRESHOLD_MS:
        return False, "TIMEOUT"

    # 崩溃检测: 响应为空
    if not output.response:
        return False, "EMPTY"

    # 语义判定
    if expected == "any_action":
        passed = output.api_code is not None or getattr(output, 'sequence', None)
        return passed, "PASS" if passed else "FAIL"
    elif expected == "conversational":
        passed = output.api_code is None
        return passed, "PASS" if passed else "WARN"
    else:  # "any"
        return True, "PASS"


def read_recent_audit_entries(n=50, after_ts=None):
    """从 audit log 读取最近 n 条 shadow_comparison 记录

    Finding #3 修复: after_ts 为 ISO 时间戳字符串，只读取该时间之后的条目，
    隔离本次运行的数据，避免历史 session 污染。
    """
    audit_dir = Path("logs/audit")
    if not audit_dir.exists():
        return []
    entries = []
    for f in sorted(audit_dir.glob("audit_*.jsonl"), reverse=True):
        with f.open('r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                    if not entry.get("shadow_comparison"):
                        continue
                    # Finding #3: 时间窗过滤
                    if after_ts and entry.get("timestamp", "") < after_ts:
                        continue
                    entries.append(entry)
                except json.JSONDecodeError:
                    pass
        if len(entries) >= n:
            break
    return entries[-n:]  # 最后 n 条


def run_smoke_test():
    modes = ["legacy", "dual", "shadow"]
    results = {}  # mode -> [(cmd, pass, detail)]

    # Finding #3: 记录本次运行的起始时间，用于隔离 audit 条目
    from datetime import datetime
    run_start_ts = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

    for mode in modes:
        print("\n" + "=" * 70)
        print("  MODE: {}".format(mode.upper()))
        print("=" * 70)

        brain = create_brain(mode)
        mode_results = []

        for cmd, expected, desc, is_hotcache in TEST_CASES:
            t0 = time.monotonic()
            try:
                # 使用原子入口 process_and_execute()（不触发弃用警告）
                output = run_async(brain.process_and_execute(cmd))
                elapsed = (time.monotonic() - t0) * 1000
                detail = format_result(output, elapsed)

                if is_hotcache:
                    # hot_cache 命令: 严格检查 api_code
                    passed = (output.api_code == expected)
                    # 特殊情况: SafetyCompiler 可能插入 StandUp
                    if not passed and output.sequence:
                        passed = (expected in output.sequence)
                    status = "PASS" if passed else "FAIL"
                else:
                    # LLM 命令: 根据期望类型和超时判定
                    passed, status = judge_llm_result(output, expected, elapsed)

                mode_results.append((cmd, passed, detail))
                mark = "  [{}]".format(status)
                print("  {} {:25s} → {}".format(mark, desc, detail))

            except Exception as e:
                elapsed = (time.monotonic() - t0) * 1000
                mode_results.append((cmd, False, "ERROR: {}".format(e)))
                print("  [ERROR] {:25s} → {} ({:.0f}ms)".format(desc, e, elapsed))

        results[mode] = mode_results

    # === Shadow 对比: 从 audit log 读取 ===
    print("\n" + "=" * 70)
    print("  SHADOW COMPARISON (from audit log)")
    print("=" * 70)

    shadow_entries = read_recent_audit_entries(50, after_ts=run_start_ts)
    if not shadow_entries:
        print("  (无 shadow_comparison 审计记录)")
    else:
        agreements = sum(1 for e in shadow_entries
                         if e["shadow_comparison"].get("raw_agreement"))
        divergences = sum(1 for e in shadow_entries
                          if e["shadow_comparison"].get("high_risk_divergence"))
        # dual_status 语义: "ok"=正常, "timeout"=超时, "error"=异常, "invalid_output"=非法输出
        timeouts = sum(1 for e in shadow_entries
                       if e["shadow_comparison"].get("dual_status") == "timeout")
        errors = sum(1 for e in shadow_entries
                     if e["shadow_comparison"].get("dual_status") == "error")
        invalids = sum(1 for e in shadow_entries
                       if e["shadow_comparison"].get("dual_status") == "invalid_output")

        print("  条目数: {}".format(len(shadow_entries)))
        print("  一致率: {:.1f}% ({}/{})".format(
            agreements / len(shadow_entries) * 100,
            agreements, len(shadow_entries)))
        print("  高风险分歧: {}".format(divergences))
        print("  Action 超时: {}".format(timeouts))
        print("  Action 错误: {}".format(errors))
        print("  非法输出:   {}".format(invalids))

        # 逐条详情（最后 5 条）
        print("\n  最近 {} 条:".format(min(5, len(shadow_entries))))
        for e in shadow_entries[-5:]:
            sc = e["shadow_comparison"]
            print("    cmd='{}' legacy={} dual={} status={} agree={} diverge={}".format(
                e.get("input_command", "?")[:15],
                sc.get("legacy_api_code"),
                sc.get("dual_api_code"),
                sc.get("dual_status", "?"),
                sc.get("raw_agreement"),
                sc.get("high_risk_divergence"),
            ))

    # === 汇总 ===
    print("\n" + "=" * 70)
    print("  SUMMARY")
    print("=" * 70)
    for mode in modes:
        total = len(results[mode])
        passed = sum(1 for _, p, _ in results[mode] if p)
        failed = total - passed
        status = "ALL PASS" if failed == 0 else "{} FAILED".format(failed)
        print("  {:8s}: {}/{} ({})".format(mode.upper(), passed, total, status))

    # 全部通过?
    all_pass = all(
        p for mode in modes for _, p, _ in results[mode]
    )
    print("\n  Overall: {}".format("ALL PASS" if all_pass else "HAS FAILURES"))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(run_smoke_test())
