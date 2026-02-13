#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_route_comparison.py — 離線モデル決策一致性比較

Action モデルと 7B モデルの決策一致性を、実行パイプライン外で直接比較する。
SafetyCompiler / 状態スナップショット / 実行チェーンを経由しないため、
純粋なモデル決策品質のみを評価できる。

仕組み:
  1. 各コマンドに対して Action モデルを呼び出す（VRAM にロード）
  2. 同コマンドに対して 7B モデルを呼び出す（VRAM スワップ）
  3. 両者の api_code / sequence を比較
  4. JSONL ログに記録

特徴:
  - 実行チェーン不使用（SafetyCompiler / BrainOutput を経由しない）
  - 単 GPU 前提の順序実行（VRAM 競合回避）
  - 各呼び出しの実測レイテンシ記録（VRAM スワップ込み）
  - hot_cache コマンドは LLM 対比に不要なのでスキップ

用法:
  python3 scripts/offline_route_comparison.py
  python3 scripts/offline_route_comparison.py --timeout 60
  python3 scripts/offline_route_comparison.py --output results/comparison.jsonl
  python3 scripts/offline_route_comparison.py --batch 2
  python3 scripts/offline_route_comparison.py --dry-run
"""

import os
import sys
import json
import time
import argparse
import subprocess
from datetime import datetime

# === コマンドリスト ===
# shadow_observation_commands.py から非 hot_cache コマンドを共有
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from claudia.brain.action_registry import VALID_API_CODES, HIGH_ENERGY_ACTIONS

# shadow_observation_commands.py のコマンドを直接インポート
# ただし hot_cache を除外（LLM 対比の対象外）
try:
    from scripts.shadow_observation_commands import COMMANDS as ALL_COMMANDS, BATCH_RANGES
except ImportError:
    # フォールバック: 直接ファイルパスから読み込み
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _obs_path = os.path.join(_script_dir, 'shadow_observation_commands.py')
    _ns = {}
    with open(_obs_path) as _f:
        _code = compile(_f.read(), _obs_path, 'exec')
        _globals = {"__file__": _obs_path, "__name__": "_shadow_obs"}
        exec(_code, _globals)  # noqa: S102 — loading sibling script, not user input
    ALL_COMMANDS = _globals['COMMANDS']
    BATCH_RANGES = _globals['BATCH_RANGES']

# hot_cache を除外した LLM 対象コマンドのみ
LLM_COMMANDS = [(cmd, cat, exp) for cmd, cat, exp in ALL_COMMANDS if cat != "hot_cache"]

# モデル設定
MODEL_ACTION = os.environ.get("BRAIN_MODEL_ACTION", "claudia-action-v1")
MODEL_7B = os.environ.get("BRAIN_MODEL_7B", "claudia-7b:v2.0")


def call_ollama(model, command, num_predict=100, num_ctx=2048, timeout=60):
    """Ollama に直接 JSON モード呼び出し（Python ollama ライブラリ使用）

    Returns:
        (parsed_dict, latency_ms, status)
        status: "ok" | "timeout" | "error" | "parse_error"
    """
    try:
        import ollama as ollama_lib
    except ImportError:
        return _call_ollama_subprocess(model, command, num_predict, num_ctx, timeout)

    t0 = time.monotonic()
    try:
        response = ollama_lib.chat(
            model=model,
            messages=[{'role': 'user', 'content': command}],
            format='json',
            options={
                'temperature': 0.0,
                'num_predict': num_predict,
                'num_ctx': num_ctx,
                'top_p': 0.9,
            }
        )
        latency = (time.monotonic() - t0) * 1000
        content = response['message']['content']
        parsed = json.loads(content)
        return (parsed, latency, "ok")

    except Exception as e:
        latency = (time.monotonic() - t0) * 1000
        err_type = type(e).__name__
        if "timeout" in err_type.lower() or latency > timeout * 1000:
            return (None, latency, "timeout")
        if "json" in err_type.lower() or "JSON" in str(e):
            return (None, latency, "parse_error")
        return (None, latency, "error")


def _call_ollama_subprocess(model, command, num_predict, num_ctx, timeout):
    """フォールバック: subprocess 経由の Ollama 呼び出し"""
    t0 = time.monotonic()
    try:
        result = subprocess.run(
            ["ollama", "run", model, command],
            capture_output=True, text=True, timeout=timeout,
        )
        latency = (time.monotonic() - t0) * 1000
        if result.returncode != 0:
            return (None, latency, "error")
        parsed = json.loads(result.stdout.strip())
        return (parsed, latency, "ok")
    except subprocess.TimeoutExpired:
        latency = (time.monotonic() - t0) * 1000
        return (None, latency, "timeout")
    except json.JSONDecodeError:
        latency = (time.monotonic() - t0) * 1000
        return (None, latency, "parse_error")
    except Exception:
        latency = (time.monotonic() - t0) * 1000
        return (None, latency, "error")


def extract_decision(parsed, model_type="7b"):
    """モデル出力から決策 (api_code, sequence) を抽出

    Action モデル: {"a": N} or {"s": [...]} or {"a": null}
    7B モデル: {"r": "...", "a": N} or {"r": "...", "s": [...]}
    """
    if parsed is None:
        return (None, None)

    api_code = parsed.get("api_code") or parsed.get("a")
    sequence = parsed.get("sequence") or parsed.get("s")

    # a=null は「会話のみ」
    if api_code is None and sequence is None:
        return (None, None)

    return (api_code, sequence)


def compare_decisions(action_decision, legacy_decision):
    """二つの決策を比較

    Returns:
        dict with comparison results
    """
    action_api, action_seq = action_decision
    legacy_api, legacy_seq = legacy_decision

    # raw_agreement: api_code と sequence の完全一致
    raw_agreement = (action_api == legacy_api and action_seq == legacy_seq)

    # semantic_agreement: 動作の「意味」が一致
    # (api_code が同じ、または sequence 内の動作が同じ)
    action_codes = set()
    if action_api is not None:
        action_codes.add(action_api)
    if action_seq:
        action_codes.update(action_seq)

    legacy_codes = set()
    if legacy_api is not None:
        legacy_codes.add(legacy_api)
    if legacy_seq:
        legacy_codes.update(legacy_seq)

    # 両方 null = 両方会話 = 一致
    if not action_codes and not legacy_codes:
        semantic_agreement = True
    elif action_codes == legacy_codes:
        semantic_agreement = True
    else:
        semantic_agreement = False

    # high_risk_divergence: 一方が高リスク動作、他方がそうでない
    action_high = action_codes & HIGH_ENERGY_ACTIONS
    legacy_high = legacy_codes & HIGH_ENERGY_ACTIONS
    high_risk_divergence = (action_high != legacy_high)

    # null 一致: 両方 null（会話と判断）
    both_null = (action_api is None and action_seq is None
                 and legacy_api is None and legacy_seq is None)

    return {
        "raw_agreement": raw_agreement,
        "semantic_agreement": semantic_agreement,
        "high_risk_divergence": high_risk_divergence,
        "both_null": both_null,
        "action_api": action_api,
        "action_seq": action_seq,
        "legacy_api": legacy_api,
        "legacy_seq": legacy_seq,
    }


def warmup_models(timeout=60):
    """モデルを VRAM にプリロード（Action → 7B の順で最後の 7B が VRAM に残る）"""
    models = [
        (MODEL_ACTION, 1024, "Action"),
        (MODEL_7B, 2048, "7B"),
    ]
    for model_name, num_ctx, label in models:
        t0 = time.monotonic()
        print("  Warmup: {} ({})...".format(label, model_name), end="", flush=True)
        try:
            result = subprocess.run(
                ["ollama", "run", model_name, '{"a":null}'],
                capture_output=True, text=True, timeout=timeout,
            )
            elapsed = (time.monotonic() - t0) * 1000
            if result.returncode == 0:
                print(" OK ({:.0f}ms)".format(elapsed))
            else:
                print(" WARN: rc={} ({:.0f}ms)".format(result.returncode, elapsed))
        except subprocess.TimeoutExpired:
            elapsed = (time.monotonic() - t0) * 1000
            print(" TIMEOUT ({:.0f}ms)".format(elapsed))
        except Exception as e:
            elapsed = (time.monotonic() - t0) * 1000
            print(" ERROR: {} ({:.0f}ms)".format(e, elapsed))


def run_comparison(commands, output_path, timeout=60, delay=2.0):
    """全コマンドの離線比較を実行

    順序: 各コマンドごとに Action → 7B（7B を最後に呼んで VRAM に残す）
    """
    results = []
    n = len(commands)

    for i, (cmd, cat, expected) in enumerate(commands):
        idx = i + 1
        # 空コマンドをスキップ（edge case のうち空文字は Ollama が拒否する可能性）
        if not cmd.strip():
            print("  [{}/{}] [{}] (empty) → SKIP".format(idx, n, cat[0].upper()))
            results.append({
                "index": idx,
                "command": cmd,
                "category": cat,
                "expected": expected,
                "action_status": "skipped",
                "legacy_status": "skipped",
                "comparison": {"raw_agreement": True, "both_null": True,
                               "semantic_agreement": True, "high_risk_divergence": False},
            })
            continue

        # Step 1: Action モデル（VRAM swap in）
        action_raw, action_ms, action_status = call_ollama(
            MODEL_ACTION, cmd,
            num_predict=30, num_ctx=1024, timeout=timeout,
        )

        # Step 2: 7B モデル（VRAM swap in → VRAM に残る）
        legacy_raw, legacy_ms, legacy_status = call_ollama(
            MODEL_7B, cmd,
            num_predict=100, num_ctx=2048, timeout=timeout,
        )

        # 決策抽出
        action_decision = extract_decision(action_raw, "action")
        legacy_decision = extract_decision(legacy_raw, "7b")

        # 比較
        if action_status == "ok" and legacy_status == "ok":
            comparison = compare_decisions(action_decision, legacy_decision)
        else:
            comparison = {
                "raw_agreement": False,
                "semantic_agreement": False,
                "high_risk_divergence": False,
                "both_null": False,
                "action_api": action_decision[0],
                "action_seq": action_decision[1],
                "legacy_api": legacy_decision[0],
                "legacy_seq": legacy_decision[1],
            }

        # 結果記録
        entry = {
            "timestamp": datetime.now().isoformat(),
            "index": idx,
            "command": cmd,
            "category": cat,
            "expected": expected,
            "action_raw": action_raw,
            "action_status": action_status,
            "action_ms": round(action_ms, 1),
            "legacy_raw": legacy_raw,
            "legacy_status": legacy_status,
            "legacy_ms": round(legacy_ms, 1),
            "comparison": comparison,
        }
        results.append(entry)

        # JSONL に即座に追記（クラッシュ耐性）
        with open(output_path, 'a') as f:
            f.write(json.dumps(entry, ensure_ascii=False) + '\n')

        # 表示
        agree_mark = "=" if comparison["raw_agreement"] else "!"
        action_api_str = str(action_decision[0]) if action_decision[0] is not None else (
            str(action_decision[1]) if action_decision[1] else "null")
        legacy_api_str = str(legacy_decision[0]) if legacy_decision[0] is not None else (
            str(legacy_decision[1]) if legacy_decision[1] else "null")

        print("  [{}/{}] [{}] {} → A:{} L:{} [{}] {:.0f}ms/{:.0f}ms".format(
            idx, n, cat[0].upper(),
            cmd[:20].ljust(20),
            action_api_str.ljust(6), legacy_api_str.ljust(6),
            agree_mark,
            action_ms, legacy_ms,
        ))

        # 遅延（Ollama GPU メモリ冷却）
        if i < n - 1:
            time.sleep(delay)

    return results


def print_summary(results):
    """結果サマリを表示"""
    total = len(results)
    if total == 0:
        print("\n  No results.")
        return

    # ステータス別集計
    action_ok = sum(1 for r in results if r["action_status"] == "ok")
    legacy_ok = sum(1 for r in results if r["legacy_status"] == "ok")
    both_ok = sum(1 for r in results
                  if r["action_status"] == "ok" and r["legacy_status"] == "ok")

    # 比較結果集計（both_ok のみ）
    ok_results = [r for r in results
                  if r["action_status"] == "ok" and r["legacy_status"] == "ok"]
    n_ok = len(ok_results)

    raw_agree = sum(1 for r in ok_results if r["comparison"]["raw_agreement"])
    semantic_agree = sum(1 for r in ok_results if r["comparison"]["semantic_agreement"])
    both_null = sum(1 for r in ok_results if r["comparison"]["both_null"])
    high_risk_div = sum(1 for r in ok_results if r["comparison"]["high_risk_divergence"])

    # カテゴリ別集計
    by_cat = {}
    for r in ok_results:
        cat = r["category"]
        if cat not in by_cat:
            by_cat[cat] = {"total": 0, "raw_agree": 0, "semantic_agree": 0}
        by_cat[cat]["total"] += 1
        if r["comparison"]["raw_agreement"]:
            by_cat[cat]["raw_agree"] += 1
        if r["comparison"]["semantic_agreement"]:
            by_cat[cat]["semantic_agree"] += 1

    # レイテンシ集計
    action_latencies = sorted([r["action_ms"] for r in ok_results])
    legacy_latencies = sorted([r["legacy_ms"] for r in ok_results])

    def percentile(arr, p):
        if not arr:
            return 0.0
        k = (len(arr) - 1) * p / 100
        f = int(k)
        c = f + 1 if f + 1 < len(arr) else f
        return arr[f] + (k - f) * (arr[c] - arr[f])

    print("\n" + "=" * 60)
    print("  Offline Route Comparison — Summary")
    print("=" * 60)

    print("\n  Status:")
    print("    Total commands:     {}".format(total))
    print("    Action model OK:    {}/{}".format(action_ok, total))
    print("    Legacy model OK:    {}/{}".format(legacy_ok, total))
    print("    Both OK:            {}/{}".format(both_ok, total))

    if n_ok > 0:
        print("\n  Decision Agreement (N={}):".format(n_ok))
        print("    Raw agreement:      {}/{} ({:.1f}%)".format(
            raw_agree, n_ok, raw_agree / n_ok * 100))
        print("    Semantic agreement: {}/{} ({:.1f}%)".format(
            semantic_agree, n_ok, semantic_agree / n_ok * 100))
        print("    Both null (conv):   {}/{} ({:.1f}%)".format(
            both_null, n_ok, both_null / n_ok * 100))
        print("    High-risk diverge:  {}/{} ({:.1f}%)".format(
            high_risk_div, n_ok, high_risk_div / n_ok * 100))

        print("\n  By Category:")
        for cat in sorted(by_cat.keys()):
            d = by_cat[cat]
            print("    {:<22} raw={}/{} ({:.0f}%)  semantic={}/{} ({:.0f}%)".format(
                cat,
                d["raw_agree"], d["total"],
                d["raw_agree"] / d["total"] * 100 if d["total"] else 0,
                d["semantic_agree"], d["total"],
                d["semantic_agree"] / d["total"] * 100 if d["total"] else 0,
            ))

        print("\n  Latency (ms) — includes VRAM swap:")
        print("    Action:  P50={:.0f}  P95={:.0f}  max={:.0f}".format(
            percentile(action_latencies, 50),
            percentile(action_latencies, 95),
            max(action_latencies) if action_latencies else 0,
        ))
        print("    Legacy:  P50={:.0f}  P95={:.0f}  max={:.0f}".format(
            percentile(legacy_latencies, 50),
            percentile(legacy_latencies, 95),
            max(legacy_latencies) if legacy_latencies else 0,
        ))

    # 不一致リスト
    disagree = [r for r in ok_results if not r["comparison"]["raw_agreement"]]
    if disagree:
        print("\n  Disagreements ({})".format(len(disagree)))
        for r in disagree[:20]:  # 最大 20 件
            a_str = str(r["comparison"].get("action_api") or r["comparison"].get("action_seq") or "null")
            l_str = str(r["comparison"].get("legacy_api") or r["comparison"].get("legacy_seq") or "null")
            sem = "sem_ok" if r["comparison"]["semantic_agreement"] else "sem_NG"
            hr = " HIGH_RISK" if r["comparison"]["high_risk_divergence"] else ""
            print("    {} → A:{} L:{} [{}]{}".format(
                r["command"][:25].ljust(25), a_str, l_str, sem, hr))

    print()


def main():
    parser = argparse.ArgumentParser(description="Offline route comparison: Action vs 7B")
    parser.add_argument("--timeout", type=int, default=60,
                        help="Per-model call timeout in seconds (default: 60)")
    parser.add_argument("--delay", type=float, default=2.0,
                        help="Delay between commands in seconds (default: 2.0)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSONL path (default: logs/offline_comparison_YYYYMMDD_HHMMSS.jsonl)")
    parser.add_argument("--batch", type=int, default=None,
                        help="Run only this batch number (2-7, batch 1=hot_cache excluded)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show commands without executing")
    parser.add_argument("--skip-warmup", action="store_true",
                        help="Skip model warmup")
    args = parser.parse_args()

    # 出力先
    if args.output:
        output_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("logs", exist_ok=True)
        output_path = "logs/offline_comparison_{}.jsonl".format(ts)

    # コマンドフィルタ
    if args.batch:
        if args.batch < 2 or args.batch > 7:
            print("Error: --batch must be 2-7 (batch 1 = hot_cache, excluded)")
            sys.exit(1)
        start, end = BATCH_RANGES[args.batch]
        commands = ALL_COMMANDS[start:end]
        # hot_cache を再除外（batch 1 は既に除外されるが念のため）
        commands = [(c, cat, e) for c, cat, e in commands if cat != "hot_cache"]
    else:
        commands = LLM_COMMANDS

    # ヘッダ
    print("=" * 60)
    print("  Offline Route Comparison")
    print("=" * 60)
    print("  Action model: {}".format(MODEL_ACTION))
    print("  Legacy model: {}".format(MODEL_7B))
    print("  Commands:     {} (LLM-routed only)".format(len(commands)))
    print("  Timeout:      {}s per model call".format(args.timeout))
    print("  Delay:        {}s between commands".format(args.delay))
    print("  Output:       {}".format(output_path))

    # カテゴリ別内訳
    cats = {}
    for _, cat, _ in commands:
        cats[cat] = cats.get(cat, 0) + 1
    for cat in sorted(cats.keys()):
        print("    {:<22} {}".format(cat, cats[cat]))

    # 時間見積もり（各コマンド: Action VRAM swap+推理 + 7B VRAM swap+推理 + delay）
    # 実測: Action ~15-25s, 7B ~15-25s = ~30-50s/command
    est_sec = len(commands) * (40 + args.delay)
    est_min = est_sec / 60
    print("  Est. time:    {:.0f} min".format(est_min))
    print()

    if args.dry_run:
        print("  [DRY RUN] Commands:")
        for i, (cmd, cat, exp) in enumerate(commands):
            print("    [{}/{}] [{}] {} → {}".format(
                i + 1, len(commands), cat, cmd, exp))
        return

    # 予熱
    if not args.skip_warmup:
        warmup_models(timeout=args.timeout)

    print("\n  Start: {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

    results = run_comparison(commands, output_path, args.timeout, args.delay)

    print("\n  End: {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

    print_summary(results)

    print("  Results saved to: {}".format(output_path))


if __name__ == "__main__":
    main()
