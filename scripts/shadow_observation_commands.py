#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
shadow_observation_commands.py — Shadow 观测批量命令脚本

在 BRAIN_ROUTER_MODE=shadow 下批量发送命令，积累 audit 数据。
目标: 100+ LLM 路由命令（hot_cache 命令不计入 LLM 路由统计）。

用法:
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py --hardware
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py --batch 1  # 只跑第1批
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py --dry-run   # 只打印不执行

每条命令间隔 3 秒（LLM 推理 + VRAM swap 冷却），总计约 10-12 分钟。
"""

import os
import sys
import asyncio
import time
import argparse
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# 禁用 ROS2 状态监控（避免 Jetson OOM / DDS 初始化）
import claudia.brain.production_brain as pb_mod
pb_mod.STATE_MONITOR_AVAILABLE = False


# === 命令清单 ===
# 格式: (command, category, expected_behavior)
# category: "hot_cache" | "llm_action" | "llm_conversational" | "llm_sequence"
# expected_behavior: 简述期望行为

COMMANDS = [
    # ============================================================
    # Batch 1: 热缓存对照组 (20 commands, ~0ms each)
    # 这些不计入 LLM 路由统计，但验证 shadow 模式下 hot_cache 仍正确
    # ============================================================
    ("座って", "hot_cache", "Sit(1009)"),
    ("立って", "hot_cache", "StandUp(1004)"),
    ("止まれ", "hot_cache", "Stop(1003)"),
    ("かわいい", "hot_cache", "Heart(1036)"),
    ("踊って", "hot_cache", "Dance1(1022)"),
    ("ちんちん", "hot_cache", "Scrape(1029)"),
    ("お辞儀", "hot_cache", "Scrape(1029)"),
    ("おすわり", "hot_cache", "Sit(1009)"),
    ("ストレッチ", "hot_cache", "Stretch(1017)"),
    ("ハート", "hot_cache", "Heart(1036)"),
    ("こんにちは", "hot_cache", "Hello(1016)"),
    ("おはよう", "hot_cache", "Hello(1016)"),
    ("バランス", "hot_cache", "Balance(1002)"),
    ("伏せる", "hot_cache", "StandDown(1005)"),
    ("チンチン", "hot_cache", "Scrape(1029)"),
    ("拜年", "hot_cache", "Scrape(1029)"),
    ("すごい", "hot_cache", "Heart(1036)"),
    ("停止", "hot_cache", "Stop(1003)"),
    ("ジャンプ", "hot_cache", "FrontJump(1031)"),
    ("比心", "hot_cache", "Heart(1036)"),

    # ============================================================
    # Batch 2: LLM 动作命令 — 基础动作 (25 commands)
    # 不在 hot_cache 中的自然表达，期望 LLM 映射到正确 api_code
    # ============================================================
    ("座りなさい", "llm_action", "expect Sit(1009)"),
    ("お座りして", "llm_action", "expect Sit(1009)"),
    ("しゃがんで", "llm_action", "expect Sit(1009)"),
    ("立ちなさい", "llm_action", "expect StandUp(1004)"),
    ("起きて", "llm_action", "expect StandUp(1004)"),
    ("立ち上がって", "llm_action", "expect StandUp(1004)"),
    ("止まりなさい", "llm_action", "expect Stop(1003)"),
    ("動かないで", "llm_action", "expect Stop(1003)"),
    ("ストップ", "llm_action", "expect Stop(1003)"),
    ("横になって", "llm_action", "expect StandDown(1005)"),
    ("寝てください", "llm_action", "expect StandDown(1005)"),
    ("伏せて", "llm_action", "expect StandDown(1005)"),
    ("休んで", "llm_action", "expect Sit(1009)/StandDown(1005)"),
    ("疲れたでしょう", "llm_action", "expect Sit(1009)/StandDown(1005)"),
    ("ゆっくりして", "llm_action", "expect Sit(1009)"),
    ("リラックスして", "llm_action", "expect Sit(1009)/StandDown(1005)"),
    ("体を起こして", "llm_action", "expect Recovery(1006)/StandUp(1004)"),
    ("回復して", "llm_action", "expect Recovery(1006)"),
    ("元に戻って", "llm_action", "expect Recovery(1006)/StandUp(1004)"),
    ("正しい姿勢に", "llm_action", "expect StandUp(1004)/Balance(1002)"),
    ("安定して", "llm_action", "expect Balance(1002)"),
    ("バランスを取って", "llm_action", "expect Balance(1002)"),
    ("しっかり立って", "llm_action", "expect StandUp(1004)"),
    ("ちゃんと座って", "llm_action", "expect Sit(1009)"),
    ("ゆっくり座って", "llm_action", "expect Sit(1009)"),

    # ============================================================
    # Batch 3: LLM 動作命令 — 表演・エモーション (25 commands)
    # ============================================================
    ("ストレッチして", "llm_action", "expect Stretch(1017)"),
    ("体を伸ばして", "llm_action", "expect Stretch(1017)"),
    ("伸びをして", "llm_action", "expect Stretch(1017)"),
    ("踊りなさい", "llm_action", "expect Dance1(1022)/Dance2(1023)"),
    ("ダンスして", "llm_action", "expect Dance1(1022)/Dance2(1023)"),
    ("かっこよく踊って", "llm_action", "expect Dance2(1023)"),
    ("楽しく踊って", "llm_action", "expect Dance1(1022)"),
    ("挨拶して", "llm_action", "expect Hello(1016)"),
    ("手を振って", "llm_action", "expect Hello(1016)"),
    ("ハートして", "llm_action", "expect Heart(1036)"),
    ("ハートを見せて", "llm_action", "expect Heart(1036)"),
    ("愛してる", "llm_action", "expect Heart(1036)"),
    ("大好き", "llm_action", "expect Heart(1036)"),
    ("いい子だね", "llm_action", "expect Heart(1036)"),
    ("元気を出して", "llm_action", "expect Dance/Jump"),
    ("かっこいいポーズして", "llm_action", "expect Pose(1028)"),
    ("ポーズを取って", "llm_action", "expect Pose(1028)"),
    ("飛んで", "llm_action", "expect FrontJump(1031)/FrontFlip(1030)"),
    ("ジャンプして", "llm_action", "expect FrontJump(1031)"),
    ("前に跳んで", "llm_action", "expect FrontJump(1031)/FrontPounce(1032)"),
    ("転がって", "llm_action", "expect RiseSit(1010)/RollOver"),
    ("ゴロンして", "llm_action", "expect RiseSit(1010)"),
    ("お辞儀して", "llm_action", "expect Scrape(1029)"),
    ("頭を下げて", "llm_action", "expect Scrape(1029)"),
    ("腰振って", "llm_action", "expect WiggleHips(1033)"),

    # ============================================================
    # Batch 4: LLM 動作命令 — 中文変体 (15 commands)
    # ============================================================
    ("坐下", "llm_action", "expect Sit(1009)"),
    ("站起来", "llm_action", "expect StandUp(1004)"),
    ("趴下", "llm_action", "expect StandDown(1005)"),
    ("跳舞", "llm_action", "expect Dance1(1022)/Dance2(1023)"),
    ("伸个懒腰", "llm_action", "expect Stretch(1017)"),
    ("打个招呼", "llm_action", "expect Hello(1016)"),
    ("做个爱心", "llm_action", "expect Heart(1036)"),
    ("跳一下", "llm_action", "expect FrontJump(1031)"),
    ("翻个跟头", "llm_action", "expect FrontFlip(1030)"),
    ("恢复正常", "llm_action", "expect Recovery(1006)"),
    ("不要动", "llm_action", "expect Stop(1003)"),
    ("摆个姿势", "llm_action", "expect Pose(1028)"),
    ("你好棒", "llm_action", "expect Heart(1036)"),
    ("真厉害", "llm_action", "expect Heart(1036)"),
    ("休息一下", "llm_action", "expect Sit(1009)/StandDown(1005)"),

    # ============================================================
    # Batch 5: LLM 会话命令 — a=null 路径 (20 commands)
    # ============================================================
    ("今日の天気は？", "llm_conversational", "expect a=null, 会話返答"),
    ("何時ですか？", "llm_conversational", "expect a=null, 会話返答"),
    ("あなたは誰？", "llm_conversational", "expect a=null, 会話返答"),
    ("名前は何ですか？", "llm_conversational", "expect a=null, 自己紹介"),
    ("元気？", "llm_conversational", "expect a=null, 相づち"),
    ("今日は何をしますか？", "llm_conversational", "expect a=null, 会話"),
    ("好きな食べ物は？", "llm_conversational", "expect a=null, 会話"),
    ("何歳ですか？", "llm_conversational", "expect a=null, 会話"),
    ("お腹すいた？", "llm_conversational", "expect a=null, 相づち"),
    ("どこに住んでる？", "llm_conversational", "expect a=null, 会話"),
    ("Claudiaちゃん", "llm_conversational", "expect a=null, 名前呼び"),
    ("今日は暑いね", "llm_conversational", "expect a=null, 天気雑談"),
    ("疲れた？", "llm_conversational", "expect a=null or Sit"),
    ("楽しい？", "llm_conversational", "expect a=null, 相づち"),
    ("ありがとう", "llm_conversational", "expect a=null or Heart"),
    ("你是谁？", "llm_conversational", "expect a=null, 自己紹介"),
    ("今天天气怎么样？", "llm_conversational", "expect a=null, 天気"),
    ("你叫什么名字？", "llm_conversational", "expect a=null, 名前"),
    ("你多大了？", "llm_conversational", "expect a=null, 年齢"),
    ("你喜欢什么？", "llm_conversational", "expect a=null, 会話"),

    # ============================================================
    # Batch 6: LLM 序列命令 — 複数動作 (10 commands)
    # ============================================================
    ("立ってから座って", "llm_sequence", "expect [1004, 1009]"),
    ("座ってから立って", "llm_sequence", "expect [1009, 1004]"),
    ("立ってから挨拶して", "llm_sequence", "expect [1004, 1016]"),
    ("ストレッチしてからダンスして", "llm_sequence", "expect [1017, 1022/1023]"),
    ("起きてから踊って", "llm_sequence", "expect [1004, 1022]"),
    ("先にストレッチして、その後ダンスして", "llm_sequence", "expect [1017, 1022]"),
    ("立って、それからお辞儀して", "llm_sequence", "expect [1004, 1029]"),
    ("站起来然后跳舞", "llm_sequence", "expect [1004, 1022]"),
    ("先坐下再站起来", "llm_sequence", "expect [1009, 1004]"),
    ("伸展然后跳", "llm_sequence", "expect [1017, 1031]"),

    # ============================================================
    # Batch 7: エッジケース + ノイズ (5 commands)
    # ============================================================
    ("", "edge_case", "空入力 → 会話 or エラー"),
    ("あいうえお", "edge_case", "意味不明 → a=null"),
    ("1234", "edge_case", "数字入力 → a=null"),
    ("asdfjkl", "edge_case", "ランダム英字 → a=null"),
    ("ちんちんして立ってから踊って", "edge_case", "hot_cache語 + 序列語の混合"),
]

# バッチ区分
BATCH_RANGES = {
    1: (0, 20),     # hot_cache 対照
    2: (20, 45),    # LLM 基礎動作
    3: (45, 70),    # LLM 表演
    4: (70, 85),    # LLM 中文
    5: (85, 105),   # LLM 会話
    6: (105, 115),  # LLM 序列
    7: (115, 120),  # エッジケース
}


def count_by_category():
    """カテゴリ別統計"""
    from collections import Counter
    cats = Counter(cat for _, cat, _ in COMMANDS)
    return cats


def create_brain(hardware=False):
    """Shadow 用 brain インスタンス作成"""
    os.environ["BRAIN_ROUTER_MODE"] = "shadow"
    from claudia.brain.production_brain import ProductionBrain
    brain = ProductionBrain(use_real_hardware=hardware)

    if not hardware:
        from types import SimpleNamespace
        mock_state = SimpleNamespace(
            battery_level=0.80,
            is_standing=True,
            is_moving=False,
            temperature=40.0,
            timestamp=0.0,
            source="shadow_observation",
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


def run_observation(brain, commands, delay=3.0):
    """命令を順次実行し、結果を記録"""
    results = []
    total = len(commands)

    for i, (cmd, cat, expected) in enumerate(commands, 1):
        if not cmd:
            # 空入力スキップ
            print("  [{}/{}] (skip empty)".format(i, total))
            continue

        t0 = time.monotonic()
        try:
            output = run_async(brain.process_and_execute(cmd))
            elapsed = (time.monotonic() - t0) * 1000

            api = output.api_code
            seq = getattr(output, 'sequence', None)
            resp = (output.response or "")[:30]
            exec_st = getattr(output, 'execution_status', None)

            status_icon = "A" if api else ("S" if seq else "C")
            result_str = "api={} seq={} exec={} resp='{}' {:.0f}ms".format(
                api, seq, exec_st, resp, elapsed)
            results.append((cmd, cat, True, result_str))

            print("  [{}/{}] [{}] {:20s} → {}".format(
                i, total, status_icon, cmd[:20], result_str))

        except Exception as e:
            elapsed = (time.monotonic() - t0) * 1000
            results.append((cmd, cat, False, str(e)))
            print("  [{}/{}] [E] {:20s} → ERROR: {} ({:.0f}ms)".format(
                i, total, cmd[:20], e, elapsed))

        # 推理間隔（VRAM swap 冷却）
        if delay > 0:
            time.sleep(delay)

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Shadow 観測バッチ: 120+ commands for audit data accumulation")
    parser.add_argument("--hardware", action="store_true",
                        help="実機接続モード")
    parser.add_argument("--batch", type=int, default=0,
                        help="特定バッチのみ実行 (1-7, 0=全部)")
    parser.add_argument("--dry-run", action="store_true",
                        help="コマンド一覧を表示（実行しない）")
    parser.add_argument("--delay", type=float, default=3.0,
                        help="コマンド間の待機秒数 (default: 3.0)")
    args = parser.parse_args()

    # 統計表示
    cats = count_by_category()
    llm_total = sum(v for k, v in cats.items() if k != "hot_cache")
    print("=" * 60)
    print("  Shadow 観測バッチ")
    print("=" * 60)
    print("  総コマンド数: {}".format(len(COMMANDS)))
    print("  LLM 路由: {} (shadow 対比対象)".format(llm_total))
    for cat, count in sorted(cats.items()):
        print("    {:25s} {}".format(cat, count))
    print("  推定時間: {:.0f}分 (delay={}s)".format(
        len(COMMANDS) * args.delay / 60, args.delay))
    print("  モード: {}".format("hardware" if args.hardware else "simulation"))
    print("  BRAIN_ROUTER_MODE: {}".format(
        os.environ.get("BRAIN_ROUTER_MODE", "(not set)")))
    print()

    # バッチ選択
    if args.batch > 0:
        if args.batch not in BATCH_RANGES:
            print("ERROR: バッチ {} は存在しません (1-7)".format(args.batch))
            return 1
        start, end = BATCH_RANGES[args.batch]
        commands = COMMANDS[start:end]
        print("  バッチ {}: commands[{}:{}] ({} commands)".format(
            args.batch, start, end, len(commands)))
    else:
        commands = COMMANDS
        print("  全バッチ実行 ({} commands)".format(len(commands)))

    # ドライラン
    if args.dry_run:
        print("\n--- Dry Run: コマンド一覧 ---")
        for i, (cmd, cat, exp) in enumerate(commands, 1):
            print("  {:3d}. [{:20s}] {:25s}  → {}".format(i, cat, cmd, exp))
        return 0

    # 実行
    print()
    brain = create_brain(hardware=args.hardware)
    start_time = datetime.now()
    print("  開始: {}".format(start_time.strftime("%Y-%m-%d %H:%M:%S")))
    print()

    results = run_observation(brain, commands, delay=args.delay)

    end_time = datetime.now()
    elapsed_total = (end_time - start_time).total_seconds()

    # サマリー
    print("\n" + "=" * 60)
    print("  完了: {} ({:.0f}秒)".format(
        end_time.strftime("%H:%M:%S"), elapsed_total))
    print("=" * 60)

    success = sum(1 for _, _, ok, _ in results if ok)
    errors = len(results) - success
    print("  成功: {}/{}".format(success, len(results)))
    if errors > 0:
        print("  エラー: {}".format(errors))

    # カテゴリ別成功率
    from collections import defaultdict
    cat_stats = defaultdict(lambda: [0, 0])  # [success, total]
    for _, cat, ok, _ in results:
        cat_stats[cat][1] += 1
        if ok:
            cat_stats[cat][0] += 1
    print("\n  カテゴリ別:")
    for cat in sorted(cat_stats):
        s, t = cat_stats[cat]
        print("    {:25s} {}/{} ({:.0f}%)".format(cat, s, t, s / t * 100))

    print("\n  次のステップ:")
    print("  python3 scripts/audit_baseline.py --min-n 100")

    return 0 if errors == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
