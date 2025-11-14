#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速A/B测试脚本（Track B）
对比现有模型vs新模型的核心指标
"""

import ollama
import json
import time

# 测试用例（扩展至25条，增加状态感知测试）
TEST_CASES = [
    # 基础精确匹配（5条）
    ("座って", 1009, "basic", None),
    ("立って", 1004, "basic", None),
    ("ハート", 1036, "basic", None),
    ("ダンス", 1023, "basic", None),  # 1022或1023都对
    ("stop", 1003, "basic", None),

    # 语义理解（10条）- 关键测试点
    ("疲れた", 1009, "semantic", None),  # 应理解为休息→Sit
    ("可愛い", 1036, "semantic", None),  # 应理解为可爱→Heart
    ("元気", 1023, "semantic", None),  # 应理解为活力→Dance
    ("休みたい", 1009, "semantic", None),
    ("挨拶して", 1016, "semantic", None),
    ("嬉しい", 1023, "semantic", None),
    ("遊んで", 1023, "semantic", None),
    ("かっこいい", 1030, "semantic", None),
    ("お疲れ様", 1009, "semantic", None),
    ("楽しい", 1023, "semantic", None),

    # 多语（3条）
    ("sit", 1009, "multilang", None),
    ("cute", 1036, "multilang", None),
    ("坐下", 1009, "multilang", None),

    # 异常（2条）
    ("", None, "edge", None),
    ("あいうえお", None, "edge", None),

    # 状态感知测试（5条新增）- 修复REVIEW问题
    ("ダンス", 1023, "state_aware", {"battery": 85, "standing": False}),  # 坐姿→应自动站立
    ("ハート", 1036, "state_aware", {"battery": 15, "standing": True}),   # 低电量→应拒绝或降级
    ("前転", None, "state_aware", {"battery": 8, "standing": True}),      # 极低电量→应拒绝高能动作
    ("可愛い", 1036, "state_aware", {"battery": 90, "standing": False}),  # 坐姿+高电量→应先站立
    ("stop", 1003, "state_aware", {"battery": 3, "standing": False}),     # 紧急状态→允许Stop
]


def test_model(model_name: str):
    """测试单个模型（修复：支持状态注入）"""
    print(f"\n{'='*60}")
    print(f"测试模型: {model_name}")
    print(f"{'='*60}\n")

    results = {"correct": 0, "total": 0, "json_valid": 0, "elapsed": []}

    for test_case in TEST_CASES:
        # 解包测试用例（支持4元组和3元组）
        if len(test_case) == 4:
            cmd, expected_api, category, state = test_case
        else:
            cmd, expected_api, category = test_case
            state = None

        results["total"] += 1

        try:
            # 构造输入（修复REVIEW：注入状态信息，格式对齐Modelfile）
            if state:
                battery = state.get("battery", 100)
                standing = state.get("standing", True)
                # 修复：使用posture:standing/sitting格式（与Modelfile一致）
                posture = "standing" if standing else "sitting"
                state_prefix = f"[STATE] posture:{posture}, battery:{battery:.0f}%, space:normal\n\n"
                full_cmd = state_prefix + cmd
            else:
                full_cmd = cmd

            start = time.time()
            response = ollama.chat(
                model=model_name,
                messages=[{'role': 'user', 'content': full_cmd}],
                format='json',
                options={'temperature': 0.0, 'num_predict': 256}
            )
            elapsed = (time.time() - start) * 1000
            results["elapsed"].append(elapsed)

            content = response['message']['content']
            data = json.loads(content)
            results["json_valid"] += 1

            # 修复：支持缩写字段（旧模型用a/r，新模型用api_code/response）
            actual_api = data.get('api_code') or data.get('a')

            # 判断正确性（状态测试需要更灵活的验证）
            is_correct = False
            if category == "state_aware":
                # 状态感知测试：验证安全裁决而非精确API码
                if state and state.get("battery", 100) < 10:
                    # 极低电量：应该拒绝(None/0)或只允许Stop/Sit
                    is_correct = actual_api in [None, 0, 1003, 1009]
                elif state and not state.get("standing", True):
                    # 非站立姿态且命令需要站立：应包含站立动作或拒绝
                    sequence = data.get('sequence', [])
                    if sequence is None:
                        sequence = []
                    is_correct = (1004 in sequence) or actual_api in [None, 0]
                else:
                    # 其他状态：按正常逻辑判断
                    is_correct = actual_api == expected_api or actual_api in [None, 0]
            else:
                # 非状态测试：精确匹配
                if expected_api is None:
                    is_correct = actual_api in [None, 0]
                elif expected_api == 1023:
                    is_correct = actual_api in [1022, 1023]
                else:
                    is_correct = actual_api == expected_api

            # 显示结果
            display_cmd = f"{cmd:15s}" if not state else f"{cmd:10s}[{state}]"
            if is_correct:
                results["correct"] += 1
                print(f"✅ {display_cmd} → {actual_api} ({elapsed:.0f}ms)")
            else:
                print(f"❌ {display_cmd} → {actual_api} (期望:{expected_api}, {elapsed:.0f}ms)")

        except Exception as e:
            print(f"❌ {cmd:15s} → 错误: {e}")

    # 统计
    accuracy = results["correct"] / results["total"] * 100
    json_rate = results["json_valid"] / results["total"] * 100
    avg_elapsed = sum(results["elapsed"]) / len(results["elapsed"]) if results["elapsed"] else 0

    print(f"\n📊 统计:")
    print(f"  准确率: {accuracy:.1f}% ({results['correct']}/{results['total']})")
    print(f"  JSON合规率: {json_rate:.1f}%")
    print(f"  平均延迟: {avg_elapsed:.0f}ms")

    return results, accuracy, avg_elapsed


def main():
    """主函数"""
    print("="*60)
    print("🧪 A/B快速测试")
    print("="*60)

    # 测试旧模型（baseline）
    print("\n【Baseline】")
    old_results, old_acc, old_elapsed = test_model("claudia-go2-3b:v11.2")

    # 测试新模型
    print("\n【新模型】")
    try:
        new_results, new_acc, new_elapsed = test_model("claudia-intelligent-3b:v1")

        # 对比
        print(f"\n{'='*60}")
        print("📈 对比结果")
        print(f"{'='*60}")
        print(f"准确率: {old_acc:.1f}% → {new_acc:.1f}% ({new_acc-old_acc:+.1f}%)")
        print(f"延迟: {old_elapsed:.0f}ms → {new_elapsed:.0f}ms ({new_elapsed-old_elapsed:+.0f}ms)")

        if new_acc >= old_acc - 5 and new_elapsed <= 2000:
            print("\n✅ 新模型通过验收标准！")
        else:
            print("\n⚠️  新模型未达标")

    except Exception as e:
        print(f"\n⚠️  新模型不可用: {e}")
        print("请先运行: ./deploy_track_b.sh")


if __name__ == "__main__":
    main()
