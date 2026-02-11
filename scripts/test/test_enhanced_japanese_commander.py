#!/usr/bin/env python3
"""
Enhanced Japanese Command Interface Test Script
增强版日语指令界面测试脚本

测试LLM集成、机器人状态管理和智能动作序列功能
"""

import sys
import asyncio
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface

async def test_enhanced_japanese_commander():
    """测试增强版日语指令界面的主要功能"""
    
    print("🧪 增强版日语指令界面测试开始")
    print("=" * 50)
    
    # 创建界面实例
    interface = EnhancedJapaneseCommandInterface()
    
    # 初始化系统
    print("\n🔧 初始化系统...")
    init_success = await interface.initialize()
    
    if not init_success:
        print("❌ 初始化失败，测试终止")
        return
    
    print("✅ 系统初始化成功")
    
    # 测试用例
    test_commands = [
        "こんにちは、お座りしてください",  # 问候+坐下指令
        "立ち上がってからダンスして",      # 站立+舞蹈组合指令
        "ストレッチしてください",           # 伸展指令
        "機器人の状態を確認して",           # 状态查询
        "緊急停止！",                     # 紧急停止
    ]
    
    print(f"\n🎯 开始测试 {len(test_commands)} 个日语指令...")
    
    for i, command in enumerate(test_commands, 1):
        print(f"\n{'='*60}")
        print(f"🧪 测试 {i}/{len(test_commands)}: {command}")
        print('='*60)
        
        try:
            result = await interface.process_japanese_command(command)
            
            # 显示测试结果摘要
            print(f"\n📊 测试结果摘要:")
            print(f"  - 执行状态: {'✅ 成功' if result['success'] else '❌ 失败'}")
            print(f"  - 总耗时: {result['total_time']:.2f}s")
            
            if result.get('llm_analysis'):
                llm_analysis = result['llm_analysis']
                print(f"  - LLM分析: 耗时 {llm_analysis.get('analysis_time', 0):.2f}s")
                if llm_analysis.get('extracted_action'):
                    print(f"  - 识别动作: {llm_analysis['extracted_action']} (置信度: {llm_analysis.get('confidence', 0):.1%})")
                
            if result.get('action_sequence'):
                seq = result['action_sequence']
                print(f"  - 动作序列: {len(seq)}个步骤")
                for j, step in enumerate(seq, 1):
                    print(f"    {j}. {step['action']} (API: {step['api']})")
            
            if result.get('execution_result'):
                exec_result = result['execution_result']
                if exec_result.get('completed_steps'):
                    print(f"  - 执行结果: {exec_result['completed_steps']}/{exec_result.get('total_steps', 0)}步骤成功")
                
        except Exception as e:
            print(f"❌ 测试异常: {str(e)}")
        
        print(f"\n{'='*60}")
        
        # 短暂暂停
        await asyncio.sleep(1)
    
    # 显示最终统计
    print(f"\n🏁 测试完成！")
    print(f"📋 命令历史记录: {len(interface.command_history)}条")
    print(f"🤖 当前机器人状态: {interface.robot_state.current_posture}")
    print(f"🔋 电池电量: {interface.robot_state.battery_level}%")
    
    # 显示详细的历史记录
    print(f"\n📚 详细执行历史:")
    for i, cmd in enumerate(interface.command_history, 1):
        status = "✅" if cmd.get('success') else "❌"
        print(f"  {i}. {status} {cmd['user_input']} ({cmd['total_time']:.1f}s)")

async def test_llm_extraction():
    """单独测试LLM动作提取功能"""
    print("\n🧠 LLM动作提取测试")
    print("-" * 40)
    
    interface = EnhancedJapaneseCommandInterface()
    
    test_phrases = [
        "お座りしてください",
        "立ち上がって",
        "ダンスしましょう",
        "こんにちは",
        "ストレッチしてみて",
        "停止してください",
        "回ってください",
        "緊急停止！",
    ]
    
    for phrase in test_phrases:
        action, confidence = interface.extract_action_from_llm_response(phrase)
        print(f"  '{phrase}' → {action or '未知'} (置信度: {confidence:.1%})")

async def test_action_sequencing():
    """测试动作序列规划功能"""
    print("\n🎯 动作序列规划测试")
    print("-" * 40)
    
    interface = EnhancedJapaneseCommandInterface()
    
    # 测试不同状态下的动作规划
    test_scenarios = [
        ("sitting", "hello", 1016, "从坐着状态执行问候"),
        ("lying", "sit", 1009, "从躺着状态坐下"),
        ("standing", "dance", 1022, "从站立状态跳舞"),
        ("unknown", "stretch", 1017, "从未知状态伸展"),
    ]
    
    for current_state, target_action, api_code, description in test_scenarios:
        interface.robot_state.current_posture = current_state
        sequence = interface.action_sequencer.plan_action_sequence(target_action, api_code)
        
        print(f"  {description}:")
        print(f"    当前状态: {current_state}")
        print(f"    目标动作: {target_action}")
        print(f"    规划序列: {len(sequence)}步")
        for i, step in enumerate(sequence, 1):
            print(f"      {i}. {step['action']} (API: {step['api']})")
        print()

async def main():
    """主函数"""
    print("🤖 Claudia Robot - 增强版日语指令界面全面测试")
    print("=" * 60)
    
    try:
        # 运行LLM提取测试
        await test_llm_extraction()
        
        # 运行动作序列测试
        await test_action_sequencing()
        
        # 运行完整功能测试
        await test_enhanced_japanese_commander()
        
        print("\n🎉 所有测试完成！")
        
    except Exception as e:
        print(f"\n❌ 测试过程发生异常: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main()) 