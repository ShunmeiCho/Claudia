#!/usr/bin/env python3
"""
Interactive Japanese Command Testing Interface - Integration Test
交互式日语指令界面集成测试

验证交互界面的核心功能和日语指令处理能力
"""

import sys
import asyncio
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.claudia.interactive_japanese_commander import JapaneseCommandInterface


class TestInteractiveJapaneseCommander(unittest.TestCase):
    """交互式日语指令界面测试"""
    
    def setUp(self):
        """测试前准备"""
        self.interface = JapaneseCommandInterface()
        
    def tearDown(self):
        """测试后清理"""
        if hasattr(self, 'interface'):
            asyncio.run(self.interface.cleanup())
    
    def test_interface_initialization(self):
        """测试界面初始化"""
        # 检查初始状态
        self.assertIsNone(self.interface.action_engine)
        self.assertIsNone(self.interface.robot_integration)
        self.assertIsNone(self.interface.llm_server)
        self.assertEqual(len(self.interface.command_history), 0)
        
        # 检查颜色配置
        self.assertIn('blue', self.interface.colors)
        self.assertIn('green', self.interface.colors)
        self.assertIn('red', self.interface.colors)
    
    @patch('src.claudia.robot_controller.action_mapping_engine_real.ActionMappingEngine')
    @patch('src.claudia.ai_components.llm_service.RobotIntegration')
    async def test_initialization_success(self, mock_robot_integration, mock_action_engine):
        """测试成功初始化"""
        # 模拟成功初始化
        mock_robot_integration.return_value = AsyncMock()
        mock_action_engine.return_value = AsyncMock()
        mock_action_engine.return_value.initialize = AsyncMock(return_value=True)
        
        result = await self.interface.initialize()
        
        self.assertTrue(result)
        self.assertIsNotNone(self.interface.robot_integration)
        self.assertIsNotNone(self.interface.action_engine)
    
    @patch('src.claudia.robot_controller.action_mapping_engine_real.ActionMappingEngine')
    async def test_initialization_failure(self, mock_action_engine):
        """测试初始化失败"""
        # 模拟初始化失败
        mock_action_engine.side_effect = Exception("连接失败")
        
        result = await self.interface.initialize()
        
        self.assertFalse(result)
    
    async def test_japanese_command_processing_structure(self):
        """测试日语指令处理结构"""
        # 模拟组件
        self.interface.robot_integration = AsyncMock()
        self.interface.action_engine = AsyncMock()
        
        # 模拟分析结果
        mock_analysis = {
            'command_type': MagicMock(value='motion'),
            'priority': MagicMock(value='normal'),
            'extracted_actions': ['move_forward'],
            'confidence': 0.8
        }
        self.interface.robot_integration.analyze_command.return_value = mock_analysis
        
        # 模拟机器人命令
        mock_robot_command = {
            'command_id': 'test_cmd_123',
            'actions': ['move_forward']
        }
        self.interface.robot_integration.create_robot_command.return_value = mock_robot_command
        
        # 模拟安全检查
        mock_safety = {
            'is_safe': True,
            'warnings': [],
            'blocking_issues': []
        }
        self.interface.robot_integration.validate_command_safety.return_value = mock_safety
        
        # 模拟动作执行
        mock_execution = {
            'success': True,
            'action_name': 'MoveForward',
            'api_id': 1001
        }
        self.interface.action_engine.execute_action.return_value = mock_execution
        
        # 执行测试
        result = await self.interface.process_japanese_command("前進")
        
        # 验证结果结构
        self.assertIn('timestamp', result)
        self.assertIn('user_input', result)
        self.assertIn('analysis', result)
        self.assertIn('robot_command', result)
        self.assertIn('execution_result', result)
        self.assertIn('success', result)
        
        self.assertEqual(result['user_input'], "前進")
        self.assertTrue(result['success'])
    
    async def test_japanese_motion_commands(self):
        """测试日语运动指令"""
        # 模拟组件
        self.interface.robot_integration = AsyncMock()
        self.interface.action_engine = AsyncMock()
        
        # 测试指令映射
        test_cases = [
            ("前進", "move_forward"),
            ("停止", "stop"),
            ("左", "turn_left"),
            ("右", "turn_right"),
            ("立つ", "stand_up"),
            ("座る", "sit_down")
        ]
        
        for japanese_cmd, expected_action in test_cases:
            with self.subTest(japanese_cmd=japanese_cmd):
                # 模拟分析结果
                mock_analysis = {
                    'command_type': MagicMock(value='motion'),
                    'priority': MagicMock(value='normal'),
                    'extracted_actions': [expected_action],
                    'confidence': 0.8
                }
                self.interface.robot_integration.analyze_command.return_value = mock_analysis
                
                # 模拟其他组件
                self.interface.robot_integration.create_robot_command.return_value = {
                    'command_id': f'test_{expected_action}',
                    'actions': [expected_action]
                }
                
                self.interface.robot_integration.validate_command_safety.return_value = {
                    'is_safe': True, 'warnings': [], 'blocking_issues': []
                }
                
                self.interface.action_engine.execute_action.return_value = {
                    'success': True, 'action_name': expected_action, 'api_id': 1001
                }
                
                # 执行测试
                result = await self.interface.process_japanese_command(japanese_cmd)
                
                # 验证结果
                self.assertTrue(result['success'])
                self.interface.robot_integration.analyze_command.assert_called_with(japanese_cmd)
    
    async def test_conversation_commands(self):
        """测试对话指令"""
        # 模拟组件
        self.interface.robot_integration = AsyncMock()
        
        # 模拟对话指令分析
        mock_analysis = {
            'command_type': MagicMock(value='conversation'),
            'priority': MagicMock(value='normal'),
            'extracted_actions': [],
            'confidence': 0.9
        }
        self.interface.robot_integration.analyze_command.return_value = mock_analysis
        
        # 测试对话指令
        result = await self.interface.process_japanese_command("こんにちは")
        
        # 验证结果
        self.assertTrue(result['success'])
        self.assertEqual(result['user_input'], "こんにちは")
        self.assertIsNone(result['error'])
    
    async def test_safety_validation_failure(self):
        """测试安全验证失败"""
        # 模拟组件
        self.interface.robot_integration = AsyncMock()
        self.interface.action_engine = AsyncMock()
        
        # 模拟分析结果
        mock_analysis = {
            'command_type': MagicMock(value='motion'),
            'priority': MagicMock(value='high'),
            'extracted_actions': ['move_forward'],
            'confidence': 0.8
        }
        self.interface.robot_integration.analyze_command.return_value = mock_analysis
        
        # 模拟机器人命令
        mock_robot_command = {
            'command_id': 'test_unsafe_cmd',
            'actions': ['move_forward']
        }
        self.interface.robot_integration.create_robot_command.return_value = mock_robot_command
        
        # 模拟安全检查失败
        mock_safety = {
            'is_safe': False,
            'warnings': ['高风险操作'],
            'blocking_issues': ['冲突的移动指令']
        }
        self.interface.robot_integration.validate_command_safety.return_value = mock_safety
        
        # 执行测试
        result = await self.interface.process_japanese_command("前進と後退")
        
        # 验证结果
        self.assertFalse(result['success'])
        self.assertIn("安全检查失败", result['error'])
    
    def test_special_commands(self):
        """测试特殊命令处理"""
        # 测试帮助命令
        with patch('builtins.print') as mock_print:
            result = self.interface.handle_special_command('/help')
            self.assertTrue(result)
            mock_print.assert_called()
        
        # 测试历史命令
        with patch('builtins.print') as mock_print:
            result = self.interface.handle_special_command('/history')
            self.assertTrue(result)
            mock_print.assert_called()
        
        # 测试状态命令
        with patch('builtins.print') as mock_print:
            result = self.interface.handle_special_command('/status')
            self.assertTrue(result)
            mock_print.assert_called()
        
        # 测试退出命令
        result = self.interface.handle_special_command('/exit')
        self.assertFalse(result)
        
        # 测试未知命令
        with patch('builtins.print') as mock_print:
            result = self.interface.handle_special_command('/unknown')
            self.assertTrue(result)
            mock_print.assert_called()
    
    async def test_emergency_stop(self):
        """测试紧急停止功能"""
        # 模拟组件
        self.interface.robot_integration = AsyncMock()
        self.interface.action_engine = AsyncMock()
        
        # 模拟紧急停止命令
        mock_emergency = {
            'command_id': 'emergency_123',
            'actions': ['stop']
        }
        self.interface.robot_integration.emergency_stop.return_value = mock_emergency
        
        # 模拟成功执行
        self.interface.action_engine.execute_action.return_value = {
            'success': True
        }
        
        # 执行紧急停止
        await self.interface.emergency_stop()
        
        # 验证调用
        self.interface.robot_integration.emergency_stop.assert_called_once()
        self.interface.action_engine.execute_action.assert_called_once()
    
    def test_command_history_tracking(self):
        """测试命令历史跟踪"""
        # 添加模拟历史记录
        mock_record = {
            'timestamp': '2024-12-26T13:45:22',
            'user_input': 'テスト',
            'success': True,
            'analysis': {'command_type': MagicMock(value='test')}
        }
        
        self.interface.command_history.append(mock_record)
        
        # 检查历史记录
        self.assertEqual(len(self.interface.command_history), 1)
        self.assertEqual(self.interface.command_history[0]['user_input'], 'テスト')
        
        # 测试历史显示
        self.interface.robot_integration = MagicMock()
        self.interface.robot_integration.get_command_history.return_value = []
        
        with patch('builtins.print') as mock_print:
            self.interface.show_history()
            mock_print.assert_called()


class TestJapaneseCommandIntegration(unittest.TestCase):
    """日语指令集成测试"""
    
    def test_japanese_keyword_coverage(self):
        """测试日语关键词覆盖度"""
        from src.claudia.ai_components.llm_service.integration import RobotIntegration
        
        integration = RobotIntegration()
        
        # 测试运动关键词
        motion_tests = [
            "前進", "進む", "前", "まえ", "すすむ",
            "後退", "戻る", "後ろ", "うしろ", "もどる",
            "左", "ひだり", "左回転",
            "右", "みぎ", "右回転",
            "停止", "止まる", "とまる", "ストップ"
        ]
        
        for keyword in motion_tests:
            self.assertIn(keyword, integration.motion_keywords)
        
        # 测试对话关键词
        conversation_tests = [
            "こんにちは", "おはよう", "こんばんは",
            "ありがとう", "感謝",
            "さようなら", "バイバイ"
        ]
        
        for keyword in conversation_tests:
            self.assertIn(keyword, integration.conversation_keywords)


def run_tests():
    """运行所有测试"""
    print("🧪 开始运行交互式日语指令界面测试")
    print("⏰ 测试时间:", __import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试类
    suite.addTests(loader.loadTestsFromTestCase(TestInteractiveJapaneseCommander))
    suite.addTests(loader.loadTestsFromTestCase(TestJapaneseCommandIntegration))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 输出结果
    print(f"\n📊 测试结果:")
    print(f"✅ 成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ 失败: {len(result.failures)}")
    print(f"🚫 错误: {len(result.errors)}")
    
    if result.failures:
        print("\n❌ 失败的测试:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split(chr(10))[-2]}")
    
    if result.errors:
        print("\n🚫 错误的测试:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split(chr(10))[-2]}")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    import asyncio
    
    # 运行异步测试
    success = run_tests()
    
    if success:
        print("\n🎉 所有测试通过！")
        exit(0)
    else:
        print("\n💥 测试失败，请检查错误信息")
        exit(1) 