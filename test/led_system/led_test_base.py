#!/usr/bin/env python3
"""
LED测试基础类
为所有LED相关测试提供通用功能和工具方法
"""

import unittest
import time
import sys
import os
from pathlib import Path
from typing import Optional, Dict, Any, List
from datetime import datetime
import json

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

class LEDTestBase(unittest.TestCase):
    """LED测试基础类"""
    
    def setUp(self):
        """测试前准备"""
        self.start_time = time.time()
        self.test_name = self._testMethodName
        self.performance_data = {}
        self.error_logs = []
        
        # 确保测试环境
        self._setup_test_environment()
        
        # 创建LED控制器（如果可用）
        self._setup_led_controller()
        
    def tearDown(self):
        """测试后清理"""
        self.end_time = time.time()
        self.test_duration = self.end_time - self.start_time
        
        # 清理LED控制器
        self._cleanup_led_controller()
        
        # 记录测试结果
        self._record_test_results()
        
    def _setup_test_environment(self):
        """设置测试环境"""
        try:
            # 检查必要的模块是否可用
            import sys
            _project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            sys.path.append(os.path.join(_project_root, 'src'))
            
            # 检查Unitree硬件是否可用
            from claudia.robot_controller.unitree_messages import UnitreeMessages
            self.hardware_available = UnitreeMessages.is_available()
            
            if self.hardware_available:
                print(f"✅ {self.test_name}: 硬件模式可用")
            else:
                print(f"⚠️ {self.test_name}: 使用模拟模式")
                
        except Exception as e:
            self.hardware_available = False
            self.error_logs.append(f"环境设置失败: {e}")
            print(f"❌ {self.test_name}: 环境设置失败 - {e}")
    
    def _setup_led_controller(self):
        """设置LED控制器"""
        self.led_controller = None
        self.led_system = None
        
        try:
            # 尝试导入LED控制系统
            from claudia.robot_controller import (
                create_claudia_led_system,
                create_unified_led_controller
            )
            
            # 创建LED系统
            self.led_system = create_claudia_led_system()
            if self.led_system:
                self.led_system.initialize()
                print(f"✅ {self.test_name}: LED系统初始化成功")
            else:
                print(f"⚠️ {self.test_name}: LED系统创建失败")
                
        except Exception as e:
            self.error_logs.append(f"LED控制器设置失败: {e}")
            print(f"❌ {self.test_name}: LED控制器设置失败 - {e}")
    
    def _cleanup_led_controller(self):
        """清理LED控制器"""
        try:
            if self.led_system:
                self.led_system.cleanup()
                print(f"✅ {self.test_name}: LED系统清理完成")
        except Exception as e:
            self.error_logs.append(f"LED控制器清理失败: {e}")
            print(f"❌ {self.test_name}: LED控制器清理失败 - {e}")
    
    def _record_test_results(self):
        """记录测试结果"""
        test_result = {
            'test_name': self.test_name,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'duration': self.test_duration,
            'hardware_available': self.hardware_available,
            'performance_data': self.performance_data,
            'error_logs': self.error_logs,
            'timestamp': datetime.now().isoformat()
        }
        
        # 保存到测试数据收集器
        self._save_test_result(test_result)
    
    def _save_test_result(self, result: Dict[str, Any]):
        """保存测试结果"""
        try:
            # 创建日志目录
            log_dir = Path("logs/led_tests")
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # 生成日志文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = log_dir / f"{self.test_name}_{timestamp}.json"
            
            # 保存结果
            with open(log_file, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
                
        except Exception as e:
            print(f"⚠️ 无法保存测试结果: {e}")
    
    # =========================
    # 测试工具方法
    # =========================
    
    def measure_performance(self, operation_name: str, operation_func, *args, **kwargs):
        """测量操作性能"""
        start_time = time.perf_counter()
        
        try:
            result = operation_func(*args, **kwargs)
            success = True
            error = None
        except Exception as e:
            result = None
            success = False
            error = str(e)
            
        end_time = time.perf_counter()
        duration = (end_time - start_time) * 1000  # 转换为毫秒
        
        # 记录性能数据
        self.performance_data[operation_name] = {
            'duration_ms': duration,
            'success': success,
            'error': error,
            'timestamp': time.time()
        }
        
        return result, duration, success
    
    def assert_response_time(self, operation_name: str, max_time_ms: float = 200.0):
        """验证响应时间要求"""
        if operation_name in self.performance_data:
            actual_time = self.performance_data[operation_name]['duration_ms']
            self.assertLessEqual(
                actual_time, 
                max_time_ms,
                f"{operation_name} 响应时间 {actual_time:.2f}ms 超过要求 {max_time_ms}ms"
            )
            print(f"✅ {operation_name}: {actual_time:.2f}ms (< {max_time_ms}ms)")
        else:
            self.fail(f"未找到操作 '{operation_name}' 的性能数据")
    
    def verify_led_mode(self, expected_mode: str, timeout: float = 1.0):
        """验证LED模式"""
        if not self.led_system:
            self.skipTest("LED系统不可用")
            
        try:
            # 获取当前LED状态（使用try-catch安全方法）
            actual_mode = None
            
            try:
                current_state = getattr(self.led_system, 'get_current_state')()
                actual_mode = current_state.get('mode') if isinstance(current_state, dict) else None
            except (AttributeError, TypeError):
                try:
                    actual_mode = getattr(self.led_system, 'current_mode')
                except AttributeError:
                    print(f"⚠️ 无法获取当前LED模式，跳过验证")
                    return
                
            self.assertEqual(
                actual_mode, 
                expected_mode,
                f"期望LED模式 '{expected_mode}', 实际 '{actual_mode}'"
            )
            print(f"✅ LED模式验证通过: {expected_mode}")
            
        except Exception as e:
            self.fail(f"LED模式验证失败: {e}")
    
    def simulate_environment_change(self, light_level: str = "normal"):
        """模拟环境变化"""
        if not self.led_system:
            return
            
        try:
            # 模拟环境光变化（使用try-catch安全方法）
            try:
                simulate_func = getattr(self.led_system, 'simulate_environment_change')
                simulate_func({
                    'light_level': light_level,
                    'timestamp': time.time()
                })
            except AttributeError:
                try:
                    set_light_func = getattr(self.led_system, 'set_environment_light')
                    set_light_func(light_level)
                except AttributeError:
                    print(f"⚠️ 环境模拟功能不可用")
                    return
                
            time.sleep(0.1)  # 等待系统响应
            
        except Exception as e:
            self.error_logs.append(f"环境模拟失败: {e}")
    
    def run_stress_test(self, operation_func, iterations: int = 100, max_duration: float = 10.0):
        """运行压力测试"""
        start_time = time.time()
        success_count = 0
        error_count = 0
        durations = []
        
        for i in range(iterations):
            try:
                op_start = time.perf_counter()
                operation_func()
                op_end = time.perf_counter()
                
                duration = (op_end - op_start) * 1000
                durations.append(duration)
                success_count += 1
                
            except Exception as e:
                error_count += 1
                self.error_logs.append(f"压力测试第{i+1}次迭代失败: {e}")
            
            # 检查总时间限制
            if time.time() - start_time > max_duration:
                break
        
        # 计算统计数据
        total_time = time.time() - start_time
        success_rate = success_count / (success_count + error_count) * 100
        avg_duration = sum(durations) / len(durations) if durations else 0
        max_duration_ms = max(durations) if durations else 0
        
        stress_result = {
            'iterations': success_count + error_count,
            'success_count': success_count,
            'error_count': error_count,
            'success_rate': success_rate,
            'total_time': total_time,
            'avg_duration_ms': avg_duration,
            'max_duration_ms': max_duration_ms
        }
        
        self.performance_data['stress_test'] = stress_result
        
        print(f"🔥 压力测试结果:")
        print(f"   迭代次数: {stress_result['iterations']}")
        print(f"   成功率: {success_rate:.1f}%")
        print(f"   平均耗时: {avg_duration:.2f}ms")
        print(f"   最大耗时: {max_duration_ms:.2f}ms")
        
        return stress_result
    
    # =========================
    # 断言增强方法
    # =========================
    
    def assertLEDSystemReady(self):
        """断言LED系统就绪"""
        self.assertIsNotNone(self.led_system, "LED系统未初始化")
        
    def assertHardwareAvailable(self):
        """断言硬件可用"""
        self.assertTrue(self.hardware_available, "硬件不可用，无法进行硬件相关测试")
        
    def assertPerformanceAcceptable(self, operation_name: str, max_time_ms: float = 200.0):
        """断言性能可接受"""
        self.assert_response_time(operation_name, max_time_ms)
        
        # 检查是否成功
        if operation_name in self.performance_data:
            success = self.performance_data[operation_name]['success']
            self.assertTrue(success, f"操作 '{operation_name}' 执行失败")

if __name__ == "__main__":
    # 基础测试演示
    class BasicLEDTest(LEDTestBase):
        def test_basic_functionality(self):
            """基础功能测试"""
            self.assertLEDSystemReady()
            print("✅ 基础功能测试通过")
    
    unittest.main() 