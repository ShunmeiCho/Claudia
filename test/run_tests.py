#!/usr/bin/env python3
"""
Claudia机器人项目测试运行器

提供统一的测试运行接口，支持不同类型的测试和配置选项。
"""

import sys
import os
import argparse
import subprocess
import time
from pathlib import Path
from typing import List, Optional

# 添加项目根目录到Python路径
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

class TestRunner:
    """测试运行器"""
    
    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.project_root = self.test_dir.parent
        self.results = {}
    
    def run_hardware_tests(self, verbose: bool = False) -> bool:
        """运行硬件测试"""
        print("🤖 运行硬件测试...")
        
        hardware_tests = [
            "test_unitree_connection.py",
            # 未来添加: "test_ros2_communication.py", "test_sensors.py"
        ]
        
        success = True
        for test_file in hardware_tests:
            test_path = self.test_dir / "hardware" / test_file
            if test_path.exists():
                print(f"  🔧 运行 {test_file}...")
                result = self._run_single_test(test_path, verbose)
                self.results[f"hardware/{test_file}"] = result
                if not result:
                    success = False
                    print(f"    ❌ {test_file} 失败")
                else:
                    print(f"    ✅ {test_file} 通过")
            else:
                print(f"  ⚠️ {test_file} 不存在，跳过")
        
        return success
    
    def run_unit_tests(self, verbose: bool = False) -> bool:
        """运行单元测试"""
        print("⚡ 运行单元测试...")
        
        unit_test_files = list((self.test_dir / "unit").glob("test_*.py"))
        if not unit_test_files:
            print("  ℹ️ 暂无单元测试文件")
            return True
        
        success = True
        for test_file in unit_test_files:
            print(f"  🔍 运行 {test_file.name}...")
            result = self._run_single_test(test_file, verbose)
            self.results[f"unit/{test_file.name}"] = result
            if not result:
                success = False
                print(f"    ❌ {test_file.name} 失败")
            else:
                print(f"    ✅ {test_file.name} 通过")
        
        return success
    
    def run_integration_tests(self, verbose: bool = False) -> bool:
        """运行集成测试"""
        print("🔗 运行集成测试...")
        
        integration_test_files = list((self.test_dir / "integration").glob("test_*.py"))
        if not integration_test_files:
            print("  ℹ️ 暂无集成测试文件")
            return True
        
        success = True
        for test_file in integration_test_files:
            print(f"  🌐 运行 {test_file.name}...")
            result = self._run_single_test(test_file, verbose)
            self.results[f"integration/{test_file.name}"] = result
            if not result:
                success = False
                print(f"    ❌ {test_file.name} 失败")
            else:
                print(f"    ✅ {test_file.name} 通过")
        
        return success
    
    def _run_single_test(self, test_path: Path, verbose: bool = False) -> bool:
        """运行单个测试文件"""
        try:
            cmd = [sys.executable, str(test_path)]
            if verbose:
                result = subprocess.run(cmd, cwd=self.project_root, 
                                      capture_output=False, text=True)
            else:
                result = subprocess.run(cmd, cwd=self.project_root,
                                      capture_output=True, text=True)
            
            return result.returncode == 0
            
        except Exception as e:
            print(f"    🚫 运行 {test_path.name} 时出错: {e}")
            return False
    
    def print_summary(self):
        """打印测试结果摘要"""
        print("\n" + "="*60)
        print("📊 测试结果摘要")
        print("="*60)
        
        total_tests = len(self.results)
        passed_tests = sum(1 for result in self.results.values() if result)
        failed_tests = total_tests - passed_tests
        
        print(f"总测试数: {total_tests}")
        print(f"通过: {passed_tests} ✅")
        print(f"失败: {failed_tests} ❌")
        
        if failed_tests > 0:
            print("\n失败的测试:")
            for test_name, result in self.results.items():
                if not result:
                    print(f"  ❌ {test_name}")
        
        print("="*60)
        return failed_tests == 0

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="Claudia机器人项目测试运行器")
    parser.add_argument("--type", choices=["all", "unit", "integration", "hardware"],
                       default="all", help="运行特定类型的测试")
    parser.add_argument("-v", "--verbose", action="store_true",
                       help="显示详细输出")
    parser.add_argument("--debug", action="store_true",
                       help="调试模式")
    
    args = parser.parse_args()
    
    print("🚀 Claudia机器人项目测试运行器")
    print(f"⏰ 开始时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"🎯 测试类型: {args.type}")
    print("-" * 60)
    
    runner = TestRunner()
    overall_success = True
    
    try:
        if args.type in ["all", "unit"]:
            success = runner.run_unit_tests(args.verbose)
            overall_success = overall_success and success
        
        if args.type in ["all", "integration"]:
            success = runner.run_integration_tests(args.verbose)
            overall_success = overall_success and success
        
        if args.type in ["all", "hardware"]:
            print("\n⚠️ 硬件测试需要机器人连接，确保:")
            print("   1. Go2机器人已开机并连接")
            print("   2. 网络配置正确")
            print("   3. CycloneDDS环境已设置")
            
            if args.debug or input("\n继续运行硬件测试? (y/N): ").lower() == 'y':
                success = runner.run_hardware_tests(args.verbose)
                overall_success = overall_success and success
            else:
                print("⏭️ 跳过硬件测试")
        
    except KeyboardInterrupt:
        print("\n\n⏹️ 测试被用户中断")
        return 1
    except Exception as e:
        print(f"\n❌ 测试运行出错: {e}")
        return 1
    
    # 打印摘要
    final_success = runner.print_summary()
    
    print(f"⏰ 结束时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    return 0 if final_success else 1

if __name__ == "__main__":
    exit(main()) 