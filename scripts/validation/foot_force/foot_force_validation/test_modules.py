#!/usr/bin/env python3
# test_modules.py - 模块功能测试

import os
import sys
import json
import logging

def test_module_imports():
    """测试模块导入"""
    print("🧪 测试静态验证框架模块导入...")
    
    failures = []
    
    # 测试基础模块（不依赖SDK）
    try:
        import numpy as np
        print("✅ numpy 导入成功")
    except ImportError as e:
        print(f"❌ numpy 导入失败: {e}")
        failures.append("numpy")
    
    try:
        import matplotlib.pyplot as plt
        print("✅ matplotlib 导入成功")
    except ImportError as e:
        print(f"❌ matplotlib 导入失败: {e}")
        failures.append("matplotlib")
    
    try:
        import scipy
        print("✅ scipy 导入成功")
    except ImportError as e:
        print(f"❌ scipy 导入失败: {e}")
        failures.append("scipy")
    
    try:
        import pandas as pd
        print("✅ pandas 导入成功")
    except ImportError as e:
        print(f"❌ pandas 导入失败: {e}")
        failures.append("pandas")
    
    return len(failures) == 0

def test_configuration():
    """测试配置文件"""
    print("\n🧪 测试配置文件...")
    
    try:
        with open('validation_config.json', 'r') as f:
            config = json.load(f)
        
        # 检查基本配置项
        required_keys = ['foot_force_config', 'data_collection', 'static_validation']
        
        for key in required_keys:
            if key in config:
                print(f"✅ 配置项 {key} 存在")
            else:
                print(f"❌ 配置项 {key} 缺失")
                return False
        
        return True
        
    except Exception as e:
        print(f"❌ 配置文件测试失败: {e}")
        return False

def test_file_structure():
    """测试文件结构"""
    print("\n🧪 测试文件结构...")
    
    required_files = [
        'static_tester.py',
        'analyzer.py', 
        'visualizer.py',
        'static_validation.py',
        'foot_force_config.py',
        'data_collector.py',
        'validation_config.json',
        'README.md'
    ]
    
    missing_files = []
    
    for file in required_files:
        if os.path.exists(file):
            print(f"✅ {file} 存在")
        else:
            print(f"❌ {file} 缺失")
            missing_files.append(file)
    
    return len(missing_files) == 0

def test_directory_structure():
    """测试目录结构"""
    print("\n🧪 测试目录结构...")
    
    required_dirs = ['logs', 'output']
    
    missing_dirs = []
    
    for dir_name in required_dirs:
        if os.path.exists(dir_name):
            print(f"✅ 目录 {dir_name}/ 存在")
        else:
            print(f"❌ 目录 {dir_name}/ 缺失")
            missing_dirs.append(dir_name)
    
    return len(missing_dirs) == 0

def test_script_syntax():
    """测试脚本语法"""
    print("\n🧪 测试脚本语法...")
    
    python_files = [
        'static_tester.py',
        'analyzer.py',
        'visualizer.py', 
        'static_validation.py'
    ]
    
    syntax_errors = []
    
    for file in python_files:
        try:
            with open(file, 'r') as f:
                code = f.read()
            
            compile(code, file, 'exec')
            print(f"✅ {file} 语法正确")
            
        except SyntaxError as e:
            print(f"❌ {file} 语法错误: {e}")
            syntax_errors.append(file)
        except Exception as e:
            print(f"⚠️ {file} 检查时出现问题: {e}")
    
    return len(syntax_errors) == 0

def main():
    """主测试函数"""
    print("=" * 60)
    print("🔧 Unitree Go2 足端力传感器静态验证框架")
    print("📋 模块功能测试")
    print("=" * 60)
    
    tests = [
        ("基础依赖模块", test_module_imports),
        ("配置文件", test_configuration),
        ("文件结构", test_file_structure),
        ("目录结构", test_directory_structure),
        ("脚本语法", test_script_syntax)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append(result)
        except Exception as e:
            print(f"\n❌ 测试 {test_name} 执行失败: {e}")
            results.append(False)
    
    # 总结
    print("\n" + "=" * 60)
    print("📊 测试结果总结")
    print("=" * 60)
    
    passed = sum(results)
    total = len(results)
    
    for i, (test_name, _) in enumerate(tests):
        status = "✅ PASS" if results[i] else "❌ FAIL"
        print(f"{status} {test_name}")
    
    print(f"\n📈 总体结果: {passed}/{total} 项测试通过")
    
    if passed == total:
        print("🎉 所有测试通过！静态验证框架准备就绪。")
        
        print("\n📋 使用说明:")
        print("1. 运行交互式验证: ../run_static_validation.sh")
        print("2. 运行快速测试: python3 static_validation.py --test-mode")
        print("3. 查看帮助信息: python3 static_validation.py --help")
        print("4. 请注意: 需要Unitree SDK正常工作才能进行实际验证")
        
    else:
        print("⚠️ 部分测试失败，请检查并修复问题。")
    
    return passed == total

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 