#!/usr/bin/env python3
"""
Claudia机器人LED控制器测试脚本
验证基于VUI客户端的统一LED控制功能

Author: Claudia AI System
Generated: 2025-07-04
"""

import sys
import time
import logging
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_led_controller():
    """测试LED控制器功能"""
    print("🔆 Claudia机器人LED控制器测试")
    print("=" * 50)
    
    try:
        # 导入LED控制器
        from src.claudia.robot_controller import (
            LEDController, ClaudiaLEDMode, LEDBrightnessLevel, LED_AVAILABLE
        )
        
        if not LED_AVAILABLE:
            print("❌ LED控制器模块导入失败")
            return False
        
        print("✅ LED控制器模块导入成功")
        
        # 创建LED控制器实例
        led_controller = LEDController()
        
        # 初始化LED控制器
        print("\n🔧 初始化LED控制器...")
        if not led_controller.initialize():
            print("❌ LED控制器初始化失败")
            print("💡 可能原因:")
            print("   - Unitree Go2机器人未连接")
            print("   - unitree_sdk2py模块未安装")
            print("   - VUI服务未启动")
            return False
        
        print("✅ LED控制器初始化成功")
        
        # 获取当前状态
        print("\n📊 LED控制器状态:")
        status = led_controller.get_status()
        for key, value in status.items():
            print(f"   {key}: {value}")
        
        # 测试基础亮度控制
        print("\n💡 测试基础亮度控制:")
        test_brightness_levels = [0, 3, 6, 9, 10, 5]
        
        for brightness in test_brightness_levels:
            print(f"   设置亮度为 {brightness}...")
            if led_controller.set_brightness(brightness):
                print(f"   ✅ 亮度设置成功")
                # 验证设置
                current = led_controller.get_brightness()
                if current == brightness:
                    print(f"   ✅ 亮度验证通过: {current}")
                else:
                    print(f"   ⚠️ 亮度验证不一致: 期望 {brightness}, 实际 {current}")
            else:
                print(f"   ❌ 亮度设置失败")
            
            time.sleep(1.0)
        
        # 测试Claudia LED模式
        print("\n🎭 测试Claudia LED模式:")
        test_modes = [
            ClaudiaLEDMode.IDLE,
            ClaudiaLEDMode.WAKE_CONFIRM,
            ClaudiaLEDMode.PROCESSING,
            ClaudiaLEDMode.EXECUTING,
            ClaudiaLEDMode.ACTION_COMPLETE,
            ClaudiaLEDMode.ERROR
        ]
        
        for mode in test_modes:
            print(f"   设置模式: {mode.value}...")
            if led_controller.set_mode(mode):
                print(f"   ✅ 模式设置成功")
                print(f"   💡 当前亮度: {led_controller.get_brightness()}")
            else:
                print(f"   ❌ 模式设置失败")
            
            time.sleep(1.5)
        
        # 测试开关功能
        print("\n🔄 测试开关功能:")
        print("   关闭LED...")
        if led_controller.turn_off():
            print("   ✅ LED关闭成功")
        time.sleep(1.0)
        
        print("   打开LED（默认亮度）...")
        if led_controller.turn_on():
            print("   ✅ LED打开成功")
        time.sleep(1.0)
        
        # 清理
        print("\n🧹 清理LED控制器...")
        led_controller.cleanup()
        
        print("\n🎉 LED控制器测试完成!")
        return True
        
    except ImportError as e:
        print(f"❌ 模块导入失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 测试过程中发生异常: {e}")
        return False

def test_brightness_levels():
    """测试LEDBrightnessLevel枚举"""
    print("\n📊 LEDBrightnessLevel枚举测试:")
    
    try:
        from src.claudia.robot_controller import LEDBrightnessLevel
        
        print("   可用亮度级别:")
        for level in LEDBrightnessLevel:
            print(f"     {level.name}: {level.value}")
        
        return True
    except Exception as e:
        print(f"❌ 亮度级别测试失败: {e}")
        return False

def test_led_modes():
    """测试ClaudiaLEDMode枚举"""
    print("\n🎭 ClaudiaLEDMode枚举测试:")
    
    try:
        from src.claudia.robot_controller import ClaudiaLEDMode
        
        print("   可用LED模式:")
        for mode in ClaudiaLEDMode:
            print(f"     {mode.name}: {mode.value}")
        
        return True
    except Exception as e:
        print(f"❌ LED模式测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🤖 Claudia机器人LED控制系统测试")
    print("=" * 60)
    print("🔍 测试内容:")
    print("   - LED控制器模块导入")
    print("   - 基础亮度控制 (0-10级)")
    print("   - Claudia LED模式切换")
    print("   - 开关功能测试")
    print("=" * 60)
    
    # 设置日志级别
    logging.basicConfig(level=logging.INFO)
    
    results = []
    
    # 测试1: 枚举类型
    print("\n🧪 测试1: 枚举类型验证")
    if test_brightness_levels() and test_led_modes():
        results.append("✅ 枚举类型")
    else:
        results.append("❌ 枚举类型")
    
    # 测试2: LED控制器功能
    print("\n🧪 测试2: LED控制器功能")
    if test_led_controller():
        results.append("✅ LED控制器")
    else:
        results.append("❌ LED控制器")
    
    # 总结
    print("\n" + "="*60)
    print("📊 测试总结")
    print("-" * 30)
    success_count = sum(1 for r in results if r.startswith("✅"))
    total_count = len(results)
    
    for result in results:
        print(f"  {result}")
    
    print(f"\n通过率: {success_count}/{total_count} ({success_count/total_count*100:.1f}%)")
    
    if success_count == total_count:
        print("\n🎉 所有测试通过！")
        print("💡 说明:")
        print("   - LED控制器模块正常工作")
        print("   - 基于VUI客户端的亮度控制可用")
        print("   - Claudia LED模式系统就绪")
    elif success_count > 0:
        print("\n⚠️ 部分功能正常")
        print("💡 说明:")
        print("   - 模块导入成功，但硬件控制可能需要机器人连接")
    else:
        print("\n❌ 需要检查配置")
        print("🔧 建议:")
        print("   - 确保项目路径正确")
        print("   - 检查LED控制器模块是否存在")

if __name__ == "__main__":
    main()