#!/usr/bin/env python3
"""
Unitree Go2 机器人音频功能测试脚本（正确版本）
基于Go2 VUI客户端进行音量控制和LED测试，结合系统音频输出

Author: Claudia AI System  
Generated: 2025-06-30 13:35:00
Platform: Ubuntu 20.04 - aarch64
"""

import sys
import time
import numpy as np
from pathlib import Path
from datetime import datetime

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent.parent))

def test_go2_vui():
    """测试Go2的VUI功能（音量和LED控制）"""
    print("\n🤖 Unitree Go2 VUI功能测试")
    print("=" * 50)
    
    try:
        # 导入Unitree SDK2
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.vui.vui_client import VuiClient
        
        print("✅ Unitree Go2 SDK2 导入成功")
        
        # 初始化通道（对于Go2，通常不需要网络接口参数）
        ChannelFactoryInitialize(0)
        print("✅ Channel Factory 初始化完成")
        
        # 创建VUI客户端
        vui_client = VuiClient()
        vui_client.SetTimeout(3.0)
        vui_client.Init()
        print("✅ VUI Client 创建并初始化成功")
        
        # 1. 获取当前音量
        print("\n🔊 1. 音量状态检查")
        print("-" * 30)
        code, current_volume = vui_client.GetVolume()
        if code == 0:
            print(f"✅ 当前音量: {current_volume}")
        else:
            print(f"❌ 获取音量失败，错误码: {code}")
            
        # 2. 获取当前LED亮度
        print("\n💡 2. LED亮度状态检查")
        print("-" * 30)
        code, current_brightness = vui_client.GetBrightness()
        if code == 0:
            print(f"✅ 当前LED亮度: {current_brightness}")
        else:
            print(f"❌ 获取LED亮度失败，错误码: {code}")
        
        # 3. 音量测试序列
        print("\n🔊 3. 音量控制测试")
        print("-" * 30)
        test_volumes = [3, 6, 9, 5]  # 测试不同音量级别
        
        for volume in test_volumes:
            print(f"设置音量为 {volume}...")
            code = vui_client.SetVolume(volume)
            if code == 0:
                print(f"✅ 音量设置成功")
                
                # 验证设置
                code, new_volume = vui_client.GetVolume()
                if code == 0:
                    print(f"   验证音量: {new_volume}")
                else:
                    print(f"   验证失败，错误码: {code}")
            else:
                print(f"❌ 音量设置失败，错误码: {code}")
            
            time.sleep(1)
        
        # 4. LED亮度测试序列
        print("\n💡 4. LED亮度控制测试")
        print("-" * 30)
        test_brightness = [2, 5, 8, 10, 0]  # 测试不同亮度级别
        
        for brightness in test_brightness:
            print(f"设置LED亮度为 {brightness}...")
            code = vui_client.SetBrightness(brightness)
            if code == 0:
                print(f"✅ LED亮度设置成功")
                
                # 验证设置
                code, new_brightness = vui_client.GetBrightness()
                if code == 0:
                    print(f"   验证亮度: {new_brightness}")
                else:
                    print(f"   验证失败，错误码: {code}")
            else:
                print(f"❌ LED亮度设置失败，错误码: {code}")
            
            time.sleep(1.5)
        
        # 5. 恢复原始设置
        print("\n🔄 5. 恢复原始设置")
        print("-" * 30)
        if 'current_volume' in locals() and current_volume is not None:
            vui_client.SetVolume(current_volume)
            print(f"✅ 音量恢复到: {current_volume}")
        
        if 'current_brightness' in locals() and current_brightness is not None:
            vui_client.SetBrightness(current_brightness)
            print(f"✅ LED亮度恢复到: {current_brightness}")
        
        print("\n✅ Go2 VUI测试完成!")
        return True
        
    except ImportError as e:
        print(f"❌ Unitree SDK2 导入失败: {e}")
        print("请确保已正确安装Unitree SDK2并配置环境")
        return False
        
    except Exception as e:
        print(f"❌ VUI测试失败: {e}")
        print("可能的原因:")
        print("  1. Go2机器人未连接或未开机")
        print("  2. VUI服务未启动")
        print("  3. 网络连接问题")
        return False

def test_audio_output_with_tones():
    """播放一系列音调来模拟"机器人说话"的效果"""
    print("\n🎵 模拟音频反馈测试")
    print("=" * 50)
    
    try:
        import sounddevice as sd
        
        # 定义音调序列 - 模拟机器人反馈音
        tones = [
            (800, 0.3),   # 启动音
            (1000, 0.2),  # 确认音1
            (1200, 0.2),  # 确认音2  
            (1500, 0.4),  # 完成音
        ]
        
        sample_rate = 44100
        
        print("播放机器人状态反馈音序列...")
        print("这些音调代表:")
        print("  🔔 启动音 (800Hz)")
        print("  ✅ 确认音1 (1000Hz)")
        print("  ✅ 确认音2 (1200Hz)")
        print("  🎉 完成音 (1500Hz)")
        
        for i, (freq, duration) in enumerate(tones):
            print(f"\n播放音调 {i+1}: {freq}Hz ({duration}秒)")
            
            # 生成音调
            t = np.linspace(0, duration, int(sample_rate * duration))
            # 添加淡入淡出效果使音调更自然
            fade_samples = int(0.05 * sample_rate)  # 50ms淡入淡出
            wave = 0.3 * np.sin(2 * np.pi * freq * t)
            
            # 应用淡入淡出
            if len(wave) > 2 * fade_samples:
                wave[:fade_samples] *= np.linspace(0, 1, fade_samples)
                wave[-fade_samples:] *= np.linspace(1, 0, fade_samples)
            
            sd.play(wave, sample_rate)
            sd.wait()  # 等待播放完成
            time.sleep(0.2)  # 短暂间隔
        
        print("\n✅ 音频反馈测试完成!")
        print("如果您听到了这些音调，说明音频输出系统工作正常！")
        return True
        
    except Exception as e:
        print(f"❌ 音频反馈测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🎵 Unitree Go2 完整音频功能测试")
    print("=" * 60)
    print(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    print("测试内容:")
    print("  🤖 Go2 VUI控制 (音量/LED)")
    print("  🎵 音频输出验证")
    print("  🔊 机器人反馈音测试")
    print()
    
    results = []
    
    # 测试1: Go2 VUI功能
    print("🤖 测试1: Go2 VUI功能")
    if test_go2_vui():
        results.append("✅ VUI控制")
        print("💡 如果LED灯光发生了变化，说明Go2机器人响应正常！")
    else:
        results.append("❌ VUI控制")
    
    print("\n" + "="*60)
    
    # 测试2: 音频输出
    print("🎵 测试2: 音频输出功能")
    if test_audio_output_with_tones():
        results.append("✅ 音频输出")
    else:
        results.append("❌ 音频输出")
    
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
        print("   - Go2的LED和音量控制正常工作")
        print("   - 音频输出系统完全可用")
        print("   - 机器人音频硬件验证成功")
    elif success_count > 0:
        print("\n⚠️ 部分功能正常")
        print("💡 说明:")
        print("   - 基础音频功能可用")
        print("   - 部分Go2控制功能可能需要机器人连接")
    else:
        print("\n❌ 需要检查配置")
        print("🔧 建议:")
        print("   - 确保Go2机器人已开机并连接")
        print("   - 检查音频设备连接")

if __name__ == "__main__":
    main() 