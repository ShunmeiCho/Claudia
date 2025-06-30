#!/usr/bin/env python3
"""
Unitree Go2 机器人语音合成(TTS)测试脚本
让机器人实际发出声音进行验证

基于Unitree SDK2 AudioClient API
包括TTS、音量控制、LED灯光联动测试

Author: Claudia AI System  
Generated: 2025-06-30 13:30:00
Platform: Ubuntu 20.04 - aarch64
"""

import sys
import time
import json
from pathlib import Path
from datetime import datetime

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent.parent))

def test_unitree_tts():
    """测试Unitree Go2的TTS功能"""
    print("\n🤖 Unitree Go2 TTS语音合成测试")
    print("=" * 50)
    
    try:
        # 导入Unitree SDK2
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.audio.audio_client import AudioClient
        
        print("✅ Unitree SDK2 导入成功")
        
        # 初始化通道
        ChannelFactoryInitialize(0, "")
        print("✅ Channel Factory 初始化完成")
        
        # 创建音频客户端
        audio_client = AudioClient()
        print("✅ AudioClient 创建成功")
        
        # 1. 获取当前音量
        print("\n🔊 1. 音量状态检查")
        print("-" * 30)
        volume_result = audio_client.GetVolume()
        print(f"当前音量: {volume_result}")
        
        # 2. 设置合适的音量
        print("\n🔊 2. 设置音量为80")
        print("-" * 30)
        audio_client.SetVolume(80)
        time.sleep(0.5)
        
        # 验证音量设置
        new_volume = audio_client.GetVolume()
        print(f"新音量: {new_volume}")
        
        # 3. TTS测试 - 中文
        print("\n🗣️ 3. 中文TTS测试")
        print("-" * 30)
        print("机器人即将说话: '你好！我是Unitree Go2机器人，音频系统验证成功！'")
        
        tts_result = audio_client.TtsMaker("你好！我是Unitree Go2机器人，音频系统验证成功！", 0)
        print(f"TTS结果: {tts_result}")
        time.sleep(3)  # 等待播放完成
        
        # 4. TTS测试 - 英文
        print("\n🗣️ 4. 英文TTS测试") 
        print("-" * 30)
        print("机器人即将说话: 'Hello! I am Unitree Go2 robot. Audio validation successful!'")
        
        tts_result = audio_client.TtsMaker("Hello! I am Unitree Go2 robot. Audio validation successful!", 0)
        print(f"TTS结果: {tts_result}")
        time.sleep(3)
        
        # 5. LED灯光联动测试
        print("\n💡 5. LED灯光联动测试")
        print("-" * 30)
        
        colors = [
            (255, 0, 0, "红色"),
            (0, 255, 0, "绿色"), 
            (0, 0, 255, "蓝色"),
            (255, 255, 0, "黄色"),
            (255, 0, 255, "紫色")
        ]
        
        for r, g, b, color_name in colors:
            print(f"设置LED为{color_name}...")
            audio_client.LedControl(r, g, b)
            audio_client.TtsMaker(f"当前LED灯光为{color_name}", 0)
            time.sleep(2)
        
        # 6. 完成提示
        print("\n🎉 6. 测试完成提示")
        print("-" * 30)
        audio_client.LedControl(0, 255, 0)  # 绿色表示成功
        audio_client.TtsMaker("音频验证测试全部完成！所有功能正常工作！", 0)
        time.sleep(3)
        
        # 关闭LED
        audio_client.LedControl(0, 0, 0)
        
        print("\n✅ TTS测试完成!")
        print("如果您听到了机器人的声音，说明音频输出功能正常！")
        
        return True
        
    except ImportError as e:
        print(f"❌ Unitree SDK2 导入失败: {e}")
        print("请确保已正确安装Unitree SDK2并配置环境")
        return False
        
    except Exception as e:
        print(f"❌ TTS测试失败: {e}")
        print("可能的原因:")
        print("  1. 机器人未连接或未开机")
        print("  2. 网络连接问题")
        print("  3. AudioClient服务未启动")
        return False

def test_basic_audio_output():
    """测试基础音频输出功能（备选方案）"""
    print("\n🔊 基础音频输出测试")
    print("=" * 50)
    
    try:
        import sounddevice as sd
        import numpy as np
        
        # 生成测试音调
        duration = 2  # 秒
        sample_rate = 44100
        frequency = 1000  # 1kHz
        
        t = np.linspace(0, duration, int(sample_rate * duration))
        wave = 0.3 * np.sin(2 * np.pi * frequency * t)
        
        print(f"播放 {frequency}Hz 测试音调 ({duration}秒)...")
        print("您应该能听到一个纯音调...")
        
        sd.play(wave, sample_rate)
        sd.wait()  # 等待播放完成
        
        print("✅ 基础音频输出测试完成")
        return True
        
    except Exception as e:
        print(f"❌ 基础音频输出测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🎵 Unitree Go2 实际音频输出测试")
    print("=" * 60)
    print(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # 首先尝试Unitree TTS
    print("🤖 尝试方案1: Unitree AudioClient TTS")
    if test_unitree_tts():
        print("\n🎉 Unitree TTS测试成功！您应该听到了机器人的声音。")
    else:
        print("\n⚠️ Unitree TTS测试失败，尝试备选方案...")
        
        # 备选方案：基础音频输出
        print("\n🔊 尝试方案2: 基础音频输出")
        if test_basic_audio_output():
            print("\n✅ 基础音频输出成功！")
            print("💡 建议检查:")
            print("   - 确保Go2机器人已开机并连接")
            print("   - 检查网络连接状态") 
            print("   - 验证AudioClient服务是否启动")
        else:
            print("\n❌ 所有音频输出测试都失败了")
            print("🔧 建议检查:")
            print("   - 音频设备连接")
            print("   - 扬声器音量设置")
            print("   - 音频驱动程序状态")

if __name__ == "__main__":
    main() 