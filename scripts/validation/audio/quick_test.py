#!/usr/bin/env python3
"""
Unitree Go2 音频I/O系统快速测试脚本
用于演示音频验证系统的基本功能

使用方法:
python3 quick_test.py

Author: Claudia AI System
Generated: 2025-06-30 13:12:00
Platform: Ubuntu 20.04 - aarch64
"""

import sys
import time
from pathlib import Path

# 添加当前目录到Python路径
sys.path.append(str(Path(__file__).parent))

try:
    from audio_validation_main import AudioValidationMain
    print("✅ 音频验证模块导入成功")
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    print("请确保所有依赖已安装: pip install sounddevice scipy librosa matplotlib numpy")
    sys.exit(1)

def quick_device_check():
    """快速检查音频设备"""
    print("\n🔍 快速音频设备检查")
    print("=" * 40)
    
    try:
        validator = AudioValidationMain()
        devices = validator.get_audio_devices()
        
        print(f"📥 输入设备: {len(devices['input_devices'])} 个")
        for device in devices['input_devices'][:3]:  # 显示前3个
            print(f"   - {device['name']} ({device['channels']} 通道)")
        
        print(f"📤 输出设备: {len(devices['output_devices'])} 个")
        for device in devices['output_devices'][:3]:  # 显示前3个
            print(f"   - {device['name']} ({device['channels']} 通道)")
        
        print(f"🎯 默认输入: {devices['default_input']}")
        print(f"🎯 默认输出: {devices['default_output']}")
        
        return True
    except Exception as e:
        print(f"❌ 设备检查失败: {e}")
        return False

def quick_recording_test():
    """快速录音测试"""
    print("\n🎤 快速录音测试 (3秒)")
    print("=" * 40)
    
    try:
        import sounddevice as sd
        import numpy as np
        
        # 录制3秒音频
        print("开始录音...")
        duration = 3.0
        sample_rate = 44100
        recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1)
        sd.wait()
        
        # 分析录音
        max_amplitude = np.max(np.abs(recording))
        rms_level = np.sqrt(np.mean(recording**2))
        
        print(f"✅ 录音完成")
        print(f"📊 最大振幅: {max_amplitude:.4f}")
        print(f"📊 RMS电平: {rms_level:.4f}")
        
        if max_amplitude > 0.001:
            print("🎉 录音测试通过 - 检测到音频信号")
            return True
        else:
            print("⚠️ 录音测试警告 - 信号较弱")
            return False
            
    except Exception as e:
        print(f"❌ 录音测试失败: {e}")
        return False

def quick_playback_test():
    """快速播放测试"""
    print("\n🔊 快速播放测试 (1kHz音调 2秒)")
    print("=" * 40)
    
    try:
        import sounddevice as sd
        import numpy as np
        
        # 生成1kHz正弦波
        duration = 2.0
        sample_rate = 44100
        frequency = 1000
        
        t = np.linspace(0, duration, int(duration * sample_rate), False)
        tone = 0.3 * np.sin(2 * np.pi * frequency * t)
        
        print(f"播放 {frequency}Hz 音调...")
        sd.play(tone, sample_rate)
        sd.wait()
        
        print("✅ 播放测试完成")
        return True
        
    except Exception as e:
        print(f"❌ 播放测试失败: {e}")
        return False

def main():
    """主函数"""
    print("🎵 Unitree Go2 音频I/O系统快速测试")
    print("=" * 50)
    print("时间:", time.strftime("%Y-%m-%d %H:%M:%S"))
    print("")
    
    # 测试计数器
    passed_tests = 0
    total_tests = 3
    
    # 1. 设备检查
    if quick_device_check():
        passed_tests += 1
    
    # 2. 录音测试
    if quick_recording_test():
        passed_tests += 1
    
    # 3. 播放测试
    if quick_playback_test():
        passed_tests += 1
    
    # 总结
    print("\n📊 测试总结")
    print("=" * 40)
    success_rate = passed_tests / total_tests
    print(f"通过测试: {passed_tests}/{total_tests} ({success_rate:.1%})")
    
    if success_rate >= 0.67:
        print("🎉 音频系统基本可用!")
        print("📋 建议运行完整验证:")
        print("   ./run_audio_validation.sh")
    else:
        print("⚠️ 音频系统存在问题")
        print("🔧 建议检查:")
        print("   - 音频设备连接")
        print("   - 系统权限设置")
        print("   - 驱动程序安装")
    
    print("\n💡 更多选项:")
    print("   - 完整验证: ./run_audio_validation.sh")
    print("   - 安装依赖: ./run_audio_validation.sh --install")
    print("   - 查看帮助: ./run_audio_validation.sh --help")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试过程发生错误: {e}")
        sys.exit(1) 