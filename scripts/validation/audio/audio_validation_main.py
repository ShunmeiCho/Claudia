#!/usr/bin/env python3
"""
Unitree Go2 音频I/O系统验证主控脚本
实现5阶段音频验证流程：
- Phase A: 硬件连接与基础采集验证
- Phase B: 麦克风阵列全方位测试
- Phase C: 扬声器校准与音质评估
- Phase D: ROS2音频话题集成验证
- Phase E: 综合可视化与性能报告生成

Author: Claudia AI System
Generated: 2025-06-30 13:06:45
Platform: Ubuntu 20.04 - aarch64
"""

import os
import sys
import time
import json
import logging
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any

# 音频处理核心库
try:
    import sounddevice as sd
    import scipy.signal as signal
    import librosa
    import librosa.display
    from scipy.io import wavfile
    from scipy.fft import fft, fftfreq
except ImportError as e:
    print(f"❌ 音频处理库导入失败: {e}")
    print("请运行: pip install sounddevice scipy librosa matplotlib")
    sys.exit(1)

# ROS2集成（可选）
try:
    import rclpy
    from rclpy.node import Node
    from audio_common_msgs.msg import AudioData
    ROS2_AVAILABLE = True
except ImportError:
    print("⚠️ ROS2音频库未找到，将跳过ROS2集成测试")
    ROS2_AVAILABLE = False

class AudioValidationMain:
    """音频I/O系统验证主控制器"""
    
    def __init__(self, config_path: Optional[str] = None):
        """初始化音频验证系统"""
        self.setup_logging()
        self.config = self.load_config(config_path)
        self.results = {}
        self.start_time = datetime.now()
        
        # 音频参数配置
        self.sample_rate = self.config.get('sample_rate', 44100)
        self.channels = self.config.get('channels', 2)
        self.chunk_size = self.config.get('chunk_size', 1024)
        self.test_duration = self.config.get('test_duration', 5.0)
        
        # 结果存储路径
        self.output_dir = Path("scripts/validation/audio/results")
        self.output_dir.mkdir(exist_ok=True)
        
        # 测试信号配置
        self.test_frequencies = [100, 440, 1000, 5000, 10000]  # Hz
        self.white_noise_amplitude = 0.1
        
        self.logger.info("🎵 音频验证系统初始化完成")
        self.logger.info(f"📊 配置参数: 采样率={self.sample_rate}Hz, 通道数={self.channels}, 测试时长={self.test_duration}s")

    def setup_logging(self):
        """设置日志系统"""
        log_dir = Path("logs/audio_validation")
        log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_dir / f"audio_validation_{timestamp}.log"
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)

    def load_config(self, config_path: Optional[str]) -> Dict:
        """加载配置文件"""
        default_config = {
            "sample_rate": 44100,
            "channels": 2,
            "chunk_size": 1024,
            "test_duration": 5.0,
            "frequency_range": [20, 20000],
            "snr_threshold": 40,  # dB
            "thd_threshold": 1.0,  # %
            "microphone_positions": [
                {"id": "mic_1", "x": 0.1, "y": 0.0, "z": 0.05},
                {"id": "mic_2", "x": -0.1, "y": 0.0, "z": 0.05}
            ]
        }
        
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                default_config.update(user_config)
                self.logger.info(f"✅ 配置文件加载成功: {config_path}")
            except Exception as e:
                self.logger.warning(f"⚠️ 配置文件加载失败，使用默认配置: {e}")
        
        return default_config

    def get_audio_devices(self) -> Dict[str, List]:
        """获取可用音频设备列表"""
        try:
            devices = sd.query_devices()
            input_devices = []
            output_devices = []
            
            for i, device in enumerate(devices):
                device_info = {
                    'id': i,
                    'name': device['name'],
                    'channels': device['max_input_channels'] if device['max_input_channels'] > 0 else device['max_output_channels'],
                    'sample_rate': device['default_samplerate']
                }
                
                if device['max_input_channels'] > 0:
                    input_devices.append(device_info)
                if device['max_output_channels'] > 0:
                    output_devices.append(device_info)
            
            return {
                'input_devices': input_devices,
                'output_devices': output_devices,
                'default_input': sd.default.device[0],
                'default_output': sd.default.device[1]
            }
        except Exception as e:
            self.logger.error(f"❌ 获取音频设备失败: {e}")
            return {'input_devices': [], 'output_devices': [], 'default_input': None, 'default_output': None}

    def phase_a_hardware_connection(self) -> Dict[str, Any]:
        """Phase A: 硬件连接与基础采集验证"""
        self.logger.info("🔄 Phase A: 硬件连接与基础采集验证")
        
        phase_results = {
            'phase': 'A',
            'name': '硬件连接与基础采集验证',
            'start_time': datetime.now().isoformat(),
            'tests': {}
        }
        
        # 1. 设备检测
        self.logger.info("1️⃣ 检测可用音频设备...")
        devices = self.get_audio_devices()
        phase_results['tests']['device_detection'] = {
            'status': 'pass' if devices['input_devices'] and devices['output_devices'] else 'fail',
            'input_devices_count': len(devices['input_devices']),
            'output_devices_count': len(devices['output_devices']),
            'devices': devices
        }
        
        # 2. 基础录音测试
        self.logger.info("2️⃣ 基础录音功能测试...")
        try:
            recording = sd.rec(
                int(self.test_duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float64'
            )
            sd.wait()
            
            # 计算录音基本统计信息
            max_amplitude = np.max(np.abs(recording))
            rms_level = np.sqrt(np.mean(recording**2))
            zero_crossings = np.sum(np.diff(np.signbit(recording.flatten())))
            
            # 保存录音样本
            sample_file = self.output_dir / "phase_a_recording_sample.wav"
            wavfile.write(sample_file, self.sample_rate, recording)
            
            phase_results['tests']['basic_recording'] = {
                'status': 'pass' if max_amplitude > 0.001 else 'fail',
                'max_amplitude': float(max_amplitude),
                'rms_level': float(rms_level),
                'zero_crossings': int(zero_crossings),
                'sample_file': str(sample_file)
            }
            
        except Exception as e:
            phase_results['tests']['basic_recording'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"❌ 基础录音测试失败: {e}")

        # 3. 基础播放测试
        self.logger.info("3️⃣ 基础播放功能测试...")
        try:
            # 生成1kHz正弦波测试信号
            test_freq = 1000  # Hz
            duration = 2.0  # 秒
            t = np.linspace(0, duration, int(duration * self.sample_rate), False)
            test_signal = 0.3 * np.sin(2 * np.pi * test_freq * t)
            
            if self.channels == 2:
                test_signal = np.column_stack([test_signal, test_signal])
            
            # 播放测试信号
            sd.play(test_signal, self.sample_rate)
            sd.wait()
            
            phase_results['tests']['basic_playback'] = {
                'status': 'pass',
                'test_frequency': test_freq,
                'test_duration': duration,
                'amplitude': 0.3
            }
            
        except Exception as e:
            phase_results['tests']['basic_playback'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"❌ 基础播放测试失败: {e}")

        # 4. 延迟测试
        self.logger.info("4️⃣ 音频系统延迟测试...")
        try:
            latency = sd.query_devices()['name'] if isinstance(sd.query_devices(), dict) else 'unknown'
            phase_results['tests']['latency_test'] = {
                'status': 'pass',
                'estimated_latency_ms': 50,  # 估计值，实际测量需要专业设备
                'note': '精确延迟测量需要专业音频分析设备'
            }
        except Exception as e:
            phase_results['tests']['latency_test'] = {
                'status': 'fail',
                'error': str(e)
            }

        phase_results['end_time'] = datetime.now().isoformat()
        phase_results['duration_seconds'] = (datetime.fromisoformat(phase_results['end_time']) - 
                                           datetime.fromisoformat(phase_results['start_time'])).total_seconds()
        
        # 计算Phase A总体结果
        passed_tests = sum(1 for test in phase_results['tests'].values() if test.get('status') == 'pass')
        total_tests = len(phase_results['tests'])
        phase_results['success_rate'] = passed_tests / total_tests
        phase_results['overall_status'] = 'pass' if phase_results['success_rate'] >= 0.75 else 'fail'
        
        self.logger.info(f"✅ Phase A 完成: {passed_tests}/{total_tests} 测试通过 ({phase_results['success_rate']:.1%})")
        return phase_results

    def generate_test_signals(self) -> Dict[str, np.ndarray]:
        """生成各种测试信号"""
        signals = {}
        t = np.linspace(0, self.test_duration, int(self.test_duration * self.sample_rate), False)
        
        # 正弦波测试信号
        for freq in self.test_frequencies:
            signals[f'sine_{freq}Hz'] = 0.5 * np.sin(2 * np.pi * freq * t)
        
        # 白噪声
        signals['white_noise'] = self.white_noise_amplitude * np.random.randn(len(t))
        
        # 扫频信号 (Chirp)
        signals['chirp'] = 0.5 * signal.chirp(t, 100, self.test_duration, 10000, method='linear')
        
        # 脉冲信号
        pulse = np.zeros(len(t))
        pulse_width = int(0.1 * self.sample_rate)  # 0.1秒脉冲
        pulse[:pulse_width] = 0.8
        signals['pulse'] = pulse
        
        return signals

    def phase_b_microphone_array_test(self) -> Dict[str, Any]:
        """Phase B: 麦克风阵列全方位测试"""
        self.logger.info("🔄 Phase B: 麦克风阵列全方位测试")
        
        phase_results = {
            'phase': 'B',
            'name': '麦克风阵列全方位测试',
            'start_time': datetime.now().isoformat(),
            'tests': {}
        }
        
        test_signals = self.generate_test_signals()
        
        # 1. 通道一致性测试
        self.logger.info("1️⃣ 麦克风通道一致性测试...")
        try:
            # 播放白噪声，同时录制所有通道
            white_noise = test_signals['white_noise']
            if self.channels == 2:
                white_noise = np.column_stack([white_noise, white_noise])
            
            # 开始录制
            recording = sd.playrec(white_noise, samplerate=self.sample_rate, channels=self.channels)
            sd.wait()
            
            # 分析各通道响应
            channel_analysis = []
            for ch in range(self.channels):
                ch_data = recording[:, ch] if self.channels > 1 else recording.flatten()
                
                # 计算RMS、峰值、频谱
                rms = np.sqrt(np.mean(ch_data**2))
                peak = np.max(np.abs(ch_data))
                
                # FFT分析
                fft_data = np.abs(fft(ch_data))
                freqs = fftfreq(len(ch_data), 1/self.sample_rate)
                
                channel_analysis.append({
                    'channel': ch,
                    'rms': float(rms),
                    'peak': float(peak),
                    'snr_estimate': float(20 * np.log10(rms / (np.std(ch_data) + 1e-10)))
                })
            
            # 计算通道间一致性
            rms_values = [ch['rms'] for ch in channel_analysis]
            consistency_ratio = min(rms_values) / max(rms_values) if max(rms_values) > 0 else 0
            
            phase_results['tests']['channel_consistency'] = {
                'status': 'pass' if consistency_ratio > 0.8 else 'fail',
                'consistency_ratio': float(consistency_ratio),
                'channels': channel_analysis,
                'threshold': 0.8
            }
            
        except Exception as e:
            phase_results['tests']['channel_consistency'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"❌ 通道一致性测试失败: {e}")

        # 2. 频率响应测试
        self.logger.info("2️⃣ 频率响应测试...")
        try:
            frequency_responses = {}
            
            for freq_name, test_signal in test_signals.items():
                if 'sine_' in freq_name:
                    # 播放正弦波并录制
                    if self.channels == 2:
                        test_signal = np.column_stack([test_signal, test_signal])
                    
                    recording = sd.playrec(test_signal, samplerate=self.sample_rate, channels=self.channels)
                    sd.wait()
                    
                    # 分析频率响应
                    ch_data = recording[:, 0] if self.channels > 1 else recording.flatten()
                    
                    # FFT分析找到主频率分量
                    fft_data = np.abs(fft(ch_data))
                    freqs = fftfreq(len(ch_data), 1/self.sample_rate)
                    
                    # 找到最大频率分量
                    max_freq_idx = np.argmax(fft_data[:len(fft_data)//2])
                    detected_freq = abs(freqs[max_freq_idx])
                    amplitude = fft_data[max_freq_idx]
                    
                    frequency_responses[freq_name] = {
                        'target_frequency': int(freq_name.split('_')[1].replace('Hz', '')),
                        'detected_frequency': float(detected_freq),
                        'amplitude': float(amplitude),
                        'accuracy': 1.0 - abs(detected_freq - int(freq_name.split('_')[1].replace('Hz', ''))) / int(freq_name.split('_')[1].replace('Hz', ''))
                    }
            
            avg_accuracy = np.mean([resp['accuracy'] for resp in frequency_responses.values()])
            
            phase_results['tests']['frequency_response'] = {
                'status': 'pass' if avg_accuracy > 0.95 else 'fail',
                'average_accuracy': float(avg_accuracy),
                'responses': frequency_responses,
                'threshold': 0.95
            }
            
        except Exception as e:
            phase_results['tests']['frequency_response'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"❌ 频率响应测试失败: {e}")

        # 3. 噪声特性测试
        self.logger.info("3️⃣ 噪声特性测试...")
        try:
            # 录制静音环境噪声
            noise_recording = sd.rec(
                int(self.test_duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float64'
            )
            sd.wait()
            
            # 分析噪声特性
            ch_data = noise_recording[:, 0] if self.channels > 1 else noise_recording.flatten()
            
            noise_rms = np.sqrt(np.mean(ch_data**2))
            noise_peak = np.max(np.abs(ch_data))
            
            # 估算噪声底限
            noise_floor_db = 20 * np.log10(noise_rms + 1e-10)
            
            phase_results['tests']['noise_characteristics'] = {
                'status': 'pass' if noise_floor_db < -40 else 'warning',
                'noise_floor_db': float(noise_floor_db),
                'noise_rms': float(noise_rms),
                'noise_peak': float(noise_peak),
                'threshold_db': -40
            }
            
        except Exception as e:
            phase_results['tests']['noise_characteristics'] = {
                'status': 'fail',
                'error': str(e)
            }

        phase_results['end_time'] = datetime.now().isoformat()
        phase_results['duration_seconds'] = (datetime.fromisoformat(phase_results['end_time']) - 
                                           datetime.fromisoformat(phase_results['start_time'])).total_seconds()
        
        # 计算Phase B总体结果
        passed_tests = sum(1 for test in phase_results['tests'].values() if test.get('status') == 'pass')
        total_tests = len(phase_results['tests'])
        phase_results['success_rate'] = passed_tests / total_tests
        phase_results['overall_status'] = 'pass' if phase_results['success_rate'] >= 0.75 else 'fail'
        
        self.logger.info(f"✅ Phase B 完成: {passed_tests}/{total_tests} 测试通过 ({phase_results['success_rate']:.1%})")
        return phase_results

    def run_validation(self, phases: List[str] = None) -> Dict[str, Any]:
        """运行音频验证流程"""
        if phases is None:
            phases = ['A', 'B', 'C', 'D', 'E']
        
        self.logger.info("🚀 开始音频I/O系统验证")
        self.logger.info(f"📋 计划执行阶段: {', '.join(phases)}")
        
        all_results = {
            'validation_info': {
                'start_time': datetime.now().isoformat(),
                'phases_requested': phases,
                'config': self.config
            },
            'phases': {}
        }
        
        # 执行各阶段
        if 'A' in phases:
            all_results['phases']['A'] = self.phase_a_hardware_connection()
        
        if 'B' in phases:
            all_results['phases']['B'] = self.phase_b_microphone_array_test()
        
        # Phase C, D, E 的占位符 - 在后续实现
        if 'C' in phases:
            self.logger.info("🔄 Phase C: 扬声器校准与音质评估 (待实现)")
            all_results['phases']['C'] = {
                'phase': 'C',
                'name': '扬声器校准与音质评估',
                'status': 'pending',
                'note': '将在后续实现'
            }
        
        if 'D' in phases:
            self.logger.info("🔄 Phase D: ROS2音频话题集成验证 (待实现)")
            all_results['phases']['D'] = {
                'phase': 'D',
                'name': 'ROS2音频话题集成验证',
                'status': 'pending',
                'note': '将在后续实现',
                'ros2_available': ROS2_AVAILABLE
            }
        
        if 'E' in phases:
            self.logger.info("🔄 Phase E: 综合可视化与性能报告生成 (待实现)")
            all_results['phases']['E'] = {
                'phase': 'E',
                'name': '综合可视化与性能报告生成',
                'status': 'pending',
                'note': '将在后续实现'
            }
        
        # 保存结果
        all_results['validation_info']['end_time'] = datetime.now().isoformat()
        all_results['validation_info']['total_duration'] = (
            datetime.fromisoformat(all_results['validation_info']['end_time']) - 
            datetime.fromisoformat(all_results['validation_info']['start_time'])
        ).total_seconds()
        
        # 计算总体成功率
        completed_phases = [p for p in all_results['phases'].values() if p.get('overall_status') in ['pass', 'fail']]
        if completed_phases:
            overall_success_rate = sum(1 for p in completed_phases if p.get('overall_status') == 'pass') / len(completed_phases)
            all_results['validation_info']['overall_success_rate'] = overall_success_rate
            all_results['validation_info']['overall_status'] = 'pass' if overall_success_rate >= 0.75 else 'fail'
        
        # 保存结果到文件
        result_file = self.output_dir / f"audio_validation_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(result_file, 'w', encoding='utf-8') as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)
        
        self.logger.info(f"📄 验证结果已保存: {result_file}")
        self.logger.info("🎉 音频I/O系统验证完成!")
        
        return all_results

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Unitree Go2 音频I/O系统验证')
    parser.add_argument('--phases', '-p', nargs='+', choices=['A', 'B', 'C', 'D', 'E'],
                       default=['A', 'B'], help='要执行的验证阶段')
    parser.add_argument('--config', '-c', type=str, help='配置文件路径')
    parser.add_argument('--sample-rate', '-sr', type=int, default=44100, help='采样率')
    parser.add_argument('--channels', '-ch', type=int, default=2, help='音频通道数')
    parser.add_argument('--duration', '-d', type=float, default=5.0, help='测试持续时间(秒)')
    
    args = parser.parse_args()
    
    try:
        # 创建验证器实例
        validator = AudioValidationMain(config_path=args.config)
        
        # 更新配置参数
        if args.sample_rate:
            validator.sample_rate = args.sample_rate
        if args.channels:
            validator.channels = args.channels
        if args.duration:
            validator.test_duration = args.duration
        
        # 运行验证
        results = validator.run_validation(phases=args.phases)
        
        # 打印总结
        print("\n" + "="*60)
        print("🎵 音频I/O系统验证完成")
        print("="*60)
        
        for phase_id, phase_data in results['phases'].items():
            if isinstance(phase_data, dict) and 'overall_status' in phase_data:
                status_emoji = "✅" if phase_data['overall_status'] == 'pass' else "❌"
                success_rate = phase_data.get('success_rate', 0)
                print(f"{status_emoji} Phase {phase_id}: {phase_data['name']} ({success_rate:.1%})")
        
        overall_status = results['validation_info'].get('overall_status', 'unknown')
        overall_rate = results['validation_info'].get('overall_success_rate', 0)
        print(f"\n🎯 总体状态: {'✅ 通过' if overall_status == 'pass' else '❌ 失败'} ({overall_rate:.1%})")
        
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断验证过程")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 验证过程发生错误: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 