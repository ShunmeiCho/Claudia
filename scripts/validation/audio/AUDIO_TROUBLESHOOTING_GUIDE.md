# Unitree Go2 音频问题排查与解决指南

## 🎯 问题现状

**症状**: 录音测试失败，最大振幅和RMS电平为0.0000  
**设备**: Unitree Go2 (NVIDIA Jetson Orin NX - aarch64)  
**系统**: Ubuntu 20.04  

---

## 🔍 **问题根因分析**

### ✅ **正常的部分**
- 权限配置正确 (用户在audio组)
- PulseAudio运行正常
- ALSA设备识别正常 (26个播放设备，22个录制设备)
- 音频设备枚举成功

### ❌ **问题所在**
1. **Jetson Orin NX复杂音频路由**：DMIC1-DMIC4数字麦克风未正确配置
2. **Unitree专用AudioClient未使用**：Go2机器人需要通过SDK2的AudioClient访问音频
3. **音频路由配置错误**：大多数mixer控制设置为'None'状态

---

## 🛠️ **解决方案 (按优先级排序)**

### 🥇 **方案1: 使用Unitree AudioClient SDK (推荐)**

根据最新的Unitree SDK2文档，Go2机器人应该通过专用的AudioClient API访问音频功能：

```python
# Unitree Go2 专用音频接口
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.audio.audio_client import AudioClient

# 初始化
ChannelFactoryInitialize(0, "")
audio_client = AudioClient()

# 音频操作
volume = audio_client.GetVolume()
audio_client.SetVolume(85)
audio_client.TtsMaker("测试语音", 0)  # TTS测试
# ASR和麦克风录音通过专用接口
```

**优势**: 
- 绕过Jetson复杂音频路由
- 直接访问Go2硬件麦克风阵列
- 与机器人系统完全兼容

---

### 🥈 **方案2: 配置Jetson音频路由**

#### 2.1 启用DMIC数字麦克风
```bash
# 尝试启用DMIC
sudo amixer -c 1 set 'ADMAIF1 Mux' 'DMIC1' 
sudo amixer -c 1 set 'ADMAIF2 Mux' 'DMIC2'

# 设置录音音量
sudo amixer -c 1 set 'DMIC1' 100%
sudo amixer -c 1 set 'DMIC2' 100%
```

#### 2.2 配置PulseAudio
```bash
# 重启PulseAudio
pulseaudio --kill
pulseaudio --start

# 列出可用输入源
pactl list sources short

# 设置默认输入源
pactl set-default-source <source_name>
```

---

### 🥉 **方案3: 手动设备测试**

#### 3.1 逐个测试ALSA设备
```bash
# 列出录制设备
arecord -l

# 测试特定设备录音
arecord -D hw:1,0 -f S16_LE -r 44100 -c 2 -d 5 test_record.wav

# 播放测试
aplay test_record.wav
```

#### 3.2 使用不同的设备ID
在Python脚本中尝试不同的sounddevice设备ID:
```python
import sounddevice as sd

# 测试所有输入设备
devices = sd.query_devices()
for i, device in enumerate(devices):
    if device['max_input_channels'] > 0:
        try:
            # 使用更低的采样率和单声道
            recording = sd.rec(
                int(1 * 8000),  # 1秒，8kHz
                samplerate=8000, 
                channels=1,
                device=i
            )
            sd.wait()
            print(f"设备 {i} 录音成功，最大振幅: {max(abs(recording))}")
        except Exception as e:
            print(f"设备 {i} 失败: {e}")
```

---

## 🚀 **立即可试的快速修复**

### 快速修复1: 重置音频系统
```bash
# 完整重置音频系统
sudo systemctl restart alsa-utils
pulseaudio --kill
pulseaudio --start

# 重新检测音频设备
sudo alsa force-reload
```

### 快速修复2: 尝试USB音频设备
如果有USB麦克风或音频接口，插入后尝试：
```bash
# 检查新设备
lsusb | grep -i audio
arecord -l
```

### 快速修复3: 检查Unitree SDK2安装
```bash
# 验证Unitree SDK2
python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('SDK2 OK')"

# 如果失败，重新安装
cd ~/unitree_sdk2_python
pip3 install -e .
```

---

## 🎯 **推荐的解决路径**

### 阶段1: 验证Unitree AudioClient (2小时)
1. 确认unitree_sdk2py安装正确
2. 编写AudioClient测试脚本
3. 验证TTS、音量控制功能
4. 测试ASR和录音功能

### 阶段2: 如果AudioClient不可用，配置Jetson音频 (4小时)  
1. 按方案2配置DMIC
2. 测试PulseAudio配置
3. 逐个验证ALSA设备

### 阶段3: 备选硬件方案 (1小时)
1. 使用USB音频设备作为备选
2. 验证外部麦克风工作

---

## 📋 **验证检查清单**

在每个解决方案后运行此检查：

```bash
# 检查清单脚本
echo "=== 音频系统状态检查 ==="
echo "1. 用户组:" $(groups | grep audio)
echo "2. PulseAudio:" $(pgrep pulseaudio > /dev/null && echo "运行中" || echo "未运行")
echo "3. ALSA播放设备:" $(aplay -l | grep "card" | wc -l)
echo "4. ALSA录制设备:" $(arecord -l | grep "card" | wc -l)
echo "5. Unitree SDK2:" $(python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('可用')" 2>/dev/null || echo "不可用")

# 快速录音测试
echo "6. 快速录音测试:"
arecord -D hw:1,0 -f S16_LE -r 16000 -c 1 -d 2 quick_test.wav 2>/dev/null && echo "录音成功" || echo "录音失败"
```

---

## 💡 **专家建议**

基于文档和您的系统配置，我**强烈推荐优先尝试方案1 (Unitree AudioClient)**，因为：

1. **专为Go2设计**：避免Jetson复杂的音频路由配置
2. **厂商支持**：Unitree提供的官方音频接口
3. **功能完整**：支持TTS、ASR、音量控制、录音等全部功能
4. **系统集成**：与Go2机器人系统完全兼容

**下一步行动**:
1. 运行检查清单确认当前状态
2. 尝试Unitree AudioClient接口
3. 如果成功，更新音频验证系统以使用AudioClient
4. 如果失败，按照方案2配置Jetson音频路由

---

## 📞 **获取帮助**

如果以上方案都无效，请收集以下信息：
- 检查清单输出结果
- `dmesg | grep -i audio` 输出
- Unitree SDK2版本信息
- Go2机器人固件版本

这将帮助进一步诊断问题。 