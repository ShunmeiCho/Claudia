# Claudia机器人LED控制系统

## 📋 概述

基于Unitree Go2 VUI客户端的统一LED控制系统，提供简洁而实用的LED亮度控制功能。

## 🔍 技术背景

### ✅ 当前可用方案
- **VUI客户端亮度控制**: `VuiClient.SetBrightness(0-10)` 
  - 权限要求低，普通用户可用
  - 11个亮度级别（0=关闭, 10=最亮）
  - 稳定可靠，已验证可用

### ❌ 已废除方案（权限不足）
- **AudioClient颜色控制**: `AudioClient.LedControl(r,g,b)`
  - 需要系统级权限（root权限）
  - 普通用户模式下无法访问
  - **已完全废除**

- **LowCmd LED字段**: `uint8[12] led`
  - 需要机器人底层控制权限
  - 涉及硬件直接操作
  - **已完全废除**

## 🏗️ 系统架构

```
scripts/led_control/
├── README.md                    # 本文档
├── led_test.py                  # LED控制器测试脚本
└── led_brightness_test.py       # 基础LED亮度测试（从audio迁移）

src/claudia/robot_controller/
├── __init__.py                  # 模块导出
└── led_controller.py            # 统一LED控制器实现
```

## 🎯 功能特性

### LED亮度级别
```python
from src.claudia.robot_controller import LEDBrightnessLevel

# 11个预定义亮度级别
LEDBrightnessLevel.OFF        # 0 - 关闭
LEDBrightnessLevel.VERY_LOW   # 1 - 很暗
LEDBrightnessLevel.LOW        # 2 - 暗
LEDBrightnessLevel.DIM        # 3 - 较暗
LEDBrightnessLevel.MEDIUM_LOW # 4 - 中等偏暗
LEDBrightnessLevel.MEDIUM     # 5 - 中等
LEDBrightnessLevel.MEDIUM_HIGH # 6 - 中等偏亮
LEDBrightnessLevel.BRIGHT     # 7 - 亮
LEDBrightnessLevel.HIGH       # 8 - 很亮
LEDBrightnessLevel.VERY_HIGH  # 9 - 极亮
LEDBrightnessLevel.MAX        # 10 - 最亮
```

### Claudia LED模式
```python
from src.claudia.robot_controller import ClaudiaLEDMode

# 6种预定义机器人状态模式
ClaudiaLEDMode.IDLE            # 空闲：低亮度常亮
ClaudiaLEDMode.WAKE_CONFIRM    # 唤醒确认：中等亮度
ClaudiaLEDMode.PROCESSING      # 处理中：中高亮度
ClaudiaLEDMode.EXECUTING       # 执行中：高亮度常亮
ClaudiaLEDMode.ACTION_COMPLETE # 完成：亮度较高
ClaudiaLEDMode.ERROR           # 错误：最高亮度
```

## 🚀 使用方法

### 基础使用
```python
from src.claudia.robot_controller import LEDController

# 创建并初始化LED控制器
led_controller = LEDController()
if led_controller.initialize():
    
    # 设置亮度（0-10）
    led_controller.set_brightness(7)
    
    # 设置Claudia模式
    led_controller.set_mode(ClaudiaLEDMode.WAKE_CONFIRM)
    
    # 开关控制
    led_controller.turn_off()
    led_controller.turn_on(brightness=5)
    
    # 获取状态
    status = led_controller.get_status()
    
    # 清理资源
    led_controller.cleanup()
```

### 便捷函数
```python
from src.claudia.robot_controller import get_led_controller

# 使用全局LED控制器实例
led = get_led_controller()
led.initialize()
led.set_mode(ClaudiaLEDMode.PROCESSING)
```

## 🧪 测试和验证

### 运行LED控制器测试
```bash
# 完整功能测试
cd /workspace
python3 scripts/led_control/led_test.py

# 基础亮度测试（需要numpy依赖）
python3 scripts/led_control/led_brightness_test.py
```

### 测试内容
- ✅ 模块导入验证
- ✅ LED控制器初始化
- ✅ 亮度级别控制（0-10）
- ✅ Claudia模式切换
- ✅ 开关功能测试
- ✅ 状态查询验证

## 📊 API参考

### LEDController类
```python
class LEDController:
    def initialize() -> bool                    # 初始化控制器
    def set_brightness(brightness: int) -> bool # 设置亮度(0-10)
    def get_brightness() -> Optional[int]       # 获取当前亮度
    def set_mode(mode: ClaudiaLEDMode) -> bool  # 设置Claudia模式
    def turn_off() -> bool                      # 关闭LED
    def turn_on(brightness: int = 5) -> bool    # 打开LED
    def get_status() -> Dict[str, Any]          # 获取控制器状态
    def cleanup()                               # 清理资源
```

## ⚠️ 限制和注意事项

1. **权限限制**: 只能控制亮度，无法控制颜色
2. **依赖要求**: 需要`unitree_sdk2py`模块和Go2机器人连接
3. **VUI服务**: 依赖Unitree Go2的VUI服务正常运行
4. **单线程**: 当前不支持并发LED操作

## 🔧 故障排除

### 常见问题

**问题**: `ModuleNotFoundError: No module named 'unitree_sdk2py'`
**解决**: 确保已安装Unitree SDK2并正确配置环境

**问题**: LED控制器初始化失败
**解决**: 
- 检查Go2机器人是否开机并连接
- 确认VUI服务是否启动
- 验证网络连接状态

**问题**: 亮度设置失败
**解决**:
- 确认亮度值在0-10范围内
- 检查VUI客户端连接状态
- 重新初始化LED控制器

## 📝 更新日志

### v1.0.0 (2025-07-04)
- ✅ 基于VUI客户端的统一LED控制器
- ✅ 11个亮度级别支持
- ✅ 6种Claudia LED模式
- ✅ 完整的测试框架
- ❌ 废除了基于权限的颜色控制方案
- ❌ 清理了无效的测试框架

---

**重要**: 此LED控制系统专为Claudia机器人设计，基于实际可用的VUI API，避免了权限问题，提供稳定可靠的LED控制功能。