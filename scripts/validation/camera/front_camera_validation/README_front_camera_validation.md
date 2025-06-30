# Unitree Go2前置摄像头验证系统

## 📋 概述

本验证系统专为Unitree Go2机器人前置摄像头设计，提供全面的性能测试、图像质量分析和环境适应性验证。

## 🎯 验证内容

### 核心验证项目
- **分辨率验证**: 确认1280x720 HD规格
- **性能测试**: 帧率、延迟、稳定性测量
- **图像质量**: 色彩准确性、清晰度、噪声分析
- **环境适应**: 不同光照条件下的表现
- **压力测试**: 高负载下的系统稳定性

### 技术指标
- **分辨率**: 目标1280x720，回退480x1280
- **帧率**: 目标30fps，最低20fps
- **延迟**: <100ms（实时控制要求）
- **图像质量**: SSIM>0.8, 色彩ΔE<5
- **成功率**: >95%捕获成功率

## 🛠️ 系统架构

```
front_camera_validation/
├── validation_config.json      # 配置文件
├── camera_config.py            # 摄像头配置管理
├── performance_tester.py       # 性能测试模块
├── image_quality_analyzer.py   # 图像质量分析
├── main_validation_script.py   # 主验证脚本
└── README_front_camera_validation.md  # 本文档
```

## 🚀 快速开始

### 环境要求
```bash
# Python依赖
pip install opencv-python numpy scikit-image matplotlib

# 系统要求
- Python 3.7+
- OpenCV 4.0+
- 摄像头访问权限
```

### 基础使用

1. **快速验证**（推荐）:
```bash
cd scripts/validation/camera/front_camera_validation
python3 main_validation_script.py
```

2. **自定义配置**:
```bash
python3 main_validation_script.py --config custom_config.json --output /path/to/output
```

3. **详细输出**:
```bash
python3 main_validation_script.py --verbose
```

### 单独模块测试

1. **摄像头配置测试**:
```bash
python3 camera_config.py
```

2. **性能测试**:
```bash
python3 performance_tester.py
```

3. **图像质量分析**:
```bash
python3 image_quality_analyzer.py
```

## 📊 验证流程

### 标准验证序列
1. **摄像头初始化** - 连接和配置验证
2. **分辨率验证** - HD规格确认
3. **基础性能测试** - 30秒性能基准测试
4. **图像质量分析** - 20样本质量评估
5. **压力测试** - 60秒高负载测试
6. **报告生成** - HTML和JSON结果报告

### 测试时长
- **快速验证**: ~5分钟
- **完整验证**: ~10分钟
- **扩展验证**: ~15分钟（包含环境测试）

## 📋 配置说明

### 主要配置项

```json
{
  "camera_config": {
    "target_resolution": [1280, 720],
    "fallback_resolution": [480, 1280],
    "target_fps": 30,
    "camera_id": 0
  },
  "performance_thresholds": {
    "max_latency_ms": 100,
    "min_fps": 20,
    "min_capture_success_rate": 0.95
  },
  "image_quality_thresholds": {
    "min_ssim": 0.8,
    "max_color_delta_e": 5.0,
    "min_sharpness_score": 0.7
  }
}
```

### 自定义验证序列

```json
{
  "validation_sequence": [
    "camera_initialization",
    "resolution_verification",
    "basic_performance_test",
    "image_quality_analysis",
    "stress_test",
    "report_generation"
  ]
}
```

## 📈 结果解读

### 性能指标

| 指标 | 优秀 | 良好 | 可接受 | 需改进 |
|------|------|------|--------|--------|
| FPS | ≥30 | 25-29 | 20-24 | <20 |
| 延迟(ms) | <50 | 50-80 | 80-100 | >100 |
| 成功率(%) | ≥99 | 97-98 | 95-96 | <95 |

### 图像质量指标

| 指标 | 优秀 | 良好 | 可接受 | 需改进 |
|------|------|------|--------|--------|
| 清晰度 | ≥90 | 80-89 | 70-79 | <70 |
| 色彩准确性 | ΔE<2 | 2-3 | 3-5 | >5 |
| 噪声水平 | <0.05 | 0.05-0.08 | 0.08-0.1 | >0.1 |

### 整体评级

- **EXCELLENT**: ≥90分，所有关键指标优秀
- **GOOD**: 80-89分，主要指标良好
- **ACCEPTABLE**: 70-79分，基本满足要求
- **POOR**: 60-69分，存在明显问题
- **UNACCEPTABLE**: <60分，需要重大改进

## 📄 输出结果

### 文件结构
```
logs/camera_validation/validation_YYYYMMDD_HHMMSS/
├── validation_results.json          # 完整验证结果
├── validation_report.html           # HTML可视化报告
├── validation_report.json          # JSON格式报告
├── basic_performance_metrics.json  # 基础性能数据
├── image_quality_metrics.json      # 图像质量数据
└── stress_test_metrics.json        # 压力测试数据
```

### HTML报告特点
- 直观的状态显示（通过/失败/警告）
- 详细的性能图表
- 图像质量样本展示
- 优化建议和问题诊断

### JSON报告用途
- 自动化分析和监控
- 历史数据对比
- 集成到CI/CD流程
- 生成趋势报告

## 🔧 故障排除

### 常见问题

1. **摄像头无法初始化**
```bash
# 检查摄像头设备
ls /dev/video*
# 检查权限
sudo usermod -a -G video $USER
# 重新登录后重试
```

2. **分辨率不匹配**
```bash
# 检查摄像头支持的分辨率
v4l2-ctl --list-formats-ext
# 修改配置文件中的target_resolution
```

3. **性能问题**
```bash
# 检查系统负载
htop
# 关闭不必要的应用程序
# 调整camera_config中的buffer_size
```

4. **权限问题**
```bash
# 添加摄像头权限
sudo usermod -a -G video $USER
# 重新登录生效
```

### 日志分析

1. **查看详细日志**:
```bash
tail -f front_camera_validation.log
```

2. **调试模式**:
```bash
python3 main_validation_script.py --verbose
```

3. **检查配置**:
```bash
python3 -c "import json; print(json.load(open('validation_config.json')))"
```

## 🎛️ 高级用法

### 自定义测试

1. **仅性能测试**:
```python
from performance_tester import PerformanceTester
from camera_config import CameraConfig

with CameraConfig() as camera:
    if camera.initialize_camera():
        tester = PerformanceTester(camera)
        metrics = tester.run_basic_performance_test(60.0)
        print(f"FPS: {metrics.fps_actual:.2f}")
```

2. **仅质量分析**:
```python
from image_quality_analyzer import ImageQualityAnalyzer
from camera_config import CameraConfig

with CameraConfig() as camera:
    if camera.initialize_camera():
        analyzer = ImageQualityAnalyzer(camera)
        metrics = analyzer.analyze_image_quality(50)
        print(f"质量评分: {metrics.overall_quality_score:.2f}")
```

### 批量验证

```bash
#!/bin/bash
# 多次验证以获得统计数据
for i in {1..5}; do
    echo "验证轮次 $i"
    python3 main_validation_script.py --output logs/batch_$i
    sleep 30
done
```

### CI/CD集成

```bash
#!/bin/bash
# 自动化验证脚本
python3 main_validation_script.py --config ci_config.json
exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo "验证通过"
else
    echo "验证失败"
    exit 1
fi
```

## 📞 技术支持

### 联系方式
- 技术文档: [内部技术文档]
- 问题报告: [内部问题跟踪系统]
- 技术支持: [内部技术支持]

### 版本信息
- 当前版本: 1.0.0
- 更新日期: 2024-12-26
- 兼容性: Unitree Go2, Python 3.7+

### 许可信息
内部使用，遵循公司技术开发规范。 