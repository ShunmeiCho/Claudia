# CycloneDDS环境配置说明

## 问题背景

Unitree机器人SDK使用CycloneDDS 0.10.x版本进行通信，而默认的pip安装版本存在符号兼容性问题，会出现以下错误：

```
ImportError: undefined symbol: ddsi_sertype_v0
```

## 解决方案

本目录提供了完整的CycloneDDS环境配置脚本，解决了版本兼容性问题。

## 使用方法

### 1. 基本配置（每次使用前执行）

```bash
# 在项目根目录执行
source scripts/setup/setup_cyclonedds.sh
```

### 2. 配置并测试

```bash
# 配置环境并测试unitree_sdk2py导入
source scripts/setup/setup_cyclonedds.sh --test
```

### 3. 将配置添加到shell启动脚本（可选）

如果希望每次打开终端自动配置环境：

```bash
echo "source ~/claudia/scripts/setup/setup_cyclonedds.sh" >> ~/.bashrc
```

**注意**: 仅在确认配置稳定后再添加到~/.bashrc

## 环境要求

- CycloneDDS 0.10.x已编译安装在 `~/cyclonedds/install`
- unitree_sdk2py已正确安装
- ROS2 Foxy环境

## 验证方法

执行以下Python代码验证配置是否成功：

```python
import unitree_sdk2py
from unitree_sdk2py.core.channel import ChannelSubscriber
print("✅ CycloneDDS环境配置成功")
```

## 故障排除

### 1. "CycloneDDS未找到"错误

```bash
# 检查CycloneDDS是否已编译
ls ~/cyclonedds/install/lib/libddsc.so
```

如果文件不存在，需要重新编译CycloneDDS：

```bash
cd ~/cyclonedds
rm -rf build install
mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

### 2. "导入失败"错误

```bash
# 重新安装unitree_sdk2py
cd ~/unitree_sdk2_python
pip3 uninstall unitree_sdk2py cyclonedx -y
source ~/claudia/scripts/setup/setup_cyclonedds.sh
pip3 install -e .
```

## 相关文件

- `setup_cyclonedds.sh` - 主要配置脚本
- `README_cyclonedds.md` - 本说明文档
- `~/cyclonedds/install/` - CycloneDDS安装目录
- `~/unitree_sdk2_python/` - Unitree SDK源码目录

---

**最后更新**: 2024年6月27日  
**适用版本**: CycloneDDS 0.10.x, ROS2 Foxy, Ubuntu 20.04 