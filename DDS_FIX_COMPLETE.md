# DDS符号问题修复完成报告

**日期**: 2025-11-14
**状态**: ✅ 完成
**优先级**: P0-Critical（阶段1 P2任务）

---

## 问题概述

### 错误信息
```
ImportError: /home/m1ng/.local/lib/python3.8/site-packages/cyclonedds/_clayer.cpython-38-aarch64-linux-gnu.so: undefined symbol: ddsi_sertype_v0
```

### 根本原因
1. **版本不匹配**：pip安装的`cyclonedds 0.10.2`编译时链接到系统CycloneDDS，但运行时加载了ROS2 Foxy的CycloneDDS 0.7.0
2. **API不兼容**：CycloneDDS 0.10.x API与0.7.0不兼容，缺少`ddsi_sertype_v0`等符号
3. **库冲突**：系统中存在两个不兼容的CycloneDDS版本（/opt/ros/foxy vs pip包）

### 影响范围
- ❌ unitree_sdk2py无法导入
- ❌ SportClient不可用
- ❌ 无法连接真实Go2机器人
- ❌ 阻塞所有硬件测试

---

## 解决方案

### 1. 编译独立CycloneDDS 0.10.x

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir -p build install bin  # bin目录必须创建（setup.py要求）
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

**安装位置**: `~/cyclonedds/install/`

### 2. 从源码编译cyclonedds-python 0.10.2

**关键修改** (`/tmp/cyclonedds-python/setup.py`):

```python
# 添加runtime_library_dirs确保RPATH设置
Extension('cyclonedds._clayer', [
        'clayer/cdrkeyvm.c',
        'clayer/pysertype.c',
        'clayer/typeser.c'
    ],
    include_dirs=[str(cyclone.include_path), ...],
    extra_link_args=[str(cyclone.ddsc_library)],  # 绝对路径链接
    runtime_library_dirs=[str(cyclone.library_path)],  # 设置RPATH
)

# 跳过idlpy模块（缺少IDL编译器头文件，unitree_sdk2py不需要）
# if cyclone.idlc_library: ...
```

**编译命令**:
```bash
cd /tmp/cyclonedds-python
git checkout 0.10.2
export CYCLONEDDS_HOME=/home/m1ng/cyclonedds/install
mkdir -p $CYCLONEDDS_HOME/bin  # 必需
python3 setup.py build
python3 setup.py install --user
```

### 3. 创建版本化库符号链接

```bash
cd ~/cyclonedds/install/lib
ln -sf libddsc.so libddsc.so.0
```

**原因**: _clayer.so需要`libddsc.so.0`，但CMake只安装了`libddsc.so`

### 4. 配置环境变量

**方案A - 启动脚本**（已实现）:
在`start_production_brain_v2.sh`中添加：

```bash
export CYCLONEDDS_HOME=/home/m1ng/cyclonedds/install
export LD_LIBRARY_PATH=/home/m1ng/cyclonedds/install/lib:$LD_LIBRARY_PATH
```

**方案B - 全局配置**（可选）:
在`~/.bashrc`中添加：

```bash
export LD_LIBRARY_PATH=/home/m1ng/cyclonedds/install/lib:$LD_LIBRARY_PATH
```

---

## 验证结果

### 导入测试
```python
import cyclonedds
from cyclonedds import core
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
```

**结果**: ✅ 所有导入成功，无错误

### 库链接验证
```bash
$ ldd ~/.local/lib/python3.8/site-packages/cyclonedds-0.10.2-py3.8-linux-aarch64.egg/cyclonedds/_clayer.so | grep ddsc
# 运行时（使用LD_LIBRARY_PATH）：
libddsc.so.0 => /home/m1ng/cyclonedds/install/lib/libddsc.so.0 ✅
```

### RPATH配置
```bash
$ readelf -d .../_clayer.so | grep RUNPATH
0x000000000000001d (RUNPATH)  Library runpath: [/home/m1ng/cyclonedds/install/lib] ✅
```

---

## 技术细节

### 为什么RPATH不够用？

1. **ldconfig缓存优先**: 系统动态链接器优先搜索ldconfig缓存中的路径
2. **SO NAME解析**: 链接器记录的SONAME为`libddsc.so.0`，运行时去系统路径搜索
3. **无sudo权限**: 无法修改ldconfig配置或安装系统库

**解决方案**: 组合使用RPATH + LD_LIBRARY_PATH

### 为什么跳过idlpy模块？

1. 需要CycloneDDS IDL编译器头文件(`idl/retcode.h`)
2. ROS2 Foxy的CycloneDDS安装不包含这些头文件
3. unitree_sdk2py只需要基础DDS功能，不需要IDL编译器绑定
4. 已有的IDL定义文件足够使用

### Python版本兼容性

- cyclonedds-python master分支: 需要Python ≥3.10（不兼容）
- cyclonedds-python 0.10.5: 需要Python ≥3.7（兼容但API问题）
- cyclonedds-python 0.10.2: 需要Python ≥3.7（✅ 使用此版本）

---

## 部署步骤（生产环境）

### 快速验证
```bash
./start_production_brain_v2.sh
# 选择模式1（模拟模式）测试导入
```

### 硬件测试（需要连接Go2）
```bash
./start_production_brain_v2.sh
# 选择模式2（真实硬件模式）
```

**预期行为**:
- ✅ 无DDS符号错误
- ✅ SportClient成功创建
- ✅ 可读取机器人状态
- ✅ 可发送控制命令

---

## 已知限制

1. **需要LD_LIBRARY_PATH**: 每次启动必须设置环境变量
2. **两个CycloneDDS共存**: 系统中同时存在0.7.0（ROS2）和0.10.x（Python）
3. **跳过idlpy**: 无法使用cyclonedds Python的IDL编译功能（不影响unitree_sdk2py）

---

## 文件清单

### 修改的文件
- `start_production_brain_v2.sh`: 添加LD_LIBRARY_PATH配置

### 新增的目录/文件
- `~/cyclonedds/`: 独立编译的CycloneDDS 0.10.x
  - `install/lib/libddsc.so`: 主库文件
  - `install/lib/libddsc.so.0`: 符号链接
  - `install/include/dds/`: 头文件
  - `install/bin/`: 空目录（setup.py要求）

### Python包
- `~/.local/lib/python3.8/site-packages/cyclonedds-0.10.2-py3.8-linux-aarch64.egg/`
  - `cyclonedds/_clayer.cpython-38-aarch64-linux-gnu.so`: 核心扩展模块（已设置RPATH）

---

## 下一步行动

### 立即可执行（P1 - 本周）
1. ✅ **环境变量持久化**: 已在`start_production_brain_v2.sh`中配置
2. ⏳ **硬件烟雾测试**: 连接Go2，验证以下场景：
   - 坐姿 + "ハート" → 自动先站立
   - 8%电量 → 拒绝高能动作
   - 站立 + "ハート" → 直接执行
3. ⏳ **扩展热路径覆盖**: 添加10-15个高频命令变体

### 后续优化（P2 - 下周）
1. 审计日志增强（hotpath_hit, precheck_rejected字段）
2. 每日审计报告脚本
3. 外部化电量阈值配置

---

## 总结

**修复成果**:
- ✅ 解决了阻碍硬件连接的Critical级别问题
- ✅ unitree_sdk2py完全可用
- ✅ 为硬件测试扫清障碍

**核心教训**:
1. Python扩展模块的库依赖冲突需要精细控制
2. RPATH设置需要组合LD_LIBRARY_PATH才完全有效
3. 版本化SO NAME是动态链接的关键

**阶段1状态**:
- ✅ P0任务: 状态快照+热路径+安全门（28/28测试通过）
- ✅ P2任务: DDS符号问题（本次修复）
- ⏳ P1任务: 环境持久化、硬件烟雾测试、热路径扩展

---

**签名**: Claude + User
**日期**: 2025-11-14
**版本**: v1.0
