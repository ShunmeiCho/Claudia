# 🖥️ SSH环境下LiDAR点云可视化指南

> **适用场景**: 远程SSH连接，无法直接查看图形界面的环境  
> **目标**: 查看Unitree Go2 4D LiDAR L1的点云数据  
> **话题**: `/utlidar/cloud`  

---

## 🚀 **快速开始**

### 环境准备
```bash
# 进入项目目录
cd ~/claudia

# 加载ROS2环境
source /opt/ros/foxy/setup.bash
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 验证点云话题可用
ros2 topic list | grep utlidar
ros2 topic info /utlidar/cloud
```

---

## 🎯 **方案选择**

### **方案1: X11转发 (最佳用户体验)**
```bash
# 重新连接SSH时启用X11转发
ssh -X username@robot_ip
# 或
ssh -Y username@robot_ip  # 可信任X11转发

# 运行RViz2查看器
./scripts/hardware_validation/rviz_pointcloud_viewer.sh
```

**配置RViz2步骤**:
1. 添加 `PointCloud2` 显示类型
2. Topic设置为: `/utlidar/cloud`
3. Fixed Frame设置为: `utlidar_lidar`
4. 调整点云大小和颜色方案

**优点**: 实时交互，3D操作  
**缺点**: 需要X11支持，带宽要求高

---

### **方案2: 生成静态图像 (推荐SSH用户)**
```bash
# 生成多视角点云图像
python3 scripts/hardware_validation/static_pointcloud_viewer.py
```

**输出结果**:
- 📁 `logs/pointcloud_images/` 目录
- 🖼️ 5张高分辨率PNG图像 
- 📊 包含统计信息和多视角视图

**特点**:
- ✅ 顶视图、侧视图、前视图
- ✅ 距离分布直方图
- ✅ 详细统计信息
- ✅ 300 DPI高清输出

**优点**: 无需图形界面，文件小，易分享  
**缺点**: 静态图像，无交互

---

### **方案3: 保存PCD文件 (专业分析)**
```bash
# 保存标准PCD格式文件
python3 scripts/hardware_validation/save_pointcloud_pcd.py
```

**输出结果**:
- 📁 `logs/pointcloud_pcd/` 目录
- 📄 3个标准PCD格式文件
- 💾 包含完整点云数据

**本地查看工具**:
```bash
# CloudCompare (推荐)
# 下载: https://www.cloudcompare.org/

# PCL Viewer
pcl_viewer filename.pcd

# MeshLab
# 下载: https://www.meshlab.net/

# Python Open3D
python3 -c "import open3d as o3d; pcd = o3d.io.read_point_cloud('filename.pcd'); o3d.visualization.draw_geometries([pcd])"
```

**优点**: 标准格式，专业工具支持，完整数据  
**缺点**: 需要下载，本地软件

---

### **方案4: 实时数据监控**
```bash
# 查看点云话题实时统计
ros2 topic hz /utlidar/cloud
ros2 topic bw /utlidar/cloud

# 查看点云消息结构
ros2 topic echo --no-arr /utlidar/cloud | head -20
```

**输出信息**:
- 发布频率 (Hz)
- 带宽使用 (MB/s)  
- 消息结构和字段
- Frame ID信息

---

## 🔧 **故障排除**

### **无法连接到话题**
```bash
# 检查ROS2环境
echo $RMW_IMPLEMENTATION  # 应显示: rmw_cyclonedds_cpp
ros2 daemon stop && ros2 daemon start

# 检查话题状态
ros2 topic list | grep -i lidar
ros2 node list | grep -i lidar
```

### **Python脚本错误**
```bash
# 检查依赖
python3 -c "import rclpy, numpy, matplotlib; print('✅ 依赖正常')"

# 重新安装必要包
pip3 install --upgrade numpy matplotlib open3d
```

### **X11转发失败**
```bash
# 测试X11
echo $DISPLAY
xeyes  # 测试程序

# 本地SSH配置 (~/.ssh/config)
Host robot
    ForwardX11 yes
    ForwardX11Trusted yes
```

---

## 📊 **性能基准**

| 方案 | 文件大小 | 生成时间 | 带宽需求 | 适用场景 |
|------|----------|----------|----------|----------|
| X11转发 | N/A | 实时 | 高 (>1MB/s) | 实时调试 |
| 静态图像 | ~2MB/张 | 10-30秒 | 低 | 报告展示 |
| PCD文件 | ~1-5MB/个 | 5-15秒 | 低 | 专业分析 |
| 数据监控 | N/A | 实时 | 极低 | 系统诊断 |

---

## 🎯 **最佳实践**

### **开发阶段**
1. 使用方案4监控数据流状态
2. 使用方案2生成静态图像验证
3. 必要时使用方案3保存关键数据

### **调试阶段**  
1. 优先尝试方案1 (X11转发)
2. 备用方案2生成多角度视图
3. 使用方案3保存问题数据

### **部署阶段**
1. 使用方案4进行持续监控
2. 定期使用方案2生成报告图像
3. 关键节点使用方案3保存数据

---

## 📁 **输出文件组织**

```
logs/
├── pointcloud_images/          # 静态图像
│   ├── pointcloud_frame_001_20250627_101530.png
│   ├── pointcloud_frame_002_20250627_101535.png
│   └── ...
├── pointcloud_pcd/            # PCD文件
│   ├── unitree_go2_lidar_20250627_101530_001.pcd
│   ├── unitree_go2_lidar_20250627_101535_002.pcd
│   └── ...
└── lidar_l1_validation_*.json # 验证报告
```

---

## 📞 **支持信息**

- **项目文档**: `docs/`
- **验证报告**: `logs/lidar_l1_validation_summary.md`
- **技术支持**: 参考TaskMaster任务4.1详情

**脚本位置**:
- `scripts/hardware_validation/rviz_pointcloud_viewer.sh`
- `scripts/hardware_validation/static_pointcloud_viewer.py`  
- `scripts/hardware_validation/save_pointcloud_pcd.py` 