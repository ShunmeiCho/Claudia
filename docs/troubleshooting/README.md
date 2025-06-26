# Claudia机器人故障排除指南

## 🔧 环境配置问题

### 问题1: DDS库加载失败

**错误信息**:
```
OSError: libddsc.so.0: cannot open shared object file: No such file or directory
```

**原因**: CycloneDDS环境未正确加载

**解决方案**:
```bash
# 1. 确保按正确顺序设置环境
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. 或使用自动化脚本
source scripts/setup/setup_environment.sh

# 3. 验证环境变量
echo $RMW_IMPLEMENTATION
```

### 问题2: 环境变量名称错误

**错误表现**: 设置了环境变量但仍然报错

**常见错误**:
```bash
# ❌ 错误写法
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # cyclonedx

# ✅ 正确写法  
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # cyclonedds
```

### 问题3: 工作空间未构建

**错误信息**:
```
cyclonedds_ws/install/setup.bash: No such file or directory
```

**解决方案**:
```bash
cd cyclonedds_ws
colcon build --symlink-install
source install/setup.bash
```

## 🤖 机器人连接问题

### 问题4: 机器人无响应

**症状**: 测试脚本运行但收不到数据

**检查清单**:
```bash
# 1. 检查网络连接
ping 192.168.1.120  # 机器人IP

# 2. 检查网络接口
ip addr show

# 3. 验证DDS通信
ros2 topic list

# 4. 检查防火墙
sudo ufw status
```

**解决方案**:
```bash
# 确保机器人和电脑在同一网络
# 确保RMW_IMPLEMENTATION正确设置
# 重启机器人和网络服务
```

### 问题5: 权限问题

**错误信息**:
```
Permission denied: /dev/ttyUSB0
```

**解决方案**:
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重启系统使更改生效
sudo reboot
```

## 🧪 测试相关问题

### 问题6: 模块导入失败

**错误信息**:
```
ModuleNotFoundError: No module named 'unitree_sdk2py'
```

**解决方案**:
```bash
# 1. 验证SDK安装
ls -la unitree_sdk2_python/

# 2. 检查Python路径
echo $PYTHONPATH

# 3. 重新设置环境
source scripts/setup/setup_environment.sh

# 4. 手动添加路径
export PYTHONPATH=$PYTHONPATH:/home/m1ng/claudia/unitree_sdk2_python
```

### 问题7: 测试超时

**症状**: 测试脚本等待数据超时

**原因分析**:
1. 机器人未开机或网络未连接
2. DDS配置不正确
3. 防火墙阻塞通信

**解决步骤**:
```bash
# 1. 检查机器人状态
# 确保机器人已开机且指示灯正常

# 2. 重新设置环境
source scripts/setup/setup_environment.sh

# 3. 运行连接测试
python3 test/hardware/test_unitree_connection.py

# 4. 如果仍然失败，检查网络配置
```

## 📊 性能问题

### 问题8: 通信延迟过高

**症状**: 控制命令响应慢

**分析方法**:
```bash
# 运行性能测试
python3 test/hardware/test_communication_performance.py

# 检查网络延迟
ping -c 10 192.168.1.120
```

**优化建议**:
1. 使用有线连接而非WiFi
2. 减少网络负载
3. 优化DDS配置
4. 检查系统资源使用

### 问题9: 内存不足

**错误信息**:
```
RuntimeError: Cannot allocate memory
```

**解决方案**:
```bash
# 1. 检查内存使用
free -h

# 2. 清理缓存
sudo apt clean
sudo apt autoremove

# 3. 关闭不必要的进程
sudo systemctl stop unnecessary-services

# 4. 重启系统
sudo reboot
```

## 🛠️ 系统配置问题

### 问题10: ROS2环境冲突

**症状**: 多个ROS版本冲突

**解决方案**:
```bash
# 1. 清理环境变量
unset ROS_DISTRO
unset ROS_VERSION  
unset ROS_ROOT

# 2. 重新source正确版本
source /opt/ros/foxy/setup.bash

# 3. 验证版本
echo $ROS_DISTRO  # 应显示 "foxy"
```

### 问题11: 磁盘空间不足

**检查命令**:
```bash
df -h
du -sh cyclonedds_ws/
```

**清理方案**:
```bash
# 清理构建文件
rm -rf cyclonedds_ws/build/
rm -rf cyclonedds_ws/log/

# 清理日志
find logs/ -name "*.log" -mtime +7 -delete

# 清理临时文件
sudo apt autoremove
sudo apt autoclean
```

## 🐛 调试技巧

### 启用详细日志

```bash
# 设置日志级别
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 运行测试
python3 test/hardware/test_unitree_connection.py
```

### 网络诊断

```bash
# 检查DDS发现
ros2 daemon status
ros2 daemon stop
ros2 daemon start

# 检查话题通信
ros2 topic hz /sportmodestate
ros2 topic echo /sportmodestate
```

### 系统诊断

```bash
# 检查系统资源
top
htop
iostat

# 检查网络
netstat -an | grep 7400
ss -tuln | grep 7400
```

## 📞 获取帮助

### 检查系统状态

运行完整的系统检查：
```bash
# 运行环境检查脚本
source scripts/setup/setup_environment.sh

# 运行基础测试
python3 test/hardware/test_unitree_connection.py
```

### 收集诊断信息

当需要技术支持时，请提供：

```bash
# 系统信息
uname -a
lsb_release -a
python3 --version

# ROS信息
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION
ros2 doctor

# 网络信息
ip addr show
ping -c 3 192.168.1.120

# 错误日志
tail -50 logs/errors/latest_error.log
```

## 📚 相关文档

- [环境配置指南](../guides/environment_setup.md)
- [测试运行指南](../guides/testing_guide.md)
- [任务状态查看](../tasks/README.md)

---

**文档更新时间**: 2025-06-26 18:40:00  
**维护团队**: Claudia Development Team 