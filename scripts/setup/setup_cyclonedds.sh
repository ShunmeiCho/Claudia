#!/bin/bash
# Claudia项目 CycloneDDS 环境配置脚本
# 解决 "undefined symbol: ddsi_sertype_v0" 兼容性问题
# 自动生成于: 2025-06-27
# 最新！！！

echo "🔧 配置Unitree CycloneDDS环境..."

# 检查CycloneDDS安装是否存在
CYCLONEDDS_INSTALL_PATH="$HOME/cyclonedds/install"
if [ ! -d "$CYCLONEDDS_INSTALL_PATH" ]; then
    echo "❌ 错误: CycloneDDS未找到在 $CYCLONEDDS_INSTALL_PATH"
    echo "请先编译安装CycloneDDS 0.10.x版本"
    echo "参考: https://github.com/eclipse-cyclonedds/cyclonedds"
    exit 1
fi

# 设置CycloneDDS环境变量
export CYCLONEDDS_HOME="$CYCLONEDDS_INSTALL_PATH"
export LD_LIBRARY_PATH="$CYCLONEDDS_INSTALL_PATH/lib:$LD_LIBRARY_PATH"

# 设置ROS2 DDS实现为CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 验证关键库文件存在
LIBDDSC_PATH="$CYCLONEDDS_INSTALL_PATH/lib/libddsc.so"
if [ ! -f "$LIBDDSC_PATH" ]; then
    echo "⚠️ 警告: 核心库文件不存在: $LIBDDSC_PATH"
    echo "可能需要重新编译CycloneDDS"
fi

echo "✅ CycloneDDS环境配置完成"
echo "   CYCLONEDDS_HOME: $CYCLONEDDS_HOME" 
echo "   LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# 可选：测试unitree_sdk2py导入
if [ "$1" = "--test" ]; then
    echo ""
    echo "🧪 测试unitree_sdk2py导入..."
    python3 -c "
try:
    import unitree_sdk2py
    from unitree_sdk2py.core.channel import ChannelSubscriber
    print('   ✅ unitree_sdk2py导入成功')
    print('   ✅ 核心通信模块可用')
except Exception as e:
    print(f'   ❌ 导入失败: {e}')
    print('   请检查CycloneDDS配置或重新安装unitree_sdk2py')
"
fi

echo ""
echo "💡 使用方法:"
echo "   source scripts/setup/setup_cyclonedds.sh        # 配置环境"
echo "   source scripts/setup/setup_cyclonedds.sh --test # 配置并测试" 