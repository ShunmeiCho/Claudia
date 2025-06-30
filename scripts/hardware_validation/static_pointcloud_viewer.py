#!/usr/bin/env python3
"""
静态点云图像生成器
用途: 在SSH环境下生成点云可视化图像
生成时间: 2025-06-27
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import struct
from datetime import datetime
import os

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Ubuntu']
plt.rcParams['axes.unicode_minus'] = False

def read_points_from_cloud(cloud_msg, field_names=None, skip_nans=True):
    """从PointCloud2消息中读取点数据"""
    if field_names is None:
        field_names = ['x', 'y', 'z']
    
    points = []
    point_step = cloud_msg.point_step
    
    for i in range(0, len(cloud_msg.data), point_step):
        point_data = cloud_msg.data[i:i+point_step]
        point = {}
        
        for field in cloud_msg.fields:
            if field.name in field_names:
                offset = field.offset
                if field.datatype == PointField.FLOAT32:
                    value = struct.unpack('f', point_data[offset:offset+4])[0]
                else:
                    continue
                
                if skip_nans and isinstance(value, float) and np.isnan(value):
                    break
                    
                point[field.name] = value
        
        if len(point) == len(field_names):
            points.append([point[name] for name in field_names])
    
    return np.array(points) if points else np.array([]).reshape(0, len(field_names))

class StaticPointCloudViewer(Node):
    def __init__(self):
        super().__init__('static_pointcloud_viewer')
        self.get_logger().info("🎨 静态点云图像生成器启动")
        
        # 确保输出目录存在
        self.output_dir = "logs/pointcloud_images"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.pointcloud_callback,
            10
        )
        
        self.image_count = 0
        self.max_images = 5  # 生成5张图像
        
        print(f"📡 订阅话题: /utlidar/cloud")
        print(f"📁 输出目录: {self.output_dir}")
        print(f"🎯 将生成 {self.max_images} 张点云图像")
    
    def pointcloud_callback(self, msg):
        """处理点云数据并生成图像"""
        if self.image_count >= self.max_images:
            return
            
        try:
            # 解析点云数据
            points = read_points_from_cloud(msg, ['x', 'y', 'z'])
            
            if len(points) == 0:
                self.get_logger().warn("⚠️ 空点云数据")
                return
            
            # 生成多视角图像
            self.generate_images(points, self.image_count + 1)
            self.image_count += 1
            
            print(f"✅ 生成第 {self.image_count} 张图像 ({len(points)} 个点)")
            
            if self.image_count >= self.max_images:
                print(f"🎉 已生成所有 {self.max_images} 张图像!")
                print(f"📁 图像保存在: {self.output_dir}/")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"❌ 生成图像失败: {e}")
    
    def generate_images(self, points, frame_num):
        """生成多视角点云图像"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 提取坐标
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        
        # 计算距离用于颜色映射
        distances = np.sqrt(x**2 + y**2 + z**2)
        
        # 创建2x2子图
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(f'Unitree Go2 LiDAR L1 Point Cloud - Frame {frame_num}\\n'
                    f'{len(points)} points, Generated: {timestamp}', 
                    fontsize=14, fontweight='bold')
        
        # 顶视图 (XY平面)
        ax1 = axes[0, 0]
        scatter1 = ax1.scatter(x, y, c=distances, cmap='viridis', s=1, alpha=0.7)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Top View (XY Plane)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        plt.colorbar(scatter1, ax=ax1, label='Distance (m)', shrink=0.8)
        
        # 侧视图 (XZ平面)
        ax2 = axes[0, 1]
        scatter2 = ax2.scatter(x, z, c=distances, cmap='plasma', s=1, alpha=0.7)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('Side View (XZ Plane)')
        ax2.grid(True, alpha=0.3)
        plt.colorbar(scatter2, ax=ax2, label='Distance (m)', shrink=0.8)
        
        # 前视图 (YZ平面)
        ax3 = axes[1, 0]
        scatter3 = ax3.scatter(y, z, c=distances, cmap='coolwarm', s=1, alpha=0.7)
        ax3.set_xlabel('Y (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('Front View (YZ Plane)')
        ax3.grid(True, alpha=0.3)
        plt.colorbar(scatter3, ax=ax3, label='Distance (m)', shrink=0.8)
        
        # 距离分布直方图
        ax4 = axes[1, 1]
        ax4.hist(distances, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax4.set_xlabel('Distance (m)')
        ax4.set_ylabel('Point Count')
        ax4.set_title('Distance Distribution')
        ax4.grid(True, alpha=0.3)
        
        # 添加统计信息
        stats_text = f'Points: {len(points)}\\n'
        stats_text += f'Range X: {x.min():.2f} ~ {x.max():.2f}m\\n'
        stats_text += f'Range Y: {y.min():.2f} ~ {y.max():.2f}m\\n'
        stats_text += f'Range Z: {z.min():.2f} ~ {z.max():.2f}m\\n'
        stats_text += f'Avg Dist: {distances.mean():.2f}m'
        
        ax4.text(0.02, 0.98, stats_text, transform=ax4.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', 
                facecolor='wheat', alpha=0.8), fontsize=9)
        
        plt.tight_layout()
        
        # 保存图像
        filename = f"{self.output_dir}/pointcloud_frame_{frame_num:03d}_{timestamp}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"💾 图像已保存: {filename}")

def main():
    rclpy.init()
    
    print("🎨 启动静态点云图像生成器...")
    print("📡 正在等待点云数据...")
    
    viewer = StaticPointCloudViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print("\\n⏹️ 用户中断")
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        print("👋 程序退出")

if __name__ == '__main__':
    main() 