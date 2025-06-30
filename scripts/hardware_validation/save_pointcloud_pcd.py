#!/usr/bin/env python3
"""
点云PCD文件保存器
用途: 将ROS2点云数据保存为PCD格式，供下载和本地查看
生成时间: 2025-06-27
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from datetime import datetime
import os

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

def save_pcd_file(points, filename, frame_id="utlidar_lidar"):
    """保存点云数据为PCD格式"""
    if len(points) == 0:
        print("⚠️ 空点云数据，跳过保存")
        return False
    
    header = f"""# .PCD v0.7 - Point Cloud library
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
    
    try:
        with open(filename, 'w') as f:
            f.write(header)
            for point in points:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
        
        return True
    except Exception as e:
        print(f"❌ 保存PCD文件失败: {e}")
        return False

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.get_logger().info("💾 点云PCD保存器启动")
        
        # 确保输出目录存在
        self.output_dir = "logs/pointcloud_pcd"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.pointcloud_callback,
            10
        )
        
        self.file_count = 0
        self.max_files = 3  # 保存3个PCD文件
        
        print(f"📡 订阅话题: /utlidar/cloud")
        print(f"📁 输出目录: {self.output_dir}")
        print(f"🎯 将保存 {self.max_files} 个PCD文件")
    
    def pointcloud_callback(self, msg):
        """处理点云数据并保存PCD文件"""
        if self.file_count >= self.max_files:
            return
            
        try:
            # 解析点云数据
            points = read_points_from_cloud(msg, ['x', 'y', 'z'])
            
            if len(points) == 0:
                self.get_logger().warn("⚠️ 空点云数据")
                return
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.output_dir}/unitree_go2_lidar_{timestamp}_{self.file_count+1:03d}.pcd"
            
            # 保存PCD文件
            if save_pcd_file(points, filename):
                self.file_count += 1
                print(f"✅ 保存第 {self.file_count} 个PCD文件: {filename}")
                print(f"   包含 {len(points)} 个点")
                
                # 显示统计信息
                x, y, z = points[:, 0], points[:, 1], points[:, 2]
                distances = np.sqrt(x**2 + y**2 + z**2)
                print(f"   范围: X[{x.min():.2f}, {x.max():.2f}] "
                      f"Y[{y.min():.2f}, {y.max():.2f}] "
                      f"Z[{z.min():.2f}, {z.max():.2f}]")
                print(f"   平均距离: {distances.mean():.2f}m")
                
                if self.file_count >= self.max_files:
                    print(f"\n🎉 已保存所有 {self.max_files} 个PCD文件!")
                    print(f"📁 文件位置: {self.output_dir}/")
                    print(f"💡 可以下载这些文件用以下工具查看:")
                    print(f"   - CloudCompare (推荐)")
                    print(f"   - PCL Viewer: pcl_viewer filename.pcd")
                    print(f"   - MeshLab")
                    print(f"   - Open3D Python")
                    rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"❌ 处理点云数据失败: {e}")

def main():
    rclpy.init()
    
    print("💾 启动点云PCD保存器...")
    print("📡 正在等待点云数据...")
    
    saver = PointCloudSaver()
    
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        print("\n⏹️ 用户中断")
    finally:
        saver.destroy_node()
        rclpy.shutdown()
        print("👋 程序退出")

if __name__ == '__main__':
    main() 