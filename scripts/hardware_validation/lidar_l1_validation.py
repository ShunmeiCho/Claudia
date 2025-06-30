#!/usr/bin/env python3
"""
4D LiDAR L1 综合验证脚本
Author: Claudia Robot Project
Date: 2025-01-27
Purpose: 验证Unitree Go2 R&D Plus的4D LiDAR L1传感器性能

功能包括:
- 点云数据流获取和完整性检查
- 距离精度测试(与已知距离对比)  
- 21600点/秒数据率验证
- 3D可视化和质量评估
- 综合验证报告生成
"""

import time
import json
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
# 注释掉unitree_sdk2py，直接使用ROS2话题
# from unitree_sdk2py import Go2Robot
import matplotlib.pyplot as plt
from datetime import datetime
import os

# 尝试导入Open3D进行3D可视化
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
    print("✅ Open3D可用于3D可视化")
except ImportError:
    OPEN3D_AVAILABLE = False
    print("⚠️ Open3D不可用，将使用matplotlib进行2D可视化")

def read_points_from_cloud(cloud_msg, field_names=None, skip_nans=True):
    """
    从PointCloud2消息中读取点云数据
    这是sensor_msgs_py.point_cloud2.read_points的替代实现
    """
    if field_names is None:
        field_names = [field.name for field in cloud_msg.fields]
    
    # 建立字段映射
    field_map = {}
    for i, field in enumerate(cloud_msg.fields):
        if field.name in field_names:
            field_map[field.name] = (field.offset, field.datatype)
    
    # 解析点云数据
    point_step = cloud_msg.point_step
    row_step = cloud_msg.row_step
    data = cloud_msg.data
    
    points = []
    for v in range(cloud_msg.height):
        for u in range(cloud_msg.width):
            # 计算数据偏移
            offset = v * row_step + u * point_step
            point_data = {}
            
            for field_name, (field_offset, datatype) in field_map.items():
                idx = offset + field_offset
                
                # 根据数据类型解析
                if datatype == PointField.FLOAT32:
                    value = struct.unpack('f', data[idx:idx+4])[0]
                elif datatype == PointField.FLOAT64:
                    value = struct.unpack('d', data[idx:idx+8])[0]
                elif datatype == PointField.UINT32:
                    value = struct.unpack('I', data[idx:idx+4])[0]
                elif datatype == PointField.INT32:
                    value = struct.unpack('i', data[idx:idx+4])[0]
                elif datatype == PointField.UINT16:
                    value = struct.unpack('H', data[idx:idx+2])[0]
                elif datatype == PointField.INT16:
                    value = struct.unpack('h', data[idx:idx+2])[0]
                elif datatype == PointField.UINT8:
                    value = struct.unpack('B', data[idx:idx+1])[0]
                elif datatype == PointField.INT8:
                    value = struct.unpack('b', data[idx:idx+1])[0]
                else:
                    value = 0
                
                # 检查是否为NaN
                if skip_nans and isinstance(value, float) and np.isnan(value):
                    continue
                    
                point_data[field_name] = value
            
            # 只有当所有字段都存在时才添加点
            if len(point_data) == len(field_names):
                if field_names == ["x", "y", "z", "intensity"]:
                    points.append((point_data["x"], point_data["y"], point_data["z"], point_data["intensity"]))
                else:
                    points.append(tuple(point_data[name] for name in field_names))
    
    return points

class LiDARValidator(Node):
    """LiDAR L1验证器类"""
    
    def __init__(self):
        super().__init__('lidar_l1_validator')
        
        # 初始化数据存储
        self.point_count = 0
        self.start_time = time.time()
        self.point_cloud_data = []
        self.data_rate_history = []
        self.timestamps = []
        
        # 验证结果存储
        self.validation_results = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'hardware': 'Unitree Go2 R&D Plus 4D LiDAR L1',
            'test_environment': {
                'indoor': True,
                'lighting': 'normal',
                'temperature': 'room_temp'
            }
        }
        
        print("🚀 LiDAR L1验证器初始化完成")
        print("=" * 60)
        
    def test_basic_connection(self):
        """测试基础连接"""
        print("🔌 测试基础连接...")
        
        try:
            # 跳过unitree_sdk2py连接，直接使用ROS2
            print("✅ 使用ROS2话题模式，跳过SDK连接")
            
            # 创建ROS2话题订阅器用于LiDAR数据
            # Unitree机器人的LiDAR话题名称
            lidar_topic = '/utlidar/cloud'
            
            self.lidar_subscription = self.create_subscription(
                PointCloud2,
                lidar_topic,
                self.lidar_callback,
                10
            )
            print(f"✅ LiDAR话题订阅成功: {lidar_topic}")
            print("📡 开始监听Unitree LiDAR数据...")
            
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def lidar_callback(self, msg):
        """LiDAR数据回调函数"""
        current_time = time.time()
        
        try:
            # 使用自定义函数提取点云数据
            points = read_points_from_cloud(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)
            point_count = len(points)
            
            # 记录数据
            self.point_count += point_count
            self.data_rate_history.append(point_count)
            self.timestamps.append(current_time)
            
            # 保存最新的点云数据用于分析
            if len(self.point_cloud_data) < 100:  # 保存最近100帧
                self.point_cloud_data.append({
                    'timestamp': current_time,
                    'points': points[:1000],  # 只保存前1000个点以节省内存
                    'total_count': point_count
                })
            else:
                # 滚动更新
                self.point_cloud_data.pop(0)
                self.point_cloud_data.append({
                    'timestamp': current_time,
                    'points': points[:1000],
                    'total_count': point_count
                })
                
            # 实时显示数据接收状态
            if len(self.data_rate_history) % 10 == 0:  # 每10帧显示一次
                print(f"📡 接收数据: {point_count} 点/帧, 总计: {self.point_count} 点")
                
        except Exception as e:
            self.get_logger().error(f"点云数据处理错误: {e}")
    
    def validate_data_rate(self, duration=10):
        """
        验证LiDAR数据率是否达到21600点/秒
        
        Args:
            duration: 测试持续时间(秒)
        
        Returns:
            dict: 包含实际数据率、稳定性等指标
        """
        print(f"📊 开始{duration}秒数据率测试...")
        
        # 清空之前的数据
        self.point_count = 0
        self.data_rate_history = []
        self.timestamps = []
        start_time = time.time()
        
        print("   等待LiDAR数据...")
        
        # 启动ROS2事件循环进行数据收集
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 显示进度
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and len(self.data_rate_history) > 0:
                current_rate = self.point_count / elapsed
                print(f"   进度: {elapsed:.1f}/{duration}s, 当前速率: {current_rate:,.0f} 点/秒")
        
        # 计算统计结果
        if len(self.data_rate_history) == 0:
            print("❌ 未收到任何LiDAR数据")
            print("💡 请检查:")
            print("   1. 机器人是否已连接并开机")
            print("   2. LiDAR话题名称是否正确")
            print("   3. 网络配置是否正确")
            return None
        
        total_time = time.time() - start_time
        actual_rate = self.point_count / total_time
        target_rate = 21600
        
        # 计算数据稳定性
        rates_per_second = []
        if len(self.timestamps) > 1:
            for i in range(len(self.timestamps)-1):
                time_diff = self.timestamps[i+1] - self.timestamps[i]
                if time_diff > 0:
                    rate = self.data_rate_history[i+1] / time_diff
                    rates_per_second.append(rate)
        
        results = {
            'actual_rate': actual_rate,
            'target_rate': target_rate,
            'deviation_percent': abs(actual_rate - target_rate) / target_rate * 100,
            'stability_std': np.std(rates_per_second) if rates_per_second else 0,
            'total_samples': len(self.data_rate_history),
            'test_duration': total_time
        }
        
        print(f"📈 数据率测试结果:")
        print(f"   目标速率: {target_rate:,} 点/秒")
        print(f"   实际速率: {actual_rate:,.1f} 点/秒")
        print(f"   偏差: {results['deviation_percent']:.2f}%")
        print(f"   稳定性(标准差): {results['stability_std']:.1f}")
        print(f"   样本数: {results['total_samples']}")
        
        return results
    
    def validate_distance_accuracy(self, test_distances=[0.5, 1.0, 2.0, 3.0]):
        """
        使用已知距离的标准目标测试距离精度
        
        Args:
            test_distances: 测试距离列表(米)
        
        Returns:
            dict: 距离精度测试结果
        """
        print("🎯 开始距离精度测试...")
        print("请在以下距离放置高反射率标准板(如白色纸板):")
        
        accuracy_results = {}
        
        for target_distance in test_distances:
            print(f"\n📏 测试距离: {target_distance}m")
            input("   请将标准板放置在指定距离，然后按Enter继续...")
            
            # 收集多次测量数据
            measurements = []
            print("   正在收集数据...")
            
            for i in range(10):  # 10次测量取平均
                # 获取最新的点云数据
                rclpy.spin_once(self, timeout_sec=0.5)
                
                if self.point_cloud_data:
                    latest_data = self.point_cloud_data[-1]
                    points = latest_data['points']
                    
                    # 寻找最近的强反射点(标准板)
                    distances = []
                    for point in points:
                        x, y, z, intensity = point
                        if intensity > 200:  # 高反射率阈值
                            distance = np.sqrt(x**2 + y**2 + z**2)
                            if 0.1 < distance < 10:  # 合理距离范围
                                distances.append(distance)
                    
                    if distances:
                        measured_distance = min(distances)  # 最近的高反射点
                        measurements.append(measured_distance)
                        print(f"     测量 {i+1}: {measured_distance:.3f}m")
                
                time.sleep(0.2)
            
            if measurements:
                mean_measured = np.mean(measurements)
                std_measured = np.std(measurements)
                error = abs(mean_measured - target_distance)
                error_percent = error / target_distance * 100
                
                accuracy_results[target_distance] = {
                    'measured_mean': mean_measured,
                    'measured_std': std_measured,
                    'absolute_error': error,
                    'relative_error_percent': error_percent,
                    'measurements': measurements
                }
                
                print(f"   📊 结果: {mean_measured:.3f}±{std_measured:.3f}m")
                print(f"   📉 误差: {error:.3f}m ({error_percent:.2f}%)")
                
                # 评估精度等级
                if error_percent < 2:
                    print("   🏆 精度等级: 优秀")
                elif error_percent < 5:
                    print("   ✅ 精度等级: 良好") 
                elif error_percent < 10:
                    print("   ⚠️ 精度等级: 一般")
                else:
                    print("   ❌ 精度等级: 需要校准")
            else:
                print("   ❌ 未能检测到标准板，请检查放置位置和反射率")
        
        return accuracy_results
    
    def setup_3d_visualization(self):
        """设置3D可视化"""
        if not OPEN3D_AVAILABLE:
            print("⚠️ Open3D不可用，跳过3D可视化")
            return False
            
        print("🎨 启动3D点云可视化...")
        print("   按ESC键退出可视化")
        
        try:
            # 创建Open3D可视化器
            vis = o3d.visualization.Visualizer()
            vis.create_window("LiDAR L1 实时点云", width=1200, height=800)
            
            # 创建点云对象
            pcd = o3d.geometry.PointCloud()
            vis.add_geometry(pcd)
            
            # 设置视角
            view_control = vis.get_view_control()
            view_control.set_front([0, 0, 1])
            view_control.set_up([0, 1, 0])
            
            visualization_start = time.time()
            frame_count = 0
            
            while True:
                # 获取最新点云数据
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.point_cloud_data:
                    latest_data = self.point_cloud_data[-1]
                    points = latest_data['points']
                    
                    if points:
                        # 转换为numpy数组
                        point_array = np.array([[p[0], p[1], p[2]] for p in points])
                        intensity_array = np.array([p[3] for p in points])
                        
                        # 根据强度生成颜色
                        colors = np.zeros((len(point_array), 3))
                        normalized_intensity = intensity_array / 255.0
                        colors[:, 0] = normalized_intensity  # 红色通道
                        colors[:, 1] = 0.5  # 绿色通道
                        colors[:, 2] = 1.0 - normalized_intensity  # 蓝色通道
                        
                        # 更新点云
                        pcd.points = o3d.utility.Vector3dVector(point_array)
                        pcd.colors = o3d.utility.Vector3dVector(colors)
                        
                        # 刷新可视化
                        vis.update_geometry(pcd)
                        
                        frame_count += 1
                
                if not vis.poll_events():
                    break
                vis.update_renderer()
                
                time.sleep(0.05)  # 20Hz刷新率
                
                # 显示运行时间
                if frame_count % 60 == 0:
                    elapsed = time.time() - visualization_start
                    print(f"   可视化运行时间: {elapsed:.1f}秒, 帧数: {frame_count}")
            
            vis.destroy_window()
            print("✅ 3D可视化已关闭")
            return True
            
        except Exception as e:
            print(f"❌ 3D可视化错误: {e}")
            return False
    
    def setup_2d_visualization(self):
        """设置2D可视化作为备选方案"""
        print("📊 启动2D点云可视化...")
        
        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
            
            visualization_start = time.time()
            frame_count = 0
            
            while frame_count < 100:  # 显示100帧然后停止
                # 获取最新点云数据
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.point_cloud_data:
                    latest_data = self.point_cloud_data[-1]
                    points = latest_data['points']
                    
                    if points and len(points) > 10:
                        # 提取坐标和强度
                        x = [p[0] for p in points]
                        y = [p[1] for p in points]
                        z = [p[2] for p in points]
                        intensity = [p[3] for p in points]
                        
                        # 清空并绘制XY平面
                        ax1.clear()
                        scatter1 = ax1.scatter(x, y, c=intensity, cmap='viridis', s=1)
                        ax1.set_xlabel('X (m)')
                        ax1.set_ylabel('Y (m)')
                        ax1.set_title(f'LiDAR XY平面视图 (帧{frame_count})')
                        ax1.grid(True)
                        ax1.axis('equal')
                        
                        # 清空并绘制XZ平面
                        ax2.clear()
                        scatter2 = ax2.scatter(x, z, c=intensity, cmap='viridis', s=1)
                        ax2.set_xlabel('X (m)')
                        ax2.set_ylabel('Z (m)')
                        ax2.set_title(f'LiDAR XZ平面视图 (帧{frame_count})')
                        ax2.grid(True)
                        
                        plt.pause(0.1)
                        frame_count += 1
                
                time.sleep(0.1)
            
            plt.show()
            print("✅ 2D可视化完成")
            return True
            
        except Exception as e:
            print(f"❌ 2D可视化错误: {e}")
            return False
    
    def generate_validation_report(self, data_rate_results, accuracy_results):
        """生成综合验证报告"""
        print("📋 生成验证报告...")
        
        self.validation_results.update({
            'data_rate_test': data_rate_results,
            'distance_accuracy_test': accuracy_results,
        })
        
        # 评估整体性能
        overall_status = "PASS"
        recommendations = []
        
        # 评估数据率
        if data_rate_results and data_rate_results['deviation_percent'] < 10:
            self.validation_results['data_rate_status'] = 'PASS'
        else:
            self.validation_results['data_rate_status'] = 'FAIL'
            overall_status = "FAIL"
            recommendations.append('检查数据传输带宽和网络配置')
        
        # 评估距离精度
        if accuracy_results:
            avg_error = np.mean([r['relative_error_percent'] for r in accuracy_results.values()])
            if avg_error < 5:
                self.validation_results['accuracy_status'] = 'PASS'
            else:
                self.validation_results['accuracy_status'] = 'FAIL'
                overall_status = "FAIL" 
                recommendations.append('需要进行距离校准')
        else:
            self.validation_results['accuracy_status'] = 'NOT_TESTED'
            recommendations.append('建议完成距离精度测试')
        
        self.validation_results['overall_status'] = overall_status
        self.validation_results['recommendations'] = recommendations
        
        # 保存报告
        os.makedirs('logs', exist_ok=True)
        report_path = f"logs/lidar_l1_validation_{int(time.time())}.json"
        
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(self.validation_results, f, indent=2, ensure_ascii=False)
        
        # 打印报告摘要
        print("\n" + "="*60)
        print("📋 LiDAR L1验证报告摘要")
        print("="*60)
        print(f"测试时间: {self.validation_results['timestamp']}")
        print(f"硬件设备: {self.validation_results['hardware']}")
        print(f"总体状态: {overall_status}")
        
        if data_rate_results:
            print(f"\n📊 数据率测试:")
            print(f"   状态: {self.validation_results['data_rate_status']}")
            print(f"   实际速率: {data_rate_results['actual_rate']:,.1f} 点/秒")
            print(f"   目标速率: {data_rate_results['target_rate']:,} 点/秒")
            print(f"   偏差: {data_rate_results['deviation_percent']:.2f}%")
        
        if accuracy_results:
            print(f"\n🎯 距离精度测试:")
            print(f"   状态: {self.validation_results['accuracy_status']}")
            for distance, result in accuracy_results.items():
                print(f"   {distance}m: 误差 {result['relative_error_percent']:.2f}%")
        
        if recommendations:
            print(f"\n💡 建议:")
            for i, rec in enumerate(recommendations, 1):
                print(f"   {i}. {rec}")
        
        print(f"\n📁 详细报告已保存: {report_path}")
        print("="*60)
        
        return report_path

def main():
    """主函数"""
    print("🤖 Unitree Go2 R&D Plus - 4D LiDAR L1 验证程序")
    print("=" * 60)
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建验证器
        validator = LiDARValidator()
        
        # 测试基础连接
        if not validator.test_basic_connection():
            print("❌ 基础连接测试失败，请检查机器人连接和ROS2环境")
            return
        
        print("\n🚀 开始LiDAR L1验证流程...")
        
        # 1. 数据率验证
        print("\n" + "="*40)
        data_rate_results = validator.validate_data_rate(duration=15)
        
        # 2. 距离精度测试
        print("\n" + "="*40)
        choice = input("是否进行距离精度测试? (y/n): ").lower().strip()
        if choice == 'y':
            accuracy_results = validator.validate_distance_accuracy()
        else:
            accuracy_results = {}
            print("⏭️ 跳过距离精度测试")
        
        # 3. 可视化测试
        print("\n" + "="*40)
        choice = input("是否进行3D可视化测试? (y/n): ").lower().strip()
        if choice == 'y':
            if not validator.setup_3d_visualization():
                print("尝试2D可视化...")
                validator.setup_2d_visualization()
        else:
            print("⏭️ 跳过可视化测试")
        
        # 4. 生成报告
        print("\n" + "="*40)
        report_path = validator.generate_validation_report(data_rate_results, accuracy_results)
        
        print("\n🎉 LiDAR L1验证完成!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断验证程序")
    except Exception as e:
        print(f"\n❌ 验证程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理ROS2
        rclpy.shutdown()
        print("👋 程序退出")

if __name__ == "__main__":
    main() 