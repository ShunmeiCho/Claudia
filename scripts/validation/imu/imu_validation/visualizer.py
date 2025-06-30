#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/visualizer.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU实时数据可视化

import time
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import logging
from typing import Dict, List, Tuple, Any, Optional
from collections import deque
import queue

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector

class IMUVisualizer:
    """IMU数据实时可视化器"""
    
    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        初始化可视化器
        
        Args:
            imu_config: IMU配置实例
            data_collector: 数据采集器实例
            config: 验证配置
        """
        self.imu_config = imu_config
        self.data_collector = data_collector
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # 可视化配置
        self.viz_config = config.get("visualization", {})
        self.enable_real_time = self.viz_config.get("real_time_plots", True)
        self.update_interval = self.viz_config.get("plot_update_interval_ms", 100)
        self.max_points = self.viz_config.get("max_plot_points", 500)
        self.enable_3d = self.viz_config.get("enable_3d_orientation", True)
        
        # 数据缓存
        self.plot_data = {
            'timestamps': deque(maxlen=self.max_points),
            'accelerometer': {
                'x': deque(maxlen=self.max_points),
                'y': deque(maxlen=self.max_points),
                'z': deque(maxlen=self.max_points),
                'magnitude': deque(maxlen=self.max_points)
            },
            'gyroscope': {
                'x': deque(maxlen=self.max_points),
                'y': deque(maxlen=self.max_points),
                'z': deque(maxlen=self.max_points),
                'magnitude': deque(maxlen=self.max_points)
            },
            'orientation': {
                'roll': deque(maxlen=self.max_points),
                'pitch': deque(maxlen=self.max_points),
                'yaw': deque(maxlen=self.max_points)
            },
            'quaternion': {
                'w': deque(maxlen=self.max_points),
                'x': deque(maxlen=self.max_points),
                'y': deque(maxlen=self.max_points),
                'z': deque(maxlen=self.max_points)
            }
        }
        
        # 可视化状态
        self.is_plotting = False
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.animation = None
        
        # 数据更新队列
        self.data_queue = queue.Queue(maxsize=100)
        
        # 3D姿态可视化
        self.orientation_fig = None
        self.orientation_ax = None
        self.orientation_lines = {}
        
    def start_visualization(self, mode: str = "all") -> bool:
        """
        启动可视化
        
        Args:
            mode: 可视化模式 ("all", "time_series", "3d_orientation")
            
        Returns:
            bool: 启动是否成功
        """
        try:
            if not self.enable_real_time:
                self.logger.info("实时可视化已禁用")
                return False
            
            if self.is_plotting:
                self.logger.warning("可视化已在运行中")
                return False
            
            # 注册数据回调
            self.data_collector.add_data_callback(self._data_callback)
            
            if mode == "all":
                success = self._start_time_series_plot() and (self._start_3d_orientation() if self.enable_3d else True)
            elif mode == "time_series":
                success = self._start_time_series_plot()
            elif mode == "3d_orientation":
                success = self._start_3d_orientation()
            else:
                self.logger.error(f"未知的可视化模式: {mode}")
                return False
            
            if success:
                self.is_plotting = True
                self.logger.info(f"IMU可视化已启动，模式: {mode}")
                
            return success
            
        except Exception as e:
            self.logger.error(f"启动可视化失败: {e}")
            return False
    
    def stop_visualization(self) -> bool:
        """停止可视化"""
        try:
            self.is_plotting = False
            
            # 移除数据回调
            self.data_collector.remove_data_callback(self._data_callback)
            
            # 停止动画
            if self.animation:
                self.animation.event_source.stop()
                self.animation = None
            
            # 关闭图形窗口
            if self.fig:
                plt.close(self.fig)
                self.fig = None
                
            if self.orientation_fig:
                plt.close(self.orientation_fig)
                self.orientation_fig = None
            
            self.logger.info("IMU可视化已停止")
            
            return True
            
        except Exception as e:
            self.logger.error(f"停止可视化失败: {e}")
            return False
    
    def _data_callback(self, reading: IMUReading):
        """数据回调函数"""
        try:
            # 将数据放入队列，避免阻塞数据采集
            if not self.data_queue.full():
                self.data_queue.put(reading, block=False)
        except queue.Full:
            pass  # 队列满时丢弃数据
        except Exception as e:
            self.logger.error(f"数据回调处理失败: {e}")
    
    def _process_data_queue(self):
        """处理数据队列"""
        try:
            while not self.data_queue.empty():
                reading = self.data_queue.get(block=False)
                self._update_plot_data(reading)
        except queue.Empty:
            pass
        except Exception as e:
            self.logger.error(f"处理数据队列失败: {e}")
    
    def _update_plot_data(self, reading: IMUReading):
        """更新绘图数据"""
        try:
            # 添加时间戳
            self.plot_data['timestamps'].append(reading.timestamp)
            
            # 添加加速度计数据
            self.plot_data['accelerometer']['x'].append(reading.accelerometer[0])
            self.plot_data['accelerometer']['y'].append(reading.accelerometer[1])
            self.plot_data['accelerometer']['z'].append(reading.accelerometer[2])
            self.plot_data['accelerometer']['magnitude'].append(np.linalg.norm(reading.accelerometer))
            
            # 添加陀螺仪数据
            self.plot_data['gyroscope']['x'].append(reading.gyroscope[0])
            self.plot_data['gyroscope']['y'].append(reading.gyroscope[1])
            self.plot_data['gyroscope']['z'].append(reading.gyroscope[2])
            self.plot_data['gyroscope']['magnitude'].append(np.linalg.norm(reading.gyroscope))
            
            # 添加四元数数据
            self.plot_data['quaternion']['w'].append(reading.quaternion[0])
            self.plot_data['quaternion']['x'].append(reading.quaternion[1])
            self.plot_data['quaternion']['y'].append(reading.quaternion[2])
            self.plot_data['quaternion']['z'].append(reading.quaternion[3])
            
            # 转换为欧拉角
            roll, pitch, yaw = self.imu_config.quaternion_to_euler(reading.quaternion)
            self.plot_data['orientation']['roll'].append(np.degrees(roll))
            self.plot_data['orientation']['pitch'].append(np.degrees(pitch))
            self.plot_data['orientation']['yaw'].append(np.degrees(yaw))
            
        except Exception as e:
            self.logger.error(f"更新绘图数据失败: {e}")
    
    def _start_time_series_plot(self) -> bool:
        """启动时序图可视化"""
        try:
            # 创建主图形
            self.fig, self.axes = plt.subplots(3, 2, figsize=(15, 10))
            self.fig.suptitle('Unitree Go2 IMU Real-time Data', fontsize=16)
            
            # 子图布局：
            # [0,0]: 加速度计XYZ    [0,1]: 加速度计幅值
            # [1,0]: 陀螺仪XYZ      [1,1]: 陀螺仪幅值  
            # [2,0]: 欧拉角         [2,1]: 四元数
            
            # 初始化空数据线
            self.lines = {}
            
            # 加速度计XYZ
            ax = self.axes[0, 0]
            ax.set_title('Accelerometer (m/s²)')
            ax.set_ylabel('Acceleration')
            ax.grid(True, alpha=0.3)
            self.lines['accel_x'], = ax.plot([], [], 'r-', label='X', alpha=0.8)
            self.lines['accel_y'], = ax.plot([], [], 'g-', label='Y', alpha=0.8)
            self.lines['accel_z'], = ax.plot([], [], 'b-', label='Z', alpha=0.8)
            ax.legend(loc='upper right')
            
            # 加速度计幅值
            ax = self.axes[0, 1]
            ax.set_title('Accelerometer Magnitude')
            ax.set_ylabel('|Acceleration| (m/s²)')
            ax.grid(True, alpha=0.3)
            self.lines['accel_mag'], = ax.plot([], [], 'k-', linewidth=2)
            ax.axhline(y=9.81, color='r', linestyle='--', alpha=0.7, label='Gravity')
            ax.legend(loc='upper right')
            
            # 陀螺仪XYZ
            ax = self.axes[1, 0]
            ax.set_title('Gyroscope (rad/s)')
            ax.set_ylabel('Angular Velocity')
            ax.grid(True, alpha=0.3)
            self.lines['gyro_x'], = ax.plot([], [], 'r-', label='X', alpha=0.8)
            self.lines['gyro_y'], = ax.plot([], [], 'g-', label='Y', alpha=0.8)
            self.lines['gyro_z'], = ax.plot([], [], 'b-', label='Z', alpha=0.8)
            ax.legend(loc='upper right')
            
            # 陀螺仪幅值
            ax = self.axes[1, 1]
            ax.set_title('Gyroscope Magnitude')
            ax.set_ylabel('|Angular Velocity| (rad/s)')
            ax.grid(True, alpha=0.3)
            self.lines['gyro_mag'], = ax.plot([], [], 'k-', linewidth=2)
            
            # 欧拉角
            ax = self.axes[2, 0]
            ax.set_title('Euler Angles (degrees)')
            ax.set_ylabel('Angle')
            ax.set_xlabel('Time (s)')
            ax.grid(True, alpha=0.3)
            self.lines['roll'], = ax.plot([], [], 'r-', label='Roll', alpha=0.8)
            self.lines['pitch'], = ax.plot([], [], 'g-', label='Pitch', alpha=0.8)
            self.lines['yaw'], = ax.plot([], [], 'b-', label='Yaw', alpha=0.8)
            ax.legend(loc='upper right')
            
            # 四元数
            ax = self.axes[2, 1]
            ax.set_title('Quaternion')
            ax.set_ylabel('Value')
            ax.set_xlabel('Time (s)')
            ax.grid(True, alpha=0.3)
            self.lines['quat_w'], = ax.plot([], [], 'k-', label='W', alpha=0.8)
            self.lines['quat_x'], = ax.plot([], [], 'r-', label='X', alpha=0.8)
            self.lines['quat_y'], = ax.plot([], [], 'g-', label='Y', alpha=0.8)
            self.lines['quat_z'], = ax.plot([], [], 'b-', label='Z', alpha=0.8)
            ax.legend(loc='upper right')
            
            # 启动动画
            self.animation = animation.FuncAnimation(
                self.fig, 
                self._update_time_series_plot, 
                interval=self.update_interval,
                blit=False,
                cache_frame_data=False
            )
            
            plt.tight_layout()
            plt.show(block=False)
            
            return True
            
        except Exception as e:
            self.logger.error(f"启动时序图可视化失败: {e}")
            return False
    
    def _update_time_series_plot(self, frame):
        """更新时序图"""
        try:
            # 处理数据队列
            self._process_data_queue()
            
            if len(self.plot_data['timestamps']) < 2:
                return list(self.lines.values())
            
            # 计算相对时间（秒）
            timestamps = list(self.plot_data['timestamps'])
            if timestamps:
                start_time = timestamps[0]
                times = [(t - start_time) for t in timestamps]
            else:
                times = []
            
            # 更新加速度计数据
            self.lines['accel_x'].set_data(times, list(self.plot_data['accelerometer']['x']))
            self.lines['accel_y'].set_data(times, list(self.plot_data['accelerometer']['y']))
            self.lines['accel_z'].set_data(times, list(self.plot_data['accelerometer']['z']))
            self.lines['accel_mag'].set_data(times, list(self.plot_data['accelerometer']['magnitude']))
            
            # 更新陀螺仪数据
            self.lines['gyro_x'].set_data(times, list(self.plot_data['gyroscope']['x']))
            self.lines['gyro_y'].set_data(times, list(self.plot_data['gyroscope']['y']))
            self.lines['gyro_z'].set_data(times, list(self.plot_data['gyroscope']['z']))
            self.lines['gyro_mag'].set_data(times, list(self.plot_data['gyroscope']['magnitude']))
            
            # 更新欧拉角数据
            self.lines['roll'].set_data(times, list(self.plot_data['orientation']['roll']))
            self.lines['pitch'].set_data(times, list(self.plot_data['orientation']['pitch']))
            self.lines['yaw'].set_data(times, list(self.plot_data['orientation']['yaw']))
            
            # 更新四元数数据
            self.lines['quat_w'].set_data(times, list(self.plot_data['quaternion']['w']))
            self.lines['quat_x'].set_data(times, list(self.plot_data['quaternion']['x']))
            self.lines['quat_y'].set_data(times, list(self.plot_data['quaternion']['y']))
            self.lines['quat_z'].set_data(times, list(self.plot_data['quaternion']['z']))
            
            # 更新坐标轴范围
            if times:
                time_range = max(times) - min(times)
                time_window = max(30.0, time_range)  # 至少30秒窗口
                
                for i in range(3):
                    for j in range(2):
                        ax = self.axes[i, j]
                        ax.set_xlim(max(times) - time_window, max(times))
                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)
            
            return list(self.lines.values())
            
        except Exception as e:
            self.logger.error(f"更新时序图失败: {e}")
            return list(self.lines.values())
    
    def _start_3d_orientation(self) -> bool:
        """启动3D姿态可视化"""
        try:
            if not self.enable_3d:
                return True
            
            # 创建3D图形
            self.orientation_fig = plt.figure(figsize=(10, 8))
            self.orientation_ax = self.orientation_fig.add_subplot(111, projection='3d')
            self.orientation_ax.set_title('IMU 3D Orientation (Robot Frame)', fontsize=14)
            
            # 设置坐标轴
            self.orientation_ax.set_xlabel('X (Forward)')
            self.orientation_ax.set_ylabel('Y (Left)')
            self.orientation_ax.set_zlabel('Z (Up)')
            
            # 设置坐标轴范围
            self.orientation_ax.set_xlim([-1.5, 1.5])
            self.orientation_ax.set_ylim([-1.5, 1.5])
            self.orientation_ax.set_zlim([-1.5, 1.5])
            
            # 创建坐标轴线（机器人坐标系）
            origin = [0, 0, 0]
            
            # X轴（前进方向，红色）
            self.orientation_lines['x_axis'] = self.orientation_ax.plot([origin[0], 1], [origin[1], 0], [origin[2], 0], 'r-', linewidth=3, label='X (Forward)')[0]
            
            # Y轴（左侧方向，绿色）
            self.orientation_lines['y_axis'] = self.orientation_ax.plot([origin[0], 0], [origin[1], 1], [origin[2], 0], 'g-', linewidth=3, label='Y (Left)')[0]
            
            # Z轴（向上方向，蓝色）
            self.orientation_lines['z_axis'] = self.orientation_ax.plot([origin[0], 0], [origin[1], 0], [origin[2], 1], 'b-', linewidth=3, label='Z (Up)')[0]
            
            # 添加参考网格
            self.orientation_ax.plot([0, 0], [0, 0], [-1.5, 1.5], 'k--', alpha=0.3)
            self.orientation_ax.plot([0, 0], [-1.5, 1.5], [0, 0], 'k--', alpha=0.3)
            self.orientation_ax.plot([-1.5, 1.5], [0, 0], [0, 0], 'k--', alpha=0.3)
            
            self.orientation_ax.legend()
            
            # 创建动画更新器
            def update_3d_orientation(frame):
                return self._update_3d_orientation()
            
            self.orientation_animation = animation.FuncAnimation(
                self.orientation_fig,
                update_3d_orientation,
                interval=self.update_interval,
                blit=False,
                cache_frame_data=False
            )
            
            plt.show(block=False)
            
            return True
            
        except Exception as e:
            self.logger.error(f"启动3D姿态可视化失败: {e}")
            return False
    
    def _update_3d_orientation(self):
        """更新3D姿态显示"""
        try:
            # 获取最新的IMU读数
            latest_reading = self.imu_config.get_latest_reading()
            
            if not latest_reading:
                return list(self.orientation_lines.values())
            
            # 提取四元数并归一化
            quat = np.array(latest_reading.quaternion)
            quat = quat / np.linalg.norm(quat)  # 确保归一化
            
            # 四元数转旋转矩阵
            w, x, y, z = quat
            
            # 旋转矩阵
            R = np.array([
                [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
            ])
            
            # 原始坐标轴向量
            x_axis = np.array([1, 0, 0])
            y_axis = np.array([0, 1, 0])
            z_axis = np.array([0, 0, 1])
            
            # 旋转后的坐标轴
            rotated_x = R @ x_axis
            rotated_y = R @ y_axis
            rotated_z = R @ z_axis
            
            # 更新坐标轴线
            origin = [0, 0, 0]
            
            self.orientation_lines['x_axis'].set_data_3d([origin[0], rotated_x[0]], [origin[1], rotated_x[1]], [origin[2], rotated_x[2]])
            self.orientation_lines['y_axis'].set_data_3d([origin[0], rotated_y[0]], [origin[1], rotated_y[1]], [origin[2], rotated_y[2]])
            self.orientation_lines['z_axis'].set_data_3d([origin[0], rotated_z[0]], [origin[1], rotated_z[1]], [origin[2], rotated_z[2]])
            
            # 更新标题显示当前角度
            roll, pitch, yaw = self.imu_config.quaternion_to_euler(latest_reading.quaternion)
            self.orientation_ax.set_title(
                f'IMU 3D Orientation\nRoll: {np.degrees(roll):.1f}° | Pitch: {np.degrees(pitch):.1f}° | Yaw: {np.degrees(yaw):.1f}°',
                fontsize=12
            )
            
            return list(self.orientation_lines.values())
            
        except Exception as e:
            self.logger.error(f"更新3D姿态失败: {e}")
            return list(self.orientation_lines.values())
    
    def get_plot_statistics(self) -> Dict[str, Any]:
        """获取绘图统计信息"""
        try:
            stats = {
                'data_points': len(self.plot_data['timestamps']),
                'plot_update_count': getattr(self, '_plot_update_count', 0),
                'active_plots': len(self.fig) if hasattr(self, 'fig') else 0,
                'visualization_duration': time.time() - self.start_time if hasattr(self, 'start_time') else 0,
                'last_update_time': getattr(self, '_last_update_time', 0)
            }
            
            if self.plot_data['timestamps']:
                stats['data_rate_hz'] = len(self.plot_data['timestamps']) / max(1, stats['visualization_duration'])
                stats['first_timestamp'] = self.plot_data['timestamps'][0]
                stats['last_timestamp'] = self.plot_data['timestamps'][-1]
            
            return stats
            
        except Exception as e:
            self.logger.error(f"获取绘图统计失败: {e}")
            return {'data_points': 0, 'error': str(e)}
    
    def save_current_plots(self, output_dir: str) -> bool:
        """保存当前绘图到指定目录"""
        try:
            from pathlib import Path
            import matplotlib.pyplot as plt
            
            output_path = Path(output_dir)
            output_path.mkdir(parents=True, exist_ok=True)
            
            # 保存所有当前的图形
            saved_files = []
            
            if hasattr(self, 'fig') and self.fig:
                filename = output_path / f"imu_timeseries.png"
                self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                saved_files.append(str(filename))
            
            if hasattr(self, 'orientation_fig') and self.orientation_fig:
                filename = output_path / f"imu_3d_orientation.png"
                self.orientation_fig.savefig(filename, dpi=300, bbox_inches='tight')
                saved_files.append(str(filename))
            
            self.logger.info(f"绘图已保存到: {output_dir}")
            self.logger.info(f"保存的文件: {saved_files}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存绘图失败: {e}")
            return False 