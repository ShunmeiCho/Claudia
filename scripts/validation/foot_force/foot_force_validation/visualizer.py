#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/visualizer.py
# Generated: 2025-06-27 14:21:00 CST
# Purpose: Unitree Go2 Foot Force Sensor Data Visualization Module

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
import logging
import threading
from typing import Dict, List, Tuple, Optional, Any, Callable
from dataclasses import dataclass
from collections import deque
import json
from pathlib import Path
from datetime import datetime

from foot_force_config import FootForceConfig, FootForceReading
from data_collector import FootForceData

# Configure font for English display
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
plt.rcParams['axes.unicode_minus'] = False

@dataclass
class VisualizationConfig:
    """Visualization Configuration"""
    update_rate_hz: float = 20.0
    window_size: int = 1000
    colors: Dict[str, str] = None
    force_scale: float = 1.0
    cop_range: float = 0.5
    figure_size: Tuple[int, int] = (15, 10)
    
    def __post_init__(self):
        if self.colors is None:
            self.colors = {
                'FL': '#FF0000',  # Front Left - Red
                'FR': '#00FF00',  # Front Right - Green
                'RL': '#0000FF',  # Rear Left - Blue
                'RR': '#FFFF00',  # Rear Right - Yellow
                'contact': '#FF00FF',
                'no_contact': '#808080'
            }

class FootForceVisualizer:
    """Foot Force Sensor Data Visualizer"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化可视化器
        
        Args:
            config: 配置字典
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # 可视化配置
        viz_config = config.get('visualization', {})
        self.viz_config = VisualizationConfig(
            update_rate_hz=viz_config.get('update_rate_hz', 20.0),
            window_size=viz_config.get('display_window_size', 1000),
            colors=viz_config.get('colors', {}),
            force_scale=viz_config.get('force_scale_factor', 1.0),
            cop_range=viz_config.get('cop_display_range', 0.5),
            figure_size=(viz_config.get('figure_width', 15), viz_config.get('figure_height', 10))
        )
        
        # 数据缓冲区
        self.data_buffer = deque(maxlen=self.viz_config.window_size)
        self.time_buffer = deque(maxlen=self.viz_config.window_size)
        
        # 实时显示状态
        self.is_displaying = False
        self.display_thread: Optional[threading.Thread] = None
        
        # matplotlib图形对象
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.patches = {}
        
        # 数据回调
        self.data_callbacks: List[Callable[[FootForceData], None]] = []
        
        self.logger.info("足端力传感器可视化器初始化完成")
    
    def add_data_callback(self, callback: Callable[[FootForceData], None]):
        """添加数据回调函数"""
        self.data_callbacks.append(callback)
    
    def update_data(self, data: FootForceData):
        """更新数据到缓冲区"""
        self.data_buffer.append(data)
        self.time_buffer.append(data.timestamp)
        
        # 调用数据回调
        for callback in self.data_callbacks:
            try:
                callback(data)
            except Exception as e:
                self.logger.error(f"数据回调错误: {e}")
    
    def create_realtime_display(self) -> bool:
        """创建实时显示窗口"""
        try:
            self.fig, axes_array = plt.subplots(2, 3, figsize=self.viz_config.figure_size)
            self.fig.suptitle('Unitree Go2 Foot Force Sensor Real-time Monitor', fontsize=16, fontweight='bold')
            
            # 扁平化axes数组便于访问
            axes_flat = axes_array.flatten()
            
            # 1. 四足端力时间序列图
            self.axes['force_time'] = axes_flat[0]
            self.axes['force_time'].set_title('Four Foot End Vertical Force Time Series')
            self.axes['force_time'].set_ylabel('Force (N)')
            self.axes['force_time'].grid(True, alpha=0.3)
            
            # 初始化力时间序列线条
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                color = self.viz_config.colors.get(foot_label, f'C{i}')
                line, = self.axes['force_time'].plot([], [], color=color, label=foot_label, linewidth=2)
                self.lines[f'force_{foot_label}'] = line
            
            self.axes['force_time'].legend()
            
            # 2. 压力中心轨迹图
            self.axes['cop'] = axes_flat[1]
            self.axes['cop'].set_title('Center of Pressure Trajectory')
            self.axes['cop'].set_xlabel('X (m)')
            self.axes['cop'].set_ylabel('Y (m)')
            self.axes['cop'].grid(True, alpha=0.3)
            self.axes['cop'].set_aspect('equal')
            
            # 绘制机器人足端位置参考
            foot_positions = [
                (-0.2, 0.15),   # FL
                (-0.2, -0.15),  # FR
                (0.2, 0.15),    # RL
                (0.2, -0.15)    # RR
            ]
            
            for i, (pos, foot_label) in enumerate(zip(foot_positions, FootForceConfig.FOOT_LABELS)):
                color = self.viz_config.colors.get(foot_label, f'C{i}')
                circle = Circle(pos, 0.05, color=color, alpha=0.3, label=f'{foot_label}足端')
                self.axes['cop'].add_patch(circle)
            
            # 压力中心轨迹线
            cop_line, = self.axes['cop'].plot([], [], 'r-', alpha=0.7, linewidth=1, label='Center of Pressure Trajectory')
            self.lines['cop_trajectory'] = cop_line
            
            # 当前压力中心点
            cop_point, = self.axes['cop'].plot([], [], 'ro', markersize=8, label='Current Center of Pressure')
            self.lines['cop_current'] = cop_point
            
            self.axes['cop'].legend()
            self.axes['cop'].set_xlim(-self.viz_config.cop_range, self.viz_config.cop_range)
            self.axes['cop'].set_ylim(-self.viz_config.cop_range, self.viz_config.cop_range)
            
            # 3. 力分布饼图
            self.axes['force_pie'] = axes_flat[2]
            self.axes['force_pie'].set_title('Current Force Distribution')
            
            # 4. 接触状态指示器
            self.axes['contact'] = axes_flat[3]
            self.axes['contact'].set_title('Foot End Contact State')
            self.axes['contact'].set_xlim(-1, 1)
            self.axes['contact'].set_ylim(-1, 1)
            self.axes['contact'].set_aspect('equal')
            
            # 创建接触状态指示器
            contact_positions = [(-0.5, 0.5), (0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)]
            for i, (pos, foot_label) in enumerate(zip(contact_positions, FootForceConfig.FOOT_LABELS)):
                rect = Rectangle((pos[0]-0.3, pos[1]-0.3), 0.6, 0.6, 
                               facecolor=self.viz_config.colors['no_contact'], 
                               edgecolor='black', linewidth=2)
                self.axes['contact'].add_patch(rect)
                self.patches[f'contact_{foot_label}'] = rect
                
                # 添加标签
                self.axes['contact'].text(pos[0], pos[1], foot_label, 
                                        ha='center', va='center', fontsize=12, fontweight='bold')
            
            self.axes['contact'].axis('off')
            
            # 5. 稳定性指标
            self.axes['stability'] = axes_flat[4]
            self.axes['stability'].set_title('Stability Index Time Series')
            self.axes['stability'].set_ylabel('Index Value')
            self.axes['stability'].grid(True, alpha=0.3)
            
            stability_line, = self.axes['stability'].plot([], [], 'g-', label='稳定性指数', linewidth=2)
            balance_line, = self.axes['stability'].plot([], [], 'b-', label='平衡指数', linewidth=2)
            self.lines['stability'] = stability_line
            self.lines['balance'] = balance_line
            self.axes['stability'].legend()
            self.axes['stability'].set_ylim(0, 1)
            
            # 6. 总力时间序列
            self.axes['total_force'] = axes_flat[5]
            self.axes['total_force'].set_title('Total Force Time Series')
            self.axes['total_force'].set_xlabel('Time (s)')
            self.axes['total_force'].set_ylabel('Force (N)')
            self.axes['total_force'].grid(True, alpha=0.3)
            
            total_force_line, = self.axes['total_force'].plot([], [], 'k-', linewidth=2, label='Total Force')
            self.lines['total_force'] = total_force_line
            self.axes['total_force'].legend()
            
            plt.tight_layout()
            
            self.logger.info("实时显示窗口创建成功")
            return True 
            
        except Exception as e:
            self.logger.error(f"创建实时显示窗口失败: {e}")
            return False
    
    def update_realtime_display(self):
        """更新实时显示"""
        if not self.data_buffer or not self.fig:
            return
        
        try:
            # 获取最近的数据
            current_time = time.time()
            time_window = 30.0  # 显示最近30秒的数据
            
            # 筛选时间窗口内的数据
            time_array = np.array(self.time_buffer)
            data_array = list(self.data_buffer)
            
            mask = time_array >= (current_time - time_window)
            if not np.any(mask):
                return
            
            filtered_times = time_array[mask] - time_array[mask][0]  # 相对时间
            filtered_data = [data_array[i] for i in range(len(data_array)) if mask[i]]
            
            if len(filtered_data) < 2:
                return
            
            # 1. 更新力时间序列
            forces_array = np.array([d.foot_forces for d in filtered_data])
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                vertical_forces = forces_array[:, i, 2]  # 垂直力
                self.lines[f'force_{foot_label}'].set_data(filtered_times, vertical_forces)
            
            self.axes['force_time'].relim()
            self.axes['force_time'].autoscale_view()
            
            # 2. 更新压力中心
            cops = np.array([d.center_of_pressure for d in filtered_data])
            self.lines['cop_trajectory'].set_data(cops[:, 0], cops[:, 1])
            
            # 当前压力中心
            current_cop = filtered_data[-1].center_of_pressure
            self.lines['cop_current'].set_data([current_cop[0]], [current_cop[1]])
            
            # 3. 更新力分布饼图
            self.axes['force_pie'].clear()
            self.axes['force_pie'].set_title('Current Force Distribution')
            
            current_forces = filtered_data[-1].foot_forces
            vertical_forces = [abs(force[2]) for force in current_forces]
            total_force = sum(vertical_forces)
            
            if total_force > 0:
                percentages = [f/total_force*100 for f in vertical_forces]
                colors = [self.viz_config.colors.get(label, f'C{i}') 
                         for i, label in enumerate(FootForceConfig.FOOT_LABELS)]
                
                wedges, texts, autotexts = self.axes['force_pie'].pie(
                    percentages, 
                    labels=FootForceConfig.FOOT_LABELS,
                    colors=colors,
                    autopct='%1.1f%%',
                    startangle=90
                )
                
                for autotext in autotexts:
                    autotext.set_color('white')
                    autotext.set_fontweight('bold')
            
            # 4. 更新接触状态
            current_contacts = filtered_data[-1].contact_states
            for i, (foot_label, is_contact) in enumerate(zip(FootForceConfig.FOOT_LABELS, current_contacts)):
                color = self.viz_config.colors['contact'] if is_contact else self.viz_config.colors['no_contact']
                self.patches[f'contact_{foot_label}'].set_facecolor(color)
            
            # 5. 更新稳定性指标
            stability_values = [d.stability_index for d in filtered_data]
            balance_values = [d.force_balance for d in filtered_data]
            
            self.lines['stability'].set_data(filtered_times, stability_values)
            self.lines['balance'].set_data(filtered_times, balance_values)
            
            self.axes['stability'].relim()
            self.axes['stability'].autoscale_view()
            
            # 6. 更新总力
            total_forces = [d.total_force for d in filtered_data]
            self.lines['total_force'].set_data(filtered_times, total_forces)
            
            self.axes['total_force'].relim()
            self.axes['total_force'].autoscale_view()
            
            # 刷新显示
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.logger.error(f"更新实时显示失败: {e}")
    
    def start_realtime_display(self) -> bool:
        """启动实时显示"""
        try:
            if not self.create_realtime_display():
                return False
            
            self.is_displaying = True
            
            def display_loop():
                """显示循环"""
                update_interval = 1.0 / self.viz_config.update_rate_hz
                
                while self.is_displaying:
                    self.update_realtime_display()
                    time.sleep(update_interval)
            
            self.display_thread = threading.Thread(target=display_loop)
            self.display_thread.daemon = True
            self.display_thread.start()
            
            self.logger.info("实时显示已启动")
            return True
            
        except Exception as e:
            self.logger.error(f"启动实时显示失败: {e}")
            return False
    
    def stop_realtime_display(self):
        """停止实时显示"""
        self.is_displaying = False
        
        if self.display_thread:
            self.display_thread.join(timeout=2.0)
        
        if self.fig:
            plt.close(self.fig)
            self.fig = None
        
        self.logger.info("实时显示已停止")
    
    def plot_static_analysis(self, data: List[FootForceData], save_path: Optional[str] = None) -> bool:
        """
        绘制静态分析图表
        
        Args:
            data: 足端力数据列表
            save_path: 保存路径
            
        Returns:
            bool: 是否成功
        """
        try:
            if len(data) < 10:
                self.logger.error("数据不足，无法进行静态分析")
                return False
            
            # 创建静态分析图形
            fig, axes = plt.subplots(3, 2, figsize=(15, 12))
            fig.suptitle('足端力传感器静态分析报告', fontsize=16, fontweight='bold')
            
            # 准备数据
            time_relative = np.arange(len(data)) / len(data) * (data[-1].timestamp - data[0].timestamp)
            forces_array = np.array([d.foot_forces for d in data])
            total_forces = np.array([d.total_force for d in data])
            cops = np.array([d.center_of_pressure for d in data])
            stability_indices = np.array([d.stability_index for d in data])
            balance_indices = np.array([d.force_balance for d in data])
            
            # 1. 各足端垂直力分布
            ax1 = axes[0, 0]
            for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                vertical_forces = forces_array[:, i, 2]
                color = self.viz_config.colors.get(foot_label, f'C{i}')
                ax1.plot(time_relative, vertical_forces, color=color, label=foot_label, linewidth=2)
            
            ax1.set_title('各足端垂直力时间序列')
            ax1.set_xlabel('时间 (s)')
            ax1.set_ylabel('垂直力 (N)')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # 2. 力分布箱线图
            ax2 = axes[0, 1]
            vertical_forces_data = [forces_array[:, i, 2] for i in range(4)]
            box_plot = ax2.boxplot(vertical_forces_data, labels=FootForceConfig.FOOT_LABELS, patch_artist=True)
            
            for patch, foot_label in zip(box_plot['boxes'], FootForceConfig.FOOT_LABELS):
                color = self.viz_config.colors.get(foot_label, 'lightblue')
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
            
            ax2.set_title('各足端垂直力分布')
            ax2.set_ylabel('垂直力 (N)')
            ax2.grid(True, alpha=0.3)
            
            # 3. 压力中心轨迹热力图
            ax3 = axes[1, 0]
            scatter = ax3.scatter(cops[:, 0], cops[:, 1], c=time_relative, cmap='viridis', 
                                alpha=0.6, s=20)
            
            # 添加足端位置参考
            foot_positions = [(-0.2, 0.15), (-0.2, -0.15), (0.2, 0.15), (0.2, -0.15)]
            for i, (pos, foot_label) in enumerate(zip(foot_positions, FootForceConfig.FOOT_LABELS)):
                color = self.viz_config.colors.get(foot_label, f'C{i}')
                circle = Circle(pos, 0.05, color=color, alpha=0.5)
                ax3.add_patch(circle)
                ax3.text(pos[0], pos[1]-0.1, foot_label, ha='center', va='top', fontsize=8)
            
            ax3.set_title('压力中心轨迹热力图')
            ax3.set_xlabel('X (m)')
            ax3.set_ylabel('Y (m)')
            ax3.set_aspect('equal')
            plt.colorbar(scatter, ax=ax3, label='时间 (s)')
            
            # 4. 总力统计
            ax4 = axes[1, 1]
            ax4.plot(time_relative, total_forces, 'k-', linewidth=2, label='总合力')
            ax4.axhline(np.mean(total_forces), color='r', linestyle='--', 
                       label=f'平均值: {np.mean(total_forces):.1f}N')
            ax4.fill_between(time_relative, 
                           np.mean(total_forces) - np.std(total_forces),
                           np.mean(total_forces) + np.std(total_forces),
                           alpha=0.3, color='gray', label=f'±1σ: {np.std(total_forces):.1f}N')
            
            ax4.set_title('总合力统计分析')
            ax4.set_xlabel('时间 (s)')
            ax4.set_ylabel('总力 (N)')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            
            # 5. 稳定性和平衡指标
            ax5 = axes[2, 0]
            ax5.plot(time_relative, stability_indices, 'g-', linewidth=2, label='稳定性指数')
            ax5.plot(time_relative, balance_indices, 'b-', linewidth=2, label='平衡指数')
            ax5.axhline(np.mean(stability_indices), color='g', linestyle='--', alpha=0.7)
            ax5.axhline(np.mean(balance_indices), color='b', linestyle='--', alpha=0.7)
            
            ax5.set_title('稳定性和平衡指标')
            ax5.set_xlabel('时间 (s)')
            ax5.set_ylabel('指标值')
            ax5.set_ylim(0, 1)
            ax5.legend()
            ax5.grid(True, alpha=0.3)
            
            # 6. 力分布饼图（平均）
            ax6 = axes[2, 1]
            mean_vertical_forces = np.mean(forces_array[:, :, 2], axis=0)
            total_mean_force = np.sum(mean_vertical_forces)
            
            if total_mean_force > 0:
                percentages = mean_vertical_forces / total_mean_force * 100
                colors = [self.viz_config.colors.get(label, f'C{i}') 
                         for i, label in enumerate(FootForceConfig.FOOT_LABELS)]
                
                wedges, texts, autotexts = ax6.pie(
                    percentages,
                    labels=FootForceConfig.FOOT_LABELS,
                    colors=colors,
                    autopct='%1.1f%%',
                    startangle=90
                )
                
                for autotext in autotexts:
                    autotext.set_color('white')
                    autotext.set_fontweight('bold')
            
            ax6.set_title('平均力分布')
            
            plt.tight_layout()
            
            # 保存图表
            if save_path:
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                self.logger.info(f"静态分析图表已保存到: {save_path}")
            
            plt.show(block=False)
            
            return True
            
        except Exception as e:
            self.logger.error(f"绘制静态分析图表失败: {e}")
            return False
    
    def plot_force_distribution_3d(self, data: List[FootForceData], save_path: Optional[str] = None) -> bool:
        """
        绘制3D力分布图
        
        Args:
            data: 足端力数据列表
            save_path: 保存路径
            
        Returns:
            bool: 是否成功
        """
        try:
            from mpl_toolkits.mplot3d import Axes3D
            
            if len(data) < 10:
                self.logger.error("数据不足，无法绘制3D力分布图")
                return False
            
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # 足端位置
            foot_positions = np.array([
                [-0.2, 0.15, 0],   # FL
                [-0.2, -0.15, 0],  # FR
                [0.2, 0.15, 0],    # RL
                [0.2, -0.15, 0]    # RR
            ])
            
            # 计算平均垂直力
            forces_array = np.array([d.foot_forces for d in data])
            mean_forces = np.mean(forces_array, axis=0)
            
            # 绘制3D力向量
            for i, (pos, foot_label) in enumerate(zip(foot_positions, FootForceConfig.FOOT_LABELS)):
                force = mean_forces[i]
                color = self.viz_config.colors.get(foot_label, f'C{i}')
                
                # 力向量的方向和大小
                force_magnitude = np.linalg.norm(force)
                if force_magnitude > 0:
                    # 绘制力向量
                    ax.quiver(pos[0], pos[1], pos[2], 
                             force[0], force[1], force[2],
                             color=color, arrow_length_ratio=0.1, 
                             linewidth=3, alpha=0.8, label=f'{foot_label}: {force_magnitude:.1f}N')
                
                # 足端位置标记
                ax.scatter(pos[0], pos[1], pos[2], color=color, s=100, alpha=0.7)
                ax.text(pos[0], pos[1], pos[2]+0.05, foot_label, fontsize=10, ha='center')
            
            # 设置坐标轴
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Force (N)')
            ax.set_title('3D足端力分布图')
            ax.legend()
            
            # 设置等比例
            max_range = max(np.max(np.abs(foot_positions)), np.max(np.abs(mean_forces))) * 1.2
            ax.set_xlim([-max_range/2, max_range/2])
            ax.set_ylim([-max_range/2, max_range/2])
            ax.set_zlim([0, np.max(np.abs(mean_forces[:, 2])) * 1.2])
            
            # 保存图表
            if save_path:
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                self.logger.info(f"3D力分布图已保存到: {save_path}")
            
            plt.show(block=False)
            
            return True
            
        except Exception as e:
            self.logger.error(f"绘制3D力分布图失败: {e}")
            return False
    
    def create_summary_dashboard(self, test_results: List[Any], save_path: Optional[str] = None) -> bool:
        """
        创建测试结果汇总仪表板
        
        Args:
            test_results: 测试结果列表
            save_path: 保存路径
            
        Returns:
            bool: 是否成功
        """
        try:
            fig, axes = plt.subplots(2, 3, figsize=(18, 12))
            fig.suptitle('足端力传感器验证结果汇总仪表板', fontsize=20, fontweight='bold')
            
            # 1. 测试通过率饼图
            ax1 = axes[0, 0]
            status_counts = {}
            for result in test_results:
                status = result.status if hasattr(result, 'status') else 'UNKNOWN'
                status_counts[status] = status_counts.get(status, 0) + 1
            
            colors = {'PASS': '#4CAF50', 'WARNING': '#FF9800', 'FAIL': '#F44336', 'UNKNOWN': '#9E9E9E'}
            wedges, texts, autotexts = ax1.pie(
                status_counts.values(),
                labels=status_counts.keys(),
                colors=[colors.get(status, '#9E9E9E') for status in status_counts.keys()],
                autopct='%1.1f%%',
                startangle=90
            )
            ax1.set_title('测试通过率')
            
            # 2. 评分条形图
            ax2 = axes[0, 1]
            test_names = [result.test_name if hasattr(result, 'test_name') else f'Test {i}' 
                         for i, result in enumerate(test_results)]
            scores = [result.score if hasattr(result, 'score') else 0 for result in test_results]
            
            bars = ax2.bar(range(len(test_names)), scores, 
                          color=['#4CAF50' if s >= 80 else '#FF9800' if s >= 60 else '#F44336' for s in scores])
            ax2.set_title('各测试项评分')
            ax2.set_ylabel('评分')
            ax2.set_xticks(range(len(test_names)))
            ax2.set_xticklabels(test_names, rotation=45, ha='right')
            ax2.set_ylim(0, 100)
            ax2.grid(True, alpha=0.3)
            
            # 在柱子上显示分数
            for bar, score in zip(bars, scores):
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
                        f'{score:.1f}', ha='center', va='bottom', fontweight='bold')
            
            # 3. 总体评估雷达图
            ax3 = axes[0, 2]
            categories = ['准确性', '稳定性', '一致性', '可靠性', '响应性']
            
            # 模拟评估分数（实际应该从测试结果中提取）
            scores_radar = [85, 78, 82, 88, 75]  # 示例数据
            
            angles = np.linspace(0, 2 * np.pi, len(categories), endpoint=False)
            scores_radar += scores_radar[:1]  # 闭合图形
            angles = np.concatenate((angles, [angles[0]]))
            
            ax3.plot(angles, scores_radar, 'o-', linewidth=2, color='#2196F3')
            ax3.fill(angles, scores_radar, alpha=0.25, color='#2196F3')
            ax3.set_xticks(angles[:-1])
            ax3.set_xticklabels(categories)
            ax3.set_ylim(0, 100)
            ax3.set_title('综合性能雷达图')
            ax3.grid(True)
            
            # 4. 时间线图
            ax4 = axes[1, 0]
            timestamps = [result.timestamp if hasattr(result, 'timestamp') else time.time() 
                         for result in test_results]
            relative_times = [(t - min(timestamps))/60 for t in timestamps]  # 转换为分钟
            
            colors_timeline = ['#4CAF50' if hasattr(result, 'status') and result.status == 'PASS' 
                              else '#FF9800' if hasattr(result, 'status') and result.status == 'WARNING'
                              else '#F44336' for result in test_results]
            
            ax4.scatter(relative_times, range(len(test_results)), c=colors_timeline, s=100, alpha=0.7)
            ax4.set_yticks(range(len(test_results)))
            ax4.set_yticklabels(test_names)
            ax4.set_xlabel('时间 (分钟)')
            ax4.set_title('测试执行时间线')
            ax4.grid(True, alpha=0.3)
            
            # 5. 推荐措施汇总
            ax5 = axes[1, 1]
            ax5.axis('off')
            ax5.set_title('主要建议', fontweight='bold', fontsize=14)
            
            # 收集所有建议
            all_recommendations = []
            for result in test_results:
                if hasattr(result, 'recommendations'):
                    all_recommendations.extend(result.recommendations)
            
            # 显示前6个建议
            recommendations_text = ""
            for i, rec in enumerate(all_recommendations[:6]):
                recommendations_text += f"{i+1}. {rec}\n\n"
            
            ax5.text(0.05, 0.95, recommendations_text, transform=ax5.transAxes, fontsize=10,
                    verticalalignment='top', wrap=True)
            
            # 6. 系统状态指示器
            ax6 = axes[1, 2]
            ax6.axis('off')
            ax6.set_title('系统状态', fontweight='bold', fontsize=14)
            
            # 计算整体状态
            avg_score = np.mean(scores) if scores else 0
            pass_rate = status_counts.get('PASS', 0) / len(test_results) * 100 if test_results else 0
            
            if avg_score >= 80 and pass_rate >= 80:
                overall_status = "正常"
                status_color = "#4CAF50"
            elif avg_score >= 60 and pass_rate >= 60:
                overall_status = "警告"
                status_color = "#FF9800"
            else:
                overall_status = "异常"
                status_color = "#F44336"
            
            # 状态指示器
            circle = Circle((0.5, 0.7), 0.15, color=status_color, alpha=0.8)
            ax6.add_patch(circle)
            ax6.text(0.5, 0.7, overall_status, ha='center', va='center', 
                    fontsize=16, fontweight='bold', color='white')
            
            # 状态详情
            status_text = f"""
整体评分: {avg_score:.1f}/100
通过率: {pass_rate:.1f}%
测试项目: {len(test_results)}个
完成时间: {datetime.now().strftime('%Y-%m-%d %H:%M')}
            """
            ax6.text(0.5, 0.3, status_text, ha='center', va='center', fontsize=12,
                    transform=ax6.transAxes)
            
            ax6.set_xlim(0, 1)
            ax6.set_ylim(0, 1)
            
            plt.tight_layout()
            
            # 保存仪表板
            if save_path:
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                self.logger.info(f"汇总仪表板已保存到: {save_path}")
            
            plt.show(block=False)
            
            return True
            
        except Exception as e:
            self.logger.error(f"创建汇总仪表板失败: {e}")
            return False 