#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/performance_tester.py
# Generated: 2025-06-27
# Purpose: Unitree Go2前置摄像头实时性能测试

import time
import threading
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
from collections import deque
from camera_config import CameraConfig

@dataclass
class PerformanceMetrics:
    """性能指标数据类"""
    fps_actual: float = 0.0
    fps_target: float = 30.0
    frame_drop_rate: float = 0.0
    capture_success_rate: float = 0.0
    avg_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    min_latency_ms: float = 0.0
    latency_std_ms: float = 0.0
    cpu_usage_percent: float = 0.0
    memory_usage_mb: float = 0.0
    test_duration_seconds: float = 0.0
    total_frames_captured: int = 0
    total_frames_attempted: int = 0
    frame_times: List[float] = field(default_factory=list)
    latencies: List[float] = field(default_factory=list)

class PerformanceTester:
    """前置摄像头性能测试器"""
    
    def __init__(self, camera_config: CameraConfig, config: Dict[str, Any] = None):
        """
        初始化性能测试器
        
        Args:
            camera_config: 摄像头配置对象
            config: 测试配置
        """
        self.camera_config = camera_config
        self.config = config or {}
        self.logger = logging.getLogger(__name__)
        
        # 性能数据收集
        self.frame_times = deque(maxlen=1000)
        self.latencies = deque(maxlen=1000)
        self.capture_results = deque(maxlen=1000)
        
        # 测试控制
        self.is_testing = False
        self.test_start_time = 0
        self.test_thread = None
        
        # 实时统计
        self.frames_captured = 0
        self.frames_attempted = 0
        self.last_fps_calculation = 0
        self.current_fps = 0.0
        
    def run_basic_performance_test(self, duration_seconds: float = 30.0) -> PerformanceMetrics:
        """
        运行基础性能测试
        
        Args:
            duration_seconds: 测试持续时间（秒）
            
        Returns:
            PerformanceMetrics: 性能测试结果
        """
        self.logger.info(f"开始基础性能测试，持续时间: {duration_seconds}秒")
        
        metrics = PerformanceMetrics()
        metrics.test_duration_seconds = duration_seconds
        
        # 获取性能阈值
        thresholds = self.config.get("performance_thresholds", {})
        metrics.fps_target = thresholds.get("target_fps", 30.0)
        
        # 重置计数器
        self._reset_counters()
        
        start_time = time.time()
        end_time = start_time + duration_seconds
        
        self.logger.info("性能测试开始...")
        
        while time.time() < end_time:
            # 记录帧捕获开始时间
            capture_start = time.time()
            
            # 尝试捕获帧
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.time()
            
            # 计算延迟
            latency_ms = (capture_end - capture_start) * 1000
            
            # 记录结果
            self.frames_attempted += 1
            if ret and frame is not None:
                self.frames_captured += 1
                self.frame_times.append(capture_end)
                self.latencies.append(latency_ms)
                self.capture_results.append(True)
            else:
                self.capture_results.append(False)
            
            # 控制测试频率以接近目标帧率
            expected_interval = 1.0 / metrics.fps_target
            elapsed = capture_end - capture_start
            if elapsed < expected_interval:
                time.sleep(expected_interval - elapsed)
        
        # 计算性能指标
        metrics = self._calculate_metrics(metrics, start_time, time.time())
        
        self.logger.info("基础性能测试完成")
        self.logger.info(f"实际FPS: {metrics.fps_actual:.2f}, 平均延迟: {metrics.avg_latency_ms:.2f}ms")
        
        return metrics
    
    def run_stress_test(self, duration_seconds: float = 60.0, high_frequency: bool = True) -> PerformanceMetrics:
        """
        运行压力测试
        
        Args:
            duration_seconds: 测试持续时间（秒）
            high_frequency: 是否使用高频率捕获
            
        Returns:
            PerformanceMetrics: 压力测试结果
        """
        self.logger.info(f"开始压力测试，持续时间: {duration_seconds}秒")
        
        metrics = PerformanceMetrics()
        metrics.test_duration_seconds = duration_seconds
        metrics.fps_target = 60.0 if high_frequency else 30.0
        
        # 重置计数器
        self._reset_counters()
        
        start_time = time.time()
        end_time = start_time + duration_seconds
        
        # 多线程压力测试
        stress_threads = []
        for i in range(2):  # 启动两个并发捕获线程
            thread = threading.Thread(target=self._stress_capture_worker, args=(end_time,))
            stress_threads.append(thread)
            thread.start()
        
        # 等待所有线程完成
        for thread in stress_threads:
            thread.join()
        
        # 计算性能指标
        metrics = self._calculate_metrics(metrics, start_time, time.time())
        
        self.logger.info("压力测试完成")
        self.logger.info(f"压力测试FPS: {metrics.fps_actual:.2f}, 丢帧率: {metrics.frame_drop_rate:.2%}")
        
        return metrics
    
    def _stress_capture_worker(self, end_time: float):
        """压力测试工作线程"""
        while time.time() < end_time:
            capture_start = time.time()
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.time()
            
            latency_ms = (capture_end - capture_start) * 1000
            
            # 线程安全的计数更新
            with threading.Lock():
                self.frames_attempted += 1
                if ret and frame is not None:
                    self.frames_captured += 1
                    self.frame_times.append(capture_end)
                    self.latencies.append(latency_ms)
                    self.capture_results.append(True)
                else:
                    self.capture_results.append(False)
    
    def run_realtime_monitoring(self, duration_seconds: float = 10.0, 
                              callback=None) -> PerformanceMetrics:
        """
        运行实时监控测试
        
        Args:
            duration_seconds: 监控持续时间（秒）
            callback: 实时数据回调函数
            
        Returns:
            PerformanceMetrics: 实时监控结果
        """
        self.logger.info(f"开始实时监控，持续时间: {duration_seconds}秒")
        
        metrics = PerformanceMetrics()
        metrics.test_duration_seconds = duration_seconds
        
        # 重置计数器
        self._reset_counters()
        
        start_time = time.time()
        end_time = start_time + duration_seconds
        last_report_time = start_time
        
        while time.time() < end_time:
            capture_start = time.time()
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.time()
            
            latency_ms = (capture_end - capture_start) * 1000
            
            # 记录数据
            self.frames_attempted += 1
            if ret and frame is not None:
                self.frames_captured += 1
                self.frame_times.append(capture_end)
                self.latencies.append(latency_ms)
                self.capture_results.append(True)
            else:
                self.capture_results.append(False)
            
            # 每秒报告一次实时数据
            if capture_end - last_report_time >= 1.0:
                current_metrics = self._calculate_current_metrics(start_time, capture_end)
                if callback:
                    callback(current_metrics)
                else:
                    self.logger.info(f"实时FPS: {current_metrics.fps_actual:.1f}, "
                                   f"延迟: {current_metrics.avg_latency_ms:.1f}ms")
                last_report_time = capture_end
            
            time.sleep(0.01)  # 短暂休眠避免过度占用CPU
        
        # 最终计算
        metrics = self._calculate_metrics(metrics, start_time, time.time())
        
        self.logger.info("实时监控完成")
        return metrics
    
    def run_latency_test(self, samples: int = 100) -> PerformanceMetrics:
        """
        运行延迟专项测试
        
        Args:
            samples: 测试样本数量
            
        Returns:
            PerformanceMetrics: 延迟测试结果
        """
        self.logger.info(f"开始延迟专项测试，样本数量: {samples}")
        
        metrics = PerformanceMetrics()
        latency_samples = []
        
        start_time = time.time()
        
        for i in range(samples):
            # 高精度时间测量
            capture_start = time.perf_counter()
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.perf_counter()
            
            if ret and frame is not None:
                latency_ms = (capture_end - capture_start) * 1000
                latency_samples.append(latency_ms)
            
            # 适当间隔
            time.sleep(0.05)
        
        end_time = time.time()
        
        if latency_samples:
            metrics.avg_latency_ms = statistics.mean(latency_samples)
            metrics.min_latency_ms = min(latency_samples)
            metrics.max_latency_ms = max(latency_samples)
            metrics.latency_std_ms = statistics.stdev(latency_samples) if len(latency_samples) > 1 else 0
            metrics.latencies = latency_samples
            metrics.capture_success_rate = len(latency_samples) / samples
        
        metrics.test_duration_seconds = end_time - start_time
        metrics.total_frames_attempted = samples
        metrics.total_frames_captured = len(latency_samples)
        
        self.logger.info(f"延迟测试完成: 平均{metrics.avg_latency_ms:.2f}ms, "
                        f"范围{metrics.min_latency_ms:.2f}-{metrics.max_latency_ms:.2f}ms")
        
        return metrics
    
    def _reset_counters(self):
        """重置计数器"""
        self.frame_times.clear()
        self.latencies.clear()
        self.capture_results.clear()
        self.frames_captured = 0
        self.frames_attempted = 0
        self.current_fps = 0.0
    
    def _calculate_metrics(self, metrics: PerformanceMetrics, 
                          start_time: float, end_time: float) -> PerformanceMetrics:
        """计算性能指标"""
        duration = end_time - start_time
        
        # 基本统计
        metrics.total_frames_attempted = self.frames_attempted
        metrics.total_frames_captured = self.frames_captured
        metrics.test_duration_seconds = duration
        
        if self.frames_attempted > 0:
            metrics.capture_success_rate = self.frames_captured / self.frames_attempted
            metrics.frame_drop_rate = 1.0 - metrics.capture_success_rate
        
        if duration > 0 and self.frames_captured > 0:
            metrics.fps_actual = self.frames_captured / duration
        
        # 延迟统计
        if self.latencies:
            latency_list = list(self.latencies)
            metrics.avg_latency_ms = statistics.mean(latency_list)
            metrics.min_latency_ms = min(latency_list)
            metrics.max_latency_ms = max(latency_list)
            metrics.latency_std_ms = statistics.stdev(latency_list) if len(latency_list) > 1 else 0
            metrics.latencies = latency_list
        
        # 帧时间统计
        if self.frame_times:
            metrics.frame_times = list(self.frame_times)
        
        return metrics
    
    def _calculate_current_metrics(self, start_time: float, current_time: float) -> PerformanceMetrics:
        """计算当前时刻的性能指标"""
        metrics = PerformanceMetrics()
        
        duration = current_time - start_time
        if duration > 0 and self.frames_captured > 0:
            metrics.fps_actual = self.frames_captured / duration
        
        if self.latencies:
            recent_latencies = list(self.latencies)[-10:]  # 最近10帧的延迟
            metrics.avg_latency_ms = statistics.mean(recent_latencies)
        
        if self.frames_attempted > 0:
            metrics.capture_success_rate = self.frames_captured / self.frames_attempted
        
        return metrics
    
    def evaluate_performance(self, metrics: PerformanceMetrics) -> Dict[str, Any]:
        """
        评估性能表现
        
        Args:
            metrics: 性能指标
            
        Returns:
            Dict[str, Any]: 评估结果
        """
        thresholds = self.config.get("performance_thresholds", {})
        
        evaluation = {
            "overall_status": "PASS",
            "issues": [],
            "recommendations": [],
            "scores": {}
        }
        
        # FPS评估
        min_fps = thresholds.get("min_fps", 20)
        target_fps = thresholds.get("target_fps", 30)
        
        if metrics.fps_actual < min_fps:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"FPS过低: {metrics.fps_actual:.1f} < {min_fps}")
        elif metrics.fps_actual < target_fps:
            evaluation["issues"].append(f"FPS未达到目标: {metrics.fps_actual:.1f} < {target_fps}")
        
        fps_score = min(100, (metrics.fps_actual / target_fps) * 100)
        evaluation["scores"]["fps"] = fps_score
        
        # 延迟评估
        max_latency = thresholds.get("max_latency_ms", 100)
        
        if metrics.avg_latency_ms > max_latency:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"延迟过高: {metrics.avg_latency_ms:.1f}ms > {max_latency}ms")
        
        latency_score = max(0, 100 - (metrics.avg_latency_ms / max_latency) * 100)
        evaluation["scores"]["latency"] = latency_score
        
        # 稳定性评估
        max_drop_rate = thresholds.get("max_frame_drop_rate", 0.05)
        min_success_rate = thresholds.get("min_capture_success_rate", 0.95)
        
        if metrics.frame_drop_rate > max_drop_rate:
            evaluation["issues"].append(f"丢帧率过高: {metrics.frame_drop_rate:.1%} > {max_drop_rate:.1%}")
        
        if metrics.capture_success_rate < min_success_rate:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"捕获成功率过低: {metrics.capture_success_rate:.1%} < {min_success_rate:.1%}")
        
        stability_score = metrics.capture_success_rate * 100
        evaluation["scores"]["stability"] = stability_score
        
        # 生成建议
        if fps_score < 80:
            evaluation["recommendations"].append("考虑降低分辨率或调整摄像头参数以提高帧率")
        
        if latency_score < 80:
            evaluation["recommendations"].append("优化图像捕获流程以减少延迟")
        
        if stability_score < 95:
            evaluation["recommendations"].append("检查摄像头连接和驱动程序状态")
        
        # 总体评分
        evaluation["scores"]["overall"] = (fps_score + latency_score + stability_score) / 3
        
        return evaluation

# 测试函数
def test_performance_tester():
    """测试性能测试器功能"""
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # 创建摄像头配置
    with CameraConfig() as camera_config:
        if not camera_config.initialize_camera():
            print("摄像头初始化失败，无法进行性能测试")
            return
        
        # 创建性能测试器
        tester = PerformanceTester(camera_config)
        
        # 运行基础性能测试
        print("运行基础性能测试...")
        metrics = tester.run_basic_performance_test(duration_seconds=10.0)
        
        # 评估性能
        evaluation = tester.evaluate_performance(metrics)
        
        print(f"\n性能测试结果:")
        print(f"实际FPS: {metrics.fps_actual:.2f}")
        print(f"平均延迟: {metrics.avg_latency_ms:.2f}ms")
        print(f"捕获成功率: {metrics.capture_success_rate:.1%}")
        print(f"整体评分: {evaluation['scores']['overall']:.1f}")
        print(f"评估状态: {evaluation['overall_status']}")

if __name__ == "__main__":
    test_performance_tester() 