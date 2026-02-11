#!/usr/bin/env python3
"""
LED系统性能基准测试
验证响应时间、资源使用、并发性能等关键指标
"""

import time
import threading
import psutil
import os
import unittest
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Any, Tuple

from .led_test_base import LEDTestBase
from .test_config import get_led_test_config
from .data_collector import get_led_test_collector


class LEDPerformanceBenchmark(LEDTestBase):
    """LED性能基准测试"""
    
    def setUp(self):
        """测试前准备"""
        super().setUp()
        self.config = get_led_test_config()
        self.collector = get_led_test_collector()
        self.collector.start_test_session("led_performance_benchmark",
                                         {'test_type': 'performance', 'module': 'led_system'})
        
        # 获取基线资源使用情况
        self.baseline_memory = self._get_memory_usage()
        self.baseline_cpu = self._get_cpu_usage()
        
        print(f"📊 基线资源使用 - 内存: {self.baseline_memory:.2f}MB, CPU: {self.baseline_cpu:.1f}%")
    
    def tearDown(self):
        """测试后清理"""
        self.collector.end_test_session("led_performance_benchmark")
        super().tearDown()
    
    def test_led_response_time_benchmark(self):
        """LED响应时间基准测试"""
        self.assertLEDSystemReady()
        
        # 测试所有LED模式的响应时间
        modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete", "error_state"]
        response_times = {}
        
        for mode in modes:
            if not hasattr(self.led_system, mode):
                continue
                
            # 多次测量取平均值
            times = []
            for i in range(self.config.performance.performance_samples):
                start_time = time.perf_counter()
                
                try:
                    getattr(self.led_system, mode)()
                    end_time = time.perf_counter()
                    response_time = (end_time - start_time) * 1000  # 转换为毫秒
                    times.append(response_time)
                    
                    # 记录每次测量
                    self.collector.record_performance_data("led_performance_benchmark", 
                                                          response_time=response_time)
                    
                except Exception as e:
                    self.collector.record_error("led_performance_benchmark", "response_time", 
                                               f"模式 {mode} 响应时间测试失败: {e}")
                    continue
                
                # 短暂等待避免过于频繁的调用
                time.sleep(0.01)
            
            if times:
                avg_time = sum(times) / len(times)
                min_time = min(times)
                max_time = max(times)
                
                response_times[mode] = {
                    'average': avg_time,
                    'min': min_time,
                    'max': max_time,
                    'samples': len(times)
                }
                
                # 验证性能要求
                self.assertLessEqual(avg_time, self.config.performance.max_response_time_ms,
                                   f"模式 {mode} 平均响应时间过长: {avg_time:.2f}ms")
                
                # 记录性能指标
                self.collector.record_metric(f"{mode}_avg_response_time", avg_time, "ms",
                                            "led_performance_benchmark", "performance")
                self.collector.record_metric(f"{mode}_max_response_time", max_time, "ms",
                                            "led_performance_benchmark", "performance")
                
                print(f"⚡ {mode}: 平均 {avg_time:.2f}ms, 范围 {min_time:.2f}-{max_time:.2f}ms")
        
        # 计算总体性能统计
        if response_times:
            all_averages = [times['average'] for times in response_times.values()]
            overall_avg = sum(all_averages) / len(all_averages)
            overall_max = max(times['max'] for times in response_times.values())
            
            self.collector.record_metric("overall_avg_response_time", overall_avg, "ms",
                                        "led_performance_benchmark", "performance")
            self.collector.record_metric("overall_max_response_time", overall_max, "ms",
                                        "led_performance_benchmark", "performance")
            
            print(f"📊 总体性能 - 平均: {overall_avg:.2f}ms, 最大: {overall_max:.2f}ms")
        
        print("✅ LED响应时间基准测试通过")
    
    def test_resource_usage_monitoring(self):
        """资源使用监控测试"""
        self.assertLEDSystemReady()
        
        # 记录初始资源使用
        initial_memory = self._get_memory_usage()
        initial_cpu = self._get_cpu_usage()
        
        resource_samples = []
        test_duration = 10.0  # 监控10秒
        sample_interval = 0.5  # 每0.5秒采样一次
        
        start_time = time.time()
        
        def resource_monitor():
            """资源监控线程"""
            while time.time() - start_time < test_duration:
                memory = self._get_memory_usage()
                cpu = self._get_cpu_usage()
                
                resource_samples.append({
                    'timestamp': time.time() - start_time,
                    'memory_mb': memory,
                    'cpu_percent': cpu,
                    'memory_delta': memory - initial_memory,
                    'cpu_delta': cpu - initial_cpu
                })
                
                # 记录实时数据
                self.collector.record_performance_data("led_performance_benchmark",
                                                      cpu_usage=cpu, memory_usage=memory)
                
                time.sleep(sample_interval)
        
        def led_activity():
            """LED活动线程"""
            modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete"]
            
            while time.time() - start_time < test_duration:
                for mode in modes:
                    if hasattr(self.led_system, mode):
                        try:
                            getattr(self.led_system, mode)()
                            time.sleep(0.2)
                        except Exception as e:
                            self.collector.record_error("led_performance_benchmark", "resource_test",
                                                       f"LED活动失败 {mode}: {e}")
                            
                        if time.time() - start_time >= test_duration:
                            break
        
        # 并发运行监控和LED活动
        monitor_thread = threading.Thread(target=resource_monitor)
        activity_thread = threading.Thread(target=led_activity)
        
        monitor_thread.start()
        activity_thread.start()
        
        monitor_thread.join()
        activity_thread.join()
        
        # 分析资源使用数据
        if resource_samples:
            memory_usage = [s['memory_mb'] for s in resource_samples]
            cpu_usage = [s['cpu_percent'] for s in resource_samples]
            memory_deltas = [s['memory_delta'] for s in resource_samples]
            
            avg_memory = sum(memory_usage) / len(memory_usage)
            max_memory = max(memory_usage)
            avg_cpu = sum(cpu_usage) / len(cpu_usage)
            max_cpu = max(cpu_usage)
            max_memory_delta = max(memory_deltas)
            
            # 验证资源使用在合理范围内
            self.assertLessEqual(max_memory_delta, self.config.performance.baseline_memory_mb,
                               f"内存增长过大: {max_memory_delta:.2f}MB")
            self.assertLessEqual(avg_cpu, self.config.performance.baseline_cpu_threshold,
                               f"CPU使用率过高: {avg_cpu:.1f}%")
            
            # 记录资源使用指标
            self.collector.record_metric("avg_memory_usage", avg_memory, "MB",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("max_memory_usage", max_memory, "MB",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("max_memory_delta", max_memory_delta, "MB",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("avg_cpu_usage", avg_cpu, "%",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("max_cpu_usage", max_cpu, "%",
                                        "led_performance_benchmark", "resource")
            
            print(f"📈 资源使用统计:")
            print(f"   内存: 平均 {avg_memory:.2f}MB, 最大 {max_memory:.2f}MB, 增长 {max_memory_delta:.2f}MB")
            print(f"   CPU: 平均 {avg_cpu:.1f}%, 最大 {max_cpu:.1f}%")
        
        print("✅ 资源使用监控测试通过")
    
    def test_concurrent_performance(self):
        """并发性能测试"""
        self.assertLEDSystemReady()
        
        # 并发级别测试
        concurrent_levels = [1, 5, 10, 20]
        performance_results = {}
        
        for concurrent_count in concurrent_levels:
            print(f"🔄 测试并发级别: {concurrent_count}")
            
            # 准备任务
            def led_task(task_id: int) -> Dict[str, Any]:
                mode = ["wake_confirm", "processing_voice", "executing_action"][task_id % 3]
                
                start_time = time.perf_counter()
                try:
                    if hasattr(self.led_system, mode):
                        getattr(self.led_system, mode)()
                    end_time = time.perf_counter()
                    
                    return {
                        'task_id': task_id,
                        'mode': mode,
                        'duration': (end_time - start_time) * 1000,
                        'success': True
                    }
                except Exception as e:
                    return {
                        'task_id': task_id,
                        'mode': mode,
                        'error': str(e),
                        'success': False
                    }
            
            # 执行并发任务
            start_time = time.perf_counter()
            
            with ThreadPoolExecutor(max_workers=concurrent_count) as executor:
                futures = [executor.submit(led_task, i) for i in range(concurrent_count)]
                results = [future.result() for future in as_completed(futures)]
            
            end_time = time.perf_counter()
            total_duration = end_time - start_time
            
            # 分析结果
            successful_tasks = [r for r in results if r['success']]
            failed_tasks = [r for r in results if not r['success']]
            
            if successful_tasks:
                durations = [r['duration'] for r in successful_tasks]
                avg_duration = sum(durations) / len(durations)
                max_duration = max(durations)
                min_duration = min(durations)
                
                success_rate = (len(successful_tasks) / len(results)) * 100
                throughput = len(successful_tasks) / total_duration  # 任务/秒
                
                performance_results[concurrent_count] = {
                    'success_rate': success_rate,
                    'avg_duration': avg_duration,
                    'max_duration': max_duration,
                    'min_duration': min_duration,
                    'throughput': throughput,
                    'total_duration': total_duration * 1000,  # 转换为毫秒
                    'failed_count': len(failed_tasks)
                }
                
                # 记录并发性能指标
                self.collector.record_metric(f"concurrent_{concurrent_count}_success_rate", success_rate, "%",
                                            "led_performance_benchmark", "concurrent")
                self.collector.record_metric(f"concurrent_{concurrent_count}_avg_duration", avg_duration, "ms",
                                            "led_performance_benchmark", "concurrent")
                self.collector.record_metric(f"concurrent_{concurrent_count}_throughput", throughput, "ops/s",
                                            "led_performance_benchmark", "concurrent")
                
                print(f"   成功率: {success_rate:.1f}%, 平均耗时: {avg_duration:.2f}ms, 吞吐量: {throughput:.1f} ops/s")
                
                # 验证性能要求
                self.assertGreaterEqual(success_rate, 95.0, 
                                       f"并发级别 {concurrent_count} 成功率过低: {success_rate:.1f}%")
                
                # 记录失败的任务
                for failed_task in failed_tasks:
                    self.collector.record_error("led_performance_benchmark", "concurrent_task",
                                               f"任务 {failed_task['task_id']} 失败: {failed_task.get('error', 'Unknown')}")
            
            # 短暂休息避免系统过载
            time.sleep(0.5)
        
        # 分析并发性能趋势
        if len(performance_results) > 1:
            throughputs = [r['throughput'] for r in performance_results.values()]
            max_throughput = max(throughputs)
            optimal_concurrent = max(performance_results.keys(), 
                                   key=lambda k: performance_results[k]['throughput'])
            
            self.collector.record_metric("max_throughput", max_throughput, "ops/s",
                                        "led_performance_benchmark", "concurrent")
            self.collector.record_metric("optimal_concurrent_level", optimal_concurrent, "count",
                                        "led_performance_benchmark", "concurrent")
            
            print(f"📊 并发性能分析 - 最大吞吐量: {max_throughput:.1f} ops/s (并发级别: {optimal_concurrent})")
        
        print("✅ 并发性能测试通过")
    
    def test_memory_leak_detection(self):
        """内存泄漏检测测试"""
        self.assertLEDSystemReady()
        
        # 记录初始内存
        initial_memory = self._get_memory_usage()
        memory_samples = [initial_memory]
        
        # 执行大量LED操作
        operations_count = 1000
        sample_interval = 100  # 每100次操作采样一次
        
        modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete"]
        
        for i in range(operations_count):
            mode = modes[i % len(modes)]
            
            try:
                if hasattr(self.led_system, mode):
                    getattr(self.led_system, mode)()
                
                # 定期采样内存使用
                if (i + 1) % sample_interval == 0:
                    current_memory = self._get_memory_usage()
                    memory_samples.append(current_memory)
                    
                    memory_growth = current_memory - initial_memory
                    self.collector.record_metric(f"memory_at_operation_{i+1}", current_memory, "MB",
                                                "led_performance_benchmark", "memory_leak")
                    
                    print(f"💾 操作 {i+1}/{operations_count}: 内存 {current_memory:.2f}MB (增长: {memory_growth:+.2f}MB)")
                
            except Exception as e:
                self.collector.record_error("led_performance_benchmark", "memory_leak_test",
                                           f"操作 {i+1} 失败: {e}")
        
        # 分析内存增长趋势
        if len(memory_samples) >= 3:
            final_memory = memory_samples[-1]
            total_growth = final_memory - initial_memory
            
            # 计算内存增长率
            memory_deltas = [memory_samples[i] - memory_samples[i-1] 
                           for i in range(1, len(memory_samples))]
            avg_growth_per_sample = sum(memory_deltas) / len(memory_deltas)
            
            # 检测是否存在显著内存泄漏
            leak_threshold = self.config.stability.memory_leak_threshold_mb
            
            self.assertLessEqual(total_growth, leak_threshold,
                               f"检测到内存泄漏: 总增长 {total_growth:.2f}MB 超过阈值 {leak_threshold}MB")
            
            # 记录内存泄漏分析结果
            self.collector.record_metric("total_memory_growth", total_growth, "MB",
                                        "led_performance_benchmark", "memory_leak")
            self.collector.record_metric("avg_growth_per_sample", avg_growth_per_sample, "MB",
                                        "led_performance_benchmark", "memory_leak")
            self.collector.record_metric("operations_tested", operations_count, "count",
                                        "led_performance_benchmark", "memory_leak")
            
            # 判断内存泄漏风险级别
            if total_growth <= leak_threshold * 0.3:
                risk_level = "低"
            elif total_growth <= leak_threshold * 0.7:
                risk_level = "中"
            else:
                risk_level = "高"
            
            print(f"🔍 内存泄漏分析:")
            print(f"   总内存增长: {total_growth:.2f}MB")
            print(f"   平均增长率: {avg_growth_per_sample:.3f}MB/sample")
            print(f"   风险级别: {risk_level}")
        
        print("✅ 内存泄漏检测测试通过")
    
    def test_performance_under_load(self):
        """负载下性能测试"""
        self.assertLEDSystemReady()
        
        # 模拟系统负载
        def cpu_load_generator():
            """CPU负载生成器"""
            end_time = time.time() + 5.0  # 运行5秒
            while time.time() < end_time:
                # 执行一些CPU密集型操作
                sum(i * i for i in range(1000))
        
        def memory_load_generator():
            """内存负载生成器"""
            # 分配一些内存（但不要太多影响系统）
            data = [list(range(1000)) for _ in range(100)]
            time.sleep(5.0)
            del data
        
        # 在不同负载条件下测试LED性能
        load_conditions = [
            ("normal", None),
            ("cpu_load", cpu_load_generator),
            ("memory_load", memory_load_generator)
        ]
        
        for condition_name, load_generator in load_conditions:
            print(f"🔄 测试负载条件: {condition_name}")
            
            # 启动负载（如果有）
            load_thread = None
            if load_generator:
                load_thread = threading.Thread(target=load_generator)
                load_thread.start()
            
            # 测试LED性能
            led_response_times = []
            test_operations = 50
            
            for i in range(test_operations):
                mode = ["wake_confirm", "processing_voice", "executing_action"][i % 3]
                
                start_time = time.perf_counter()
                try:
                    if hasattr(self.led_system, mode):
                        getattr(self.led_system, mode)()
                    end_time = time.perf_counter()
                    
                    response_time = (end_time - start_time) * 1000
                    led_response_times.append(response_time)
                    
                except Exception as e:
                    self.collector.record_error("led_performance_benchmark", "load_test",
                                               f"负载 {condition_name} 下操作失败: {e}")
                
                time.sleep(0.05)  # 短暂间隔
            
            # 等待负载线程结束
            if load_thread:
                load_thread.join()
            
            # 分析在当前负载下的性能
            if led_response_times:
                avg_response = sum(led_response_times) / len(led_response_times)
                max_response = max(led_response_times)
                min_response = min(led_response_times)
                
                # 记录负载下的性能指标
                self.collector.record_metric(f"response_time_under_{condition_name}_avg", avg_response, "ms",
                                            "led_performance_benchmark", "load_test")
                self.collector.record_metric(f"response_time_under_{condition_name}_max", max_response, "ms",
                                            "led_performance_benchmark", "load_test")
                
                # 验证在负载下性能仍然可接受
                acceptable_threshold = self.config.performance.max_response_time_ms * 1.5  # 允许50%的性能下降
                self.assertLessEqual(avg_response, acceptable_threshold,
                                   f"负载 {condition_name} 下平均响应时间过长: {avg_response:.2f}ms")
                
                print(f"   平均响应时间: {avg_response:.2f}ms, 范围: {min_response:.2f}-{max_response:.2f}ms")
        
        print("✅ 负载下性能测试通过")
    
    def _get_memory_usage(self) -> float:
        """获取当前进程内存使用量（MB）"""
        try:
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            return memory_info.rss / 1024 / 1024  # 转换为MB
        except Exception:
            return 0.0
    
    def _get_cpu_usage(self) -> float:
        """获取当前CPU使用率（%）"""
        try:
            return psutil.cpu_percent(interval=0.1)
        except Exception:
            return 0.0


class LEDPerformanceRegression(LEDTestBase):
    """LED性能回归测试"""
    
    def setUp(self):
        """测试前准备"""
        super().setUp()
        self.config = get_led_test_config()
        self.collector = get_led_test_collector()
        self.collector.start_test_session("led_performance_regression",
                                         {'test_type': 'regression', 'module': 'led_system'})
    
    def tearDown(self):
        """测试后清理"""
        self.collector.end_test_session("led_performance_regression")
        super().tearDown()
    
    def test_performance_baseline_comparison(self):
        """性能基线对比测试"""
        self.assertLEDSystemReady()
        
        # 定义性能基线（这些值应该基于之前的测试建立）
        performance_baselines = {
            'wake_confirm_response_time': 50.0,      # ms
            'processing_voice_response_time': 45.0,  # ms
            'executing_action_response_time': 40.0,  # ms
            'action_complete_response_time': 55.0,   # ms
            'error_state_response_time': 30.0,       # ms
            'memory_usage_threshold': 150.0,         # MB
            'cpu_usage_threshold': 25.0              # %
        }
        
        # 测试当前性能
        current_performance = {}
        
        modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete", "error_state"]
        
        for mode in modes:
            if not hasattr(self.led_system, mode):
                continue
            
            # 多次测量求平均
            response_times = []
            for _ in range(20):
                start_time = time.perf_counter()
                try:
                    getattr(self.led_system, mode)()
                    end_time = time.perf_counter()
                    response_times.append((end_time - start_time) * 1000)
                except Exception as e:
                    self.collector.record_error("led_performance_regression", "baseline_test",
                                               f"模式 {mode} 基线测试失败: {e}")
                time.sleep(0.01)
            
            if response_times:
                avg_response_time = sum(response_times) / len(response_times)
                current_performance[f"{mode}_response_time"] = avg_response_time
                
                # 与基线对比
                baseline_key = f"{mode}_response_time"
                if baseline_key in performance_baselines:
                    baseline_value = performance_baselines[baseline_key]
                    performance_ratio = avg_response_time / baseline_value
                    
                    # 允许10%的性能波动
                    self.assertLessEqual(performance_ratio, 1.1,
                                       f"模式 {mode} 性能回归: 当前 {avg_response_time:.2f}ms > 基线 {baseline_value:.2f}ms")
                    
                    # 记录性能对比
                    self.collector.record_metric(f"{mode}_performance_ratio", performance_ratio, "ratio",
                                                "led_performance_regression", "comparison")
                    
                    status = "✅" if performance_ratio <= 1.0 else "⚠️" if performance_ratio <= 1.1 else "❌"
                    print(f"{status} {mode}: {avg_response_time:.2f}ms (基线: {baseline_value:.2f}ms, 比率: {performance_ratio:.2f})")
        
        # 检查资源使用
        current_memory = self._get_memory_usage()
        current_cpu = self._get_cpu_usage()
        
        current_performance['memory_usage'] = current_memory
        current_performance['cpu_usage'] = current_cpu
        
        # 与基线对比资源使用
        if current_memory > performance_baselines['memory_usage_threshold']:
            print(f"⚠️ 内存使用超过基线: {current_memory:.2f}MB > {performance_baselines['memory_usage_threshold']}MB")
        
        if current_cpu > performance_baselines['cpu_usage_threshold']:
            print(f"⚠️ CPU使用超过基线: {current_cpu:.1f}% > {performance_baselines['cpu_usage_threshold']}%")
        
        # 记录完整的性能报告
        self.collector.record_metric("performance_baseline_check", "completed", "status",
                                    "led_performance_regression", "regression")
        
        print("✅ 性能基线对比测试完成")
    
    def _get_memory_usage(self) -> float:
        """获取当前进程内存使用量（MB）"""
        try:
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            return memory_info.rss / 1024 / 1024
        except Exception:
            return 0.0
    
    def _get_cpu_usage(self) -> float:
        """获取当前CPU使用率（%）"""
        try:
            return psutil.cpu_percent(interval=0.1)
        except Exception:
            return 0.0


if __name__ == "__main__":
    # 创建测试套件
    suite = unittest.TestSuite()
    
    # 添加基准测试
    suite.addTest(unittest.makeSuite(LEDPerformanceBenchmark))
    
    # 添加回归测试
    suite.addTest(unittest.makeSuite(LEDPerformanceRegression))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 生成报告
    collector = get_led_test_collector()
    collector.save_data("led_performance_test_results")
    
    print(f"\n{'='*60}")
    print(f"LED性能测试完成 - 成功: {result.testsRun - len(result.failures) - len(result.errors)}, "
          f"失败: {len(result.failures)}, 错误: {len(result.errors)}")
    print(f"{'='*60}") 