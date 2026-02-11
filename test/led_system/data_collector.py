#!/usr/bin/env python3
"""
LED测试数据收集器
收集、存储和分析测试数据
"""

import time
import json
import csv
import threading
from collections import defaultdict, deque
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional, Union, Deque
import statistics

@dataclass
class TestMetric:
    """测试指标数据"""
    name: str
    value: Union[float, int, str]
    unit: str
    timestamp: float
    test_name: str
    category: str = "general"

@dataclass
class PerformanceMetrics:
    """性能指标集合"""
    response_times: List[float]
    cpu_usage: List[float]
    memory_usage: List[float]
    success_rate: float
    error_count: int
    total_operations: int
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计数据"""
        stats = {}
        
        if self.response_times:
            stats['response_time'] = {
                'mean': statistics.mean(self.response_times),
                'median': statistics.median(self.response_times),
                'min': min(self.response_times),
                'max': max(self.response_times),
                'stdev': statistics.stdev(self.response_times) if len(self.response_times) > 1 else 0.0
            }
        
        if self.cpu_usage:
            stats['cpu'] = {
                'mean': statistics.mean(self.cpu_usage),
                'max': max(self.cpu_usage)
            }
        
        if self.memory_usage:
            stats['memory'] = {
                'mean': statistics.mean(self.memory_usage),
                'max': max(self.memory_usage)
            }
        
        stats['reliability'] = {
            'success_rate': self.success_rate,
            'error_count': self.error_count,
            'total_operations': self.total_operations
        }
        
        return stats

class LEDTestDataCollector:
    """LED测试数据收集器"""
    
    def __init__(self, output_dir: Optional[str] = None):
        """初始化数据收集器"""
        self.output_dir = Path(output_dir or "logs/led_tests")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 数据存储
        self.metrics: List[TestMetric] = []
        self.performance_data: Dict[str, PerformanceMetrics] = {}
        self.test_sessions: Dict[str, Dict[str, Any]] = {}
        self.error_logs: List[Dict[str, Any]] = []
        
        # 实时数据缓冲区
        self._response_times: Deque[float] = deque(maxlen=1000)
        self._cpu_usage: Deque[float] = deque(maxlen=100)
        self._memory_usage: Deque[float] = deque(maxlen=100)
        
        # 线程安全
        self._lock = threading.Lock()
        
        # 会话信息
        self.session_id = self._generate_session_id()
        self.session_start_time = time.time()
        
    def _generate_session_id(self) -> str:
        """生成会话ID"""
        return f"led_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    def start_test_session(self, test_name: str, config: Dict[str, Any] = None):
        """开始测试会话"""
        with self._lock:
            session_data = {
                'test_name': test_name,
                'start_time': time.time(),
                'config': config or {},
                'metrics': [],
                'status': 'running'
            }
            self.test_sessions[test_name] = session_data
            print(f"📊 开始测试会话: {test_name}")
    
    def end_test_session(self, test_name: str, status: str = "completed"):
        """结束测试会话"""
        with self._lock:
            if test_name in self.test_sessions:
                session = self.test_sessions[test_name]
                session['end_time'] = time.time()
                session['duration'] = session['end_time'] - session['start_time']
                session['status'] = status
                print(f"📊 结束测试会话: {test_name} ({status})")
    
    def record_metric(self, name: str, value: Union[float, int, str], 
                     unit: str = "", test_name: str = "default", 
                     category: str = "general"):
        """记录指标"""
        metric = TestMetric(
            name=name,
            value=value,
            unit=unit,
            timestamp=time.time(),
            test_name=test_name,
            category=category
        )
        
        with self._lock:
            self.metrics.append(metric)
            
            # 更新实时缓冲区
            if name == "response_time" and isinstance(value, (int, float)):
                self._response_times.append(float(value))
            elif name == "cpu_usage" and isinstance(value, (int, float)):
                self._cpu_usage.append(float(value))
            elif name == "memory_usage" and isinstance(value, (int, float)):
                self._memory_usage.append(float(value))
    
    def record_performance_data(self, test_name: str, 
                               response_time: Optional[float] = None,
                               cpu_usage: Optional[float] = None,
                               memory_usage: Optional[float] = None,
                               success: bool = True):
        """记录性能数据"""
        with self._lock:
            if test_name not in self.performance_data:
                self.performance_data[test_name] = PerformanceMetrics(
                    response_times=[],
                    cpu_usage=[],
                    memory_usage=[],
                    success_rate=0.0,
                    error_count=0,
                    total_operations=0
                )
            
            perf = self.performance_data[test_name]
            
            if response_time is not None:
                perf.response_times.append(response_time)
                self.record_metric("response_time", response_time, "ms", test_name, "performance")
            
            if cpu_usage is not None:
                perf.cpu_usage.append(cpu_usage)
                self.record_metric("cpu_usage", cpu_usage, "%", test_name, "performance")
            
            if memory_usage is not None:
                perf.memory_usage.append(memory_usage)
                self.record_metric("memory_usage", memory_usage, "MB", test_name, "performance")
            
            # 更新成功率
            perf.total_operations += 1
            if not success:
                perf.error_count += 1
            
            perf.success_rate = ((perf.total_operations - perf.error_count) / 
                                perf.total_operations * 100) if perf.total_operations > 0 else 0.0
    
    def record_error(self, test_name: str, error_type: str, error_message: str, 
                    context: Dict[str, Any] = None):
        """记录错误"""
        error_data = {
            'test_name': test_name,
            'error_type': error_type,
            'error_message': error_message,
            'context': context or {},
            'timestamp': time.time(),
            'formatted_time': datetime.now().isoformat()
        }
        
        with self._lock:
            self.error_logs.append(error_data)
            self.record_metric("error_count", 1, "count", test_name, "error")
    
    def get_real_time_stats(self) -> Dict[str, Any]:
        """获取实时统计数据"""
        with self._lock:
            stats = {}
            
            if self._response_times:
                recent_times = list(self._response_times)[-20:]  # 最近20次
                stats['response_time'] = {
                    'current_avg': statistics.mean(recent_times),
                    'current_max': max(recent_times),
                    'overall_avg': statistics.mean(self._response_times),
                    'overall_max': max(self._response_times),
                    'sample_count': len(self._response_times)
                }
            
            if self._cpu_usage:
                stats['cpu_usage'] = {
                    'current': self._cpu_usage[-1] if self._cpu_usage else 0,
                    'avg': statistics.mean(self._cpu_usage),
                    'max': max(self._cpu_usage)
                }
            
            if self._memory_usage:
                stats['memory_usage'] = {
                    'current': self._memory_usage[-1] if self._memory_usage else 0,
                    'avg': statistics.mean(self._memory_usage),
                    'max': max(self._memory_usage)
                }
            
            stats['session_duration'] = time.time() - self.session_start_time
            stats['total_metrics'] = len(self.metrics)
            stats['error_count'] = len(self.error_logs)
            
            return stats
    
    def get_test_summary(self, test_name: str) -> Optional[Dict[str, Any]]:
        """获取测试摘要"""
        if test_name not in self.performance_data:
            return None
        
        perf = self.performance_data[test_name]
        stats = perf.get_stats()
        
        # 添加测试会话信息
        if test_name in self.test_sessions:
            session = self.test_sessions[test_name]
            stats['session'] = {
                'duration': session.get('duration', 0),
                'status': session.get('status', 'unknown'),
                'config': session.get('config', {})
            }
        
        # 添加错误信息
        test_errors = [e for e in self.error_logs if e['test_name'] == test_name]
        stats['errors'] = {
            'count': len(test_errors),
            'recent_errors': test_errors[-5:] if test_errors else []
        }
        
        return stats
    
    def save_data(self, filename: Optional[str] = None, format_type: str = "json"):
        """保存数据到文件"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"led_test_data_{timestamp}"
        
        # 准备导出数据
        export_data = {
            'session_id': self.session_id,
            'session_start_time': self.session_start_time,
            'session_duration': time.time() - self.session_start_time,
            'metrics': [asdict(m) for m in self.metrics],
            'performance_data': {k: asdict(v) for k, v in self.performance_data.items()},
            'test_sessions': self.test_sessions,
            'error_logs': self.error_logs,
            'real_time_stats': self.get_real_time_stats()
        }
        
        if format_type == "json":
            file_path = self.output_dir / f"{filename}.json"
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=2, ensure_ascii=False, default=str)
        
        elif format_type == "csv":
            # 保存指标数据为CSV
            file_path = self.output_dir / f"{filename}_metrics.csv"
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'test_name', 'metric_name', 'value', 'unit', 'category'])
                for metric in self.metrics:
                    writer.writerow([
                        metric.timestamp, metric.test_name, metric.name, 
                        metric.value, metric.unit, metric.category
                    ])
        
        print(f"💾 测试数据已保存: {file_path}")
        return file_path
    
    def generate_report(self, output_file: Optional[str] = None) -> str:
        """生成测试报告"""
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"led_test_report_{timestamp}.html"
        
        report_path = self.output_dir / output_file
        
        # 生成HTML报告
        html_content = self._generate_html_report()
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        print(f"📋 测试报告已生成: {report_path}")
        return str(report_path)
    
    def _generate_html_report(self) -> str:
        """生成HTML报告内容"""
        real_time_stats = self.get_real_time_stats()
        
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>LED控制系统测试报告</title>
    <meta charset="utf-8">
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        h1, h2, h3 {{ color: #333; }}
        .summary {{ background: #f5f5f5; padding: 15px; border-radius: 5px; margin: 10px 0; }}
        .metric {{ display: inline-block; margin: 10px; padding: 10px; background: #e9f7ef; border-radius: 3px; }}
        .error {{ background: #fadbd8; }}
        table {{ border-collapse: collapse; width: 100%; margin: 10px 0; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
        .success {{ color: green; }}
        .warning {{ color: orange; }}
        .error-text {{ color: red; }}
    </style>
</head>
<body>
    <h1>🤖 LED控制系统测试报告</h1>
    
    <div class="summary">
        <h2>📊 测试会话概览</h2>
        <div class="metric">
            <strong>会话ID:</strong> {self.session_id}
        </div>
        <div class="metric">
            <strong>测试时长:</strong> {real_time_stats.get('session_duration', 0):.2f}秒
        </div>
        <div class="metric">
            <strong>总指标数:</strong> {real_time_stats.get('total_metrics', 0)}
        </div>
        <div class="metric">
            <strong>错误数:</strong> {real_time_stats.get('error_count', 0)}
        </div>
    </div>
    
    <h2>⚡ 性能统计</h2>
"""
        
        # 添加性能数据
        for test_name, summary in [(name, self.get_test_summary(name)) for name in self.performance_data.keys()]:
            if summary:
                html += f"""
    <h3>测试: {test_name}</h3>
    <table>
        <tr><th>指标</th><th>值</th><th>状态</th></tr>
"""
                if 'response_time' in summary:
                    rt = summary['response_time']
                    status = "success" if rt['mean'] < 200 else "warning"
                    html += f"""
        <tr><td>平均响应时间</td><td>{rt['mean']:.2f}ms</td><td class="{status}">{'✅' if rt['mean'] < 200 else '⚠️'}</td></tr>
        <tr><td>最大响应时间</td><td>{rt['max']:.2f}ms</td><td>-</td></tr>
        <tr><td>响应时间标准差</td><td>{rt['stdev']:.2f}ms</td><td>-</td></tr>
"""
                
                if 'reliability' in summary:
                    rel = summary['reliability']
                    status = "success" if rel['success_rate'] >= 95 else "error"
                    html += f"""
        <tr><td>成功率</td><td>{rel['success_rate']:.1f}%</td><td class="{status}">{'✅' if rel['success_rate'] >= 95 else '❌'}</td></tr>
        <tr><td>操作总数</td><td>{rel['total_operations']}</td><td>-</td></tr>
        <tr><td>错误次数</td><td>{rel['error_count']}</td><td>-</td></tr>
"""
                html += "    </table>\n"
        
        # 添加错误日志
        if self.error_logs:
            html += """
    <h2>❌ 错误日志</h2>
    <table>
        <tr><th>时间</th><th>测试</th><th>类型</th><th>消息</th></tr>
"""
            for error in self.error_logs[-10:]:  # 显示最近10个错误
                html += f"""
        <tr>
            <td>{datetime.fromtimestamp(error['timestamp']).strftime('%H:%M:%S')}</td>
            <td>{error['test_name']}</td>
            <td>{error['error_type']}</td>
            <td class="error-text">{error['error_message'][:100]}...</td>
        </tr>
"""
            html += "    </table>\n"
        
        html += f"""
    <div class="summary">
        <h2>📝 报告生成信息</h2>
        <p><strong>生成时间:</strong> {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p><strong>数据来源:</strong> LED控制系统测试框架 v1.0</p>
    </div>
</body>
</html>
"""
        return html
    
    def clear_data(self):
        """清空数据"""
        with self._lock:
            self.metrics.clear()
            self.performance_data.clear()
            self.test_sessions.clear()
            self.error_logs.clear()
            self._response_times.clear()
            self._cpu_usage.clear()
            self._memory_usage.clear()
        print("🧹 测试数据已清空")

# 全局数据收集器实例
_global_collector = None

def get_led_test_collector() -> LEDTestDataCollector:
    """获取全局数据收集器实例"""
    global _global_collector
    if _global_collector is None:
        _global_collector = LEDTestDataCollector()
    return _global_collector

def reset_led_test_collector():
    """重置全局数据收集器"""
    global _global_collector
    _global_collector = None

if __name__ == "__main__":
    # 数据收集器演示
    collector = LEDTestDataCollector()
    
    # 模拟测试数据
    collector.start_test_session("demo_test")
    
    for i in range(10):
        collector.record_performance_data(
            "demo_test",
            response_time=50 + i * 5,
            cpu_usage=30 + i * 2,
            memory_usage=100 + i,
            success=i < 9  # 最后一次失败
        )
        time.sleep(0.1)
    
    collector.end_test_session("demo_test")
    
    # 显示统计
    stats = collector.get_real_time_stats()
    print("📊 实时统计:", json.dumps(stats, indent=2, default=str))
    
    # 生成报告
    collector.generate_report()
    collector.save_data()
    
    print("✅ 数据收集器演示完成") 