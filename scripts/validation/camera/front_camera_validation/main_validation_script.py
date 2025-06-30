#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/main_validation_script.py
# Generated: 2025-06-27
# Purpose: Unitree Go2前置摄像头主验证脚本

import os
import sys
import time
import json
import logging
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Tuple

# 添加当前目录到Python路径
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

from camera_config import CameraConfig
from performance_tester import PerformanceTester, PerformanceMetrics
from image_quality_analyzer import ImageQualityAnalyzer, ImageQualityMetrics

class FrontCameraValidator:
    """前置摄像头综合验证器"""
    
    def __init__(self, config_path: str = None, output_dir: str = None):
        """
        初始化验证器
        
        Args:
            config_path: 配置文件路径
            output_dir: 输出目录
        """
        # 设置日志
        self._setup_logging()
        self.logger = logging.getLogger(__name__)
        
        # 加载配置
        self.config = self._load_config(config_path)
        
        # 设置输出目录
        self.output_dir = Path(output_dir) if output_dir else Path("logs/camera_validation")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 创建时间戳目录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"validation_{timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        
        self.logger.info(f"验证会话目录: {self.session_dir}")
        
        # 验证组件
        self.camera_config = None
        self.performance_tester = None
        self.image_quality_analyzer = None
        
        # 验证结果
        self.validation_results = {}
        
    def _setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler('front_camera_validation.log')
            ]
        )
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        default_config_path = current_dir / "validation_config.json"
        config_file = Path(config_path) if config_path else default_config_path
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
                self.logger.info(f"加载配置文件: {config_file}")
                return config
        except Exception as e:
            self.logger.error(f"配置文件加载失败: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            "camera_config": {
                "target_resolution": [1280, 720],
                "target_fps": 30,
                "timeout_seconds": 10
            },
            "validation_sequence": [
                "camera_initialization",
                "resolution_verification", 
                "basic_performance_test",
                "image_quality_analysis",
                "stress_test",
                "report_generation"
            ]
        }
    
    def run_full_validation(self) -> Dict[str, Any]:
        """
        运行完整验证流程
        
        Returns:
            Dict[str, Any]: 验证结果
        """
        self.logger.info("=" * 60)
        self.logger.info("开始Unitree Go2前置摄像头完整验证")
        self.logger.info("=" * 60)
        
        start_time = time.time()
        
        try:
            # 获取验证序列
            sequence = self.config.get("validation_sequence", [])
            
            for step in sequence:
                self.logger.info(f"\n{'='*20} {step.upper()} {'='*20}")
                
                if step == "camera_initialization":
                    result = self._step_camera_initialization()
                elif step == "resolution_verification":
                    result = self._step_resolution_verification()
                elif step == "basic_performance_test":
                    result = self._step_basic_performance_test()
                elif step == "image_quality_analysis":
                    result = self._step_image_quality_analysis()
                elif step == "stress_test":
                    result = self._step_stress_test()
                elif step == "report_generation":
                    result = self._step_report_generation()
                else:
                    self.logger.warning(f"未知验证步骤: {step}")
                    result = {"status": "SKIPPED", "reason": "Unknown step"}
                
                self.validation_results[step] = result
                
                # 如果关键步骤失败，停止验证
                if step in ["camera_initialization"] and result.get("status") == "FAIL":
                    self.logger.error(f"关键步骤失败: {step}，停止验证")
                    break
            
            # 计算总体结果
            end_time = time.time()
            self.validation_results["summary"] = self._generate_summary(start_time, end_time)
            
            # 保存结果
            self._save_validation_results()
            
            self.logger.info("=" * 60)
            self.logger.info("前置摄像头验证完成")
            self.logger.info("=" * 60)
            
            return self.validation_results
            
        except Exception as e:
            self.logger.error(f"验证过程中发生错误: {e}")
            self.validation_results["error"] = str(e)
            return self.validation_results
        
        finally:
            # 清理资源
            self._cleanup()
    
    def _step_camera_initialization(self) -> Dict[str, Any]:
        """步骤1: 摄像头初始化"""
        self.logger.info("初始化摄像头配置...")
        
        try:
            self.camera_config = CameraConfig(config_path=None)
            
            # 尝试初始化摄像头
            if self.camera_config.initialize_camera("opencv"):
                self.logger.info("摄像头初始化成功")
                
                # 获取摄像头属性
                properties = self.camera_config.get_camera_properties()
                
                # 初始化其他组件
                self.performance_tester = PerformanceTester(self.camera_config, self.config)
                self.image_quality_analyzer = ImageQualityAnalyzer(self.camera_config, self.config)
                
                return {
                    "status": "PASS",
                    "camera_properties": properties,
                    "actual_spec": self.camera_config.actual_spec.__dict__ if self.camera_config.actual_spec else None
                }
            else:
                self.logger.error("摄像头初始化失败")
                return {
                    "status": "FAIL",
                    "reason": "Camera initialization failed"
                }
        
        except Exception as e:
            self.logger.error(f"摄像头初始化异常: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during initialization: {str(e)}"
            }
    
    def _step_resolution_verification(self) -> Dict[str, Any]:
        """步骤2: 分辨率验证"""
        self.logger.info("验证摄像头分辨率...")
        
        if not self.camera_config or not self.camera_config.is_initialized:
            return {"status": "SKIP", "reason": "Camera not initialized"}
        
        try:
            # 捕获测试帧
            ret, frame = self.camera_config.capture_frame()
            
            if ret and frame is not None:
                actual_resolution = (frame.shape[1], frame.shape[0])  # (width, height)
                target_resolution = tuple(self.config.get("camera_config", {}).get("target_resolution", [1280, 720]))
                
                resolution_match = actual_resolution == target_resolution
                
                self.logger.info(f"实际分辨率: {actual_resolution}")
                self.logger.info(f"目标分辨率: {target_resolution}")
                self.logger.info(f"分辨率匹配: {resolution_match}")
                
                return {
                    "status": "PASS" if resolution_match else "WARNING",
                    "actual_resolution": actual_resolution,
                    "target_resolution": target_resolution,
                    "resolution_match": resolution_match
                }
            else:
                return {
                    "status": "FAIL",
                    "reason": "Failed to capture frame for resolution verification"
                }
        
        except Exception as e:
            self.logger.error(f"分辨率验证异常: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during resolution verification: {str(e)}"
            }
    
    def _step_basic_performance_test(self) -> Dict[str, Any]:
        """步骤3: 基础性能测试"""
        self.logger.info("运行基础性能测试...")
        
        if not self.performance_tester:
            return {"status": "SKIP", "reason": "Performance tester not initialized"}
        
        try:
            # 运行基础性能测试
            metrics = self.performance_tester.run_basic_performance_test(duration_seconds=30.0)
            
            # 评估性能
            evaluation = self.performance_tester.evaluate_performance(metrics)
            
            # 保存性能数据
            self._save_performance_data(metrics, "basic_performance")
            
            self.logger.info(f"基础性能测试完成 - FPS: {metrics.fps_actual:.2f}, "
                           f"延迟: {metrics.avg_latency_ms:.2f}ms")
            
            return {
                "status": evaluation["overall_status"],
                "metrics": self._serialize_metrics(metrics),
                "evaluation": evaluation
            }
        
        except Exception as e:
            self.logger.error(f"基础性能测试异常: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during basic performance test: {str(e)}"
            }
    
    def _step_image_quality_analysis(self) -> Dict[str, Any]:
        """步骤4: 图像质量分析"""
        self.logger.info("运行图像质量分析...")
        
        if not self.image_quality_analyzer:
            return {"status": "SKIP", "reason": "Image quality analyzer not initialized"}
        
        try:
            # 运行图像质量分析
            metrics = self.image_quality_analyzer.analyze_image_quality(sample_count=20)
            
            # 评估质量
            evaluation = self.image_quality_analyzer.evaluate_quality_thresholds(metrics)
            
            # 保存质量数据
            self._save_quality_data(metrics, "image_quality")
            
            self.logger.info(f"图像质量分析完成 - 整体评分: {metrics.overall_quality_score:.2f} "
                           f"({metrics.quality_grade})")
            
            return {
                "status": evaluation["overall_status"],
                "metrics": self._serialize_quality_metrics(metrics),
                "evaluation": evaluation
            }
        
        except Exception as e:
            self.logger.error(f"图像质量分析异常: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during image quality analysis: {str(e)}"
            }
    
    def _step_stress_test(self) -> Dict[str, Any]:
        """步骤5: 压力测试"""
        self.logger.info("运行压力测试...")
        
        if not self.performance_tester:
            return {"status": "SKIP", "reason": "Performance tester not initialized"}
        
        try:
            # 运行压力测试
            metrics = self.performance_tester.run_stress_test(duration_seconds=60.0)
            
            # 评估性能
            evaluation = self.performance_tester.evaluate_performance(metrics)
            
            # 保存压力测试数据
            self._save_performance_data(metrics, "stress_test")
            
            self.logger.info(f"压力测试完成 - FPS: {metrics.fps_actual:.2f}, "
                           f"丢帧率: {metrics.frame_drop_rate:.2%}")
            
            return {
                "status": evaluation["overall_status"],
                "metrics": self._serialize_metrics(metrics),
                "evaluation": evaluation
            }
        
        except Exception as e:
            self.logger.error(f"压力测试异常: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during stress test: {str(e)}"
            }
    
    def _step_report_generation(self) -> Dict[str, Any]:
        """步骤6: 报告生成"""
        self.logger.info("生成验证报告...")
        
        try:
            # 生成HTML报告
            report_path = self._generate_html_report()
            
            # 生成JSON报告
            json_report_path = self._generate_json_report()
            
            self.logger.info(f"报告生成完成:")
            self.logger.info(f"  HTML报告: {report_path}")
            self.logger.info(f"  JSON报告: {json_report_path}")
            
            return {
                "status": "PASS",
                "html_report": str(report_path),
                "json_report": str(json_report_path)
            }
        
        except Exception as e:
            self.logger.error(f"报告生成异常: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during report generation: {str(e)}"
            }
    
    def _generate_summary(self, start_time: float, end_time: float) -> Dict[str, Any]:
        """生成验证总结"""
        duration = end_time - start_time
        
        # 统计各步骤状态
        status_counts = {"PASS": 0, "FAIL": 0, "WARNING": 0, "SKIP": 0}
        for step, result in self.validation_results.items():
            if step != "summary" and isinstance(result, dict):
                status = result.get("status", "UNKNOWN")
                if status in status_counts:
                    status_counts[status] += 1
        
        # 确定整体状态
        if status_counts["FAIL"] > 0:
            overall_status = "FAIL"
        elif status_counts["WARNING"] > 0:
            overall_status = "WARNING"
        else:
            overall_status = "PASS"
        
        return {
            "overall_status": overall_status,
            "duration_seconds": duration,
            "test_timestamp": datetime.now().isoformat(),
            "status_counts": status_counts,
            "session_directory": str(self.session_dir)
        }
    
    def _serialize_metrics(self, metrics: PerformanceMetrics) -> Dict[str, Any]:
        """序列化性能指标"""
        return {
            "fps_actual": metrics.fps_actual,
            "fps_target": metrics.fps_target,
            "avg_latency_ms": metrics.avg_latency_ms,
            "max_latency_ms": metrics.max_latency_ms,
            "min_latency_ms": metrics.min_latency_ms,
            "capture_success_rate": metrics.capture_success_rate,
            "frame_drop_rate": metrics.frame_drop_rate,
            "test_duration_seconds": metrics.test_duration_seconds,
            "total_frames_captured": metrics.total_frames_captured,
            "total_frames_attempted": metrics.total_frames_attempted
        }
    
    def _serialize_quality_metrics(self, metrics: ImageQualityMetrics) -> Dict[str, Any]:
        """序列化质量指标"""
        return {
            "resolution_actual": metrics.resolution_actual,
            "resolution_target": metrics.resolution_target,
            "resolution_match": metrics.resolution_match,
            "overall_quality_score": metrics.overall_quality_score,
            "quality_grade": metrics.quality_grade,
            "sharpness_score": metrics.sharpness_score,
            "color_accuracy_score": metrics.color_accuracy_score,
            "brightness_score": metrics.brightness_score,
            "contrast_score": metrics.contrast_score,
            "noise_level": metrics.noise_level,
            "snr_db": metrics.snr_db
        }
    
    def _save_performance_data(self, metrics: PerformanceMetrics, test_name: str):
        """保存性能数据"""
        data_file = self.session_dir / f"{test_name}_metrics.json"
        with open(data_file, 'w', encoding='utf-8') as f:
            json.dump(self._serialize_metrics(metrics), f, indent=2, ensure_ascii=False)
    
    def _save_quality_data(self, metrics: ImageQualityMetrics, test_name: str):
        """保存质量数据"""
        data_file = self.session_dir / f"{test_name}_metrics.json"
        with open(data_file, 'w', encoding='utf-8') as f:
            json.dump(self._serialize_quality_metrics(metrics), f, indent=2, ensure_ascii=False)
    
    def _save_validation_results(self):
        """保存验证结果"""
        results_file = self.session_dir / "validation_results.json"
        with open(results_file, 'w', encoding='utf-8') as f:
            json.dump(self.validation_results, f, indent=2, ensure_ascii=False)
    
    def _generate_html_report(self) -> Path:
        """生成HTML报告"""
        report_path = self.session_dir / "validation_report.html"
        
        # 简化的HTML报告模板
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Unitree Go2前置摄像头验证报告</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .header {{ background-color: #f0f0f0; padding: 10px; margin-bottom: 20px; }}
        .section {{ margin-bottom: 20px; border: 1px solid #ddd; padding: 10px; }}
        .pass {{ color: green; }} .fail {{ color: red; }} .warning {{ color: orange; }}
        table {{ border-collapse: collapse; width: 100%; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>Unitree Go2前置摄像头验证报告</h1>
        <p>生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p>整体状态: <span class="{self.validation_results.get('summary', {}).get('overall_status', 'unknown').lower()}">{self.validation_results.get('summary', {}).get('overall_status', 'UNKNOWN')}</span></p>
    </div>
    
    <div class="section">
        <h2>验证结果概览</h2>
        {self._generate_results_table()}
    </div>
    
    <div class="section">
        <h2>详细测试结果</h2>
        {self._generate_detailed_results()}
    </div>
</body>
</html>
        """
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        return report_path
    
    def _generate_json_report(self) -> Path:
        """生成JSON报告"""
        report_path = self.session_dir / "validation_report.json"
        
        # 创建简化的JSON报告
        report_data = {
            "meta": {
                "test_type": "front_camera_validation",
                "timestamp": datetime.now().isoformat(),
                "version": "1.0.0"
            },
            "results": self.validation_results
        }
        
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)
        
        return report_path
    
    def _generate_results_table(self) -> str:
        """生成结果表格HTML"""
        html = "<table><tr><th>测试步骤</th><th>状态</th><th>备注</th></tr>"
        
        for step, result in self.validation_results.items():
            if step != "summary" and isinstance(result, dict):
                status = result.get("status", "UNKNOWN")
                reason = result.get("reason", "")
                html += f"<tr><td>{step}</td><td class='{status.lower()}'>{status}</td><td>{reason}</td></tr>"
        
        html += "</table>"
        return html
    
    def _generate_detailed_results(self) -> str:
        """生成详细结果HTML"""
        html = ""
        for step, result in self.validation_results.items():
            if step not in ["summary"] and isinstance(result, dict):
                html += f"<h3>{step}</h3>"
                html += f"<pre>{json.dumps(result, indent=2, ensure_ascii=False)}</pre>"
        return html
    
    def _cleanup(self):
        """清理资源"""
        if self.camera_config:
            self.camera_config.release()
            self.logger.info("摄像头资源已释放")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="Unitree Go2前置摄像头验证")
    parser.add_argument("--config", help="配置文件路径")
    parser.add_argument("--output", help="输出目录路径")
    parser.add_argument("--verbose", action="store_true", help="详细输出")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 创建验证器并运行
    validator = FrontCameraValidator(args.config, args.output)
    results = validator.run_full_validation()
    
    # 打印总结
    summary = results.get("summary", {})
    print(f"\n验证完成!")
    print(f"整体状态: {summary.get('overall_status', 'UNKNOWN')}")
    print(f"耗时: {summary.get('duration_seconds', 0):.1f}秒")
    print(f"会话目录: {summary.get('session_directory', 'N/A')}")
    
    # 根据结果设置退出码
    exit_code = 0 if summary.get('overall_status') == 'PASS' else 1
    sys.exit(exit_code)

if __name__ == "__main__":
    main() 