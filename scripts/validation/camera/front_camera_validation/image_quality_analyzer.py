#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/image_quality_analyzer.py
# Generated: 2025-06-27
# Purpose: Unitree Go2前置摄像头图像质量分析

import cv2
import numpy as np
import logging
import statistics
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
from skimage import measure, filters, exposure
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt
from camera_config import CameraConfig

@dataclass
class ImageQualityMetrics:
    """图像质量指标数据类"""
    resolution_actual: Tuple[int, int] = (0, 0)
    resolution_target: Tuple[int, int] = (1280, 720)
    resolution_match: bool = False
    
    # 色彩指标
    color_accuracy_score: float = 0.0
    color_delta_e: float = 0.0
    white_balance_score: float = 0.0
    saturation_score: float = 0.0
    
    # 清晰度指标
    sharpness_score: float = 0.0
    blur_detection_score: float = 0.0
    edge_density: float = 0.0
    
    # 曝光和对比度
    brightness_score: float = 0.0
    contrast_score: float = 0.0
    dynamic_range: float = 0.0
    exposure_accuracy: float = 0.0
    
    # 噪声指标
    noise_level: float = 0.0
    snr_db: float = 0.0
    
    # 整体质量
    overall_quality_score: float = 0.0
    quality_grade: str = "UNKNOWN"
    
    # 原始数据
    sample_images: List[np.ndarray] = field(default_factory=list)
    quality_history: List[float] = field(default_factory=list)

class ImageQualityAnalyzer:
    """图像质量分析器"""
    
    def __init__(self, camera_config: CameraConfig, config: Dict[str, Any] = None):
        """
        初始化图像质量分析器
        
        Args:
            camera_config: 摄像头配置对象
            config: 分析配置
        """
        self.camera_config = camera_config
        self.config = config or {}
        self.logger = logging.getLogger(__name__)
        
        # 参考图像和模板
        self.reference_patterns = self._generate_reference_patterns()
        self.color_checker = self._create_color_checker()
        
    def analyze_image_quality(self, sample_count: int = 20) -> ImageQualityMetrics:
        """
        全面分析图像质量
        
        Args:
            sample_count: 分析的样本图像数量
            
        Returns:
            ImageQualityMetrics: 图像质量分析结果
        """
        self.logger.info(f"开始图像质量分析，样本数量: {sample_count}")
        
        metrics = ImageQualityMetrics()
        
        # 收集样本图像
        sample_images = self._collect_sample_images(sample_count)
        if not sample_images:
            self.logger.error("无法收集到有效的样本图像")
            return metrics
        
        metrics.sample_images = sample_images[:5]  # 只保留前5张作为示例
        
        # 分辨率验证
        metrics = self._analyze_resolution(sample_images, metrics)
        
        # 色彩分析
        metrics = self._analyze_color_quality(sample_images, metrics)
        
        # 清晰度分析
        metrics = self._analyze_sharpness(sample_images, metrics)
        
        # 曝光和对比度分析
        metrics = self._analyze_exposure_contrast(sample_images, metrics)
        
        # 噪声分析
        metrics = self._analyze_noise(sample_images, metrics)
        
        # 计算整体质量评分
        metrics = self._calculate_overall_quality(metrics)
        
        self.logger.info(f"图像质量分析完成，整体评分: {metrics.overall_quality_score:.2f}")
        
        return metrics
    
    def _collect_sample_images(self, count: int) -> List[np.ndarray]:
        """收集样本图像"""
        images = []
        
        for i in range(count * 2):  # 多采集一些，以防部分失败
            ret, frame = self.camera_config.capture_frame()
            if ret and frame is not None:
                images.append(frame.copy())
                if len(images) >= count:
                    break
            
            # 适当间隔以获得不同的图像
            import time
            time.sleep(0.1)
        
        self.logger.info(f"成功收集到 {len(images)} 张样本图像")
        return images
    
    def _analyze_resolution(self, images: List[np.ndarray], 
                          metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """分析分辨率"""
        if not images:
            return metrics
        
        # 获取实际分辨率
        height, width = images[0].shape[:2]
        metrics.resolution_actual = (width, height)
        
        # 检查是否匹配目标分辨率
        target_resolution = self.config.get("camera_config", {}).get("target_resolution", [1280, 720])
        metrics.resolution_target = tuple(target_resolution)
        
        metrics.resolution_match = (
            metrics.resolution_actual[0] == metrics.resolution_target[0] and
            metrics.resolution_actual[1] == metrics.resolution_target[1]
        )
        
        self.logger.info(f"分辨率分析: 实际{metrics.resolution_actual}, "
                        f"目标{metrics.resolution_target}, 匹配: {metrics.resolution_match}")
        
        return metrics
    
    def _analyze_color_quality(self, images: List[np.ndarray], 
                             metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """分析色彩质量"""
        color_scores = []
        delta_e_values = []
        wb_scores = []
        saturation_scores = []
        
        for image in images:
            # 转换到LAB色彩空间进行分析
            lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 色彩准确性评估
            color_score = self._evaluate_color_accuracy(image, lab_image)
            color_scores.append(color_score)
            
            # Delta E计算（简化版）
            delta_e = self._calculate_delta_e(lab_image)
            delta_e_values.append(delta_e)
            
            # 白平衡评估
            wb_score = self._evaluate_white_balance(image)
            wb_scores.append(wb_score)
            
            # 饱和度评估
            saturation_score = self._evaluate_saturation(hsv_image)
            saturation_scores.append(saturation_score)
        
        # 统计结果
        metrics.color_accuracy_score = statistics.mean(color_scores)
        metrics.color_delta_e = statistics.mean(delta_e_values)
        metrics.white_balance_score = statistics.mean(wb_scores)
        metrics.saturation_score = statistics.mean(saturation_scores)
        
        self.logger.info(f"色彩质量分析: 准确性{metrics.color_accuracy_score:.2f}, "
                        f"Delta E{metrics.color_delta_e:.2f}")
        
        return metrics
    
    def _analyze_sharpness(self, images: List[np.ndarray], 
                         metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """分析图像清晰度"""
        sharpness_scores = []
        blur_scores = []
        edge_densities = []
        
        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Laplacian方差法测量清晰度
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            sharpness_score = min(100, laplacian_var / 100)  # 归一化到0-100
            sharpness_scores.append(sharpness_score)
            
            # 模糊检测
            blur_score = self._detect_blur(gray)
            blur_scores.append(blur_score)
            
            # 边缘密度
            edge_density = self._calculate_edge_density(gray)
            edge_densities.append(edge_density)
        
        metrics.sharpness_score = statistics.mean(sharpness_scores)
        metrics.blur_detection_score = statistics.mean(blur_scores)
        metrics.edge_density = statistics.mean(edge_densities)
        
        self.logger.info(f"清晰度分析: 锐度{metrics.sharpness_score:.2f}, "
                        f"边缘密度{metrics.edge_density:.2f}")
        
        return metrics
    
    def _analyze_exposure_contrast(self, images: List[np.ndarray], 
                                 metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """分析曝光和对比度"""
        brightness_scores = []
        contrast_scores = []
        dynamic_ranges = []
        exposure_scores = []
        
        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 亮度评估
            mean_brightness = np.mean(gray)
            brightness_score = self._evaluate_brightness(mean_brightness)
            brightness_scores.append(brightness_score)
            
            # 对比度评估
            contrast_score = self._evaluate_contrast(gray)
            contrast_scores.append(contrast_score)
            
            # 动态范围
            dynamic_range = np.max(gray) - np.min(gray)
            dynamic_ranges.append(dynamic_range)
            
            # 曝光准确性
            exposure_score = self._evaluate_exposure(gray)
            exposure_scores.append(exposure_score)
        
        metrics.brightness_score = statistics.mean(brightness_scores)
        metrics.contrast_score = statistics.mean(contrast_scores)
        metrics.dynamic_range = statistics.mean(dynamic_ranges)
        metrics.exposure_accuracy = statistics.mean(exposure_scores)
        
        self.logger.info(f"曝光对比度分析: 亮度{metrics.brightness_score:.2f}, "
                        f"对比度{metrics.contrast_score:.2f}")
        
        return metrics
    
    def _analyze_noise(self, images: List[np.ndarray], 
                      metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """分析图像噪声"""
        noise_levels = []
        snr_values = []
        
        for image in images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 噪声水平估计
            noise_level = self._estimate_noise_level(gray)
            noise_levels.append(noise_level)
            
            # 信噪比计算
            snr = self._calculate_snr(gray)
            snr_values.append(snr)
        
        metrics.noise_level = statistics.mean(noise_levels)
        metrics.snr_db = statistics.mean(snr_values)
        
        self.logger.info(f"噪声分析: 噪声水平{metrics.noise_level:.3f}, "
                        f"SNR{metrics.snr_db:.2f}dB")
        
        return metrics
    
    def _calculate_overall_quality(self, metrics: ImageQualityMetrics) -> ImageQualityMetrics:
        """计算整体质量评分"""
        # 权重设置
        weights = {
            'resolution': 0.15,
            'color': 0.25,
            'sharpness': 0.25,
            'exposure': 0.20,
            'noise': 0.15
        }
        
        # 计算各项评分
        resolution_score = 100 if metrics.resolution_match else 50
        color_score = (metrics.color_accuracy_score + metrics.white_balance_score) / 2
        sharpness_score = metrics.sharpness_score
        exposure_score = (metrics.brightness_score + metrics.contrast_score) / 2
        noise_score = max(0, 100 - metrics.noise_level * 100)
        
        # 加权平均
        overall_score = (
            resolution_score * weights['resolution'] +
            color_score * weights['color'] +
            sharpness_score * weights['sharpness'] +
            exposure_score * weights['exposure'] +
            noise_score * weights['noise']
        )
        
        metrics.overall_quality_score = overall_score
        
        # 质量等级
        if overall_score >= 90:
            metrics.quality_grade = "EXCELLENT"
        elif overall_score >= 80:
            metrics.quality_grade = "GOOD"
        elif overall_score >= 70:
            metrics.quality_grade = "ACCEPTABLE"
        elif overall_score >= 60:
            metrics.quality_grade = "POOR"
        else:
            metrics.quality_grade = "UNACCEPTABLE"
        
        return metrics
    
    # 辅助方法
    def _generate_reference_patterns(self) -> Dict[str, np.ndarray]:
        """生成参考图案"""
        patterns = {}
        
        # 棋盘格图案（用于几何校正和清晰度测试）
        patterns['checkerboard'] = np.zeros((480, 640), dtype=np.uint8)
        for i in range(0, 480, 40):
            for j in range(0, 640, 40):
                if (i//40 + j//40) % 2 == 0:
                    patterns['checkerboard'][i:i+40, j:j+40] = 255
        
        return patterns
    
    def _create_color_checker(self) -> np.ndarray:
        """创建颜色检查器"""
        # 简化的颜色检查器，包含标准颜色
        colors = [
            [115, 82, 68],   # 深棕色
            [194, 150, 130], # 浅棕色
            [98, 122, 157],  # 蓝色
            [87, 108, 67],   # 绿色
            [133, 128, 177], # 紫色
            [103, 189, 170], # 青色
        ]
        
        checker = np.zeros((120, 360, 3), dtype=np.uint8)
        for i, color in enumerate(colors):
            x_start = i * 60
            checker[:, x_start:x_start+60] = color
        
        return checker
    
    def _evaluate_color_accuracy(self, image: np.ndarray, lab_image: np.ndarray) -> float:
        """评估色彩准确性"""
        # 简化的色彩准确性评估
        # 计算各通道的标准差作为色彩丰富度指标
        l_std = np.std(lab_image[:, :, 0])
        a_std = np.std(lab_image[:, :, 1])
        b_std = np.std(lab_image[:, :, 2])
        
        # 归一化评分
        score = min(100, (l_std + a_std + b_std) / 3)
        return score
    
    def _calculate_delta_e(self, lab_image: np.ndarray) -> float:
        """计算Delta E（简化版）"""
        # 计算与标准白点的色差
        reference_white = [100, 0, 0]  # LAB空间中的白点
        
        # 取图像中心区域的平均值
        h, w = lab_image.shape[:2]
        center_region = lab_image[h//4:3*h//4, w//4:3*w//4]
        mean_lab = np.mean(center_region.reshape(-1, 3), axis=0)
        
        # 简化的Delta E计算
        delta_e = np.sqrt(np.sum((mean_lab - reference_white) ** 2))
        return delta_e
    
    def _evaluate_white_balance(self, image: np.ndarray) -> float:
        """评估白平衡"""
        # 计算RGB通道的均值比例
        b_mean = np.mean(image[:, :, 0])
        g_mean = np.mean(image[:, :, 1])
        r_mean = np.mean(image[:, :, 2])
        
        # 理想情况下，RGB比例应该接近1:1:1
        ratio_rg = r_mean / (g_mean + 1e-6)
        ratio_bg = b_mean / (g_mean + 1e-6)
        
        # 计算偏离理想比例的程度
        deviation = abs(ratio_rg - 1.0) + abs(ratio_bg - 1.0)
        score = max(0, 100 - deviation * 50)
        
        return score
    
    def _evaluate_saturation(self, hsv_image: np.ndarray) -> float:
        """评估饱和度"""
        saturation = hsv_image[:, :, 1]
        mean_saturation = np.mean(saturation)
        
        # 适中的饱和度得分更高
        if mean_saturation < 50:
            score = mean_saturation * 2  # 饱和度过低
        elif mean_saturation > 200:
            score = (255 - mean_saturation) * 2  # 饱和度过高
        else:
            score = 100  # 适中的饱和度
        
        return min(100, max(0, score))
    
    def _detect_blur(self, gray_image: np.ndarray) -> float:
        """检测模糊程度"""
        # 使用Sobel算子检测边缘
        sobelx = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)
        edge_magnitude = np.sqrt(sobelx**2 + sobely**2)
        
        # 高边缘强度表示图像清晰
        blur_score = np.mean(edge_magnitude)
        return min(100, blur_score / 100)
    
    def _calculate_edge_density(self, gray_image: np.ndarray) -> float:
        """计算边缘密度"""
        edges = cv2.Canny(gray_image, 50, 150)
        edge_pixels = np.sum(edges > 0)
        total_pixels = edges.size
        edge_density = edge_pixels / total_pixels
        return edge_density
    
    def _evaluate_brightness(self, mean_brightness: float) -> float:
        """评估亮度"""
        # 理想亮度在100-150之间
        if 100 <= mean_brightness <= 150:
            score = 100
        elif mean_brightness < 100:
            score = mean_brightness  # 过暗
        else:
            score = max(0, 200 - mean_brightness)  # 过亮
        
        return min(100, max(0, score))
    
    def _evaluate_contrast(self, gray_image: np.ndarray) -> float:
        """评估对比度"""
        # 使用标准差作为对比度指标
        contrast = np.std(gray_image)
        
        # 适当的对比度应该在40-80之间
        if 40 <= contrast <= 80:
            score = 100
        elif contrast < 40:
            score = contrast * 2.5  # 对比度过低
        else:
            score = max(0, 160 - contrast)  # 对比度过高
        
        return min(100, max(0, score))
    
    def _evaluate_exposure(self, gray_image: np.ndarray) -> float:
        """评估曝光准确性"""
        # 计算直方图
        hist = cv2.calcHist([gray_image], [0], None, [256], [0, 256])
        hist = hist.flatten() / hist.sum()
        
        # 良好的曝光应该在整个动态范围内有分布
        # 检查过曝（高亮部分）和欠曝（阴影部分）
        overexposed = np.sum(hist[240:])  # 过曝像素比例
        underexposed = np.sum(hist[:15])  # 欠曝像素比例
        
        # 计算曝光评分
        exposure_penalty = (overexposed + underexposed) * 100
        score = max(0, 100 - exposure_penalty)
        
        return score
    
    def _estimate_noise_level(self, gray_image: np.ndarray) -> float:
        """估计噪声水平"""
        # 使用高频成分估计噪声
        # 应用高斯滤波器然后计算差值
        blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
        noise = gray_image.astype(np.float32) - blurred.astype(np.float32)
        noise_level = np.std(noise) / 255.0  # 归一化
        
        return noise_level
    
    def _calculate_snr(self, gray_image: np.ndarray) -> float:
        """计算信噪比"""
        # 信号功率（图像方差）
        signal_power = np.var(gray_image)
        
        # 噪声功率估计
        noise_level = self._estimate_noise_level(gray_image)
        noise_power = (noise_level * 255) ** 2
        
        # SNR计算（dB）
        if noise_power > 0:
            snr_db = 10 * np.log10(signal_power / noise_power)
        else:
            snr_db = float('inf')
        
        return min(60, max(0, snr_db))  # 限制在合理范围内

    def evaluate_quality_thresholds(self, metrics: ImageQualityMetrics) -> Dict[str, Any]:
        """根据阈值评估图像质量"""
        thresholds = self.config.get("image_quality_thresholds", {})
        
        evaluation = {
            "overall_status": "PASS",
            "issues": [],
            "recommendations": [],
            "scores": {}
        }
        
        # 分辨率检查
        if not metrics.resolution_match:
            evaluation["issues"].append(
                f"分辨率不匹配: 实际{metrics.resolution_actual}, 目标{metrics.resolution_target}"
            )
        
        # 色彩准确性检查
        max_delta_e = thresholds.get("max_color_delta_e", 5.0)
        if metrics.color_delta_e > max_delta_e:
            evaluation["issues"].append(f"色彩偏差过大: ΔE={metrics.color_delta_e:.1f} > {max_delta_e}")
        
        # 清晰度检查
        min_sharpness = thresholds.get("min_sharpness_score", 0.7) * 100
        if metrics.sharpness_score < min_sharpness:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"清晰度不足: {metrics.sharpness_score:.1f} < {min_sharpness}")
        
        # 对比度检查
        min_contrast = thresholds.get("min_contrast_ratio", 0.3) * 100
        if metrics.contrast_score < min_contrast:
            evaluation["issues"].append(f"对比度过低: {metrics.contrast_score:.1f} < {min_contrast}")
        
        # 噪声检查
        max_noise = thresholds.get("max_noise_level", 0.1)
        if metrics.noise_level > max_noise:
            evaluation["issues"].append(f"噪声过高: {metrics.noise_level:.3f} > {max_noise}")
        
        # 生成建议
        if metrics.overall_quality_score < 70:
            evaluation["recommendations"].extend([
                "检查摄像头镜头是否清洁",
                "调整光照条件",
                "验证摄像头焦距设置"
            ])
        
        if metrics.color_delta_e > 3:
            evaluation["recommendations"].append("调整白平衡设置")
        
        if metrics.noise_level > 0.05:
            evaluation["recommendations"].append("降低ISO或改善光照条件")
        
        evaluation["scores"] = {
            "resolution": 100 if metrics.resolution_match else 0,
            "color": metrics.color_accuracy_score,
            "sharpness": metrics.sharpness_score,
            "exposure": metrics.brightness_score,
            "overall": metrics.overall_quality_score
        }
        
        return evaluation

# 测试函数
def test_image_quality_analyzer():
    """测试图像质量分析器"""
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # 创建摄像头配置
    with CameraConfig() as camera_config:
        if not camera_config.initialize_camera():
            print("摄像头初始化失败，无法进行图像质量分析")
            return
        
        # 创建图像质量分析器
        analyzer = ImageQualityAnalyzer(camera_config)
        
        # 运行图像质量分析
        print("运行图像质量分析...")
        metrics = analyzer.analyze_image_quality(sample_count=10)
        
        # 评估质量
        evaluation = analyzer.evaluate_quality_thresholds(metrics)
        
        print(f"\n图像质量分析结果:")
        print(f"分辨率: {metrics.resolution_actual} (匹配: {metrics.resolution_match})")
        print(f"整体质量评分: {metrics.overall_quality_score:.2f} ({metrics.quality_grade})")
        print(f"清晰度: {metrics.sharpness_score:.2f}")
        print(f"色彩准确性: {metrics.color_accuracy_score:.2f}")
        print(f"噪声水平: {metrics.noise_level:.3f}")
        print(f"评估状态: {evaluation['overall_status']}")

if __name__ == "__main__":
    test_image_quality_analyzer() 