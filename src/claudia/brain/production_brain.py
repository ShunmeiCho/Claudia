#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Brain Fixed - 修复SportClient初始化和提示词问题
"""

import json
import time
import asyncio
import logging
import subprocess
import random
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
from functools import lru_cache

# Track A新增导入
try:
    import ollama  # Python ollama库
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

try:
    from claudia.robot_controller.system_state_monitor import (
        create_system_state_monitor,
        SystemStateInfo,
        SystemState
    )
    STATE_MONITOR_AVAILABLE = True
except ImportError:
    STATE_MONITOR_AVAILABLE = False

try:
    from claudia.brain.safety_validator import get_safety_validator, SafetyCheckResult
    SAFETY_VALIDATOR_AVAILABLE = True
except ImportError:
    SAFETY_VALIDATOR_AVAILABLE = False

try:
    from claudia.brain.audit_logger import get_audit_logger, AuditEntry
    AUDIT_LOGGER_AVAILABLE = True
except ImportError:
    AUDIT_LOGGER_AVAILABLE = False

@dataclass
class BrainOutput:
    """大脑输出格式"""
    response: str           # 日语TTS回复
    api_code: Optional[int] # 单个动作API
    sequence: Optional[List[int]] = None # 动作序列
    confidence: float = 1.0
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        result = {
            "response": self.response,
            "api_code": self.api_code
        }
        if self.sequence:
            result["sequence"] = self.sequence
        return result

class ProductionBrain:
    """生产大脑 - 使用修复后的模型"""
    
    def __init__(self, use_real_hardware: bool = False):
        self.logger = self._setup_logger()
        self.use_real_hardware = use_real_hardware

        # Track A/B模型配置（修复REVIEW：支持环境变量切换）
        import os
        self.model_3b = os.getenv("BRAIN_MODEL_3B", "claudia-go2-3b:v11.2")  # Track A默认
        self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-go2-7b:v7")      # Track A默认

        # 灰度切流配置（可选）
        self.ab_test_ratio = float(os.getenv("AB_TEST_RATIO", "0.0"))  # 0.0=全Track A, 1.0=全Track B
        if self.ab_test_ratio > 0:
            self.logger.info(f"🔬 A/B测试启用: {self.ab_test_ratio*100:.0f}%流量→Track B")

        self.logger.info(f"📌 3B模型: {self.model_3b}")
        self.logger.info(f"📌 7B模型: {self.model_7b}")
        
        # 扩展动作缓存（包含容易出错的命令）
        self.hot_cache = {
            # 基础命令
            "お手": {"response": "こんにちは", "api_code": 1016},  # Hello动作代替握手
            "座って": {"response": "座ります", "api_code": 1009},
            "おすわり": {"response": "お座りします", "api_code": 1009},
            "立って": {"response": "立ちます", "api_code": 1004},
            "タッテ": {"response": "立ちます", "api_code": 1004},
            "比心": {"response": "ハートします", "api_code": 1036},  # 修正：使用Heart而不是Wallow
            "ハート": {"response": "ハートします", "api_code": 1036},  # 修正：使用Heart而不是Wallow
            "ダンス": {"response": "踊ります", "api_code": 1022},
            "踊って": {"response": "踊ります", "api_code": 1022},
            "停止": {"response": "止まります", "api_code": 1003},
            "止まれ": {"response": "止まります", "api_code": 1003},
            "挨拶": {"response": "挨拶します", "api_code": 1016},
            "こんにちは": {"response": "こんにちは", "api_code": 1016},
            
            # 修复容易出错的命令
            "お辞儀": {"response": "お辞儀します", "api_code": 1030},  # 鞠躬
            "礼": {"response": "お辞儀します", "api_code": 1030},
            "礼して": {"response": "お辞儀します", "api_code": 1030},
            "ちんちん": {"response": "お辞儀します", "api_code": 1016},  # 拜年动作用挨拶
            "ちんちんして": {"response": "お辞儀します", "api_code": 1016},  # 拜年动作变形
            "チンチン": {"response": "お辞儀します", "api_code": 1016},
            "拜年": {"response": "お辞儀します", "api_code": 1016},  # 拜年动作用挨拶
            "お祝い": {"response": "こんにちは", "api_code": 1016},  # 用挨拶代替
            
            # 添加更多容易误解的命令
            "伸懒腰": {"response": "伸びをします", "api_code": 1017},  # 伸展
            "伸び": {"response": "伸びをします", "api_code": 1017},
            "ノビ": {"response": "伸びをします", "api_code": 1017},
            "ストレッチ": {"response": "伸びをします", "api_code": 1017},
            "倒立": {"response": "倒立します", "api_code": 1031},  # 倒立
            "サカダチ": {"response": "倒立します", "api_code": 1031},
            "横になる": {"response": "横になります", "api_code": 1005},  # 趴下
            "横になって": {"response": "横になります", "api_code": 1005},
            "翻身": {"response": "ゴロンします", "api_code": 1010},  # 翻身
            "ゴロン": {"response": "ゴロンします", "api_code": 1010},
            
            # 添加测试验证的新动作
            "前空翻": {"response": "前転します", "api_code": 1030},  # FrontFlip
            "前跳": {"response": "前跳します", "api_code": 1031},    # FrontJump  
            "前扑": {"response": "前扑します", "api_code": 1032},    # FrontPounce
            "刮擦": {"response": "擦ります", "api_code": 1029},      # Scrape
            
            # 新发现的支持动作
            "阻尼": {"response": "ダンプモード", "api_code": 1001},   # Damp
            "ダンプ": {"response": "ダンプモード", "api_code": 1001},
            "平衡": {"response": "バランスします", "api_code": 1002}, # BalanceStand
            "バランス": {"response": "バランスします", "api_code": 1002},
            "恢复": {"response": "回復します", "api_code": 1006},     # RecoveryStand
            "回復": {"response": "回復します", "api_code": 1006},
            "起坐": {"response": "起き上がります", "api_code": 1010},  # RiseSit
            "起き上がる": {"response": "起き上がります", "api_code": 1010},
            "舞踊2": {"response": "踊ります2", "api_code": 1023},     # Dance2
            "ダンス2": {"response": "踊ります2", "api_code": 1023},
            "摆姿势": {"response": "ポーズします", "api_code": 1028},  # Pose
            "ポーズ": {"response": "ポーズします", "api_code": 1028},
            
            # 日语语义理解缓存 - 扩展可愛い变形
            "可愛い": {"response": "ハートします", "api_code": 1036},
            "可愛いね": {"response": "ハートします", "api_code": 1036},
            "可愛いな": {"response": "ハートします", "api_code": 1036},
            "可愛いよ": {"response": "ハートします", "api_code": 1036},
            "可愛いです": {"response": "ハートします", "api_code": 1036},
            "可愛すぎる": {"response": "ハートします", "api_code": 1036},
            "かわいい": {"response": "ハートします", "api_code": 1036},
            "かわいいね": {"response": "ハートします", "api_code": 1036},
            "かわいいな": {"response": "ハートします", "api_code": 1036},
            "暗ちゃん可愛い": {"response": "ハートします", "api_code": 1036},
            "暗ちゃん可愛いね": {"response": "ハートします", "api_code": 1036},
            "暗ちゃん　可愛いね": {"response": "ハートします", "api_code": 1036},
            "くらちゃん可愛い": {"response": "ハートします", "api_code": 1036},
            "疲れた": {"response": "座ります", "api_code": 1009},
            "元気": {"response": "踊ります", "api_code": 1023},
            "ジャンプ": {"response": "前跳します", "api_code": 1031},
            
            # 英文命令缓存
            "damp": {"response": "ダンプモード", "api_code": 1001},
            "balance": {"response": "バランスします", "api_code": 1002},
            "stop": {"response": "止まります", "api_code": 1003},
            "stand": {"response": "立ちます", "api_code": 1004},
            "down": {"response": "伏せます", "api_code": 1005},
            "recovery": {"response": "回復します", "api_code": 1006},
            "sit": {"response": "座ります", "api_code": 1009},
            "rise": {"response": "起き上がります", "api_code": 1010},
            "hello": {"response": "挨拶します", "api_code": 1016},
            "hi": {"response": "こんにちは", "api_code": 1016},
            "stretch": {"response": "伸びをします", "api_code": 1017},
            # 舞蹈动作 - 支持明确选择和随机选择
            "dance1": {"response": "踊ります1", "api_code": 1022},
            "dance2": {"response": "踊ります2", "api_code": 1023},
            "ダンス1": {"response": "踊ります1", "api_code": 1022},
            "ダンス2": {"response": "踊ります2", "api_code": 1023},
            "跳舞1": {"response": "踊ります1", "api_code": 1022},
            "跳舞2": {"response": "踊ります2", "api_code": 1023},
            "舞蹈1": {"response": "踊ります1", "api_code": 1022},
            "舞蹈2": {"response": "踊ります2", "api_code": 1023},
            # dance/ダンス将在special_处理中随机选择
            "scrape": {"response": "擦ります", "api_code": 1029},
            "heart": {"response": "ハートします", "api_code": 1036},
            "pose": {"response": "ポーズします", "api_code": 1028},
            "jump": {"response": "前跳します", "api_code": 1031},
            "flip": {"response": "前転します", "api_code": 1030},
            "pounce": {"response": "前扑します", "api_code": 1032},
            "cute": {"response": "ハートします", "api_code": 1036},
            "tired": {"response": "座ります", "api_code": 1009},
            
            # 动作变形缓存（して后缀）
            "座って": {"response": "座ります", "api_code": 1009},
            "立って": {"response": "立ちます", "api_code": 1004},
            "挨拶して": {"response": "挨拶します", "api_code": 1016},
            "ダンス1して": {"response": "踊ります1", "api_code": 1022},
            "ダンス2して": {"response": "踊ります2", "api_code": 1023},
            "ジャンプして": {"response": "前跳します", "api_code": 1031},
            "ハートして": {"response": "ハートします", "api_code": 1036},
            "ストレッチして": {"response": "伸びをします", "api_code": 1017},
            
            # 常见复杂序列缓存
            "座ってから挨拶": {"response": "座って挨拶します", "sequence": [1009, 1004, 1016]},
            # 序列中的舞蹈使用Dance2作为默认，也可明确指定
            "座ってからダンス": {"response": "座って踊ります", "sequence": [1009, 1004, 1023]},
            "座ってからダンス1": {"response": "座って踊ります1", "sequence": [1009, 1004, 1022]},
            "座ってからダンス2": {"response": "座って踊ります2", "sequence": [1009, 1004, 1023]},
            "挨拶してからダンス": {"response": "挨拶してから踊ります", "sequence": [1016, 1023]},
            "挨拶したらダンス": {"response": "挨拶してから踊ります", "sequence": [1016, 1023]},
            "挨拶してからダンス1": {"response": "挨拶してから踊ります1", "sequence": [1016, 1022]},
            "挨拶したらダンス1": {"response": "挨拶してから踊ります1", "sequence": [1016, 1022]},
        }
        
        # 复杂序列检测关键词 - 扩展日语连接词
        self.sequence_keywords = [
            # 中文连接词
            "然后", "接着", "一套", "表演",
            
            # 日语连接词（重点扩展）
            "てから", "その後", "それから",    # 然后、之后
            "したら", "すれば", "なら",        # 如果...就...
            "次に", "つぎに", "それで",        # 接下来
            "してから", "したあと",           # 做了...之后
            
            # 组合动作关键词  
            "連続", "れんぞく",               # 连续
            "パフォーマンス", "芸", "技",      # 表演、技能
            "一緒に", "同時に",               # 一起、同时
            "順番に", "順序",                 # 按顺序
        ]
        
        # SportClient连接（如果是真实硬件）
        self.sport_client = None
        if use_real_hardware:
            self._init_sport_client()
        
        # 机器人状态管理
        self.robot_state = "unknown"  # unknown, standing, sitting, lying
        self.actions_need_standing = [
            1016, 1017, 1022, 1023, 1029, 1030, 1031, 1032, 1036  # 修正：1036是真正的比心
        ]

        # Track A新增：状态监控器
        self.state_monitor = None
        if STATE_MONITOR_AVAILABLE:
            try:
                self.state_monitor = create_system_state_monitor(
                    node_name="claudia_brain_monitor",
                    update_rate=5.0  # 5Hz更新
                )
                if self.state_monitor.initialize():
                    self.state_monitor.start_monitoring()
                    self.logger.info("✅ 状态监控器已启动")
                else:
                    self.logger.warning("⚠️ 状态监控器初始化失败，使用默认状态")
            except Exception as e:
                self.logger.warning(f"⚠️ 状态监控器不可用: {e}")
        else:
            self.logger.warning("⚠️ 状态监控器模块不可用")

        # Track A新增：安全验证器
        if SAFETY_VALIDATOR_AVAILABLE:
            self.safety_validator = get_safety_validator(enable_high_risk=False)  # Track A初期禁用高风险
            self.logger.info("✅ 安全验证器已加载（高风险动作已禁用）")
        else:
            self.safety_validator = None
            self.logger.warning("⚠️ 安全验证器不可用")

        # Track A新增：审计日志器（用于A/B决策和回滚）
        if AUDIT_LOGGER_AVAILABLE:
            self.audit_logger = get_audit_logger()
            self.logger.info("✅ 审计日志器已启动 (logs/audit/)")
        else:
            self.audit_logger = None
            self.logger.warning("⚠️ 审计日志器不可用")

        self.logger.info("🧠 生产大脑初始化完成")
        self.logger.info(f"   3B模型: {self.model_3b}")
        self.logger.info(f"   7B模型: {self.model_7b}")
        self.logger.info(f"   硬件模式: {'真实' if use_real_hardware else '模拟'}")
    
    def _setup_logger(self) -> logging.Logger:
        """设置日志"""
        logger = logging.getLogger("ProductionBrain")
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('🧠 %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        return logger
    
    def _init_sport_client(self):
        """修复的SportClient初始化 - 包含正确的网络配置"""
        try:
            import sys
            import os
            
            # 添加正确的路径
            sys.path.append('/home/m1ng/claudia')
            sys.path.append('/home/m1ng/claudia/unitree_sdk2_python')
            
            # 设置正确的环境变量 - 这是关键修复！
            os.environ['CYCLONEDDS_HOME'] = '/home/m1ng/claudia/cyclonedds/install'
            
            # 设置LD_LIBRARY_PATH
            ld_path = os.environ.get('LD_LIBRARY_PATH', '')
            cyclone_lib = '/home/m1ng/claudia/cyclonedds/install/lib'
            unitree_lib = '/home/m1ng/claudia/cyclonedds_ws/install/unitree_sdk2/lib'
            
            if cyclone_lib not in ld_path:
                os.environ['LD_LIBRARY_PATH'] = f"{cyclone_lib}:{unitree_lib}:{ld_path}"
            
            # 设置RMW实现
            os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
            
            # 设置网络配置 - 使用官方推荐的内联配置！
            os.environ['CYCLONEDDS_URI'] = '''<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'''
            
            # 导入必要的模块
            from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            
            # 初始化DDS通道工厂 - 这是关键步骤！
            self.logger.info("📡 初始化DDS通道工厂 (eth0)...")
            ChannelFactoryInitialize(0, "eth0")
            
            # 创建SportClient实例
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            
            # 测试连接 - 使用更可靠的命令
            import time
            time.sleep(0.5)  # 给DDS一点时间建立连接
            
            # 测试连接
            try:
                # 使用RecoveryStand测试连接
                test_result = self.sport_client.RecoveryStand()
                
                # 分析返回码
                if test_result == 0:
                    self.logger.info("✅ 真实SportClient初始化成功 - 机器人已连接")
                    self.logger.info(f"   网络接口: eth0")
                    self.logger.info(f"   本机IP: 192.168.123.18")
                    self.logger.info(f"   机器人IP: 192.168.123.161")
                    self.logger.info(f"   测试返回码: {test_result}")
                    
                elif test_result == 3103:
                    # APP占用问题 - 这是最常见的问题
                    self.logger.error("="*60)
                    self.logger.error("❌ 检测到APP占用sport_mode (错误码3103)")
                    self.logger.error("")
                    self.logger.error("原因：SDK和APP不能同时控制机器人")
                    self.logger.error("这是Unitree的安全设计，不是故障")
                    self.logger.error("")
                    self.logger.error("解决步骤：")
                    self.logger.error("1. 关闭手机上的Unitree Go APP")
                    self.logger.error("2. 按住机器人电源键重启")
                    self.logger.error("3. 等待30秒后重新运行程序")
                    self.logger.error("")
                    self.logger.error("或使用: ./start_sdk_exclusive.sh")
                    self.logger.error("="*60)
                    self.logger.warning("切换到模拟模式继续...")
                    self._init_mock_client()
                    return  # 使用模拟客户端
                    
                elif test_result == 3203:
                    self.logger.warning("⚠️ API未实现 (3203) - 该机器人可能不支持某些动作")
                    self.logger.info("   SportClient已创建，继续运行...")
                    
                else:
                    self.logger.warning(f"⚠️ 连接测试返回码: {test_result}")
                    self.logger.info("   SportClient已创建，继续运行...")
                    
            except Exception as e:
                self.logger.warning(f"⚠️ 连接测试异常: {e}")
                self.logger.info("   SportClient已创建，继续运行...")
            
        except ImportError as e:
            self.logger.error(f"❌ 导入错误: {e}")
            self.logger.info("   使用MockSportClient模拟硬件")
            self._init_mock_client()
            
        except Exception as e:
            self.logger.error(f"❌ SportClient初始化失败: {e}")
            self.logger.info("   提示: 机器人可能未连接")
            self.logger.info("   使用MockSportClient模拟硬件")
            self._init_mock_client()
    
    def _init_mock_client(self):
        """初始化模拟客户端"""
        try:
            from src.claudia.brain.mock_sport_client import MockSportClient
            self.sport_client = MockSportClient()
            self.sport_client.Init()
            self.logger.info("🎭 MockSportClient初始化成功（模拟模式）")
            # 保持硬件模式标志，但使用模拟客户端
            # 这样用户知道系统在尝试硬件控制，只是用模拟代替
        except Exception as e:
            self.logger.error(f"❌ MockSportClient初始化失败: {e}")
            self.sport_client = None
            self.use_real_hardware = False
    
    def _is_complex_command(self, command: str) -> bool:
        """判断是否为复杂指令"""
        return any(keyword in command for keyword in self.sequence_keywords)
    
    @lru_cache(maxsize=128)
    def _call_ollama(self, model: str, command: str, timeout: int = 10) -> Optional[Dict]:
        """调用Ollama模型"""
        try:
            # 首先检查模型是否存在
            check_cmd = f"ollama list | grep {model.split(':')[0]}"
            check_result = subprocess.run(
                check_cmd,
                shell=True,
                capture_output=True,
                text=True
            )
            
            if model not in check_result.stdout:
                self.logger.error(f"模型不存在: {model}")
                # 尝试创建模型
                if "v7.0" in model:
                    create_cmd = f"ollama create {model} -f ClaudiaProduction3B_v7.0"
                elif "v8.0" in model:
                    create_cmd = f"ollama create {model} -f ClaudiaFinal3B_v8.0"
                else:
                    return None
                    
                subprocess.run(create_cmd, shell=True, capture_output=True)
                self.logger.info(f"创建模型: {model}")
            
            cmd = f'echo "{command}" | timeout {timeout} ollama run {model}'
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                encoding='utf-8'
            )
            
            if result.returncode == 124:  # Timeout
                self.logger.warning(f"模型超时: {model}")
                return None
            
            # 解析JSON响应
            if result.stdout:
                response_text = result.stdout.strip()
                try:
                    # 尝试提取JSON对象
                    if "{" in response_text and "}" in response_text:
                        start_idx = response_text.find("{")
                        end_idx = response_text.rfind("}")  # 使用rfind找最后一个}
                        if start_idx != -1 and end_idx != -1:
                            json_str = response_text[start_idx:end_idx+1]
                            # 清理可能的特殊字符
                            json_str = json_str.replace("\n", " ").replace("\r", "")
                            return json.loads(json_str)
                    # 如果没有JSON格式，尝试直接解析
                    return json.loads(response_text)
                except json.JSONDecodeError:
                    self.logger.error(f"JSON解析失败: {response_text[:100]}...")  # 只显示前100字符
                    return None
            
            return None

        except Exception as e:
            self.logger.error(f"Ollama调用错误: {e}")
            return None

    def _normalize_battery(self, level: Optional[float]) -> Optional[float]:
        """
        统一电量归一化到 0.0~1.0

        Args:
            level: 电量值（可能是0-1或0-100）

        Returns:
            归一化后的电量值（0.0-1.0），如果输入为None则返回None
        """
        if level is None:
            return None
        return (level / 100.0) if level > 1.0 else level

    def _quick_safety_precheck(self, command: str, state: Optional['SystemStateInfo']) -> Optional[str]:
        """
        快速安全预检：在LLM前执行（毫秒级）

        Args:
            command: 用户命令
            state: 当前状态（已归一化）

        Returns:
            如果不安全返回拒绝理由，否则返回None（允许继续）
        """
        if not state or state.battery_level is None:
            return None

        b = state.battery_level  # 已归一化到0.0-1.0
        cmd = command.lower()

        # 极低电量（≤10%）: 只允许sit/stop/stand关键词
        if b <= 0.10:
            safe_kw = ('sit', 'stop', 'stand', '座', '立', '止', 'やめ', 'とまれ')
            if not any(k in cmd for k in safe_kw):
                return f"電池残量が極めて低い状態です ({b*100:.0f}%)。Sit/Stand/Stopのみ使用できます。"

        # 低电量（≤20%）: 拒绝明显的高能关键词
        if b <= 0.20:
            high_kw = ('flip', '転', 'jump', '跳', 'pounce', '飛', 'かっこいい')
            if any(k in cmd for k in high_kw):
                return f"電池残量が低い状態です ({b*100:.0f}%)。高エネルギー動作は禁止されています。"

        return None  # 允许继续

    def _final_safety_gate(self, api_code: Optional[int], state: Optional['SystemStateInfo']) -> Tuple[Optional[int], str]:
        """
        最终安全门：在执行前硬性收口（不依赖LLM/SafetyValidator）

        Args:
            api_code: LLM返回的动作码
            state: 当前状态（已归一化）

        Returns:
            (safe_api_code, reason) - 如果拒绝则返回(None, reason)；降级则返回(new_code, reason)
        """
        if api_code is None or not state or state.battery_level is None:
            return api_code, "ok"

        b = state.battery_level  # 已归一化到0.0-1.0
        HIGH = (1030, 1031, 1032)  # Flip, Jump, Pounce

        # 极低电量（≤10%）: 只允许1003/1009/1004
        if b <= 0.10:
            if api_code not in (1003, 1009, 1004, None):
                return None, f"Final gate: Battery {b*100:.0f}% too low for action {api_code}"

        # 低电量（≤20%）: 禁止高能动作
        elif b <= 0.20:
            if api_code in HIGH:
                return None, f"Final gate: Battery {b*100:.0f}% insufficient for high-energy action {api_code}"

        # 中等电量（≤30%）: 高能动作降级为Dance
        elif b <= 0.30:
            if api_code in HIGH:
                return 1023, f"Final gate: Downgraded {api_code}→Dance at {b*100:.0f}%"

        return api_code, "ok"

    def _try_hotpath(self, command: str) -> Optional[int]:
        """
        热路径：高频基础命令直达（仍走安全门）

        Args:
            command: 用户命令

        Returns:
            api_code或None（None表示需要走LLM）
        """
        cmd = command.strip().lower()
        HOTPATH_MAP = {
            # 日语
            '座って': 1009, 'すわって': 1009, '座る': 1009,
            '立って': 1004, 'たって': 1004, '立つ': 1004,
            'とまれ': 1003, 'やめて': 1003, 'ストップ': 1003,
            'ハート': 1036, 'はーと': 1036,
            'ダンス': 1023, 'だんす': 1023,
            'こんにちは': 1016, 'ハロー': 1016,
            # 英语
            'sit': 1009, 'stand': 1004, 'stop': 1003,
            'heart': 1036, 'dance': 1023, 'hello': 1016,
            # 中文
            '坐下': 1009, '站立': 1004, '停止': 1003,
            '爱心': 1036, '跳舞': 1023, '你好': 1016,
        }
        return HOTPATH_MAP.get(cmd)

    async def _call_ollama_v2(self, model: str, command: str, timeout: int = 10) -> Optional[Dict]:
        """
        调用Ollama（Track A优化版）
        - 使用Python ollama库
        - asyncio.to_thread避免阻塞事件循环
        - 结构化JSON输出
        """
        if not OLLAMA_AVAILABLE:
            self.logger.warning("ollama库不可用，使用旧方法")
            return self._call_ollama(model, command, timeout)

        try:
            # 在线程池中执行ollama调用（避免阻塞事件循环）
            def _sync_ollama_call():
                try:
                    ollama.show(model)  # 检查模型存在
                except Exception:
                    self.logger.error(f"模型不存在: {model}")
                    return None

                # 修复REVIEW：Track B模型需要更大的num_predict避免JSON截断
                # 生成参数收敛优化
                is_track_b = "intelligent" in model.lower()
                if is_track_b:
                    num_predict = 128  # 从256降到128（足够生成完整JSON）
                    num_ctx = 512      # 从2048降到512（只需理解当前指令）
                else:
                    num_predict = 30   # Track A保持不变
                    num_ctx = 512      # Track A也缩减上下文

                response = ollama.chat(
                    model=model,
                    messages=[{'role': 'user', 'content': command}],
                    format='json',  # 强制JSON输出
                    options={
                        'temperature': 0.0,  # 改为0.0确保确定性输出
                        'num_predict': num_predict,
                        'num_ctx': num_ctx,
                        'top_p': 0.9,
                    }
                )

                content = response['message']['content']
                return json.loads(content)

            # 使用asyncio.to_thread避免阻塞（修复REVIEW问题）
            result = await asyncio.wait_for(
                asyncio.to_thread(_sync_ollama_call),
                timeout=timeout
            )
            return result

        except asyncio.TimeoutError:
            self.logger.warning(f"模型超时({timeout}s): {model}")
            return None
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON解析失败: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Ollama调用错误: {e}")
            return None

    def _build_enhanced_prompt(self, command: str, model_name: str,
                               current_state: Optional['SystemStateInfo']) -> str:
        """
        构建增强提示（Track B关键修复：注入状态信息）

        Args:
            command: 用户原始命令
            model_name: 模型名称（仅Track B模型注入状态）
            current_state: 当前系统状态

        Returns:
            增强后的提示（Track B添加[STATE]前缀，Track A保持原样）
        """
        # 仅对Track B模型（包含"intelligent"）注入状态
        is_track_b = "intelligent" in model_name.lower()

        if not is_track_b or not current_state:
            return command  # Track A或无状态时保持原样

        # 构造Track B期望的状态前缀格式（state已归一化）
        posture = "standing" if current_state.is_standing else "sitting"
        battery = (current_state.battery_level or 0.0) * 100.0  # 已归一化后还原为百分比显示
        space = "normal"  # 默认正常空间（可扩展）

        # Track B Modelfile期望格式: [STATE] posture:X, battery:Y%, space:Z
        state_prefix = f"[STATE] posture:{posture}, battery:{battery:.0f}%, space:{space}\n\n"
        enhanced = state_prefix + command

        self.logger.debug(f"🔧 状态注入: {state_prefix.strip()}")
        return enhanced

    def _log_audit(self, command: str, output: BrainOutput, route: str,
                   elapsed_ms: float, cache_hit: bool, model_used: str,
                   current_state: Optional['SystemStateInfo'],
                   llm_output: Optional[str], safety_verdict: str,
                   safety_reason: Optional[str] = None):
        """记录审计日志（Track A增强）"""
        if not self.audit_logger:
            return

        from datetime import datetime
        try:
            entry = AuditEntry(
                timestamp=datetime.now().isoformat(),
                model_name=model_used,
                input_command=command,
                state_battery=current_state.battery_level if current_state else None,
                state_standing=current_state.is_standing if current_state else None,
                state_emergency=current_state.state.name == "EMERGENCY" if current_state else None,
                llm_output=llm_output,
                api_code=output.api_code,
                sequence=output.sequence,
                safety_verdict=safety_verdict,
                safety_reason=safety_reason,
                elapsed_ms=elapsed_ms,
                cache_hit=cache_hit,
                route=route,
                success=output.api_code is not None or output.sequence is not None
            )
            self.audit_logger.log_entry(entry)
        except Exception as e:
            self.logger.warning(f"⚠️ 审计日志记录失败: {e}")

    async def process_command(self, command: str) -> BrainOutput:
        """处理用户指令（状态快照+热路径+安全门优化版）"""
        start_time = time.time()
        self.logger.info(f"📥 接收指令: '{command}'")

        # ===== 1) 一次性快照并统一归一化 =====
        state_snapshot = self.state_monitor.get_current_state() if self.state_monitor else None
        if state_snapshot:
            raw_batt = state_snapshot.battery_level
            state_snapshot.battery_level = self._normalize_battery(raw_batt)
            self.logger.info(
                f"📊 状态快照: 电池{state_snapshot.battery_level*100:.0f}%, "
                f"姿态{'站立' if state_snapshot.is_standing else '非站立'}"
            )

        # 0. 紧急指令快速通道（绕过LLM，修复REVIEW问题）
        EMERGENCY_BYPASS = {
            "緊急停止": {"response": "緊急停止しました", "api_code": 1003},
            "stop": {"response": "止まります", "api_code": 1003},
            "停止": {"response": "止まります", "api_code": 1003},
            "やめて": {"response": "止まります", "api_code": 1003},
            "ストップ": {"response": "止まります", "api_code": 1003},
        }
        if command.strip() in EMERGENCY_BYPASS:
            cached = EMERGENCY_BYPASS[command.strip()]
            elapsed = (time.time() - start_time) * 1000
            self.logger.info(f"🚨 紧急指令旁路 ({elapsed:.0f}ms)")
            output = BrainOutput(
                response=cached["response"],
                api_code=cached["api_code"]
            )
            # Track A：审计日志
            self._log_audit(command, output, route="emergency", elapsed_ms=elapsed,
                          cache_hit=False, model_used="bypass",
                          current_state=None, llm_output=None,
                          safety_verdict="bypass")
            return output

        # ===== 2) 快速安全预检（在LLM前，毫秒级） =====
        rejection_reason = self._quick_safety_precheck(command, state_snapshot)
        if rejection_reason:
            self.logger.warning(f"🛡️ 快速安全预检拒绝: {rejection_reason}")
            elapsed = (time.time() - start_time) * 1000
            self._log_audit(command, BrainOutput(response=rejection_reason, api_code=None),
                          route="precheck_reject", elapsed_ms=elapsed, cache_hit=False,
                          model_used="precheck", current_state=state_snapshot,
                          llm_output=None, safety_verdict="reject_precheck")
            return BrainOutput(
                response=rejection_reason,
                api_code=None,
                confidence=1.0,
                reasoning="Rejected by quick safety precheck before LLM"
            )

        # ===== 3) 热路径尝试（高频命令直达，节省秒级延迟） =====
        hotpath_api = self._try_hotpath(command)
        if hotpath_api is not None:
            self.logger.info(f"⚡ 热路径命中: {command} → {hotpath_api}")

            # 热路径也要走最终安全门
            safe_api, gate_reason = self._final_safety_gate(hotpath_api, state_snapshot)
            if safe_api != hotpath_api:
                # 被拒绝或降级
                elapsed = (time.time() - start_time) * 1000
                response_text = "安全のため動作を停止しました" if safe_api is None else "安全のため動作を調整しました"
                self._log_audit(command, BrainOutput(response=response_text, api_code=safe_api),
                              route="hotpath_gate", elapsed_ms=elapsed, cache_hit=True,
                              model_used="hotpath", current_state=state_snapshot,
                              llm_output=None, safety_verdict=f"gate:{gate_reason}")
                return BrainOutput(
                    response=response_text,
                    api_code=safe_api,
                    confidence=1.0,
                    reasoning=f"Hotpath + {gate_reason}"
                )

            # 热路径安全通过，直接返回
            elapsed = (time.time() - start_time) * 1000
            self.logger.info(f"✅ 热路径执行 ({elapsed:.0f}ms)")
            return BrainOutput(
                response="了解しました",
                api_code=safe_api,
                confidence=1.0,
                reasoning="Hotpath execution"
            )

        # 热路径未命中，记录日志
        self.logger.info(f"🔍 热路径未命中，使用LLM处理: {command}")

        # 0.5. 特殊命令处理 - 舞蹈随机选择（使用state_snapshot）
        dance_commands = ["dance", "ダンス", "跳舞", "舞蹈", "踊る", "踊って"]
        if command.lower() in dance_commands:
            # 随机选择Dance1或Dance2
            dance_choice = random.choice([1022, 1023])
            dance_name = "1" if dance_choice == 1022 else "2"

            # 使用状态快照（不再重复读取）
            # 安全验证（修复REVIEW漏洞：dance分支绕过安全栅格）
            api_code = dance_choice
            sequence = None
            if self.safety_validator:
                safety_result = self.safety_validator.validate_action(api_code, state_snapshot)
                if not safety_result.is_safe:
                    self.logger.warning(f"🛡️ 舞蹈动作安全拒绝: {safety_result.reason}")
                    return BrainOutput(
                        response=safety_result.reason,
                        api_code=None,
                        confidence=0.5
                    )
                # 如果需要自动补全姿态（如自动站立）
                if safety_result.modified_sequence:
                    self.logger.info(f"🔧 舞蹈自动补全: {safety_result.modified_sequence}")
                    sequence = safety_result.modified_sequence
                    if safety_result.should_use_sequence_only:
                        api_code = None

            elapsed = (time.time() - start_time) * 1000
            self.logger.info(f"🎲 随机选择舞蹈{dance_name} ({elapsed:.0f}ms)")
            return BrainOutput(
                response=f"踊ります{dance_name}",
                api_code=api_code,
                sequence=sequence
            )
        
        # 1. 检查热点缓存（使用state_snapshot）
        if command in self.hot_cache:
            cached = self.hot_cache[command]

            # 使用状态快照（不再重复读取）
            # 安全验证（修复REVIEW漏洞：hot_cache绕过安全栅格）
            api_code = cached.get("api_code")
            sequence = cached.get("sequence")

            if self.safety_validator and api_code:
                safety_result = self.safety_validator.validate_action(api_code, state_snapshot)
                if not safety_result.is_safe:
                    self.logger.warning(f"🛡️ 缓存命令安全拒绝: {safety_result.reason}")
                    return BrainOutput(
                        response=safety_result.reason,
                        api_code=None,
                        confidence=0.5
                    )
                # 如果需要自动补全姿态
                if safety_result.modified_sequence:
                    self.logger.info(f"🔧 缓存命令自动补全: {safety_result.modified_sequence}")
                    sequence = safety_result.modified_sequence
                    if safety_result.should_use_sequence_only:
                        api_code = None

            # 序列安全验证
            if self.safety_validator and sequence:
                seq_safety = self.safety_validator.validate_sequence(sequence, state_snapshot)
                if not seq_safety.is_safe:
                    self.logger.warning(f"🛡️ 缓存序列安全拒绝: {seq_safety.reason}")
                    return BrainOutput(
                        response=seq_safety.reason,
                        api_code=None,
                        confidence=0.5
                    )

            elapsed = (time.time() - start_time) * 1000
            self.logger.info(f"⚡ 缓存命中 ({elapsed:.0f}ms)")
            return BrainOutput(
                response=cached["response"],
                api_code=api_code,
                sequence=sequence
            )

        # 2. 判断复杂度并路由（修复REVIEW：支持灰度切流，使用state_snapshot）
        is_complex = self._is_complex_command(command)

        # 2.5. 灰度切流决策（可选）
        selected_3b = self.model_3b
        selected_7b = self.model_7b
        if self.ab_test_ratio > 0:
            import random
            if random.random() < self.ab_test_ratio:
                # 切换到Track B模型（假设intelligent模型为Track B）
                if "intelligent" not in selected_3b:
                    selected_3b = "claudia-intelligent-7b:v1"  # 暂时用7B代替3B
                    self.logger.debug(f"🔬 灰度切流→Track B: {selected_3b}")
                if "intelligent" not in selected_7b:
                    selected_7b = "claudia-intelligent-7b:v1"
                    self.logger.debug(f"🔬 灰度切流→Track B: {selected_7b}")

        if is_complex:
            self.logger.info("🔄 路由到7B模型（复杂指令）")
            # Track B关键修复：注入状态信息（使用state_snapshot）
            enhanced_cmd = self._build_enhanced_prompt(command, selected_7b, state_snapshot)
            result = await self._call_ollama_v2(selected_7b, enhanced_cmd, timeout=10)
            model_used = "7B"
        else:
            self.logger.info("⚡ 路由到3B模型（简单指令）")
            # Track B关键修复：注入状态信息（使用state_snapshot）
            enhanced_cmd = self._build_enhanced_prompt(command, selected_3b, state_snapshot)
            result = await self._call_ollama_v2(selected_3b, enhanced_cmd, timeout=5)
            model_used = "3B"

        # 3. 处理结果
        if result:
            elapsed = (time.time() - start_time) * 1000
            self.logger.info(f"✅ {model_used}模型响应 ({elapsed:.0f}ms)")

            # 提取字段 (支持完整字段名和缩写字段名)
            response = result.get("response") or result.get("r", "実行します")
            api_code = result.get("api_code") or result.get("a")
            sequence = result.get("sequence") or result.get("s")

            # SafetyValidator检查（使用state_snapshot）
            if self.safety_validator and api_code:
                safety_result = self.safety_validator.validate_action(api_code, state_snapshot)

                if not safety_result.is_safe:
                    self.logger.warning(f"🛡️ SafetyValidator拒绝: {safety_result.reason}")
                    return BrainOutput(
                        response=safety_result.reason,
                        api_code=None,
                        confidence=0.5
                    )

                # 如果安全校验建议修改序列（修复：避免双轨执行）
                if safety_result.modified_sequence:
                    self.logger.info(f"🔧 自动补全前置动作: {safety_result.modified_sequence}")
                    sequence = safety_result.modified_sequence
                    if safety_result.should_use_sequence_only:
                        api_code = None

            # 序列安全校验
            if self.safety_validator and sequence:
                seq_safety = self.safety_validator.validate_sequence(sequence, state_snapshot)
                if not seq_safety.is_safe:
                    self.logger.warning(f"🛡️ 序列安全拒绝: {seq_safety.reason}")
                    return BrainOutput(
                        response=seq_safety.reason,
                        api_code=None,
                        confidence=0.5
                    )

            # ===== 4) 最终安全门（代码层硬性收口，执行前最后检查） =====
            safe_api, gate_reason = self._final_safety_gate(api_code, state_snapshot)
            if safe_api != api_code:
                self.logger.warning(f"🚫 最终安全门: {gate_reason}")

                if safe_api is None:
                    # 被拒绝
                    return BrainOutput(
                        response=f"安全のため動作を停止しました ({state_snapshot.battery_level*100:.0f}%)" if state_snapshot else "安全のため動作を停止しました",
                        api_code=None,
                        confidence=1.0,
                        reasoning=gate_reason
                    )
                else:
                    # 被降级
                    api_code = safe_api
                    response = f"電池を節約するため、動作を調整します ({state_snapshot.battery_level*100:.0f}%)" if state_snapshot else "電池を節約するため、動作を調整します"

            return BrainOutput(
                response=response,
                api_code=api_code,
                sequence=sequence
            )
        
        # 4. 降级处理
        elapsed = (time.time() - start_time) * 1000
        self.logger.warning(f"⚠️ 模型无响应，使用默认 ({elapsed:.0f}ms)")
        return BrainOutput(
            response="すみません、理解できませんでした",
            api_code=None
        )
    
    async def execute_action(self, brain_output: BrainOutput) -> bool:
        """执行动作"""
        # 检查硬件模式和SportClient状态
        if self.use_real_hardware and self.sport_client:
            self.logger.info("🤖 使用真实硬件执行")
            return await self._execute_real(brain_output)
        else:
            if self.use_real_hardware:
                self.logger.warning("⚠️ 硬件模式但SportClient未初始化，使用模拟")
            return await self._execute_mock(brain_output)
    
    async def _execute_mock(self, brain_output: BrainOutput) -> bool:
        """模拟执行"""
        if brain_output.api_code:
            self.logger.info(f"🎭 [模拟] 执行API: {brain_output.api_code}")
            await asyncio.sleep(0.5)
            return True
        
        if brain_output.sequence:
            self.logger.info(f"🎭 [模拟] 执行序列: {brain_output.sequence}")
            for api in brain_output.sequence:
                self.logger.info(f"   → API: {api}")
                await asyncio.sleep(0.3)
            return True
        
        return False
    
    async def _execute_real(self, brain_output: BrainOutput) -> bool:
        """真实执行（带状态管理）"""
        try:
            # 处理序列动作
            if brain_output.sequence:
                self.logger.info(f"🤖 执行序列: {brain_output.sequence}")
                for api in brain_output.sequence:
                    brain_output_single = BrainOutput("", api)
                    success = await self._execute_real(brain_output_single)
                    if not success:
                        self.logger.warning(f"序列中API {api} 执行失败")
                    await asyncio.sleep(1)  # 动作间隔
                return True
            
            # 处理单个动作
            if brain_output.api_code:
                # 映射到SportClient方法
                method_map = {
                    # 基础动作（已验证100%成功）
                    1001: "Damp",
                    1002: "BalanceStand",
                    1003: "StopMove", 
                    1004: "StandUp",
                    1005: "StandDown",
                    1006: "RecoveryStand",
                    1009: "Sit",
                    1010: "RiseSit",      # 起坐
                    
                    # 表演动作（已验证100%成功）
                    1016: "Hello",
                    1017: "Stretch",
                    1036: "Heart",        # ✅ 真正的比心API！
                    1029: "Scrape",       # 刮擦
                    
                    # 高级动作（已验证100%成功）
                    1030: "FrontFlip",    # 前空翻
                    1031: "FrontJump",    # 前跳
                    1032: "FrontPounce",  # 前扑
                    
                    # 舞蹈动作（返回3104，成功码）
                    1022: "Dance1",
                    1023: "Dance2",
                    
                    # 其他已验证动作
                    1007: "Euler",         # 姿态控制(需要参数)
                    1008: "Move",          # 移动(需要参数) 
                    1015: "SpeedLevel",    # 速度等级(需要参数)
                    1019: "ContinuousGait",# 连续步态(需要参数)
                    1027: "SwitchJoystick",# 切换摇杆(需要参数)
                    1028: "Pose",          # 摆姿势(需要参数)
                }
                
                # 检查是否需要先站立
                if brain_output.api_code in self.actions_need_standing and self.robot_state != "standing":
                    self.logger.info(f"⚡ 动作需要站立状态，先执行站立...")
                    if hasattr(self.sport_client, "StandUp"):
                        stand_result = self.sport_client.StandUp()
                        self.logger.info(f"   站立返回码: {stand_result}")
                        if stand_result in [0, -1]:  # 0成功，-1已经站立
                            self.robot_state = "standing"
                            await asyncio.sleep(1.5)  # 等待站立完成
                        else:
                            self.logger.warning(f"   站立失败: {stand_result}")
                
                method_name = method_map.get(brain_output.api_code)
                
                # 特殊处理某些方法
                if method_name == "Dance2":
                    # Dance2直接调用
                    if hasattr(self.sport_client, "Dance2"):
                        self.logger.info(f"🤖 执行: Dance2 (API:{brain_output.api_code})")
                        result = self.sport_client.Dance2()
                    else:
                        self.logger.warning(f"未找到Dance2方法，跳过")
                        return False
                elif method_name == "Rollover":
                    # 尝试Rollover，如果没有则跳过
                    if hasattr(self.sport_client, "Rollover"):
                        self.logger.info(f"🤖 执行: Rollover (API:{brain_output.api_code})")
                        result = self.sport_client.Rollover()
                    else:
                        self.logger.warning(f"未找到Rollover方法，跳过")
                        return False
                elif method_name == "Handstand":
                    # 高级动作，可能需要特殊处理
                    if hasattr(self.sport_client, "Handstand"):
                        self.logger.info(f"🤖 执行: Handstand (API:{brain_output.api_code})")
                        result = self.sport_client.Handstand()
                    else:
                        self.logger.warning(f"未找到Handstand方法，跳过")
                        return False
                elif method_name and hasattr(self.sport_client, method_name):
                    method = getattr(self.sport_client, method_name)
                    self.logger.info(f"🤖 执行: {method_name} (API:{brain_output.api_code})")
                    
                    # 处理需要参数的方法
                    if brain_output.api_code in [1007, 1008, 1015, 1019, 1027, 1028]:
                        if brain_output.api_code == 1007:  # Euler
                            result = method(0, 0, 0)  # 默认姿态
                        elif brain_output.api_code == 1008:  # Move
                            result = method(0.2, 0, 0)  # 缓慢前进
                        elif brain_output.api_code == 1015:  # SpeedLevel
                            result = method(0)  # 默认速度
                        elif brain_output.api_code == 1019:  # ContinuousGait
                            result = method(1)  # 启用
                        elif brain_output.api_code == 1027:  # SwitchJoystick
                            result = method(True)  # 启用
                        elif brain_output.api_code == 1028:  # Pose
                            result = method(True)  # 启用摆姿势
                    else:
                        result = method()  # 无参数方法
                else:
                    self.logger.error(f"未找到API方法: {brain_output.api_code} - {method_name}")
                    return False
                
                self.logger.info(f"   返回码: {result}")
                
                # 更新状态
                if brain_output.api_code == 1004:  # StandUp
                    self.robot_state = "standing"
                elif brain_output.api_code == 1009:  # Sit
                    self.robot_state = "sitting"
                elif brain_output.api_code == 1005:  # StandDown
                    self.robot_state = "lying"
                
            # 判断执行结果（包含3104成功码）
            if result == 0:
                return True
            elif result == -1:  # 已经在该状态
                return True
            elif result == 3104:  # 舞蹈/触发等动作完成码
                self.logger.info(f"   ✅ 动作完成 (3104)")
                return True
            else:
                # 分析错误码
                if result == 3103:
                    self.logger.error(f"   ❌ APP占用中 (3103)")
                    self.logger.error("      请关闭APP并重启机器人")
                elif result == 3203:
                    self.logger.warning(f"   ⚠️ 动作不支持 (3203)")
                    self.logger.warning("      该API在Go2固件中未实现")
                else:
                    self.logger.warning(f"   ⚠️ 未知错误: {result}")
                return False
            
            # 如果没有API code也没有sequence，返回False
            return False
            
        except Exception as e:
            self.logger.error(f"执行错误: {e}")
            return False
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        return {
            "models": {
                "3b": self.model_3b,
                "7b": self.model_7b
            },
            "cache_size": len(self.hot_cache),
            "hardware_mode": self.use_real_hardware,
            "sport_client": self.sport_client is not None
        }


# 导出
__all__ = ['ProductionBrain', 'BrainOutput']
