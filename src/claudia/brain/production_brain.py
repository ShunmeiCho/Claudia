#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Brain Fixed - 修复SportClient初始化和提示词问题
"""

import copy
import json
import time
import asyncio
import logging
import subprocess
import random
import threading
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from enum import Enum
from functools import lru_cache

from claudia.brain.action_registry import (
    ACTION_REGISTRY, VALID_API_CODES, EXECUTABLE_API_CODES,
    REQUIRE_STANDING, HIGH_ENERGY_ACTIONS,
    METHOD_MAP, ACTION_RESPONSES, SAFE_DEFAULT_PARAMS,
    get_response_for_action, get_response_for_sequence,
)
from claudia.brain.safety_compiler import SafetyCompiler, SafetyVerdict
from claudia.brain.audit_routes import (
    ROUTE_EMERGENCY, ROUTE_HOTPATH, ROUTE_HOTPATH_REJECTED,
    ROUTE_SEQUENCE, ROUTE_DANCE, ROUTE_CONVERSATIONAL,
    ROUTE_PRECHECK_REJECTED, ROUTE_LLM_7B,
    ALL_ROUTES,
)

# 可选依赖导入
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
    api_code: Optional[int] = None  # 单个动作API
    sequence: Optional[List[int]] = None  # 动作序列
    confidence: float = 1.0
    reasoning: str = ""     # 推理过程/路由标记（用于审计和调试）
    success: bool = True    # 向后兼容（逐步废弃，用 execution_status 代替）
    execution_status: Optional[str] = None  # "success" | "unknown" | "failed" | None
    raw_decision: Optional[List[int]] = None  # Shadow 用: 安全编译前的原始 LLM 决策

    def to_dict(self):
        # type: () -> Dict
        """转换为字典"""
        result = {
            "response": self.response,
            "api_code": self.api_code,
            "success": self.success,
        }
        if self.sequence:
            result["sequence"] = self.sequence
        if self.reasoning:
            result["reasoning"] = self.reasoning
        if self.execution_status is not None:
            result["execution_status"] = self.execution_status
        return result

class ProductionBrain:
    """生产大脑 - 使用修复后的模型"""
    
    def __init__(self, use_real_hardware: bool = False):
        self.logger = self._setup_logger()
        self.use_real_hardware = use_real_hardware

        # 统一使用7B模型（支持环境变量切换）
        import os
        self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-go2-7b:v12.2-complete")  # 完整API支持（v12.2新增运动控制）

        self.logger.info(f"🧠 📌 7B模型: {self.model_7b}")
        
        # 精简动作缓存（仅保留文化特定词和LLM容易出错的核心命令）
        self.hot_cache = {
            # === 文化特定词（必须保留）===
            "ちんちん": {"response": "お辞儀します", "api_code": 1016},
            "ちんちんして": {"response": "お辞儀します", "api_code": 1016},
            "チンチン": {"response": "お辞儀します", "api_code": 1016},
            "拜年": {"response": "お辞儀します", "api_code": 1016},

            # === 多语言急停（安全关键）===
            "止まって": {"response": "止まります", "api_code": 1003},
            "止まれ": {"response": "止まります", "api_code": 1003},
            "停止": {"response": "止まります", "api_code": 1003},
            "停下": {"response": "止まります", "api_code": 1003},
            "stop": {"response": "止まります", "api_code": 1003},
            "halt": {"response": "止まります", "api_code": 1003},
            "ダンプ": {"response": "ダンプモード", "api_code": 1001},  # 紧急阻尼
            "damp": {"response": "ダンプモード", "api_code": 1001},
            "阻尼": {"response": "ダンプモード", "api_code": 1001},
            "バランス": {"response": "バランスします", "api_code": 1002},  # 紧急平衡
            "balance": {"response": "バランスします", "api_code": 1002},
            "平衡": {"response": "バランスします", "api_code": 1002},

            # === 核心基础命令 ===
            "座って": {"response": "座ります", "api_code": 1009},
            "おすわり": {"response": "お座りします", "api_code": 1009},
            "立って": {"response": "立ちます", "api_code": 1004},
            "タッテ": {"response": "立ちます", "api_code": 1004},
            "伏せる": {"response": "伏せます", "api_code": 1005},
            "横になる": {"response": "横になります", "api_code": 1005},

            # === 核心表演动作 ===
            "お手": {"response": "こんにちは", "api_code": 1016},
            "挨拶": {"response": "挨拶します", "api_code": 1016},
            "こんにちは": {"response": "こんにちは", "api_code": 1016},
            "hello": {"response": "挨拶します", "api_code": 1016},
            "ストレッチ": {"response": "伸びをします", "api_code": 1017},
            "伸び": {"response": "伸びをします", "api_code": 1017},
            "ダンス": {"response": "踊ります", "api_code": 1022},
            "踊って": {"response": "踊ります", "api_code": 1022},
            "ハート": {"response": "ハートします", "api_code": 1036},
            "比心": {"response": "ハートします", "api_code": 1036},

            # === 友好问候 → Hello(1016) ===
            "おはよう": {"response": "おはようございます！挨拶します", "api_code": 1016},
            "おはようございます": {"response": "おはようございます！挨拶します", "api_code": 1016},
            "こんばんは": {"response": "こんばんは！挨拶します", "api_code": 1016},
            "こんばんわ": {"response": "こんばんは！挨拶します", "api_code": 1016},
            "さようなら": {"response": "さようなら！またね。", "api_code": 1016},
            "おやすみ": {"response": "おやすみなさい！", "api_code": 1016},
            "おやすみなさい": {"response": "おやすみなさい！", "api_code": 1016},
            "good morning": {"response": "Good morning! 挨拶します", "api_code": 1016},
            "good evening": {"response": "Good evening! 挨拶します", "api_code": 1016},
            "good night": {"response": "Good night! 挨拶します", "api_code": 1016},
            "goodbye": {"response": "Goodbye! またね。", "api_code": 1016},
            "bye": {"response": "Goodbye! またね。", "api_code": 1016},
            "早上好": {"response": "早上好！挨拶します", "api_code": 1016},
            "晚上好": {"response": "晚上好！挨拶します", "api_code": 1016},
            "晚安": {"response": "晚安！", "api_code": 1016},
            "再见": {"response": "再见！またね。", "api_code": 1016},

            # === 褒め言葉 → Heart(1036) ===
            "かわいい": {"response": "ありがとう！ハートします", "api_code": 1036},
            "可愛い": {"response": "ありがとう！ハートします", "api_code": 1036},
            "すごい": {"response": "ありがとう！ハートします", "api_code": 1036},
            "凄い": {"response": "ありがとう！ハートします", "api_code": 1036},
            "いい子": {"response": "ありがとう！ハートします", "api_code": 1036},
            "可爱": {"response": "ありがとう！ハートします", "api_code": 1036},
            "cute": {"response": "ありがとう！ハートします", "api_code": 1036},

            # === 特例词（容易误解）===
            "お辞儀": {"response": "お辞儀します", "api_code": 1016},  # 鞠躬/拜年用Hello而非前空翻
            "礼": {"response": "お辞儀します", "api_code": 1016},
            "ジャンプ": {"response": "前跳します", "api_code": 1031},
            "ポーズ": {"response": "ポーズします", "api_code": 1028},
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
        # 站立前置列表已迁移至 action_registry.REQUIRE_STANDING，
        # SafetyCompiler 在 compile() 中自动处理。

        # 状态监控器
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

        # 安全验证器（旧，deprecated — 保留供其他模块引用）
        if SAFETY_VALIDATOR_AVAILABLE:
            self.safety_validator = get_safety_validator(enable_high_risk=False)
        else:
            self.safety_validator = None

        # 安全编译器（新，统一安全管线）
        allow_high_risk = os.getenv("SAFETY_ALLOW_HIGH_RISK", "0") == "1"
        self.safety_compiler = SafetyCompiler(allow_high_risk=allow_high_risk)
        if allow_high_risk:
            self.logger.warning("!! SAFETY_ALLOW_HIGH_RISK=1: 高风险动作已启用 !!")
        else:
            self.logger.info("SafetyCompiler 已加载（高风险动作已禁用）")

        # RPC 锁（SportClient 非线程安全，所有 RPC 调用必须通过 _rpc_call）
        self._rpc_lock = threading.RLock()
        self._current_timeout = 10.0  # 跟踪当前 SDK 超时值

        # 命令级串行锁（PR1 引入框架，PR2 强制迁移调用方）
        self._command_lock = asyncio.Lock()

        # 审计日志器
        if AUDIT_LOGGER_AVAILABLE:
            self.audit_logger = get_audit_logger()
            self.logger.info("✅ 审计日志器已启动 (logs/audit/)")
        else:
            self.audit_logger = None
            self.logger.warning("⚠️ 审计日志器不可用")

        # 姿态跟踪（用于模拟模式状态准确性）
        self.last_posture_standing = False  # 初始假设坐姿
        self.last_executed_api = None       # 最后执行的API代码

        self.logger.info("🧠 生产大脑初始化完成")
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
            
            # P0-5: 使用只读 API 测试连接（不再触发运动）
            try:
                try:
                    test_result, _ = self.sport_client.GetState(["mode"])
                except Exception:
                    # 固件兼容 fallback: 无参数 GetState
                    test_result, _ = self.sport_client.GetState([])
                
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
    
    def _rpc_call(self, method_name, *args, **kwargs):
        """统一 RPC 包装 — 所有 SportClient 调用必须通过此方法

        特性:
          - RLock 保证线程安全（支持同一线程嵌套调用）
          - 栈式超时保存/恢复（timeout_override 不污染全局状态）
          - 异常安全：即使 SetTimeout 失败也能恢复跟踪值

        Args:
            method_name: SportClient 方法名（如 "StandUp", "Dance1"）
            *args: 方法参数
            **kwargs: timeout_override=float 可临时覆盖超时
        """
        timeout_override = kwargs.pop("timeout_override", None)
        with self._rpc_lock:
            previous_timeout = self._current_timeout
            timeout_changed = False
            if timeout_override is not None:
                try:
                    self.sport_client.SetTimeout(timeout_override)
                    self._current_timeout = timeout_override
                    timeout_changed = True
                except Exception:
                    pass  # SetTimeout 失败则保持原超时
            try:
                method = getattr(self.sport_client, method_name)
                return method(*args)
            finally:
                if timeout_changed:
                    try:
                        self.sport_client.SetTimeout(previous_timeout)
                        self._current_timeout = previous_timeout
                    except Exception:
                        # SDK 恢复失败，至少保持跟踪值一致
                        self._current_timeout = previous_timeout

    # === 紧急停止关键词（检查在获取锁之前）===
    EMERGENCY_KEYWORDS = frozenset([
        "止まれ", "止めて", "停止", "stop", "halt", "emergency",
        "緊急停止", "やめて", "ストップ",
    ])

    async def process_and_execute(self, command):
        # type: (str) -> BrainOutput
        """原子化命令处理+执行入口（PR1 引入框架，PR2 迁移调用方）

        紧急指令绕过锁直接执行，普通指令在锁内串行处理。
        """
        cmd_lower = command.strip().lower()
        if cmd_lower in self.EMERGENCY_KEYWORDS:
            return await self._handle_emergency(command)

        async with self._command_lock:
            brain_output = await self.process_command(command)
            if brain_output.api_code or brain_output.sequence:
                result = await self.execute_action(brain_output)
                if result is True:
                    brain_output.execution_status = "success"
                elif result == "unknown":
                    brain_output.execution_status = "unknown"
                else:
                    brain_output.execution_status = "failed"
            return brain_output

    async def _handle_emergency(self, command):
        # type: (str) -> BrainOutput
        """紧急停止处理 — 不获取锁，直接调用 StopMove

        返回码语义:
          - sport_client 不存在（模拟模式）→ success（无需物理停止）
          - RPC 返回 0 或 -1（已停止）→ success
          - RPC 返回其他值 → failed
          - RPC 异常 → failed
        """
        self.logger.warning("!! 紧急停止: {} !!".format(command))
        exec_status = "success"  # 默认: 模拟模式无需物理停止
        response = "緊急停止しました"
        if self.sport_client:
            try:
                result = self._rpc_call("StopMove")
                if isinstance(result, tuple):
                    result = result[0]
                if result == 0 or result == -1:
                    exec_status = "success"
                else:
                    exec_status = "failed"
                    response = "緊急停止を試みましたが、エラーが発生しました（コード:{}）".format(result)
                    self.logger.error("紧急停止返回异常: {}".format(result))
            except Exception as e:
                exec_status = "failed"
                response = "緊急停止に失敗しました"
                self.logger.error("紧急停止 RPC 失败: {}".format(e))
        output = BrainOutput(
            response=response,
            api_code=1003,
            reasoning="emergency",
            execution_status=exec_status,
        )
        self._log_audit(
            command, output,
            route=ROUTE_EMERGENCY,
            elapsed_ms=0.0,
            cache_hit=False,
            model_used="bypass",
            current_state=None,
            llm_output=None,
            safety_verdict="bypass",
        )
        return output

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
                # 尝试创建模型（v12.2统一使用7B modelfile）
                if "v12" in model:
                    create_cmd = f"ollama create {model} -f models/ClaudiaIntelligent_7B_v2.0.modelfile"
                    subprocess.run(create_cmd, shell=True, capture_output=True)
                    self.logger.info(f"创建模型: {model}")
                else:
                    self.logger.warning(f"不支持的模型版本: {model}")
                    return None
            
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

    def _normalize_battery(self, level):
        # type: (Optional[float]) -> Optional[float]
        """电量透传（不做自动 /100 修正）

        SafetyCompiler.compile() 会对 >1.0 做 fail-safe 拒绝，
        迫使上游（state_monitor / _normalize_battery 调用方）修正数据源。
        自动 /100 会掩盖上游归一化 bug，因此移除。
        """
        if level is None:
            return None
        if level > 1.0:
            self.logger.error(
                "battery_level={} > 1.0，上游归一化异常！"
                "SafetyCompiler 将 fail-safe 拒绝所有动作".format(level)
            )
        return level

    def _sanitize_response(self, r: str) -> str:
        """
        清理LLM输出的response字段，防止无意义或非日语输出

        修复边缘案例问题：
        - "今日はいい天気ですね" → " godee" ❌
        - "ちんちん" → " pong" ❌

        Args:
            r: LLM输出的response字段

        Returns:
            清理后的response，如果无效则返回默认回复
        """
        if not r or not r.strip():
            return "すみません、よく分かりません"

        r = r.strip()

        # 检查是否包含日语字符（平假名、片假名、汉字）
        has_hiragana = any('\u3040' <= ch <= '\u309f' for ch in r)
        has_katakana = any('\u30a0' <= ch <= '\u30ff' for ch in r)
        has_kanji = any('\u4e00' <= ch <= '\u9faf' for ch in r)
        has_japanese = has_hiragana or has_katakana or has_kanji

        # 如果没有日语字符，返回默认回复
        if not has_japanese:
            self.logger.warning(f"⚠️ LLM输出无日语字符: '{r}' → 使用默认回复")
            return "すみません、よく分かりません"

        # 检查是否是无意义的单词（godee, pong等）
        nonsense_patterns = ['godee', 'pong', 'hi', 'hello', 'ok', 'yes', 'no']
        r_lower = r.lower()
        if any(pattern in r_lower for pattern in nonsense_patterns):
            self.logger.warning(f"⚠️ LLM输出包含无意义词: '{r}' → 使用默认回复")
            return "すみません、よく分かりません"

        return r

    def _quick_safety_precheck(self, command, state):
        # type: (str, Optional[Any]) -> Optional[str]
        """DEPRECATED in V2: 使用 SafetyCompiler.compile() 替代。
        保留代码供参考，不再被 process_command 调用。

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

    def _final_safety_gate(self, api_code, state):
        # type: (Optional[int], Optional[Any]) -> Tuple[Optional[int], str]
        """DEPRECATED in V2: 使用 SafetyCompiler.compile() 替代。
        保留代码供参考，不再被 process_command 调用。

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

    def _is_conversational_query(self, command: str) -> bool:
        """
        检测是否为对话型查询（不应返回动作API）

        Args:
            command: 用户命令

        Returns:
            True表示对话查询，False表示动作命令
        """
        cmd = command.strip().lower()

        # 对话型关键词模式
        CONVERSATIONAL_PATTERNS = [
            # 日语（褒め言葉は hot_cache へ移動: かわいい/すごい → Heart(1036)）
            'あなた', '君', 'きみ', '名前', 'なまえ', '誰', 'だれ',
            '何', 'なに', 'どう', 'なぜ', 'いつ', 'どこ',
            'ありがとう', 'ごめん',
            'おはよう', 'こんばんは', 'さようなら', 'おやすみ',
            # 英语 (cute moved to hot_cache → Heart)
            'who are you', 'what is your name', 'your name',
            'who', 'what', 'why', 'when', 'where', 'how',
            'you are', "you're", 'thank you', 'thanks', 'sorry',
            'good morning', 'good evening', 'good night', 'goodbye',
            'cool', 'awesome', 'nice',
            # 中文 (可爱 moved to hot_cache → Heart)
            '你是', '你叫', '你的名字', '谁', '什么', '为什么',
            '怎么', '哪里', '什么时候',
            '厉害', '谢谢', '对不起',
            '早上好', '晚上好', '晚安', '再见',
        ]

        # 检查是否包含对话关键词
        for pattern in CONVERSATIONAL_PATTERNS:
            if pattern in cmd:
                return True

        return False

    def _generate_conversational_response(self, command: str) -> str:
        """
        生成对话型回复（不执行动作）

        Args:
            command: 用户命令

        Returns:
            友好的对话回复
        """
        cmd = command.strip().lower()

        # 名字/身份相关
        if any(k in cmd for k in ['あなた', '誰', '名前', 'who', 'your name', '你是', '你叫']):
            return "私はClaudiaです。Unitree Go2のAIアシスタントです。"

        # 赞美相关
        if any(k in cmd for k in ['可愛い', 'かわいい', 'cute', '可爱']):
            return "ありがとうございます！"

        if any(k in cmd for k in ['すごい', '凄い', 'cool', 'awesome', '厉害']):
            return "ありがとうございます！頑張ります。"

        # 感谢相关
        if any(k in cmd for k in ['ありがとう', 'thank', '谢谢']):
            return "どういたしまして！"

        # 问候相关
        if any(k in cmd for k in ['おはよう', 'good morning', '早上好']):
            return "おはようございます！"

        if any(k in cmd for k in ['こんばんは', 'good evening', '晚上好']):
            return "こんばんは！"

        if any(k in cmd for k in ['おやすみ', 'good night', '晚安']):
            return "おやすみなさい！"

        if any(k in cmd for k in ['さようなら', 'goodbye', 'bye', '再见']):
            return "さようなら！またね。"

        # 默认对话回复
        return "はい、何でしょうか？"

    async def _call_ollama_v2(self, model: str, command: str, timeout: int = 10) -> Optional[Dict]:
        """
        调用Ollama LLM推理
        - 使用Python ollama库
        - loop.run_in_executor避免阻塞事件循环
        - 结构化JSON输出
        """
        if not OLLAMA_AVAILABLE:
            self.logger.warning("ollama库不可用，使用旧方法")
            return self._call_ollama(model, command, timeout)

        try:
            # P0-7: 移除每次推理时的 ollama.show() 开销
            # 模型存在性在启动时一次性验证（_call_ollama 降级路径保留 show）
            def _sync_ollama_call():
                # 生成参数优化（统一7B配置）
                num_predict = 100
                num_ctx = 2048

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

            # 使用run_in_executor避免阻塞（Python 3.8兼容）
            loop = asyncio.get_event_loop()
            result = await asyncio.wait_for(
                loop.run_in_executor(None, _sync_ollama_call),
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

    def _log_audit(self, command, output, route,
                   elapsed_ms, cache_hit, model_used,
                   current_state,
                   llm_output, safety_verdict,
                   safety_reason=None):
        # type: (str, BrainOutput, str, float, bool, str, Optional[Any], Optional[str], str, Optional[str]) -> None
        """记录完整审计日志（route 必须使用 audit_routes.py 常量）"""
        assert route in ALL_ROUTES, (
            "非法 route='{}'，必须使用 audit_routes.py 中的常量。"
            "合法值: {}".format(route, ALL_ROUTES)
        )
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
        snapshot_monotonic_ts = time.monotonic()  # SafetyCompiler 新鲜度校验用

        if state_snapshot:
            # 浅拷贝: 不修改 state_monitor 缓存的原始对象
            state_snapshot = copy.copy(state_snapshot)
            raw_batt = state_snapshot.battery_level
            state_snapshot.battery_level = self._normalize_battery(raw_batt)

            # P0-3: 仅在 ROS2 未真正初始化时使用内部姿态跟踪
            ros_initialized = (
                self.state_monitor
                and hasattr(self.state_monitor, 'is_ros_initialized')
                and self.state_monitor.is_ros_initialized
            )
            if not ros_initialized:
                state_snapshot.is_standing = self.last_posture_standing

            self.logger.info(
                "状态快照: 电池{:.0f}%, 姿态{}".format(
                    state_snapshot.battery_level * 100 if state_snapshot.battery_level else 0,
                    '站立' if state_snapshot.is_standing else '非站立'
                )
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
            # 记录审计日志
            self._log_audit(command, output, route=ROUTE_EMERGENCY, elapsed_ms=elapsed,
                          cache_hit=False, model_used="bypass",
                          current_state=None, llm_output=None,
                          safety_verdict="bypass")
            return output

        # ===== 2) 安全预检 — DEPRECATED (SafetyCompiler 统一处理) =====
        # _quick_safety_precheck 已被 SafetyCompiler 取代。
        # SafetyCompiler 在每条产出动作的路径上执行，覆盖了旧预检的所有场景。
        # 旧预检基于文本关键词，而 SafetyCompiler 基于 api_code，更精确。

        # ===== 3) 热点缓存检查 → SafetyCompiler 统一安全编译 =====
        # 三层归一化:
        #   1) strip() 精确匹配
        #   2) 去除末尾常见标点 (!！?？。．、,)
        #   3) lower() 降级匹配（英文/混合输入）
        cmd_stripped = command.strip()
        cmd_normalized = cmd_stripped.rstrip("!！?？。．、,")
        cmd_lower = cmd_normalized.lower()
        cached = (
            self.hot_cache.get(cmd_stripped)
            or self.hot_cache.get(cmd_normalized)
            or self.hot_cache.get(cmd_lower)
        )
        if cached:
            self.logger.info("热点缓存命中: {}".format(command))

            api_code = cached.get("api_code")
            sequence = cached.get("sequence")
            candidate = sequence if sequence else ([api_code] if api_code else [])

            if candidate:
                # fail-closed: state_snapshot=None → battery=0.0, is_standing=False
                # 只有 SAFE_ACTIONS 能通过（电量门控 ≤0.10 策略）
                _batt = state_snapshot.battery_level if state_snapshot else 0.0
                _stand = state_snapshot.is_standing if state_snapshot else False
                _ts = snapshot_monotonic_ts if state_snapshot else None
                if not state_snapshot:
                    self.logger.warning("状態監視なし: fail-safe安全コンパイル (battery=0.0)")
                verdict = self.safety_compiler.compile(
                    candidate, _batt, _stand, snapshot_timestamp=_ts,
                )
                if verdict.is_blocked:
                    self.logger.warning("热路径安全拒绝: {}".format(verdict.block_reason))
                    elapsed = (time.time() - start_time) * 1000
                    rejected_output = BrainOutput(
                        response=verdict.response_override or "安全のため動作を停止しました",
                        api_code=None, confidence=1.0,
                        reasoning="hotpath_safety_rejected", success=False,
                    )
                    self._log_audit(
                        command, rejected_output, route=ROUTE_HOTPATH_REJECTED,
                        elapsed_ms=elapsed, cache_hit=True, model_used="hotpath",
                        current_state=state_snapshot, llm_output=None,
                        safety_verdict="rejected:{}".format(verdict.block_reason),
                    )
                    return rejected_output

                # verdict.executable_sequence 已含自动 StandUp + 降级
                exec_seq = verdict.executable_sequence
                if len(exec_seq) == 1:
                    final_api = exec_seq[0]
                    final_sequence = None
                else:
                    final_api = None
                    final_sequence = exec_seq
            else:
                final_api = api_code
                final_sequence = sequence

            brain_output = BrainOutput(
                response=cached.get("response", "実行します"),
                api_code=final_api,
                sequence=final_sequence,
                confidence=1.0,
                reasoning="hotpath_executed",
                success=True,
            )

            elapsed = (time.time() - start_time) * 1000
            self.logger.info("热路径处理完成 ({:.0f}ms)".format(elapsed))
            self._log_audit(
                command, brain_output, route=ROUTE_HOTPATH,
                elapsed_ms=elapsed, cache_hit=True, model_used="hotpath",
                current_state=state_snapshot, llm_output=None, safety_verdict="ok",
            )
            return brain_output

        # 热路径未命中，记录日志
        self.logger.info(f"🔍 热路径未命中，检查序列预定义...")

        # ===== 3.3) 常见序列预定义（避免LLM调用） =====
        cmd_lower = command.strip().lower()
        SEQUENCE_HOTPATH = {
            # 站立+动作系列
            '立ってから挨拶': [1004, 1016],
            '立って挨拶': [1004, 1016],
            '立ってそして挨拶': [1004, 1016],
            '立ってこんにちは': [1004, 1016],
            '立ってからハート': [1004, 1036],
            '立ってハート': [1004, 1036],
            '立ってダンス': [1004, 1023],
            '立ってから踊る': [1004, 1023],

            # 坐下+动作系列
            '座ってから挨拶': [1009, 1016],
            '座って挨拶': [1009, 1016],
            '座ってこんにちは': [1009, 1016],

            # 英文
            'stand and hello': [1004, 1016],
            'stand then hello': [1004, 1016],
            'sit and hello': [1009, 1016],

            # 中文
            '站立然后问好': [1004, 1016],
            '坐下然后问好': [1009, 1016],
        }

        for key, seq in SEQUENCE_HOTPATH.items():
            if key in cmd_lower:
                self.logger.info("序列预定义命中: {} -> {}".format(key, seq))

                # P0-9: 序列路径必须走 SafetyCompiler（旧版无安全检查）
                # fail-closed: state_snapshot=None → battery=0.0, is_standing=False
                _batt = state_snapshot.battery_level if state_snapshot else 0.0
                _stand = state_snapshot.is_standing if state_snapshot else False
                _ts = snapshot_monotonic_ts if state_snapshot else None
                if not state_snapshot:
                    self.logger.warning("状態監視なし: fail-safe安全コンパイル (battery=0.0)")
                verdict = self.safety_compiler.compile(
                    seq, _batt, _stand, snapshot_timestamp=_ts,
                )
                if verdict.is_blocked:
                    self.logger.warning("序列安全拒绝: {}".format(verdict.block_reason))
                    elapsed = (time.time() - start_time) * 1000
                    rejected_output = BrainOutput(
                        response=verdict.response_override or "安全のため動作を停止しました",
                        api_code=None, reasoning="sequence_safety_rejected",
                    )
                    self._log_audit(
                        command, rejected_output, route=ROUTE_SEQUENCE,
                        elapsed_ms=elapsed, cache_hit=False, model_used="sequence_hotpath",
                        current_state=state_snapshot, llm_output=None,
                        safety_verdict="rejected:{}".format(verdict.block_reason),
                    )
                    return rejected_output
                exec_seq = verdict.executable_sequence

                seq_output = BrainOutput(
                    response=get_response_for_sequence(exec_seq),
                    sequence=exec_seq,
                    confidence=1.0,
                    reasoning="sequence_predefined",
                    success=True,
                )

                elapsed = (time.time() - start_time) * 1000
                self._log_audit(
                    command, seq_output, route=ROUTE_SEQUENCE,
                    elapsed_ms=elapsed, cache_hit=False, model_used="sequence_hotpath",
                    current_state=state_snapshot, llm_output=None, safety_verdict="ok",
                )
                return seq_output

        self.logger.info("序列预定义未命中，检查对话查询...")

        # ===== 3.5) 对话查询检测（避免LLM将对话误解为动作） =====
        if self._is_conversational_query(command):
            conversational_response = self._generate_conversational_response(command)
            elapsed = (time.time() - start_time) * 1000
            self.logger.info(f"💬 对话查询识别 ({elapsed:.0f}ms)")

            dialog_output = BrainOutput(
                response=conversational_response,
                api_code=None,  # 对话不执行动作
                sequence=None,
                confidence=1.0,
                reasoning="conversational_query",
                success=True
            )

            # 审计日志
            self._log_audit(command, dialog_output,
                          route=ROUTE_CONVERSATIONAL, elapsed_ms=elapsed, cache_hit=False,
                          model_used="dialog_detector", current_state=state_snapshot,
                          llm_output=None, safety_verdict="dialog")

            return dialog_output

        # 0.5. 特殊命令处理 - 舞蹈随机选择 → SafetyCompiler
        dance_commands = ["dance", "ダンス", "跳舞", "舞蹈", "踊る", "踊って"]
        if command.lower() in dance_commands:
            dance_choice = random.choice([1022, 1023])
            dance_name = "1" if dance_choice == 1022 else "2"

            # fail-closed: state_snapshot=None → battery=0.0, is_standing=False
            _batt = state_snapshot.battery_level if state_snapshot else 0.0
            _stand = state_snapshot.is_standing if state_snapshot else False
            _ts = snapshot_monotonic_ts if state_snapshot else None
            if not state_snapshot:
                self.logger.warning("状態監視なし: fail-safe安全コンパイル (battery=0.0)")
            verdict = self.safety_compiler.compile(
                [dance_choice], _batt, _stand, snapshot_timestamp=_ts,
            )
            if verdict.is_blocked:
                self.logger.warning("舞蹈安全拒绝: {}".format(verdict.block_reason))
                elapsed = (time.time() - start_time) * 1000
                rejected_output = BrainOutput(
                    response=verdict.response_override or "安全のため動作を停止しました",
                    api_code=None, reasoning="dance_safety_rejected",
                )
                self._log_audit(
                    command, rejected_output, route=ROUTE_DANCE,
                    elapsed_ms=elapsed, cache_hit=False, model_used="dance_random",
                    current_state=state_snapshot, llm_output=None,
                    safety_verdict="rejected:{}".format(verdict.block_reason),
                )
                return rejected_output

            exec_seq = verdict.executable_sequence
            if len(exec_seq) == 1:
                final_api = exec_seq[0]
                final_sequence = None
            else:
                final_api = None
                final_sequence = exec_seq

            elapsed = (time.time() - start_time) * 1000
            self.logger.info("随机选择舞蹈{} ({:.0f}ms)".format(dance_name, elapsed))
            dance_output = BrainOutput(
                response="踊ります{}".format(dance_name),
                api_code=final_api,
                sequence=final_sequence,
            )
            self._log_audit(
                command, dance_output, route=ROUTE_DANCE,
                elapsed_ms=elapsed, cache_hit=False, model_used="dance_random",
                current_state=state_snapshot, llm_output=None, safety_verdict="ok",
            )
            return dance_output

        # 2. 统一使用7B模型推理 → SafetyCompiler 统一安全编译
        self.logger.info("使用7B模型推理...")
        result = await self._call_ollama_v2(
            self.model_7b,
            command,
            timeout=25,
        )

        if result:
            elapsed = (time.time() - start_time) * 1000
            self.logger.info("7B模型响应 ({:.0f}ms)".format(elapsed))

            # 提取字段 (支持完整字段名和缩写字段名)
            raw_response = result.get("response") or result.get("r", "実行します")
            response = self._sanitize_response(raw_response)
            api_code = result.get("api_code") or result.get("a")
            sequence = result.get("sequence") or result.get("s")

            # 解析层白名单过滤（VALID_API_CODES: 无参数 + 已启用）
            # 非法 api_code 视为纯对话（不进 SafetyCompiler 以免误阻）
            if api_code is not None and api_code not in VALID_API_CODES:
                self.logger.warning("LLM 输出非法 api_code={}，降级为纯文本".format(api_code))
                api_code = None
            if sequence:
                valid_seq = [c for c in sequence if c in VALID_API_CODES]
                if len(valid_seq) != len(sequence):
                    dropped = [c for c in sequence if c not in VALID_API_CODES]
                    self.logger.warning("LLM 序列含非法码 {}，过滤后: {}".format(dropped, valid_seq))
                    sequence = valid_seq if valid_seq else None

            # 构建候选动作列表
            candidate = sequence if sequence else ([api_code] if api_code else [])

            if candidate:
                # fail-closed: state_snapshot=None → battery=0.0, is_standing=False
                _batt = state_snapshot.battery_level if state_snapshot else 0.0
                _stand = state_snapshot.is_standing if state_snapshot else False
                _ts = snapshot_monotonic_ts if state_snapshot else None
                if not state_snapshot:
                    self.logger.warning("状態監視なし: fail-safe安全コンパイル (battery=0.0)")
                verdict = self.safety_compiler.compile(
                    candidate, _batt, _stand, snapshot_timestamp=_ts,
                )
                if verdict.is_blocked:
                    self.logger.warning("LLM 路径安全拒绝: {}".format(verdict.block_reason))
                    rejected_output = BrainOutput(
                        response=verdict.response_override or "安全のため動作を停止しました",
                        api_code=None, confidence=1.0,
                        reasoning="llm_safety_rejected",
                    )
                    self._log_audit(
                        command, rejected_output, route=ROUTE_LLM_7B,
                        elapsed_ms=elapsed, cache_hit=False, model_used="7B",
                        current_state=state_snapshot,
                        llm_output=str(result)[:200],
                        safety_verdict="rejected:{}".format(verdict.block_reason),
                    )
                    return rejected_output

                exec_seq = verdict.executable_sequence
                if len(exec_seq) == 1:
                    final_api = exec_seq[0]
                    final_sequence = None
                else:
                    final_api = None
                    final_sequence = exec_seq

                # 降级时替换响应
                if verdict.warnings:
                    for w in verdict.warnings:
                        self.logger.info("SafetyCompiler: {}".format(w))
            else:
                final_api = api_code
                final_sequence = sequence

            llm_output = BrainOutput(
                response=response,
                api_code=final_api,
                sequence=final_sequence,
            )
            self._log_audit(
                command, llm_output, route=ROUTE_LLM_7B,
                elapsed_ms=elapsed, cache_hit=False, model_used="7B",
                current_state=state_snapshot,
                llm_output=str(result)[:200],
                safety_verdict="ok",
            )
            return llm_output

        # 4. 降级处理
        elapsed = (time.time() - start_time) * 1000
        self.logger.warning("模型无响应，使用默认 ({:.0f}ms)".format(elapsed))
        return BrainOutput(
            response="すみません、理解できませんでした",
            api_code=None,
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
    
    async def _execute_real(self, brain_output):
        # type: (BrainOutput) -> Any
        """真实执行（使用 _rpc_call + registry METHOD_MAP）

        Returns:
            True/"success" — 成功
            "unknown" — 3104 超时但机器人可达（动作可能仍在执行）
            False/"failed" — 失败
        """
        try:
            # P0-8: 序列中间失败则中止（不再静默继续）
            if brain_output.sequence:
                self.logger.info("执行序列: {}".format(brain_output.sequence))
                for i, api in enumerate(brain_output.sequence):
                    single = BrainOutput("", api)
                    success = await self._execute_real(single)
                    if not success and success != "unknown":
                        self.logger.error(
                            "序列中止: API {} (第{}/{}) 执行失败".format(
                                api, i + 1, len(brain_output.sequence)
                            )
                        )
                        return False
                    await asyncio.sleep(1)
                return True

            if not brain_output.api_code:
                return False

            # 从 registry 查询方法名（替代内联 method_map）
            method_name = METHOD_MAP.get(brain_output.api_code)
            if not method_name:
                self.logger.error("未注册的 API: {}".format(brain_output.api_code))
                return False

            # SafetyCompiler 已处理站立前置（自动前插 StandUp），
            # 此处不再重复检查 actions_need_standing。

            # 使用 _rpc_call 统一调用（线程安全 + 超时管理）
            self.logger.info("执行: {} (API:{})".format(method_name, brain_output.api_code))

            # 参数化动作使用 SAFE_DEFAULT_PARAMS
            if brain_output.api_code in SAFE_DEFAULT_PARAMS:
                params = SAFE_DEFAULT_PARAMS[brain_output.api_code]
                result = self._rpc_call(method_name, *params)
            else:
                result = self._rpc_call(method_name)

            # 处理元组返回值（如 GetState 返回 (code, data)）
            if isinstance(result, tuple):
                result = result[0]

            self.logger.info("   返回码: {}".format(result))

            # 更新姿态跟踪
            if brain_output.api_code == 1004:  # StandUp
                self.robot_state = "standing"
                self.last_posture_standing = True
            elif brain_output.api_code == 1009:  # Sit
                self.robot_state = "sitting"
                self.last_posture_standing = False
            elif brain_output.api_code == 1005:  # StandDown
                self.robot_state = "lying"
                self.last_posture_standing = False

            self.last_executed_api = brain_output.api_code

            # P0-1: 修复 3104 误判（超时 != 成功）
            if result == 0:
                return True
            elif result == -1:  # 已处于目标状态
                return True
            elif result == 3104:  # RPC_ERR_CLIENT_API_TIMEOUT
                self.logger.warning("   动作响应超时 (3104)")
                try:
                    state_code, _ = self._rpc_call(
                        "GetState", ["mode"], timeout_override=3.0
                    )
                    if state_code == 0:
                        self.logger.warning("   连通性确认OK，动作可能仍在执行")
                        return "unknown"
                    else:
                        self.logger.error("   连通性异常 ({})".format(state_code))
                        return False
                except Exception as e:
                    self.logger.error("   连通性确认失败: {}".format(e))
                    return False
            else:
                # P0-2: 修复 3103 注释和日志
                if result == 3103:
                    self.logger.error("   控制冲突 (3103): APP可能占用sport_mode")
                    self.logger.error("      请关闭APP并重启机器人，或检查Init()是否成功")
                elif result == 3203:
                    self.logger.warning("   动作不支持 (3203): 该API在Go2固件中未实现")
                else:
                    self.logger.warning("   未知错误: {}".format(result))
                return False

        except Exception as e:
            self.logger.error("执行错误: {}".format(e))
            return False
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        return {
            "model": self.model_7b,
            "cache_size": len(self.hot_cache),
            "hardware_mode": self.use_real_hardware,
            "sport_client": self.sport_client is not None
        }


# 导出
__all__ = ['ProductionBrain', 'BrainOutput']
