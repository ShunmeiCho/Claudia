#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
审计日志系统 - Track A增强
用于A/B测试决策、灰度观测和故障回滚
"""

import json
import logging
import threading
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict


@dataclass
class AuditEntry:
    """审计日志条目"""
    timestamp: str              # ISO格式时间戳
    model_name: str             # 使用的模型名称
    input_command: str          # 用户输入
    state_battery: Optional[float]  # 电量百分比
    state_standing: Optional[bool]  # 是否站立
    state_emergency: Optional[bool] # 是否紧急状态
    llm_output: Optional[str]   # LLM原始输出
    api_code: Optional[int]     # 最终API码
    sequence: Optional[list]    # 动作序列
    safety_verdict: str         # 安全裁决: ok/rejected/modified
    safety_reason: Optional[str]  # 拒绝/修改原因
    elapsed_ms: float           # 耗时（毫秒）
    cache_hit: bool             # 是否缓存命中
    route: str                  # 路由: emergency/cache/3B/7B
    success: bool               # 是否成功

    # PR2 扩展字段（全部 Optional，默认 None — 旧日志反序列化兼容）
    request_id: Optional[str] = None
    router_mode: Optional[str] = None       # "legacy" | "dual" | "shadow"
    shadow_comparison: Optional[Dict[str, Any]] = None
    action_latency_ms: Optional[float] = None
    voice_latency_ms: Optional[float] = None


class AuditLogger:
    """审计日志管理器"""

    def __init__(self, log_dir: str = "logs/audit", max_size_mb: int = 100):
        """
        初始化审计日志

        Args:
            log_dir: 日志目录
            max_size_mb: 单个日志文件最大大小（MB）
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.max_size_bytes = max_size_mb * 1024 * 1024

        # 获取当前日志文件
        self.current_log_file = self._get_current_log_file()

        # 写入锁 (run_in_executor 等多线程场景下保护 JSONL 行完整性)
        self._write_lock = threading.Lock()

        # 内部日志器
        self.logger = logging.getLogger("AuditLogger")

    def _get_current_log_file(self) -> Path:
        """获取当前日志文件路径（按日期轮转）"""
        date_str = datetime.now().strftime("%Y%m%d")
        log_file = self.log_dir / f"audit_{date_str}.jsonl"
        return log_file

    def log_entry(self, entry: AuditEntry) -> bool:
        """
        记录审计条目（追加到JSONL文件）

        Args:
            entry: 审计条目

        Returns:
            True 写入成功，False 写入失败
        """
        try:
            with self._write_lock:
                # 检查日志文件是否需要轮转
                current_file = self._get_current_log_file()
                if current_file != self.current_log_file:
                    self.logger.info(f"日志文件轮转: {self.current_log_file} -> {current_file}")
                    self.current_log_file = current_file

                # 检查文件大小
                if self.current_log_file.exists():
                    if self.current_log_file.stat().st_size > self.max_size_bytes:
                        # 归档当前文件
                        archive_name = self.current_log_file.with_suffix(
                            f".{datetime.now().strftime('%H%M%S')}.jsonl"
                        )
                        self.current_log_file.rename(archive_name)
                        self.logger.warning(f"日志文件过大，归档为: {archive_name}")

                # 追加JSONL条目
                with self.current_log_file.open('a', encoding='utf-8') as f:
                    json_line = json.dumps(asdict(entry), ensure_ascii=False)
                    f.write(json_line + '\n')
            return True

        except Exception as e:
            self.logger.error(f"审计日志写入失败: {e}")
            return False

    def get_recent_entries(self, limit: int = 100) -> List[AuditEntry]:
        """
        读取最近的审计条目

        Args:
            limit: 最多返回条目数

        Returns:
            审计条目列表
        """
        entries = []
        try:
            if not self.current_log_file.exists():
                return entries

            with self.current_log_file.open('r', encoding='utf-8') as f:
                lines = f.readlines()
                for line in lines[-limit:]:
                    data = json.loads(line.strip())
                    entries.append(AuditEntry(**data))
        except Exception as e:
            self.logger.error(f"读取审计日志失败: {e}")

        return entries

    def get_stats(self, model_name: Optional[str] = None,
                  hours: int = 24) -> Dict[str, Any]:
        """
        获取统计指标（用于A/B决策）

        Args:
            model_name: 模型名称（None=全部）
            hours: 统计时间窗口（小时）

        Returns:
            统计字典: {
                "total": 总请求数,
                "success_rate": 成功率,
                "avg_latency_ms": 平均延迟,
                "p95_latency_ms": P95延迟,
                "safety_reject_rate": 安全拒绝率,
                "cache_hit_rate": 缓存命中率,
                ...
            }
        """
        entries = self.get_recent_entries(limit=10000)

        # 时间过滤
        cutoff = datetime.now().timestamp() - hours * 3600
        filtered = [
            e for e in entries
            if datetime.fromisoformat(e.timestamp).timestamp() > cutoff
        ]

        # 模型过滤
        if model_name:
            filtered = [e for e in filtered if e.model_name == model_name]

        if not filtered:
            return {"total": 0}

        # 计算统计
        total = len(filtered)
        success = sum(1 for e in filtered if e.success)
        rejected = sum(1 for e in filtered if e.safety_verdict == "rejected")
        cache_hits = sum(1 for e in filtered if e.cache_hit)
        latencies = sorted([e.elapsed_ms for e in filtered])

        return {
            "total": total,
            "success_rate": success / total if total > 0 else 0,
            "avg_latency_ms": sum(latencies) / len(latencies) if latencies else 0,
            "p50_latency_ms": latencies[len(latencies)//2] if latencies else 0,
            "p95_latency_ms": latencies[int(len(latencies)*0.95)] if latencies else 0,
            "p99_latency_ms": latencies[int(len(latencies)*0.99)] if latencies else 0,
            "safety_reject_rate": rejected / total if total > 0 else 0,
            "cache_hit_rate": cache_hits / total if total > 0 else 0,
        }


# 全局单例（线程安全）
_global_audit_logger: Optional[AuditLogger] = None
_audit_logger_lock = threading.Lock()


def get_audit_logger(log_dir: str = "logs/audit") -> AuditLogger:
    """获取全局审计日志器（线程安全: run_in_executor 场景）"""
    global _global_audit_logger
    if _global_audit_logger is None:
        with _audit_logger_lock:
            if _global_audit_logger is None:
                _global_audit_logger = AuditLogger(log_dir=log_dir)
    return _global_audit_logger
