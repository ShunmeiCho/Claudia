#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Commander - 生产环境交互式命令器
使用修复后的LLM大脑架构进行实机测试
"""

import asyncio
import json
import logging
import time
import sys
import os
from datetime import datetime
from typing import Optional

# 添加项目路径（相对于脚本位置，避免硬编码绝对路径）
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.append(_PROJECT_ROOT)
sys.path.append(os.path.join(_PROJECT_ROOT, 'src'))

from claudia.brain.production_brain import ProductionBrain, BrainOutput

# フェーズ表示の幅定数
_PHASE_WIDTH = 48


class _QuietFilter(logging.Filter):
    """起動フェーズ中にログを抑制 (ERROR 以上のみ通す)"""

    def filter(self, record):
        return record.levelno >= logging.ERROR


def _phase_start(step, total, label):
    """フェーズ開始行を表示 (改行なし)、開始時刻を返す"""
    prefix = "  [{}/{}] {}".format(step, total, label)
    dots = "." * (_PHASE_WIDTH - len(prefix) - 1)
    print("{} {}".format(prefix, dots), end="", flush=True)
    return time.time()


def _phase_ok(start_time):
    """フェーズ完了を追記"""
    elapsed = time.time() - start_time
    print(" OK ({:.1f}s)".format(elapsed))


def _phase_ok_detail(start_time, detail):
    """フェーズ完了 + 補足情報を追記"""
    elapsed = time.time() - start_time
    print(" OK ({:.1f}s)".format(elapsed))
    print("        {}".format(detail))


def _display_width(text):
    """テキストの端末表示幅を計算 (CJK 全角文字を 2 カラムとして扱う)"""
    w = 0
    for ch in text:
        cp = ord(ch)
        if (0x3000 <= cp <= 0x9FFF
                or 0xF900 <= cp <= 0xFAFF
                or 0xFF01 <= cp <= 0xFF60):
            w += 2
        else:
            w += 1
    return w


def _box_header(lines):
    """ボックス罫線ヘッダーを生成 (CJK 幅対応)"""
    inner = 44
    parts = []
    parts.append("  +{}+".format("-" * (inner + 2)))
    for line in lines:
        dw = _display_width(line)
        pad = inner - dw
        if pad < 0:
            pad = 0
        parts.append("  | {}{} |".format(line, " " * pad))
    parts.append("  +{}+".format("-" * (inner + 2)))
    return "\n".join(parts)


class ProductionCommander:
    """生产环境命令器"""

    def __init__(self, use_real_hardware: bool = False):
        """初始化命令器

        Args:
            use_real_hardware: 是否使用真实硬件（默认False为模拟模式）
        """
        self.brain = None  # defer init to run() for phased output
        self._use_real_hardware = use_real_hardware
        self.running = True
        self.command_history = []
        self.session_start = datetime.now()

    def print_header(self):
        """打印界面头部"""
        hw = "実機" if self._use_real_hardware else "sim"
        router_mode = ""
        model_info = ""
        if self.brain:
            router_mode = self.brain._router_mode.value
            if router_mode == "dual":
                model_info = "{} ({})".format(
                    self.brain._channel_router._action_model, router_mode)
            elif router_mode == "shadow":
                model_info = "{} ({})".format(self.brain.model_7b, router_mode)
            else:
                model_info = self.brain.model_7b
        ts = self.session_start.strftime("%Y-%m-%d %H:%M:%S")

        line2 = "{} / {}".format(hw, model_info) if model_info else hw
        print()
        print(_box_header([
            "Claudia Production Commander",
            line2,
            ts,
        ]))
        print()
        print("  日本語/中文/English OK, /help でヘルプ")
        print("  例: お手, 坐下, dance, 座ってから挨拶")
        print()

    def print_help(self):
        """打印帮助信息"""
        print()
        print("  " + "-" * 38)
        print("  基本命令:")
        print("    お手, おすわり, タッテ, ハート, ダンス")
        print("    坐下, 站立, 比心, 握手, 跳舞")
        print("    sit, stand, heart, dance, hello")
        print()
        print("  複合命令:")
        print("    座ってから挨拶 - 坐下然后打招呼")
        print("    運動して - 做运动")
        print("    表演一套 - 表演一套动作")
        print()
        print("  システム:")
        print("    /help    - ヘルプ表示")
        print("    /stats   - 統計情報")
        print("    /history - 履歴表示")
        print("    /clear   - 画面クリア")
        print("    /exit    - 終了")
        print("  " + "-" * 38)
        print()

    def print_stats(self):
        """打印统计信息"""
        stats = self.brain.get_statistics()
        print()
        print("  " + "-" * 38)
        print("  model:    {}".format(stats['model']))
        print("  cache:    {} entries".format(stats['cache_size']))
        print("  hardware: {}".format(
            "real" if stats['hardware_mode'] else "sim"))
        print("  client:   {}".format(
            "connected" if stats['sport_client'] else "none"))
        print("  commands: {}".format(len(self.command_history)))
        runtime = datetime.now() - self.session_start
        print("  uptime:   {:.0f}s".format(runtime.total_seconds()))
        print("  " + "-" * 38)
        print()

    def print_history(self):
        """打印历史记录"""
        print()
        print("  " + "-" * 38)
        if not self.command_history:
            print("  (履歴なし)")
        else:
            for i, (timestamp, cmd, response) in enumerate(self.command_history[-10:], 1):
                print("  {}. [{}] {}".format(i, timestamp, cmd))
                print("     -> {}".format(response))
        print("  " + "-" * 38)
        print()

    async def _warmup_model(self):
        """预热 LLM 模型 — 直接调用 Ollama API 将模型加载到 GPU 显存

        不走 process_command 管线（会命中 hot_cache 绕过 LLM）。
        发送一个极短的推理请求，触发 Ollama 将模型权重加载到显存。

        Dual/Shadow 模式时，同时预热 Action 模型（num_ctx=1024 匹配 _action_channel）。
        Jetson 16GB 统一内存，系统占 ~5.4GB，剩余 ~10GB 只够一个 ~5.6GB 模型常驻。
        预热顺序按模式优化，确保首条命令的主路径模型在显存中:
          - Legacy: 只预热 7B
          - Shadow: Action 先 → 7B 后（7B 是主路径，应驻留显存）
          - Dual:   7B 先 → Action 后（Action channel 先执行，应驻留显存）
        """
        model_name = self.brain.model_7b

        def _sync_warmup_http(model, num_ctx):
            from urllib.request import Request, urlopen
            payload = json.dumps({
                "model": model,
                "prompt": "hi",
                "stream": False,
                "options": {
                    "num_predict": 1,
                    "num_ctx": num_ctx,
                },
                "keep_alive": "30m",
            }).encode("utf-8")
            req = Request(
                "http://localhost:11434/api/generate",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urlopen(req, timeout=20) as resp:
                resp.read()

        sync_warmup_fn = _sync_warmup_http
        try:
            import ollama as _ollama

            def _sync_warmup_ollama(model, num_ctx):
                return _ollama.chat(
                    model=model,
                    messages=[{'role': 'user', 'content': 'hi'}],
                    format='json',
                    options={
                        'num_predict': 1,
                        'num_ctx': num_ctx,
                    },
                    keep_alive='30m',
                )

            sync_warmup_fn = _sync_warmup_ollama
        except ImportError:
            pass

        loop = asyncio.get_event_loop()
        router_mode = self.brain._router_mode.value

        warmup_sequence = []
        if router_mode == "shadow":
            action_model = self.brain._channel_router._action_model
            warmup_sequence = [
                (action_model, 1024, "Action", 30),
                (model_name, 2048, "7B", 60),
            ]
        elif router_mode == "dual":
            action_model = self.brain._channel_router._action_model
            warmup_sequence = [
                (action_model, 1024, "Action", 30),
            ]
        else:
            warmup_sequence = [
                (model_name, 2048, "7B", 60),
            ]

        for model, num_ctx, label, timeout_s in warmup_sequence:
            try:
                await asyncio.wait_for(
                    loop.run_in_executor(None, sync_warmup_fn, model, num_ctx),
                    timeout=timeout_s,
                )
            except asyncio.TimeoutError:
                pass  # phase display handles messaging
            except Exception:
                pass  # phase display handles messaging

    async def _wakeup_animation(self):
        """唤醒动画 — 机器人起立+伸懒腰

        直接调用 _rpc_call 绕过 pipeline（已知安全动作，无需 SafetyCompiler）。
        与 _warmup_model 并行执行，利用 LLM 加载等待时间。

        安全门控:
          - 仅在真实硬件模式且 SportClient 可用时执行
          - 需要 COMMANDER_WAKEUP_ANIMATION=1 显式启用（默认关闭）
          - 姿态跟踪仅在确认站立后更新（不做乐观写入）
        """
        if not self.brain.use_real_hardware or not self.brain.sport_client:
            return

        if os.environ.get("COMMANDER_WAKEUP_ANIMATION") != "1":
            return

        print("  wakeup: StandUp -> Stretch")
        wakeup_start = time.time()
        standup_code = None
        stretch_code = None
        standup_confirmed = False
        try:
            result = self.brain._rpc_call("StandUp")
            standup_code = result[0] if isinstance(result, tuple) else result
            if standup_code not in (0, -1, 3104):
                print("  [warn] standup failed (code={})".format(standup_code))
                return

            if standup_code in (0, -1):
                standup_confirmed = True
                self.brain._update_posture_tracking(1004)
                await asyncio.sleep(1.5)
            elif standup_code == 3104:
                await asyncio.sleep(2.0)
                standing_ok = await self.brain._verify_standing_after_unknown()
                if standing_ok:
                    standup_confirmed = True
                    self.brain._update_posture_tracking(1004)
                else:
                    print("  [warn] standup unconfirmed (3104)")
                    return

            result = self.brain._rpc_call("Stretch")
            stretch_code = result[0] if isinstance(result, tuple) else result
            if stretch_code in (0, -1, 3104):
                await asyncio.sleep(4.0)
            else:
                print("  [warn] stretch failed (code={})".format(stretch_code))

        except Exception as e:
            print("  [warn] wakeup error: {}".format(e))
        finally:
            self._log_wakeup_audit(
                standup_code, stretch_code, wakeup_start, standup_confirmed)

    def _log_wakeup_audit(self, standup_code, stretch_code, start_time,
                          standup_confirmed=False):
        """记录唤醒动画的审计条目"""
        try:
            from claudia.brain.audit_logger import AuditEntry, get_audit_logger
            from claudia.brain.audit_routes import ROUTE_STARTUP
            elapsed = (time.time() - start_time) * 1000
            stretch_ok = (stretch_code is None
                          or stretch_code in (0, -1, 3104))
            wakeup_success = standup_confirmed and stretch_ok
            if not standup_confirmed:
                verdict = "standup_failed"
                reason = "standup_code={}".format(standup_code)
            elif not stretch_ok:
                verdict = "stretch_failed"
                reason = "stretch_code={}".format(stretch_code)
            else:
                verdict = "ok"
                reason = None
            entry = AuditEntry(
                timestamp=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
                model_name="wakeup_animation",
                input_command="__wakeup__",
                state_battery=None,
                state_standing=None,
                state_emergency=False,
                llm_output=None,
                api_code=1004,
                sequence=[1004, 1017] if stretch_code is not None else [1004],
                safety_verdict=verdict,
                safety_reason=reason,
                elapsed_ms=elapsed,
                cache_hit=False,
                route=ROUTE_STARTUP,
                success=wakeup_success,
            )
            get_audit_logger().log_entry(entry)
        except Exception:
            pass

    async def process_command(self, command):
        """处理单个命令"""
        if command.startswith("/"):
            if command == "/help":
                self.print_help()
            elif command == "/stats":
                self.print_stats()
            elif command == "/history":
                self.print_history()
            elif command == "/clear":
                print("\033[2J\033[H", end="")  # ANSI clear screen
                self.print_header()
            elif command == "/exit":
                self.running = False
                print("\n  bye!\n")
            else:
                print("  [error] unknown: {}".format(command))
        else:
            print()
            print("  cmd> '{}'".format(command))
            print("  " + "-" * 38)

            start_time = time.time()
            brain_output = await self.brain.process_and_execute(command)
            process_time = (time.time() - start_time) * 1000

            print("  res> {}".format(brain_output.response))

            if brain_output.api_code:
                print("  api: {}".format(brain_output.api_code))

            if brain_output.sequence:
                print("  seq: {}".format(brain_output.sequence))

            print("  time: {:.0f}ms".format(process_time))

            if brain_output.api_code or brain_output.sequence:
                status = brain_output.execution_status
                if status == "success":
                    print("  [ok]")
                elif status == "unknown":
                    print("  [timeout] (実行中の可能性)")
                elif status == "failed":
                    print("  [failed]")

            timestamp = datetime.now().strftime("%H:%M:%S")
            self.command_history.append((
                timestamp,
                command,
                brain_output.response
            ))

            print("  " + "-" * 38)
            print()

    async def run(self):
        """运行主循环"""
        total = 2
        print()

        # 起動フェーズ中の INFO ログを抑制
        # Brain: logger 級 (handler は __init__ 内で作られる)
        # Root: handler 級 (propagate 経由の record を捕捉)
        quiet = _QuietFilter()
        brain_logger = logging.getLogger("ProductionBrain")
        brain_logger.addFilter(quiet)
        _suppressed_handlers = []
        for h in logging.getLogger().handlers:
            h.addFilter(quiet)
            _suppressed_handlers.append(h)
        logging.getLogger("httpx").setLevel(logging.WARNING)

        try:
            # 1. Brain 初期化
            t = _phase_start(1, total, "Brain")
            self.brain = ProductionBrain(use_real_hardware=self._use_real_hardware)

            hw = "実機" if self._use_real_hardware else "sim"
            mode = self.brain._router_mode.value
            detail = "{} / {}".format(hw, mode)
            try:
                if self.brain.state_monitor:
                    batt = self.brain.state_monitor.battery_level
                    if batt is not None:
                        detail += " / battery {}%".format(batt)
            except Exception:
                pass
            _phase_ok_detail(t, detail)

            # 2. LLM 予熱 + 唤醒動画
            t = _phase_start(2, total, "LLM warmup")
            await asyncio.gather(
                self._warmup_model(),
                self._wakeup_animation(),
            )
            _phase_ok(t)
        finally:
            brain_logger.removeFilter(quiet)
            for h in _suppressed_handlers:
                h.removeFilter(quiet)

        # ヘッダー表示 (Brain 情報が揃った状態で)
        self.print_header()

        # 主循环
        while self.running:
            try:
                command = input("  > ").strip()

                if command:
                    await self.process_command(command)

            except KeyboardInterrupt:
                print("\n\n  Ctrl+C, shutting down...")
                self.running = False
            except Exception as e:
                print("\n  [error] {}\n".format(e))

        runtime = datetime.now() - self.session_start
        print("  session: {} commands, {:.0f}s\n".format(
            len(self.command_history), runtime.total_seconds()))


async def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="Claudia Production Commander")
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="使用真实硬件模式（默认为模拟模式）"
    )

    args = parser.parse_args()

    commander = ProductionCommander(use_real_hardware=args.hardware)
    await commander.run()


if __name__ == "__main__":
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(main())
