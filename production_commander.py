#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Commander - 生产环境交互式命令器
使用修复后的LLM大脑架构进行实机测试
"""

import asyncio
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


class ProductionCommander:
    """生产环境命令器"""
    
    def __init__(self, use_real_hardware: bool = False):
        """初始化命令器
        
        Args:
            use_real_hardware: 是否使用真实硬件（默认False为模拟模式）
        """
        self.brain = ProductionBrain(use_real_hardware=use_real_hardware)
        self.running = True
        self.command_history = []
        self.session_start = datetime.now()
        
    def print_header(self):
        """打印界面头部"""
        print("\n" + "="*60)
        print("🤖 Claudia Production Commander - LLM大脑实机测试")
        print("="*60)
        print(f"⚙️  模式: {'真实硬件' if self.brain.use_real_hardware else '模拟执行'}")
        print(f"🧠 模型: {self.brain.model_7b}")
        print(f"⏰ 会话开始: {self.session_start.strftime('%Y-%m-%d %H:%M:%S')}")
        print("-"*60)
        print("💡 提示: 输入日语/中文/英文命令，输入 /help 查看帮助")
        print("💡 示例: お手, 坐下, dance, 座ってから挨拶")
        print("-"*60 + "\n")
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "="*40)
        print("📖 帮助信息")
        print("="*40)
        print("\n基本命令:")
        print("  お手, おすわり, タッテ, ハート, ダンス")
        print("  坐下, 站立, 比心, 握手, 跳舞")
        print("  sit, stand, heart, dance, hello")
        print("\n复杂命令:")
        print("  座ってから挨拶 - 坐下然后打招呼")
        print("  運動して - 做运动")
        print("  表演一套 - 表演一套动作")
        print("\n系统命令:")
        print("  /help    - 显示帮助")
        print("  /stats   - 显示统计")
        print("  /history - 显示历史")
        print("  /clear   - 清屏")
        print("  /exit    - 退出")
        print("="*40 + "\n")
    
    def print_stats(self):
        """打印统计信息"""
        stats = self.brain.get_statistics()
        print("\n" + "="*40)
        print("📊 统计信息")
        print("="*40)
        print(f"🧠 模型: {stats['model']}")
        print(f"⚡ 缓存大小: {stats['cache_size']} 条")
        print(f"🤖 硬件模式: {'真实' if stats['hardware_mode'] else '模拟'}")
        print(f"🔌 SportClient: {'已连接' if stats['sport_client'] else '未连接'}")
        print(f"📝 历史命令: {len(self.command_history)} 条")
        runtime = datetime.now() - self.session_start
        print(f"⏱️ 运行时间: {runtime.total_seconds():.0f} 秒")
        print("="*40 + "\n")
    
    def print_history(self):
        """打印历史记录"""
        print("\n" + "="*40)
        print("📜 命令历史")
        print("="*40)
        if not self.command_history:
            print("(暂无历史记录)")
        else:
            for i, (timestamp, cmd, response) in enumerate(self.command_history[-10:], 1):
                print(f"{i}. [{timestamp}] {cmd}")
                print(f"   → {response}")
        print("="*40 + "\n")
    
    async def _warmup_model(self):
        """预热 LLM 模型 — 直接调用 Ollama API 将模型加载到 GPU 显存

        不走 process_command 管线（会命中 hot_cache 绕过 LLM）。
        发送一个极短的推理请求，触发 Ollama 将模型权重加载到显存。

        Dual/Shadow 模式时，同时预热 Action 模型（num_ctx=1024 匹配 _action_channel）。
        Jetson 8GB VRAM 只能容纳一个 ~4.7GB 模型，后加载的模型驻留显存。
        预热顺序按模式优化，确保首条命令的主路径模型在显存中:
          - Legacy: 只预热 7B
          - Shadow: Action 先 → 7B 后（7B 是主路径，应驻留显存）
          - Dual:   7B 先 → Action 后（Action channel 先执行，应驻留显存）
        """
        print("🔄 预热模型中...")
        model_name = self.brain.model_7b

        def _sync_warmup_http(model, num_ctx):
            # 标准库兜底: 不依赖 Python ollama 包，直接走本地 Ollama HTTP API
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

        use_ollama_py = False
        sync_warmup_fn = _sync_warmup_http
        try:
            import ollama as _ollama
            use_ollama_py = True

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
            print("ℹ️ 未检测到Python ollama包，使用HTTP API预热")

        loop = asyncio.get_event_loop()
        router_mode = self.brain._router_mode.value

        # 构建预热序列: (model, num_ctx, label, timeout_s)
        # 最后预热的模型驻留 VRAM，应为该模式首条命令的主路径模型
        # Action 模型 num_predict=30 推理轻量，30s 超时足够
        warmup_sequence = []
        if router_mode == "shadow":
            # Shadow: legacy(7B) 是主路径 → 7B 最后预热
            action_model = self.brain._channel_router._action_model
            warmup_sequence = [
                (action_model, 1024, "Action", 30),
                (model_name, 2048, "7B", 60),
            ]
        elif router_mode == "dual":
            # Dual: action channel 先执行 → Action 最后预热
            action_model = self.brain._channel_router._action_model
            warmup_sequence = [
                (model_name, 2048, "7B", 60),
                (action_model, 1024, "Action", 30),
            ]
        else:
            # Legacy: 只有 7B
            warmup_sequence = [
                (model_name, 2048, "7B", 60),
            ]

        for model, num_ctx, label, timeout_s in warmup_sequence:
            try:
                start = time.time()
                await asyncio.wait_for(
                    loop.run_in_executor(None, sync_warmup_fn, model, num_ctx),
                    timeout=timeout_s,
                )
                elapsed = (time.time() - start) * 1000
                if use_ollama_py:
                    print("✅ {} 模型就绪 ({}: {:.0f}ms)".format(
                        label, model, elapsed))
                else:
                    print("✅ {} 模型就绪[HTTP] ({}: {:.0f}ms)".format(
                        label, model, elapsed))
            except asyncio.TimeoutError:
                print("⚠️ {} 模型预热超时 ({}s)，继续".format(
                    label, timeout_s))
            except Exception as e:
                print("⚠️ {} 模型预热失败: {}，继续".format(label, e))

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

        print("🐕 唤醒动画: 起立 → 伸懒腰")
        wakeup_start = time.time()
        standup_code = None
        stretch_code = None
        standup_confirmed = False  # 3104 后验确认结果（审计 success 语义用）
        try:
            # StandUp(1004)
            result = self.brain._rpc_call("StandUp")
            standup_code = result[0] if isinstance(result, tuple) else result
            if standup_code not in (0, -1, 3104):
                print("⚠️ 起立失败 (code={}), 跳过伸懒腰".format(standup_code))
                return

            if standup_code in (0, -1):
                standup_confirmed = True
                self.brain._update_posture_tracking(1004)
                await asyncio.sleep(1.5)
            elif standup_code == 3104:
                # 3104: 通过 GetState 短轮询确认站立，不做乐观写入
                await asyncio.sleep(2.0)
                standing_ok = await self.brain._verify_standing_after_unknown()
                if standing_ok:
                    standup_confirmed = True
                    self.brain._update_posture_tracking(1004)
                else:
                    print("⚠️ 起立未确认 (3104), 跳过伸懒腰")
                    return

            # Stretch(1017)
            result = self.brain._rpc_call("Stretch")
            stretch_code = result[0] if isinstance(result, tuple) else result
            if stretch_code in (0, -1, 3104):
                await asyncio.sleep(4.0)
                print("✅ 唤醒动画完成")
            else:
                print("⚠️ 伸懒腰失败 (code={})".format(stretch_code))

        except Exception as e:
            print("⚠️ 唤醒动画异常: {}，继续启动".format(e))
        finally:
            self._log_wakeup_audit(
                standup_code, stretch_code, wakeup_start, standup_confirmed)

    def _log_wakeup_audit(self, standup_code, stretch_code, start_time,
                          standup_confirmed=False):
        """记录唤醒动画的审计条目

        Args:
            standup_confirmed: StandUp 是否最终确认成功
                (code=0/-1 直接确认, code=3104 需 _verify_standing_after_unknown 后验)
        """
        try:
            from claudia.brain.audit_logger import AuditEntry, get_audit_logger
            from claudia.brain.audit_routes import ROUTE_STARTUP
            elapsed = (time.time() - start_time) * 1000
            # success, safety_verdict, safety_reason 从同一逻辑派生，避免不一致
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
            pass  # 审计失败不阻塞启动

    async def process_command(self, command: str):
        """处理单个命令"""
        if command.startswith("/"):
            # 系统命令
            if command == "/help":
                self.print_help()
            elif command == "/stats":
                self.print_stats()
            elif command == "/history":
                self.print_history()
            elif command == "/clear":
                os.system('clear' if os.name == 'posix' else 'cls')
                self.print_header()
            elif command == "/exit":
                self.running = False
                print("\n👋 再见！感谢使用Claudia Production Commander\n")
            else:
                print(f"❌ 未知命令: {command}")
        else:
            # 用户指令 — 使用原子入口 process_and_execute（PR2 迁移）
            print(f"\n🎯 处理指令: '{command}'")
            print("-"*40)

            start_time = time.time()
            brain_output = await self.brain.process_and_execute(command)
            process_time = (time.time() - start_time) * 1000

            # 显示结果
            print(f"💬 回复: {brain_output.response}")

            if brain_output.api_code:
                print(f"🔧 API: {brain_output.api_code}")

            if brain_output.sequence:
                print(f"📋 序列: {brain_output.sequence}")

            print(f"⏱️ 处理时间: {process_time:.0f}ms")

            # 执行状态（process_and_execute 内已完成执行）
            if brain_output.api_code or brain_output.sequence:
                print("-"*40)
                if brain_output.execution_status == "success":
                    print("✅ 执行成功")
                elif brain_output.execution_status == "unknown":
                    print("⚠️ 动作超时（机器人可达，可能仍在执行）")
                elif brain_output.execution_status == "failed":
                    print("❌ 执行失败")

            # 记录历史
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.command_history.append((
                timestamp,
                command,
                brain_output.response
            ))

            print("-"*40 + "\n")
    
    async def run(self):
        """运行主循环"""
        self.print_header()

        # 并行执行: LLM 预热 + 唤醒动画（起立→伸懒腰）
        # LLM 冷加载 5-25s，唤醒动画 ~8s，并行执行不增加等待时间
        await asyncio.gather(
            self._warmup_model(),
            self._wakeup_animation(),
        )
        print("")
        
        # 主循环
        while self.running:
            try:
                # 获取用户输入
                command = input("くら> ").strip()
                
                if command:
                    await self.process_command(command)
                    
            except KeyboardInterrupt:
                print("\n\n⚠️ 检测到Ctrl+C，正在退出...")
                self.running = False
            except Exception as e:
                print(f"\n❌ 错误: {e}\n")
        
        # 清理
        print("\n🧹 清理资源...")
        print("✅ 会话结束\n")


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
    
    # 创建并运行命令器
    commander = ProductionCommander(use_real_hardware=args.hardware)
    await commander.run()


if __name__ == "__main__":
    # 设置事件循环
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    
    # 运行主程序
    asyncio.run(main())
