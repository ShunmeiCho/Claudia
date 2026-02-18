#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Voice Commander — 音声入力による Claudia 操作エントリポイント

USB マイク → ASR 転写 → LLM 処理 → ロボット実行
production_commander.py の音声版。既存 REPL は変更しない。

使い方:
    python3 voice_commander.py              # 音声 + シミュレーション
    python3 voice_commander.py --hardware   # 音声 + 実機
    python3 voice_commander.py --asr-mock   # ASR mock (マイク不要)
"""

import argparse
import asyncio
import json
import logging
import os
import signal
import sys
import time
from datetime import datetime
from typing import Optional

# プロジェクトパス設定 (production_commander.py と同じ方式)
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.append(_PROJECT_ROOT)
sys.path.append(os.path.join(_PROJECT_ROOT, "src"))

from claudia.brain.production_brain import ProductionBrain
from claudia.audio.audio_capture import AudioCapture
from claudia.audio.asr_bridge import ASRBridge
from claudia.audio.asr_service.ipc_protocol import ASR_CTRL_SOCKET, connect_uds, write_json_line

logger = logging.getLogger("claudia.voice")


class VoiceCommander:
    """音声コマンダー — ASR + LLM + ロボット実行を統合

    Args:
        use_real_hardware: True=実機接続, False=シミュレーション
        asr_mock: True=ASR mock モード (マイク不要)
    """

    def __init__(
        self,
        use_real_hardware: bool = False,
        asr_mock: bool = False,
    ) -> None:
        self._use_real_hardware = use_real_hardware
        self._asr_mock = asr_mock
        self._brain: Optional[ProductionBrain] = None
        self._asr_process: Optional[asyncio.subprocess.Process] = None
        self._bridge: Optional[ASRBridge] = None
        self._capture: Optional[AudioCapture] = None
        self._capture_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._stderr_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._shutdown_event = asyncio.Event()
        self._command_count = 0
        self._session_start = datetime.now()

    # ------------------------------------------------------------------
    # メイン実行
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """メインエントリ: 起動 → 監視 → 終了"""
        self._print_header()

        try:
            await self._startup()
            print("\n{} Listening... (Ctrl+C で終了)\n".format(
                "🎙️" if not self._asr_mock else "🧪"))

            # シグナル待ち
            await self._shutdown_event.wait()

        except KeyboardInterrupt:
            print("\n\n⚠️ Ctrl+C 検出、シャットダウン中...")

        except Exception as e:
            logger.error("VoiceCommander エラー: %s", e, exc_info=True)
            print("\n❌ エラー: {}".format(e))

        finally:
            await self._shutdown()

    # ------------------------------------------------------------------
    # 起動シーケンス
    # ------------------------------------------------------------------

    async def _startup(self) -> None:
        """起動: Brain → LLM 予熱 → ASR → Bridge(ready待ち) → AudioCapture"""
        # 1. ProductionBrain 作成
        print("🧠 ProductionBrain 初期化中...")
        self._brain = ProductionBrain(use_real_hardware=self._use_real_hardware)

        # 2. LLM 予熱
        await self._warmup_model()

        # 3. ASR サブプロセス起動
        await self._start_asr_process()

        # 4. ASRBridge 起動 + ready 待ち
        print("🔗 ASRBridge 接続中...")
        self._bridge = ASRBridge(
            brain=self._brain,
            on_result=self._display_result,
        )
        await self._bridge.start()

        # ready ハンドシェイク待ち (ASR モデル読込に時間がかかる場合がある)
        try:
            await asyncio.wait_for(self._bridge.ready_event.wait(), timeout=90)
            print("✅ ASR サーバー準備完了")
        except asyncio.TimeoutError:
            print("❌ ASR ready タイムアウト (90s) — ASR サーバーが起動できません")
            raise RuntimeError("ASR サーバー ready タイムアウト")

        # 5. AudioCapture 起動
        print("🎙️ AudioCapture 起動中...")
        self._capture = AudioCapture(mock=self._asr_mock)
        self._capture_task = asyncio.ensure_future(self._capture.run())
        self._capture_task.add_done_callback(self._on_capture_done)

        # シグナルハンドラ登録
        loop = asyncio.get_event_loop()
        for sig in (signal.SIGTERM, signal.SIGINT):
            loop.add_signal_handler(sig, self._signal_handler)

    def _signal_handler(self) -> None:
        """SIGINT/SIGTERM ハンドラ"""
        self._shutdown_event.set()

    def _on_capture_done(self, task: "asyncio.Task") -> None:
        """AudioCapture 完了/例外コールバック"""
        if not task.cancelled() and task.exception():
            logger.error("AudioCapture 異常終了: %s", task.exception())

    # ------------------------------------------------------------------
    # ASR サブプロセス管理
    # ------------------------------------------------------------------

    async def _start_asr_process(self) -> None:
        """ASR サーバーをサブプロセスとして起動 (-m モジュール方式)"""
        print("🚀 ASR サーバー起動中...")

        # PYTHONPATH 前置追加
        src_dir = os.path.join(_PROJECT_ROOT, "src")
        env = dict(os.environ)
        env["PYTHONPATH"] = src_dir + os.pathsep + env.get("PYTHONPATH", "")

        # ASR モデル: 未指定なら small (精度優先、CPU ~3-5s/utterance)
        # 高速優先: CLAUDIA_ASR_MODEL=base python3 voice_commander.py
        if "CLAUDIA_ASR_MODEL" not in env:
            env["CLAUDIA_ASR_MODEL"] = "small"

        # mock フラグ
        cmd = [sys.executable, "-m", "claudia.audio.asr_service.asr_server"]
        if self._asr_mock:
            cmd.append("--mock")

        self._asr_process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        logger.info("ASR プロセス起動: PID=%d, mock=%s",
                     self._asr_process.pid, self._asr_mock)

        # stderr drain タスク (パイプバッファ溢れ防止)
        self._stderr_task = asyncio.ensure_future(
            self._drain_stderr(self._asr_process.stderr)
        )

    async def _drain_stderr(self, stream: asyncio.StreamReader) -> None:
        """ASR stderr を読み取り logger に転送"""
        try:
            while True:
                line = await stream.readline()
                if not line:
                    break
                text = line.decode("utf-8", errors="replace").rstrip()
                if text:
                    logger.debug("[ASR] %s", text)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.debug("stderr drain 終了: %s", e)

    # ------------------------------------------------------------------
    # LLM 予熱 (production_commander.py から移植)
    # ------------------------------------------------------------------

    async def _warmup_model(self) -> None:
        """LLM モデルをGPUに予熱ロード"""
        print("🔄 LLM 予熱中...")
        model_name = self._brain.model_7b

        def _sync_warmup_http(model, num_ctx):
            from urllib.request import Request, urlopen
            payload = json.dumps({
                "model": model,
                "prompt": "hi",
                "stream": False,
                "options": {"num_predict": 1, "num_ctx": num_ctx},
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
                    messages=[{"role": "user", "content": "hi"}],
                    format="json",
                    options={"num_predict": 1, "num_ctx": num_ctx},
                    keep_alive="30m",
                )

            sync_warmup_fn = _sync_warmup_ollama
        except ImportError:
            print("ℹ️ ollama パッケージ未検出、HTTP API 予熱")

        loop = asyncio.get_event_loop()
        router_mode = self._brain._router_mode.value

        warmup_sequence = []
        if router_mode == "shadow":
            action_model = self._brain._channel_router._action_model
            warmup_sequence = [
                (action_model, 1024, "Action", 30),
                (model_name, 2048, "7B", 60),
            ]
        elif router_mode == "dual":
            action_model = self._brain._channel_router._action_model
            warmup_sequence = [
                (action_model, 1024, "Action", 30),
            ]
        else:
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
                print("✅ {} 予熱完了 ({}: {:.0f}ms)".format(label, model, elapsed))
            except asyncio.TimeoutError:
                print("⚠️ {} 予熱タイムアウト ({}s)".format(label, timeout_s))
            except Exception as e:
                print("⚠️ {} 予熱失敗: {}".format(label, e))

    # ------------------------------------------------------------------
    # 結果表示
    # ------------------------------------------------------------------

    def _display_result(self, event_type: str, text: str, data) -> None:
        """ASRBridge からのコールバック: 結果をターミナルに表示"""
        if event_type == "vad_start":
            print("  ... (聴取中)", end="", flush=True)
            return
        elif event_type == "emergency":
            print("\n🚨 Emergency: '{}'".format(text))
        elif event_type == "transcript":
            print("\n🎤 認識: '{}' (conf={:.2f})".format(text, data))
        elif event_type == "result":
            self._command_count += 1
            result = data
            print("─" * 40)
            print("💬 応答: {}".format(result.response))
            if result.api_code:
                print("🔧 API: {}".format(result.api_code))
            if result.sequence:
                print("📋 シーケンス: {}".format(result.sequence))
            status = result.execution_status
            if status == "success":
                print("✅ 実行成功")
            elif status == "unknown":
                print("⚠️ タイムアウト (実行中の可能性)")
            elif status == "failed":
                print("❌ 実行失敗")
            print("─" * 40)

    # ------------------------------------------------------------------
    # シャットダウン
    # ------------------------------------------------------------------

    async def _shutdown(self) -> None:
        """優雅なシャットダウン: Capture → ctrl shutdown → Bridge → ASR 終了"""
        print("\n🛑 シャットダウン中...")

        # 1. AudioCapture 停止
        if self._capture:
            await self._capture.shutdown()

        if self._capture_task and not self._capture_task.done():
            self._capture_task.cancel()
            try:
                await self._capture_task
            except asyncio.CancelledError:
                pass

        # 2. ASR に shutdown 送信 (ctrl socket 経由)
        await self._send_ctrl_shutdown()

        # 3. ASRBridge 停止
        if self._bridge:
            await self._bridge.stop()

        # 4. ASR プロセス終了待ち
        await self._stop_asr_process()

        # 5. stderr drain 停止
        if self._stderr_task and not self._stderr_task.done():
            self._stderr_task.cancel()
            try:
                await self._stderr_task
            except asyncio.CancelledError:
                pass

        runtime = datetime.now() - self._session_start
        print("\n✅ セッション終了 (コマンド: {}件, 時間: {:.0f}s)".format(
            self._command_count, runtime.total_seconds()))

    async def _send_ctrl_shutdown(self) -> None:
        """ctrl socket 経由で ASR に shutdown メッセージを送信"""
        try:
            _, writer = await asyncio.wait_for(
                connect_uds(ASR_CTRL_SOCKET, retries=2, delay=0.5),
                timeout=3.0,
            )
            await write_json_line(writer, {"type": "shutdown", "reason": "voice_commander_exit"})
            writer.close()
            logger.info("ASR shutdown メッセージ送信完了")
        except Exception as e:
            logger.debug("ctrl shutdown 送信失敗 (ASR 既に終了?): %s", e)

    async def _stop_asr_process(self) -> None:
        """ASR サブプロセスを終了 (3s待ち → terminate → 5s → kill)"""
        if not self._asr_process:
            return

        if self._asr_process.returncode is None:
            # まだ実行中 → 正常終了を待つ
            try:
                await asyncio.wait_for(self._asr_process.wait(), timeout=3.0)
                logger.info("ASR プロセス正常終了")
            except asyncio.TimeoutError:
                # TERM → KILL エスカレーション
                logger.warning("ASR プロセス TERM 送信")
                try:
                    self._asr_process.terminate()
                    await asyncio.wait_for(self._asr_process.wait(), timeout=5.0)
                except asyncio.TimeoutError:
                    logger.warning("ASR プロセス KILL 送信")
                    self._asr_process.kill()
                    await self._asr_process.wait()
                except ProcessLookupError:
                    pass
        else:
            logger.info("ASR プロセス既に終了: code=%d", self._asr_process.returncode)

        # Python 3.8: subprocess transport を明示的に close し、
        # GC 時の "Event loop is closed" RuntimeError を防ぐ
        # 全 return 経路で必ず実行される
        try:
            transport = self._asr_process._transport  # type: ignore[attr-defined]
            if transport is not None:
                transport.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _print_header(self) -> None:
        """ヘッダー表示"""
        print("\n" + "=" * 60)
        print("🎙️ Claudia Voice Commander — 音声対話モード")
        print("=" * 60)
        hw = "実機" if self._use_real_hardware else "シミュレーション"
        asr = "mock" if self._asr_mock else "production"
        print("⚙️  モード: {} / ASR: {}".format(hw, asr))
        print("⏰ セッション: {}".format(self._session_start.strftime("%Y-%m-%d %H:%M:%S")))
        print("-" * 60)
        print("💡 日本語で話しかけてください (例: お手, 座って, 踊って)")
        print("💡 緊急停止: 止まれ / stop / 停止")
        print("-" * 60)


# ======================================================================
# エントリ
# ======================================================================

def _setup_logging() -> None:
    """ログ設定"""
    log_level = os.getenv("CLAUDIA_VOICE_LOG_LEVEL", "INFO").upper()
    logging.basicConfig(
        level=getattr(logging, log_level, logging.INFO),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def _parse_args() -> argparse.Namespace:
    """CLI 引数"""
    parser = argparse.ArgumentParser(description="Claudia Voice Commander")
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="実機接続モード (デフォルト: シミュレーション)",
    )
    parser.add_argument(
        "--asr-mock",
        action="store_true",
        help="ASR mock モード (マイク不要、テスト用)",
    )
    return parser.parse_args()


async def _async_main(args: argparse.Namespace) -> None:
    """非同期メイン"""
    asr_mock = args.asr_mock or os.getenv("ASR_MOCK", "0") == "1"

    commander = VoiceCommander(
        use_real_hardware=args.hardware,
        asr_mock=asr_mock,
    )
    await commander.run()


def main() -> None:
    """同期エントリ"""
    _setup_logging()
    args = _parse_args()

    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(_async_main(args))


if __name__ == "__main__":
    main()
