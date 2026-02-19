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
import re
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

# ASR stderr からメッセージ部分を抽出する正規表現
# 例: "2026-02-19 15:31:03 [claudia.asr.server] INFO: 実際のメッセージ"
#   → "実際のメッセージ"
_ASR_LOG_LEVELS = ["INFO", "WARNING", "ERROR", "DEBUG", "CRITICAL"]
_ASR_LEVEL_PAT = "(?:" + "|".join(_ASR_LOG_LEVELS) + ")"
_ASR_LOG_RE = re.compile(
    r"^\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}\s+"     # timestamp
    r"\[[\w.]+\]\s+"                                    # [logger.name]
    + _ASR_LEVEL_PAT + r":\s*"                          # LEVEL:
    r"(.+)$"                                            # message
)

# フェーズ表示の幅定数
_PHASE_WIDTH = 48


class _QuietFilter(logging.Filter):
    """起動フェーズ中にログを抑制するフィルター

    ERROR 以上のみ通す。WARNING (GetState 探測失敗等) は
    Go2 の既知動作であり phase サマリーで十分。
    """

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


def _phase_fail(msg):
    """フェーズ失敗を追記"""
    print(" FAIL")
    print("        {}".format(msg))


def _display_width(text):
    """テキストの端末表示幅を計算 (CJK 全角文字を 2 カラムとして扱う)"""
    w = 0
    for ch in text:
        cp = ord(ch)
        # CJK Unified Ideographs, Hiragana, Katakana, Fullwidth forms
        if (0x3000 <= cp <= 0x9FFF
                or 0xF900 <= cp <= 0xFAFF
                or 0xFF01 <= cp <= 0xFF60):
            w += 2
        else:
            w += 1
    return w


def _box_header(lines):
    """ボックス罫線ヘッダーを生成 (CJK 幅対応)"""
    inner = 44  # | + space + content + space + |
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
        self._startup_phase = True  # ASR stderr drain 抑制フラグ
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

        # 起動フェーズ中のログを抑制 (ERROR 以上のみ通す)
        # Brain: logger 級 (handler は __init__ 内生成)
        # Root: handler 級 (propagate 経由を捕捉)
        quiet = _QuietFilter()
        brain_logger = logging.getLogger("ProductionBrain")
        brain_logger.addFilter(quiet)
        _suppressed_handlers = []
        for h in logging.getLogger().handlers:
            h.addFilter(quiet)
            _suppressed_handlers.append(h)

        try:
            await self._startup()
            icon = "mic" if not self._asr_mock else "mock"
            print("\n  [{}] Listening... (Ctrl+C で終了)\n".format(icon))

            # AudioCapture 初期接続ログを吸収するため短時間待機後に解除
            await asyncio.sleep(0.5)
        finally:
            brain_logger.removeFilter(quiet)
            for h in _suppressed_handlers:
                h.removeFilter(quiet)
            self._startup_phase = False

        try:
            # ASR プロセス死活監視 + シグナル待ちを並行
            asr_monitor = asyncio.ensure_future(self._monitor_asr_process())
            done, _ = await asyncio.wait(
                [self._shutdown_event.wait(), asr_monitor],
                return_when=asyncio.FIRST_COMPLETED,
            )
            # monitor が先に完了 = ASR クラッシュ
            if asr_monitor in done and not self._shutdown_event.is_set():
                print("\n  [error] ASR サーバーが予期せず終了しました")
                logger.error("ASR プロセスが予期せず終了 — シャットダウン開始")

        except KeyboardInterrupt:
            print("\n\n  Ctrl+C 検出、シャットダウン中...")

        except Exception as e:
            logger.error("VoiceCommander エラー: %s", e, exc_info=True)
            print("\n  [error] {}".format(e))

        finally:
            await self._shutdown()

    # ------------------------------------------------------------------
    # 起動シーケンス
    # ------------------------------------------------------------------

    async def _startup(self) -> None:
        """起動: Brain → LLM 予熱 → ASR → Bridge(ready待ち) → AudioCapture

        ログ抑制は run() 側で管理 (Listening 表示後に解除するため)。
        """
        total = 5
        print()

        # 1. ProductionBrain 作成
        t = _phase_start(1, total, "Brain")
        self._brain = ProductionBrain(use_real_hardware=self._use_real_hardware)

        hw = "実機" if self._use_real_hardware else "sim"
        mode = self._brain._router_mode.value
        detail = "{} / {}".format(hw, mode)
        try:
            if self._brain.state_monitor:
                batt = self._brain.state_monitor.battery_level
                if batt is not None:
                    detail += " / battery {}%".format(batt)
        except Exception:
            pass
        _phase_ok_detail(t, detail)

        # 2. LLM 予熱
        t = _phase_start(2, total, "LLM warmup")
        await self._warmup_model(quiet=True)
        _phase_ok(t)

        # 3. ASR サブプロセス起動
        t = _phase_start(3, total, "ASR server")
        await self._start_asr_process()
        _phase_ok(t)

        # 4. ASRBridge 起動 + ready 待ち
        t = _phase_start(4, total, "ASR bridge")
        self._bridge = ASRBridge(
            brain=self._brain,
            on_result=self._display_result,
        )
        await self._bridge.start()

        try:
            await asyncio.wait_for(self._bridge.ready_event.wait(), timeout=90)
            _phase_ok(t)
        except asyncio.TimeoutError:
            _phase_fail("ready タイムアウト (90s)")
            raise RuntimeError("ASR サーバー ready タイムアウト")

        # 5. AudioCapture 起動
        t = _phase_start(5, total, "Mic capture")
        self._capture = AudioCapture(mock=self._asr_mock)
        self._capture_task = asyncio.ensure_future(self._capture.run())
        self._capture_task.add_done_callback(self._on_capture_done)
        _phase_ok(t)

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

    async def _monitor_asr_process(self) -> None:
        """ASR サブプロセス死活監視: プロセス終了を検出"""
        if not self._asr_process:
            return
        await self._asr_process.wait()
        if self._shutdown_event.is_set():
            return  # 正常シャットダウン中
        rc = self._asr_process.returncode
        logger.error("ASR プロセスが予期せず終了: code=%s", rc)

    # ------------------------------------------------------------------
    # ASR サブプロセス管理
    # ------------------------------------------------------------------

    async def _start_asr_process(self) -> None:
        """ASR サーバーをサブプロセスとして起動 (-m モジュール方式)"""
        # PYTHONPATH 前置追加
        src_dir = os.path.join(_PROJECT_ROOT, "src")
        env = dict(os.environ)
        env["PYTHONPATH"] = src_dir + os.pathsep + env.get("PYTHONPATH", "")

        # ASR モデル: 未指定なら base (速度優先、CPU ~2-3s/utterance)
        if "CLAUDIA_ASR_MODEL" not in env:
            env["CLAUDIA_ASR_MODEL"] = "base"

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
        """ASR stderr を読み取り、二重フォーマットを排除して表示

        ASR サブプロセスの stderr は既にフォーマット済み
        (例: "2026-02-19 15:31:03 [claudia.asr.server] INFO: message")。
        正規表現でメッセージ部分のみ抽出し、print で直接出力する。
        ERROR/WARNING 含有行は [ASR!] プレフィックスで強調表示。
        起動フェーズ中は WARNING 以上のみ表示。
        """
        _error_indicators = ("error", "exception", "traceback", "critical",
                             "fatal", "segfault", "sigabrt", "killed")
        try:
            while True:
                line = await stream.readline()
                if not line:
                    break
                text = line.decode("utf-8", errors="replace").rstrip()
                if not text:
                    continue

                # フォーマット済みログからメッセージ部分を抽出
                m = _ASR_LOG_RE.match(text)
                msg = m.group(1) if m else text

                text_lower = text.lower()
                is_error = any(ind in text_lower for ind in _error_indicators)

                # 起動フェーズ中は WARNING 以上のみ通す
                if self._startup_phase and not is_error:
                    continue

                if is_error:
                    print("  [ASR!] {}".format(msg), flush=True)
                else:
                    print("  [ASR] {}".format(msg), flush=True)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.debug("stderr drain 終了: %s", e)

    # ------------------------------------------------------------------
    # LLM 予熱 (production_commander.py から移植)
    # ------------------------------------------------------------------

    async def _warmup_model(self, quiet: bool = False) -> None:
        """LLM モデルをGPUに予熱ロード

        Args:
            quiet: True でフェーズ進捗のサブ出力を抑制
        """
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
            pass

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
                await asyncio.wait_for(
                    loop.run_in_executor(None, sync_warmup_fn, model, num_ctx),
                    timeout=timeout_s,
                )
            except asyncio.TimeoutError:
                if not quiet:
                    print("  [warn] {} warmup timeout ({}s)".format(label, timeout_s))
            except Exception as e:
                if not quiet:
                    print("  [warn] {} warmup failed: {}".format(label, e))

    # ------------------------------------------------------------------
    # 結果表示
    # ------------------------------------------------------------------

    def _display_result(self, event_type: str, text: str, data) -> None:
        """ASRBridge からのコールバック: 結果をターミナルに表示"""
        if event_type == "vad_start":
            print("  ... (聴取中)", end="", flush=True)
            return
        elif event_type == "emergency":
            print("\n  >>> EMERGENCY: '{}'".format(text))
        elif event_type == "transcript":
            print("\n  mic> '{}' (conf={:.2f})".format(text, data))
        elif event_type == "e2e_timing":
            timing = data
            asr = timing.get("asr_ms", 0)
            brain = timing.get("brain_ms", 0)
            e2e = timing.get("e2e_ms", 0)
            print("  time: {:.0f}ms (ASR={:.0f} + Brain={:.0f})".format(
                e2e, asr, brain))
        elif event_type == "result":
            self._command_count += 1
            result = data
            print("  " + "-" * 38)
            print("  res> {}".format(result.response))
            if result.api_code:
                print("  api: {}".format(result.api_code))
            if result.sequence:
                print("  seq: {}".format(result.sequence))
            status = result.execution_status
            if status == "success":
                print("  [ok]")
            elif status == "unknown":
                print("  [timeout] (実行中の可能性)")
            elif status == "failed":
                print("  [failed]")
            print("  " + "-" * 38)

    # ------------------------------------------------------------------
    # シャットダウン
    # ------------------------------------------------------------------

    async def _shutdown(self) -> None:
        """優雅なシャットダウン: Capture → ctrl shutdown → Bridge → ASR 終了"""
        print("\n  shutting down...")

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
        print("  session: {} commands, {:.0f}s\n".format(
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

        # Python 3.8: subprocess transport + pipe transport を明示的に close し、
        # GC 時の "Event loop is closed" RuntimeError を防ぐ
        try:
            transport = self._asr_process._transport  # type: ignore[attr-defined]
            if transport is not None:
                transport.close()
        except Exception:
            pass

        # stderr pipe transport を明示 close
        for pipe_name in ("stderr", "stdout", "stdin"):
            try:
                pipe = getattr(self._asr_process, pipe_name, None)
                if pipe is not None:
                    pipe_transport = getattr(pipe, "_transport", None)
                    if pipe_transport is not None:
                        pipe_transport.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _print_header(self) -> None:
        """ヘッダー表示"""
        hw = "実機" if self._use_real_hardware else "sim"
        asr = "mock" if self._asr_mock else "production"
        ts = self._session_start.strftime("%Y-%m-%d %H:%M:%S")

        print()
        print(_box_header([
            "Claudia Voice Commander",
            "{} / ASR {} ".format(hw, asr),
            ts,
        ]))
        print()
        print("  話しかけてください (例: お手, 座って, 踊って)")
        print("  緊急停止: 止まれ / stop / 停止")
        print("  Ctrl+C で終了")


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
    # httpx の HTTP リクエストログは常に不要
    logging.getLogger("httpx").setLevel(logging.WARNING)


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
