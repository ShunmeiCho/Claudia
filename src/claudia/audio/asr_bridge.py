#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASR ブリッジ — ASR サーバー結果を消費し ProductionBrain へ橋渡し

result.sock から JSON Lines を読み取り、メッセージタイプに応じて分岐:
  - emergency → 即時 brain 呼出 (キュー迂回) + キュー空化 + 冷却
  - transcript → 信頼度フィルタ + 重複排除 → コマンドキュー
  - heartbeat → watchdog 更新
  - ready → プロトコルバージョン検証 + 準備完了シグナル

コマンドキュー (bounded, maxsize=3) → ワーカーが直列消費 → brain.process_and_execute
"""

import asyncio
import logging
import time
from typing import Any, Dict, Optional

from .asr_service.ipc_protocol import (
    ASR_RESULT_SOCKET,
    PROTO_VERSION,
    connect_uds,
    read_json_lines,
    validate_proto_version,
)

logger = logging.getLogger("claudia.asr.bridge")

# 設定定数
QUEUE_MAXSIZE = 3
MIN_CONFIDENCE = 0.55
DEDUP_TTL_S = 10.0
HEARTBEAT_TIMEOUT_S = 15.0
EMERGENCY_COOLDOWN_S = 0.5
RECONNECT_DELAY_S = 2.0


class ASRBridge:
    """ASR 結果消費 + コマンドキュー管理

    Args:
        brain: ProductionBrain インスタンス (process_and_execute を呼ぶ)
        socket_path: result socket パス
        on_result: 結果コールバック (デバッグ表示用, Optional)
    """

    def __init__(
        self,
        brain: Any,
        socket_path: str = ASR_RESULT_SOCKET,
        on_result: Any = None,
    ) -> None:
        self._brain = brain
        self._socket_path = socket_path
        self._on_result = on_result

        # 準備完了イベント (ready メッセージ受信で set)
        self.ready_event = asyncio.Event()

        # コマンドキュー (bounded, drop-oldest)
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=QUEUE_MAXSIZE)

        # 重複排除: utterance_id → 処理時刻
        self._processed_ids: Dict[str, float] = {}

        # ハートビート watchdog
        self._last_heartbeat: float = time.monotonic()

        # Emergency 冷却ウィンドウ
        self._cooldown_until: float = 0.0

        # タスク参照
        self._consumer_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._worker_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._watchdog_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]

        self._running = False

    # ------------------------------------------------------------------
    # 公開 API
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """ブリッジ開始: 結果消費 + コマンドワーカーを起動

        watchdog は ready ハンドシェイク後に開始 (ASR モデル読込中の誤検知防止)。
        """
        self._running = True
        self._last_heartbeat = time.monotonic()

        self._consumer_task = asyncio.ensure_future(self._result_consumer_loop())
        self._worker_task = asyncio.ensure_future(self._command_worker())
        # watchdog は _handle_ready() 内で開始する

        logger.info("ASRBridge 開始")

    async def stop(self) -> None:
        """ブリッジ停止"""
        logger.info("ASRBridge 停止中...")
        self._running = False

        for task in (self._consumer_task, self._worker_task, self._watchdog_task):
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        self._consumer_task = None
        self._worker_task = None
        self._watchdog_task = None

    # ------------------------------------------------------------------
    # 結果消費ループ (自動再接続)
    # ------------------------------------------------------------------

    async def _result_consumer_loop(self) -> None:
        """外層ループ: 接続断 → リトライ"""
        while self._running:
            try:
                reader, writer = await connect_uds(
                    self._socket_path, retries=10, delay=1.0,
                )
                logger.info("result.sock 接続完了")

                try:
                    await self._consume_results(reader)
                finally:
                    try:
                        writer.close()
                    except Exception:
                        pass

            except asyncio.CancelledError:
                break
            except ConnectionError as e:
                if not self._running:
                    break
                logger.warning("result.sock 接続失敗: %s — %.1fs 後リトライ",
                               e, RECONNECT_DELAY_S)
                await asyncio.sleep(RECONNECT_DELAY_S)
            except Exception as e:
                if not self._running:
                    break
                logger.error("結果消費エラー: %s", e, exc_info=True)
                await asyncio.sleep(RECONNECT_DELAY_S)

    async def _consume_results(self, reader: asyncio.StreamReader) -> None:
        """JSON Lines 読取 → メッセージ分岐"""
        async for msg in read_json_lines(reader):
            if not self._running:
                break
            msg_type = msg.get("type", "")
            try:
                if msg_type == "ready":
                    await self._handle_ready(msg)
                elif msg_type == "transcript":
                    await self._handle_transcript(msg)
                elif msg_type == "emergency":
                    await self._handle_emergency(msg)
                elif msg_type == "heartbeat":
                    self._handle_heartbeat(msg)
                elif msg_type == "vad_start":
                    if self._on_result:
                        try:
                            self._on_result("vad_start", "", 0)
                        except Exception:
                            pass
                elif msg_type in ("vad_end", "error", "gate_timeout_audit"):
                    logger.debug("ASR イベント: %s", msg_type)
                else:
                    logger.warning("未知メッセージタイプ: %s", msg_type)
            except Exception as e:
                logger.error("メッセージ処理エラー (%s): %s", msg_type, e)

    # ------------------------------------------------------------------
    # メッセージハンドラ
    # ------------------------------------------------------------------

    async def _handle_ready(self, msg: dict) -> None:
        """ready ハンドシェイク: プロトコルバージョン検証 + watchdog 開始"""
        version = msg.get("proto_version", "")
        model = msg.get("model", "unknown")

        if not validate_proto_version(version):
            logger.error(
                "プロトコルバージョン不一致: remote=%s, local=%s",
                version, PROTO_VERSION,
            )
            return

        logger.info("ASR 準備完了: model=%s, version=%s", model, version)

        # ready 受信後にハートビート watchdog を開始
        # (ASR モデル読込中の誤検知を防ぐ)
        self._last_heartbeat = time.monotonic()
        if self._watchdog_task is None or self._watchdog_task.done():
            self._watchdog_task = asyncio.ensure_future(self._heartbeat_watchdog())

        self.ready_event.set()

    async def _handle_transcript(self, msg: dict) -> None:
        """transcript 処理: emergency 兜底復検 + 信頼度フィルタ + 重複排除 + キュー投入"""
        text = msg.get("text", "").strip()
        confidence = msg.get("confidence", 0.0)
        utterance_id = msg.get("utterance_id", "")

        if not text:
            return

        # 兜底: transcript でも emergency キーワードチェック
        try:
            from .asr_service.emergency_keywords import match_emergency
            emg = match_emergency(text)
            if emg is not None:
                logger.warning("transcript 経由 emergency 検出: '%s'", text)
                await self._handle_emergency_action(utterance_id, text)
                return
        except ImportError:
            pass  # emergency_keywords が無い場合はスキップ

        # 信頼度フィルタ
        if confidence < MIN_CONFIDENCE:
            logger.debug("低信頼度スキップ: '%s' (conf=%.2f)", text, confidence)
            return

        # 重複排除 (emergency が先に処理済みの場合スキップ)
        if self._is_processed(utterance_id):
            logger.debug("重複排除: utt=%s", utterance_id)
            return

        # キューに投入 (mark_processed はワーカー実行時に行う)
        await self._enqueue_command(text, utterance_id)

        if self._on_result:
            try:
                self._on_result("transcript", text, confidence)
            except Exception:
                pass

    async def _handle_emergency(self, msg: dict) -> None:
        """emergency 即時処理"""
        keyword = msg.get("keyword", "")
        utterance_id = msg.get("utterance_id", "")

        logger.warning("Emergency 受信: keyword='%s', utt=%s", keyword, utterance_id)

        await self._handle_emergency_action(utterance_id, keyword)

        if self._on_result:
            try:
                self._on_result("emergency", keyword, 1.0)
            except Exception:
                pass

    async def _handle_emergency_action(self, utterance_id: str, keyword: str) -> None:
        """Emergency 共通処理: brain 呼出 + キュー空化 + 冷却"""
        # 重複排除チェック
        if self._is_processed(utterance_id):
            logger.debug("emergency 重複排除: utt=%s", utterance_id)
            return

        self._mark_processed(utterance_id)

        # 即座に brain へ (キュー迂回)
        logger.warning("Emergency 実行: '%s'", keyword)
        try:
            result = await self._brain.process_and_execute("止まれ")
            logger.info("Emergency 結果: %s", result)
        except Exception as e:
            logger.error("Emergency brain 呼出失敗: %s", e)

        # キュー空化 (stop 後に古いコマンドを実行させない)
        flushed = 0
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
                flushed += 1
            except asyncio.QueueEmpty:
                break
        if flushed:
            logger.info("Emergency 後キュー空化: %d 件破棄", flushed)

        # 冷却ウィンドウ
        self._cooldown_until = time.monotonic() + EMERGENCY_COOLDOWN_S

    def _handle_heartbeat(self, msg: dict) -> None:
        """heartbeat 更新"""
        self._last_heartbeat = time.monotonic()

    # ------------------------------------------------------------------
    # 重複排除 (utterance_id)
    # ------------------------------------------------------------------

    def _is_processed(self, utterance_id: str) -> bool:
        """utterance_id が処理済みか確認 (空文字列は常に未処理扱い)"""
        if not utterance_id:
            return False
        self._cleanup_expired_ids()
        return utterance_id in self._processed_ids

    def _mark_processed(self, utterance_id: str) -> None:
        """utterance_id を処理済みとしてマーク"""
        if utterance_id:
            self._processed_ids[utterance_id] = time.monotonic()

    def _cleanup_expired_ids(self) -> None:
        """TTL 超過した utterance_id を削除"""
        now = time.monotonic()
        expired = [
            uid for uid, ts in self._processed_ids.items()
            if now - ts > DEDUP_TTL_S
        ]
        for uid in expired:
            del self._processed_ids[uid]

    # ------------------------------------------------------------------
    # コマンドキュー
    # ------------------------------------------------------------------

    async def _enqueue_command(self, text: str, utterance_id: str) -> None:
        """bounded キューにコマンドを投入 (満杯時は最古を破棄)"""
        if self._queue.full():
            try:
                discarded = self._queue.get_nowait()
                logger.warning("キュー満杯: 最古コマンド破棄 '%s'", discarded[0])
            except asyncio.QueueEmpty:
                pass
        await self._queue.put((text, utterance_id))
        logger.debug("コマンドキュー投入: '%s' (size=%d)", text, self._queue.qsize())

    async def _command_worker(self) -> None:
        """コマンドワーカー: キューから直列消費 → brain.process_and_execute"""
        while self._running:
            try:
                text, utterance_id = await asyncio.wait_for(
                    self._queue.get(), timeout=1.0,
                )
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break

            # 冷却ウィンドウチェック
            now = time.monotonic()
            if now < self._cooldown_until:
                remaining = self._cooldown_until - now
                logger.debug("冷却中: %.0fms 残り, '%s' スキップ",
                             remaining * 1000, text)
                continue

            # 重複排除 (キュー投入から実行までに同一 ID が処理された場合)
            if self._is_processed(utterance_id):
                logger.debug("ワーカー重複排除: utt=%s", utterance_id)
                continue

            self._mark_processed(utterance_id)

            # brain 呼出
            logger.info("コマンド実行: '%s'", text)
            try:
                result = await self._brain.process_and_execute(text)

                # 結果表示
                if self._on_result:
                    try:
                        self._on_result("result", text, result)
                    except Exception:
                        pass

                logger.info("コマンド結果: response='%s', api=%s, status=%s",
                            result.response[:50] if result.response else "",
                            result.api_code,
                            result.execution_status)

            except Exception as e:
                logger.error("コマンド実行失敗: '%s': %s", text, e)

    # ------------------------------------------------------------------
    # ハートビート watchdog
    # ------------------------------------------------------------------

    async def _heartbeat_watchdog(self) -> None:
        """ハートビート監視: タイムアウト時に再接続を促す"""
        while self._running:
            try:
                await asyncio.sleep(5.0)
            except asyncio.CancelledError:
                break

            elapsed = time.monotonic() - self._last_heartbeat
            if elapsed > HEARTBEAT_TIMEOUT_S:
                logger.warning(
                    "ハートビートタイムアウト: %.0fs 経過 (閾値=%ds)",
                    elapsed, HEARTBEAT_TIMEOUT_S,
                )
                # consumer_task をキャンセル → 再接続ループに入る
                if self._consumer_task and not self._consumer_task.done():
                    self._consumer_task.cancel()
                    # consumer_loop の外側 while が再接続する
                    self._consumer_task = asyncio.ensure_future(
                        self._result_consumer_loop()
                    )
                self._last_heartbeat = time.monotonic()  # リセット
