[English](README.md) | [日本語](README.ja.md) | [中文](README.zh.md)

# Claudia — LLM頭脳ロボットインテリジェンス

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-green.svg)](https://developer.nvidia.com/embedded/jetson-orin)
[![Robot](https://img.shields.io/badge/Robot-Unitree%20Go2-orange.svg)](https://www.unitree.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Claudia**は、**Unitree Go2**四足歩行ロボット向けのLLM頭脳AIシステムです。日本語・中国語・英語の自然言語コマンドを、ローカルLLM推論（Ollama上のQwen2.5-7B）によってロボット動作に変換します。NVIDIA Jetson Orin NX上で完全にオンデバイスで動作します。

> *「LLMがロボットの頭脳」* — キーワードマッチングではなく、意味理解。

---

## 主な特徴

### LLM頭脳アーキテクチャ
- **意味理解**: 抽象概念をロボット動作にマッピング（例：「可愛い」→ ハートジェスチャー、「疲れた」→ 座る）
- **直接API出力**: LLMがアクションコード付きの構造化JSONを出力 — 中間マッピング層不要
- **唯一の真実源**: 全27動作定義が`action_registry.py`に集約。ホワイトリスト・起立要件・メソッドマップは自動派生
- **決定論的推論**: `temperature=0.0`のJSONモードで一貫した動作を保証

### 安全システム (SafetyCompiler)
- **統一安全パイプライン**: 全動作パスが`SafetyCompiler.compile()`を通過 — バイパス不可
- **3段階バッテリーゲーティング**: <=10%: 安全動作のみ | <=20%: 高エネルギー禁止 | <=30%: フリップをダンスに降格
- **起立前提条件**: 必要時にStandUpを自動前挿入（例：Helloは起立が必要）
- **仮想姿勢追跡**: 動作シーケンス内の姿勢変化を追跡し、正確な前提条件挿入を実現
- **ホワイトリスト強制**: 登録済み・有効化された動作のみ実行可能

### ハードウェア制御
- **15の検証済み動作**: 基本姿勢8 + パフォーマンス7 + 高度3（[対応動作](#対応動作)参照）
- **リアルタイム制御**: 1ms（キャッシュ）〜約5秒（Jetson上LLM推論）の応答時間
- **状態認識シーケンス**: 動作依存関係の自動解決
- **グレースフルフォールバック**: 実機ハードウェア → モックシミュレーション

### 多言語対話
- **日本語優先**: ロボットとの自然な日本語会話
- **中国語対応**: 中国語コマンドの完全認識
- **英語互換**: 基本的な英語コマンドに対応
- **ASRカナ正規化**: 音声認識出力のクリーンアップ用KANA_ALIASESパイプライン内蔵

---

## クイックスタート

### 必要環境

| コンポーネント | 要件 |
|:---:|---|
| ロボット | Unitree Go2（R&D Plus推奨） |
| コンピュート | NVIDIA Jetson Orin NX |
| OS | Ubuntu 20.04 (aarch64) |
| Python | 3.8以上 |
| LLMランタイム | [Ollama](https://ollama.ai/) |
| ミドルウェア | ROS2 Foxy + CycloneDDS |
| ネットワーク | ロボットへのイーサネット (`192.168.123.x`) |

### インストール

```bash
git clone https://github.com/ShunmeiCho/Claudia.git
cd claudia
pip install -e .

# 環境設定
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

### 起動

```bash
# インタラクティブランチャー（推奨）
./start_production_brain.sh

# キーボードモード:
python3 production_commander.py              # シミュレーションモード
python3 production_commander.py --hardware   # 実機モード

# 音声モード（Phase 2: USBマイク → ASR → LLM → ロボット）
python3 voice_commander.py                   # 音声、シミュレーション
python3 voice_commander.py --hardware        # 音声、実機
python3 voice_commander.py --asr-mock        # 音声、モックASR（マイク不要）
```

---

## 使用例

```
Claudia> こんにちは          → 挨拶ジェスチャー (1016)
Claudia> 座って              → 座る (1009)
Claudia> 可愛いね            → ハートジェスチャー (1036) — 意味理解: 「可愛い」→ 愛情表現
Claudia> 立ってから挨拶して  → シーケンス: StandUp(1004) → Hello(1016)
Claudia> 疲れた              → 座る (1009) — 意味理解: 「疲れた」→ 休憩
Claudia> dance               → ランダムにDance1(1022)またはDance2(1023)
Claudia> 量子力学について教えて → 会話応答（動作なし）
```

---

## 対応動作

### 基本姿勢（8動作）

| APIコード | メソッド | 日本語 | 中国語 | 英語 | 起立必要 |
|:-:|--------|--------|---------|---------|:-:|
| 1001 | Damp | ダンプモード | 阻尼模式 | Damp | - |
| 1002 | BalanceStand | バランス | 平衡站立 | Balance | - |
| 1003 | StopMove | 止まる | 停止 | Stop | - |
| 1004 | StandUp | 立つ | 站立 | Stand Up | - |
| 1005 | StandDown | 伏せる | 趴下 | Stand Down | - |
| 1006 | RecoveryStand | 回復 | 恢复站立 | Recovery | - |
| 1009 | Sit | 座る | 坐下 | Sit | Yes |
| 1010 | RiseSit | 起き上がる | 起立 | Rise Sit | - |

### パフォーマンス（7動作）

| APIコード | メソッド | 日本語 | 中国語 | 英語 | 起立必要 |
|:-:|--------|--------|---------|---------|:-:|
| 1016 | Hello | 挨拶 | 打招呼 | Hello | Yes |
| 1017 | Stretch | 伸び | 伸懒腰 | Stretch | Yes |
| 1021 | Wallow | 転がる | 翻滚 | Wallow | - |
| 1022 | Dance1 | ダンス1 | 舞蹈1 | Dance 1 | Yes |
| 1023 | Dance2 | ダンス2 | 舞蹈2 | Dance 2 | Yes |
| 1029 | Scrape | 刮る | 刮擦 | Scrape | Yes |
| 1033 | WiggleHips | 腰振り | 摇臀 | Wiggle Hips | Yes |
| 1036 | Heart | ハート | 比心 | Heart | Yes |

### 高度・高リスク（3動作）

| APIコード | メソッド | 日本語 | 中国語 | 英語 | リスク |
|:-:|--------|--------|---------|---------|------|
| 1030 | FrontFlip | 前転 | 前空翻 | Front Flip | 高 |
| 1031 | FrontJump | ジャンプ | 前跳 | Front Jump | 高 |
| 1032 | FrontPounce | 飛びかかる | 前扑 | Front Pounce | 高 |

> 高リスク動作はバッテリーゲーティングされ、起立状態が必要です。デフォルトでは無効（`allow_high_risk=False`）。

---

## アーキテクチャ

### コマンド処理パイプライン

```
ユーザー入力（日/中/英）
  |
  v
1. 緊急バイパス .............. ハードコードされた停止コマンド、~0ms
  |
  v
2. ホットキャッシュ .......... 60+件のキャッシュ済みコマンド→APIマッピング、~1ms
  |                           （文化的表現、カナエイリアス、サフィックス除去）
  v
3. 会話検出 .................. 挨拶/質問 → テキストのみの応答
  |
  v
4. LLM推論（BRAIN_ROUTER_MODEでルーティング制御）:
  |  - dual（デフォルト）: Actionモデル（~30トークン、Jetsonで3-5秒）
  |  - legacy: 7Bモデル（フル応答＋アクションコード）
  |  - shadow: 両チャネル、A/B比較ログ
  v
SafetyCompiler.compile() ...... ホワイトリスト→バッテリーゲート→起立前提
  |
  v
実行 ......................... SportClient RPC via CycloneDDS/DDS
```

### モジュール概要

| モジュール | 責務 |
|--------|------|
| `brain/production_brain.py` | コアパイプライン: キャッシュ→ルーター→安全→実行 |
| `brain/channel_router.py` | デュアルチャネルLLMルーター（legacy/dual/shadowモード） |
| `brain/action_registry.py` | 全動作定義の唯一の真実源 |
| `brain/safety_compiler.py` | 統一安全パイプライン（バッテリー、起立、ホワイトリスト） |
| `brain/audit_logger.py` | 構造化監査ログ (`logs/audit/`) |
| `brain/audit_routes.py` | 監査ログ用の正規ルート名 |
| `brain/sdk_state_provider.py` | 直接SDK状態クエリ（ROS2モニターの代替） |
| `brain/mock_sport_client.py` | テスト用SportClientシミュレーター |
| `robot_controller/system_state_monitor.py` | ROS2ベースのバッテリー/姿勢監視（5Hz） |
| `robot_controller/unified_led_controller.py` | LEDモードAPI（思考中/成功/エラー/リスニング） |
| `production_commander.py` | キーボードREPLエントリーポイント |
| `voice_commander.py` | 音声モードエントリーポイント: ASRサブプロセス + AudioCapture + ASRBridge |
| `audio/audio_capture.py` | USBマイク取得: arecordサブプロセス → リサンプル → UDS |
| `audio/asr_bridge.py` | ASR結果コンシューマー: 重複排除/フィルター → Queue → Brain |
| `audio/asr_service/` | ASRサーバー: faster-whisper + silero-vad + UDS |

---

## 音声認識 (ASR)

> ステータス: **Phase 2稼働中** — Jetson上でUSBマイクを使用した完全な音声パイプラインが稼働。

### 音声パイプライン

```
USBマイク (AT2020USB-XP, カード自動検出, 44100Hz)
  │ arecordサブプロセス → リサンプル → 16kHz 960byteフレーム
  v
AudioCapture ──→ /tmp/claudia_audio.sock ──→ ASR Server（サブプロセス）
                                                ├── silero-vad + 緊急検出
                                                ├── faster-whisper base (ja, beam=1, CPU int8)
                                                v
ASRBridge ←── /tmp/claudia_asr_result.sock ←─── JSON Lines
  ├── emergency → キューフラッシュ + クールダウン → brain呼び出し（ロックバイパス）
  ├── transcript → 信頼度 ≥0.55 フィルター → 重複排除 → Queue(3)
  └── command worker → brain.process_and_execute(text)
```

### ASR環境変数オーバーライド

| 変数 | デフォルト | 選択肢 |
|------|-----------|--------|
| `CLAUDIA_ASR_MODEL` | `base` | `base` / `small` / `medium` |
| `CLAUDIA_ASR_BEAM_SIZE` | `1`（貪欲デコード） | `1` / `3`+（ビームサーチ） |
| `CLAUDIA_ASR_DEVICE` | `cpu` | `cpu` / `cuda` |

### カナ正規化

- **KANA_ALIASESパイプライン**: ホットキャッシュ層に統合済み。一般的なASRカナ出力を漢字表記に正規化（例：`おすわり` → `お座り`、`おて` → `お手`、`はーと` → `ハート`）。日本語音声コマンドにおけるASRミスマッチの最大の原因を排除。
- **日本語サフィックス除去**: 丁寧語サフィックス（です/ます/ください）はホットキャッシュマッチング時に自動除去（例：`かわいいです` → `かわいい`）。
- **緊急コマンドカナバリアント**: `EMERGENCY_COMMANDS`辞書にカナのみのバリアント（`とまれ`、`とめて`、`ていし`）を含み、不完全なASR転写でも緊急停止が確実に動作。

---

## テキスト読み上げ (TTS)

> ステータス: **エコーゲーティング実装済み** — ASRサーバーにTTSエコーゲーティング（ctrlソケット経由のtts_start/tts_end）を実装済み。TTSプロバイダー統合は未実装。

### 現在の状態

応答は現在REPLにテキストとして表示されます。ロボットは日本語で応答します（`_sanitize_response()`がひらがな/カタカナ/漢字の存在を検証）。ASRサーバーはTTS再生中の認識をミュートするエコーゲーティングを実装しています。

### 計画アーキテクチャ (PR3)

```
ProductionBrain
  |
  v
BrainOutput.response（日本語テキスト）
  |
  v
Commander._speak_nonblocking()
  |  - TTSプロバイダー（プラグイン可能）
  |    - VOICEVOX（日本語、高品質）
  |    - Google TTS
  |    - gTTS（軽量）
  |  - ThreadPoolExecutor（ノンブロッキング）
  |  - generation_idキャンセル機構
  v
音声出力（スピーカー）
```

**重要な設計原則**: 頭脳はTTSに**一切関与しない**。`ProductionBrain`は`BrainOutput`（テキスト＋アクションコード）を生成するのみ。TTS再生はコマンダー層が完全に管理。これにより：
- 頭脳のテストにTTSモックが不要
- TTS障害が動作実行をブロックしない
- 新しいコマンドが再生中の音声を自動キャンセル

---

## 技術スタック

| コンポーネント | 技術 |
|:---:|---|
| LLM | Qwen2.5-7B via [Ollama](https://ollama.ai/) |
| ロボット | Unitree Go2 + unitree_sdk2_python |
| 通信 | CycloneDDS + ROS2 Foxy |
| プラットフォーム | NVIDIA Jetson Orin NX (aarch64) |
| 言語 | Python 3.8.10 |
| OS | Ubuntu 20.04 |
| GPU | CUDA 11.4 |

---

## 開発

### インストール（開発モード）

```bash
pip install -e ".[dev]"    # pytest, black, flake8, mypy を含む
```

### テスト

```bash
python3 test/run_tests.py                    # 全テスト
python3 test/run_tests.py --type unit        # ユニットのみ
python3 test/run_tests.py --type hardware    # ハードウェアのみ
pytest test/ -v                               # pytest経由
```

### リント/フォーマット

```bash
black --line-length 88 src/
flake8 src/
mypy src/
```

---

## トラブルシューティング

| 問題 | 原因 | 解決策 |
|------|------|--------|
| エラー3103 | Unitreeアプリがsport modeを占有 | アプリを閉じてロボットを再起動 |
| DDS接続失敗 | ネットワーク設定の問題 | `eth0`が`192.168.123.x`であることを確認、`RMW_IMPLEMENTATION`をチェック |
| LLMタイムアウト | モデル未読み込み | `ollama list`を実行、`curl localhost:11434/api/tags`をチェック |
| インポートエラー | PYTHONPATHの不足 | `export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH` |
| エラー3104 | RPCタイムアウト（非同期動作） | ロボットがまだ実行中の可能性あり。接続性を確認 |
| Actionチャネル10秒タイムアウト | Jetson GPUコールドスタート | アイドル後の最初のコマンドは正常。モデルは自動的にウォームアップ |
| `(聴取中)`だが認識なし | マイクミュート/ゲイン不足 | マイクゲインを確認、テスト: `arecord -D hw:X,0 -d 3 /tmp/t.raw` |
| ASR遅い（>5秒/発話） | CPUでwhisper-small使用 | base（デフォルト）を使用: `CLAUDIA_ASR_MODEL=base` |

---

## ロードマップ

| フェーズ | 内容 | ステータス |
|:---:|---|:---:|
| PR1 | SafetyCompiler + action_registry + P0安全修正 | 完了 |
| PR2 | デュアルチャネルLLMルーティング（アクション+音声分離） | 完了 |
| PR3 | ASR/TTS統合 | Phase 2完了（ASR）、TTS未実装 |
| P2 | パラメーター化動作（Move, Euler, SpeedLevel） | 将来 |
| P2 | 3Bアクションチャネル A/Bテスト | 将来 |

---

## ライセンス

MIT License — 詳細は[LICENSE](LICENSE)を参照。

## コントリビューター

- **ShunmeiCho** — プロジェクトビジョンとコアインサイト
- **Claude AI** — 技術実装とアーキテクチャ設計

---

*最終更新: 2026-02-19*
