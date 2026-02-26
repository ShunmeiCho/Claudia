<p align="center">
  <a href="README.md">English</a> ·
  <strong>日本語</strong> ·
  <a href="README.zh.md">中文</a>
</p>

<p align="center">
  <img src="docs/images/cover.jpg" alt="Claudia — Unitree Go2 LLM頭脳ロボット" width="800">
  <br>
  <sub>画像提供: <a href="https://www.unitree.com/go2">Unitree Robotics</a></sub>
</p>

# Claudia — LLM頭脳ロボットインテリジェンス

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-green.svg)](https://developer.nvidia.com/embedded/jetson-orin)
[![Robot](https://img.shields.io/badge/Robot-Unitree%20Go2-orange.svg)](https://www.unitree.com/)
[![LLM](https://img.shields.io/badge/LLM-Qwen2.5--7B-purple.svg)](https://huggingface.co/Qwen/Qwen2.5-7B)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Claudia**は、**Unitree Go2**四足歩行ロボット向けのLLM頭脳AIシステムです。日本語・中国語・英語の自然言語コマンドを、ローカルLLM推論（Ollama上のQwen2.5-7B）によってロボット動作に変換します。NVIDIA Jetson Orin NX上で完全にオンデバイスで動作します。

> *「LLMがロボットの頭脳」* — キーワードマッチングではなく、意味理解。

---

<details>
<summary><strong>目次</strong></summary>

- [デモ](#デモ)
- [主な特徴](#主な特徴)
- [クイックスタート](#クイックスタート)
- [使用例](#使用例)
- [対応動作](#対応動作)
- [アーキテクチャ](#アーキテクチャ)
- [音声認識 (ASR)](#音声認識-asr)
- [技術スタック](#技術スタック)
- [開発](#開発)
- [トラブルシューティング](#トラブルシューティング)
- [ロードマップ](#ロードマップ)
- [謝辞](#謝辞)
- [ライセンス](#ライセンス)

</details>

## デモ

### Claudia 音声パイプライン

[![Voice Demo](https://img.youtube.com/vi/O6TpC9Se3Ds/sddefault.jpg)](https://youtu.be/O6TpC9Se3Ds)

> 音声 → ASR → ローカル LLM → ロボット実行 — 完全オンデバイス、クラウド不要。
> 実演コマンド: 立って / 座って / 伏せて（日本語音声入力、Unitree Go2 実機）

### Claudia ターミナル操作

[![Terminal Demo](https://img.youtube.com/vi/gdyy1p4hkY4/sddefault.jpg)](https://youtu.be/gdyy1p4hkY4)

> ランチャー → キーボードモード → 日本語テキストコマンド → ロボット実行。
> 実演コマンド: 立って / 座って / 伏せて / 挨拶して（テキスト入力、Unitree Go2 実機）

### Claudia 高度な動作

[![Advanced Actions Demo](https://img.youtube.com/vi/-Sj_ffcLJzA/sddefault.jpg)](https://youtu.be/-Sj_ffcLJzA)

> 高リスクモード切替 (c) → ストレッチ / 前ジャンプ / 前宙 — SafetyCompiler ゲート制御のデモ。
> ターミナルモードで実演 — 音声とターミナルは同一の LLM パイプラインを共有し、入力方法のみが異なります。

### Claudia 完全ワイヤレスデモ

[![Fully Wireless Demo](https://img.youtube.com/vi/Pi2ocDLCMOk/sddefault.jpg)](https://youtu.be/Pi2ocDLCMOk)

> 完全ワイヤレス音声パイプライン：DJI MIC 2 → ASR → ローカル LLM → ロボット実行 — 有線接続ゼロ（Wi-Fi + ワイヤレスマイク）。

<details>
<summary><strong>Go2 動作プレビュー</strong> — Unitree公式映像（クリックで展開）</summary>
<br>
<p align="center">
  <a href="https://youtu.be/8gaULsglOQE"><img src="docs/images/go2-dance.gif" alt="ダンス" width="260"></a>
  <a href="https://youtu.be/DXRojz4N8K8"><img src="docs/images/go2-flip.gif" alt="バク転" width="260"></a>
  <a href="https://youtu.be/8ReuPIKcydw"><img src="docs/images/go2-run.gif" alt="走行" width="260"></a>
</p>
<p align="center">
  <a href="https://youtu.be/F1JtFksc_k0"><img src="docs/images/go2-avoid.gif" alt="障害物回避" width="260"></a>
  <a href="https://youtu.be/rjVfRanqUC4"><img src="docs/images/go2-lidar.gif" alt="4D LiDAR" width="260"></a>
</p>

| 動画 | リンク |
|------|--------|
| ダンスパフォーマンス | [YouTube](https://youtu.be/8gaULsglOQE) |
| フロントフリップ | [YouTube](https://youtu.be/DXRojz4N8K8) |
| 高速走行 | [YouTube](https://youtu.be/8ReuPIKcydw) |
| 障害物回避 | [YouTube](https://youtu.be/F1JtFksc_k0) |
| 4D LiDAR マッピング | [YouTube](https://youtu.be/rjVfRanqUC4) |
| 性能概要 | [YouTube](https://youtu.be/N6burwXML70) |
| バッテリー＆持久力 | [YouTube](https://youtu.be/klw6Hvu4EzI) |
| モバイルアプリ操作 | [YouTube](https://youtu.be/IM2MKeuHtu4) |

<sub>全映像は <a href="https://www.unitree.com/go2">Unitree Robotics 公式サイト</a>より。教育・研究デモ目的で使用。</sub>
</details>

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
- **18の検証済み動作**: 基本姿勢8 + パフォーマンス7 + 高度3（[対応動作](#対応動作)参照）
- **リアルタイム制御**: 1ms（キャッシュ）〜約5秒（Jetson上LLM推論）の応答時間
- **状態認識シーケンス**: 動作依存関係の自動解決
- **グレースフルフォールバック**: 実機ハードウェア → モックシミュレーション

### 多言語対話
- **日本語優先**: 日本語会話に最適化（ホットキャッシュ、応答検証、ASR）
- **多言語対応アーキテクチャ**: Qwen2.5-7B は日本語・中国語・英語をネイティブに理解
- **ASR言語切替**: Whisper の言語パラメータをセッションごとに設定可能
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
| ネットワーク | Wi-Fi ([TP-Link Archer T2UB Nano](https://www.tp-link.com/jp/home-networking/adapter/archer-t2ub-nano/)) またはイーサネット (`192.168.123.x`) |

### インストール

```bash
git clone https://github.com/ShunmeiCho/Claudia.git
cd claudia
pip install -e .

# Ollamaのインストール（未導入の場合）
curl -fsSL https://ollama.ai/install.sh | sh

# Claudia頭脳モデルの作成
ollama create claudia-7b:v2.0 -f models/ClaudiaIntelligent_7B_v2.0

# 環境設定
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/path/to/unitree_sdk2_python:$PYTHONPATH
```

### 起動

```bash
# インタラクティブランチャー（推奨）
./start_production_brain.sh
```

ランチャーはモード選択メニューを表示します：

```
==================================
🤖 Claudia Production Brain Launcher
==================================

🔧 網路配置:
   本機IP: 192.168.123.18
   機器人IP: 192.168.123.161 (Go2)
   DDS配置: eth0
   Python: /usr/bin/python3 (Python 3.8.10)

运行モード選択:
  1) キーボード + シミュレーション
  2) キーボード + 実機
  3) 語音 + シミュレーション
  4) 語音 + 実機
  c) 設定パネル
  t) 後台モード (tmux)
```

| オプション | モード | 説明 |
|:---:|--------|------|
| **1** | キーボード + シミュレーション | REPLでコマンド入力、動作はログのみ（実機未接続可）。開発・テスト用 |
| **2** | キーボード + 実機 | コマンド入力、DDS経由で実機Go2に送信。接続確認プロンプトあり |
| **3** | 語音 + シミュレーション | USBマイク → ASR → LLMパイプライン、動作はシミュレーション。音声パイプラインのテスト用 |
| **4** | 語音 + 実機 | フルパイプライン：音声入力 → ASR → LLM → SafetyCompiler → 実機ロボット実行 |
| **c** | 設定パネル | 起動前に各種設定を変更（下記参照） |
| **t** | 後台モード (tmux) | SSH切断に耐えるtmuxセッションで起動 |

#### 設定パネル

オプション `c` で設定パネルを開き、実行時設定を変更できます：

| 設定項目 | デフォルト | 説明 |
|----------|-----------|------|
| 唤醒詞 | OFF | 「クラちゃん」唤醒詞ゲーティングの有効/無効 |
| 起動アニメーション | OFF | 起動時にRecoveryStand + Hello動作を実行 |
| LLMモデル | `claudia-7b:v2.0` | 利用可能なOllamaモデルから選択 |
| ルーティングモード | `dual` | `dual`（Actionチャネル）/ `legacy`（7Bのみ）/ `shadow`（A/B比較） |
| ASRモデル | `base` | `base`（~2-3秒）/ `small`（~5-8秒）/ `medium`（~10-15秒） |
| 高リスク動作 | OFF | FrontFlip、FrontJump、FrontPounceの許可 |
| マイクデバイス | `auto` | USBマイク自動検出、または手動指定（例：`hw:2,0`） |

#### 直接起動（メニュースキップ）

```bash
./start_production_brain.sh --voice          # 音声 + シミュレーション
./start_production_brain.sh --voice-hw       # 音声 + 実機

# Pythonスクリプト直接実行:
python3 production_commander.py              # キーボード + シミュレーション
python3 production_commander.py --hardware   # キーボード + 実機
python3 voice_commander.py                   # 音声 + シミュレーション
python3 voice_commander.py --hardware        # 音声 + 実機
python3 voice_commander.py --asr-mock        # 音声 + モックASR（マイク不要）
python3 voice_commander.py --daemon          # 後台モード（tmux用）
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
| 1005 | StandDown | 伏せる | 趴下 | Stand Down | Yes |
| 1006 | RecoveryStand | 回復 | 恢复站立 | Recovery | - |
| 1009 | Sit | 座る | 坐下 | Sit | Yes |
| 1010 | RiseSit | 起き上がる | 起立 | Rise Sit | - |

### パフォーマンス（7動作）

| APIコード | メソッド | 日本語 | 中国語 | 英語 | 起立必要 |
|:-:|--------|--------|---------|---------|:-:|
| 1016 | Hello | 挨拶 | 打招呼 | Hello | Yes |
| 1017 | Stretch | 伸び | 伸懒腰 | Stretch | Yes |
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

### ハードウェア構成

<p align="center">
  <img src="docs/images/go2-hardware.jpg" alt="Unitree Go2 ハードウェア構成" width="800">
  <br>
  <sub>画像提供: <a href="https://www.unitree.com/go2">Unitree Robotics</a></sub>
</p>

Claudia は **Unitree Go2** 四足歩行ロボット上で、外付け **NVIDIA Jetson Orin NX** を AI 計算モジュールとして動作します:

| コンポーネント | 仕様 |
|----------------|------|
| **ロボット** | Unitree Go2（12自由度、8000mAhバッテリー、4D LiDAR L2） |
| **AIモジュール** | NVIDIA Jetson Orin NX 16GB（Developer Kit） |
| **GPU** | 1024コア NVIDIA Ampere、612MHz max、CUDA 11.4 |
| **CPU** | 8コア Arm Cortex-A78AE（aarch64） |
| **RAM / ストレージ** | 16GB LPDDR5 / 512GB NVMe SSD |
| **OS** | Ubuntu 20.04、L4T R35.3.1（JetPack 5.1.1）、Python 3.8.10 |
| **LLMランタイム** | Ollama + Qwen2.5-7B（Q4_K_M、VRAM ~4.7GB） |
| **ASR** | faster-whisper base（CPU int8、~1.6秒/発話） |
| **マイク** | DJI MIC 2（ワイヤレス）または Audio-Technica AT2020USB-XP（USB有線） |
| **ネットワーク** | Wi-Fi ([TP-Link Archer T2UB Nano](https://www.tp-link.com/jp/home-networking/adapter/archer-t2ub-nano/)、`192.168.123.x`) — イーサネット不要 |

### 完全ワイヤレス構成

最新のアップデートにより、Claudia は Jetson への**有線接続ゼロ**で動作可能になりました。従来の有線周辺機器はすべて無線化されています：

| 接続 | 従来（有線） | 現在（ワイヤレス） |
|------|-------------|---------------------|
| **ネットワーク** | Ethernet (eth0) | [TP-Link Archer T2UB Nano](https://www.tp-link.com/jp/home-networking/adapter/archer-t2ub-nano/) (AC600, USB Wi-Fi + Bluetooth 4.2) |
| **マイク** | Audio-Technica AT2020USB-XP (USB) | [DJI MIC 2](https://www.dji.com/mic-2)（ワイヤレス、USB-Cレシーバー） |

**Wi-Fi設定リファレンス**: [Jetson Orin NX Wi-Fi設定ガイド (Qiita)](https://qiita.com/todateman/items/aea5b12f1456308ed981)

> TP-Link Archer T2UB Nano は USB 接続で Wi-Fi（`192.168.123.x` ロボットネットワーク用）と Bluetooth 4.2 を提供します。DJI MIC 2 は専用 USB-C レシーバーにより放送品質のワイヤレスオーディオを提供し、有線 USB マイクが不要になりました。

### コマンド処理パイプライン

```mermaid
flowchart TD
    Input["ユーザー入力 (JA / ZH / EN)"] --> Emergency{"1. 緊急バイパス<br/>~0ms"}
    Emergency -->|match| Stop["即座に停止"]
    Emergency -->|miss| Cache{"2. ホットキャッシュ<br/>80+件, ~1ms"}
    Cache -->|hit| Execute
    Cache -->|miss| Conv{"3. 会話検出"}
    Conv -->|"挨拶 / 質問"| TextOnly["テキスト応答<br/>(動作なし)"]
    Conv -->|command| LLM["4. LLM推論<br/>(Jetsonで3-5秒)"]
    LLM --> Safety["SafetyCompiler.compile()<br/>ホワイトリスト / バッテリー / 起立"]
    Safety --> Execute["動作実行"]
    Execute --> Robot["SportClient RPC<br/>via CycloneDDS"]
```

> **ルーティングモード** (`BRAIN_ROUTER_MODE`): `dual`（デフォルト、アクション専用モデル、~30トークン）| `legacy`（7Bフル応答）| `shadow`（A/B比較ログ）

<details>
<summary><strong>モジュール概要</strong>（クリックで展開）</summary>

| モジュール | 責務 |
|--------|------|
| `brain/production_brain.py` | コアパイプライン: キャッシュ→ルーター→安全→実行 |
| `brain/channel_router.py` | デュアルチャネルLLMルーター（legacy/dual/shadowモード） |
| `brain/action_registry.py` | 全動作定義の唯一の真実源 |
| `brain/safety_compiler.py` | 統一安全パイプライン（バッテリー、起立、ホワイトリスト） |
| `brain/audit_logger.py` | 構造化監査ログ (`logs/audit/`) |
| `brain/mock_sport_client.py` | テスト用SportClientシミュレーター |
| `robot_controller/system_state_monitor.py` | ROS2ベースのバッテリー/姿勢監視（5Hz） |
| `robot_controller/unified_led_controller.py` | LEDモードAPI（思考中/成功/エラー/リスニング） |
| `production_commander.py` | キーボードREPLエントリーポイント |
| `voice_commander.py` | 音声モードエントリーポイント: ASR + AudioCapture + ASRBridge |
| `audio/asr_service/` | ASRサーバー: faster-whisper + silero-vad + UDS |

</details>

---

## 音声認識 (ASR)

> ステータス: **Phase 2稼働中** — Jetson上でUSBマイクを使用した完全な音声パイプラインが稼働。

### 音声パイプライン

```
ワイヤレスマイク (DJI MIC 2) または USBマイク (AT2020USB-XP), カード自動検出
  │ arecordサブプロセス → リサンプル → 16kHz 960byteフレーム
  v
AudioCapture ──→ /tmp/claudia_audio.sock ──→ ASR Server（サブプロセス）
                                                ├── silero-vad + 緊急検出
                                                ├── faster-whisper base (ja, beam=1, CPU int8)
                                                v
ASRBridge ←── /tmp/claudia_asr_result.sock ←─── JSON Lines
  ├── emergency → キューフラッシュ + クールダウン → brain呼び出し（ロックバイパス）
  ├── transcript → 信頼度 ≥0.35 フィルター → 重複排除 → Queue(3)
  └── command worker → brain.process_and_execute(text)
```

### プロセス耐障害性

- **SIGHUP 処理**: 両コマンダーが SIGHUP を無視 — SSH 切断時にプロセスが終了しない
- **ASR 自動再起動**: ASR サブプロセスがクラッシュした場合、VoiceCommander が完全なパイプラインを自動再構築（Bridge → Capture → ASR → 再構築）。最大 3 回まで試行後、degraded モード（キーボードのみ）に移行
- **tmux 統合**: `start_production_brain.sh` のオプション `t` で tmux セッション内で起動。環境変数を完全転送し、SSH 切断に耐える
- **Ollama GPU クリーンアップ**: シャットダウン時にモデルを GPU メモリから即座にアンロード（`keep_alive=0`）。30 分間の無駄な VRAM 占有を防止

### ASR環境変数オーバーライド

| 変数 | デフォルト | 選択肢 |
|------|-----------|--------|
| `CLAUDIA_ASR_MODEL` | `base` | `base` / `small` / `medium` |
| `CLAUDIA_ASR_BEAM_SIZE` | `1`（貪欲デコード） | `1` / `3`+（ビームサーチ） |
| `CLAUDIA_ASR_DEVICE` | `cpu` | `cpu` / `cuda` |
| `CLAUDIA_WAKE_WORD_ENABLED` | `0`（無効） | `0` / `1` |
| `CLAUDIA_WAKE_WORD_TIMEOUT` | `5`（秒） | standalone 唤醒詞後の監聴ウィンドウ |

### カナ正規化

- **KANA_ALIASESパイプライン**: ホットキャッシュ層に統合済み。一般的なASRカナ出力を漢字表記に正規化（例：`おすわり` → `お座り`、`おて` → `お手`、`はーと` → `ハート`）。日本語音声コマンドにおけるASRミスマッチの最大の原因を排除。
- **日本語サフィックス除去**: 丁寧語サフィックス（です/ます/ください）はホットキャッシュマッチング時に自動除去（例：`かわいいです` → `かわいい`）。
- **緊急コマンドカナバリアント**: `EMERGENCY_COMMANDS`辞書にカナのみのバリアント（`とまれ`、`とめて`、`ていし`）を含み、不完全なASR転写でも緊急停止が確実に動作。

---

## テキスト読み上げ (TTS)

> ステータス: **エコーゲーティング実装済み** — ASRサーバーにエコーゲーティングを実装済み。TTSプロバイダー統合はPR3で予定（VOICEVOX / Google TTS）。

応答は現在REPLにテキストとして表示されます。ロボットは日本語で応答（`_sanitize_response()`による検証）。頭脳はTTSに**一切関与しない** — `ProductionBrain`はテキスト＋アクションコードを生成するのみで、TTS再生はコマンダー層が管理。

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
| PR3 | ASR/TTS統合 | ASR完了、TTS未実装 |
| P2 | パラメーター化動作（Move, Euler, SpeedLevel） | 計画中 |
| P2 | 3Bアクションチャネル A/Bテスト | 計画中 |

---

## 謝辞

Claudiaは以下のオープンソースプロジェクトの上に構築されています：

- [Qwen2.5](https://github.com/QwenLM/Qwen2.5) — ロボット頭脳を動かす多言語LLM
- [Ollama](https://ollama.ai/) — ローカルLLM推論ランタイム
- [Unitree SDK2](https://github.com/unitreerobotics) — Go2ロボット制御SDK
- [faster-whisper](https://github.com/SYSTRAN/faster-whisper) — CTranslate2ベースのASRエンジン
- [silero-vad](https://github.com/snakers4/silero-vad) — 音声活動検出
- [ROS 2 Foxy](https://docs.ros.org/en/foxy/) + [CycloneDDS](https://cyclonedds.io/) — ロボットミドルウェア

---

## ライセンス

MIT License — 詳細は[LICENSE](LICENSE)を参照。
