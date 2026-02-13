# v12.1-simple 快速测试指南

**目的**: 验证边缘案例修复 ("godee"/"pong" 问题已解决)

---

## 前置条件检查

```bash
# 1. 确认v12.1-simple模型已创建
ollama list | grep v12.1-simple
# 期望输出：claudia-go2-7b:v12.1-simple

# 2. 如果不存在，创建模型
ollama create claudia-go2-7b:v12.1-simple -f models/ClaudiaIntelligent_7B_v2.0

# 3. 确认ProductionBrain配置
grep "v12.1-simple" src/claudia/brain/production_brain.py
# 期望输出：self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-go2-7b:v12.1-simple")
```

---

## 测试1: Ollama直接测试（推荐）

**优点**: 直接测试模型输出，无其他干扰

```bash
# 边缘案例（之前失败，现在应修复）
echo "今日はいい天気ですね" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"そうですね","a":null}
# ❌ v12-simple输出: {"r":" godee"}

echo "ちんちん" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"お辞儀します","a":1016}
# ❌ v12-simple输出: {"r":" pong"}

echo "おはよう" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"おはようございます","a":null}

# 核心功能（确保未破坏）
echo "可愛いね" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"ありがとうございます","a":1036}

echo "立ってそしてダンス" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"立ってからダンスします","s":[1004,1023]}

echo "疲れた" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"休みますね","a":1009}

echo "座って" | ollama run claudia-go2-7b:v12.1-simple
# ✅ 期望: {"r":"座ります","a":1009}
```

---

## 测试2: ProductionBrain交互测试

**优点**: 测试完整系统，包括代码级防护

```bash
# 启动生产指挥官
./start_production_brain.sh
# 选择: 1) Simulation mode (安全测试)

# 在交互界面输入以下命令：
```

### 边缘案例测试

```
Claudia> 今日はいい天気ですね
✅ 期望回复包含日语，无 "godee"
✅ 期望 api_code: null (闲聊无动作)

Claudia> ちんちん
✅ 期望回复包含日语，无 "pong"
✅ 期望 api_code: 1016 (鞠躬) 或 null

Claudia> おはよう
✅ 期望回复: "おはようございます" 或类似
✅ 期望 api_code: null

Claudia> よくわからない言葉xyzabc
✅ 期望回复: "すみません、よく分かりません"
✅ 期望 api_code: null
```

### 核心功能测试

```
Claudia> 可愛いね
✅ 期望: ハート动作 (1036)

Claudia> 立ってそしてダンス
✅ 期望: 序列 [1004, 1023]

Claudia> 疲れた
✅ 期望: 座る (1009)

Claudia> 座って
✅ 期望: 座る (1009)
```

---

## 测试3: 代码级单元测试

**优点**: 验证 `_sanitize_response()` 函数逻辑

```bash
python3 << 'EOF'
from src.claudia.brain.production_brain import ProductionBrain

brain = ProductionBrain(use_real_hardware=False)

# 测试无效输入应返回默认回复
test_cases = [
    (' godee', 'すみません、よく分かりません'),
    (' pong', 'すみません、よく分かりません'),
    ('', 'すみません、よく分かりません'),
    ('   ', 'すみません、よく分かりません'),
    ('ok', 'すみません、よく分かりません'),
    ('hello', 'すみません、よく分かりません'),
]

print("测试 _sanitize_response() 函数:")
all_pass = True
for input_str, expected in test_cases:
    result = brain._sanitize_response(input_str)
    status = "✅" if result == expected else "❌"
    print(f"{status} '{input_str}' → '{result}'")
    all_pass = all_pass and (result == expected)

# 测试有效日语应保留
valid_cases = [
    'ありがとうございます',
    'こんにちは',
    '座ります',
    'そうですね',
]

for input_str in valid_cases:
    result = brain._sanitize_response(input_str)
    status = "✅" if result == input_str else "❌"
    print(f"{status} '{input_str}' → '{result}' (应保留)")
    all_pass = all_pass and (result == input_str)

if all_pass:
    print("\n🎉 所有单元测试通过！")
else:
    print("\n⚠️ 部分测试失败")
EOF
```

---

## 测试4: 审计日志检查

**优点**: 监控实际使用中的输出质量

```bash
# 使用一段时间后，检查审计日志
tail -100 logs/$(date '+%Y%m')/audit_*.jsonl | jq '.llm_output.response'

# 检查是否存在无意义输出
grep -E 'godee|pong' logs/$(date '+%Y%m')/audit_*.jsonl
# ✅ 期望: 无结果（找不到godee/pong）

# 统计非日语回复（应该只有 "すみません、よく分かりません"）
tail -100 logs/$(date '+%Y%m')/audit_*.jsonl | \
  jq -r '.llm_output.response' | \
  grep -v '[\u3040-\u309f\u30a0-\u30ff\u4e00-\u9faf]'
# ✅ 期望: 无结果或只有极少数默认回复
```

---

## 预期输出对比

### 边缘案例修复对比

| 输入 | v12-simple ❌ | v12.1-simple ✅ |
|------|--------------|----------------|
| "今日はいい天気ですね" | `{"r":" godee"}` | `{"r":"そうですね","a":null}` |
| "ちんちん" | `{"r":" pong"}` | `{"r":"お辞儀します","a":1016}` |
| "おはよう" | 未知 | `{"r":"おはようございます","a":null}` |
| 无意义输入 | 可能异常 | `{"r":"すみません、よく分かりません","a":null}` |

### 核心功能保持不变

| 输入 | 期望输出 | v12.1-simple |
|------|----------|-------------|
| "可愛いね" | `{"r":"ありがとうございます","a":1036}` | ✅ 保持 |
| "立ってそしてダンス" | `{"r":"立ってからダンスします","s":[1004,1023]}` | ✅ 保持 |
| "疲れた" | `{"r":"休みますね","a":1009}` | ✅ 保持 |
| "座って" | `{"r":"座ります","a":1009}` | ✅ 保持 |

---

## 故障排查

### 问题1: 模型不存在

```bash
# 症状
$ ollama run claudia-go2-7b:v12.1-simple
Error: model not found

# 解决
$ ollama create claudia-go2-7b:v12.1-simple -f models/ClaudiaIntelligent_7B_v2.0
```

### 问题2: 仍然输出 "godee"/"pong"

```bash
# 症状
$ echo "今日はいい天気ですね" | ollama run claudia-go2-7b:v12.1-simple
{"r":" godee"}

# 诊断
1. 确认使用的是v12.1-simple模型，不是v12-simple
   ollama list | grep claudia

2. 确认Modelfile已更新
   grep "Rule 6" models/ClaudiaIntelligent_7B_v2.0
   grep "今日はいい天気ですね" models/ClaudiaIntelligent_7B_v2.0

3. 重新创建模型
   ollama rm claudia-go2-7b:v12.1-simple
   ollama create claudia-go2-7b:v12.1-simple -f models/ClaudiaIntelligent_7B_v2.0
```

### 问题3: ProductionBrain未使用v12.1

```bash
# 症状
日志显示: 🧠 📌 7B模型: claudia-go2-7b:v12-simple

# 诊断
grep "model_7b" src/claudia/brain/production_brain.py | head -5
# 应显示: self.model_7b = os.getenv("BRAIN_MODEL_7B", "claudia-go2-7b:v12.1-simple")

# 如果显示v12-simple，手动更新：
# 编辑 src/claudia/brain/production_brain.py Line 80
# 或设置环境变量：
export BRAIN_MODEL_7B=claudia-go2-7b:v12.1-simple
./start_production_brain.sh
```

---

## 成功标志

✅ **测试1通过**: Ollama直接测试所有案例输出日语，无"godee"/"pong"

✅ **测试2通过**: ProductionBrain交互测试边缘案例和核心功能正常

✅ **测试3通过**: `_sanitize_response()` 单元测试全部通过

✅ **测试4通过**: 审计日志无异常输出

---

## 后续行动

测试通过后：

1. ✅ **标记v12.1为生产版本**
   ```bash
   ollama tag claudia-go2-7b:v12.1-simple claudia-go2-7b:production
   ```

2. ✅ **删除旧版本（可选）**
   ```bash
   ollama rm claudia-go2-7b:v12-simple
   ```

3. ✅ **更新文档**
   - 在README.md中标记v12.1-simple为推荐版本

4. ⏳ **持续监控**
   - 每周检查审计日志
   - 收集新的边缘案例
   - 逐步扩展few-shot示例

---

**作者**: Claude Code
**创建日期**: 2025-11-18
**用途**: v12.1-simple边缘案例修复验证
