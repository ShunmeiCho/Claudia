#!/bin/bash
# Track B部署脚本：拉取/创建新模型并快速测试

set -e

cd $HOME/claudia

echo "=================================="
echo "🔬 Track B: 新模型部署"
echo "=================================="
echo ""

# 1. 确保基础模型存在
echo "📦 检查基础模型..."
if ! ollama list | grep -q "qwen2.5:7b"; then
    echo "⬇️  拉取qwen2.5:7b (约4.7GB, INT4量化)..."
    ollama pull qwen2.5:7b
else
    echo "   ✅ qwen2.5:7b已存在"
fi

# 2. 创建新Modelfile模型
echo ""
echo "🔧 创建claudia-intelligent-7b:v1..."
if ollama list | grep -q "claudia-intelligent-7b:v1"; then
    echo "   ⚠️  模型已存在，删除旧版本..."
    ollama rm claudia-intelligent-7b:v1 2>/dev/null || true
fi

ollama create claudia-intelligent-7b:v1 -f ClaudiaIntelligent_Qwen7B

echo "   ✅ 模型创建完成"

# 3. 快速测试（修复：使用Python库替代CLI --format json）
echo ""
echo "🧪 快速测试新模型..."
echo ""

python3 - <<'PYEOF'
import ollama
import json

test_commands = [
    "座って",
    "立って",
    "可愛い",
    "stop"
]

print("使用Python ollama库测试 (format='json')...\n")
for cmd in test_commands:
    print(f"测试: {cmd}")
    try:
        response = ollama.chat(
            model="claudia-intelligent-7b:v1",
            messages=[{'role': 'user', 'content': cmd}],
            format='json',
            options={'temperature': 0.1, 'num_predict': 30}
        )
        content = response['message']['content']
        data = json.loads(content)
        api_code = data.get('api_code') or data.get('a')
        print(f"  → api_code: {api_code}")
        print(f"  → raw: {content[:80]}...")
    except Exception as e:
        print(f"  ❌ 错误: {e}")
    print("")
PYEOF

echo "=================================="
echo "✅ Track B部署完成！"
echo ""
echo "下一步："
echo "1. 运行完整评测: python3 test/test_model_comparison.py"
echo "2. 对比结果查看: test/model_comparison_results.json"
echo "=================================="
