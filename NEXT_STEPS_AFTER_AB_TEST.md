# 🚦 A/B测试完成 - 立即行动清单

**测试完成时间**: 2025-11-14
**决策**: ❌ 不部署Track B，执行Track A加固方案

---

## 🔴 Critical - 本周必须完成

### 1. 确保Track B未部署到生产环境

```bash
# 检查当前配置
echo $BRAIN_MODEL_3B
echo $AB_TEST_RATIO

# 如果有任何Track B配置，立即回退
export BRAIN_MODEL_3B="claudia-go2-3b:v11.2"
export BRAIN_MODEL_7B="claudia-go2-7b:v7"
export AB_TEST_RATIO="0.0"

# 重启生产服务
./start_production_brain.sh
```

### 2. 修复Critical安全问题 - 电量硬性检查 ⚠️

**问题**: 目前8%电量时仍允许前转(1030)等高能动作，可能导致机器人动作中途断电摔倒。

**修复位置**: [src/claudia/brain/production_brain.py](src/claudia/brain/production_brain.py)

**需要添加的代码**:

```python
# 在ProductionBrain类中添加新方法（建议在第550行左右，_build_enhanced_prompt之后）
def _enforce_battery_safety(self, api_code: Optional[int],
                           battery_level: float) -> Optional[int]:
    """
    代码层强制电量安全检查（不依赖LLM判断）

    Args:
        api_code: 请求的动作API码
        battery_level: 当前电量（0.0-1.0或0-100）

    Returns:
        安全的API码，如果需要拒绝则返回None
    """
    # 标准化电量值到0.0-1.0
    if battery_level > 1.0:
        battery_level = battery_level / 100.0

    HIGH_ENERGY_ACTIONS = [1030, 1031, 1032]  # FrontFlip, FrontJump, FrontPounce
    MEDIUM_ENERGY_ACTIONS = [1022, 1023, 1029]  # Dance1, Dance2, Scrape

    # 极低电量（≤10%）: 只允许Stop(1003)和Sit(1009)
    if battery_level <= 0.10:
        if api_code not in [1003, 1009, None]:
            logger.warning(
                f"Battery safety: Rejected action {api_code} at {battery_level*100:.1f}% battery"
            )
            return None  # 强制拒绝
        return api_code

    # 低电量（≤20%）: 禁止高能动作
    if battery_level <= 0.20:
        if api_code in HIGH_ENERGY_ACTIONS:
            logger.warning(
                f"Battery safety: Rejected high-energy action {api_code} at {battery_level*100:.1f}% battery"
            )
            return None  # 强制拒绝高能动作
        return api_code

    # 中等电量（≤30%）: 高能动作降级为中能动作
    if battery_level <= 0.30:
        if api_code in HIGH_ENERGY_ACTIONS:
            logger.info(
                f"Battery safety: Downgraded action {api_code} to Dance at {battery_level*100:.1f}% battery"
            )
            return 1023  # 降级到Dance
        return api_code

    return api_code  # 正常电量，无限制


# 在execute_command方法中调用（约第730行，LLM调用之后，执行之前）
async def execute_command(self, command: str) -> BrainOutput:
    """执行用户命令"""
    try:
        # ... 现有代码 ...

        # LLM调用
        llm_result = await asyncio.wait_for(
            asyncio.to_thread(self._sync_ollama_call, ...),
            timeout=30.0
        )

        # 解析结果
        api_code = llm_result.get('api_code') or llm_result.get('a')

        # ✅ 添加电量安全检查（在SafetyValidator之后）
        current_state = self.state_monitor.get_current_state()
        if current_state:
            battery_level = current_state.battery_level
            safe_api_code = self._enforce_battery_safety(api_code, battery_level)

            if safe_api_code != api_code:
                # 动作被拒绝或降级
                if safe_api_code is None:
                    return BrainOutput(
                        response=f"電池残量が不足しています ({battery_level*100:.0f}%)",
                        api_code=None,
                        confidence=1.0,
                        reasoning=f"Battery safety: Rejected {api_code} at low battery"
                    )
                else:
                    # 动作被降级
                    api_code = safe_api_code
                    response = f"電池を節約するため、動作を調整します ({battery_level*100:.0f}%)"

        # 继续正常执行流程...

    except Exception as e:
        logger.error(f"Execute command error: {e}")
        return BrainOutput(response="エラーが発生しました", api_code=None)
```

**验证步骤**:

1. 添加代码后运行单元测试:
   ```bash
   python3 -m pytest test/unit/test_battery_safety.py -v
   ```

2. 创建测试用例 `test/unit/test_battery_safety.py`:
   ```python
   import pytest
   from src.claudia.brain.production_brain import ProductionBrain

   def test_battery_safety_enforcement():
       brain = ProductionBrain(simulation_mode=True)

       # 极低电量（8%）应拒绝高能动作
       result = brain._enforce_battery_safety(1030, 0.08)
       assert result is None, "Should reject FrontFlip at 8% battery"

       # 极低电量应允许Stop/Sit
       result = brain._enforce_battery_safety(1003, 0.08)
       assert result == 1003, "Should allow Stop at 8% battery"

       # 低电量（15%）应拒绝高能动作
       result = brain._enforce_battery_safety(1031, 0.15)
       assert result is None, "Should reject FrontJump at 15% battery"

       # 中等电量（25%）应降级高能动作
       result = brain._enforce_battery_safety(1030, 0.25)
       assert result == 1023, "Should downgrade FrontFlip to Dance at 25% battery"

       # 正常电量（60%）应无限制
       result = brain._enforce_battery_safety(1030, 0.60)
       assert result == 1030, "Should allow FrontFlip at 60% battery"
   ```

3. 硬件实测（需要Go2机器人）:
   ```bash
   # 设置模拟低电量进行测试
   # 在production_brain.py临时修改:
   # current_state.battery_level = 0.08  # 强制模拟8%电量

   python3 production_commander.py --hardware
   # 输入: "前転"
   # 期望: 拒绝并提示"電池残量が不足しています"
   ```

---

## 🟡 Medium - 下周优化

### 3. 优化Track A的Few-shot示例

**目标**: 提升Track A准确率从64%到70%+

**需要添加的示例** (在Modelfile中):

```
Q: かっこいい動作して
A: {"r":"前転します","a":1030}

Q: お疲れ様
A: {"r":"座ります","a":1009}

Q: 楽しく遊んで
A: {"r":"踊ります","a":1023}

Q: [STATE] posture:sitting, battery:85%, space:normal
   ダンス
A: {"r":"立ってから踊ります","a":1023,"seq":[1004,1023]}
```

**操作步骤**:

1. 编辑现有Modelfile: `claudia-go2-3b-v11.2.Modelfile`
2. 在SYSTEM prompt的Few-shot区域添加上述4个示例
3. 重新创建模型:
   ```bash
   ollama create claudia-go2-3b:v11.3 -f claudia-go2-3b-v11.2.Modelfile
   ```
4. 测试新版本:
   ```bash
   python3 test_ab_quick.py  # 修改为测试v11.3
   # 期望准确率: ≥70%
   ```

### 4. 增强SafetyValidator

**新增功能**:

1. **空间检测**: 检查是否有足够空间执行Flip/Jump/Pounce
   ```python
   # 在safety_validator.py中添加
   REQUIRE_LARGE_SPACE = [1030, 1031, 1032]

   if api_code in REQUIRE_LARGE_SPACE:
       if state.space_status == "narrow":  # 需要在SystemStateMonitor添加空间检测
           return SafetyCheckResult(
               is_safe=False,
               reason="空間が狭すぎます。広い場所で試してください。"
           )
   ```

2. **连续高能动作限制**: 防止过度消耗电量
   ```python
   # 记录最近N次动作，如果连续3次高能动作则拒绝
   if len(self.recent_high_energy_actions) >= 3:
       if api_code in HIGH_ENERGY_ACTIONS:
           return SafetyCheckResult(
               is_safe=False,
               reason="休憩が必要です。少し待ってください。"
           )
   ```

3. **日志记录所有安全拦截事件**:
   ```python
   # 在每次拦截时写入audit log
   audit_logger.log_safety_rejection(
       api_code=api_code,
       battery=battery_level,
       posture=posture,
       reason=reason
   )
   ```

---

## 🟢 Low - 中期规划（本月）

### 5. 收集真实用户数据

**目标**: 为未来优化收集真实交互数据

**实施**:

1. 在audit_logger.py中记录所有用户指令
2. 每周统计高频失败场景
3. 识别语义理解gap

**数据格式**:
```json
{
  "timestamp": "2025-11-14T10:30:00",
  "user_command": "かっこいい",
  "llm_output": {"a": 1036, "r": "ハートします"},
  "user_feedback": "wrong_action",  // 需要添加反馈机制
  "expected_action": 1030
}
```

### 6. 评估LoRA微调可行性

**调研内容**:

1. Qwen2.5-1.5B在Jetson Orin NX上的推理延迟基准
2. 准备100+条标注训练数据
3. 评估LoRA微调成本（时间、GPU资源）

**决策标准**:
- 如果1.5B推理延迟≤1500ms → 启动LoRA微调项目
- 否则继续优化Track A

---

## 📊 已完成的工作

- [x] 部署Track B 7B模型
- [x] 部署Track B 3B模型
- [x] 运行完整25用例A/B测试
- [x] 修复测试脚本bug (sequence=None处理)
- [x] 创建完整技术报告 (docs/AB_TEST_FINAL_REPORT.md)
- [x] 创建执行摘要 (AB_TEST_EXECUTIVE_SUMMARY.md)
- [x] 归档测试日志 (test/ab_test_results/)
- [x] 更新Track B使用指南添加警告

---

## 📂 相关文档

| 文档 | 用途 |
|------|------|
| [AB_TEST_EXECUTIVE_SUMMARY.md](AB_TEST_EXECUTIVE_SUMMARY.md) | 一页纸执行摘要 |
| [docs/AB_TEST_FINAL_REPORT.md](docs/AB_TEST_FINAL_REPORT.md) | 完整技术分析报告 |
| [TRACK_B_USAGE_GUIDE.md](TRACK_B_USAGE_GUIDE.md) | Track B环境变量配置（已标记不建议使用） |
| [test/ab_test_results/](test/ab_test_results/) | 原始测试日志 |

---

## ✅ 验收标准

完成上述Critical和Medium优化后，系统应满足:

1. **安全性**: ✅ 10%以下电量拒绝所有高能动作
2. **准确率**: ✅ ≥70% (目标从64%提升)
3. **延迟**: ✅ ≤3000ms (维持Track A水平)
4. **稳定性**: ✅ JSON合规率100%
5. **可维护性**: ✅ 代码清晰，有单元测试覆盖

---

**优先级排序**: Critical > Medium > Low
**建议执行顺序**: 1 → 2 → 3 → 4 → 5 → 6

**预计总工作量**: 1-2周（Critical + Medium）
