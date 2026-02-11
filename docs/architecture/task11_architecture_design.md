# Task 11: Preset Action Mapping & Execution Architecture

## 🎯 **总体架构设计**

### **系统目标**
将LLM解析的日语意图转换为Unitree Go2机器人的具体动作执行，实现自然语言到机器人控制的映射。

### **核心组件架构**
```
日语指令输入 → LLM意图解析 → 映射决策引擎 → SportClient执行 → 动作反馈
     ↓              ↓              ↓              ↓           ↓
  "お座り"     {intent:"sit"}  → SIT(1009)  → client.Sit() → LED反馈
```

---

## 🚀 **第一轨道：Go2 Sport API完整分析（0风险 - 立即执行）**

### **1.1 27个Go2 Sport API功能分级**

#### **🟢 高频基础动作 (6个)**
| API编号 | 函数名称 | 日语映射 | 安全等级 | 执行时间 | 优先级 |
|---------|----------|----------|----------|----------|---------|
| 1009 | SIT | お座り、座って | 安全 | 2-3秒 | 高 |
| 1004 | STANDUP | 立って、起立 | 安全 | 3-4秒 | 高 |
| 1016 | HELLO | お手、握手、挨拶 | 安全 | 3-4秒 | 高 |
| 1017 | STRETCH | 伸展、ストレッチ | 安全 | 4-5秒 | 高 |
| 1002 | BALANCESTAND | バランス、安定 | 安全 | 2-3秒 | 中 |
| 1001 | DAMP | 停止、ダンプ | 安全 | 1秒 | 紧急 |

#### **🟡 运动控制动作 (4个)**
| API编号 | 函数名称 | 日语映射 | 安全等级 | 执行时间 | 优先级 |
|---------|----------|----------|----------|----------|---------|
| 1008 | MOVE | 移动、歩いて | 中等 | 持续 | 高 |
| 1007 | EULER | 姿勢、角度 | 中等 | 2-3秒 | 中 |
| 1005 | STANDDOWN | 伏せ、下へ | 安全 | 3-4秒 | 中 |
| 1010 | RISESIT | 座りから立つ | 安全 | 3-4秒 | 中 |

#### **🔴 高级表演动作 (4个)**
| API编号 | 函数名称 | 日语映射 | 安全等级 | 执行时间 | 优先级 |
|---------|----------|----------|----------|----------|---------|
| 1022 | DANCE1 | ダンス、踊って | 中等 | 5-8秒 | 高 |
| 1023 | DANCE2 | サークル、回って | 中等 | 5-8秒 | 高 |
| 1024 | FRONTFLIP | フリップ、宙返り | 危险 | 3-5秒 | 低 |
| 1025 | FRONTJUMP | ジャンプ、跳んで | 危险 | 2-3秒 | 低 |

#### **⚡ 控制与恢复 (4个)**
| API编号 | 函数名称 | 日语映射 | 安全等级 | 执行时间 | 优先级 |
|---------|----------|----------|----------|----------|---------|
| 1003 | STOPMOVE | 移動停止 | 安全 | 即时 | 紧急 |
| 1006 | RECOVERYSTAND | 回復、立て直し | 安全 | 4-6秒 | 中 |
| 1026 | FRONTPOUNCE | 前扑、アタック | 危险 | 2-3秒 | 低 |

### **1.2 API安全分级与执行策略**

#### **🛡️ 安全分级定义**
- **安全 (Green)**: 可在任何环境安全执行
- **中等 (Yellow)**: 需要确认周围环境安全
- **危险 (Red)**: 仅在空旷安全区域执行
- **紧急 (Emergency)**: 立即执行，用于紧急停止

#### **🎯 执行优先级策略**
```python
执行优先级排序:
1. 紧急级 (DAMP, STOPMOVE) - 立即执行
2. 高优先级安全动作 (SIT, STANDUP, HELLO) - 优先处理  
3. 高优先级表演动作 (DANCE1, DANCE2) - 展示场景优先
4. 中等优先级 (运动控制) - 根据环境决策
5. 低优先级危险动作 - 需要特殊权限确认
```

---

## 🏗️ **第二轨道：LLM输出接口设计**

### **2.1 标准LLM输出格式定义**
```json
{
  "intent": "基础意图类型",
  "action": "具体动作名称", 
  "confidence": 0.85,
  "parameters": {
    "direction": "forward/backward/left/right",
    "speed": "slow/normal/fast",
    "duration": 5.0
  },
  "safety_check": true,
  "composite": false
}
```

### **2.2 意图分类映射表**
| LLM意图类型 | 目标API | 日语示例 | 安全验证 |
|-------------|---------|----------|----------|
| greet | HELLO(1016) | お手、挨拶して | ✓ |
| sit | SIT(1009) | お座り、座って | ✓ |
| stand | STANDUP(1004) | 立って、起立 | ✓ |
| dance | DANCE1(1022) | ダンス、踊って | ✓ |
| stretch | STRETCH(1017) | 伸展、ストレッチ | ✓ |
| move | MOVE(1008) | 歩いて、移動 | 环境检查 |
| stop | DAMP(1001) | 停止、止まって | 紧急 |

### **2.3 复合动作支持**
```json
{
  "intent": "composite",
  "sequence": [
    {"action": "sit", "wait": 3.0},
    {"action": "dance", "wait": 5.0}, 
    {"action": "hello", "wait": 2.0}
  ],
  "safety_check": true,
  "total_duration": 10.0
}
```

---

## ⚙️ **第三轨道：映射决策引擎架构**

### **3.1 核心映射引擎设计**
```python
class ActionMappingEngine:
    def __init__(self):
        self.api_registry = self._load_api_registry()
        self.safety_checker = SafetyChecker()
        self.execution_controller = ExecutionController()
    
    def map_intent_to_action(self, llm_output):
        """LLM输出 → SportClient API调用"""
        
        # 1. 解析LLM输出
        parsed = self.parse_llm_output(llm_output)
        
        # 2. 安全检查
        if not self.safety_checker.validate(parsed):
            return self.safe_fallback()
        
        # 3. 映射决策
        api_call = self.resolve_api_mapping(parsed)
        
        # 4. 执行控制
        return self.execution_controller.execute(api_call)
```

### **3.2 决策流程图**
```
LLM输出 → 格式验证 → 意图识别 → 安全检查 → API映射 → 执行控制
    ↓         ↓         ↓         ↓         ↓         ↓
  JSON    ✓有效     sit     ✓安全   SIT(1009)  client.Sit()
    ↓         ↓         ↓         ↓         ↓         ↓
 异常处理   格式错误   未知意图   危险动作   API错误   执行失败
    ↓         ↓         ↓         ↓         ↓         ↓  
  日志记录   默认动作   询问用户   权限确认   重试机制   错误恢复
```

---

## 🎮 **SportClient集成架构**

### **4.1 SportClient封装设计**
```python
class ClaudiaRobotController:
    def __init__(self):
        self.client = None
        self.initialized = False
        self.safety_mode = True
        
    def initialize(self):
        """按照官方示例初始化SportClient"""
        ChannelFactoryInitialize(0, "eth0")
        self.client = SportClient()
        self.client.SetTimeout(10.0)
        self.client.Init()
        self.initialized = True
        
    async def execute_action(self, api_code, parameters=None):
        """执行具体的机器人动作"""
        if not self.initialized:
            raise RuntimeError("SportClient未初始化")
            
        action_method = self.get_action_method(api_code)
        return await action_method(parameters)
```

### **4.2 API调用映射**
```python
API_MAPPING = {
    1001: ("damp", lambda client: client.Damp()),
    1002: ("balance_stand", lambda client: client.BalanceStand()),
    1003: ("stop_move", lambda client: client.StopMove()),
    1004: ("stand_up", lambda client: client.StandUp()),
    1005: ("stand_down", lambda client: client.StandDown()),
    1006: ("recovery_stand", lambda client: client.RecoveryStand()),
    1007: ("euler", lambda client, params: client.Euler(params)),
    1008: ("move", lambda client, params: client.Move(params)),
    1009: ("sit", lambda client: client.Sit()),
    1010: ("rise_sit", lambda client: client.RiseSit()),
    1016: ("hello", lambda client: client.Hello()),
    1017: ("stretch", lambda client: client.Stretch()),
    1022: ("dance1", lambda client: client.Dance1()),
    1023: ("dance2", lambda client: client.Dance2()),
    1024: ("front_flip", lambda client: client.FrontFlip()),
    1025: ("front_jump", lambda client: client.FrontJump()),
    1026: ("front_pounce", lambda client: client.FrontPounce()),
}
```

---

## 🛡️ **安全与错误处理机制**

### **5.1 多层安全检查**
```python
class SafetyChecker:
    def validate_intent(self, intent):
        """意图安全性验证"""
        dangerous_actions = ['front_flip', 'front_jump', 'front_pounce']
        return intent not in dangerous_actions or self.has_permission()
    
    def validate_environment(self):
        """环境安全性检查"""
        # 检查传感器数据，确认周围安全
        return self.check_obstacles() and self.check_space()
    
    def validate_robot_state(self):
        """机器人状态检查"""
        return self.check_battery() and self.check_temperature()
```

### **5.2 错误恢复策略**
```python
错误类型处理:
1. LLM输出格式错误 → 请求重新输入
2. 未知意图 → 提供可用动作列表
3. 安全检查失败 → 执行安全替代动作
4. API调用失败 → 重试机制(最多3次)
5. 机器人异常 → 紧急停止 + 恢复流程
```

---

## 📊 **Mock数据与测试策略**

### **6.1 LLM输出Mock数据**
```json
// 基础动作测试
{
  "intent": "sit",
  "confidence": 0.92,
  "parameters": {},
  "safety_check": true
}

// 复合动作测试  
{
  "intent": "composite",
  "sequence": [
    {"action": "sit", "wait": 3.0},
    {"action": "hello", "wait": 2.0}
  ],
  "safety_check": true
}

// 带参数动作测试
{
  "intent": "move", 
  "parameters": {
    "direction": "forward",
    "speed": "slow",
    "duration": 5.0
  },
  "safety_check": true
}
```

### **6.2 测试用例设计**
```python
TEST_CASES = [
    # 基础动作测试
    ("お座り", "sit", "SIT(1009)"),
    ("お手", "greet", "HELLO(1016)"),
    ("踊って", "dance", "DANCE1(1022)"),
    
    # 复合动作测试
    ("座ってから手を振って", "composite", ["SIT", "HELLO"]),
    
    # 异常情况测试
    ("不明な指令", "unknown", "error_handling"),
    ("危険な動作", "dangerous", "safety_block"),
]
```

---

## 🔧 **实施路径与里程碑**

### **Phase 1: 核心映射系统 (0风险)**
- [x] API分析与分级 ✓ 
- [ ] 映射引擎框架设计
- [ ] Mock数据格式定义
- [ ] 基础测试用例准备

### **Phase 2: SportClient集成 (低风险)**
- [ ] SportClient封装类实现
- [ ] API调用方法映射
- [ ] 基础动作测试验证

### **Phase 3: 安全与错误处理 (中风险)**
- [ ] 安全检查机制
- [ ] 错误恢复流程
- [ ] 异常处理测试

### **Phase 4: 复合动作支持 (中风险)**
- [ ] 序列化执行引擎
- [ ] 状态机设计
- [ ] 复合动作测试

### **Phase 5: 端到端集成 (待任务10修复)**
- [ ] 与任务10 LLM输出接口对接
- [ ] 完整流程测试
- [ ] 性能优化

---

## 📈 **性能指标要求**

| 指标 | 目标值 | 测量方法 |
|------|--------|----------|
| 映射响应时间 | <50ms | LLM输出到API调用的处理时间 |
| 安全检查时间 | <20ms | 安全验证流程耗时 |
| API调用成功率 | >95% | 有效API调用占比 |
| 错误恢复时间 | <2秒 | 异常到恢复正常的时间 |
| 复合动作精度 | >90% | 序列动作按预期执行的比例 |

---

**🎯 总结**：此架构设计为任务11提供了完整的0风险实施路径，可以在任务10修复期间独立开发和测试，确保两个轨道的并行推进不会相互阻塞。 