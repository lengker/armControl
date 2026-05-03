# 代码结构说明文档

## 📁 项目结构

```
├── Core/
│   ├── Inc/                    # STM32 HAL 头文件
│   └── Src/
│       ├── app.c               # 应用层任务入口
│       └── app_freertos.c      # FreeRTOS 任务配置
│
├── User/
│   ├── Inc/
│   │   ├── types.h             # 【新】全局类型定义
│   │   ├── logger.h            # 【新】日志系统
│   │   ├── config.h            # 【新】配置参数集中管理
│   │   ├── state_machine.h     # 【新】状态机头文件
│   │   ├── motion.h            # 运动控制
│   │   ├── myuart.h            # 串口通信
│   │   ├── pump.h              # 气泵控制
│   │   └── ...
│   │
│   └── Src/
│       ├── state_machine.c     # 【新】状态机实现
│       ├── motion.c            # 运动控制实现
│       ├── myuart.c            # 串口通信实现
│       ├── pump.c              # 气泵控制实现
│       └── ...
│
├── CODE_OPTIMIZATION_REPORT.md # 优化报告
└── README_CODE_STRUCTURE.md    # 本文档
```

## 🎯 核心模块说明

### 1. 类型定义模块 (`types.h`)
**作用：** 定义系统中所有数据结构和枚举类型

**关键类型：**
- `SystemContext_t`: 全局系统上下文，封装所有全局状态
- `WorkMode_t`: 工作模式枚举（跟随/自动）
- `MissionState_t`: 状态机状态枚举
- `CameraData_t`: 相机数据结构
- `ArmTarget_t`: 机械臂目标坐标

**使用方式：**
```c
extern SystemContext_t g_system;  // 全局上下文

// 访问工作模式
if (g_system.work_mode == WORK_MODE_AUTO) {
    // 自动模式逻辑
}

// 访问相机数据
float cam_x = g_system.camera.raw.x;
```

### 2. 日志系统 (`logger.h`)
**作用：** 提供分级日志输出，方便调试

**日志等级：**
- `LOG_E`: 错误（系统异常，必须处理）
- `LOG_W`: 警告（可能的问题）
- `LOG_I`: 信息（关键流程节点）
- `LOG_D`: 调试（详细调试信息）
- `LOG_T`: 跟踪（最详细信息）

**使用示例：**
```c
#define TAG "MyModule"

LOG_I(TAG, "System initialized");
LOG_E(TAG, "Motion failed, ret=%d", ret);
LOG_COORD(TAG, "Target", x, y, z);
LOG_VISION(TAG, cam_x, cam_y, cam_z, arm_x, arm_y, arm_z);
```

**配置：**
```c
// 在 logger.h 中修改日志等级
#define LOG_LEVEL LOG_LEVEL_INFO  // 只显示 INFO 及以上
```

### 3. 配置管理 (`config.h`)
**作用：** 集中管理所有配置参数

**参数分类：**
- 系统配置：初始化延时、电机就绪时间
- 抓取流程：预备点偏移、抬升高度、气泵延时
- 放置点坐标：默认放置位置
- 运动控制：跟随模式更新周期

**修改参数：**
```c
// 修改抓取预备点高度
#define PICK_APPROACH_OFFSET_Z_MM   100.0f  // 从80改为100

// 修改放置点坐标
#define PLACE_POINT_X_MM            250.0f
```

### 4. 状态机模块 (`state_machine.h/c`)
**作用：** 实现自动抓取流程的完整状态机

**状态流程：**
```
IDLE → MOVE_TO_BOX → GRAB_BOX → LIFT_BOX → MOVE_TO_PLACE → RELEASE_BOX → IDLE
                                     ↓
                                  ERROR (超时/失败)
```

**关键功能：**
- 状态超时检测
- 自动重试机制（最多3次）
- 状态转换日志
- 错误恢复

**使用方式：**
```c
// 初始化（在系统启动时调用一次）
StateMachine_Init();

// 主循环（在任务中周期调用）
StateMachine_Run();

// 手动触发状态转换
StateMachine_Transition(STATE_MOVE_TO_BOX1);

// 重置到待机状态
StateMachine_Reset(true);  // true=清除重试计数
```

### 5. 运动控制模块 (`motion.h/c`)
**作用：** 机械臂运动控制和坐标转换

**关键函数：**
```c
// 回零
Motion_SetHome();

// Ruckig平滑运动（推荐）
int ret = Motion_MoveToXYZ_RuckigSmooth(x, y, z, pitch, duration);

// 相机坐标转机械臂坐标
Calculate_CameraToArm(cam_x, cam_y, cam_z, &arm_x, &arm_y, &arm_z);
```

**坐标系说明：**
- **相机坐标系**：X右 Y下 Z前，单位cm，向下倾斜21.12°
- **机械臂坐标系**：X前 Y左 Z上，单位mm

### 6. 串口通信模块 (`myuart.h/c`)
**作用：** 处理VOFA调参和视觉数据协议

**协议说明：**
- **USART1 (VOFA)**：7字节，帧头0xFF
- **USART2 (视觉)**：19字节，帧头0xAA55，帧尾0x0D

**视觉数据处理流程：**
1. 接收相机坐标（cm）
2. 校验和验证
3. 坐标转换（cm → mm）
4. 打印日志（方便核对）
5. 触发状态机或更新目标

## 🔍 调试指南

### 如何知道系统卡在哪里？

**1. 查看日志输出**
```
[1234][I][StateMachine] State: IDLE -> MOVE_TO_BOX
[1235][D][StateMachine] Box approach: X=250.0 Y=0.0 Z=180.0
[5678][I][StateMachine] Move to approach: (250.0, 0.0, 180.0) OK
```

**2. 状态超时检测**
如果某个状态卡住超过设定时间，会自动打印：
```
[10000][E][StateMachine] State timeout! State=MOVE_TO_BOX Duration=10000ms
[10001][W][StateMachine] Retry 1/3
```

**3. 运动失败日志**
```
[5678][E][StateMachine] Move to approach: (250.0, 0.0, 180.0) FAILED ret=-11
[5679][E][StateMachine] Failed to move to approach point, ret=-11
```

### 如何知道是否收到视觉坐标？

**查看日志：**
```
[2345][I][UART] Vision RX: Cam(25.5, 10.2, 30.0)cm -> Arm(245.0, -300.0, 102.0)mm
[2346][I][UART] Auto mode: Box detected, starting mission
[2347][I][StateMachine] State: IDLE -> MOVE_TO_BOX
```

**如果没有收到：**
- 检查串口连接
- 检查波特率配置
- 查看是否有校验和错误：
  ```
  [2345][W][UART] Vision: Checksum error! calc=0xAB recv=0xCD
  ```

### 流水线每一步的打印

**完整流程日志示例：**
```
[0][I][FreeRTOS] === System Initializing ===
[1000][I][FreeRTOS] Moving to home position...
[3000][I][FreeRTOS] === System Ready ===
[3001][I][FreeRTOS] Work mode: AUTO
[3002][I][StateMachine] State machine initialized

// 收到视觉数据
[5000][I][UART] Vision RX: Cam(25.0, 10.0, 30.0)cm -> Arm(250.0, -300.0, 100.0)mm
[5001][I][UART] Auto mode: Box detected, starting mission
[5002][I][StateMachine] State: IDLE -> MOVE_TO_BOX

// 步骤1：移动到预备点
[5003][I][StateMachine] Step 1: Moving to box approach point
[5004][D][StateMachine] Box approach: X=250.0 Y=-300.0 Z=180.0
[7000][I][StateMachine] Move to approach: (250.0, -300.0, 180.0) OK
[7001][I][StateMachine] State: MOVE_TO_BOX -> GRAB_BOX

// 步骤2：抓取
[7002][I][StateMachine] Step 2: Grabbing box
[7003][D][StateMachine] Box grab: X=250.0 Y=-300.0 Z=100.0
[9000][I][StateMachine] Move to grab: (250.0, -300.0, 100.0) OK
[9001][I][StateMachine] Pump ON
[9500][D][StateMachine] Box lift: X=250.0 Y=-300.0 Z=240.0
[11000][I][StateMachine] Lift box: (250.0, -300.0, 240.0) OK
[11001][I][StateMachine] State: GRAB_BOX -> LIFT_BOX

// 步骤3：回零
[11002][I][StateMachine] Step 3: Returning to home position
[13500][I][StateMachine] Home position reached
[13501][I][StateMachine] State: LIFT_BOX -> MOVE_TO_PLACE

// 步骤4：移动到放置点
[13502][I][StateMachine] Step 4: Moving to place point
[13503][D][StateMachine] Place approach: X=200.0 Y=0.0 Z=280.0
[15500][I][StateMachine] Move to place approach: (200.0, 0.0, 280.0) OK
[15501][I][StateMachine] State: MOVE_TO_PLACE -> RELEASE_BOX

// 步骤5：放置
[15502][I][StateMachine] Step 5: Releasing box
[15503][D][StateMachine] Place point: X=200.0 Y=0.0 Z=200.0
[17000][I][StateMachine] Move to place: (200.0, 0.0, 200.0) OK
[17300][I][StateMachine] Pump OFF
[17800][D][StateMachine] Place lift: X=200.0 Y=0.0 Z=280.0
[19000][I][StateMachine] Lift from place: (200.0, 0.0, 280.0) OK
[19001][I][StateMachine] Returning to home
[21000][I][StateMachine] === Mission completed successfully ===
[21001][I][StateMachine] Resetting state machine
[21002][I][StateMachine] State: RELEASE_BOX -> IDLE
```

## ⚙️ 常见配置修改

### 1. 修改日志等级（减少打印）
```c
// logger.h
#define LOG_LEVEL LOG_LEVEL_WARN  // 只显示警告和错误
```

### 2. 修改抓取高度
```c
// config.h
#define PICK_APPROACH_OFFSET_Z_MM   100.0f  // 预备点高度
#define PICK_LIFT_OFFSET_Z_MM       150.0f  // 抬升高度
```

### 3. 修改超时时间
```c
// state_machine.h
#define SM_TIMEOUT_MOVE_MS          15000   // 移动超时改为15秒
```

### 4. 修改重试次数
```c
// state_machine.h
#define SM_MAX_RETRY_COUNT          5       // 最多重试5次
```

### 5. 切换工作模式
```c
// myuart.c 初始化部分
g_system.work_mode = WORK_MODE_FOLLOW;  // 改为跟随模式
```

## 🐛 常见问题排查

### 问题1：系统一直在IDLE状态
**可能原因：**
- 没有收到视觉数据
- 工作模式不是AUTO

**排查：**
1. 检查日志是否有 `Vision RX` 打印
2. 检查工作模式：`Work mode: AUTO`
3. 检查串口连接和波特率

### 问题2：运动失败 ret=-11
**原因：** IK逆运动学无解，目标点超出工作空间

**解决：**
1. 检查目标坐标是否合理
2. 检查坐标转换是否正确
3. 调整目标点到工作空间内

### 问题3：状态超时
**原因：** 运动时间过长或卡住

**解决：**
1. 检查电机是否正常
2. 检查运动轨迹是否合理
3. 增大超时时间（临时方案）

### 问题4：坐标转换不准确
**排查步骤：**
1. 查看视觉数据打印，核对相机坐标
2. 查看转换后的机械臂坐标
3. 检查偏移量配置：`K_ARM_X/Y/Z_OFFSET_MM`
4. 检查倾角参数：`K_COS_TILT`, `K_SIN_TILT`

## 📝 代码规范

### 命名规范
- 全局变量：`g_` 前缀，如 `g_system`
- 静态变量：`s_` 前缀
- 常量：全大写+下划线，如 `PICK_APPROACH_OFFSET_Z_MM`
- 函数：模块名_动作_对象，如 `StateMachine_Transition`

### 注释规范
- 每个函数必须有 Doxygen 风格注释
- 复杂算法必须有原理说明
- 魔法数字必须有单位和来源

### 日志规范
- 每个模块定义自己的TAG
- 关键流程节点使用 LOG_I
- 错误必须使用 LOG_E
- 调试信息使用 LOG_D

## 🎓 学习路径

**新手入门：**
1. 阅读 `types.h` 了解数据结构
2. 阅读 `config.h` 了解配置参数
3. 阅读 `state_machine.c` 了解流程逻辑
4. 运行系统，观察日志输出

**进阶开发：**
1. 修改配置参数，观察效果
2. 添加新的状态或功能
3. 优化运动轨迹
4. 添加错误恢复逻辑

**高级优化：**
1. 性能分析和优化
2. 添加更多传感器
3. 实现更复杂的抓取策略
4. 多机械臂协同

---

**文档版本：** v1.0  
**更新日期：** 2026-05-03  
**维护者：** Your Team
