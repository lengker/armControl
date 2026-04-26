# RoboArm CAN陀螺仪稳定系统集成指南

## 系统架构

```
┌─────────────────────┐
│   机械狗或外部MCU   │
│   (带BMI088陀螺仪)  │
└──────────┬──────────┘
           │ CAN消息 (ID: 0x200)
           │ 传输 Roll/Pitch/Yaw
           ▼
┌─────────────────────────────────────┐
│     RoboArm主控 (STM32G474)         │
│  ┌───────────────────────────────┐  │
│  │  CAN接收 (imu_can.c)         │  │
│  │  - 解析姿态数据               │  │
│  │  - 更新imu_data全局变量      │  │
│  └───────┬───────────────────────┘  │
│          │                          │
│  ┌───────▼───────────────────────┐  │
│  │  运动控制 (motion.c)          │  │
│  │  - 实时PID稳定补偿           │  │
│  │  - 修正关节角度               │  │
│  └───────┬───────────────────────┘  │
│          │                          │
│  ┌───────▼───────────────────────┐  │
│  │  CAN电机驱动 (DrEmpower_can)  │  │
│  │  - 控制3个关节步进电机        │  │
│  │  - 控制舵机                   │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
```

---

## CAN消息格式定义

### **陀螺仪数据帧**

| 字段 | 字节 | 类型 | 范围 | 分辨率 | 说明 |
|------|------|------|------|--------|------|
| Roll (Φ) | 0-1 | int16 | ±327.67° | 0.01° | 滚转角 |
| Pitch (θ) | 2-3 | int16 | ±327.67° | 0.01° | 俯仰角 |
| Yaw (ψ) | 4-5 | int16 | ±327.67° | 0.01° | 偏航角 |
| 保留 | 6-7 | - | - | - | 预留 |

**示例CAN数据包**（大端字节序）：
```
ID:  0x200
DLC: 8 bytes
Data:
  [0-1]: 0x0010  → Roll = 16 * 0.01 = 0.16°
  [2-3]: 0xFFAE  → Pitch = -82 * 0.01 = -0.82°
  [4-5]: 0x0000  → Yaw = 0°
  [6-7]: 0x0000  → 保留
```

---

## 代码集成清单

### ✅ **已实现的文件**

1. **imu_can.h / imu_can.c**
   - CAN数据解析和格式化
   - `IMU_ParseCANData()`: 解析接收到的CAN数据
   - `IMU_FormatCANData()`: 格式化陀螺仪数据（可选，用于转发）

2. **can.c 修改**
   - 在 `HAL_FDCAN_RxFifo0Callback()` 中添加陀螺仪消息识别
   - 根据 CAN ID (0x200) 判断消息类型
   - 自动调用 `IMU_ParseCANData()` 解析数据

3. **motion.h / motion.c 修改**
   - `Motion_StabilityInit()`: 初始化PID控制器
   - `Motion_CalcStabilityCompensation()`: 计算稳定补偿
   - `Motion_MoveToXYZ()`: 在运动控制中应用补偿

4. **main.c 修改**
   - 添加初始化调用

---

## 调试和参数调节

### **1. 启用/禁用稳定功能**

在 `motion.h` 中修改：
```c
#define STABILITY_ENABLED    1  // 1=启用, 0=禁用
```

### **2. PID参数调节**

在 `motion.c` 中修改 PID 增益：
```c
// 当前值：Kp=1.5, Ki=0.05, Kd=0.3
PID_Controller_t roll_pid = {1.5f, 0.05f, 0.3f, 0, 0};
PID_Controller_t pitch_pid = {1.5f, 0.05f, 0.3f, 0, 0};
```

**调参指南**：
- **增加Kp**: 响应速度更快，但易震荡。(建议范围: 0.5~3.0)
- **增加Ki**: 消除稳态误差，但响应变慢。(建议范围: 0~0.2)
- **增加Kd**: 减少震荡，提高稳定性。(建议范围: 0~0.5)

### **3. 其他可调参数**

```c
#define ROLL_COMP_GAIN      0.05f    // Roll补偿增益
#define PITCH_COMP_GAIN     0.05f    // Pitch补偿增益
#define COMP_DEADZONE       1.0f     // 死区阈值(°) - 小于此值不补偿
```

---

## 陀螺仪数据源说明

### 如果陀螺仪连接到**机械狗主控板**

你需要在那个板子上：
1. 连接BMI088传感器（I2C或SPI）
2. 读取陀螺仪数据（Roll, Pitch, Yaw）
3. **以 0x200 ID 周期性发送 CAN 消息**（建议100Hz）

**发送伪代码**：
```c
// 在机械狗的FreeRTOS任务中
void IMU_TransmitTask(void *arg) {
    for(;;) {
        uint8_t can_data[8] = {0};
        
        // 读取BMI088数据
        bmi088_read(&roll, &pitch, &yaw);
        
        // 格式化为CAN消息
        IMU_FormatCANData(&imu_data, can_data);
        
        // 或手动打包
        int16_t roll_raw = (int16_t)(roll / 0.01f);
        can_data[0] = (roll_raw >> 8) & 0xFF;
        can_data[1] = roll_raw & 0xFF;
        // ... 类似处理 pitch 和 yaw
        
        // 发送CAN消息
        FDCAN_SendMessage(FDCAN1, 0x200, can_data, 8);
        
        osDelay(10);  // 100Hz
    }
}
```

### 如果陀螺仪连接到**当前STM32G474**（不推荐，因为已有多个任务）

需要额外实现BMI088驱动：
1. 配置I2C或SPI接口
2. 读取陀螺仪原始数据
3. 自己调用 `IMU_ParseCANData()` 将数据加载到 `imu_data`

---

## 实时监测

### **过滤器配置（CAN ID 0x200）**

在STM32CubeMX中，FDCAN1应已配置过滤器。如需手动设置：

```c
// 在 fdcan.c 的初始化中检查是否有类似以下过滤器配置
FDCAN_FilterTypeDef sFilterConfig;
sFilterConfig.IdType = FDCAN_STANDARD_ID;
sFilterConfig.FilterIndex = 0;
sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
sFilterConfig.FilterID1 = 0x200;  // 匹配 ID 0x200
sFilterConfig.FilterID2 = 0x200;
HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
```

### **串口调试输出**

系统会自动打印陀螺仪数据：
```
[IMU_CAN] Roll=2.15°, Pitch=-1.23°, Yaw=0.00°
[Stab] Roll=2.15°(→-0.10°), Pitch=-1.23°(→-0.08°)
```

---

## 拓展功能

### **1. 多轴稳定（当前仅支持Roll/Pitch）**

要添加Yaw补偿，修改 `Motion_CalcStabilityCompensation()`，添加Yaw PID。

### **2. 自适应增益**

根据机械臂的伸展长度动态调整PID增益（长度越长，需要更大的补偿）。

### **3. 低通滤波**

在 `IMU_ParseCANData()` 中添加一阶低通滤波，减少噪声：
```c
imu_data.roll = 0.9f * imu_data.roll + 0.1f * (raw_roll * 0.01f);  // α=0.1
```

---

## 故障排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 收不到陀螺仪数据 | CAN无数据 | 检查外部MCU是否正常发送；检查CAN线连接 |
| 收到数据但不稳定 | IMU_DATA_VALID为0 | 检查 `IMU_ParseCANData()` 是否被调用 |
| 补偿过度（震荡） | PID增益过大 | 降低Kp和Kd |
| 补偿不足（抖动不消除） | PID增益过小 | 增加Kp |
| 机械臂移动缓慢 | 补偿延迟太高 | 降低Ki和integral_error积累值 |

---

## 参考资源

- **CAN消息标准**: CANopen (DS402)
- **陀螺仪**:Bosch BMI088 数据手册
- **S32K系列FDCAN**: NXP S32G2xx Reference Manual, Chapter 15

---

**最后修改**: 2026-03-25
**作者**: GitHub Copilot
