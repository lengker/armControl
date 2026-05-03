# 机械卡住问题修复方案

## 问题描述

在抓取过程中，1号或2号电机因机械结构卡住，电流过大触发保护断电。

### 问题分析

从FK正解核对日志可以看出：
```
[FK 正解核对] 电机角: Th1=16.7, Th2=-5.4, Th3=0.0
-> L1 绝对角=16.7 | X投影=335.19, Z高度=100.75
-> L2 绝对角=-67.8 | X投影=88.90, Z高度=-218.08
```

**关键问题：**
1. **Th2=-5.4°（负角度）**：小臂角度为负，说明小臂向后折叠，这是不合理的机械姿态
2. **L2绝对角=-67.8°**：小臂几乎水平向下，机械臂过度前伸
3. **预备点高度不足**：PICK_APPROACH_OFFSET_Z_MM=80mm太低，导致机械臂在下降时就已经过度伸展

## 修复方案

### 1. 增加预备点高度（已在config.h中完成）

```c
#define PICK_APPROACH_OFFSET_Z_MM   150.0f   // 从80mm增加到150mm
#define PICK_LIFT_OFFSET_Z_MM       180.0f   // 从140mm增加到180mm
```

**原理：**
- 更高的预备点意味着机械臂在接近箱子时保持更垂直的姿态
- 减少水平伸展距离，避免大臂和小臂夹角过小

### 2. 降低运动速度，便于观察（已在config.h中完成）

```c
#define MOTION_APPROACH_DURATION_S  4.0f    // 移动到预备点：4秒
#define MOTION_GRAB_DURATION_S      5.0f    // 下压抓取：5秒（最慢）
#define MOTION_LIFT_DURATION_S      4.0f    // 抬升：4秒
```

**目的：**
- 慢速运动可以观察机械臂在哪个位置开始卡住
- 减少惯性冲击，降低电机负载

### 3. 添加安全高度中间点（已在state_machine.c中实现）

**State_MoveToBox() 修改：**
```c
// 先抬高到安全高度Z=100mm
if (approach_z < safe_z) {
    Motion_MoveToXYZ_RuckigSmooth(box_x, box_y, 100.0f, pitch, 3.0f);
    osDelay(1000);  // 等待1秒观察
}

// 然后再移动到预备点
Motion_MoveToXYZ_RuckigSmooth(approach_x, approach_y, approach_z, pitch, 4.0f);
osDelay(1000);  // 等待1秒观察
```

**原理：**
- 避免机械臂直接从home位置斜向下运动到箱子上方
- 先垂直下降到安全高度，再水平移动，路径更安全

### 4. 启用关节角度限位保护（已在motion.c和calculate.c中实现）

**motion.c 修改：**
```c
// 取消注释，启用角度限位
ik_result = optimize_and_limit_angles(ik_result, 0.0f);
```

**calculate.c 修改：**
```c
// 启用调试打印
printf("[限位前] Th0=%.1f, Th1=%.1f, Th2=%.1f\r\n", ...);
printf("[夹角检查] 当前夹角=%.1f° (范围: %.1f°~%.1f°)\r\n", ...);
printf("[限位后] Th0=%.1f, Th1=%.1f, Th2=%.1f\r\n", ...);
```

**关节限制：**
```c
#define J1_MIN  0.0f      // 大臂最小角度（水平）
#define J1_MAX  120.0f    // 大臂最大角度
#define J2_MIN  0.0f      // 小臂最小角度（防止负角度）
#define J2_MAX  120.0f    // 小臂最大角度
#define ARM_ANGLE_MIN  30.0f   // 大小臂最小夹角（避免碰撞）
#define ARM_ANGLE_MAX  150.0f  // 大小臂最大夹角（避免过度伸展）
```

**原理：**
- 强制限制theta2 >= 0°，避免小臂向后折叠
- 限制大小臂夹角在30°~150°之间，避免机械干涉

### 5. 增加观察延时（已在state_machine.c中实现）

**State_GrabBox() 修改：**
```c
// 下降到抓取点（5秒慢速）
Motion_MoveToXYZ_RuckigSmooth(grab_x, grab_y, grab_z, pitch, 5.0f);
osDelay(1000);  // 到达后等待1秒观察

// 开启气泵
Pump_On();
osDelay(2000);

// 抬升（4秒）
Motion_MoveToXYZ_RuckigSmooth(lift_x, lift_y, lift_z, pitch, 4.0f);
osDelay(1000);  // 抬升后等待1秒观察
```

## 测试建议

### 第一次测试：观察运动轨迹
1. 将箱子放在之前卡住的位置
2. 启动自动模式
3. **重点观察：**
   - 机械臂是否先抬高到Z=100mm
   - 从预备点到抓取点的下降过程是否平滑
   - 打印的关节角度是否都为正值
   - 是否还有电机卡住的情况

### 第二次测试：检查角度限位
1. 查看串口打印的角度限位信息：
   ```
   [限位前] Th0=..., Th1=..., Th2=...
   [夹角检查] 当前夹角=...
   [限位后] Th0=..., Th1=..., Th2=...
   ```
2. 确认theta2是否被限制在0°以上
3. 确认夹角是否在30°~150°范围内

### 第三次测试：验证抓取精度
1. 确认末端是否准确到达箱子顶部中心
2. 如果位置偏差，可能需要微调：
   - K_ARM_Z_OFFSET_MM（相机Z偏移）
   - PICK_APPROACH_OFFSET_Z_MM（预备点高度）

## 如果问题仍然存在

### 可能的进一步调整：

1. **增加预备点高度到200mm：**
   ```c
   #define PICK_APPROACH_OFFSET_Z_MM   200.0f
   ```

2. **调整关节角度限制：**
   ```c
   #define J2_MIN  5.0f      // 小臂最小角度改为5°（更保守）
   #define ARM_ANGLE_MIN  40.0f   // 最小夹角改为40°
   ```

3. **添加更多中间点：**
   - 在State_MoveToBox()中添加更多waypoint
   - 例如：Home → Z=150mm → Z=100mm → 预备点 → 抓取点

4. **检查机械结构：**
   - 确认大臂和小臂之间是否有物理干涉
   - 检查电机是否有异常阻力
   - 确认连杆机构是否顺畅

## 修改文件清单

1. **User/Inc/config.h**
   - 已增加预备点高度
   - 已添加运动速度参数

2. **User/Src/state_machine.c**
   - State_MoveToBox()：添加安全高度中间点和观察延时
   - State_GrabBox()：增加慢速运动和观察延时

3. **User/Src/motion.c**
   - 启用optimize_and_limit_angles()函数

4. **User/Src/calculate.c**
   - 启用角度限位调试打印

## 预期效果

1. **机械臂运动更安全：**
   - 不会出现负角度
   - 大小臂夹角保持在安全范围
   - 运动路径更合理

2. **便于调试观察：**
   - 慢速运动可以清楚看到卡住的位置
   - 每个关键点都有1秒停顿
   - 打印详细的角度限位信息

3. **减少电机过载：**
   - 避免过度伸展导致的大负载
   - 慢速运动减少冲击
   - 合理的姿态降低扭矩需求
