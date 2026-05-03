#ifndef CONFIG_H
#define CONFIG_H

/* ========== 系统配置 ========== */
#define SYSTEM_INIT_DELAY_MS        1000    // 系统初始化延时
#define MOTOR_READY_DELAY_MS        2000    // 电机回零完成延时

/* ========== 抓取流程参数 ========== */
#define PICK_APPROACH_OFFSET_Z_MM   150.0f   // 抓取预备点Z轴偏移（箱子上方的安全距离）
#define PICK_LIFT_OFFSET_Z_MM       630.0f   // [已弃用] 改为使用绝对高度抬升
#define PICK_SAFE_LIFT_HEIGHT_MM    200.0f   // 抓取后抬升到的绝对高度（安全高度）
#define PICK_PITCH_DEG              -90.0f   // 抓取时俯仰角（垂直向下）

// 注意：抬升采用分段策略（避免远距离低位置时IK无解）
// 1. 先在原地小幅抬升50mm
// 2. 然后Y方向收回100mm，同时抬升50mm
// 3. 最后抬升到PICK_SAFE_LIFT_HEIGHT_MM

#define PUMP_ON_DELAY_MS            2000     // 气泵开启后真空压力建立等待时间
#define PUMP_OFF_DELAY_MS           300     // 放置前等待时间
#define RELEASE_LIFT_DELAY_MS       500     // 放置后抬升前等待，确认物体稳固落地了，机械臂再离开

/* ========== 运动速度参数（超慢速调试模式） ========== */
#define MOTION_APPROACH_DURATION_S  12.0f   // 移动到预备点的时间（秒）- 超慢速观察
#define MOTION_GRAB_DURATION_S      15.0f   // 下压到抓取点的时间（秒）- 超慢速观察
#define MOTION_LIFT_DURATION_S      12.0f   // 抬升的时间（秒）
#define MOTION_PLACE_DURATION_S     10.0f   // 移动到放置点的时间（秒）

/* ========== 调试打印参数 ========== */
#define DEBUG_MOTION_PRINT_INTERVAL_MS  500  // 运动过程中打印间隔（毫秒）

/* ========== 放置点坐标 ========== */
#define PLACE_POINT_X_MM            200.0f
#define PLACE_POINT_Y_MM            0.0f
#define PLACE_POINT_Z_MM            200.0f

/* ========== 运动控制参数 ========== */
#define FOLLOW_MODE_UPDATE_MS       200     // 跟随模式更新周期
#define FOLLOW_MODE_DURATION_S      0.8f    // 跟随模式运动时长

/* ========== 调试开关 ========== */
#define DEBUG_PRINT_POSITION        1       // 打印位置信息
#define DEBUG_PRINT_STATE           1       // 打印状态转换

#endif // CONFIG_H
