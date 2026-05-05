#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "main.h"
#include "dm_imu.h"

// 电机 ID 定义 (从 app_freertos 移到这里统一管理)
#define ID_BASE   1
#define ID_BIG    2
#define ID_SMALL  3

/* ========== 陀螺仪安装位置 ========== */
#define IMU_MOUNT_POSITION      IMU_ON_WRIST  // 陀螺仪安装位置
#define IMU_ON_WRIST            0             // 在三臂（手腕）上
#define IMU_ON_GRIPPER          1             // 在末端执行器上（保留）

/* ========== 姿态稳定控制参数 ========== */
#define STABILITY_ENABLED       1           // 启用稳定补偿 (0=关闭, 1=启用)
#define ROLL_COMP_GAIN          0.05f       // Roll补偿增益（调节灵敏度）
#define PITCH_COMP_GAIN         0.05f       // Pitch补偿增益（三臂模式下最重要）
#define YAW_COMP_GAIN           0.02f       // Yaw补偿增益
#define COMP_DEADZONE           1.0f        // 死区 (小于此值不补偿，单位度)

/* ========== 轨迹模式开关 ========== */
// 0: 传统模式（直接 set_angle/set_angles）
// 1: Ruckig 丝滑模式（关节空间 jerk-limited 轨迹）
#define MOTION_USE_RUCKIG_SMOOTH  1

/* ========== 自动抓取中间点配置 ========== */
#define MID_POINT_X_SCALE    1.2f    // x轴外伸比例（1.2=外伸20%）
#define MID_POINT_Y_SCALE    1.2f    // y轴外伸比例（1.2=外伸20%）
#define MID_POINT_Z_OFFSET   60.0f   // z轴抬高高度（单位：mm）

static const float K_COORD_SCALE_CM_TO_MM = 10.0f;
static const float K_COS_TILT = 0.9328f;
static const float K_SIN_TILT = 0.3603f;
static const float K_ARM_X_OFFSET_MM = 27.46f;
static const float K_ARM_Y_OFFSET_MM = 219.42f;
static const float K_ARM_Z_OFFSET_MM = 38.01f;

/* ========== Z轴标定补偿 ========== */
// 如果实物到点普遍“偏低”，就增大这个值（单位 mm）。
// 例如偏低约 100~120mm，可先设为 110 再微调。
#define MOTION_Z_CALIB_BIAS_MM  0.0f
#define MOTION_Y_CALIB_BIAS_MM  0.0f
/**
 * @brief PID控制器结构体
 */
typedef struct {
    float kp, ki, kd;
    float integral_error;
    float prev_error;
} PID_Controller_t;

/**
 * @brief 姿态补偿结果结构体
 * 包含各轴的补偿值供运动控制使用
 */
typedef struct {
    float roll_comp;         // 底座旋转补偿 (°)
    float pitch_comp;        // 大臂俯仰补偿 (°)
    float yaw_comp;          // 偏航补偿 (°)
    float wrist_pitch_comp;  // 三臂俯仰补偿 (°) - 陀螺仪在三臂时使用
} StabilityCompensation_t;

/* ========== 全局PID控制器 ========== */
extern PID_Controller_t roll_pid;
extern PID_Controller_t pitch_pid;
extern volatile uint8_t motion_stream_active;

/**
 * @brief 机械臂回初始位（直立待机姿态）
 */
void Motion_SetHome(void);

/**
 * @brief 驱动机械臂移动到指定的空间坐标和姿态
 * @param x, y, z 目标坐标 (单位: mm)
 * @param pitch 末端吸盘绝对俯仰角 (例如垂直向下传 -90.0f)
 * @note 由于末端通过机械连杆保持垂直向下，pitch 通常固定传 -90.0f。
 * @return 0: 移动成功; -1: 坐标超出机械臂工作范围
 */
int Motion_MoveToXYZ(float x, float y, float z, float pitch);

/**
 * @brief 只移动底座和大臂，保留小臂延迟启动
 */
int Motion_MoveToXYZ_BaseBig(float x, float y, float z, float pitch);

/**
 * @brief 只移动小臂，适用于先移动底座/大臂后再下压的场景
 */
int Motion_MoveToXYZ_SmallArm(float x, float y, float z, float pitch);

/**
 * @brief 初始化PID控制器（稳定补偿）
 */
void Motion_StabilityInit(void);

/**
 * @brief 计算姿态稳定补偿（返回所有轴的补偿）
 * 
 * 两种模式：
 * 1. 陀螺仪在三臂上（IMU_ON_WRIST）:
 *    - wrist_pitch_comp: 三臂俯仰补偿，应用到theta2
 *    - roll_comp: 底座滚转补偿
 * 
 * 2. 其他位置：
 *    - roll_comp: 底座补偿
 *    - pitch_comp: 大臂补偿
 *
 * @return 补偿结果结构体
 */
StabilityCompensation_t Motion_CalcStabilityCompensation(void);

// ==============================
// 1. 安全移动到目标（自动走中间点避障）
int Motion_MoveToXYZ_Safe(float x, float y, float z, float pitch);
// 2. 抓取箱子完整流程（封装所有抓取动作）
void Motion_GrabBox(float x, float y, float z);
// 3. 卸货完整流程（封装所有卸货动作）
void Motion_PlaceBox(void);

int Motion_MoveStraightSmooth(float end_x, float end_y, float end_z, float duration_s);

/**
 * @brief 关节空间 Ruckig 丝滑移动（先 IK，再用 Ruckig 生成关节轨迹）
 * @note  这是在 Motion_MoveToXYZ 基础上的“更丝滑”实现：把一次 set_angle 改为 50Hz 连续下发关节角 setpoint。
 * @return 0: success; -1: IK failed; -2: Ruckig failed
 */
int Motion_MoveToXYZ_RuckigSmooth(float x, float y, float z, float pitch, float duration_hint_s);

#endif // MOTION_H
