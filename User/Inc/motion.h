/**
 ******************************************************************************
 * @file    motion.h
 * @brief   机械臂运动控制模块
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 * @attention
 * 提供机械臂运动控制、坐标转换、姿态稳定等功能
 ******************************************************************************
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "main.h"
#include "dm_imu.h"

/* ========== 电机 ID 定义 ========== */
#define ID_BASE   1  /**< 底座旋转电机 (关节0) */
#define ID_BIG    2  /**< 大臂电机 (关节1) */
#define ID_SMALL  3  /**< 小臂电机 (关节2) */

/**
 * @note 重要：电机3（小臂）的实际转动方向与数学定义相反
 *       - 数学定义：正角度 = 向上，负角度 = 向下
 *       - 电机实际：正角度 = 向下，负角度 = 向上
 *       - 解决方案：发送给电机时取反（-theta2）
 *       - FK/IK计算仍使用数学定义，只在电机控制层取反
 */

/* ========== 相机坐标转换参数 ========== */
/**
 * @brief 坐标单位转换系数
 * @note 相机坐标单位为厘米(cm)，机械臂坐标单位为毫米(mm)
 */
#define K_COORD_SCALE_CM_TO_MM  10.0f

/**
 * @brief 相机倾角余弦值
 * @note 相机向下倾斜21.12度，cos(21.12°) ≈ 0.9328
 */
#define K_COS_TILT              0.9328f

/**
 * @brief 相机倾角正弦值
 * @note 相机向下倾斜21.12度，sin(21.12°) ≈ 0.3603
 */
#define K_SIN_TILT              0.3603f

/**
 * @brief 相机原点相对机械臂原点的X轴偏移
 * @note 单位：毫米(mm)，通过实际测量标定得出
 */
#define K_ARM_X_OFFSET_MM       27.46f

/**
 * @brief 相机原点相对机械臂原点的Y轴偏移
 * @note 单位：毫米(mm)，通过实际测量标定得出
 */
#define K_ARM_Y_OFFSET_MM       219.42f

/**
 * @brief 相机原点相对机械臂原点的Z轴偏移
 * @note 单位：毫米(mm)，通过实际测量标定得出
 */
#define K_ARM_Z_OFFSET_MM       38.01f

/**
 * @brief Y轴标定补偿值
 * @note 单位：毫米(mm)，用于微调Y轴误差，默认0
 */
#define MOTION_Y_CALIB_BIAS_MM  0.0f

/**
 * @brief Z轴标定补偿值
 * @note 单位：毫米(mm)，用于微调Z轴误差，默认0
 */
#define MOTION_Z_CALIB_BIAS_MM  0.0f

/* ========== 姿态稳定控制参数 ========== */
#define STABILITY_ENABLED       1      /**< 启用姿态稳定补偿 (0=关闭, 1=启用) */
#define ROLL_COMP_GAIN          0.05f  /**< Roll补偿增益 */
#define PITCH_COMP_GAIN         0.05f  /**< Pitch补偿增益 */
#define YAW_COMP_GAIN           0.02f  /**< Yaw补偿增益 */
#define COMP_DEADZONE           1.0f   /**< 补偿死区 (度) */

/* ========== 轨迹模式开关 ========== */
/**
 * @brief 运动轨迹模式选择
 * @note 0: 传统模式（直接 set_angle）
 *       1: Ruckig 丝滑模式（关节空间 jerk-limited 轨迹）
 */
#define MOTION_USE_RUCKIG_SMOOTH  1

/* ========== 自动抓取中间点配置 ========== */
#define MID_POINT_X_SCALE    1.2f    /**< X轴外伸比例 (1.2=外伸20%) */
#define MID_POINT_Y_SCALE    1.2f    /**< Y轴外伸比例 (1.2=外伸20%) */
#define MID_POINT_Z_OFFSET   60.0f   /**< Z轴抬高高度 (mm) */

/* ========== 数据结构定义 ========== */

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float kp;              /**< 比例增益 */
    float ki;              /**< 积分增益 */
    float kd;              /**< 微分增益 */
    float integral_error;  /**< 积分误差累积 */
    float prev_error;      /**< 上一次误差 */
} PID_Controller_t;

/**
 * @brief 姿态补偿结果结构体
 */
typedef struct {
    float roll_comp;         /**< 底座旋转补偿 (度) */
    float pitch_comp;        /**< 大臂俯仰补偿 (度) */
    float yaw_comp;          /**< 偏航补偿 (度) */
    float wrist_pitch_comp;  /**< 三臂俯仰补偿 (度) */
} StabilityCompensation_t;

/* ========== 全局变量声明 ========== */
extern PID_Controller_t roll_pid;   /**< Roll轴PID控制器 */
extern PID_Controller_t pitch_pid;  /**< Pitch轴PID控制器 */
extern volatile uint8_t motion_stream_active; /**< 运动流激活标志 */

/* ========== 函数声明 ========== */

/**
 * @brief 机械臂回初始位（直立待机姿态）
 * @note 底座0度，大臂90度（垂直），小臂0度（水平）
 */
void Motion_SetHome(void);

/**
 * @brief 驱动机械臂移动到指定的空间坐标和姿态
 * @param[in] x 目标X坐标 (mm)
 * @param[in] y 目标Y坐标 (mm)
 * @param[in] z 目标Z坐标 (mm)
 * @param[in] pitch 末端吸盘绝对俯仰角 (度)，垂直向下传-90.0f
 * @return 0: 移动成功; -1: 坐标超出机械臂工作范围
 */
int Motion_MoveToXYZ(float x, float y, float z, float pitch);

/**
 * @brief 只移动底座和大臂，保留小臂延迟启动
 * @param[in] x 目标X坐标 (mm)
 * @param[in] y 目标Y坐标 (mm)
 * @param[in] z 目标Z坐标 (mm)
 * @param[in] pitch 末端俯仰角 (度)
 * @return 0: 成功; -1: 失败
 */
int Motion_MoveToXYZ_BaseBig(float x, float y, float z, float pitch);

/**
 * @brief 只移动小臂
 * @param[in] x 目标X坐标 (mm)
 * @param[in] y 目标Y坐标 (mm)
 * @param[in] z 目标Z坐标 (mm)
 * @param[in] pitch 末端俯仰角 (度)
 * @return 0: 成功; -1: 失败
 */
int Motion_MoveToXYZ_SmallArm(float x, float y, float z, float pitch);

/**
 * @brief 初始化姿态稳定PID控制器
 * @note 清空积分项和历史误差
 */
void Motion_StabilityInit(void);

/**
 * @brief 计算姿态稳定补偿
 * @return 补偿结果结构体
 * @note 根据IMU数据计算各轴补偿值
 */
StabilityCompensation_t Motion_CalcStabilityCompensation(void);

/**
 * @brief 笛卡尔空间直线轨迹移动（五次多项式平滑）
 * @param[in] end_x 目标X坐标 (mm)
 * @param[in] end_y 目标Y坐标 (mm)
 * @param[in] end_z 目标Z坐标 (mm)
 * @param[in] duration_s 期望到达时间 (秒)
 * @return 0: 成功; -1: 失败
 */
int Motion_MoveStraightSmooth(float end_x, float end_y, float end_z, float duration_s);

/**
 * @brief 关节空间 Ruckig 丝滑移动
 * @param[in] x 目标X坐标 (mm)
 * @param[in] y 目标Y坐标 (mm)
 * @param[in] z 目标Z坐标 (mm)
 * @param[in] pitch 末端俯仰角 (度)
 * @param[in] duration_hint_s 期望时间提示 (秒)，0表示时间最优
 * @return 0: 成功; -11: IK失败; -2xx: Ruckig启动失败; -3xx: Ruckig执行失败; -401: 超时
 * @note 使用 Ruckig 库生成 jerk-limited 轨迹，运动更平滑
 */
int Motion_MoveToXYZ_RuckigSmooth(float x, float y, float z, float pitch, float duration_hint_s);

/**
 * @brief 相机坐标转换为机械臂坐标
 * @param[in] cam_x 相机X坐标 (cm)
 * @param[in] cam_y 相机Y坐标 (cm)
 * @param[in] cam_z 相机Z坐标 (cm)
 * @param[out] arm_x 机械臂X坐标 (mm)
 * @param[out] arm_y 机械臂Y坐标 (mm)
 * @param[out] arm_z 机械臂Z坐标 (mm)
 * @note 相机坐标系：X右 Y下 Z前，倾斜21.12度
 *       机械臂坐标系：X前 Y左 Z上
 */
void Calculate_CameraToArm(float cam_x, float cam_y, float cam_z,
                           float* arm_x, float* arm_y, float* arm_z);

/**
 * @brief 测试模式：单独控制指定电机到指定角度
 * @param[in] motor_id 电机ID (1=底座, 2=大臂, 3=小臂)
 * @param[in] angle 目标角度 (度)
 * @note 用于验证角度定义和FK正解
 */
void Motion_TestSingleMotor(uint8_t motor_id, float angle);

#endif // MOTION_H
