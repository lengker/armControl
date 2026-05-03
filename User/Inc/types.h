/**
 ******************************************************************************
 * @file    types.h
 * @brief   系统类型定义和数据结构
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 * @attention
 * 本文件定义了系统中使用的所有数据结构和枚举类型
 ******************************************************************************
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ========== 工作模式定义 ========== */
/**
 * @brief 系统工作模式枚举
 */
typedef enum {
    WORK_MODE_FOLLOW = 0,    /**< 实时跟随模式（VOFA调参/手动控制） */
    WORK_MODE_AUTO = 1,      /**< 自动抓取模式（比赛模式） */
    WORK_MODE_TEST = 2,      /**< 测试模式（单独控制每个电机验证角度定义） */
    WORK_MODE_EMERGENCY = 0xFF /**< 紧急停止模式 */
} WorkMode_t;

/* ========== 状态机定义 ========== */
/**
 * @brief 自动抓取流程状态枚举
 */
typedef enum {
    STATE_IDLE = 0,          /**< 待机状态，等待视觉目标 */
    STATE_MOVE_TO_BOX1,      /**< 移动到箱子上方预备点 */
    STATE_GRAB_BOX1,         /**< 下压并开启气泵吸取 */
    STATE_LIFT_BOX1,         /**< 抬起箱子 */
    STATE_MOVE_TO_PLACE,     /**< 移动到放置区 */
    STATE_RELEASE_ALL,       /**< 关气泵卸货，返回待机 */
    STATE_ERROR              /**< 错误状态 */
} MissionState_t;

/* ========== 坐标数据结构 ========== */
/**
 * @brief 三维坐标结构体
 */
typedef struct {
    float x;  /**< X坐标 (mm) */
    float y;  /**< Y坐标 (mm) */
    float z;  /**< Z坐标 (mm) */
} Coordinate3D_t;

/**
 * @brief 相机坐标数据
 * @note 相机坐标系定义：
 *       - X轴：水平向右
 *       - Y轴：垂直向下
 *       - Z轴：水平向前
 *       - 单位：厘米 (cm)
 */
typedef struct {
    Coordinate3D_t raw;      /**< 原始相机坐标 (cm) */
    Coordinate3D_t converted; /**< 转换后的机械臂坐标 (mm) */
    uint32_t timestamp;      /**< 接收时间戳 (ms) */
    bool valid;              /**< 数据有效标志 */
} CameraData_t;

/**
 * @brief 机械臂目标坐标
 * @note 机械臂坐标系定义：
 *       - X轴：水平前方
 *       - Y轴：水平左侧
 *       - Z轴：垂直向上
 *       - 单位：毫米 (mm)
 */
typedef struct {
    Coordinate3D_t target;   /**< 目标坐标 (mm) */
    float pitch;             /**< 俯仰角 (度) */
    uint32_t timestamp;      /**< 更新时间戳 (ms) */
} ArmTarget_t;

/* ========== 状态机上下文 ========== */
/**
 * @brief 状态机运行时上下文
 */
typedef struct {
    MissionState_t current_state;  /**< 当前状态 */
    MissionState_t prev_state;     /**< 上一个状态 */
    uint32_t state_enter_time;     /**< 状态进入时间 (ms) */
    uint32_t state_duration;       /**< 状态持续时间 (ms) */
    uint8_t retry_count;           /**< 当前状态重试次数 */
    bool timeout_flag;             /**< 超时标志 */
} StateMachineContext_t;

/* ========== 系统全局上下文 ========== */
/**
 * @brief 系统全局上下文结构体
 * @note 封装所有全局状态，避免全局变量污染
 */
typedef struct {
    WorkMode_t work_mode;          /**< 工作模式 */
    StateMachineContext_t sm;      /**< 状态机上下文 */
    CameraData_t camera;           /**< 相机数据 */
    ArmTarget_t arm_target;        /**< 机械臂目标 */
    
    struct {
        Coordinate3D_t box;        /**< 箱子坐标 (mm) */
        Coordinate3D_t place;      /**< 放置点坐标 (mm) */
    } mission;                     /**< 任务相关坐标 */
    
    struct {
        bool pump_on;              /**< 气泵状态 */
        bool motion_active;        /**< 运动中标志 */
        bool emergency_stop;       /**< 紧急停止标志 */
    } status;                      /**< 系统状态标志 */
    
} SystemContext_t;

/* ========== 全局上下文声明 ========== */
extern SystemContext_t g_system;

#endif // TYPES_H
