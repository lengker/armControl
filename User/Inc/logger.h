/**
 ******************************************************************************
 * @file    logger.h
 * @brief   统一日志系统
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 * @attention
 * 提供分级日志输出，方便调试和现场问题定位
 ******************************************************************************
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include "cmsis_os2.h"

/* ========== 日志等级定义 ========== */
typedef enum {
    LOG_LEVEL_ERROR = 0,  /**< E 错误：系统异常，必须处理 */
    LOG_LEVEL_WARN  = 1,  /**< E + W 警告：可能的问题，需要注意 */
    LOG_LEVEL_INFO  = 2,  /**< E + W + I 信息：关键流程节点 */
    LOG_LEVEL_DEBUG = 3,  /**< E + W + I + D 调试：详细调试信息 */
    LOG_LEVEL_TRACE = 4   /**< E + W + I + D + T 跟踪：最详细的信息 */
} LogLevel_t;

/* ========== 日志配置 ========== */
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO  /**< 默认日志等级 */
#endif

#define LOG_ENABLE_TIMESTAMP  1   /**< 启用时间戳 */
#define LOG_ENABLE_COLOR      0   /**< 启用颜色（串口终端支持ANSI） */

/* ========== 日志宏定义 ========== */
#if LOG_ENABLE_TIMESTAMP
    #define LOG_TIMESTAMP() printf("[%lu] ", (unsigned long)osKernelGetTickCount())
#else
    #define LOG_TIMESTAMP()
#endif

/**
 * @brief 错误日志（红色）
 * @note 用于系统错误、运动失败、通信异常等
 */
#define LOG_E(tag, fmt, ...) \
    do { \
        LOG_TIMESTAMP(); \
        printf("[E][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 警告日志（黄色）
 * @note 用于超时、重试、边界限制等
 */
#define LOG_W(tag, fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_WARN) { \
            LOG_TIMESTAMP(); \
            printf("[W][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief 信息日志（绿色）
 * @note 用于状态转换、关键动作完成等
 */
#define LOG_I(tag, fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_INFO) { \
            LOG_TIMESTAMP(); \
            printf("[I][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief 调试日志（蓝色）
 * @note 用于坐标数据、中间计算结果等
 */
#define LOG_D(tag, fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_DEBUG) { \
            LOG_TIMESTAMP(); \
            printf("[D][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * @brief 跟踪日志（灰色）
 * @note 用于函数进入/退出、循环计数等
 */
#define LOG_T(tag, fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_TRACE) { \
            LOG_TIMESTAMP(); \
            printf("[T][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)

/* ========== 专用日志宏 ========== */

/**
 * @brief 打印三维坐标
 */
#define LOG_COORD(tag, label, x, y, z) \
    LOG_D(tag, "%s: X=%.1f Y=%.1f Z=%.1f", label, x, y, z)

/**
 * @brief 打印状态转换
 */
#define LOG_STATE(tag, from, to) \
    LOG_I(tag, "State: %s -> %s", StateMachine_GetStateName(from), StateMachine_GetStateName(to))

/**
 * @brief 打印视觉数据接收
 */
#define LOG_VISION(tag, cam_x, cam_y, cam_z, arm_x, arm_y, arm_z) \
    do { \
        LOG_I(tag, "Vision RX: Cam(%.1f, %.1f, %.1f)cm -> Arm(%.1f, %.1f, %.1f)mm", \
              cam_x, cam_y, cam_z, arm_x, arm_y, arm_z); \
    } while(0)

/**
 * @brief 打印运动命令
 */
#define LOG_MOTION(tag, label, x, y, z, ret) \
    do { \
        if (ret == 0) { \
            LOG_I(tag, "%s: (%.1f, %.1f, %.1f) OK", label, x, y, z); \
        } else { \
            LOG_E(tag, "%s: (%.1f, %.1f, %.1f) FAILED ret=%d", label, x, y, z, ret); \
        } \
    } while(0)

#endif // LOGGER_H
