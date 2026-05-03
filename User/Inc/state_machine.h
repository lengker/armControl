/**
 ******************************************************************************
 * @file    state_machine.h
 * @brief   自动抓取流程状态机
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 * @attention
 * 实现自动抓取的完整状态机逻辑，包括超时检测和错误恢复
 ******************************************************************************
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "types.h"

/* ========== 状态机配置参数 ========== */
#define SM_TIMEOUT_IDLE_MS          60000   /**< 待机状态超时 (60s) */
#define SM_TIMEOUT_MOVE_MS          10000   /**< 移动状态超时 (10s) */
#define SM_TIMEOUT_GRAB_MS          5000    /**< 抓取状态超时 (5s) */
#define SM_TIMEOUT_LIFT_MS          5000    /**< 抬升状态超时 (5s) */
#define SM_TIMEOUT_PLACE_MS         10000   /**< 放置状态超时 (10s) */
#define SM_TIMEOUT_RELEASE_MS       5000    /**< 释放状态超时 (5s) */

#define SM_MAX_RETRY_COUNT          3       /**< 最大重试次数 */

/* ========== 状态机函数声明 ========== */

/**
 * @brief 初始化状态机
 * @note 必须在使用状态机前调用
 */
void StateMachine_Init(void);

/**
 * @brief 状态机主循环
 * @note 在 PickAndPlace 任务中周期性调用
 */
void StateMachine_Run(void);

/**
 * @brief 重置状态机到待机状态
 * @param[in] clear_retry 是否清除重试计数
 */
void StateMachine_Reset(bool clear_retry);

/**
 * @brief 触发状态转换
 * @param[in] new_state 目标状态
 * @note 会自动记录状态转换时间和重置超时计时器
 */
void StateMachine_Transition(MissionState_t new_state);

/**
 * @brief 检查状态超时
 * @return true: 超时, false: 未超时
 */
bool StateMachine_CheckTimeout(void);

/**
 * @brief 获取状态名称字符串
 * @param[in] state 状态枚举
 * @return 状态名称字符串
 */
const char* StateMachine_GetStateName(MissionState_t state);

/**
 * @brief 获取当前状态持续时间
 * @return 持续时间 (ms)
 */
uint32_t StateMachine_GetStateDuration(void);

/**
 * @brief 增加重试计数
 * @return 当前重试次数
 */
uint8_t StateMachine_IncrementRetry(void);

/**
 * @brief 清除重试计数
 */
void StateMachine_ClearRetry(void);

/**
 * @brief 检查是否达到最大重试次数
 * @return true: 达到最大次数, false: 未达到
 */
bool StateMachine_IsMaxRetry(void);

/* ========== 状态处理函数声明 ========== */
void State_Idle(void);
void State_MoveToBox(void);
void State_GrabBox(void);
void State_LiftBox(void);
void State_MoveToPlace(void);
void State_ReleaseBox(void);
void State_Error(void);

#endif // STATE_MACHINE_H
