/**
 ******************************************************************************
 * @file    safety.h
 * @brief   系统安全检查和紧急处理
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 */

#ifndef SAFETY_H
#define SAFETY_H

#include "main.h"
#include <stdbool.h>

/**
 * @brief 系统安全初始化（在main函数中尽早调用）
 * @note 强制关闭所有危险输出，复位关键状态
 */
void Safety_EarlyInit(void);

/**
 * @brief 紧急停止处理
 * @note 立即停止所有运动，关闭气泵
 */
void Safety_EmergencyStop(void);

/**
 * @brief 检查系统状态是否安全
 * @return true: 安全; false: 不安全
 */
bool Safety_CheckSystemState(void);

#endif // SAFETY_H
