/**
 ******************************************************************************
 * @file    safety.c
 * @brief   系统安全检查和紧急处理实现
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 */

#include "safety.h"
#include "pump.h"
#include "types.h"
#include <stdio.h>

extern SystemContext_t g_system;

/**
 * @brief 系统安全初始化（在main函数中尽早调用）
 * @note 在GPIO初始化之后立即调用，确保所有危险输出处于安全状态
 */
void Safety_EarlyInit(void) {
    printf("\r\n[Safety] System boot - initializing...\r\n");
    
    // 清除复位标志
    __HAL_RCC_CLEAR_RESET_FLAGS();
    
    // 1. 强制关闭气泵（最高优先级）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    
    // 2. 复位全局状态标志
    g_system.status.pump_on = false;
    g_system.status.motion_active = false;
    g_system.status.emergency_stop = false;
    
    // 3. 复位状态机到IDLE
    g_system.sm.current_state = STATE_IDLE;
    g_system.sm.prev_state = STATE_IDLE;
    g_system.sm.retry_count = 0;
    g_system.sm.timeout_flag = false;
    
    printf("[Safety] Pump OFF, state reset to IDLE\r\n");
}

/**
 * @brief 紧急停止处理
 * @note 立即停止所有运动，关闭气泵
 */
void Safety_EmergencyStop(void) {
    printf("\r\n[Safety] *** EMERGENCY STOP ***\r\n");
    
    // 1. 立即关闭气泵
    Pump_Off();
    g_system.status.pump_on = false;
    
    // 2. 设置紧急停止标志
    g_system.status.emergency_stop = true;
    
    // 3. 停止运动
    g_system.status.motion_active = false;
    
    // 4. 复位状态机
    g_system.sm.current_state = STATE_ERROR;
    
    printf("[Safety] All systems halted\r\n");
}

/**
 * @brief 检查系统状态是否安全
 * @return true: 安全; false: 不安全
 */
bool Safety_CheckSystemState(void) {
    // 检查气泵状态是否与标志一致
    GPIO_PinState pump_pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    bool pump_actual = (pump_pin == GPIO_PIN_SET);
    
    if (pump_actual != g_system.status.pump_on) {
        printf("[Safety] WARNING: Pump state mismatch! Flag=%d, Actual=%d\r\n",
               g_system.status.pump_on, pump_actual);
        return false;
    }
    
    // 检查是否处于紧急停止状态
    if (g_system.status.emergency_stop) {
        printf("[Safety] WARNING: System in emergency stop state\r\n");
        return false;
    }
    
    return true;
}
