/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app.c
  * Description        : This file provides code for the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"
#include "logger.h"
#include "config.h"
#include "state_machine.h"
#include "motion.h"
#include "pump.h"
#include "dm_imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define TAG "App"

extern SystemContext_t g_system;
extern volatile uint8_t motion_stream_active;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
 * @brief 计算任务主循环
 * @note 在跟随模式下周期性更新机械臂目标位置
 */
void CalculationTask_Run(void)
{
    static uint32_t last_follow_tick = 0;

    if (g_system.work_mode == WORK_MODE_FOLLOW) {
        uint32_t now = osKernelGetTickCount();
        if ((now - last_follow_tick) >= FOLLOW_MODE_UPDATE_MS && 
            g_system.sm.current_state == STATE_IDLE) {
            last_follow_tick = now;
            
            LOG_T(TAG, "Follow mode: moving to (%.1f, %.1f, %.1f)",
                  g_system.arm_target.target.x,
                  g_system.arm_target.target.y,
                  g_system.arm_target.target.z);
            
            (void)Motion_MoveToXYZ_RuckigSmooth(
                g_system.arm_target.target.x,
                g_system.arm_target.target.y,
                g_system.arm_target.target.z,
                PICK_PITCH_DEG,
                FOLLOW_MODE_DURATION_S
            );
        } else {
            osDelay(20);
        }
    } else if (g_system.work_mode == WORK_MODE_TEST) {
        // 测试模式：等待串口命令
        osDelay(100);
    } else {
        osDelay(100);
    }

    // 心跳LED
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
}

/**
 * @brief IMU任务主循环
 * @note 周期性读取IMU数据用于姿态稳定
 */
void ImuTask_Run(void)
{
    static uint32_t print_tick = 0;

    // 1. 发送请求给 IMU (100Hz)
    if (!motion_stream_active) {
        imu_request_euler();
    }
    
    // 2. 限流打印 (每 500ms 打印一次)
    if(imu.valid && (osKernelGetTickCount() - print_tick > 500))
    {
        print_tick = osKernelGetTickCount();
        LOG_D("IMU", "Pitch=%.2f Roll=%.2f", imu.pitch, imu.roll);
    }
    
    osDelay(10);
}

/**
 * @brief 自动抓取任务主循环
 * @note 运行状态机，处理自动抓取流程
 */
void PickAndPlace_Run(void)
{
    StateMachine_Run();
}

/* USER CODE END 1 */
