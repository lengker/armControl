/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"
#include "logger.h"
#include "config.h"
#include "state_machine.h"
#include "myuart.h"
#include "motion.h"
#include "pump.h"
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAG "FreeRTOS"

extern SystemContext_t g_system;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for CalculationTask */
osThreadId_t CalculationTaskHandle;
const osThreadAttr_t CalculationTask_attributes = {
  .name = "CalculationTask",
  .priority = (osPriority_t) osPriorityNormal,
  // Ruckig C++ trajectory generation has high stack usage on first update().
  .stack_size = 3072 * 4
};
/* Definitions for PickAndPlace */
osThreadId_t PickAndPlaceHandle;
const osThreadAttr_t PickAndPlace_attributes = {
  .name = "PickAndPlace",
  .priority = (osPriority_t) osPriorityLow,
  // Automatic pick-and-place path now runs IK + Ruckig + debug prints in this task.
  // 1024*4 bytes is not enough and triggers stack overflow on the first motion step.
  .stack_size = 4096 * 4
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for AngleQueue */
osMessageQueueId_t AngleQueueHandle;
const osMessageQueueAttr_t AngleQueue_attributes = {
  .name = "AngleQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCalculationTask(void *argument);
void StartPickAndPlace(void *argument);
void StartImuTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of AngleQueue */
  AngleQueueHandle = osMessageQueueNew (1, 16, &AngleQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CalculationTask */
  CalculationTaskHandle = osThreadNew(StartCalculationTask, NULL, &CalculationTask_attributes);
  if (CalculationTaskHandle == NULL) {
    printf("[RTOS] Create CalculationTask failed\r\n");
  }

  /* creation of PickAndPlace */
  PickAndPlaceHandle = osThreadNew(StartPickAndPlace, NULL, &PickAndPlace_attributes);
  if (PickAndPlaceHandle == NULL) {
    printf("[RTOS] Create PickAndPlace failed\r\n");
  }

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);
  if (ImuTaskHandle == NULL) {
    printf("[RTOS] Create ImuTask failed\r\n");
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCalculationTask */
extern osMessageQueueId_t AngleQueueHandle; /* 声明外部队列句柄 */

/**
  * @brief  Function implementing the CalculationTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCalculationTask */
/**
 * @brief  CalculationTask任务入口函数
 * @param  argument: 未使用
 * @retval None
 * @note   负责跟随模式下的实时运动控制
 */
void StartCalculationTask(void *argument)
{
  /* USER CODE BEGIN StartCalculationTask */
  // 等待系统初始化完成（由 PickAndPlace 任务负责）
  osDelay(4000);
  
  LOG_I(TAG, "CalculationTask started");

  /* Infinite loop */
  for(;;)
  {
    CalculationTask_Run();
  }
  /* USER CODE END StartCalculationTask */
}

/* USER CODE BEGIN Header_StartPickAndPlace */
/**
* @brief Function implementing the PickAndPlace thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPickAndPlace */
/**
 * @brief  PickAndPlace任务入口函数
 * @param  argument: 未使用
 * @retval None
 * @note   负责系统初始化和自动抓取流程
 */
void StartPickAndPlace(void *argument)
{
  /* USER CODE BEGIN StartPickAndPlace */
  LOG_I(TAG, "=== System Initializing ===");
  
  // 【安全】立即强制关闭气泵
  Pump_Off();
  osDelay(100);
  
  // 【安全】强制复位全局状态
  g_system.status.pump_on = false;
  g_system.status.emergency_stop = false;
  g_system.sm.current_state = STATE_IDLE;
  g_system.sm.prev_state = STATE_IDLE;
  g_system.sm.retry_count = 0;
  
  osDelay(SYSTEM_INIT_DELAY_MS); // 等待电机上电稳定
  
  // 初始化外设
  Pump_Init();
  My_UART_Init();
  
  // 初始化状态机
  StateMachine_Init();
  
  // 机械臂回零
  LOG_I(TAG, "Moving to home position...");
  Motion_SetHome();
  
  osDelay(MOTOR_READY_DELAY_MS); // 等待回零完成
  
  LOG_I(TAG, "=== System Ready ===");
  LOG_I(TAG, "Work mode: %s", 
        g_system.work_mode == WORK_MODE_AUTO ? "AUTO" : 
        g_system.work_mode == WORK_MODE_TEST ? "TEST" : "FOLLOW");
  
  /* Infinite loop */
  for(;;)
  {
    PickAndPlace_Run();
  }
  /* USER CODE END StartPickAndPlace */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
/**
 * @brief  ImuTask任务入口函数
 * @param  argument: 未使用
 * @retval None
 * @note   负责IMU数据采集和姿态稳定
 */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  LOG_I(TAG, "ImuTask started");
  
  /* Infinite loop */
  for(;;)
  {
    ImuTask_Run();
  }
  /* USER CODE END StartImuTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
