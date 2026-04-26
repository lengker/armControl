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
#include "calculate.h"
#include "stdio.h"
#include "DrEmpower_can.h"
#include "fdcan.h"
#include "myuart.h"
#include "cmd.h"
#include "can.h"
#include "motion.h"
#include "pump.h"
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 定义测试点结构体
// 引入全局坐标变量（它们在 myuart.c 中更新）
extern float target_x;
extern float target_y;
extern float target_z;

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
void StartCalculationTask(void *argument)
{
  /* USER CODE BEGIN StartCalculationTask */
  printf("--- CalculationTask初始化 ---\r\n");
  osDelay(1000); // 给电机一点时间上电
  // 2. 初始化泵和机械臂
  Pump_Init();
  //Motion_SetHome();
  osDelay(2000);

  /* Infinite loop */
  for(;;)
  {
    //printf("进入 ---\r\n");
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
void StartPickAndPlace(void *argument)
{
  /* USER CODE BEGIN StartPickAndPlace */
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
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
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
