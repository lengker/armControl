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
#include "servo.h"
#include "cmd.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 通过上位机设置零点
// 2. 定义电机 ID
#define ID_BASE   1  // 底座
#define ID_BIG    2  // 大臂
#define ID_SMALL  3  // 小臂
// 定义舵机参数
#define SERVO_CENTER_PWM_ANGLE  90.0f  // 舵机物理中点（通常对应直臂）
#define SERVO_DIR_INVERT        1      // 1: 正常, -1: 反向 (如果舵机转反了改这里)
// 【新增】舵机物理补偿值
// 作用：如果觉得吸盘往里扣（指向大臂），就减小这个值
//      如果觉得吸盘往外翘（指向天空），就增大这个值
// 建议调试步骤：先填 0，观察偏多少度，然后直接加减那个角度
#define SERVO_OFFSET    0.0f
// 2. 比例系数 (主要调节 点头/伸出去时的垂直度)
// 现象：回正时垂直，一伸出去就"上翘" -> 说明补偿过头了 -> 设为 0.7 或 0.8
// 现象：回正时垂直，一伸出去就"向里扣" -> 说明补偿不够 -> 设为 1.1 或 1.2
#define SERVO_SCALE     1.00f
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 定义测试点结构体
typedef struct {
  float x;
  float y;
  float z;
  const char* desc; // 描述
} TestPoint_t;

typedef enum {
  STATE_IDLE,            // 待机
  STATE_MOVE_TO_BOX1,    // 走向 1 号箱
  STATE_GRAB_BOX1,       // 抓 1 号箱
  STATE_MOVE_TO_BOX2,    // 走向 2 号箱
  STATE_GRAB_BOX2,       // 抓 2 号箱
  STATE_GO_HOME,         // 回归位区
  STATE_RELEASE_ALL      // 卸货
} MissionState_t;

MissionState_t current_state = STATE_IDLE;

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
  .stack_size = 128 * 4
};
/* Definitions for CAN2Task */
osThreadId_t CAN2TaskHandle;
const osThreadAttr_t CAN2Task_attributes = {
  .name = "CAN2Task",
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
void StartCAN2Task(void *argument);

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

  /* creation of CAN2Task */
  CAN2TaskHandle = osThreadNew(StartCAN2Task, NULL, &CAN2Task_attributes);

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
  printf("--- 初始化 ---\r\n");
  osDelay(1000); // 给电机一点时间上电
  set_mode(ID_BASE, 2);
  set_mode(ID_BIG, 2);
  set_mode(ID_SMALL, 2);
  osDelay(500);
  // 确认初始状态
  set_angle(ID_BASE, 0, 10, 5, 1);
  set_angle(ID_BIG, 90, 10, 5, 1);
  set_angle(ID_SMALL, 0, 10, 5, 1);
  set_servo_angle(90 + SERVO_OFFSET);
  osDelay(5000);
  // 固定变量
  float target_absolute_pitch = -90.0f;
  JointAngles_t ik_result;

  // float target_x = 15.0f;   // 默认 x 坐标 (cm)
  // float target_y = 10.0f;    // 默认 y 坐标 (cm)
  // float target_z = 20.0f;   // 默认 z 坐标 (cm)
  // 【新增】：开启视觉串口USART2的第一次接收 (只收1字节)
  extern UART_HandleTypeDef huart2;
  HAL_UART_Receive_IT(&huart2, &vision_rx_byte, 1); // 启动监听，接受到数据后跳进 myuart.c 的 HAL_UART_RxCpltCallback 函数

  /* Infinite loop */
  for(;;)
  {
    // 从全局变量读取当前目标坐标（使用 volatile 或临时拷贝确保一致性）
    float x = target_x;
    float y = target_y;
    float z = target_z;

    // 持续执行逆解算并保持位置 (防止掉电或外力移动)
    if (inverse_kinematics(x, y, z, target_absolute_pitch, &ik_result) == 0)
    {
      // 发送电机指令
      set_angle(ID_BASE,  ik_result.theta0, 50.0f, 40.0f, 1);
      set_angle(ID_BIG,   ik_result.theta1, 50.0f, 40.0f, 1);
      set_angle(ID_SMALL, ik_result.theta2, 50.0f, 40.0f, 1);

      // 计算舵机 PWM (使用你调好的 SCALE 和 OFFSET)
      float raw_pwm = -ik_result.theta3;
      float delta = raw_pwm - 90.0f;
      float final_pwm = 90.0f + (delta * SERVO_SCALE) + SERVO_OFFSET;

      // 限幅
      if(final_pwm < 0.0f) final_pwm = 0.0f;
      if(final_pwm > 270.0f) final_pwm = 270.0f;
      set_servo_angle(final_pwm);

      // 打印此时的状态供记录
      // printf("目标: (%.1f, %.1f, %.1f) | 逆解角: B:%.1f L:%.1f S:%.1f | 舵机PWM: %.1f\r\n",
      //        x, y, z,
      //        ik_result.theta0, ik_result.theta1, ik_result.theta2,
      //        final_pwm);
    }
    else
    {
      //printf("[错误] 该坐标 (%.1f, %.1f, %.1f) 超出机械臂范围！\r\n", x, y, z);
    }

    osDelay(1000); // 每秒刷新一次日志
  }
  /* USER CODE END StartCalculationTask */
}

/* USER CODE BEGIN Header_StartCAN2Task */
/**
* @brief Function implementing the CAN2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN2Task */
void StartCAN2Task(void *argument)
{
  /* USER CODE BEGIN StartCAN2Task */
  printf("[System] FDCAN2 发送任务已启动 (Target ID: 0x301)\r\n");
  /* Infinite loop */
  for(;;)
  {
    // 只需要调用封装好的函数，不再关心 Header 和 ID
    CAN_G4_Send_To_F4(0x01, 0x00, 100, 200);
    if (f4_new_msg_flag) {
      printf("[G4 <- F4] 收到反馈: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
              f4_rx_buf[0], f4_rx_buf[1], f4_rx_buf[2], f4_rx_buf[3], f4_rx_buf[4], f4_rx_buf[5], f4_rx_buf[6], f4_rx_buf[7]);
      f4_new_msg_flag = 0;
    }

    // uint32_t state = HAL_FDCAN_GetState(&hfdcan2);
    // if (state != HAL_FDCAN_STATE_READY) {
    //   printf("[Error] CAN2 State Error: %lu\r\n", state);
    // }
    //
    // // 检查错误状态
    // uint32_t err = HAL_FDCAN_GetError(&hfdcan2);
    // if (err != HAL_FDCAN_ERROR_NONE) {
    //   printf("[Error] CAN2 Error Code: 0x%08lX\r\n", err);
    // }

    osDelay(1000); // 每一秒发一次
  }
  /* USER CODE END StartCAN2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

