/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app.h
  * Description        : Header for app.c
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

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

// 状态机函数指针类型
typedef void (*StateFunction)(void);

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void CalculationTask_Run(void);
void CAN2Task_Run(void);
void PickAndPlace_Run(void);
void ImuTask_Run(void);
void App_CameraToArmCoordinates(float cam_x, float cam_y, float cam_z,
                                float* arm_x, float* arm_y, float* arm_z);

// 状态机相关函数
void State_Idle(void);
void State_MoveToBox1(void);
void State_GrabBox1(void);
void State_LiftBox1(void);
void State_MoveToPlace(void);
void State_ReleaseAll(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
