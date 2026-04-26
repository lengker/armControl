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
#include "calculate.h"
#include "stdio.h"
#include "DrEmpower_can.h"
#include "fdcan.h"
#include "myuart.h"
#include "cmd.h"
#include "can.h"
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
extern float target_x, target_y, target_z;
extern float camera_raw_x, camera_raw_y, camera_raw_z;
extern uint8_t system_work_mode;
extern MissionState_t current_state;
extern uint8_t f4_new_msg_flag;
extern uint8_t f4_rx_buf[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

typedef void (*StateFunction)(void);

void State_Idle(void);
void State_MoveToBox1(void);
void State_GrabBox1(void);
void State_LiftBox1(void);
void State_MoveToPlace(void);
void State_ReleaseAll(void);

static StateFunction current_state_function = State_Idle;
static float place_x = 200.0f;
static float place_y = 0.0f;
static float place_z = 200.0f;
static float box_x, box_y, box_z;
static const float k_pick_approach_offset_z = 80.0f;
static const float k_pick_lift_offset_z = 140.0f;
static const float k_pick_pitch_deg = -90.0f;
static const float k_coord_scale_cm_to_mm = 10.0f;
static const float k_coord_yz_scale = 0.9328f;
static const float k_arm_x_offset_mm = 27.46f;
static const float k_arm_y_offset_mm = 219.42f;
static const float k_arm_z_offset_mm = 38.01f;

void App_CameraToArmCoordinates(float cam_x, float cam_y, float cam_z,
                                float* arm_x, float* arm_y, float* arm_z) {
    if (arm_x == NULL || arm_y == NULL || arm_z == NULL) {
        return;
    }

    // 串口接收的 cam_x/cam_y/cam_z 单位为 cm，这里统一转换为机械臂使用的 mm。
    *arm_x = cam_x * k_coord_scale_cm_to_mm + k_arm_x_offset_mm;
    *arm_y = -cam_z * k_coord_scale_cm_to_mm * k_coord_yz_scale + k_arm_y_offset_mm + MOTION_Y_CALIB_BIAS_MM;
    *arm_z = -cam_y * k_coord_scale_cm_to_mm * k_coord_yz_scale + k_arm_z_offset_mm;
}

static void print_current_pos(const char* prefix, float x, float y, float z) {
    printf("[%s] X:%.1f Y:%.1f Z:%.1f\r\n", prefix, x, y, z);
}

static void App_ResetMission(void) {
    current_state = STATE_IDLE;
    current_state_function = State_Idle;
}

static int App_MoveRuckigChecked(const char* tag, float x, float y, float z) {
    int ret = Motion_MoveToXYZ_RuckigSmooth(x, y, z, k_pick_pitch_deg, 2.0f);
    print_current_pos(tag, x, y, z);
    if (ret != 0) {
        printf("[%s] move failed, ret=%d\r\n", tag, ret);
    }
    return ret;
}

void State_Idle(void) {
    current_state = STATE_IDLE;
    osDelay(20);
}

void State_MoveToBox1(void) {
    printf("[Auto] 1. 锁定视觉目标并前往抓取预备点\r\n");
    App_CameraToArmCoordinates(camera_raw_x, camera_raw_y, camera_raw_z,
                               &box_x, &box_y, &box_z);
    current_state = STATE_MOVE_TO_BOX1;

    if (App_MoveRuckigChecked("抓取预备点", box_x, box_y, box_z + k_pick_approach_offset_z) == 0) {
        current_state = STATE_GRAB_BOX1;
        current_state_function = State_GrabBox1;
    } else {
        Pump_Off();
        App_ResetMission();
    }
}

void State_GrabBox1(void) {
    printf("[Auto] 2. 下压到目标点并吸取\r\n");
    current_state = STATE_GRAB_BOX1;

    if (App_MoveRuckigChecked("抓取点", box_x, box_y, box_z) != 0) {
        Pump_Off();
        App_ResetMission();
        return;
    }

    Pump_On();
    osDelay(500);

    if (App_MoveRuckigChecked("抓取抬升点", box_x, box_y, box_z + k_pick_lift_offset_z) == 0) {
        current_state = STATE_LIFT_BOX1;
        current_state_function = State_LiftBox1;
    } else {
        Pump_Off();
        App_ResetMission();
    }
}

void State_LiftBox1(void) {
    printf("[Auto] 3. 已吸取，回初始位\r\n");
    current_state = STATE_LIFT_BOX1;

    Motion_SetHome();
    osDelay(2500);

    current_state = STATE_MOVE_TO_PLACE;
    current_state_function = State_MoveToPlace;
}

void State_MoveToPlace(void) {
    printf("[Auto] 4. 到达初始位，移动到放置点上方\r\n");
    current_state = STATE_MOVE_TO_PLACE;

    if (App_MoveRuckigChecked("放置预备点", place_x, place_y, place_z + k_pick_approach_offset_z) == 0) {
        current_state = STATE_RELEASE_ALL;
        current_state_function = State_ReleaseAll;
    } else {
        Pump_Off();
        App_ResetMission();
    }
}

void State_ReleaseAll(void) {
    printf("[Auto] 5. 放下目标并回到待机\r\n");
    current_state = STATE_RELEASE_ALL;

    if (App_MoveRuckigChecked("放置点", place_x, place_y, place_z) != 0) {
        Pump_Off();
        App_ResetMission();
        return;
    }

    osDelay(300);
    Pump_Off();
    osDelay(500);

    if (App_MoveRuckigChecked("放置后抬升点", place_x, place_y, place_z + k_pick_approach_offset_z) != 0) {
        App_ResetMission();
        return;
    }

    Motion_SetHome();
    osDelay(2000);
    printf("[Auto] 本轮抓取完成，等待下一个视觉目标\r\n");
    App_ResetMission();
}

/* USER CODE END 1 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void CalculationTask_Run(void)
{
    static uint8_t initialized = 0;
    static uint32_t last_follow_tick = 0;

    if (!initialized) {
        initialized = 1;
        printf("[FW] Ruckig debug probe v2\r\n");
        Motion_SetHome();
        Pump_Off();
        osDelay(3000);
    }

    if (system_work_mode == 0) {
        uint32_t now = osKernelGetTickCount();
        if ((now - last_follow_tick) >= 200U && current_state == STATE_IDLE) {
            last_follow_tick = now;
            (void)Motion_MoveToXYZ_RuckigSmooth(target_x, target_y, target_z, k_pick_pitch_deg, 0.8f);
        } else {
            osDelay(20);
        }
    } else {
        osDelay(100);
    }

    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
}

void ImuTask_Run(void)
{
    static uint32_t print_tick = 0;

    // 1. 发送请求给 IMU (100Hz)
    if (!motion_stream_active) {
        imu_request_euler();
    }
    // 2. 检查数据有效性并限流打印 (每 500ms 打印一次，降低串口负载)
    // 这样不会干扰你观察数据，也能防止串口塞满
    if(imu.valid && (osKernelGetTickCount() - print_tick > 500))
    {
        print_tick = osKernelGetTickCount();
        // 这里只留这一个打印，删掉其他的 heartbeat，防止互相干扰
        printf("P:%.2f, R:%.2f\r\n", imu.pitch, imu.roll);
    }
    osDelay(10);
}

void PickAndPlace_Run(void)
{
    if (system_work_mode != 1) {
        App_ResetMission();
        osDelay(20);
        return;
    }

    if (current_state_function == State_Idle) {
        switch (current_state) {
        case STATE_MOVE_TO_BOX1:
            current_state_function = State_MoveToBox1;
            break;
        case STATE_GRAB_BOX1:
            current_state_function = State_GrabBox1;
            break;
        case STATE_LIFT_BOX1:
            current_state_function = State_LiftBox1;
            break;
        case STATE_MOVE_TO_PLACE:
            current_state_function = State_MoveToPlace;
            break;
        case STATE_RELEASE_ALL:
            current_state_function = State_ReleaseAll;
            break;
        case STATE_IDLE:
        default:
            current_state_function = State_Idle;
            break;
        }
    }

    if (current_state_function != NULL) {
        current_state_function();
    }
}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
