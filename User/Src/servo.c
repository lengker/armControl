#include "servo.h"
#include "tim.h"

// 定义舵机参数
// 注意：MG995 0-180度对应的脉宽通常是 0.5ms-2.5ms
// 对应定时器计数值 (1us记一次数的情况下)
#define SERVO_MIN_PULSE   500   // 0度
#define SERVO_MAX_PULSE   2500  // 180度
#define SERVO_TIM_HANDLE  &htim1 // 你用的定时器句柄
#define SERVO_TIM_CHANNEL TIM_CHANNEL_1 // 你用的通道
// 请根据实际测试修改这个值。舵机物理最大行程
#define SERVO_MAX_PHYSICAL_ANGLE  210.0f

/**
 * @brief 设置舵机角度
 * @param angle: 0.0 ~ 180.0 度
 */
void set_servo_angle(float angle)
{
    // 1. 范围限制 (使用新的物理最大角度宏)
    if(angle < 0.0f) angle = 0.0f;
    if(angle > SERVO_MAX_PHYSICAL_ANGLE) angle = SERVO_MAX_PHYSICAL_ANGLE;

    // 2. 角度转脉宽值 (线性映射)
    // 映射公式: Pulse = Min + (angle / 物理最大角度) * (Max - Min)
    uint32_t pulse = SERVO_MIN_PULSE + (uint32_t)(angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / SERVO_MAX_PHYSICAL_ANGLE);

    // 3. 修改定时器比较寄存器 (CCR)
    __HAL_TIM_SET_COMPARE(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL, pulse);
}