//
// Created by 18375 on 2026/3/6.
//

#ifndef DR_TRY_MYUART_H
#define DR_TRY_MYUART_H

#include "stm32g4xx_hal.h"  // 包含这个就足够了，它会自动关联所有已开启的外设
#include "usart.h"

extern float target_x;
extern float target_y;
extern float target_z;

extern uint8_t Data[7];

// 声明视觉接收的单字节缓存，以便在外边启动中断
extern uint8_t vision_rx_byte;

#endif //DR_TRY_MYUART_H