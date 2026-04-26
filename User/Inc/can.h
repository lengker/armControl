#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#include "fdcan.h"
#include "stdio.h"  // 用于调试打印
#include "dm_imu.h" // 陀螺仪CAN驱动



// 全局变量声明
extern uint8_t f4_rx_buf[8]; // 找个地方定义这个全局变量
extern uint8_t f4_new_msg_flag;
extern uint8_t TxData[8];  // 发送缓冲区
extern uint8_t RxData[8];  // 接收缓冲区
extern FDCAN_TxHeaderTypeDef TxHeader; //
extern FDCAN_RxHeaderTypeDef RxHeader; // can的句柄


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif