//
// Created by Lenovo on 2026/3/12.
//

#ifndef ARM2_FR_CMD_H
#define ARM2_FR_CMD_H

#include "main.h" // 包含标准整型和 HAL 库头文件

// ---------------------------------------------------------
// 1. 定义由 G4 发送给 F4 的数据结构 (F4 专属接收邮箱)
// ---------------------------------------------------------
typedef struct {
    uint8_t robot_state;  // G4状态 -> 0:待机, 1:移动中, 2:到达抓取点
    uint8_t grab_cmd;     // G4指令 -> 0:不执行, 1:执行抓取, 2:执行放置
    int16_t target_x;     // 物块X坐标 (毫米)
    int16_t target_y;     // 物块Y坐标 (毫米)
} G4_To_F4_Data_t;

// 声明为外部全局变量，方便你在 main.c 里的任何地方直接读取
extern G4_To_F4_Data_t g4_rx_data;

// ---------------------------------------------------------
// 2. 函数声明
// ---------------------------------------------------------
void CAN_G4_Send_To_F4(uint8_t robot_state, uint8_t grab_cmd, int16_t x, int16_t y);

// F4 向 G4 发送机械臂反馈的函数
void CAN_F4_Send_To_G4(uint8_t arm_state, uint8_t task_finished, uint8_t can_robot_move);

// F4 解析 G4 发来数据的函数
void CAN_F4_Parse_G4_Data(uint8_t *rx_data);

#endif //ARM2_FR_CMD_H
