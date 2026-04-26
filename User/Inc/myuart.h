#ifndef __MYUART_H
#define __MYUART_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ========== 框架定义 ========== */
typedef struct {
    uint8_t* buffer;               // 数据缓冲区
    uint8_t package_length;        // 数据总长度
    uint16_t header;               // 帧头 (支持1-2字节)
    uint8_t header_length;         // 帧头长度
    bool tail_flag;                // 是否有帧尾
    uint8_t tail;                  // 帧尾字节
    void (*callback)(uint8_t*);    // 解析成功后的回调函数

    uint8_t buffer_index;          // 当前接收索引
    bool header_found;             // 是否找到帧头
    uint8_t receive_byte;          // 单字节接收缓存
} ProtocolHandler;

// 工作模式切换：
// 0: 实时跟随模式 (VOFA/代码直接改坐标调参时使用)
// 1: 自动抓取模式 (比赛/视觉下发坐标时使用)

typedef enum {
    STATE_IDLE,            // 待机，等待目标坐标更新
    STATE_MOVE_TO_BOX1,    // 移动到箱子正上方预备点
    STATE_GRAB_BOX1,       // 下压并开启气泵吸取
    STATE_LIFT_BOX1,       // 抬起箱子
    STATE_MOVE_TO_PLACE,   // 移动到放置区
    STATE_RELEASE_ALL      // 关气泵卸货，返回待机
  } MissionState_t;

/* ========== 外部全局变量 ========== */
extern uint8_t system_work_mode;
extern MissionState_t current_state;

extern float target_x;
extern float target_y;
extern float target_z;
extern float camera_raw_x;
extern float camera_raw_y;
extern float camera_raw_z;
extern uint8_t pump_state;

/* ========== 句柄声明 ========== */
extern ProtocolHandler vofa_handler;   // USART1
extern ProtocolHandler vision_handler; // USART2

/* ========== 函数声明 ========== */
void My_UART_Init(void); // 初始化所有串口协议
void uart_RX_decode(UART_HandleTypeDef *huart, ProtocolHandler *protocolHandler);

#endif
