/**
 ******************************************************************************
 * @file    myuart.h
 * @brief   串口通信协议处理
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 * @attention
 * 实现VOFA调参协议和视觉数据协议的解析
 ******************************************************************************
 */

#ifndef __MYUART_H
#define __MYUART_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 串口协议处理器结构体
 * @note 通用的串口协议解析框架，支持自定义帧头、帧尾和回调
 */
typedef struct {
    uint8_t* buffer;               /**< 数据缓冲区指针 */
    uint8_t package_length;        /**< 数据包总长度（包括帧头帧尾） */
    uint16_t header;               /**< 帧头 (支持1-2字节) */
    uint8_t header_length;         /**< 帧头长度 (1或2) */
    bool tail_flag;                /**< 是否有帧尾 */
    uint8_t tail;                  /**< 帧尾字节 */
    void (*callback)(uint8_t*);    /**< 解析成功后的回调函数 */

    uint8_t buffer_index;          /**< 当前接收索引 */
    bool header_found;             /**< 是否找到帧头 */
    uint8_t receive_byte;          /**< 单字节接收缓存 */
} ProtocolHandler;

/* ========== 协议处理器实例声明 ========== */
extern ProtocolHandler vofa_handler;   /**< USART1: VOFA调参协议 */
extern ProtocolHandler vision_handler; /**< USART2: 视觉数据协议 */

/* ========== 函数声明 ========== */

/**
 * @brief 初始化所有串口协议
 * @note 必须在使用串口前调用
 */
void My_UART_Init(void);

/**
 * @brief 串口接收解码状态机
 * @param[in] huart 串口句柄
 * @param[in] protocolHandler 协议处理器
 * @note 在串口中断回调中调用
 */
void uart_RX_decode(UART_HandleTypeDef *huart, ProtocolHandler *protocolHandler);

#endif // __MYUART_H
