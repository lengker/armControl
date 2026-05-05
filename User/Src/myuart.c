#include "myuart.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "pump.h"
#include "motion.h"

// 引用外部串口句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* ========== 缓冲区定义 ========== */
uint8_t vofa_buf[10];
uint8_t vision_buf[25];

/* ========== 全局变量初始化 ========== */
uint8_t system_work_mode = 1;      // 默认自动模式
MissionState_t current_state = STATE_IDLE;

float target_x = 235.0f;
float target_y = 0.0f;
float target_z = 350.0f;
float camera_raw_x = 0.0f;
float camera_raw_y = 0.0f;
float camera_raw_z = 0.0f;
uint8_t pump_state = 0;

float v_x, v_y, v_z;

/* ========== 内部辅助函数：大端转float ========== */
static float bytes_to_float_big_endian(uint8_t* b) {
    uint32_t u = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                 ((uint32_t)b[2] << 8)  | ((uint32_t)b[3]);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static void uart_rearm_rx(UART_HandleTypeDef *huart, ProtocolHandler *ph, const char* name) {
    HAL_StatusTypeDef ret;

    if (huart == NULL || ph == NULL) {
        return;
    }

    ret = HAL_UART_Receive_IT(huart, &ph->receive_byte, 1);
    if (ret != HAL_OK) {
        printf("[UART] arm %s failed, ret=%d state=%lu err=0x%08lX\r\n",
               name,
               (int)ret,
               (unsigned long)huart->RxState,
               (unsigned long)huart->ErrorCode);
    }
}

static void camera_to_arm_coordinates_local(float cam_x, float cam_y, float cam_z,
                                            float* arm_x, float* arm_y, float* arm_z) {
    if (arm_x == NULL || arm_y == NULL || arm_z == NULL) {
        return;
    }

    // 串口接收的 cam_x/cam_y/cam_z 单位为 cm，这里统一转换为机械臂使用的 mm。
    *arm_x = cam_x * K_COORD_SCALE_CM_TO_MM + K_ARM_X_OFFSET_MM;
    *arm_y = -cam_z * K_COORD_SCALE_CM_TO_MM * K_COS_TILT + K_ARM_Y_OFFSET_MM + MOTION_Y_CALIB_BIAS_MM;
    *arm_z = -cam_y * K_COORD_SCALE_CM_TO_MM * K_COS_TILT + K_ARM_Z_OFFSET_MM;
}

/* =========================================================
 * 中断总入口
 * ========================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    if (UartHandle->Instance == USART1) {
        // vofa 实时调参
        uart_RX_decode(&huart1, &vofa_handler);
    }
    else if (UartHandle->Instance == USART2) {
        // 视觉 机械臂末端位置通信
        uart_RX_decode(&huart2, &vision_handler);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    if (UartHandle == NULL) {
        return;
    }

    if (UartHandle->Instance == USART1) {
        printf("[UART1] error=0x%08lX\r\n", (unsigned long)UartHandle->ErrorCode);
        __HAL_UART_CLEAR_OREFLAG(UartHandle);
        __HAL_UART_CLEAR_NEFLAG(UartHandle);
        __HAL_UART_CLEAR_FEFLAG(UartHandle);
        __HAL_UART_CLEAR_PEFLAG(UartHandle);
        vofa_handler.buffer_index = 0;
        vofa_handler.header_found = false;
        uart_rearm_rx(&huart1, &vofa_handler, "USART1");
    } else if (UartHandle->Instance == USART2) {
        printf("[UART2] error=0x%08lX\r\n", (unsigned long)UartHandle->ErrorCode);
        __HAL_UART_CLEAR_OREFLAG(UartHandle);
        __HAL_UART_CLEAR_NEFLAG(UartHandle);
        __HAL_UART_CLEAR_FEFLAG(UartHandle);
        __HAL_UART_CLEAR_PEFLAG(UartHandle);
        vision_handler.buffer_index = 0;
        vision_handler.header_found = false;
        uart_rearm_rx(&huart2, &vision_handler, "USART2");
    }
}

/* =========================================================
 * 回调函数 A：处理 USART1 (VOFA 调参)
 * ========================================================= */
void VOFA_Callback(uint8_t* buf) {
    uint32_t raw_bits = (buf[5]<<24) | (buf[4]<<16) | (buf[3]<<8) | buf[2];
    float value;
    memcpy(&value, &raw_bits, 4);

    switch (buf[1]) {
    case 0x10: target_x = value; break;
    case 0x11: target_y = value; break;
    case 0x12: target_z = value; break;
    case 0x20:
        if (raw_bits == 1) { pump_state = 1; Pump_On(); }
        else { pump_state = 0; Pump_Off(); }
        break;
    }
}

/* =========================================================
 * 回调函数 B：处理 USART2 (视觉数据)
 * ========================================================= */
void Vision_Callback(uint8_t* buf) {
    // 校验和验证 (buf[0..16]是数据, buf[17]是校验)
    uint8_t checksum = 0;
    for (int i = 0; i < 17; i++) checksum += buf[i];
    if (checksum != buf[17]) return; // 校验失败直接退出

    // 1. 解析当前坐标
    float cur_v_x = bytes_to_float_big_endian(&buf[5]);
    float cur_v_y = bytes_to_float_big_endian(&buf[9]);
    float cur_v_z = bytes_to_float_big_endian(&buf[13]);

    camera_raw_x = cur_v_x;
    camera_raw_y = cur_v_y;
    camera_raw_z = cur_v_z;

    // 2. 从相机坐标系转换到机械臂坐标系
    float cur_target_x = 0.0f;
    float cur_target_y = 0.0f;
    float cur_target_z = 0.0f;
    camera_to_arm_coordinates_local(cur_v_x, cur_v_y, cur_v_z,
                                    &cur_target_x, &cur_target_y, &cur_target_z);

    // 3. 逻辑处理
    if (current_state == STATE_IDLE) {
        // 如果机械臂当前没事干
        target_x = cur_target_x;
        target_y = cur_target_y;
        target_z = cur_target_z;

        if (system_work_mode == 1) {
            current_state = STATE_MOVE_TO_BOX1;
        }
    }
}

/* =========================================================
 * 协议初始化配置
 * ========================================================= */
ProtocolHandler vofa_handler;
ProtocolHandler vision_handler;

void My_UART_Init(void) {
    // USART1: VOFA (7字节, 头0xFF)
    vofa_handler.buffer = vofa_buf;
    vofa_handler.package_length = 7;
    vofa_handler.header = 0xFF;
    vofa_handler.header_length = 1;
    vofa_handler.tail_flag = false;
    vofa_handler.callback = VOFA_Callback;
    vofa_handler.buffer_index = 0;
    vofa_handler.header_found = false;
    uart_rearm_rx(&huart1, &vofa_handler, "USART1");

    // USART2: Vision (19字节, 头0xAA55, 尾0x0D)
    vision_handler.buffer = vision_buf;
    vision_handler.package_length = 19;
    vision_handler.header = 0x55AA; // 小端存储在寄存器，匹配时等效于 0xAA 0x55
    vision_handler.header_length = 2;
    vision_handler.tail_flag = true;
    vision_handler.tail = 0x0D;
    vision_handler.callback = Vision_Callback;
    vision_handler.buffer_index = 0;
    vision_handler.header_found = false;
    uart_rearm_rx(&huart2, &vision_handler, "USART2");

}

/* =========================================================
 * 核心：通用解析状态机
 * ========================================================= */
void uart_RX_decode(UART_HandleTypeDef *huart, ProtocolHandler *ph) {
    if (!ph->header_found) { // 寻找帧头
        ph->buffer[ph->buffer_index] = ph->receive_byte; // // 当前收到的字节存入 buffer[0]，buffer[1]
        ph->buffer_index++;

        // 匹配帧头
        bool match = true;
        for (int i = 0; i < ph->buffer_index; i++) {
            uint8_t target_head = (ph->header >> (i * 8)) & 0xFF;
            if (ph->buffer[i] != target_head) {
                match = false;
                break;
            }
        }

        if (match && ph->buffer_index == ph->header_length) {
            ph->header_found = true; // 找到了完整帧头
            ph->buffer_index = 0; // 接下来开始存header之后的内容
        } else if (!match) {
            ph->buffer_index = 0;
        }
    }
    else { //匹配上帧头，开始从 buffer[2] 存数据
        // 填充帧头之后的数据
        ph->buffer[ph->header_length + ph->buffer_index] = ph->receive_byte;
        ph->buffer_index++;

        // 填满了整个包 (总长度 package_length)
        if (ph->buffer_index == (ph->package_length - ph->header_length)) {
            bool tail_ok = true;
            // 如果协议规定了有帧尾，检查最后一个字节对不对
            if (ph->tail_flag) {
                if (ph->buffer[ph->package_length - 1] != ph->tail) tail_ok = false;
            }
            // 这包数据完整了，交给对应的函数去处理
            if (tail_ok) {
                ph->callback(ph->buffer);
            }
            // 复位状态机，准备找下一帧
            ph->buffer_index = 0;
            ph->header_found = false;
        }
    }
    HAL_UART_Receive_IT(huart, &ph->receive_byte, 1);
}
