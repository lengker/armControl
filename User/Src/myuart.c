#include "myuart.h"
#include "types.h"
#include "logger.h"
#include "config.h"
#include "pump.h"
#include "motion.h"
#include "state_machine.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define TAG "UART"

// 引用外部串口句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* ========== 全局系统上下文 ========== */
SystemContext_t g_system = {
    .work_mode = WORK_MODE_AUTO,
    .sm = {
        .current_state = STATE_IDLE,
        .prev_state = STATE_IDLE,
        .state_enter_time = 0,
        .state_duration = 0,
        .retry_count = 0,
        .timeout_flag = false
    },
    .camera = {
        .raw = {0, 0, 0},
        .converted = {0, 0, 0},
        .timestamp = 0,
        .valid = false
    },
    .arm_target = {
        .target = {235.0f, 0.0f, 240.0f},
        .pitch = PICK_PITCH_DEG,
        .timestamp = 0
    },
    .mission = {
        .box = {0, 0, 0},
        .place = {PLACE_POINT_X_MM, PLACE_POINT_Y_MM, PLACE_POINT_Z_MM}
    },
    .status = {
        .pump_on = false,
        .motion_active = false,
        .emergency_stop = false
    }
};

/* ========== 缓冲区定义 ========== */
uint8_t vofa_buf[10];
uint8_t vision_buf[25];
uint8_t text_cmd_buf[64];  // 文本命令缓冲区
uint8_t text_cmd_index = 0;

/* ========== 协议处理器实例 ========== */
ProtocolHandler vofa_handler;
ProtocolHandler vision_handler;

/* ========== 内部辅助函数：大端转float ========== */
static float bytes_to_float_big_endian(uint8_t* b) {
    uint32_t u = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                 ((uint32_t)b[2] << 8)  | ((uint32_t)b[3]);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

/**
 * @brief 文本命令解析器
 * @param[in] cmd 命令字符串
 * @note 支持的命令：
 *       - set <motor_id> <angle>  : 控制单个电机
 *       - mode <auto|test|follow> : 切换工作模式
 *       - pump <on|off>           : 控制气泵
 *       - reset                   : 软件复位
 */
static void parse_text_command(char* cmd) {
    char* token;
    char cmd_copy[64];
    strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';
    
    // 获取第一个单词（命令）
    token = strtok(cmd_copy, " \r\n");
    if (token == NULL) return;
    
    // set 命令：set <motor_id> <angle>
    if (strcmp(token, "set") == 0) {
        char* motor_str = strtok(NULL, " \r\n");
        char* angle_str = strtok(NULL, " \r\n");
        
        if (motor_str && angle_str) {
            int motor_id = atoi(motor_str);
            float angle = atof(angle_str);
            
            // 电机ID: 1=底座, 2=大臂, 3=小臂
            if (motor_id >= 1 && motor_id <= 3) {
                printf("[CMD] Set motor %d to %.1f°\r\n", motor_id, angle);
                Motion_TestSingleMotor(motor_id, angle);
            } else {
                printf("[CMD] Error: Invalid motor ID (1=base, 2=big, 3=small)\r\n");
            }
        } else {
            printf("[CMD] Usage: set <motor_id> <angle>\r\n");
            printf("[CMD] Motor IDs: 1=base, 2=big, 3=small\r\n");
        }
    }
    // mode 命令：mode <auto|test|follow>
    else if (strcmp(token, "mode") == 0) {
        char* mode_str = strtok(NULL, " \r\n");
        
        if (mode_str) {
            if (strcmp(mode_str, "auto") == 0) {
                g_system.work_mode = WORK_MODE_AUTO;
                printf("[CMD] Switched to AUTO mode\r\n");
            } else if (strcmp(mode_str, "test") == 0) {
                g_system.work_mode = WORK_MODE_TEST;
                printf("[CMD] Switched to TEST mode\r\n");
            } else if (strcmp(mode_str, "follow") == 0) {
                g_system.work_mode = WORK_MODE_FOLLOW;
                printf("[CMD] Switched to FOLLOW mode\r\n");
            } else {
                printf("[CMD] Error: Unknown mode (auto|test|follow)\r\n");
            }
        } else {
            printf("[CMD] Usage: mode <auto|test|follow>\r\n");
        }
    }
    // pump 命令：pump <on|off>
    else if (strcmp(token, "pump") == 0) {
        char* state_str = strtok(NULL, " \r\n");
        
        if (state_str) {
            if (strcmp(state_str, "on") == 0) {
                Pump_On();
                g_system.status.pump_on = true;
                printf("[CMD] Pump ON\r\n");
            } else if (strcmp(state_str, "off") == 0) {
                Pump_Off();
                g_system.status.pump_on = false;
                printf("[CMD] Pump OFF\r\n");
            } else {
                printf("[CMD] Error: Unknown state (on|off)\r\n");
            }
        } else {
            printf("[CMD] Usage: pump <on|off>\r\n");
        }
    }
    // reset 命令
    else if (strcmp(token, "reset") == 0) {
        printf("[CMD] Software reset...\r\n");
        osDelay(100);
        NVIC_SystemReset();
    }
    // help 命令
    else if (strcmp(token, "help") == 0) {
        printf("\r\n=== Available Commands ===\r\n");
        printf("set <motor_id> <angle>  - Control single motor\r\n");
        printf("                          1=base, 2=big_arm, 3=small_arm\r\n");
        printf("mode <auto|test|follow> - Switch work mode\r\n");
        printf("pump <on|off>           - Control pump\r\n");
        printf("reset                   - Software reset\r\n");
        printf("help                    - Show this help\r\n");
        printf("==========================\r\n\r\n");
    }
    // 未知命令
    else {
        printf("[CMD] Unknown command: %s (type 'help' for commands)\r\n", token);
    }
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

/* =========================================================
 * 中断总入口
 * ========================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    if (UartHandle->Instance == USART1) {
        uint8_t byte = vofa_handler.receive_byte;
        
        // 判断是文本命令还是二进制协议
        if (byte == 0xFF) {
            // 二进制VOFA协议（以0xFF开头）
            uart_RX_decode(&huart1, &vofa_handler);
        } else {
            // 文本命令（以ASCII字符开头）
            if (byte == '\n' || byte == '\r') {
                // 命令结束，执行解析
                if (text_cmd_index > 0) {
                    text_cmd_buf[text_cmd_index] = '\0';
                    parse_text_command((char*)text_cmd_buf);
                    text_cmd_index = 0;
                }
            } else if (text_cmd_index < sizeof(text_cmd_buf) - 1) {
                // 累积字符
                text_cmd_buf[text_cmd_index++] = byte;
            } else {
                // 缓冲区满，重置
                text_cmd_index = 0;
                printf("[CMD] Error: Command too long\r\n");
            }
            
            // 重新启动接收
            HAL_UART_Receive_IT(&huart1, &vofa_handler.receive_byte, 1);
        }
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
        printf("[UART1] error=0x%08lX state=%lu\r\n", 
               (unsigned long)UartHandle->ErrorCode,
               (unsigned long)UartHandle->RxState);
        
        // 清除所有错误标志
        __HAL_UART_CLEAR_OREFLAG(UartHandle);
        __HAL_UART_CLEAR_NEFLAG(UartHandle);
        __HAL_UART_CLEAR_FEFLAG(UartHandle);
        __HAL_UART_CLEAR_PEFLAG(UartHandle);
        
        // 重置状态机
        vofa_handler.buffer_index = 0;
        vofa_handler.header_found = false;
        
        // 重新启动接收
        uart_rearm_rx(&huart1, &vofa_handler, "USART1");
        
    } else if (UartHandle->Instance == USART2) {
        printf("[UART2] error=0x%08lX state=%lu\r\n", 
               (unsigned long)UartHandle->ErrorCode,
               (unsigned long)UartHandle->RxState);
        
        // 清除所有错误标志
        __HAL_UART_CLEAR_OREFLAG(UartHandle);
        __HAL_UART_CLEAR_NEFLAG(UartHandle);
        __HAL_UART_CLEAR_FEFLAG(UartHandle);
        __HAL_UART_CLEAR_PEFLAG(UartHandle);
        
        // 重置状态机
        vision_handler.buffer_index = 0;
        vision_handler.header_found = false;
        
        // 重新启动接收
        uart_rearm_rx(&huart2, &vision_handler, "USART2");
    }
}

/**
 * @brief VOFA调参协议回调函数
 * @param[in] buf 接收到的数据包
 * @note 协议格式：0xFF + ID + 4字节float
 */
void VOFA_Callback(uint8_t* buf) {
    uint32_t raw_bits = (buf[5]<<24) | (buf[4]<<16) | (buf[3]<<8) | buf[2];
    float value;
    memcpy(&value, &raw_bits, 4);

    switch (buf[1]) {
    case 0x10: 
        g_system.arm_target.target.x = value;
        LOG_D(TAG, "VOFA: target_x = %.1f", value);
        break;
    case 0x11: 
        g_system.arm_target.target.y = value;
        LOG_D(TAG, "VOFA: target_y = %.1f", value);
        break;
    case 0x12: 
        g_system.arm_target.target.z = value;
        LOG_D(TAG, "VOFA: target_z = %.1f", value);
        break;
    case 0x20:
        if (raw_bits == 1) { 
            Pump_On();
            g_system.status.pump_on = true;
            LOG_I(TAG, "VOFA: Pump ON");
        } else { 
            Pump_Off();
            g_system.status.pump_on = false;
            LOG_I(TAG, "VOFA: Pump OFF");
        }
        break;
    case 0x30:  // 测试模式：控制单个电机
        // 格式：0xFF 0x30 motor_id angle(float)
        // motor_id在buf[2]，angle在buf[3-6]
        {
            uint8_t motor_id = buf[2];
            uint32_t angle_bits = (buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | buf[3];
            float angle;
            memcpy(&angle, &angle_bits, 4);
            LOG_I(TAG, "VOFA: Test mode - Motor %d -> %.1f°", motor_id, angle);
            Motion_TestSingleMotor(motor_id, angle);
        }
        break;
    case 0x31:  // 切换到测试模式
        g_system.work_mode = WORK_MODE_TEST;
        LOG_I(TAG, "VOFA: Switched to TEST mode");
        break;
    case 0x32:  // 切换到自动模式
        g_system.work_mode = WORK_MODE_AUTO;
        LOG_I(TAG, "VOFA: Switched to AUTO mode");
        break;
    case 0x33:  // 切换到跟随模式
        g_system.work_mode = WORK_MODE_FOLLOW;
        LOG_I(TAG, "VOFA: Switched to FOLLOW mode");
        break;
    case 0x3F:  // 软件复位（用于测试）
        LOG_I(TAG, "VOFA: Software reset requested");
        osDelay(100);  // 等待打印完成
        NVIC_SystemReset();  // 触发软件复位
        break;
    default:
        LOG_W(TAG, "VOFA: Unknown ID 0x%02X", buf[1]);
        break;
    }
}

/**
 * @brief 视觉数据协议回调函数
 * @param[in] buf 接收到的数据包
 * @note 协议格式：0xAA 0x55 + 3个字节 + 3个float(12字节) + 校验和 + 0x0D
 *       数据内容：相机坐标系下的目标位置 (单位: cm)
 */
void Vision_Callback(uint8_t* buf) {
    // 校验和验证 (buf[0..16]是数据, buf[17]是校验)
    uint8_t checksum = 0;
    for (int i = 0; i < 17; i++) checksum += buf[i];
    
    if (checksum != buf[17]) {
        LOG_W(TAG, "Vision: Checksum error! calc=0x%02X recv=0x%02X", checksum, buf[17]);
        return; // 校验失败直接退出
    }

    // 1. 解析相机坐标 (大端序float)
    float cam_x = bytes_to_float_big_endian(&buf[5]);
    float cam_y = bytes_to_float_big_endian(&buf[9]);
    float cam_z = bytes_to_float_big_endian(&buf[13]);

    // 保存原始相机数据
    g_system.camera.raw.x = cam_x;
    g_system.camera.raw.y = cam_y;
    g_system.camera.raw.z = cam_z;
    g_system.camera.timestamp = osKernelGetTickCount();
    g_system.camera.valid = true;

    // 2. 从相机坐标系转换到机械臂坐标系
    float arm_x, arm_y, arm_z;
    Calculate_CameraToArm(cam_x, cam_y, cam_z, &arm_x, &arm_y, &arm_z);
    
    // 保存转换后的坐标
    g_system.camera.converted.x = arm_x;
    g_system.camera.converted.y = arm_y;
    g_system.camera.converted.z = arm_z;

    // 打印接收到的原始数据
    printf("\r\n[Vision] 收到原始数据: Cam(%.1f, %.1f, %.1f)cm\r\n", 
           cam_x, cam_y, cam_z);
    printf("[Vision] 转换后目标: Arm(%.1f, %.1f, %.1f)mm\r\n", 
           arm_x, arm_y, arm_z);

    // 3. 逻辑处理
    if (g_system.sm.current_state == STATE_IDLE) {
        // 如果机械臂当前没事干
        if (g_system.work_mode == WORK_MODE_FOLLOW) {
            // 跟随模式：直接更新目标坐标
            g_system.arm_target.target.x = arm_x;
            g_system.arm_target.target.y = arm_y;
            g_system.arm_target.target.z = arm_z;
            g_system.arm_target.timestamp = osKernelGetTickCount();
            LOG_D(TAG, "Follow mode: target updated");
            
        } else if (g_system.work_mode == WORK_MODE_AUTO) {
            // 自动模式：保存箱子坐标并触发抓取流程
            g_system.mission.box.x = arm_x;
            g_system.mission.box.y = arm_y;
            g_system.mission.box.z = arm_z;
            
            LOG_I(TAG, "Auto mode: Box detected, starting mission");
            StateMachine_Transition(STATE_MOVE_TO_BOX1);
        }
    } else {
        LOG_D(TAG, "Vision data received but system busy (state=%s)", 
              StateMachine_GetStateName(g_system.sm.current_state));
    }
}

/**
 * @brief 初始化串口协议处理器
 * @note 配置VOFA和视觉协议的参数，并启动中断接收
 */
void My_UART_Init(void) {
    LOG_I(TAG, "Initializing UART protocols");
    
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
    
    LOG_I(TAG, "UART protocols initialized");
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
