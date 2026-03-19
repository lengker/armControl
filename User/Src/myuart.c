#include "myuart.h"

#include <string.h>
#include "stdio.h"

// 引用外部串口句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

uint8_t Data[7];
uint32_t Data1;

float target_x = 30.0f;   // 默认 x 坐标 (cm)
float target_y = 0.0f;    // 默认 y 坐标 (cm)
float target_z = 20.0f;   // 默认 z 坐标 (cm)

// 2. 视觉通信专用变量
uint8_t vision_rx_byte;           // 每次接收1个字节的缓存
uint8_t vision_rx_buffer[13];     // 存满一帧13个字节的数组
uint8_t vision_rx_index = 0;      // 当前接收到第几个字节了

// 3. 合并后的串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    // ============================================
    // 情况 A：这是 USART1 的中断 (实时调参 VOFA)
    // ============================================
    if(UartHandle->Instance == USART1)
    {
        if(Data[0] == 0xFF) // 帧头
        {
            switch (Data[1]) {
                case 0x10:   // 设置 target_x
                    Data1 = Data[5]<<24 | Data[4]<<16 | Data[3]<<8 | Data[2];
                    memcpy(&target_x, &Data1, sizeof(float));
                    break;
                case 0x11:   // 设置 target_y
                    Data1 = Data[5]<<24 | Data[4]<<16 | Data[3]<<8 | Data[2];
                    memcpy(&target_y, &Data1, sizeof(float));
                    break;
                case 0x12:   // 设置 target_z
                    Data1 = Data[5]<<24 | Data[4]<<16 | Data[3]<<8 | Data[2];
                    memcpy(&target_z, &Data1, sizeof(float));
                    // printf("target_z updated to %f\r\n", target_z);
                    break;
            }
        }
        HAL_UART_Receive_IT(&huart1, Data, 7);   // 重新开启 USART1 接收
    }

    // ============================================
    // 情况 B：这是 USART2 的中断 (视觉发来的数据)
    // ============================================
    else if(UartHandle->Instance == USART2)
    {
        // 1) 存入当前字节
        vision_rx_buffer[vision_rx_index] = vision_rx_byte;

        // 2) 状态机解析（防错位机制）
        if (vision_rx_index == 0 && vision_rx_buffer[0] != 0xAA) {
            vision_rx_index = 0;
        }
        else if (vision_rx_index == 1 && vision_rx_buffer[1] != 0x55) {
            vision_rx_index = 0;
        }
        else {
            vision_rx_index++;

            // 3) 凑齐 13 个字节，开始解包！
            if (vision_rx_index == 13)
            {
                uint8_t checksum = 0;
                for (int i = 0; i < 11; i++) {
                    checksum += vision_rx_buffer[i];
                }

                // 校验数据合法性
                if (vision_rx_buffer[2] == 0x01 &&
                    vision_rx_buffer[3] == 0x10 &&
                    checksum == vision_rx_buffer[11] &&
                    vision_rx_buffer[12] == 0x0D)
                {
                    // 提取 XYZ (大端模式)
                    int16_t parsed_x = (int16_t)((vision_rx_buffer[5] << 8) | vision_rx_buffer[6]);
                    int16_t parsed_y = (int16_t)((vision_rx_buffer[7] << 8) | vision_rx_buffer[8]);
                    int16_t parsed_z = (int16_t)((vision_rx_buffer[9] << 8) | vision_rx_buffer[10]);

                    // 更新全局坐标
                    target_x = (float)parsed_x;
                    target_y = (float)parsed_y;
                    target_z = (float)parsed_z;

                    printf("[收到视觉数据] 更新坐标: X:%.1f, Y:%.1f, Z:%.1f\r\n", target_x, target_y, target_z);
                }
                vision_rx_index = 0; // 归零，准备接收下一帧
            }
        }
        //处理完毕，等待下个收到的字节，凑齐13字节后一起解包算出XYZ 赋值给 target_x/y/z
        HAL_UART_Receive_IT(&huart2, &vision_rx_byte, 1); // 重新开启 USART2 接收
    }
}
