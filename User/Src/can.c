// can.c 只能做三件事：启停 CAN，，收发 CAN 帧，把“原始数据”交出去

#include "can.h"
#include "DrEmpower_can.h" /* 需要电机库的全局变量 */
#include "dm_imu.h"       /* 陀螺仪CAN驱动 */
#include "string.h"
#include "fdcan.h"

// --- 正确：只定义我们需要的新变量 ---
uint8_t f4_rx_buf[8] = {0};
uint8_t f4_new_msg_flag = 0;


// CAN接收回调函数（中断中调用）
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan2){
        FDCAN_RxHeaderTypeDef RxHeader2;
        uint8_t local_rx_buf[8];
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader2, local_rx_buf);

        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE){
            switch (RxHeader2.Identifier) {
                case 0x302:{
                    printf("get\n");
                    memcpy(f4_rx_buf, local_rx_buf, 8); // 将数据存入全局数组
                    f4_new_msg_flag = 1;
                    break;
                }
                default:
                    break;
            }
        }
    }


    if (hfdcan == &hfdcan1){
        FDCAN_RxHeaderTypeDef RxHeader1;
        uint8_t local_rx_buf[8];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader1, local_rx_buf);
        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE){
            // 区分消息类型：陀螺仪 vs 电机反馈
            // 达妙 IMU 在应答模式下，默认返回的 ID 是你设置的 MST_ID (我们在 main 中设置为 0x11)
            // 在主动输出模式下，ID 是设置的 CAN_ID (0x01)
            if (RxHeader1.Identifier == 0x11 || RxHeader1.Identifier == 0x01) {
                // ================== 陀螺仪数据 ==================
                IMU_UpdateData(local_rx_buf); // 调用达妙官方解析函数
            } 
            else {
                // ================== 电机反馈数据 ==================
                memcpy(rx_buffer, local_rx_buf, 8);
                can_id = RxHeader1.Identifier;
                READ_FLAG = 1;
            }
        }
    }
}

