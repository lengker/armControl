// can.c 只能做三件事：启停 CAN，，收发 CAN 帧，把“原始数据”交出去

#include "can.h"
#include "DrEmpower_can.h" /* 需要电机库的全局变量 */
#include "string.h"
#include "fdcan.h"

// --- 正确：只定义我们需要的新变量 ---
uint8_t f4_rx_buf[8] = {0};
uint8_t f4_new_msg_flag = 0;


// // CAN接收回调函数（中断中调用）
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
//     // 大然库里的 get_angle, get_state 等函数会一直死等 READ_FLAG == 1。你必须在中断里给它赋值
//     if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
//     {
//         FDCAN_RxHeaderTypeDef RxHeader;
//         uint8_t local_rx_buf[8];
//         // 接收数据并存入库的 rx_buffer
//         if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, local_rx_buf) == HAL_OK)
//         {
//             // 情况 A: 来自 FDCAN1 (电机反馈)
//             if (hfdcan->Instance == FDCAN1) {
//                 memcpy(rx_buffer, local_rx_buf, 8);
//                 can_id = RxHeader.Identifier;
//                 READ_FLAG = 1; // 解除库函数的死等状态
//             }
//
//             // 情况 B: 来自 FDCAN2 (队友 F4 的机械臂状态反馈)
//             else if (hfdcan->Instance == FDCAN2) {
//                 printf("get\n");
//                 uint8_t temp_buf[8];
//                 // 检查 F4 发给 G4 的 ID: 0x302 (标准帧)
//                 if (RxHeader.IdType == FDCAN_STANDARD_ID && RxHeader.Identifier == 0x302) {
//                     // 修正：直接从 local_rx_buf 拷贝数据
//                     memcpy(f4_rx_buf, local_rx_buf, 8);
//                     f4_new_msg_flag = 1;
//                 }
//             }
//         }
//     }
// }

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
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader1, rx_buffer);
        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE){
            can_id = RxHeader1.Identifier;
            READ_FLAG = 1;
        }
    }
}

