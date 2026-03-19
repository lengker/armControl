//
// Created by Lenovo on 2026/3/12.
//
#include "cmd.h"

// 外部声明 CubeMX 生成的句柄
extern FDCAN_HandleTypeDef hfdcan2;
// 定义一个静态的头部，这样它就在内存里一直存在，不用每次去写赋值语句
static FDCAN_TxHeaderTypeDef F4_TxHeader = {
    .Identifier = 0x301,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0
};


void CAN_G4_Send_To_F4(uint8_t robot_state, uint8_t grab_cmd, int16_t x, int16_t y) {
    uint8_t TxData[8];
    // 严格按照队友的解析逻辑打包 (高位在前，低位在后)
    TxData[0] = robot_state;
    TxData[1] = grab_cmd;
    TxData[2] = (uint8_t)(x >> 8);   // target_x 高8位
    TxData[3] = (uint8_t)(x & 0xFF); // target_x 低8位
    TxData[4] = (uint8_t)(y >> 8);   // target_y 高8位
    TxData[5] = (uint8_t)(y & 0xFF); // target_y 低8位
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    // 直接发送
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &F4_TxHeader, TxData);
}



// // 实例化全局变量，专门用来存储 G4 发来的数据
// G4_To_F4_Data_t g4_rx_data = {0};
//
// /**
//  * @brief  F4 解析从 G4 接收到的数据 (ID = 0x301)
//  * @param  std_id 收到的 CAN ID
//  * @param  rx_data 收到的 8 字节数据指针
//  * @note   这个函数将在你的 CAN 接收回调中断里被调用
//  */
// // 在 cmd.c 中，修改解析函数：
// void CAN_F4_Parse_G4_Data(uint8_t *rx_data) {
//     // 因为在 can.c 的回调里已经确保了只有 0x301 才会进这个函数，
//     // 所以这里直接闭眼拆包即可，不需要再判断 if (std_id == 0x301)
//
//     g4_rx_data.robot_state = rx_data[0];
//     g4_rx_data.grab_cmd    = rx_data[1];
//
//     g4_rx_data.target_x = (int16_t)((rx_data[2] << 8) | rx_data[3]);
//     g4_rx_data.target_y = (int16_t)((rx_data[4] << 8) | rx_data[5]);
// }
//
// /**
//  * @brief  F4 发送状态反馈给 G4 (ID = 0x302)
//  * @param  arm_state 机械臂状态 (0:空闲, 1:正在抓, 2:故障)
//  * @param  task_finished 任务是否完成 (0:未完成, 1:抓取完成, 2:放置完成)
//  * @param  can_robot_move 机器人是否该移动 (0:等待不要动, 1:可以移动)
//  */
// void CAN_F4_Send_To_G4(uint8_t arm_state, uint8_t task_finished, uint8_t can_robot_move) {
//     CAN_TxHeaderTypeDef tx_header;
//     uint8_t tx_data[8] = {0};
//     uint32_t tx_mailbox;
//
//     // 1. 配置 F4 -> G4 的标准帧头
//     tx_header.StdId = 0x302;          // F4 专属发送 ID
//     tx_header.IDE = CAN_ID_STD;       // 标准帧
//     tx_header.RTR = CAN_RTR_DATA;     // 数据帧
//     tx_header.DLC = 8;                // 8字节长度
//     tx_header.TransmitGlobalTime = DISABLE;
//
//     // 2. 装载数据
//     tx_data[0] = arm_state;
//     tx_data[1] = task_finished;
//     tx_data[2] = can_robot_move;
//     // tx_data[3] 到 tx_data[7] 默认为 0，作为预留位给以后可能的新需求
//
//     // 3. 调用 HAL 库底层发送，避开大然的发送函数以防冲突
//     HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
// }

