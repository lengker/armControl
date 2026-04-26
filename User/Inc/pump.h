#ifndef PUMP_H
#define PUMP_H

#include "main.h" // 包含 GPIO 定义

// 初始化气泵引脚 (如果你已经在 CubeMX 里配置了 PB10，可以不调这个，但留着接口更规范)
void Pump_Init(void);

// 开启气泵 (PB10 拉高)
void Pump_On(void);

// 关闭气泵 (PB10 拉低)
void Pump_Off(void);

#endif // PUMP_H