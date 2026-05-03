#include "pump.h"

void Pump_Init(void) {
    Pump_Off();
    HAL_Delay(50);
}

void Pump_On(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}

void Pump_Off(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}