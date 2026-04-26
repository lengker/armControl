#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

void vApplicationMallocFailedHook(void) {
    // Called if pvPortMalloc() fails
    printf("[RTOS] malloc failed\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    printf("[RTOS] stack overflow: %s\r\n", pcTaskName ? pcTaskName : "unknown");
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

