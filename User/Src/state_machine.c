/**
 ******************************************************************************
 * @file    state_machine.c
 * @brief   自动抓取流程状态机实现
 * @author  Your Team
 * @date    2026-05-03
 ******************************************************************************
 */

#include "state_machine.h"
#include "logger.h"
#include "config.h"
#include "motion.h"
#include "pump.h"
#include "cmsis_os2.h"

/* ========== 日志标签 ========== */
#define TAG "StateMachine"

/* ========== 外部全局变量 ========== */
extern SystemContext_t g_system;

/* ========== 状态超时配置表 ========== */
static const uint32_t state_timeout_table[] = {
    [STATE_IDLE]          = SM_TIMEOUT_IDLE_MS,
    [STATE_MOVE_TO_BOX1]  = SM_TIMEOUT_MOVE_MS,
    [STATE_GRAB_BOX1]     = SM_TIMEOUT_GRAB_MS,
    [STATE_LIFT_BOX1]     = SM_TIMEOUT_LIFT_MS,
    [STATE_MOVE_TO_PLACE] = SM_TIMEOUT_PLACE_MS,
    [STATE_RELEASE_ALL]   = SM_TIMEOUT_RELEASE_MS,
    [STATE_ERROR]         = 0xFFFFFFFF,
};

/* ========== 状态名称字符串表 ========== */
static const char* state_name_table[] = {
    [STATE_IDLE]          = "IDLE",
    [STATE_MOVE_TO_BOX1]  = "MOVE_TO_BOX",
    [STATE_GRAB_BOX1]     = "GRAB_BOX",
    [STATE_LIFT_BOX1]     = "LIFT_BOX",
    [STATE_MOVE_TO_PLACE] = "MOVE_TO_PLACE",
    [STATE_RELEASE_ALL]   = "RELEASE_BOX",
    [STATE_ERROR]         = "ERROR",
};

/**
 * @brief 初始化状态机
 */
void StateMachine_Init(void) {
    g_system.sm.current_state = STATE_IDLE;
    g_system.sm.prev_state = STATE_IDLE;
    g_system.sm.state_enter_time = osKernelGetTickCount();
    g_system.sm.state_duration = 0;
    g_system.sm.retry_count = 0;
    g_system.sm.timeout_flag = false;
    
    LOG_I(TAG, "State machine initialized");
}

/**
 * @brief 状态机主循环
 */
void StateMachine_Run(void) {
    // 检查工作模式
    if (g_system.work_mode != WORK_MODE_AUTO) {
        if (g_system.sm.current_state != STATE_IDLE) {
            LOG_W(TAG, "Not in AUTO mode, resetting to IDLE");
            StateMachine_Reset(true);
        }
        osDelay(20);
        return;
    }
    
    // 检查紧急停止
    if (g_system.status.emergency_stop) {
        LOG_E(TAG, "Emergency stop activated!");
        StateMachine_Transition(STATE_ERROR);
        return;
    }
    
    // 更新状态持续时间
    g_system.sm.state_duration = osKernelGetTickCount() - g_system.sm.state_enter_time;
    
    // 检查超时
    if (StateMachine_CheckTimeout()) {
        LOG_E(TAG, "State timeout! State=%s Duration=%lums", 
              StateMachine_GetStateName(g_system.sm.current_state),
              (unsigned long)g_system.sm.state_duration);
        
        if (StateMachine_IncrementRetry() >= SM_MAX_RETRY_COUNT) {
            LOG_E(TAG, "Max retry reached, entering ERROR state");
            StateMachine_Transition(STATE_ERROR);
            return;
        } else {
            LOG_W(TAG, "Retry %d/%d", g_system.sm.retry_count, SM_MAX_RETRY_COUNT);
            StateMachine_Reset(false);
            return;
        }
    }
    
    // 执行当前状态
    switch (g_system.sm.current_state) {
        case STATE_IDLE:
            State_Idle();
            break;
        case STATE_MOVE_TO_BOX1:
            State_MoveToBox();
            break;
        case STATE_GRAB_BOX1:
            State_GrabBox();
            break;
        case STATE_LIFT_BOX1:
            State_LiftBox();
            break;
        case STATE_MOVE_TO_PLACE:
            State_MoveToPlace();
            break;
        case STATE_RELEASE_ALL:
            State_ReleaseBox();
            break;
        case STATE_ERROR:
            State_Error();
            break;
        default:
            LOG_E(TAG, "Unknown state: %d", g_system.sm.current_state);
            StateMachine_Transition(STATE_ERROR);
            break;
    }
}

/**
 * @brief 重置状态机到待机状态
 */
void StateMachine_Reset(bool clear_retry) {
    LOG_I(TAG, "Resetting state machine");
    
    g_system.sm.current_state = STATE_IDLE;
    g_system.sm.state_enter_time = osKernelGetTickCount();
    g_system.sm.timeout_flag = false;
    
    if (clear_retry) {
        g_system.sm.retry_count = 0;
    }
    
    // 确保气泵关闭
    Pump_Off();
    g_system.status.pump_on = false;
}

/**
 * @brief 触发状态转换
 */
void StateMachine_Transition(MissionState_t new_state) {
    if (new_state == g_system.sm.current_state) {
        return; // 相同状态不转换
    }
    
    LOG_STATE(TAG, g_system.sm.current_state, new_state);
    
    g_system.sm.prev_state = g_system.sm.current_state;
    g_system.sm.current_state = new_state;
    g_system.sm.state_enter_time = osKernelGetTickCount();
    g_system.sm.state_duration = 0;
    g_system.sm.timeout_flag = false;
    
    // 成功转换到下一状态，清除重试计数
    if (new_state != STATE_ERROR && new_state != STATE_IDLE) {
        StateMachine_ClearRetry();
    }
}

/**
 * @brief 检查状态超时
 */
bool StateMachine_CheckTimeout(void) {
    uint32_t timeout = state_timeout_table[g_system.sm.current_state];
    
    if (timeout == 0xFFFFFFFF) {
        return false; // 无超时限制
    }
    
    if (g_system.sm.state_duration > timeout) {
        g_system.sm.timeout_flag = true;
        return true;
    }
    
    return false;
}

/**
 * @brief 获取状态名称字符串
 */
const char* StateMachine_GetStateName(MissionState_t state) {
    if (state < sizeof(state_name_table) / sizeof(state_name_table[0])) {
        return state_name_table[state];
    }
    return "UNKNOWN";
}

/**
 * @brief 获取当前状态持续时间
 */
uint32_t StateMachine_GetStateDuration(void) {
    return g_system.sm.state_duration;
}

/**
 * @brief 增加重试计数
 */
uint8_t StateMachine_IncrementRetry(void) {
    return ++g_system.sm.retry_count;
}

/**
 * @brief 清除重试计数
 */
void StateMachine_ClearRetry(void) {
    if (g_system.sm.retry_count > 0) {
        LOG_D(TAG, "Retry count cleared");
        g_system.sm.retry_count = 0;
    }
}

/**
 * @brief 检查是否达到最大重试次数
 */
bool StateMachine_IsMaxRetry(void) {
    return g_system.sm.retry_count >= SM_MAX_RETRY_COUNT;
}

/* ========================================================================== */
/*                           状态处理函数实现                                  */
/* ========================================================================== */

/**
 * @brief 待机状态处理
 * @note 等待视觉系统发送目标坐标
 */
void State_Idle(void) {
    // 在待机状态下，等待视觉数据触发状态转换
    // 状态转换由 Vision_Callback 触发
    osDelay(20);
}

/**
 * @brief 移动到箱子上方状态处理
 * @note 从当前位置移动到箱子上方预备点
 */
void State_MoveToBox(void) {
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║  阶段1: 移动到箱子上方                ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    // 计算预备点坐标（箱子上方 + 偏移）
    float approach_x = g_system.mission.box.x;
    float approach_y = g_system.mission.box.y;
    float approach_z = g_system.mission.box.z + PICK_APPROACH_OFFSET_Z_MM;
    
    printf("箱子位置: X=%.1f, Y=%.1f, Z=%.1f\r\n", 
          g_system.mission.box.x, g_system.mission.box.y, g_system.mission.box.z);
    printf("预备点: X=%.1f, Y=%.1f, Z=%.1f\r\n", approach_x, approach_y, approach_z);
    
    // 【新增】先抬高到安全高度（避免碰撞）
    float safe_z = 100.0f;  // 安全高度，比如Z=100mm
    if (approach_z < safe_z) {
        printf("\n→ 先移动到安全高度 Z=%.1f\r\n", safe_z);
        
        // 先在当前XY位置抬高到安全高度
        int ret = Motion_MoveToXYZ_RuckigSmooth(
            g_system.mission.box.x, 
            g_system.mission.box.y, 
            safe_z,  // 先到安全高度
            PICK_PITCH_DEG, 5.0f
        );
        
        if (ret != 0) {
            printf("[错误] 移动到安全高度失败, ret=%d\r\n", ret);
            Pump_Off();
            StateMachine_Reset(false);
            return;
        }
        
        printf("✓ 到达安全高度\r\n\n");
        osDelay(500);
    }
    
    // 然后再移动到预备点
    printf("→ 移动到预备点\r\n");
    int ret = Motion_MoveToXYZ_RuckigSmooth(
        approach_x, approach_y, approach_z, 
        PICK_PITCH_DEG, MOTION_APPROACH_DURATION_S
    );
    
    if (ret == 0) {
        printf("✓ 到达预备点\r\n");
        osDelay(500);
        StateMachine_Transition(STATE_GRAB_BOX1);
    } else {
        printf("[错误] 移动到预备点失败, ret=%d\r\n", ret);
        Pump_Off();
        StateMachine_Reset(false);
    }
}

/**
 * @brief 抓取箱子状态处理
 * @note 下压到目标点，开启气泵，然后抬升
 */
void State_GrabBox(void) {
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║  阶段2: 抓取箱子                      ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    // 2.1 下降到抓取点
    float grab_x = g_system.mission.box.x;
    float grab_y = g_system.mission.box.y;
    float grab_z = g_system.mission.box.z;
    
    printf("抓取点: X=%.1f, Y=%.1f, Z=%.1f\r\n", grab_x, grab_y, grab_z);
    printf("\n→ 下降到抓取点\r\n");
    
    int ret = Motion_MoveToXYZ_RuckigSmooth(
        grab_x, grab_y, grab_z,
        PICK_PITCH_DEG, MOTION_GRAB_DURATION_S
    );
    
    if (ret != 0) {
        printf("[错误] 下降失败\r\n");
        Pump_Off();
        StateMachine_Reset(false);
        return;
    }
    
    printf("✓ 到达抓取点\r\n");
    osDelay(500);
    
    // 2.2 开启气泵
    printf("\n→ 开启气泵\r\n");
    Pump_On();
    g_system.status.pump_on = true;
    osDelay(PUMP_ON_DELAY_MS);
    printf("✓ 气泵已开启\r\n");
    
    // 2.3 分段抬升（避免IK无解）
    // 策略：先在当前XY位置小幅抬升，然后移动到更近的位置，再继续抬升
    
    // 第一段：小幅抬升（相对当前位置+70mm）
    float lift1_x = grab_x;
    float lift1_y = grab_y;
    float lift1_z = grab_z + 70.0f;  // 先抬升50mm
    
    printf("\n→ 第一段抬升: Z=%.1f (当前位置+70mm)\r\n", lift1_z);
    
    ret = Motion_MoveToXYZ_RuckigSmooth(
        lift1_x, lift1_y, lift1_z,
        PICK_PITCH_DEG, 3.0f
    );
    
    if (ret != 0) {
        printf("[错误] 第一段抬升失败\r\n");
        Pump_Off();
        g_system.status.pump_on = false;
        StateMachine_Reset(false);
        return;
    }
    
    printf("✓ 第一段抬升完成\r\n");
    osDelay(500);
    
    // 第二段：移动到更近的XY位置（Y方向收回80mm）
    float lift2_x = grab_x;
    float lift2_y = grab_y + 30.0f;  // Y方向收回80mm（更靠近机械臂）
    float lift2_z = lift1_z + 70.0f;  // 再抬升60mm
    
    printf("\n→ 第二段抬升: Y=%.1f (收回80mm), Z=%.1f\r\n", lift2_y, lift2_z);
    
    ret = Motion_MoveToXYZ_RuckigSmooth(
        lift2_x, lift2_y, lift2_z,
        PICK_PITCH_DEG, 3.0f
    );
    
    if (ret != 0) {
        printf("[错误] 第二段抬升失败\r\n");
        Pump_Off();
        g_system.status.pump_on = false;
        StateMachine_Reset(false);
        return;
    }
    
    printf("✓ 第二段抬升完成\r\n");
    osDelay(500);
    
    // 第三段：继续抬升到安全高度
    float lift3_x = lift2_x;
    float lift3_y = lift2_y;
    float lift3_z = PICK_SAFE_LIFT_HEIGHT_MM;  // 最终安全高度
    
    printf("\n→ 第三段抬升: 到达安全高度 Z=%.1f\r\n", lift3_z);
    
    ret = Motion_MoveToXYZ_RuckigSmooth(
        lift3_x, lift3_y, lift3_z,
        PICK_PITCH_DEG, MOTION_LIFT_DURATION_S
    );
    
    if (ret == 0) {
        printf("✓ 箱子已抬升到安全高度\r\n");
        osDelay(1000);
        StateMachine_Transition(STATE_LIFT_BOX1);
    } else {
        printf("[错误] 第三段抬升失败\r\n");
        Pump_Off();
        g_system.status.pump_on = false;
        StateMachine_Reset(false);
    }
}

/**
 * @brief 抬升箱子状态处理
 * @note 回到初始位置（安全姿态）
 */
void State_LiftBox(void) {
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║  阶段3: 回到初始位置                  ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    printf("→ 回零中...\r\n");
    Motion_SetHome();
    osDelay(MOTOR_READY_DELAY_MS + 500); // 回零需要更长时间
    
    printf("✓ 已回到初始位置\r\n");
    StateMachine_Transition(STATE_MOVE_TO_PLACE);
}

/**
 * @brief 移动到放置点状态处理
 * @note 从初始位置移动到放置区上方
 */
void State_MoveToPlace(void) {
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║  阶段4: 移动到放置点                  ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    // 计算放置预备点坐标
    float place_approach_x = g_system.mission.place.x;
    float place_approach_y = g_system.mission.place.y;
    float place_approach_z = g_system.mission.place.z + PICK_APPROACH_OFFSET_Z_MM;
    
    printf("放置预备点: X=%.1f, Y=%.1f, Z=%.1f\r\n", 
           place_approach_x, place_approach_y, place_approach_z);
    printf("\n→ 移动到放置预备点\r\n");
    
    int ret = Motion_MoveToXYZ_RuckigSmooth(
        place_approach_x, place_approach_y, place_approach_z,
        PICK_PITCH_DEG, 2.0f
    );
    
    if (ret == 0) {
        printf("✓ 到达放置预备点\r\n");
        StateMachine_Transition(STATE_RELEASE_ALL);
    } else {
        printf("[错误] 移动到放置预备点失败\r\n");
        Pump_Off();
        g_system.status.pump_on = false;
        StateMachine_Reset(false);
    }
}

/**
 * @brief 释放箱子状态处理
 * @note 下压到放置点，关闭气泵，抬升，回零
 */
void State_ReleaseBox(void) {
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║  阶段5: 释放箱子                      ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    // 5.1 下压到放置点
    float place_x = g_system.mission.place.x;
    float place_y = g_system.mission.place.y;
    float place_z = g_system.mission.place.z;
    
    printf("放置点: X=%.1f, Y=%.1f, Z=%.1f\r\n", place_x, place_y, place_z);
    printf("\n→ 下降到放置点\r\n");
    
    int ret = Motion_MoveToXYZ_RuckigSmooth(
        place_x, place_y, place_z,
        PICK_PITCH_DEG, 2.0f
    );
    
    if (ret != 0) {
        printf("[错误] 下降失败\r\n");
        Pump_Off();
        g_system.status.pump_on = false;
        StateMachine_Reset(false);
        return;
    }
    
    printf("✓ 到达放置点\r\n");
    
    // 5.2 关闭气泵
    osDelay(PUMP_OFF_DELAY_MS);
    printf("\n→ 关闭气泵\r\n");
    Pump_Off();
    g_system.status.pump_on = false;
    osDelay(RELEASE_LIFT_DELAY_MS);
    printf("✓ 气泵已关闭\r\n");
    
    // 5.3 抬升
    float lift_x = place_x;
    float lift_y = place_y;
    float lift_z = place_z + PICK_APPROACH_OFFSET_Z_MM;
    
    printf("\n→ 抬升到 Z=%.1f\r\n", lift_z);
    
    ret = Motion_MoveToXYZ_RuckigSmooth(
        lift_x, lift_y, lift_z,
        PICK_PITCH_DEG, 2.0f
    );
    
    if (ret != 0) {
        printf("[警告] 抬升失败\r\n");
    } else {
        printf("✓ 已抬升\r\n");
    }
    
    // 5.4 回零
    printf("\n→ 回零中...\r\n");
    Motion_SetHome();
    osDelay(MOTOR_READY_DELAY_MS);
    printf("✓ 已回到初始位置\r\n");
    
    // 5.5 任务完成
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║  ✓ 任务完成！                         ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    printf("起点: X=%.1f, Y=%.1f, Z=%.1f\r\n", 235.0f, 0.0f, 240.0f);
    printf("箱子: X=%.1f, Y=%.1f, Z=%.1f\r\n", 
           g_system.mission.box.x, g_system.mission.box.y, g_system.mission.box.z);
    printf("终点: X=%.1f, Y=%.1f, Z=%.1f\r\n", 
           g_system.mission.place.x, g_system.mission.place.y, g_system.mission.place.z);
    printf("\r\n");
    
    StateMachine_Reset(true);
}

/**
 * @brief 错误状态处理
 * @note 系统进入安全状态，等待手动复位
 */
void State_Error(void) {
    static uint32_t last_print = 0;
    
    // 确保气泵关闭
    if (g_system.status.pump_on) {
        Pump_Off();
        g_system.status.pump_on = false;
    }
    
    // 每秒打印一次错误信息
    uint32_t now = osKernelGetTickCount();
    if (now - last_print > 1000) {
        last_print = now;
        LOG_E(TAG, "System in ERROR state! Manual reset required.");
        LOG_E(TAG, "Last state: %s, Retry count: %d", 
              StateMachine_GetStateName(g_system.sm.prev_state),
              g_system.sm.retry_count);
    }
    
    osDelay(100);
    
    // 可以通过外部命令或按钮复位
    // 这里简单实现：10秒后自动复位
    if (g_system.sm.state_duration > 10000) {
        LOG_W(TAG, "Auto-reset after 10s in ERROR state");
        StateMachine_Reset(true);
    }
}
