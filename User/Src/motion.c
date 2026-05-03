#include "motion.h"
#include "calculate.h"
#include "DrEmpower_can.h"
#include "servo.h"
#include "ruckig_smooth.h"
#include <math.h>
#include <stdio.h>
#include "cmsis_os2.h" // 包含系统延时
#include "logger.h"

/* ========== 全局PID控制器 ========== */
PID_Controller_t roll_pid = {1.5f, 0.05f, 0.3f, 0, 0};    // Roll稳定增益
PID_Controller_t pitch_pid = {1.5f, 0.05f, 0.3f, 0, 0};   // Pitch稳定增益

// IMU local switch for motion.c:
// 0 = disable all IMU-based compensation in this file
// 1 = enable IMU-based compensation
#define MOTION_IMU_ENABLE 0

extern float target_x, target_y, target_z;
volatile uint8_t motion_stream_active = 0;

// 坐标记忆
float current_arm_x = 235.0f;
float current_arm_y = 0.0f;
float current_arm_z = 240.0f; //350-110
static float current_joint_0 = 0.0f;
static float current_joint_1 = 90.0f;
static float current_joint_2 = 0.0f;

/**
 * @brief 初始化稳定控制系统
 */
void Motion_StabilityInit(void)
{
    roll_pid.integral_error = 0; // 清空历史积分，防止启动时猛一甩
    roll_pid.prev_error = 0; // 清空上次误差
    
    pitch_pid.integral_error = 0;
    pitch_pid.prev_error = 0;
    
}


/**
 * @brief 计算姿态稳定补偿（核心稳定算法）
 * @return 补偿结构体，包含所有轴的补偿值
 * 
 * 稳定原理（陀螺仪在三臂上）：
 * - Pitch：直接稳定三臂的俯仰角，应用到关节3（小臂 theta2）
 * - Roll：稳定底座，补偿机械狗的水平倾斜
 * 
 * 这种方法比传统的通过底座+大臂间接补偿更加直观和有效。
 */
StabilityCompensation_t Motion_CalcStabilityCompensation(void)
{
    StabilityCompensation_t result = {0, 0, 0, 0};  // 初始化为0

    if (!MOTION_IMU_ENABLE || !STABILITY_ENABLED || !imu.valid) {
        return result;  // 未启用或数据无效，返回零补偿
    }

#if (IMU_MOUNT_POSITION == IMU_ON_WRIST)
    // ========== 陀螺仪在三臂上（手腕）- 推荐方案 ==========
    
    /* --- Pitch补偿：直接稳定三臂俯仰 --- */
    // 三臂俯仰角直接测量，应用补偿到theta2
    float pitch_error = -imu.pitch;  // 负号表示反向补偿
    
    if (fabsf(pitch_error) < COMP_DEADZONE) {
        pitch_error = 0;
    }
    
    // 使用pitch_pid计算PID输出
    pitch_pid.integral_error += pitch_error;
    float pitch_comp = pitch_pid.kp * pitch_error + 
                       pitch_pid.ki * pitch_pid.integral_error + 
                       pitch_pid.kd * (pitch_error - pitch_pid.prev_error);
    pitch_pid.prev_error = pitch_error;
    
    // 限制幅度（三臂补偿范围较小，±3度比较合理）
    result.wrist_pitch_comp = (pitch_comp > 3.0f) ? 3.0f : (pitch_comp < -3.0f) ? -3.0f : pitch_comp;

    /* --- Roll补偿：稳定底座 --- */
    float roll_error = -imu.roll;
    
    if (fabsf(roll_error) < COMP_DEADZONE) {
        roll_error = 0;
    }
    
    roll_pid.integral_error += roll_error;
    float roll_comp = roll_pid.kp * roll_error + 
                      roll_pid.ki * roll_pid.integral_error + 
                      roll_pid.kd * (roll_error - roll_pid.prev_error);
    roll_pid.prev_error = roll_error;
    
    result.roll_comp = (roll_comp > 5.0f) ? 5.0f : (roll_comp < -5.0f) ? -5.0f : roll_comp;

    // 调试输出
    static uint32_t debug_count = 0;
    if (++debug_count >= 10) {
        debug_count = 0;
        // printf("[Stab-Wrist] Roll=%.2f°(→%.2f°), Pitch(Wrist)=%.2f°(→%.2f°)\r\n",
        //        imu.roll, result.roll_comp,
        //        imu.pitch, result.wrist_pitch_comp);
    }

#else
    // ========== 其他位置（保留，暂不使用） ==========
    // 这里保留了原来的底座+大臂补偿逻辑，以备后用
    
    float roll_error = -imu.roll;
    if (fabsf(roll_error) < COMP_DEADZONE) {
        roll_error = 0;
    }
    
    roll_pid.integral_error += roll_error;
    float roll_comp = roll_pid.kp * roll_error + 
                      roll_pid.ki * roll_pid.integral_error + 
                      roll_pid.kd * (roll_error - roll_pid.prev_error);
    roll_pid.prev_error = roll_error;
    result.roll_comp = (roll_comp > 5.0f) ? 5.0f : (roll_comp < -5.0f) ? -5.0f : roll_comp;

    float pitch_error = -imu.pitch;
    if (fabsf(pitch_error) < COMP_DEADZONE) {
        pitch_error = 0;
    }
    
    pitch_pid.integral_error += pitch_error;
    float pitch_comp = pitch_pid.kp * pitch_error + 
                       pitch_pid.ki * pitch_pid.integral_error + 
                       pitch_pid.kd * (pitch_error - pitch_pid.prev_error);
    pitch_pid.prev_error = pitch_error;
    result.pitch_comp = (pitch_comp > 5.0f) ? 5.0f : (pitch_comp < -5.0f) ? -5.0f : pitch_comp;

#endif

    return result;
}


void Motion_SetHome(void) {
    // 初始状态：底座0，大臂90(垂直)，小臂0(水平)
    // 速度 10, 加速度 5
    set_angle(ID_BASE,  0.0f,  10.0f, 5.0f, 1);
    set_angle(ID_BIG,   90.0f, 10.0f, 5.0f, 1);
    set_angle(ID_SMALL, -0.0f,  10.0f, 5.0f, 1);  // 注意：电机3角度取反
    current_joint_0 = 0.0f;
    current_joint_1 = 90.0f;
    current_joint_2 = 0.0f;

    // 末端用机械结构固定垂直向下，无需舵机控制
}

int Motion_MoveToXYZ(float x, float y, float z, float pitch) {
    JointAngles_t ik_result;

#if MOTION_USE_RUCKIG_SMOOTH
    // Ruckig 模式：统一走关节平滑轨迹（内部已包含 IK、限位检查和连续下发）
    return Motion_MoveToXYZ_RuckigSmooth(x, y, z, pitch, 0.0f);
#else
    // 末端通过机械连杆保持垂直向下，pitch 通常固定为 -90.0f。
    // 这一步仍然使用逆运动学计算完整关节角度。
    if (inverse_kinematics(x, y, z, pitch, &ik_result) == 0) {
        
        // 应用关节角度限位保护（防止机械臂过度伸展或负角度）
        ik_result = optimize_and_limit_angles(ik_result, 0.0f);  // current_theta0暂时传0
        
#if STABILITY_ENABLED
        // ========== 启用稳定补偿 ==========
        StabilityCompensation_t stability = Motion_CalcStabilityCompensation();
        
#if (IMU_MOUNT_POSITION == IMU_ON_WRIST)
        // ===== 陀螺仪在三臂（手腕）上 =====
        // 直接应用Pitch补偿到theta2（三臂/小臂角度）
        float theta2_comp = ik_result.theta2 + stability.wrist_pitch_comp;
        
        // 底座补偿
        float theta0_comp = ik_result.theta0 + stability.roll_comp * 0.8f;
        
        // 控制大然电机（带补偿）
        set_angle(ID_BASE,  theta0_comp, 30.0f, 20.0f, 1);
        set_angle(ID_BIG,   ik_result.theta1, 30.0f, 20.0f, 1);  // 大臂不补偿（陀螺仪不测量）
        // 机械结构要求小臂稍后启动时，应该在状态机层面处理，而不是在通用运动函数里直接阻塞。
        set_angle(ID_SMALL, -theta2_comp, 30.0f, 20.0f, 1);       // 小臂/三臂补偿（注意：电机3角度取反）

#else
        // ===== 其他陀螺仪位置（保留） =====
        float theta0_comp = ik_result.theta0 + stability.roll_comp * 0.8f;
        float theta1_comp = ik_result.theta1 + stability.pitch_comp * 0.6f;
        
        set_angle(ID_BASE,  theta0_comp, 50.0f, 40.0f, 1);
        set_angle(ID_BIG,   theta1_comp, 50.0f, 40.0f, 1);
        set_angle(ID_SMALL, -ik_result.theta2, 50.0f, 40.0f, 1);  // 注意：电机3角度取反
#endif

#else
        // ========== 禁用稳定补偿 ==========
        // 控制大然电机（无补偿）
        set_angle(ID_BASE,  ik_result.theta0, 50.0f, 40.0f, 1);
        set_angle(ID_BIG,   ik_result.theta1, 50.0f, 40.0f, 1);
        set_angle(ID_SMALL, -ik_result.theta2, 50.0f, 40.0f, 1);  // 注意：电机3角度取反
#endif
        return 0;
    }
    //return -1; // 解算失败
    return 0;
#endif
}

int Motion_MoveToXYZ_BaseBig(float x, float y, float z, float pitch)
{
    JointAngles_t ik_result;
    if (inverse_kinematics(x, y, z, pitch, &ik_result) != 0) {
        return -1;
    }

    StabilityCompensation_t stability = Motion_CalcStabilityCompensation();
    float theta0_comp = ik_result.theta0 + stability.roll_comp * 0.8f;

    set_angle(ID_BASE, theta0_comp, 30.0f, 20.0f, 1);
    set_angle(ID_BIG,  ik_result.theta1, 30.0f, 20.0f, 1);

    return 0;
}

int Motion_MoveToXYZ_SmallArm(float x, float y, float z, float pitch)
{
    JointAngles_t ik_result;
    if (inverse_kinematics(x, y, z, pitch, &ik_result) != 0) {
        return -1;
    }

    StabilityCompensation_t stability = Motion_CalcStabilityCompensation();
    float theta2_comp = ik_result.theta2 + stability.wrist_pitch_comp;

    set_angle(ID_SMALL, -theta2_comp, 30.0f, 20.0f, 1);  // 注意：电机3角度取反

    // 【新增】：下发控制指令成功后，更新当前记忆坐标
    current_arm_x = x;
    current_arm_y = y;
    current_arm_z = z;

    return 0;
}

//--------------------------------------------------------------

/**
 * @brief 笛卡尔空间 + 五次多项式平滑移动 (直线轨迹，S型速度)
 * @param end_x, end_y, end_z 目标 XYZ 坐标 (mm)
 * @param duration_s          期望到达的时间 (秒)
 */
int Motion_MoveStraightSmooth(float end_x, float end_y, float end_z, float duration_s)
{
    // 1. 获取当前机械臂物理起点
    float start_x = current_arm_x;
    float start_y = current_arm_y;
    float start_z = current_arm_z;

    // 2. 设定控制周期 (20ms = 50Hz刷新率，足够丝滑)
    const uint32_t step_ms = 20;
    int total_steps = (int)(duration_s * 1000.0f / step_ms);
    if(total_steps < 1) total_steps = 1;

    // 3. 开始插补循环
    for(int i = 1; i <= total_steps; i++) {
        float tau = (float)i / (float)total_steps;

        // 五次多项式速度曲线 s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        float tau3 = tau * tau * tau;
        float tau4 = tau3 * tau;
        float tau5 = tau4 * tau;
        float s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;

        // 计算当前瞬间的目标坐标点
        float cur_x = start_x + (end_x - start_x) * s;
        float cur_y = start_y + (end_y - start_y) * s;
        float cur_z = start_z + (end_z - start_z) * s;

        JointAngles_t ik_result;
        if (inverse_kinematics(cur_x, cur_y, cur_z, -90.0f, &ik_result) == 0) {

            // 极限解除大然电机内部缓冲，把速度设为极大值 300，让电机服从我们的细分位置下发
            #if STABILITY_ENABLED
                StabilityCompensation_t stability = Motion_CalcStabilityCompensation();
                #if (IMU_MOUNT_POSITION == IMU_ON_WRIST)
                    float theta2_comp = ik_result.theta2 + stability.wrist_pitch_comp;
                    float theta0_comp = ik_result.theta0 + stability.roll_comp * 0.8f;
                    set_angle(ID_BASE,  theta0_comp,   30.0f, 30.0f, 1);
                    set_angle(ID_BIG,   ik_result.theta1, 30.0f, 30.0f, 1);
                    set_angle(ID_SMALL, -theta2_comp,   30.0f, 30.0f, 1);  // 注意：电机3角度取反
                #endif
            #else
                set_angle(ID_BASE,  ik_result.theta0, 300.0f, 300.0f, 1);
                set_angle(ID_BIG,   ik_result.theta1, 300.0f, 300.0f, 1);
                set_angle(ID_SMALL, -ik_result.theta2, 300.0f, 300.0f, 1);  // 注意：电机3角度取反
            #endif

            // 更新记忆坐标
            current_arm_x = cur_x;
            current_arm_y = cur_y;
            current_arm_z = cur_z;

        } else {
            return -1; // 运动学无解或超限，立刻中断
        }

        // 关键：把控制权交还给 FreeRTOS 系统，延时 20ms 等待电机转到位
        osDelay(step_ms);
    }

    return 0;
}

// ---------------- Ruckig smooth joint motion ----------------

static float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int Motion_MoveToXYZ_RuckigSmooth(float x, float y, float z, float pitch, float duration_hint_s) {
    (void)duration_hint_s; // 目前不强制 duration（Ruckig 会给时间最优 jerk-limited 轨迹）
    const float z_cmd = z + MOTION_Z_CALIB_BIAS_MM;

    JointAngles_t ik_result;
    if (inverse_kinematics(x, y, z_cmd, pitch, &ik_result) != 0) {
        printf("[Ruckig] IK failed\r\n");
        return -11; // IK 无解
    }

    // 【关键】应用关节角度限位保护
    ik_result = optimize_and_limit_angles(ik_result, current_joint_0);

    // 目标角（含稳定补偿，只对目标做一次补偿；如果需要"实时稳定"，应在上层闭环里做）
    float qT[3] = {ik_result.theta0, ik_result.theta1, ik_result.theta2};


#if STABILITY_ENABLED
    StabilityCompensation_t stability = Motion_CalcStabilityCompensation();
#if (IMU_MOUNT_POSITION == IMU_ON_WRIST)
    qT[0] = qT[0] + stability.roll_comp * 0.8f;
    qT[2] = qT[2] + stability.wrist_pitch_comp;
#else
    qT[0] = qT[0] + stability.roll_comp * 0.8f;
    qT[1] = qT[1] + stability.pitch_comp * 0.6f;
#endif
#endif

    // 当前关节角（使用软件记忆值，避免总线回读阻塞）
    float q0[3] = {current_joint_0, current_joint_1, current_joint_2};

    // 用户要求移除限位约束：这里不做关节角夹紧与越限拦截。

    // 50Hz 下发（和 Motion_MoveStraightSmooth 一致）
    const uint32_t step_ms = 20;
    const float dt_s = 0.02f;

    RuckigSmooth_Init(dt_s);
    const int start_ret = RuckigSmooth_Start(q0, qT);
    if (start_ret != 0) {
        printf("[Ruckig] start failed ret=%d q0=(%.2f, %.2f, %.2f) qT=(%.2f, %.2f, %.2f)\r\n",
               start_ret, q0[0], q0[1], q0[2], qT[0], qT[1], qT[2]);
        return (-200 + start_ret); // -21x / -22x 保留底层错误码
    }

    float q_cmd[3] = {q0[0], q0[1], q0[2]};

    // mode=0：轨迹跟踪模式。param=滤波带宽（建议=指令频率一半），50Hz -> 25
    const float tracking_bandwidth = 25.0f;
    uint32_t loop_count = 0;
    const uint32_t max_loop_count = 1000; // 1000 * 20ms = 20s timeout（增加以适应慢速运动）
    
    // 【调试模式】实时打印运动过程
    uint32_t last_print_time = osKernelGetTickCount();
    const uint32_t print_interval_ms = 1000;  // 每1秒打印一次
    
    printf("\r\n========== 开始运动 ==========\r\n");
    printf("目标: X=%.1f, Y=%.1f, Z=%.1f\r\n", x, y, z);
    printf("目标角度: Th0=%.1f°, Th1=%.1f°, Th2=%.1f°\r\n", qT[0], qT[1], qT[2]);
    printf("起始角度: Th0=%.1f°, Th1=%.1f°, Th2=%.1f°\r\n", q0[0], q0[1], q0[2]);
    printf("=============================\r\n");

    motion_stream_active = 1;
    while (1) {
        const int st = RuckigSmooth_Step(q_cmd);

        set_angle(ID_BASE, q_cmd[0], 0.0f, tracking_bandwidth, 0);
        osDelay(1);
        set_angle(ID_BIG, q_cmd[1], 0.0f, tracking_bandwidth, 0);
        osDelay(1);
        set_angle(ID_SMALL, -q_cmd[2], 0.0f, tracking_bandwidth, 0);  // 注意：电机3角度取反
        
        // 【调试模式】每1秒打印当前状态
        uint32_t current_time = osKernelGetTickCount();
        if (current_time - last_print_time >= print_interval_ms) {
            last_print_time = current_time;
            
            // 计算当前末端位置（FK正解）
            float current_x, current_y, current_z;
            forward_kinematics_default(q_cmd[0], q_cmd[1], q_cmd[2], 0, 
                                      &current_x, &current_y, &current_z);
            
            // 计算大小臂夹角
            float angle_diff = fabsf(q_cmd[2] - q_cmd[1]);
            float arm_angle_inner = 180.0f - angle_diff;
            
            printf("[%.1fs] 电机角: Th0=%.1f° Th1=%.1f° Th2=%.1f° | 大臂角=%.1f° 小臂角=%.1f° 夹角=%.1f° | 末端: X=%.1f Y=%.1f Z=%.1f\r\n",
                   (float)loop_count * dt_s,
                   q_cmd[0], q_cmd[1], q_cmd[2],
                   q_cmd[1], q_cmd[2], arm_angle_inner,
                   current_x, current_y, current_z);
        }

        if (st == 1) {
            // 计算最终末端位置
            float final_x, final_y, final_z;
            forward_kinematics_default(q_cmd[0], q_cmd[1], q_cmd[2], 0, 
                                      &final_x, &final_y, &final_z);
            float final_angle_diff = fabsf(q_cmd[2] - q_cmd[1]);
            float final_arm_angle = 180.0f - final_angle_diff;
            
            printf("\r\n========== 运动完成 ==========\r\n");
            printf("最终角度: Th0=%.1f°, Th1=%.1f°, Th2=%.1f°\r\n", q_cmd[0], q_cmd[1], q_cmd[2]);
            printf("大小臂夹角: %.1f°\r\n", final_arm_angle);
            printf("末端位置: X=%.1f, Y=%.1f, Z=%.1f\r\n", final_x, final_y, final_z);
            printf("总用时: %.2f秒\r\n", (float)loop_count * dt_s);
            printf("=============================\r\n");
            break;
        }
        if (st < 0) {
            printf("[Ruckig] step failed st=%d loop=%lu\r\n", st, (unsigned long)loop_count);
            motion_stream_active = 0;
            return (-300 + st); // -32x 保留底层错误码
        }
        if (++loop_count >= max_loop_count) {
            printf("[Ruckig] timeout loop=%lu\r\n", (unsigned long)loop_count);
            motion_stream_active = 0;
            return -401; // 超时保护，避免卡死
        }

        osDelay(step_ms);
    }

    // 更新记忆坐标（到达目标后更新）
    current_arm_x = x;
    current_arm_y = y;
    current_arm_z = z;
    current_joint_0 = qT[0];
    current_joint_1 = qT[1];
    current_joint_2 = qT[2];
    motion_stream_active = 0;

    // motion.c 中，Ruckig运动完成后
    float verify_x, verify_y, verify_z;
    forward_kinematics_default(qT[0], qT[1], qT[2], 0, &verify_x, &verify_y, &verify_z);
    LOG_I("Motion", "Target: (%.1f, %.1f, %.1f), FK verify: (%.1f, %.1f, %.1f)",
          x, y, z, verify_x, verify_y, verify_z);


    return 0;
}

void Calculate_CameraToArm(float cam_x, float cam_y, float cam_z,
                           float* arm_x, float* arm_y, float* arm_z) {
    if (!arm_x || !arm_y || !arm_z) return;

    // 相机坐标系定义（无倾角时）：
    // 相机X轴：水平向右 → 机械臂-X
    // 相机Y轴：垂直向下 → 机械臂-Z
    // 相机Z轴：水平向前 → 机械臂-Y
    //
    // 相机安装时绕自身X轴向下倾斜21.12度
    // 旋转矩阵（绕X轴顺时针旋转θ=21.12°）：
    // | cam_x' |   | 1      0         0     |   | cam_x |
    // | cam_y' | = | 0   cos(θ)   sin(θ)   | × | cam_y |
    // | cam_z' |   | 0  -sin(θ)   cos(θ)   |   | cam_z |
    //
    // 旋转后的相机坐标：
    float cam_x_rot = cam_x;  // X轴不变
    float cam_y_rot = cam_y * K_COS_TILT + cam_z * K_SIN_TILT;
    float cam_z_rot = -cam_y * K_SIN_TILT + cam_z * K_COS_TILT;

    // 映射到机械臂坐标系并转换单位（cm -> mm）：
    // 机械臂X = -相机X'
    // 机械臂Y = -相机Z'
    // 机械臂Z = -相机Y'
    *arm_x = -cam_x_rot * K_COORD_SCALE_CM_TO_MM + K_ARM_X_OFFSET_MM;
    *arm_y = -cam_z_rot * K_COORD_SCALE_CM_TO_MM - K_ARM_Y_OFFSET_MM + MOTION_Y_CALIB_BIAS_MM;
    *arm_z = -cam_y_rot * K_COORD_SCALE_CM_TO_MM - K_ARM_Z_OFFSET_MM - 180.0f;
}


/**
 * @brief 测试模式：单独控制指定电机到指定角度
 * @param motor_id 电机ID (1=底座, 2=大臂, 3=小臂)
 * @param angle 目标角度 (度)
 * @note 用于验证角度定义和FK正解
 */
void Motion_TestSingleMotor(uint8_t motor_id, float angle) {
    printf("\r\n========== 测试模式：单电机控制 ==========\r\n");
    printf("电机ID: %d, 目标角度: %.1f°\r\n", motor_id, angle);
    
    // 发送角度到指定电机
    switch(motor_id) {
        case ID_BASE:
            set_angle(ID_BASE, angle, 20, 20, 1);
            current_joint_0 = angle;
            printf("底座电机(ID=%d) -> %.1f°\r\n", ID_BASE, angle);
            break;
            
        case ID_BIG:
            set_angle(ID_BIG, angle, 20, 20, 1);
            current_joint_1 = angle;
            printf("大臂电机(ID=%d) -> %.1f°\r\n", ID_BIG, angle);
            break;
            
        case ID_SMALL:
            set_angle(ID_SMALL, -angle, 20, 20, 1);  // 注意：电机3角度取反
            current_joint_2 = angle;  // 保存的是数学角度（未取反）
            printf("小臂电机(ID=%d) -> 数学角度%.1f°, 电机角度%.1f°\r\n", 
                   ID_SMALL, angle, -angle);
            break;
            
        default:
            printf("[错误] 无效的电机ID: %d\r\n", motor_id);
            return;
    }
    
    // 等待电机到位
    osDelay(1000);
    
    // 使用FK正解计算当前末端位置
    float x, y, z;
    forward_kinematics_default(current_joint_0, current_joint_1, current_joint_2, 0, 
                              &x, &y, &z);
    
    printf("\r\n[FK正解验证]\r\n");
    printf("当前关节角度: Th0=%.1f°, Th1=%.1f°, Th2=%.1f°\r\n", 
           current_joint_0, current_joint_1, current_joint_2);
    printf("计算末端位置: X=%.1fmm, Y=%.1fmm, Z=%.1fmm\r\n", x, y, z);
    
    float angle_diff = fabsf(current_joint_2 - current_joint_1);
    float arm_angle_inner = 180.0f - angle_diff;
    printf("角度差: %.1f°, 大小臂内角: %.1f°\r\n", angle_diff, arm_angle_inner);
    printf("========================================\r\n");
}
