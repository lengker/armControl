#include "calculate.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

/* ==============================================================================
 *                                 参数定义与宏
 * ============================================================================== */

/* 是否开启解算过程中的调试打印 (1:开启, 0:关闭) */
#define DEBUG_KINEMATICS 1
/* 机械臂几何参数 (单位: mm) */
// 原点在大臂转轴中心处
#define ARM_L1      350.00f  // 大臂长度 (mm)
#define ARM_L2      235.50f  // 小臂长度 (mm)
#define ARM_L3      110.00f  // 三臂长度 (mm)

/* 数学常数 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD  (M_PI / 180.0f)
#define RAD_TO_DEG  (180.0f / M_PI)


/* ==============================================================================
 *                                 内部辅助函数
 * ============================================================================== */

/**
 * @brief 限制数值范围
 */
static float clamp_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief 计算最短路径
 * 原理：将差值限制在 -180 到 180 之间
 */
static float find_shortest_path(float target, float current) {
    float diff = target - current;

    while (diff <= -180.0f) diff += 360.0f;
    while (diff > 180.0f)   diff -= 360.0f;

    return current + diff;
}


/* ==============================================================================
 *                                 解算核心代码
 * ============================================================================== */

/**
 * @brief 正运动学解算：已知关节角度，计算末端坐标
 * @note  【关键假设】theta1和theta2都是相对于水平线的绝对角度
 *        theta1: 大臂与水平方向的绝对角度（0度=水平，90度=垂直向上）
 *        theta2: 小臂与水平方向的绝对角度（0度=水平，90度=垂直向上）
 *        theta3: 三臂与水平方向的绝对角度
 */
void forward_kinematics(float theta0, float theta1, float theta2, float theta3,
                        float L1, float L2, float L3,
                        float *x, float *y, float *z)
{
    float th0_rad = theta0 * DEG_TO_RAD;
    
    // 【关键】直接使用绝对角度
    float A1 = theta1;  // 大臂绝对角度
    float A2 = theta2;  // 小臂绝对角度
    float A3 = theta3;  // 三臂绝对角度
    
    float math_th1_rad = A1 * DEG_TO_RAD;
    float math_th2_rad = A2 * DEG_TO_RAD;
    float math_th3_rad = A3 * DEG_TO_RAD;

    // 1. 计算在垂直平面内的投影
    float r1 = L1 * cosf(math_th1_rad);
    float z1 = L1 * sinf(math_th1_rad);
    float r2 = L2 * cosf(math_th2_rad);
    float z2 = L2 * sinf(math_th2_rad);
    float r3 = L3 * cosf(math_th3_rad);
    float z3 = L3 * sinf(math_th3_rad);

    // 2. 叠加所有分量
    float r_total = r1 + r2 + r3;
    float z_total = z1 + z2 + z3;

    // 3. 映射到 3D 空间
    *x = r_total * cosf(th0_rad);
    *y = r_total * sinf(th0_rad);
    *z = z_total;

#if DEBUG_KINEMATICS == 1
    // 简化的FK打印（仅在需要时调用）
    // 大小臂夹角（内角）= 180° - |theta2 - theta1|
    float angle_diff = fabsf(theta2 - theta1);
    float arm_angle_inner = 180.0f - angle_diff;
    // 不在这里打印，由调用者决定是否打印
#endif
}

/**
 * @brief 带默认参数的正运动学解算
 */
void forward_kinematics_default(float theta0, float theta1, float theta2, float theta3,
                                float *x, float *y, float *z)
{
    forward_kinematics(theta0, theta1, theta2, theta3,
                       ARM_L1, ARM_L2, ARM_L3, x, y, z);
}

/**
 * @brief 逆运动学解算
 * @param x, y, z 目标坐标 (Z为高度)
 * @param target_absolute_pitch 吸盘相对地面的绝对角度。例如垂直向下抓取传 -90.0f
 * @return 0: 成功, -1: 目标不可达
 */
int inverse_kinematics(float x, float y, float z, float target_absolute_pitch, JointAngles_t *angles)
{
    angles->theta0 = atan2f(y, x) * RAD_TO_DEG; // 底座角度
    
    // 计算末端在机械臂平面内的坐标 (r, z)
    float r_target = sqrtf(x*x + y*y);
    float z_target = z;

    // 腕部解耦 (Wrist Decoupling)
    float pitch_rad = target_absolute_pitch * DEG_TO_RAD;
    float r_wrist = r_target - ARM_L3 * cosf(pitch_rad);
    float z_wrist = z_target - ARM_L3 * sinf(pitch_rad);

    // 两连杆解算 (大臂 L1，小臂 L2)
    float D_sq = r_wrist*r_wrist + z_wrist*z_wrist;
    float D = sqrtf(D_sq);
    
    // 检查工作空间
    if (D > (ARM_L1 + ARM_L2) || D < fabsf(ARM_L1 - ARM_L2)) {
        return -1; // 不可达
    }

    // 利用余弦定理计算几何角度
    float cos_alpha = (ARM_L1*ARM_L1 + D_sq - ARM_L2*ARM_L2) / (2.0f * ARM_L1 * D);
    float alpha = acosf(clamp_float(cos_alpha, -1.0f, 1.0f));
    float beta = atan2f(z_wrist, r_wrist);

    // 计算大臂绝对角度
    float A1_rad = beta + alpha;  // "肘部朝上"的解
    float A1 = A1_rad * RAD_TO_DEG;

    // 计算小臂绝对角度
    float r_L1_end = ARM_L1 * cosf(A1_rad);
    float z_L1_end = ARM_L1 * sinf(A1_rad);
    float A2 = atan2f(z_wrist - z_L1_end, r_wrist - r_L1_end) * RAD_TO_DEG;

    // 【关键】直接赋值绝对角度
    angles->theta1 = A1;
    angles->theta2 = A2;

    return 0;
}

/**
 * @brief 对目标角度进行安全限制和路径优化
 */
JointAngles_t optimize_and_limit_angles(JointAngles_t target_input, float current_theta0)
{
    JointAngles_t safe_angles;
    
    printf("[限位前] Th0=%.1f, Th1=%.1f, Th2=%.1f\r\n",
           target_input.theta0, target_input.theta1, target_input.theta2);
    
    // 1. 关节0：最短路径优化
    safe_angles.theta0 = find_shortest_path(target_input.theta0, current_theta0);
    
    // 2. 关节0~2：安全范围限制
    safe_angles.theta0 = clamp_float(safe_angles.theta0, J0_MIN, J0_MAX);
    safe_angles.theta1 = clamp_float(target_input.theta1, J1_MIN, J1_MAX);
    safe_angles.theta2 = clamp_float(target_input.theta2, J2_MIN, J2_MAX);
    
    // 3. 大臂和小臂夹角限制
    // 内角 = 180° - |theta2 - theta1|
    float angle_diff = fabsf(safe_angles.theta2 - safe_angles.theta1);
    float arm_angle_inner = 180.0f - angle_diff;
    
    printf("[夹角检查] 角度差=%.1f°, 内角=%.1f° (范围: %.1f°~%.1f°)\r\n", 
           angle_diff, arm_angle_inner, ARM_ANGLE_MIN, ARM_ANGLE_MAX);
    
    if (arm_angle_inner < ARM_ANGLE_MIN) {
        // 内角太小（过度折叠），需要减小角度差
        float target_diff = 180.0f - ARM_ANGLE_MIN;
        if (safe_angles.theta2 > safe_angles.theta1) {
            // theta2更大，减小theta2
            safe_angles.theta2 = safe_angles.theta1 + target_diff;
        } else {
            // theta1更大，增大theta2
            safe_angles.theta2 = safe_angles.theta1 - target_diff;
        }
        printf("[夹角限位] 内角过小，调整theta2: %.1f -> %.1f\r\n", 
               target_input.theta2, safe_angles.theta2);
    }
    else if (arm_angle_inner > ARM_ANGLE_MAX) {
        // 内角太大（过度伸展），需要增大角度差
        float target_diff = 180.0f - ARM_ANGLE_MAX;
        if (safe_angles.theta2 > safe_angles.theta1) {
            // theta2更大，增大theta2
            safe_angles.theta2 = safe_angles.theta1 + target_diff;
        } else {
            // theta1更大，减小theta2
            safe_angles.theta2 = safe_angles.theta1 - target_diff;
        }
        printf("[夹角限位] 内角过大，调整theta2: %.1f -> %.1f\r\n", 
               target_input.theta2, safe_angles.theta2);
    }
    
    // 再次确保 theta2 在其机械范围内
    safe_angles.theta2 = clamp_float(safe_angles.theta2, J2_MIN, J2_MAX);
    
    printf("[限位后] Th0=%.1f, Th1=%.1f, Th2=%.1f\r\n",
           safe_angles.theta0, safe_angles.theta1, safe_angles.theta2);
    
    return safe_angles;
}
