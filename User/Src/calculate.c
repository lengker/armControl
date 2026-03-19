#include "calculate.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

/* ==============================================================================
 *                                 参数定义与宏
 * ============================================================================== */

/* 是否开启解算过程中的调试打印 (1:开启, 0:关闭) */
#define DEBUG_KINEMATICS 1
/* 机械臂几何参数 (单位: cm) */
// 原点在大臂转轴中心处
#define ARM_L1      35.00f   // 大臂长度
#define ARM_L2      23.55f   // 小臂长度
//#define ARM_L3      8.836f   // 三臂长度，有点问题
#define ARM_L3      10.00f   // 三臂长度


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
 * @note  坐标系定义：
 *        Z轴：垂直向上 (高度)
 *        X轴：底座0度时的水平正前方
 *        Y轴：底座0度时的水平左侧
 * @param theta0 关节0 (底座旋转)
 * @param theta1~3 关节1~3 (抬升角度，0度为水平，90度为垂直向上)
 */
void forward_kinematics(float theta0, float theta1, float theta2, float theta3,
                        float L1, float L2, float L3,
                        float *x, float *y, float *z)
{
    // 【统一为“向上为正”的标准数学角度】
    float th0_rad = theta0 * DEG_TO_RAD;
    // 【核心串联映射：使用你推导出的完美公式】
    float A1 = theta1;
    float A2 = theta1 - theta2 - 90.0f;  // 即你推导的 -(90 - theta1 + theta2)
    float A3 = A2 - theta3;
    float math_th1_rad = A1 * DEG_TO_RAD;
    float math_th2_rad = A2 * DEG_TO_RAD;
    float math_th3_rad = A3 * DEG_TO_RAD;

    // 1. 计算在垂直平面内的投影
    // r 是水平伸出的距离，z 是垂直高度
    float r1 = L1 * cosf(math_th1_rad);
    float z1 = L1 * sinf(math_th1_rad);
    float r2 = L2 * cosf(math_th2_rad);
    float z2 = L2 * sinf(math_th2_rad);
    float r3 = L3 * cosf(math_th3_rad);
    float z3 = L3 * sinf(math_th3_rad);

    // 2. 叠加所有分量
    // r_total = 底座水平偏移 + 大臂投影 + 小臂投影
    float r_total =  r1 + r2 + r3;

    // z_total = 底座高度 + 大臂升高度 + 小臂升高度 - 三臂下垂长度
    float z_total = z1 + z2 + z3;

    // 3. 映射到 3D 空间
    *x = r_total * cosf(th0_rad);
    *y = r_total * sinf(th0_rad);
    *z = z_total;

#if DEBUG_KINEMATICS == 1
    printf("\r\n[FK 正解核对] 电机角: Th1=%.1f, Th2=%.1f, Th3=%.1f\r\n", theta1, theta2, theta3);
    printf("  -> L1 绝对角=%.1f | X投影=%.2f, Z高度=%.2f\r\n", A1, r1, z1);
    printf("  -> L2 绝对角=%.1f | X投影=%.2f, Z高度=%.2f\r\n", A2, r2, z2);
    printf("  -> L3 绝对角=%.1f | X投影=%.2f, Z高度=%.2f\r\n", A3, r3, z3);
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
    float r_target = sqrtf(x*x + y*y);     // 计算末端在机械臂平面内的投影距离 r_target
    float z_target = z;

    // 腕部解耦 (Wrist Decoupling)
    // 我们已知吸盘末端坐标，需要反推“小臂末端(手腕)”在哪里。
    // 吸盘的绝对俯仰角target_absolute_pitch（相对于地面）为 -90°。
    // 因为吸盘垂直向下 (-90度)，所以手腕就在吸盘正上方 L3 的位置。
    // 如果 target_absolute_pitch 不一定是 -90，通用公式如下：
    float pitch_rad = target_absolute_pitch * DEG_TO_RAD;
    // 手腕 r = 目标 r - (L3 * cos(角度))
    float r_wrist = r_target - ARM_L3 * cosf(pitch_rad);
    // 手腕 z = 目标 z - (L3 * sin(角度))
    // sin(-90) = -1, 所以这里 z_wrist = z_target + L3，符合直觉
    float z_wrist = z_target - ARM_L3 * sinf(pitch_rad);

    // 4. 两连杆解算 (大臂 L1，小臂 L2)
    float D_sq = r_wrist*r_wrist + z_wrist*z_wrist;
    float D = sqrtf(D_sq); // 从原点到手腕的距离D
    // 检查工作空间
    if (D > (ARM_L1 + ARM_L2) || D < fabsf(ARM_L1 - ARM_L2)) {
        return -1; // 不可达
    }

    // 利用余弦定理计算几何角度
    // Alpha: 大臂与 D 连线的夹角
    float cos_alpha = (ARM_L1*ARM_L1 + D_sq - ARM_L2*ARM_L2) / (2.0f * ARM_L1 * D);
    float alpha = acosf(clamp_float(cos_alpha, -1.0f, 1.0f));

    // Beta: D 连线与水平线的夹角
    float beta = atan2f(z_wrist, r_wrist);

    // --- 计算大臂电机角度 theta1 ---
    // 几何定义：0度水平，90度垂直。逆时针为正。
    float A1_rad = beta + alpha; // 这里选择“肘部朝上/朝外”的解
    // 大臂与手腕连线之间的夹角 α 使大臂在连线之上，即大臂角度 A1 = β + α
    float A1 = A1_rad * RAD_TO_DEG;

    float r_L1_end = ARM_L1 * cosf(A1_rad); // 大臂末端的水平投影
    float z_L1_end = ARM_L1 * sinf(A1_rad); // 大臂末端的高度
    float A2 = atan2(z_wrist - z_L1_end, r_wrist - r_L1_end) * RAD_TO_DEG;

    // --- 5. 映射回你的电机定义 (最容易错的地方) ---
    // 大臂：定义一致 (90=垂直)
    angles->theta1 = A1;
    angles->theta2 = angles->theta1 - A2 - 90.0f;
    angles->theta3 = target_absolute_pitch - A2;

    return 0;
}


/**
 * @brief 对目标角度进行安全限制和路径优化
 */
JointAngles_t optimize_and_limit_angles(JointAngles_t target_input, float current_theta0)
{
    // JointAngles_t safe_angles;
    //
    // /* 1. 关节0：最短路径优化 */
    // // 自动选择顺时针或逆时针旋转
    // safe_angles.theta0 = find_shortest_path(target_input.theta0, current_theta0);
    //
    // /* 2. 关节1~3：安全范围限制 (Clamping) */
    // // 强制将角度限制在机械结构允许的范围内
    // safe_angles.theta1 = clamp_float(target_input.theta1, J1_MIN, J1_MAX);
    // safe_angles.theta2 = clamp_float(target_input.theta2, J2_MIN, J2_MAX);
    // safe_angles.theta3 = clamp_float(target_input.theta3, J3_MIN, J3_MAX);
    //
    // return safe_angles;
    return target_input;
}