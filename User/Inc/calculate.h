//
// Created by 18375 on 2026/3/2.
//

#ifndef DR_CALCULATE_CALCULATE_H
#define DR_CALCULATE_CALCULATE_H

#include <stdint.h>
#include <math.h>

/* 关节角度结构体 */
typedef struct {
    float theta0;
    float theta1;
    float theta2;
} JointAngles_t;

/* --- 关节安全范围定义 --- */
// 关节0：底座旋转
#define J0_MIN  -180.0f
#define J0_MAX  180.0f
// 关节1：大臂角度
#define J1_MIN  0.0f
#define J1_MAX  120.0f
// 关节2：小臂角度 (直接角度限制)
#define J2_MIN  -20.0f
#define J2_MAX  120.0f

/* --- 大臂和小臂夹角范围定义 --- */
// 夹角 = theta1 - theta2
#define ARM_ANGLE_MIN  30.0f   // 最小夹角 (避免碰撞)
#define ARM_ANGLE_MAX  150.0f  // 最大夹角 (避免过度伸展)

void forward_kinematics(float theta0, float theta1, float theta2, float theta3,
                        float L1, float L2, float L3,
                        float *x, float *y, float *z);
void forward_kinematics_default(float theta0, float theta1, float theta2, float theta3,
                                float *x, float *y, float *z);
int inverse_kinematics(float x, float y, float z, float target_servo, JointAngles_t *angles);
/**
 * @brief 对目标角度进行安全限制和路径优化
 * @param target_input 期望到达的角度（用户输入）
 * @param current_theta0 当前theta0电机的实际角度（用于计算最短路径）
 * @return 处理后的安全角度结构体
 */
JointAngles_t optimize_and_limit_angles(JointAngles_t target_input, float current_theta0);

#endif //DR_CALCULATE_CALCULATE_H