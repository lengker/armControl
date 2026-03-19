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
    float theta3;
} JointAngles_t;

/* --- 关节安全范围定义 --- */
// 关节1：0 ~ 180
#define J1_MIN  0.0f
#define J1_MAX  180.0f

// 关节2：-140 ~ -46.24 (注意：-140比-46小，所以-140是MIN)
#define J2_MIN  -30.0f
#define J2_MAX  50.0f//还有问题！！！

// 关节3：-40 ~ 140
#define J3_MIN  -20.0f
#define J3_MAX  120.0f

void forward_kinematics(float theta0, float theta1, float theta2, float theta3,
                        float L1, float L2, float L3,
                        float *x, float *y, float *z);
void forward_kinematics_default(float theta0, float theta1, float theta2, float theta3,
                                float *x, float *y, float *z);
int inverse_kinematics(float x, float y, float z, float target_servo, JointAngles_t *angles);
int inverse_kinematics2(float x, float y, float z, float target_servo, JointAngles_t *angles);
/**
 * @brief 对目标角度进行安全限制和路径优化
 * @param target_input 期望到达的角度（用户输入）
 * @param current_theta0 当前theta0电机的实际角度（用于计算最短路径）
 * @return 处理后的安全角度结构体
 */
JointAngles_t optimize_and_limit_angles(JointAngles_t target_input, float current_theta0);

#endif //DR_CALCULATE_CALCULATE_H