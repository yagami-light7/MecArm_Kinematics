/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Alg_DH_Kinematics.c
  * @brief      机械臂正逆解算
  * @note       本文件初步尝试使用C++进行
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-1-2025      Light           1.Not done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "Alg_DH_Kinematics.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense> // 使用Eigen库进行矩阵运算

using namespace Eigen;

// DH参数结构体
struct DH_Params {
    double theta; // 关节角度
    double d;     // 关节偏移
    double a;     // 连杆长度
    double alpha; // 连杆扭转角
};

// 计算单个关节的变换矩阵
Matrix4d dh_transform(DH_Params dh)
{
    /*将角度转化为弧度*/
    double theta = dh.theta * M_PI / 180.0;
    double alpha = dh.alpha * M_PI / 180.0;

    Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), dh.a * cos(theta),
            sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), dh.a * sin(theta),
            0, sin(alpha), cos(alpha), dh.d,
            0, 0, 0, 1;

    return T;
}

// 正解计算
Matrix4d forward_kinematics(DH_Params dh_params[], int num_joints)
{
    Matrix4d T = Matrix4d::Identity(); // 单位矩阵

    for (int i = 0; i < num_joints; i++) {
        T = T * dh_transform(dh_params[i]);
    }

    return T;
}

int test() {
    // 定义DH参数
    DH_Params dh_params[6] = {
            {30, 0.1, 0.2, 90}, // 关节1
            {45, 0.1, 0.2, 0},  // 关节2
            {60, 0.1, 0.2, 90}, // 关节3
            {90, 0.1, 0.2, 0},  // 关节4
            {45, 0.1, 0.2, 90}, // 关节5
            {30, 0.1, 0.2, 0}   // 关节6
    };

    // 计算正解
    Matrix4d T = forward_kinematics(dh_params, 6);

    // 输出末端执行器的位置
    std::cout << "End Effector Position: [" << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3) << "]\n";

    return 0;
}