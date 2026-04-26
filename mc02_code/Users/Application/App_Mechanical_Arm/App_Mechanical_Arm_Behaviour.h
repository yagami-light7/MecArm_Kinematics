#ifndef MACHANICAL_ARM_BEHAVIOUR_H
#define MACHANICAL_ARM_BEHAVIOUR_H

#include "main.h"
#include "App_Mechanical_Arm_Task.h"

// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f

typedef enum
{
    MEC_ARM_ZERO_FORCE = 0, // 无力模式
    MEC_ARM_ZERO_POSITION,  // 零位模式
    MEC_ARM_ANGLE_CONTROL,  // 角度控制模式
    MEC_ARM_MOTIONLESS,     // 静置模式
    MEC_ARM_CUSTOM,         // 自定义控制器模式

    MEC_AUTO_GET_GND_MINE   // 一键取地矿模式
} mec_arm_behaviour_e;

extern mec_arm_behaviour_e  mec_arm_behaviour;

#ifdef __cplusplus
extern "C"{
#endif


extern void mec_arm_behaviour_mode_set(mec_arm_control_t *mec_arm_mode_set);
extern void mec_arm_behaviour_control_set(float *add_s_j0, float *add_s_j1, float *add_s_j2, mec_arm_control_t *mec_arm_control_set);
extern bool mec_arm_cmd_to_chassis_stop(void);

#ifdef __cplusplus
}
#endif

#endif
