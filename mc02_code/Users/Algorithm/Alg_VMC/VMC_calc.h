#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#include "main.h"
#include "INS_task.h"

#define pi 3.1415926f
#define LEG_PID_KP  700.0f
#define LEG_PID_KI  0.5f//不积分
#define LEG_PID_KD  3000.0f
#define LEG_PID_MAX_OUT  100.0f 
#define LEG_PID_MAX_IOUT 50.0f

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5; // AE长度 //单位为m
	float l1; // 单位为m
	float l2; // 单位为m
	float l3; // 单位为m
	float l4; // 单位为m

	float XB,YB;//B点的坐标
	float XD,YD;//D点的坐标
	
	float XC,YC;//C点的直角坐标
	float L0,phi0;//C点的极坐标
	float alpha;
	float d_alpha;	
	
	float	lBD;//BD两点的距离
	
	float d_phi0;//现在C点角度phi0的变换率
	float last_phi0;//上一次C点角度，用于计算角度phi0的变换率d_phi0

	float A0,B0,C0;//中间变量
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float torque_set[4];

	float F0;
	float Tp;
	
	float theta;
	float d_theta;//theta的一阶导数
	float last_d_theta;
	float dd_theta;//theta的二阶导数
	
	float d_L0;//L0的一阶导数
	float dd_L0;//L0的二阶导数
	float last_L0;
	float last_d_L0;
	
	float FN;//支持力
	
	uint8_t first_flag;
	uint8_t leg_set_flag;//腿长完成标志

	float leg_set;//期望腿长
} vmc_leg_t;

extern void VMC_init(vmc_leg_t *vmc);//给杆长赋值

extern void VMC_calc_1(vmc_leg_t *vmc,INS_t *ins,float dt);//计算theta和d_theta给lqr用，同时也计算腿长L0
extern void VMC_calc_1_left(vmc_leg_t *vmc,INS_t *ins,float dt);
extern void VMC_calc_2(vmc_leg_t *vmc);//计算期望的关节输出力矩

extern uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins);//右腿离地检测
extern uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins);//左腿离地检测

extern float LQR_K_calc(float *coe,float len);
	
#endif




