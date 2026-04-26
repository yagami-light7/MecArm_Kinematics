#include "Alg_Filter.h"

#define NUM_STAGES 2  // 假设我们有 2 个二阶滤波器级联

// 定义滤波器系数 (示例)
float32_t filterCoeffs[NUM_STAGES * 5] = {
        0.2929f, 0.5858f, 0.2929f, 1.0000f, -0.1716f,  // 第一阶二阶滤波器系数
        0.2929f, 0.5858f, 0.2929f, 1.0000f, -0.1716f   // 第二阶二阶滤波器系数
};

// 定义状态缓冲区
float32_t state[NUM_STAGES * 4] = {0};  // 初始化为零

// 定义滤波器实例
arm_biquad_cascade_df2T_instance_f32 filterInstance;

// 初始化滤波器 IIR (Infinite Impulse Response)
void IIR_Filter_init(void)
{
    arm_biquad_cascade_df2T_init_f32(&filterInstance, NUM_STAGES, filterCoeffs, state);
}

// 假设输入数据
float32_t inputData[10] = {6000,6000,6000,6000,6000,6000,6000,6000,6000,6000};
float32_t outputData[10];

// 使用IIR (Infinite Impulse Response)滤波器进行数据处理
void IIR_Filter_calc(void)
{
    arm_biquad_cascade_df2T_f32(&filterInstance, inputData, outputData, 10);
}
