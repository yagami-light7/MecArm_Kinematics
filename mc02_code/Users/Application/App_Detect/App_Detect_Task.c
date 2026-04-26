/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      检测错误任务， 通过接收数据时间来判断.提供 检测钩子函数,错误存在函数.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    如果要添加一个新设备
    1.第一步在detect_task.h，添加设备名字在errorList的最后，像
    enum errorList
    {
        ...
        XXX_TOE,    //新设备
        ERROR_LIST_LENGHT,
    };
    2.在detect_init函数,添加offlineTime, onlinetime, priority参数
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.如果有data_is_error_fun ,solve_lost_fun,solve_data_error_fun函数，赋值到函数指针
    4.在XXX_TOE设备数据来的时候, 添加函数detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "App_Detect_Task.h"
#include "cmsis_os.h"

/**
 * @brief          初始化error_list,赋值 offline_time, online_time, priority
 * @param[in]      time:系统时间戳
 * @retval         none
 */
static void detect_init(uint32_t time);

error_t error_list[ERROR_LIST_LENGHT + 1];


#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t detect_task_stack;
#endif


/**
 * @brief          检测任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void Detect_Task(void *pvParameters)
{
    static TickType_t system_time;
    system_time = xTaskGetTickCount();

    /* 初始化 */
    detect_init(system_time);

    /* 空闲以待初始化 */
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        static uint8_t error_num_display = 0;
        system_time = xTaskGetTickCount();

        error_num_display = ERROR_LIST_LENGHT;
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            // 未使能，跳过
            if (error_list[i].enable == 0)
            {
                continue;
            }

            // 判断掉线
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
                if (error_list[i].error_exist == 0)
                {
                    // 记录错误以及掉线时间
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_time;
                }

                // 判断错误优先级， 保存优先级最高的错误码
                if (error_list[i].priority > error_list[error_num_display].priority)
                {
                    error_num_display = i;
                }

                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;

                // 如果提供解决函数，运行解决函数
                if (error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }
            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                // 刚刚上线，可能存在数据不稳定，只记录不丢失，
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].is_lost = 0;
                // 判断是否存在数据错误
                if (error_list[i].data_is_error != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }
                // 计算频率
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (float)(error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          获取设备对应的错误状态
 * @param[in]      toe:设备目录
 * @retval         true(错误) 或者false(没错误)
 */
bool toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
 * @brief          记录时间
 * @param[in]      toe:设备目录
 * @retval         none
 */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();

    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }

    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}


/**
 * @brief          得到错误列表
 * @param[in]      none
 * @retval         error_list的指针
 */
const error_t *get_error_list_point(void)
{
    return error_list;
}


/**
 * @brief          初始化error_list,赋值 offline_time, online_time, priority
 * @param[in]      time:系统时间戳
 * @retval         none
 */
static void detect_init(uint32_t time)
{
    // 设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            {500, 40, ERROR_LIST_LENGHT - 0},    // DBUS
            {500, 40, ERROR_LIST_LENGHT - 0},    // VT_TOE
            {500, 40, ERROR_LIST_LENGHT - 0},    // CUSTOM_TOE
            {1000, 500, ERROR_LIST_LENGHT - 1}, // CHASSIS_MOTOR1
            {1000, 500, ERROR_LIST_LENGHT - 2}, // CHASSIS_MOTOR2
            {1000, 500, ERROR_LIST_LENGHT - 3}, // CHASSIS_MOTOR3
            {1000, 500, ERROR_LIST_LENGHT - 4}, // CHASSIS_MOTOR4

            {20, 3, ERROR_LIST_LENGHT - 5},      // GIMBAL_YAW_MOTOR
            {20, 3, ERROR_LIST_LENGHT - 6},      // L_JOINT_MOTOR1
            {20, 3, ERROR_LIST_LENGHT - 7},      // L_JOINT_MOTOR2
            {20, 3, ERROR_LIST_LENGHT - 8},      // S_JOINT_MOTOR3
            {20, 3, ERROR_LIST_LENGHT - 9},     // S_JOINT_MOTOR4
            {20,3, ERROR_LIST_LENGHT - 10},     // S_JOINT_MOTOR5

        };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }


    //    error_list[DBUSTOE].dataIsErrorFun = RC_data_is_error;
    //    error_list[DBUSTOE].solveLostFun = slove_RC_lost;
    //    error_list[DBUSTOE].solveDataErrorFun = slove_data_error;
}
