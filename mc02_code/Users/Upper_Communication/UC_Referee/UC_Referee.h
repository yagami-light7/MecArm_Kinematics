#ifndef INFANTRY_ROBOT_UC_REFEREE_H
#define INFANTRY_ROBOT_UC_REFEREE_H

#include "main.h"

#include "UC_Protocol.h"

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;
typedef __packed struct // 0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct // 0x0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct // 0x0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct // 0x0005
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
    uint8_t lurk_mode;
    uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;
typedef __packed struct // 0x0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct // 0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct // 0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct // 0x0104
{
    uint8_t level;
    uint8_t foul_robot_id;
    uint8_t count;
} ext_referee_warning_t;
typedef __packed struct // 0x0105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_remaining_time_t;

typedef __packed struct // 0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_barrel_cooling_value; //枪口每秒冷却值
    uint16_t shooter_barrel_heat_limit; //枪口热量上限
    uint16_t chassis_power_limit; //底盘功率上限
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct // 0x202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
typedef __packed struct // 0x0203
{
    float x;
    float y;
    float angle; //机器人测速模块朝向，单位：度。正北为0度
} ext_game_robot_pos_t;

typedef __packed struct // 0x0204
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_t;
typedef __packed struct // 0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct // 0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct // 0x0207
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct // 0x0208
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct // 0x0209
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/*
数据段头结构 6字节
内容               ID长度（头结构长度+内容数据段长度）              功能说明
0x0200~0x02FF                   6+n                          己方机器人间通信
0x0100                          6+2                           客户端删除图形
0x0101                          6+15                        客户端绘制一个图形
0x0102                          6+30                        客户端绘制二个图形
0x0103                          6+75                        客户端绘制五个图形
0x0104                          6+105                       客户端绘制七个图形
0x0110                          6+45                        客户端绘制字符图形
*/
typedef __packed struct // 0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;



typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

extern ext_game_robot_status_t robot_state;

#ifdef __cplusplus
extern "C"{
#endif

extern void init_referee_struct_data(void);

extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(float *power, uint16_t *buffer);

extern void get_robot_id(uint16_t *robot_id, uint16_t *client_id);

extern void robot_id(uint16_t *robot_id);

extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

extern void get_shoot_heat2_limit_and_heat2(uint16_t *heat2_limit, uint16_t *heat2);

extern void get_robot_hurt(uint8_t *armor_id, uint8_t *hurt_type);

extern void get_chassis_power_limit(uint16_t *power_limit);

extern void get_robot_level(uint8_t *level);

extern void get_ammo(uint16_t *bullet_remaining);

extern void get_sentry_HP(uint16_t *sentry_HP);

#ifdef __cplusplus
}
#endif

#endif

