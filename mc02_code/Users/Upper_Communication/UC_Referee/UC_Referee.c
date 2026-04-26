#include "UC_Referee.h"
#include "string.h"
#include "stdio.h"
#include "Board_CRC8_CRC16.h"
#include "UC_Protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_status_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;


ext_game_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_header_data_t student_interactive_data_t;




void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_status_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_header_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_status_t));
        }
            break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
            break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
            break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
            break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
            break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
            break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
            break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
            break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
            break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
            break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
            break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
            break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
            break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
            break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
            break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(float *power, uint16_t *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}

void get_robot_id(uint16_t *robot_id, uint16_t *client_id)
{
    *robot_id = robot_state.robot_id;
    *client_id = robot_state.robot_id + 256;
}

void robot_id(uint16_t *robot_id)
{
    *robot_id = robot_state.robot_id;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_heat_limit;
    *heat1 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

void get_shoot_heat2_limit_and_heat2(uint16_t *heat2_limit, uint16_t *heat2)
{
    *heat2_limit = robot_state.shooter_barrel_heat_limit;
    *heat2 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
}

void get_chassis_power_limit(uint16_t *power_limit)
{
    *power_limit = robot_state.chassis_power_limit;
}

void get_robot_hurt(uint8_t *armor_id, uint8_t *hurt_type)
{
    *armor_id = robot_hurt_t.armor_type;
    *hurt_type = robot_hurt_t.hurt_type;
}

void get_robot_level(uint8_t *level)
{
    *level = robot_state.robot_level;
}

void get_ammo(uint16_t *bullet_remaining)
{
    *bullet_remaining = bullet_remaining_t.bullet_remaining_num_17mm;
}

void get_sentry_HP(uint16_t *sentry_HP)
{
    if(robot_state.robot_id < 100)
    {
        *sentry_HP = game_robot_HP_t.red_7_robot_HP;
    }
    else
    {
        *sentry_HP = game_robot_HP_t.blue_7_robot_HP;
    }
}
