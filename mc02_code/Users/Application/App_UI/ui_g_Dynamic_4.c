//
// Created by RM UI Designer
//

#include "ui_g_Dynamic_4.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 4
#define OBJ_NUM 3
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_Dynamic_4;
ui_interface_round_t *ui_g_Dynamic_sucker_master_on = (ui_interface_round_t *)&(ui_g_Dynamic_4.data[0]);
ui_interface_round_t *ui_g_Dynamic_sucker_slave_left_on = (ui_interface_round_t *)&(ui_g_Dynamic_4.data[1]);
ui_interface_round_t *ui_g_Dynamic_sucker_slave_right_on = (ui_interface_round_t *)&(ui_g_Dynamic_4.data[2]);

void _ui_init_g_Dynamic_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Dynamic_4.data[i].figure_name[0] = FRAME_ID;
        ui_g_Dynamic_4.data[i].figure_name[1] = GROUP_ID;
        ui_g_Dynamic_4.data[i].figure_name[2] = i + START_ID;
        ui_g_Dynamic_4.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_Dynamic_4.data[i].operate_tpyel = 0;
    }

    ui_g_Dynamic_sucker_master_on->figure_tpye = 2;
    ui_g_Dynamic_sucker_master_on->layer = 0;
    ui_g_Dynamic_sucker_master_on->r = 5;
    ui_g_Dynamic_sucker_master_on->start_x = 1574;
    ui_g_Dynamic_sucker_master_on->start_y = 727;
    ui_g_Dynamic_sucker_master_on->color = 2;
    ui_g_Dynamic_sucker_master_on->width = 90;

    ui_g_Dynamic_sucker_slave_left_on->figure_tpye = 2;
    ui_g_Dynamic_sucker_slave_left_on->layer = 0;
    ui_g_Dynamic_sucker_slave_left_on->r = 5;
    ui_g_Dynamic_sucker_slave_left_on->start_x = 1474;
    ui_g_Dynamic_sucker_slave_left_on->start_y = 619;
    ui_g_Dynamic_sucker_slave_left_on->color = 2;
    ui_g_Dynamic_sucker_slave_left_on->width = 70;

    ui_g_Dynamic_sucker_slave_right_on->figure_tpye = 2;
    ui_g_Dynamic_sucker_slave_right_on->layer = 0;
    ui_g_Dynamic_sucker_slave_right_on->r = 5;
    ui_g_Dynamic_sucker_slave_right_on->start_x = 1674;
    ui_g_Dynamic_sucker_slave_right_on->start_y = 619;
    ui_g_Dynamic_sucker_slave_right_on->color = 2;
    ui_g_Dynamic_sucker_slave_right_on->width = 70;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Dynamic_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_4, sizeof(ui_g_Dynamic_4));
}

void _ui_update_g_Dynamic_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Dynamic_4.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Dynamic_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_4, sizeof(ui_g_Dynamic_4));
}

void _ui_remove_g_Dynamic_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Dynamic_4.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Dynamic_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_4, sizeof(ui_g_Dynamic_4));
}
