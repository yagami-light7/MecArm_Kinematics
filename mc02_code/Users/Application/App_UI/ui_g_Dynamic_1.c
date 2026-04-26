//
// Created by RM UI Designer
//

#include "ui_g_Dynamic_1.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 1

ui_string_frame_t ui_g_Dynamic_1;

ui_interface_string_t* ui_g_Dynamic_CUSTOM_ON = &ui_g_Dynamic_1.option;

void _ui_init_g_Dynamic_1() {
    ui_g_Dynamic_1.option.figure_name[0] = FRAME_ID;
    ui_g_Dynamic_1.option.figure_name[1] = GROUP_ID;
    ui_g_Dynamic_1.option.figure_name[2] = START_ID;
    ui_g_Dynamic_1.option.operate_tpyel = 1;
    ui_g_Dynamic_1.option.figure_tpye = 7;
    ui_g_Dynamic_1.option.layer = 0;
    ui_g_Dynamic_1.option.font_size = 30;
    ui_g_Dynamic_1.option.start_x = 297;
    ui_g_Dynamic_1.option.start_y = 874;
    ui_g_Dynamic_1.option.color = 6;
    ui_g_Dynamic_1.option.str_length = 2;
    ui_g_Dynamic_1.option.width = 3;
    strcpy(ui_g_Dynamic_CUSTOM_ON->string, "ON");

    ui_proc_string_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}

void _ui_update_g_Dynamic_1() {
    ui_g_Dynamic_1.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}

void _ui_remove_g_Dynamic_1() {
    ui_g_Dynamic_1.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}