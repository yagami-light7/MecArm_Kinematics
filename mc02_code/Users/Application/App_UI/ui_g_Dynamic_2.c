//
// Created by RM UI Designer
//

#include "ui_g_Dynamic_2.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 2

ui_string_frame_t ui_g_Dynamic_2;

ui_interface_string_t* ui_g_Dynamic_AUTO_ON = &ui_g_Dynamic_2.option;

void _ui_init_g_Dynamic_2() {
    ui_g_Dynamic_2.option.figure_name[0] = FRAME_ID;
    ui_g_Dynamic_2.option.figure_name[1] = GROUP_ID;
    ui_g_Dynamic_2.option.figure_name[2] = START_ID;
    ui_g_Dynamic_2.option.operate_tpyel = 1;
    ui_g_Dynamic_2.option.figure_tpye = 7;
    ui_g_Dynamic_2.option.layer = 0;
    ui_g_Dynamic_2.option.font_size = 30;
    ui_g_Dynamic_2.option.start_x = 297;
    ui_g_Dynamic_2.option.start_y = 800;
    ui_g_Dynamic_2.option.color = 5;
    ui_g_Dynamic_2.option.str_length = 2;
    ui_g_Dynamic_2.option.width = 3;
    strcpy(ui_g_Dynamic_AUTO_ON->string, "ON");

    ui_proc_string_frame(&ui_g_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_2, sizeof(ui_g_Dynamic_2));
}

void _ui_update_g_Dynamic_2() {
    ui_g_Dynamic_2.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_2, sizeof(ui_g_Dynamic_2));
}

void _ui_remove_g_Dynamic_2() {
    ui_g_Dynamic_2.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_2, sizeof(ui_g_Dynamic_2));
}