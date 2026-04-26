//
// Created by RM UI Designer
//

#include "ui_g_Dynamic_3.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 3

ui_string_frame_t ui_g_Dynamic_3;

ui_interface_string_t* ui_g_Dynamic_CUSTOM_ENABLE = &ui_g_Dynamic_3.option;

void _ui_init_g_Dynamic_3() {
    ui_g_Dynamic_3.option.figure_name[0] = FRAME_ID;
    ui_g_Dynamic_3.option.figure_name[1] = GROUP_ID;
    ui_g_Dynamic_3.option.figure_name[2] = START_ID;
    ui_g_Dynamic_3.option.operate_tpyel = 1;
    ui_g_Dynamic_3.option.figure_tpye = 7;
    ui_g_Dynamic_3.option.layer = 0;
    ui_g_Dynamic_3.option.font_size = 30;
    ui_g_Dynamic_3.option.start_x = 1400;
    ui_g_Dynamic_3.option.start_y = 478;
    ui_g_Dynamic_3.option.color = 6;
    ui_g_Dynamic_3.option.str_length = 13;
    ui_g_Dynamic_3.option.width = 3;
    strcpy(ui_g_Dynamic_CUSTOM_ENABLE->string, "CUSTOM_ENABLE");

    ui_proc_string_frame(&ui_g_Dynamic_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_3, sizeof(ui_g_Dynamic_3));
}

void _ui_update_g_Dynamic_3() {
    ui_g_Dynamic_3.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Dynamic_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_3, sizeof(ui_g_Dynamic_3));
}

void _ui_remove_g_Dynamic_3() {
    ui_g_Dynamic_3.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Dynamic_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_3, sizeof(ui_g_Dynamic_3));
}