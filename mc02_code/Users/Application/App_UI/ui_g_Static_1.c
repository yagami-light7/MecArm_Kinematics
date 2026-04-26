//
// Created by RM UI Designer
//

#include "ui_g_Static_1.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 1

ui_string_frame_t ui_g_Static_1;

ui_interface_string_t* ui_g_Static_Custom = &ui_g_Static_1.option;

void _ui_init_g_Static_1() {
    ui_g_Static_1.option.figure_name[0] = FRAME_ID;
    ui_g_Static_1.option.figure_name[1] = GROUP_ID;
    ui_g_Static_1.option.figure_name[2] = START_ID;
    ui_g_Static_1.option.operate_tpyel = 1;
    ui_g_Static_1.option.figure_tpye = 7;
    ui_g_Static_1.option.layer = 0;
    ui_g_Static_1.option.font_size = 30;
    ui_g_Static_1.option.start_x = 28;
    ui_g_Static_1.option.start_y = 872;
    ui_g_Static_1.option.color = 7;
    ui_g_Static_1.option.str_length = 6;
    ui_g_Static_1.option.width = 3;
    strcpy(ui_g_Static_Custom->string, "CUSTOM");

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_update_g_Static_1() {
    ui_g_Static_1.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_remove_g_Static_1() {
    ui_g_Static_1.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}