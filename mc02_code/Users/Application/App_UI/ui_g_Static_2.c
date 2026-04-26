//
// Created by RM UI Designer
//

#include "ui_g_Static_2.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 2

ui_string_frame_t ui_g_Static_2;

ui_interface_string_t* ui_g_Static_AUTO = &ui_g_Static_2.option;

void _ui_init_g_Static_2() {
    ui_g_Static_2.option.figure_name[0] = FRAME_ID;
    ui_g_Static_2.option.figure_name[1] = GROUP_ID;
    ui_g_Static_2.option.figure_name[2] = START_ID;
    ui_g_Static_2.option.operate_tpyel = 1;
    ui_g_Static_2.option.figure_tpye = 7;
    ui_g_Static_2.option.layer = 0;
    ui_g_Static_2.option.font_size = 30;
    ui_g_Static_2.option.start_x = 27;
    ui_g_Static_2.option.start_y = 796;
    ui_g_Static_2.option.color = 7;
    ui_g_Static_2.option.str_length = 4;
    ui_g_Static_2.option.width = 3;
    strcpy(ui_g_Static_AUTO->string, "AUTO");

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_update_g_Static_2() {
    ui_g_Static_2.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_remove_g_Static_2() {
    ui_g_Static_2.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}