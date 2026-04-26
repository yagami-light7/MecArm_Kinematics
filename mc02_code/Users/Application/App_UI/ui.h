//
// Created by RM UI Designer
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_g_Static_0.h"
#include "ui_g_Static_1.h"
#include "ui_g_Static_2.h"

#define ui_init_g_Static()  \
_ui_init_g_Static_0();      \
vTaskDelay(50);             \
_ui_init_g_Static_1();      \
vTaskDelay(50);             \
_ui_init_g_Static_2();      \
vTaskDelay(50)             \

#define ui_update_g_Static() \
_ui_update_g_Static_0(); \
_ui_update_g_Static_1(); \
_ui_update_g_Static_2()

#define ui_remove_g_Static() \
_ui_remove_g_Static_0(); \
_ui_remove_g_Static_1(); \
_ui_remove_g_Static_2()
    

#include "ui_g_Dynamic_0.h"
#include "ui_g_Dynamic_1.h"
#include "ui_g_Dynamic_2.h"
#include "ui_g_Dynamic_3.h"
#include "ui_g_Dynamic_4.h"

#define ui_init_g_Dynamic() \
_ui_init_g_Dynamic_1();     \
vTaskDelay(50);             \
_ui_init_g_Dynamic_2();     \
vTaskDelay(50);             \
_ui_init_g_Dynamic_3();     \
vTaskDelay(50);             \
_ui_init_g_Dynamic_4();     \
vTaskDelay(50)              \

#define ui_update_g_Dynamic() \
_ui_update_g_Dynamic_0();     \
vTaskDelay(10);               \
_ui_update_g_Dynamic_1();     \
vTaskDelay(10);               \
_ui_update_g_Dynamic_2();     \
vTaskDelay(10);               \
_ui_update_g_Dynamic_3();     \
vTaskDelay(10);               \
_ui_update_g_Dynamic_4();     \
vTaskDelay(10);               \

#define ui_remove_g_Dynamic() \
_ui_remove_g_Dynamic_0(); \
_ui_remove_g_Dynamic_1(); \
_ui_remove_g_Dynamic_2(); \
_ui_remove_g_Dynamic_3(); \
_ui_remove_g_Dynamic_4()
    


#ifdef __cplusplus
}
#endif

#endif //UI_H
