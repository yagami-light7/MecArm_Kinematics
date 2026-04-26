#ifndef ENGINEERING_APP_REFEREE_TASK_H
#define ENGINEERING_APP_REFEREE_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "UC_Referee.h"
#include "UC_Protocol.h"
#include "Dev_Referee.h"
#include "Board_CRC8_CRC16.h"
#include "Board_USART.h"

#define Referee_TASK_INIT_TIME 501
#define Referee_TASK_CONTROL_TIME 2

extern void Referee_Task(void *pvParameters);

#endif
