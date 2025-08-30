#ifndef TIMER_H
#define TIMER_H

#include "driver/timer.h"
#include <stdio.h>

void TIMER_INIT();//定时器初始化
bool IRAM_ATTR speed(void *para);//定时器中断函数配置
bool IRAM_ATTR Pid_controller(void *para);//pid控制器中断函数配置
void esp_timer_cb(void *arg);


#endif
