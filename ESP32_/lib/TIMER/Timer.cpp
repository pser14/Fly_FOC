#include "Timer.h"
#include "Arduino.h"
void TIMER_INIT()
{
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,
    };

    timer_init(TIMER_GROUP_0,TIMER_0,&config);//初始化定时器
    timer_init(TIMER_GROUP_0,TIMER_1,&config);//初始化pid定时器

    timer_set_counter_value(TIMER_GROUP_0,TIMER_0,0);//设置定时值和中断
    timer_set_counter_value(TIMER_GROUP_0,TIMER_1,0);
    timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,1000000);//定时一秒
    timer_set_alarm_value(TIMER_GROUP_0,TIMER_1,10000);//pid计算频率1khz

    timer_isr_callback_add(TIMER_GROUP_0,TIMER_0,speed,NULL,0);//注册中断回调函数
    timer_isr_callback_add(TIMER_GROUP_0,TIMER_1,Pid_controller,NULL,0);//注册Pid控制器中断函数

    timer_start(TIMER_GROUP_0,TIMER_0);//启动定时器
    timer_start(TIMER_GROUP_0,TIMER_1);//启动定时器
}

