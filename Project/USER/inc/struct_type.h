#ifndef _STRUCTTYPE_H
#define _STRUCTTYPE_H

#include "common.h"

//计数器类型
typedef struct time_count_tag {
    uint16 count_1ms;   //1ms计时器   
    uint8 balance_count;     //直立控制计数器
    uint8 get_speed_conut;  //速度采集计数器
    uint8 control_speed_count;  //速度控制计数器
    uint8 turn_count;    //转向控制计数器
    uint8 turn_gyro_count;     //转向角速度环计数器
    uint8 uart_section;   //串口发送数据时序计数器   
} Time_Count_Type_Def;

//时间类型
typedef struct time_tag {
    uint16 ms;  
    uint8 hour;
    uint8 minute;
    uint8 second;
} Time_Type_Def;

typedef struct debug_mod_tag {
    int8 balance;
    int8 turn;
    int8 speed;
    int8 protect;
} Debug_Mod_Type_Def;

#endif