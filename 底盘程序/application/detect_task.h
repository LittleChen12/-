#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"

//错误码以及对应设备顺序
enum errorList
{
    DBUS_TOE = 0,//遥控器
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    BOARD_ACCEL_TOE,
    BOARD_GYRO_TOE,
    BOARD_MAG_TOE,
    MINIPC_TOE,

    errorListLength,
};


typedef __packed struct
{
    uint32_t newTime;                 //最新记录时间
    uint32_t lastTime;                //上次记录时间
    uint32_t Losttime;                //掉线时间
    uint32_t worktime;                //上线时间
    uint32_t setOfflineTime : 12;     //设置离线时间阈值
    uint32_t setOnlineTime  : 12;     //设置上线时间阈值
    uint32_t enable : 1;               //设备监测使能按钮
    uint32_t isLost : 1;               //设备掉线状态

} error_t;

void detect_task(void const *pvParameters);
void detect_hook(uint8_t toe);
uint8_t toe_is_error(uint8_t err);
const error_t *get_error_list_point(void);
void toe_enable_cmd(uint8_t toe,uint8_t cmd);
#endif
