#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"

//�������Լ���Ӧ�豸˳��
enum errorList
{
    DBUS_TOE = 0,//ң����
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
    uint32_t newTime;                 //���¼�¼ʱ��
    uint32_t lastTime;                //�ϴμ�¼ʱ��
    uint32_t Losttime;                //����ʱ��
    uint32_t worktime;                //����ʱ��
    uint32_t setOfflineTime : 12;     //��������ʱ����ֵ
    uint32_t setOnlineTime  : 12;     //��������ʱ����ֵ
    uint32_t enable : 1;               //�豸���ʹ�ܰ�ť
    uint32_t isLost : 1;               //�豸����״̬

} error_t;

void detect_task(void const *pvParameters);
void detect_hook(uint8_t toe);
uint8_t toe_is_error(uint8_t err);
const error_t *get_error_list_point(void);
void toe_enable_cmd(uint8_t toe,uint8_t cmd);
#endif
