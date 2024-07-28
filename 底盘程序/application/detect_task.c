#include "detect_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*------------------------------------------------------*/
/*                     �������豸�ṹ��               */
/*------------------------------------------------------*/
static error_t errorList[errorListLength];

void DetectInit(uint32_t time);
void DetectDisplay(void);

//�����ж�����
void detect_task(void const *pvParameters)
{
    static uint8_t i;
    static uint32_t systemTime;

    systemTime = xTaskGetTickCount();

    //��ʼ��
    DetectInit(systemTime);

    while (1)
    {
        systemTime = xTaskGetTickCount();

        for (i = 0; i < errorListLength; i++)
        {
            //δʹ�ܣ������豸���
            if (errorList[i].enable == 0)
            {
                continue;
            }

            //�������ų�
            else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)
            {
                errorList[i].isLost = 0;
            }

            //��ʱ����
            else if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].isLost == 0)
                {
                    //��¼�����Լ�����ʱ��
                    errorList[i].isLost = 1;
                    errorList[i].Losttime = systemTime;
                }
            }
            //��������
            else
            {
                errorList[i].isLost = 0;
            }
        }

        vTaskDelay(10);
    }
}
/*------------------------------------------------------*/
/*                       ���߼���ʼ��                 */
/*------------------------------------------------------*/
void DetectInit(uint32_t time)
{
    //����ʱ����ֵ ������ʱ����ֵ��ʹ��״̬
    uint16_t setItem[errorListLength][3] =
    {
        {50, 0, 1},   //DBUS
        {40, 0, 1},   //MOTOR1
        {40, 0, 1},   //MOTOR1
        {40, 0, 1},   //MOTOR1
        {40, 0, 1},   //MOTOR1
        {50, 0, 1},   //ACCEL
        {50, 0, 1},   //GYRO
        {50, 0, 1},   //MAG
        {50, 0, 0},   //MINIPC
    };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime =  setItem[i][1];
        errorList[i].enable = setItem[i][2];

        errorList[i].isLost = 1;
        errorList[i].newTime =  time;
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

}

/*------------------------------------------------------*/
/*                   ���ض�Ӧ���豸�Ƿ�ʧ             */
/*------------------------------------------------------*/

uint8_t toe_is_error(uint8_t err)
{
    return (errorList[err].isLost == 1);
}
/**
  * @brief          �õ������б�
  * @param[in]      none
  * @retval         error_list��ָ��
  */
const error_t *get_error_list_point(void)
{
    return errorList;
}
/*------------------------------------------------------*/
/*                   �豸�������ݹ��Ӻ���               */
/*------------------------------------------------------*/

void detect_hook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime  = xTaskGetTickCount();

    //���¶�ʧ���
    if (errorList[toe].isLost)
    {
        errorList[toe].isLost   = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
}
void toe_enable_cmd(uint8_t toe,uint8_t cmd)
{
    if(cmd) errorList[toe].enable = 1;
    else errorList[toe].enable = 0;
}

