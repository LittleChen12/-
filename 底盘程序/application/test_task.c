/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.��������������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;



/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();

    while(1)
    {
        //find error
        //���ִ���
        for(error_num = 0; error_num < errorListLength; error_num++)
        {
            if(error_list_test_local[error_num].isLost && error_list_test_local[error_num].enable)
            {
                break;
            }
        }

        //buzzer_warn_error(error_num + 1);

        osDelay(10);
    }
}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps
  * @retval         none
  */
/**
  * @brief          ʹ�÷�������
  * @param[in]      num:��������
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    static uint8_t last_num = 0;
    if(num == errorListLength + 1)
    {
        buzzer_off();
        show_num = 0;
        stop_num = 0;
        return;
    }
    if(last_num != num)
    {
        last_num = num;
        show_num = 0;
        stop_num = 0;
    }
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 25)
        {
            buzzer_off();
        }
        else if(tick < 50)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


