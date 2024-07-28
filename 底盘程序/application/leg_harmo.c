#include "leg_harmo.h"

/*
    #˫��Э��
    1. ˫��Э��PD����
    2. �������������ҽǶȲ�ֵ(RIGHT-LEFT)
    3. PD�����LQR��TP�Ӽ�(RIGHT/LEFT)��������VMC
    
*/

leg_harmo_t leg_harmo;
pid_type_def right_leg;
float LEG_LEG_HARMO_PID[3] = {80,0,100};
float LEG_LEG_TEST_PID[3];

float leg_LEG_HARMO_max_out = 10;
float leg_LEG_HARMO_max_iout = 10;

float leg_LEG_TEST_max_out;
float leg_LEG_TEST_max_iout;


/**
 * @description: ˫��Э��PID������ʼ��
 * @param {*}
 * @return {*}
 */
void leg_harmo_pid_init(void)
{
    PID_init(&leg_harmo.leg_harmo_pid,PID_POSITION, LEG_LEG_HARMO_PID, leg_LEG_HARMO_max_out,leg_LEG_HARMO_max_iout);
    //PID_init(&right_leg,PID_POSITION,LEG_LEG_TEST_PID,leg_LEG_TEST_max_out,leg_LEG_TEST_max_iout);
}

/**
 * @description: ���PID
 * @param {*}
 * @return {*}
 */
void leg_harmo_pid_clear(void)
{
    PID_clear(&leg_harmo.leg_harmo_pid);
}


/**
 * @description: ˫�ȽǶȲ��PD������ں�LQR����TP����VMC�õ��Źؽڵ������
 * @param �����ȽǶȲ�
 * @return {*}
 */
float angle_error_set = 0;
float leg_harmo_out_right;
void leg_harmo_pid_cal(float leg_left_angle,float leg_right_angle)
{
    float leg_angle_error = (leg_right_angle + leg_left_angle)-3.14;
//    float leg_angle_error_left = motion_resolve[LEFT].point[X];
//    float leg_angle_error_right = motion_resolve[RIGHT].point[X];
    
    leg_harmo.leg_harmo_out = PID_calc(&leg_harmo.leg_harmo_pid,leg_angle_error,angle_error_set);//leg_harmo.leg_harmo_outΪpd�����������TP��
//    leg_harmo_out_right = pid_calc(&right_leg,leg_angle_error_right,angle_error_set);
}



