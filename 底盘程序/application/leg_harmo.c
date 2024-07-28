#include "leg_harmo.h"

/*
    #双腿协调
    1. 双腿协调PD计算
    2. 期望参数：左右角度差值(RIGHT-LEFT)
    3. PD输出与LQR中TP加减(RIGHT/LEFT)后输入至VMC
    
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
 * @description: 双腿协调PID参数初始化
 * @param {*}
 * @return {*}
 */
void leg_harmo_pid_init(void)
{
    PID_init(&leg_harmo.leg_harmo_pid,PID_POSITION, LEG_LEG_HARMO_PID, leg_LEG_HARMO_max_out,leg_LEG_HARMO_max_iout);
    //PID_init(&right_leg,PID_POSITION,LEG_LEG_TEST_PID,leg_LEG_TEST_max_out,leg_LEG_TEST_max_iout);
}

/**
 * @description: 清除PID
 * @param {*}
 * @return {*}
 */
void leg_harmo_pid_clear(void)
{
    PID_clear(&leg_harmo.leg_harmo_pid);
}


/**
 * @description: 双腿角度差传入PD，结果融合LQR计算TP后传入VMC得到髋关节电机力矩
 * @param 左右腿角度差
 * @return {*}
 */
float angle_error_set = 0;
float leg_harmo_out_right;
void leg_harmo_pid_cal(float leg_left_angle,float leg_right_angle)
{
    float leg_angle_error = (leg_right_angle + leg_left_angle)-3.14;
//    float leg_angle_error_left = motion_resolve[LEFT].point[X];
//    float leg_angle_error_right = motion_resolve[RIGHT].point[X];
    
    leg_harmo.leg_harmo_out = PID_calc(&leg_harmo.leg_harmo_pid,leg_angle_error,angle_error_set);//leg_harmo.leg_harmo_out为pd输出，作用于TP上
//    leg_harmo_out_right = pid_calc(&right_leg,leg_angle_error_right,angle_error_set);
}



