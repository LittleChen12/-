/*
 * @Description:
    #�ȳ�����
    1. �����������������ȳ�
    2. ״̬�������������ȳ�
    3. ��� = ���� - ״̬
    4. ��� -> PID&Feed Forward -> �Ӽ�(LEFT/RIGHT)Roll�Ჹ��[Kp]
    5. �������ȳ��������ɻ��������������̬�������õ��ĵ�����Ǽ���õ�����ʵ���ڷ�ˮƽ�����ϱ��ֻ����˻�������̬ˮƽ��
 */

#include "leg_length.h"
#include "VMC.h"
#include "chassis_task.h"

leg_length_t leg_length[2]; //[LEFT][RIGHT]
pid_type_def roll_comp;            // roll�Ჹ��

float LEG_LEFT_PID[3] = {80,0,4000};

float LEG_RIGHT_PID[3] = {80,0,4000};
float LEG_ROLL_PID[3] = {280,0,2500};//ROLL����

float leg_length_max_out = 60;
float leg_length_max_iout = 40;

float leg_roll_max_out = 400;
float leg_roll_max_iout = 400;

/**
 * @description: �ȳ�����PID������ʼ��
 * @param {*}
 * @return {*}
 */
void leg_length_pid_init(void)
{
	PID_init(&leg_length[0].leg_length_pid,PID_POSITION,LEG_LEFT_PID,leg_length_max_out,leg_length_max_iout);
	PID_init(&leg_length[1].leg_length_pid,PID_POSITION,LEG_RIGHT_PID,leg_length_max_out,leg_length_max_iout);	
	PID_init(&roll_comp,PID_POSITION,LEG_ROLL_PID,leg_roll_max_out,leg_roll_max_iout);
}

/**
 * @description: �ȳ������ȳ�״̬��ֵ
 * @param {*}
 * @return {*}
 */
void leg_length_state_assign(void)
{
    balance_chassis.chassis_status.chassis_state.leg_length[0] = motion_resolve[0].L0;
    balance_chassis.chassis_status.chassis_state.leg_length[1] = motion_resolve[1].L0;

    leg_length[0].state = balance_chassis.chassis_status.chassis_state.leg_length[0];
    leg_length[1].state = balance_chassis.chassis_status.chassis_state.leg_length[1];
}

/**
 * @brief �ȳ�б��
 * @note  
 */
void leg_slow(float *rec , float target , float slow_Inc)
{
  if(fabs(*rec) - fabs(target) < 0) slow_Inc = slow_Inc;
  if(fabs(*rec) - fabs(target) > 0) slow_Inc = slow_Inc;
  
  if(fabs(*rec - target) < slow_Inc) *rec = target;
  else {
    if((*rec) > target) (*rec) -= slow_Inc;
    if((*rec) < target) (*rec) += slow_Inc;
  }
}

/**
 * @description: ƽ����
 * @param
 * @return
 */
float balance_angle_test = 0.1;
int balance_time_thres = 200;
int balance_time = 0;
void balance_check(void)
{
    if (fabs(lqr.error[PITCH]) <= balance_angle_test)
    {
        balance_time++;
        if (balance_time > balance_time_thres)
				{
					balance_chassis.chassis_flag.balance_flag = BALANCE;
				}
					
    }
    else
    {
        balance_chassis.chassis_flag.balance_flag = UNBALANCE;
        balance_time = 0;
    }
}

/**
 * @description: �ȳ�����PID����
 * @param {*}
 * @return {*}
 */
float left_desire_slow;
float right_desire_slow;
float leg_slow_inc = 0.0006f;
int mode_gogo_flag;
void leg_length_pid_cal(void)
{
    leg_length_state_assign();
		if(rc_ctrl.rc.s[0] == 1)
		{
			balance_chassis.chassis_status.chassis_desire.leg_length[0] = 0.11;
			balance_chassis.chassis_status.chassis_desire.leg_length[1] = 0.11;
			balance_check();
		}
		if(rc_ctrl.rc.s[0] == 2 && balance_chassis.chassis_flag.balance_flag == BALANCE)
		{
			if(rc_ctrl.rc.s[1] == 1 && jump_mode == 0)
			{
				mode_gogo_flag = 1;
				balance_chassis.chassis_status.chassis_desire.leg_length[0] = 0.15;
				balance_chassis.chassis_status.chassis_desire.leg_length[1] = 0.15;
			}
			else if(rc_ctrl.rc.s[1] == 3 && mode_gogo_flag == 1 && jump_mode == 0)
			{
				balance_chassis.chassis_status.chassis_desire.leg_length[0] = 0.22;
				balance_chassis.chassis_status.chassis_desire.leg_length[1] = 0.22;
			}
			else if(rc_ctrl.rc.s[1] == 2 && mode_gogo_flag == 1 && jump_mode == 0)
			{
				balance_chassis.chassis_status.chassis_desire.leg_length[0] = 0.35;
				balance_chassis.chassis_status.chassis_desire.leg_length[1] = 0.35;			
			}
		
		
		}
	
    leg_length[0].desire = balance_chassis.chassis_status.chassis_desire.leg_length[0];
    leg_length[1].desire = balance_chassis.chassis_status.chassis_desire.leg_length[1];

    leg_slow(&left_desire_slow, leg_length[0].desire, leg_slow_inc);
    leg_slow(&right_desire_slow, leg_length[1].desire, leg_slow_inc);
		
		if(balance_chassis.chassis_flag.balance_flag == BALANCE)
		{
			if(ground_detectionL_TEST == 0 && ground_detectionR_TEST == 0)
			{
				leg_length[0].leg_push_force = PID_calc(&leg_length[0].leg_length_pid, leg_length[0].state, left_desire_slow) + PID_calc(&roll_comp,INS_angle[1], 0);
				leg_length[1].leg_push_force = PID_calc(&leg_length[1].leg_length_pid, leg_length[1].state, right_desire_slow) - PID_calc(&roll_comp,INS_angle[1], 0);
			}
			else if(ground_detectionL_TEST == 0 && ground_detectionR_TEST != 0) //�������
			{
				leg_length[0].leg_push_force = PID_calc(&leg_length[0].leg_length_pid, leg_length[0].state, left_desire_slow) + PID_calc(&roll_comp,INS_angle[1], 0);
				leg_length[1].leg_push_force = PID_calc(&leg_length[1].leg_length_pid, leg_length[1].state, right_desire_slow);				
			}
			else if(ground_detectionL_TEST != 0 && ground_detectionR_TEST == 0) //�������
			{
				leg_length[0].leg_push_force = PID_calc(&leg_length[0].leg_length_pid, leg_length[0].state, left_desire_slow);
				leg_length[1].leg_push_force = PID_calc(&leg_length[1].leg_length_pid, leg_length[1].state, right_desire_slow) - PID_calc(&roll_comp,INS_angle[1], 0);				
			}
			else if(start_jump_flag == 1 || (ground_detectionL_TEST == 1 && ground_detectionR_TEST == 1))
			{
				leg_length[0].leg_push_force = PID_calc(&leg_length[0].leg_length_pid, leg_length[0].state, left_desire_slow);
				leg_length[1].leg_push_force = PID_calc(&leg_length[1].leg_length_pid, leg_length[1].state, right_desire_slow);				
			}
		}
		else 
		{
			leg_length[0].leg_push_force = PID_calc(&leg_length[0].leg_length_pid, leg_length[0].state, left_desire_slow);
			leg_length[1].leg_push_force = PID_calc(&leg_length[1].leg_length_pid, leg_length[1].state, right_desire_slow);		
		}
}


float degree_to_rad(float degree)
{
    return degree * 0.0174532925f;
}


/**
 * @description: �ȳ�����roll����
 * @param {*}
 * @return {*}
 */
float balance_roll = 0; // ƽ��ʱroll����[������д��robo_config��Ϊ����]
void roll_pid_cal(void)
{
    float roll_pid_out;
	
		balance_chassis.INS_ANGLE = get_INS_angle_point();
    // roll��pid����
    roll_pid_out = PID_calc(&roll_comp, balance_chassis.INS_ANGLE[2], balance_roll);

    // roll����������������
    leg_length[0].roll_comp = roll_pid_out;
    leg_length[1].roll_comp = -roll_pid_out;
}

/**
 * @description: �ȳ����
 * @param {*}
 * @return {*}
 */
void clear_leg_pid(void)
{
    // ����ȳ�PID
    PID_clear(&leg_length[0].leg_length_pid);
    PID_clear(&leg_length[1].leg_length_pid);
    PID_clear(&roll_comp);
}
