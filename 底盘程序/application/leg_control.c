/*
 * #�����ܿ���
 * 1. lqr���:lqr.pos_out[DRIVING],lqr.pos_out[JOINT]
 * 2. �����ּӼ�yaw��pid������������챵��:chassis_yaw_angle_pid.pos_out ->�Ӽ���ֵ
 * 3. �ؽڵ�����ؼӼ�˫��Э��pid��������VMC��leg_harmo.leg_harmo_out -> �Ӽ���ֵ
 * 4. �ȳ�����������ҵ��������VMC���м��㣺 leg_length[LEFT].leg_pull_force,leg_length[RIGHT].leg_pull_force -> ����ֵ
 */
 
#include "leg_control.h"

leg_control_t leg_control;
float LEG_SPEED_LEFT_CAL[3] = {3200,4,350000};
float LEG_SPEED_RIGHT_CAL[3] = {400,0,10};
pid_type_def leg_speed_left_cal;
pid_type_def leg_speed_right_cal;

float leg_speed_cal_max_out = 1300;
float leg_speed_cal_iout = 300;
//ǿ���ٶ�������pidpid_type_def SL_LEG_speed_right_cal;
float SL_LEG_SPEED_CAL[3] = {0,0,0};
float SL_LEG_speed_cal_max_out = 500;
float SL_LEG_speed_cal_iout = 500;


first_order_filter_type_t L0_ddot_filter;//�ȳ��˲�
int start_jump_flag;//��ʼ������׼

/**
 * @description: �Ȳ������ʼ��
 * @param {*}
 * @return {*}
 */
float SYS_PERIOD = 0.01;
void leg_init(void)
{
    matrix_init();
		matrix_init_R();
    filter_init();
    leg_length_pid_init();
    leg_harmo_pid_init();	
		PID_init(&leg_speed_left_cal,PID_POSITION,LEG_SPEED_LEFT_CAL,leg_speed_cal_max_out,leg_speed_cal_iout);
		first_order_filter_init(&L0_ddot_filter,0.01,&SYS_PERIOD);
	
	
	
//		PID_init(&SL_LEG_speed_right_cal,PID_POSITION,SL_LEG_SPEED_CAL,SL_LEG_speed_cal_max_out,SL_LEG_speed_cal_iout);	
	//�����λ�趨����Ҫ���Ȱᵽ��λ
//		DM8009_Save_position(1);
//		DM8009_Save_position(2);
//		DM8009_Save_position(3);
//		DM8009_Save_position(4);
}


/**
 * @description: ˫��������������
 * @param {*}
 * @return {*}
 */
float g_l_force_k = 30;
void leg_pushforce_cal(void)
{
	leg_length_pid_cal();
  leg_length[0].leg_push_force = leg_length[0].leg_push_force;
  leg_length[1].leg_push_force = leg_length[1].leg_push_force;
}

/**
 * @brief ת��б��
 * @note  
 */
void turn_slow(float *rec , float target , float slow_Inc)
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
 * @description: ת��
 * @param {*}    
 * @return {*}
 */
float WHEEL_ADD;
float SL_ADD;

float fx_add;
float turn_add;
float turn_add_inc = 0.01;

int jump_flag;
void leg_speed_cal(void)
{
	fx_add = fx_add - rc_ctrl.rc.ch[0]/200.0f * 0.002f;
	turn_slow(&turn_add,fx_add,turn_add_inc);
	if(ground_detectionL_TEST == 1 || ground_detectionR_TEST == 1 || start_jump_flag == 1)
	{
		WHEEL_ADD = 0;
	}
	else
	{
		WHEEL_ADD = PID_calc(&leg_speed_left_cal,INS_YawTotalAngle,turn_add);
	}
	//SL_ADD = PID_calc(&SL_LEG_speed_right_cal,lqr.state[3],lqr.desire[3]);
}


float speed_kp;
/**
 * @description: �Ȳ��ܽ����ۺ�
 * @param {*}
 * @return {*}
 */
float leg_feedforward_left;
float leg_feedforward_right;
float leg_feedforward_add_r = 20;
float leg_feedforward_add_l = 20;
float jidihuifu_cont = 200;
float jidihuifu_cont_R = 200;
float jump_count = 50;
int jump_mode;
int leg_f;
void leg_control_cal(void)
{
    // �����ֵ�������
		if(ground_detectionL_TEST == 1)//���������
		{
				leg_control.driving_iq[LEFT] = 0;
		}
		else
		{
			if(jiliflag_l == 2)//�����ʱ�������нǶȱ�׼λ2���˴�������ʱ
			{
				jidihuifu_cont --;
				leg_control.driving_iq[LEFT] = (-lqr.pos_out[DRIVING] * CAN_MAP / TORQUE_CONS);
				if(jidihuifu_cont < 0)
				{
					jidihuifu_cont = 200;
					jiliflag_l = 0;
				}
			}
			else
			{
				leg_control.driving_iq[LEFT] = (-lqr.pos_out[DRIVING] * CAN_MAP / TORQUE_CONS) - WHEEL_ADD;		
			}
		}
		
		if(ground_detectionR_TEST == 1)//�ұ�������
		{
				leg_control.driving_iq[RIGHT] = 0;
		}
		else
		{
				if(jiliflag_R == 2)//�����ʱ�������нǶȱ�׼λ2���˴�������ʱ
				{
					jidihuifu_cont_R--;
					leg_control.driving_iq[RIGHT] = (lqr_R.pos_out[DRIVING_R] * CAN_MAP / TORQUE_CONS);
					if(jidihuifu_cont_R < 0)
					{
						jidihuifu_cont_R = 200;
						jiliflag_R = 0;
					}
				}
				else
				{
					leg_control.driving_iq[RIGHT] = (lqr_R.pos_out[DRIVING_R] * CAN_MAP / TORQUE_CONS) - WHEEL_ADD;
				}

		}
    
		// �ؽڵ���������ؼ���
    leg_control.joint_tq[LEFT] = -lqr.pos_out[JOINT] + leg_harmo.leg_harmo_out ;
    leg_control.joint_tq[RIGHT] = lqr_R.pos_out[JOINT_R] + leg_harmo.leg_harmo_out;
	
    // �ȳ��������ؼ���
		if(rc_ctrl.rc.s[0] == 1 || rc_ctrl.rc.s[0] == 3)
		{
			leg_control.leg_length_tq[LEFT] = leg_length[LEFT].leg_push_force;
			leg_control.leg_length_tq[RIGHT] = leg_length[RIGHT].leg_push_force;
		}
		if(mode_gogo_flag == 1)
		{
			if(rc_ctrl.rc.s[1] == 1 && rc_ctrl.rc.ch[4] < -600) //������Ծģʽ 0.25m���ȳ�
			{
				jump_mode = 1;//��Ծģʽ��һ�׶�
				jump_count = 65;//��һ�׶���ʱ
			}
			else if(rc_ctrl.rc.s[1] != 1)
			{
				jump_mode = 0;//�ر���Ծģʽ
			}
			//��Ծģʽ��һ�׶Σ���ʱ��һ������������ά�� 30*0.002 s
			if(jump_mode == 1)
			{
				if(rc_ctrl.rc.ch[4] > 80)
				{
					leg_f = 1;
				}
				if(leg_f == 1 && jump_mode == 1)
				{
					start_jump_flag = 1;
					if(jump_count-- < 0)
					{
						leg_f = 0;
						jump_mode = 2;//��Ծģʽ�ڶ��׶Ρ�
						jump_count = 40;//�ڶ��׶���ʱ
					}
					leg_control.leg_length_tq[LEFT] = 70;
					leg_control.leg_length_tq[RIGHT] = 70;
					if(balance_chassis.chassis_status.chassis_state.leg_length[0] >= 0.3)
					{
						leg_control.leg_length_tq[LEFT] = 40;
					}
					if(balance_chassis.chassis_status.chassis_state.leg_length[1] >= 0.3)
					{
						leg_control.leg_length_tq[RIGHT] = 40;
					}
					if(balance_chassis.chassis_status.chassis_state.leg_length[0] >= 0.35)
					{
						leg_control.leg_length_tq[LEFT] = 20;
					}
					if(balance_chassis.chassis_status.chassis_state.leg_length[1] >= 0.35)
					{
						leg_control.leg_length_tq[RIGHT] = 20;
					}
				}			
			}
			//��Ծģʽ�ڶ��׶Σ�����
			if(jump_mode == 2)
			{
				leg_feedforward_add_l = 0;
				leg_feedforward_add_r = 0;
				
				
//				balance_chassis.chassis_status.chassis_desire.leg_length[0] = 0.15;
//				balance_chassis.chassis_status.chassis_desire.leg_length[1] = 0.15;		
//				left_desire_slow = 0.15;
//				right_desire_slow = 0.15;
				leg_control.leg_length_tq[LEFT] = -40;
				leg_control.leg_length_tq[RIGHT] = -40;
//				leg_control.driving_iq[LEFT] = 0;
//				leg_control.driving_iq[RIGHT] = 0;
				
				if(jump_count-- < 0)
				{
					jump_mode = 3;//��Ծģʽ�����׶Ρ�
					jump_count = 200;
				}
			}
			//��Ծģʽ�����׶Σ����Ȼ���
			if(jump_mode == 3)
			{
				//�˴���������
				leg_feedforward_add_l = 2;
				leg_feedforward_add_r = 2;
//				balance_chassis.chassis_status.chassis_desire.leg_length[0] = 0.2;
//				balance_chassis.chassis_status.chassis_desire.leg_length[1] = 0.2;		
				leg_control.leg_length_tq[LEFT] = leg_length[LEFT].leg_push_force + leg_feedforward_add_l;
				leg_control.leg_length_tq[RIGHT] = leg_length[RIGHT].leg_push_force + leg_feedforward_add_r;
//				leg_control.driving_iq[LEFT] = 0;
//				leg_control.driving_iq[RIGHT] = 0;
				
				if(jump_count-- < 0)
				{
					jump_mode = 4;//��Ծģʽ�����׶Ρ�
					jump_count = 20;
				}
			}
			//��Ծģʽ���Ľ׶�,��Ծ���
			if(jump_mode  == 4)
			{
				//�˴���������
				leg_control.leg_length_tq[LEFT] = leg_length[LEFT].leg_push_force + leg_feedforward_add_l;
				leg_control.leg_length_tq[RIGHT] = leg_length[RIGHT].leg_push_force + leg_feedforward_add_r;
				if(jump_count-- < 0)
				{
					start_jump_flag = 0;
					jump_mode = 0;      //��Ծģʽ��׼λ�ָ�
				}
			}
			
			//vmc�ȳ�����
			if(start_jump_flag == 0)
			{
				leg_control.leg_length_tq[LEFT] = leg_length[LEFT].leg_push_force + leg_feedforward_add_l;
				leg_control.leg_length_tq[RIGHT] = leg_length[RIGHT].leg_push_force + leg_feedforward_add_r;
			}
		}

}

/**
 * @description: �Ȳ�ǰ����ֵ
 * @param {*}
 * @return {*}
 */
void leg_feedforward(leg_control_t *legcontrol, float left, float right)
{
    legcontrol->leg_feedforward[LEFT] = left;
    legcontrol->leg_feedforward[RIGHT] = right;
}

/**
 * @description: �Ȳ��������
 * @param {*}
 * @return {*}
 */
int lqr_cal_flag_1;
int lqr_cal_flag_r;
void leg_cal(void)
{
    VMC_calculate();
		leg_pushforce_cal();
		leg_harmo_pid_cal(motion_resolve[0].phi0, motion_resolve[1].phi0);
    lqr_cal_flag_1 = matrix_calculate();                                        // lqr����
		lqr_cal_flag_r = matrix_calculate_R(); 
    leg_control_cal();                                                        // �Ȳ������ۺϼ���
    joint_torque_calculate();                                                 // �ۺ��ſɱȼ������չؽڵ���������

}



