#include "lqr.h"

lqr_t lqr;
float p[48] = {
-469.1998, 607.4464, -332.0132, -1.3418, -96.0956, 94.8588, -55.5511, -0.4511, -33.5688, 26.9375, -6.7618, -3.5473,
-141.4624, 110.7227, -27.6676, -15.1246, -133.3729, 158.7707, -81.9658, 26.8152, -4.3046, 23.6872, -18.2807, 5.7499,
156.7672, -189.7718, 73.0304, 2.5541, 20.5274, -26.5921, 12.4237, 0.5401, -9.3911, 13.1906, -7.7716, 2.6471,
-39.5526, 54.4612, -31.5502, 10.8111, 110.6629, -114.3837, 45.0895, 39.5027, 21.1213, -24.2357, 10.2646, 2.2381
};
float L0[4];
float k[12];
float k_cal[12];
float k_copy[12];
float k_para[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
float torque[2]; //[驱动轮输出力矩/髋关节输出力矩]
float jacobi_left[4];
float inv_jacobi_left[4];
float jacobi_right[4];
float inv_jacobi_right[4];
float k_noleg[12] = {-24.1120, -4.2577, -13.0094, -12.3912, 13.3169, 2.7918, 4.1299, 0.8031, 3.9801, 3.2559, 43.2680, 3.6232};

//离地检测标准位
uint8_t ground_detectionL_TEST;

arm_matrix_instance_f32 ptr_error;
arm_matrix_instance_f32 ptr_torque;

// 加入腿部长度变量后的参数定义
arm_matrix_instance_f32 ptr_p;
arm_matrix_instance_f32 ptr_L0;
arm_matrix_instance_f32 ptr_k;
arm_matrix_instance_f32 ptr_k_cal;
arm_matrix_instance_f32 ptr_jacobi_left;
arm_matrix_instance_f32 ptr_inv_jacobi_left;
arm_matrix_instance_f32 ptr_jacobi_right;
arm_matrix_instance_f32 ptr_inv_jacobi_right;

//(9.81f - INS_accel[2]/arm_cos_f32(lqr.state[PITCH]))

//离地检测
uint8_t ground_detectionL(motion_resolve_t *vmc,leg_control_t *LEG_F)
{
	lqr.theta_dot = lqr.state[POSE_SPEED];
	lqr.theta_ddot = 0;
	vmc->FN= LEG_F->leg_length_tq[0]*arm_cos_f32(lqr.state[POSE_ANGLE]) + LEG_F->joint_tq[0]*arm_sin_f32(lqr.state[POSE_ANGLE])/vmc->L0 + 15.0f;
//	+ 1.5f*(
//	- vmc->L0_ddot*arm_cos_f32(lqr.state[POSE_ANGLE]) 
//	+ 2.0f*vmc->L0_dot*lqr.state[POSE_SPEED]*arm_sin_f32(lqr.state[POSE_ANGLE]) 
//	+ vmc->L0*lqr.theta_ddot*arm_sin_f32(lqr.state[POSE_ANGLE]) 
//  + vmc->L0*lqr.theta_dot*lqr.theta_dot*arm_cos_f32(lqr.state[POSE_ANGLE]));
	
	if(vmc->FN<18.0f)
	{//离地了
			return 1;
	}
	else
	{
			return 0;
	}
}

/**
 * @brief 遥控器控制值斜坡
 * @note  
 */
void rc_slow(float *rec , float target , float slow_Inc)
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
 * @brief 前馈控制值斜坡
 * @note  
 */
void QIANKUI_slow(float *rec , float target , float slow_Inc)
{
  if(fabs(*rec) - fabs(target) < 0) slow_Inc = slow_Inc;
  if(fabs(*rec) - fabs(target) > 0) slow_Inc = slow_Inc;
  
  if(fabs(*rec - target) < slow_Inc) *rec = target;
  else {
    if((*rec) > target) (*rec) -= slow_Inc;
    if((*rec) < target) (*rec) += slow_Inc;
  }
}

void matrix_init(void)
{
    ptr_error.numCols = 1;
    ptr_error.numRows = 6;
    ptr_error.pData = lqr.error;

    ptr_torque.numCols = 1;
    ptr_torque.numRows = 2;
    ptr_torque.pData = torque;

    ptr_p.numCols = 4;
    ptr_p.numRows = 12;
    ptr_p.pData = p;

    ptr_L0.numCols = 1;
    ptr_L0.numRows = 4;
    ptr_L0.pData = L0;

    ptr_k.numCols = 1;
    ptr_k.numRows = 12;
    ptr_k.pData = k;

    ptr_k_cal.numCols = 6;
    ptr_k_cal.numRows = 2;
    ptr_k_cal.pData = k_cal;

    ptr_jacobi_left.numCols = 2;
    ptr_jacobi_left.numRows = 2;
    ptr_jacobi_left.pData = jacobi_left;

    ptr_inv_jacobi_left.numCols = 2;
    ptr_inv_jacobi_left.numRows = 2;
    ptr_inv_jacobi_left.pData = inv_jacobi_left;

    ptr_jacobi_right.numCols = 2;
    ptr_jacobi_right.numRows = 2;
    ptr_jacobi_right.pData = jacobi_right;

    ptr_inv_jacobi_right.numCols = 2;
    ptr_inv_jacobi_right.numRows = 2;
    ptr_inv_jacobi_right.pData = inv_jacobi_right;
}


/**
 * @description: 腿长拟合k值
 * @param {}
 * @return {*}
 */
int jiliflag_l;
void match_k(void)
{
    float L;

    L = motion_resolve[0].L0 ;

    // 更新腿长矩阵
    L0[0] = L * L * L;
    L0[1] = L * L;
    L0[2] = L;
    L0[3] = 1;

    // 计算当前腿长下的k值
    arm_mat_mult_f32(&ptr_p, &ptr_L0, &ptr_k);

    for (int i = 0; i <= 11; i++)
    {
        k_cal[i] = k[i];  // 矩阵大小转换
        k_copy[i] = k[i]; // 矩阵复制
        k_cal[i] *= k_para[i];
    }
    if( (ground_detectionL(&motion_resolve[0],&leg_control) == 1 && mode_gogo_flag == 1 ) || jump_mode == 3)
		{
			ground_detectionL_TEST = 1;
			k_cal[0] = 0;k_cal[1] = 0;k_cal[2] = 0;k_cal[3] = 0;k_cal[4] = 0;k_cal[5] = 0;
													      k_cal[8] = 0;k_cal[9] = 0;k_cal[10] = 0;k_cal[11] = 0;
//			leg_length[0].leg_length_pid.Kp = 400;	
//			leg_length[0].leg_length_pid.Kd = 10;
			if(jump_mode != 2)
			{
				leg_feedforward_add_l = 0;
			}
			jiliflag_l = 1;//落地延时开启航行角度标准位1
		}
		else
		{
			ground_detectionL_TEST = 0;
			//QIANKUI_slow(&leg_feedforward_add_l,18,1);
			leg_feedforward_add_l = 20;

			if(jiliflag_l == 1) 
			{
				jiliflag_l = 2;//落地延时开启航行角度标准位2
			}
		}
}


float Rz = 0.1; 
float Qz = 0.0001;
/*
 * 速度估计
 *
 * 输入：加速度 acc
 * 			 速度 speed
 * 输出：估计速度 vel_esti		 
 */
void speed_est(const float dt, const float acc, const float speed, float *vel_esti)
{
//	static float Rz = 1;
//	static float Qz = 1;
	static float _x = 0;
	static float _Pz = 1;
    
 	/* 卡尔曼滤波矩阵定义  */
	float H;
	float A;
	float B = dt;
	float _x_est;
	float _p_est;
	float K;
	float x_;
	
	A = 1;
	B = dt;
	H = 1;
/**********************************************************/
	// 预测阶段
	_x_est = A*_x + B*acc;
	_p_est = _Pz + Qz; // A*_P*A_T + Q
	// 更新阶段
	K = _p_est / (_p_est + Rz);
	_x = _x_est + K * (speed - _x_est);
	_Pz = (1 - K) * _p_est;   
    
	*vel_esti = _x;
}
/**
 * @description: 数据融合
 * @param {}
 * @return {*}
 */
float wr,wl=0.0f;
float vrb,vlb=0.0f;
float aver_v=0.0f;
float aver_c_test=0.0f;

float wr_TEST,wl_TEST=0.0f;

float data_fusion(void)
{
//		if(balance_chassis.chassis_flag.balance_flag == UNBALANCE)
//		{
//				lqr_speed = (driving_motor[LEFT].speed - driving_motor[RIGHT].speed) / 2.0f;
//				return lqr_speed;
//		}
//		else
//		{

	
				wl = -driving_motor[LEFT].speed*10.0f + lqr.state[POSE_SPEED];
				wr = driving_motor[RIGHT].speed*10.0f + lqr_R.state[POSE_SPEED];
	
				vlb=wl*0.1f  +  motion_resolve[0].L0*lqr.state[POSE_SPEED]*arm_cos_f32(lqr.state[POSE_ANGLE])  +  motion_resolve[0].L0_dot*arm_sin_f32(lqr.state[POSE_ANGLE]);//机体b系的速度
				vrb=wr*0.1f  +  motion_resolve[1].L0*lqr.state[POSE_SPEED]*arm_cos_f32(lqr.state[POSE_ANGLE])  +  motion_resolve[1].L0_dot*arm_sin_f32(lqr.state[POSE_ANGLE]);//机体b系的速度

				aver_c_test = (vlb + vrb)/2.0f;

////				xvEstimateKF_Update(&vaEstimateKF,INS_accel[1],aver_v);
//							xvEstimateKF_Update(&vaEstimateKF,0.02,aver_v);
			speed_est(0.002,INS_accel[1],aver_c_test,&lqr_speed);
			return lqr_speed;
//		}
}

/**
 * @description: 矩阵状态量赋值
 * @param {}
 * @return {*}
 */
pose_t pose;

void matrix_state_assign(void)
{
    pose.pose = motion_resolve[0].phi0;
    pose.Dpose = phi0_dot_cal(&motion_resolve[0]);

		lqr.state[SPEED] = data_fusion();
    lqr.state[DISPLAYCEMENT] += lqr.state[SPEED] * 0.002f; // 积分
	
    lqr.state[PITCH] = -INS_angle[2];

    lqr.state[PITCH_SPEED] = -INS_gyro[0];
    lqr.state[POSE_ANGLE] = 1.57f - (pose.pose + lqr.state[PITCH]); // 角度(弧度制) 向后时角度大于1.57°
    lqr.state[POSE_SPEED] = -pose.Dpose - lqr.state[PITCH_SPEED];
}

/**
 * @description: 矩阵期望赋值
 * @param {}
 * @return {*}
 */
float balance_angle = 0.0f;
float theta_angle = 0.0f;
float disp_para = 0.0f;
char first_desire;
float rc_control_speed;
float inc_speed = 0.005;
void matrix_desire_assign(void)
{
    lqr.desire[POSE_ANGLE] = theta_angle;
    lqr.desire[POSE_SPEED] = 0.0f;
		rc_slow(&rc_control_speed , rc_ctrl.rc.ch[3]/360.0f , inc_speed);
		lqr.desire[SPEED] = rc_control_speed;
		if(lqr.desire[SPEED] == 0 && first_desire == 0)
		{
			first_desire = 1;
			lqr.desire[DISPLAYCEMENT] = lqr.state[DISPLAYCEMENT];
		}
		if(lqr.desire[SPEED] != 0)
		{
			first_desire = 0;
//			lqr.desire[DISPLAYCEMENT] += lqr.desire[SPEED] * 0.001;	
			lqr.desire[DISPLAYCEMENT] = lqr.state[DISPLAYCEMENT];	
		}	
    lqr.desire[PITCH] = balance_angle;
    lqr.desire[PITCH_SPEED] = 0.0f;
}

/**
 * @description: 矩阵误差值计算赋值
 * @param {}
 * @return {*}
 */
void matrix_error_calculate(void)
{
    matrix_desire_assign();
    matrix_state_assign();
    match_k();
    for (int i = 0; i <= 5; i++)
    {
        lqr.error[i] = lqr.desire[i] - lqr.state[i];
    }
    
    
}

/**
 * @description: 矩阵运算
 * @param {}
 * @return {标志位}
 */
int matrix_judge;
int matrix_calculate(void)
{
    matrix_error_calculate();

    arm_mat_mult_f32(&ptr_k_cal, &ptr_error, &ptr_torque);

    lqr.pos_out[DRIVING] = torque[DRIVING];
    lqr.pos_out[JOINT] = torque[JOINT];
    return matrix_judge;
}
