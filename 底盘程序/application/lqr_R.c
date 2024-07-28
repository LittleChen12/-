#include "lqr_R.h"

lqr_t_R lqr_R;
float p_R[48] = {
-405.1169, 532.8363, -292.2913, -1.6956, -88.4449, 86.4128, -48.9164, -0.4822, -21.5757, 17.0745, -4.1333, -2.9613,
-98.0403, 75.3264, -18.0962, -13.4830, -133.0322, 153.0214, -75.2435, 23.2699, -7.6141, 24.1039, -17.0331, 5.1693,
92.5780, -111.9728, 42.7676, 1.7882, 11.9946, -15.4469, 7.1691, 0.3793, -6.7667, 8.6562, -4.6866, 1.5014,
-30.4936, 38.4432, -20.4955, 6.5846, 71.6652, -73.2545, 28.3532, 35.3347, 14.0389, -15.7916, 6.5518, 2.5940
};
float L0_R[4];
float k_R[12];
float k_cal_R[12];
float k_copy_R[12];
float k_para_R[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
float torque_R[2]; //[驱动轮输出力矩/髋关节输出力矩]
float jacobi_left_R[4];
float inv_jacobi_left_R[4];
float jacobi_right_R[4];
float inv_jacobi_right_R[4];
float k_noleg_R[12] = {-24.1120, -4.2577, -13.0094, -12.3912, 13.3169, 2.7918, 4.1299, 0.8031, 3.9801, 3.2559, 43.2680, 3.6232};

arm_matrix_instance_f32 ptr_error_R;
arm_matrix_instance_f32 ptr_torque_R;

// 加入腿部长度变量后的参数定义
arm_matrix_instance_f32 ptr_p_R;
arm_matrix_instance_f32 ptr_L0_R;
arm_matrix_instance_f32 ptr_k_R;
arm_matrix_instance_f32 ptr_k_cal_R;
arm_matrix_instance_f32 ptr_jacobi_left_R;
arm_matrix_instance_f32 ptr_inv_jacobi_left_R;
arm_matrix_instance_f32 ptr_jacobi_right_R;
arm_matrix_instance_f32 ptr_inv_jacobi_right_R;


uint8_t ground_detectionR_TEST;

//(9.81f - INS_accel[2]/arm_cos_f32(lqr_R.state[PITCH])) 
uint8_t ground_detectionR(motion_resolve_t *vmc,leg_control_t *LEG_F)
{
	lqr_R.theta_dot = lqr_R.state[POSE_SPEED];
	lqr_R.theta_ddot = 0;
	
	vmc->FN= LEG_F->leg_length_tq[1]*arm_cos_f32(lqr_R.state[POSE_ANGLE]) + LEG_F->joint_tq[1]*arm_sin_f32(lqr_R.state[POSE_ANGLE])/vmc->L0 + 15.0f;
//	+ 1.5f*(
//	- vmc->L0_ddot*arm_cos_f32(lqr_R.state[POSE_ANGLE]) 
//	+ 2.0f*vmc->L0_dot*lqr_R.state[POSE_SPEED]*arm_sin_f32(lqr_R.state[POSE_ANGLE]) 
//	+ vmc->L0*lqr_R.theta_ddot*arm_sin_f32(lqr_R.state[POSE_ANGLE])
//  + vmc->L0*lqr_R.theta_dot*lqr_R.theta_dot*arm_cos_f32(lqr_R.state[POSE_ANGLE]));
	
	
	if(vmc->FN<18.0f)
	{//离地了
			return 1;
	}
	else
	{
			return 0;
	}
}

void matrix_init_R(void)
{
    ptr_error_R.numCols = 1;
    ptr_error_R.numRows = 6;
    ptr_error_R.pData = lqr_R.error;

    ptr_torque_R.numCols = 1;
    ptr_torque_R.numRows = 2;
    ptr_torque_R.pData = torque_R;


    ptr_p_R.numCols = 4;
    ptr_p_R.numRows = 12;
    ptr_p_R.pData = p_R;

    ptr_L0_R.numCols = 1;
    ptr_L0_R.numRows = 4;
    ptr_L0_R.pData = L0_R;

    ptr_k_R.numCols = 1;
    ptr_k_R.numRows = 12;
    ptr_k_R.pData = k_R;

    ptr_k_cal_R.numCols = 6;
    ptr_k_cal_R.numRows = 2;
    ptr_k_cal_R.pData = k_cal_R;

    ptr_jacobi_left_R.numCols = 2;
    ptr_jacobi_left_R.numRows = 2;
    ptr_jacobi_left_R.pData = jacobi_left_R;

    ptr_inv_jacobi_left_R.numCols = 2;
    ptr_inv_jacobi_left_R.numRows = 2;
    ptr_inv_jacobi_left_R.pData = inv_jacobi_left_R;

    ptr_jacobi_right_R.numCols = 2;
    ptr_jacobi_right_R.numRows = 2;
    ptr_jacobi_right_R.pData = jacobi_right_R;

    ptr_inv_jacobi_right_R.numCols = 2;
    ptr_inv_jacobi_right_R.numRows = 2;
    ptr_inv_jacobi_right_R.pData = inv_jacobi_right_R;
}


/**
 * @description: 腿长拟合k值
 * @param {}
 * @return {*}
 */
int jiliflag_R;
void match_k_R(void)
{
    float L;

    L = motion_resolve[1].L0 ;

    // 更新腿长矩阵
    L0_R[0] = L * L * L;
    L0_R[1] = L * L;
    L0_R[2] = L;
    L0_R[3] = 1;

    // 计算当前腿长下的k值
    arm_mat_mult_f32(&ptr_p_R, &ptr_L0_R, &ptr_k_R);

    for (int i = 0; i <= 11; i++)
    {
        k_cal_R[i] = k_R[i];  // 矩阵大小转换
        k_copy_R[i] = k_R[i]; // 矩阵复制
        k_cal_R[i] *= k_para_R[i];
    }
    if((ground_detectionR(&motion_resolve[1],&leg_control) == 1 && mode_gogo_flag == 1) || jump_mode == 3)
		{
			ground_detectionR_TEST = 1;
			k_cal_R[0] = 0;k_cal_R[1] = 0;k_cal_R[2] = 0;k_cal_R[3] = 0;k_cal_R[4] = 0;k_cal_R[5] = 0;
													          k_cal_R[8] = 0;k_cal_R[9] = 0;k_cal_R[10] = 0;k_cal_R[11] = 0;
			
			

			if(jump_mode != 2)
			{
				leg_feedforward_add_r = 0;
			}
			jiliflag_R = 1;
		}
		else
		{
			ground_detectionR_TEST = 0;
			//QIANKUI_slow(&leg_feedforward_add_r,18,1);
			leg_feedforward_add_r = 20;

			if(jiliflag_R == 1)
			{
				jiliflag_R = 2;
			}
		}
}


/**
 * @description: 矩阵状态量赋值
 * @param {}
 * @return {*}
 */
pose_t_R pose_R;
float lqr_speed;
void matrix_state_assign_R(void)
{
    pose_R.pose = 3.14159f - motion_resolve[1].phi0;
    pose_R.Dpose = -phi0_dot_cal_R(&motion_resolve[1]);

		lqr_R.state[SPEED] = lqr_speed;
    lqr_R.state[DISPLAYCEMENT] += lqr_R.state[SPEED] * 0.002f; // 积分
	
    lqr_R.state[PITCH] = -INS_angle[2];

    lqr_R.state[PITCH_SPEED] = -INS_gyro[0];
    lqr_R.state[POSE_ANGLE] = 1.57f - (pose_R.pose + lqr_R.state[PITCH]); // 角度(弧度制) 向后时角度大于1.57°
    lqr_R.state[POSE_SPEED] = -pose_R.Dpose - lqr_R.state[PITCH_SPEED];
}

/**
 * @description: 矩阵期望赋值
 * @param {}
 * @return {*}
 */
float balance_angle_R = 0.0f;
float theta_angle_R = 0.0f;
float disp_para_R = 0.0f;
char first_desire_R;
void matrix_desire_assign_R(void)
{
    lqr_R.desire[POSE_ANGLE] = theta_angle_R;
    lqr_R.desire[POSE_SPEED] = 0.0f;
		if(lqr_R.desire[SPEED] == 0 && first_desire_R == 0)
		{
			first_desire_R = 1;
			lqr_R.desire[DISPLAYCEMENT] = lqr_R.state[DISPLAYCEMENT];
		}
		if(lqr_R.desire[SPEED] != 0)
		{
			first_desire_R = 0;
//			lqr_R.desire[DISPLAYCEMENT] += lqr_R.desire[SPEED] * 0.001;	
			lqr_R.desire[DISPLAYCEMENT] = lqr_R.state[DISPLAYCEMENT];	
		}
		lqr_R.desire[SPEED] = rc_control_speed;		
    lqr_R.desire[PITCH] = balance_angle_R;
    lqr_R.desire[PITCH_SPEED] = 0.0f;
}

/**
 * @description: 矩阵误差值计算赋值
 * @param {}
 * @return {*}
 */
void matrix_error_calculate_R(void)
{
    matrix_desire_assign_R();
    matrix_state_assign_R();
    match_k_R();
    
    for (int i = 0; i <= 5; i++)
    {
        lqr_R.error[i] = lqr_R.desire[i] - lqr_R.state[i];
    }
    
    
}

/**
 * @description: 矩阵运算
 * @param {}
 * @return {标志位}
 */
int matrix_judge_R;
int matrix_calculate_R(void)
{
    matrix_error_calculate_R();

    arm_mat_mult_f32(&ptr_k_cal_R, &ptr_error_R, &ptr_torque_R);

    lqr_R.pos_out[DRIVING_R] = torque_R[DRIVING_R];
    lqr_R.pos_out[JOINT_R] = torque_R[JOINT_R];

    return matrix_judge_R;
}
