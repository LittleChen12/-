#include "VMC.h"


/*
    1.车体参数：腿长 L1,L2
    2.电机参数：髋关节电机 phi1，phi4
*/

motion_resolve_t motion_resolve[2]; //[LEFT][RIGHT]

float mid_point[2] = {0.0f, 0.0f};

/**
 * @description: 运动解算获得C点坐标
 * @param {髋关节电机}
 * @return {Point phi2 phi3}
 */
float xd, yd, xb, yb;
float A0, B0, C0;
void Motion_calculate(motion_resolve_t *motion, float phi1, float phi4) // phi1是钝角 phi4是锐角
{

    motion->phi1 = phi1;
    motion->phi4 = phi4;

    xd = 0.5f * L3 + L1 * arm_cos_f32(phi4);
    yd = L1 * arm_sin_f32(phi4);
    xb = -0.5f * L3 + L1 * arm_cos_f32(phi1);
    yb = L1 * arm_sin_f32(phi1);

    A0 = 2 * L2 * (xd - xb);
    B0 = 2 * L2 * (yd - yb);
    C0 = (xd - xb) * (xd - xb) + (yd - yb) * (yd - yb);

    motion->phi2 = 2 * atan((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)) / (A0 + C0));
    motion->point[X] = xb + L2 * arm_cos_f32((motion->phi2));
    motion->point[Y] = yb + L2 * arm_sin_f32((motion->phi2));

    motion->phi3 = atan((motion->point[Y] - yd) / (motion->point[X] - xd));
}


/**
 * @description: 雅可比矩阵计算
 * @param {髋关节电机}
 * @return {J1 J2 J3 J4}
 */
void Jacobi_calculate(motion_resolve_t *motion)
{
    motion->jacobi.J1 = L1 * arm_sin_f32(motion->phi3) * arm_sin_f32(motion->phi1 - motion->phi2) / arm_sin_f32(motion->phi2 - motion->phi3);
    motion->jacobi.J2 = L1 * arm_sin_f32(motion->phi2) * arm_sin_f32(motion->phi3 - motion->phi4) / arm_sin_f32(motion->phi2 - motion->phi3);
    motion->jacobi.J3 = -L1 * arm_cos_f32(motion->phi3) * arm_sin_f32(motion->phi1 - motion->phi2) / arm_sin_f32(motion->phi2 - motion->phi3);
    motion->jacobi.J4 = -L1 * arm_cos_f32(motion->phi2) * arm_sin_f32(motion->phi3 - motion->phi4) / arm_sin_f32(motion->phi2 - motion->phi3);
	
}

/**
 * @description: 雅可比矩阵计算
 * @param {髋关节电机}
 * @return {J1 J2 J3 J4}
 */
void Jacobi_calculate_2(motion_resolve_t *motion)
{
    motion->jacobi.J1 = L1 * arm_sin_f32(motion->phi0 - motion->phi3) * arm_sin_f32(motion->phi1 - motion->phi2) / arm_sin_f32(motion->phi3 - motion->phi2);
    motion->jacobi.J2 = L1 * arm_cos_f32(motion->phi0 - motion->phi3) * arm_sin_f32(motion->phi1 - motion->phi2) / (arm_sin_f32(motion->phi3 - motion->phi2) * motion_resolve->L0);
    motion->jacobi.J3 = L1 * arm_sin_f32(motion->phi0 - motion->phi2) * arm_sin_f32(motion->phi3 - motion->phi4) / arm_sin_f32(motion->phi3 - motion->phi2);
    motion->jacobi.J4 = L1 * arm_cos_f32(motion->phi0 - motion->phi2) * arm_sin_f32(motion->phi3 - motion->phi4) / (arm_sin_f32(motion->phi3 - motion->phi2) * motion_resolve->L0);
}


/**
 * @description: L0与姿态角解算
 * @param {髋关节电机}
 * @return {L0 phi0}
 */
int ddL0_count = 5;
void leg_para_calculate(motion_resolve_t *motion)
{
    float distance, edge, angle;
    distance = ((motion->point[X] - mid_point[X]) * (motion->point[X] - mid_point[X])) + ((motion->point[Y] - mid_point[Y]) * (motion->point[Y] - mid_point[Y]));
    arm_sqrt_f32(distance, &distance);
	  motion->L0 = distance;
//		motion->L0_dot = (motion->L0 - motion->last_L0)/0.002f;//腿长L0的一阶导数
//		motion->last_L0 = motion->L0;
	
		if(--(motion->dd_L0_count) == -1)
		{
			motion->dd_L0_count = 4;
			motion->L0_dot = (motion->L0 - motion->last_L0)/0.01f;//腿长L0的一阶导数
			motion->last_L0 = motion->L0;
//			first_order_filter_cali(&L0_ddot_filter,motion->L0_dot);
//			motion->L0_ddot = (motion->L0_dot - motion->last_L0_dot)/0.01f;//腿长L0的二阶导数
			motion->L0_ddot = 0;
			motion->last_L0_dot = motion->L0_dot;
		}

    edge = ((motion->point[X] * motion->point[X]) + (motion->point[Y] * motion->point[Y]));
    arm_sqrt_f32(edge, &edge);
    angle = acos(motion->point[X] / edge);
    motion->phi0 = angle;
		//(暂时屏蔽)
//    motion->theta = 1.57f - (motion->phi0 - lqr.state[PITCH]);    ****
}

/**
 * @description: VMC状态量计算
 * @param {void}
 * @return {*}
 */
void VMC_calculate(void)
{
		// 物理量解算3.1415926f+DM8009_READ_data_3[3]+0.33581f
		Motion_calculate(&motion_resolve[0],3.1415926f+DM8009_READ_data_3[3]+0.38081f,DM8009_READ_data_4[3]-0.33581f);			
		Motion_calculate(&motion_resolve[1],3.1415926f+DM8009_READ_data_2[3]+0.34581f,DM8009_READ_data_1[3]-0.33581f);
		// 雅可比矩阵求解
		Jacobi_calculate(&motion_resolve[0]);
		Jacobi_calculate(&motion_resolve[1]);
		// 雅可比逆矩阵求解			
		jacobi_left[0] = motion_resolve[0].jacobi.J1;
		jacobi_left[1] = motion_resolve[0].jacobi.J2;
		jacobi_left[2] = motion_resolve[0].jacobi.J3;
		jacobi_left[3] = motion_resolve[0].jacobi.J4;

		jacobi_right[0] = motion_resolve[1].jacobi.J1;
		jacobi_right[1] = motion_resolve[1].jacobi.J2;
		jacobi_right[2] = motion_resolve[1].jacobi.J3;
		jacobi_right[3] = motion_resolve[1].jacobi.J4;

//		arm_mat_inverse_f32(&ptr_jacobi_left, &ptr_inv_jacobi_left);			
//		arm_mat_inverse_f32(&ptr_jacobi_right, &ptr_inv_jacobi_right);//矩阵求逆

		// L0与姿态角解算			
		leg_para_calculate(&motion_resolve[0]);			
		leg_para_calculate(&motion_resolve[1]);
		
		balance_chassis.L0_avg = (motion_resolve[0].L0 + motion_resolve[1].L0) / 2.0f; // 平均腿长计算 
}

/**
 * @description: 力矩计算
 * @param {void}
 * @return {*}
 */
/*
    LEFT-0 顺时针扭矩从负趋于0     扭矩为负数   绝对值大扭矩大 -> 正扭矩逆时针
    LEFT-1 逆时针扭矩从正逐渐增大  扭矩为正数   绝对值大扭矩大 -> 负扭矩顺时针
*/
void joint_torque_calculate(void)
{
		Jacobi_calculate_2(&motion_resolve[LEFT]);
    motion_resolve[LEFT].torque[FRONT] = motion_resolve[LEFT].jacobi.J2 * leg_control.joint_tq[LEFT] + motion_resolve[LEFT].jacobi.J1 * leg_control.leg_length_tq[LEFT];
    motion_resolve[LEFT].torque[BACK] = motion_resolve[LEFT].jacobi.J4 * leg_control.joint_tq[LEFT] + motion_resolve[LEFT].jacobi.J3 * leg_control.leg_length_tq[LEFT];

		Jacobi_calculate_2(&motion_resolve[RIGHT]);
    motion_resolve[RIGHT].torque[FRONT] = motion_resolve[RIGHT].jacobi.J2 * leg_control.joint_tq[RIGHT] + motion_resolve[RIGHT].jacobi.J1 * leg_control.leg_length_tq[RIGHT];
    motion_resolve[RIGHT].torque[BACK] = motion_resolve[RIGHT].jacobi.J4 * leg_control.joint_tq[RIGHT] + motion_resolve[RIGHT].jacobi.J3 * leg_control.leg_length_tq[RIGHT];

}

float phi0_dot;
float phi0_dot_cal(motion_resolve_t *motion)
{
    filter_cal(); 
    float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    float sq1, sq2;
    float phi1_dot, phi4_dot;
    float phi1, phi4;

    phi1 = motion->phi1;
    phi4 = motion->phi4;
    phi1_dot = joint_w_filter[1].out;
    phi4_dot = joint_w_filter[0].out;
    
    t12 = L1 * arm_sin_f32(phi1) - L1 * arm_sin_f32(phi4);
    t11 = L3 - L1 * arm_cos_f32(phi1) + L1 * arm_cos_f32(phi4);
    t10 = 4 * L2 * L2 * t12 * t12 - ((t12 * t12 + t11 * t11) * (t12 * t12 + t11 * t11)) + 4 * L2 * L2 * t11 * t11;
    arm_sqrt_f32(t10, &t10);
    t9 = t12 * t12 + t11 * t11 + 2 * L2 * t11;
    t8 = 2 * L2 * t12 - t10;
    t7 = L1 * arm_sin_f32(phi1) * phi1_dot - L1 * arm_sin_f32(phi4) * phi4_dot;
    t6 = L1 * arm_cos_f32(phi1) * phi1_dot - L1 * arm_cos_f32(phi4) * phi4_dot;
    t5 = L2 * arm_sin_f32(2 * atan(t8 / t9)) - L1 * arm_sin_f32(phi1);
    t4 = L2 * arm_cos_f32(2 * atan(t8 / t9)) - L3 / 2 + L1 * arm_cos_f32(phi1);
    t3 = ((((-(t12 * t12 + t11 * t11) * (2 * t12 * t6 + 2 * t7 * t11) * 2 + L2 * L2 * t7 * t11 * 8 + L2 * L2 * t12 * t6 * 8) / (2 * t10)) - (L2 * t6 * 2)) / t9) + (t8 * (t12 * t6 * 2 + t7 * t11 * 2 + L2 * t7 * 2) / (t9 * t9));
    t2 = t4 * t4 + t5 * t5;
    t1 = L1 * arm_sin_f32(phi1) * phi1_dot - (L2 * arm_sin_f32(2 * atan(t8 / t9)) * t3 * 2) / (((t8 * t8) / (t9 * t9)) + 1);
    sq1 = t2;
    arm_sqrt_f32(sq1, &sq1);
    sq2 = (1 - ((t4 * t4) / t2));
    arm_sqrt_f32(sq2, &sq2);
    phi0_dot = ((t1 / sq1) - (t4 * ((L1 * arm_cos_f32(phi1) * phi1_dot + (L2 * arm_cos_f32(2 * atan(t8 / t9)) * 2 * t3) / ((t8 * t8 / (t9 * t9)) + 1)) * (2 * t5 + t1 * t4 * 2)) / (2 * sq1 * sq1 * sq1))) / sq2;

    return phi0_dot;
}
float phi0_dot_R;
float phi0_dot_cal_R(motion_resolve_t *motion)
{
    filter_cal(); 
    float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    float sq1, sq2;
    float phi1_dot, phi4_dot;
    float phi1, phi4;

    phi1 = motion->phi1;
    phi4 = motion->phi4;
    phi1_dot = joint_w_filter[3].out;
    phi4_dot = joint_w_filter[2].out;
    
    t12 = L1 * arm_sin_f32(phi1) - L1 * arm_sin_f32(phi4);
    t11 = L3 - L1 * arm_cos_f32(phi1) + L1 * arm_cos_f32(phi4);
    t10 = 4 * L2 * L2 * t12 * t12 - ((t12 * t12 + t11 * t11) * (t12 * t12 + t11 * t11)) + 4 * L2 * L2 * t11 * t11;
    arm_sqrt_f32(t10, &t10);
    t9 = t12 * t12 + t11 * t11 + 2 * L2 * t11;
    t8 = 2 * L2 * t12 - t10;
    t7 = L1 * arm_sin_f32(phi1) * phi1_dot - L1 * arm_sin_f32(phi4) * phi4_dot;
    t6 = L1 * arm_cos_f32(phi1) * phi1_dot - L1 * arm_cos_f32(phi4) * phi4_dot;
    t5 = L2 * arm_sin_f32(2 * atan(t8 / t9)) - L1 * arm_sin_f32(phi1);
    t4 = L2 * arm_cos_f32(2 * atan(t8 / t9)) - L3 / 2 + L1 * arm_cos_f32(phi1);
    t3 = ((((-(t12 * t12 + t11 * t11) * (2 * t12 * t6 + 2 * t7 * t11) * 2 + L2 * L2 * t7 * t11 * 8 + L2 * L2 * t12 * t6 * 8) / (2 * t10)) - (L2 * t6 * 2)) / t9) + (t8 * (t12 * t6 * 2 + t7 * t11 * 2 + L2 * t7 * 2) / (t9 * t9));
    t2 = t4 * t4 + t5 * t5;
    t1 = L1 * arm_sin_f32(phi1) * phi1_dot - (L2 * arm_sin_f32(2 * atan(t8 / t9)) * t3 * 2) / (((t8 * t8) / (t9 * t9)) + 1);
    sq1 = t2;
    arm_sqrt_f32(sq1, &sq1);
    sq2 = (1 - ((t4 * t4) / t2));
    arm_sqrt_f32(sq2, &sq2);
    phi0_dot_R = ((t1 / sq1) - (t4 * ((L1 * arm_cos_f32(phi1) * phi1_dot + (L2 * arm_cos_f32(2 * atan(t8 / t9)) * 2 * t3) / ((t8 * t8 / (t9 * t9)) + 1)) * (2 * t5 + t1 * t4 * 2)) / (2 * sq1 * sq1 * sq1))) / sq2;

    return phi0_dot_R;
}