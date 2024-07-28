#ifndef VMC_H
#define VMC_H

#include "main.h"
#include "arm_math.h"

#include "lqr.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "arm_math.h"
#include "pid.h"
#include "chassis_task.h"
#include "user_lib.h"
#include "chassis_filter.h"

//C点坐标定义
#define X 0
#define Y 1

//腿长参数
#define L1 0.15061f
#define L2 0.25447f
#define L3 0.10884f

typedef struct
{
    float J1;
    float J2;
    float J3;
    float J4;
}Jacobi_t;

typedef struct
{
    float point[2];
    float phi1;
    float phi2;
    float phi3;
    float phi4;
    Jacobi_t jacobi;
	
    float L0;
		float L0_dot;
	  float last_L0;
		float L0_ddot;
	  float last_L0_dot;
	
    float phi0;
    float theta;
    float torque[2];
		float FN;
		int dd_L0_count;
}motion_resolve_t;

extern motion_resolve_t motion_resolve[2];

void Motion_calculate(motion_resolve_t *motion, float phi1, float phi4); // phi1是钝角 phi4是锐角
void leg_para_calculate(motion_resolve_t *motion);
void Jacobi_calculate(motion_resolve_t *motion);
void VMC_calculate(void);
float phi0_dot_cal(motion_resolve_t *motion);
float phi0_dot_cal_R(motion_resolve_t *motion);
void joint_torque_calculate(void);
#endif
