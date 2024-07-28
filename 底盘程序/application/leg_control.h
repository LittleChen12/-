#ifndef _LEG_CONTROL_H
#define _LEG_CONTROL_H

#include "main.h"
#include "leg_length.h"
#include "VMC.h"
#include "leg_harmo.h"
#include "lqr.h"
#include "lqr_R.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

#define NO_FEEDFORWARD 0.0f
#define NORMAL_FEEDFORWARD 20.0f
#define FLY_FEEDFORWARD 70.0f
#define LAND_FEEDFORWARD 70.0f

extern first_order_filter_type_t L0_ddot_filter;

typedef struct
{
    float driving_iq[2];
    float joint_tq[2];
    float leg_length_tq[2];
    float leg_feedforward[2];
} leg_control_t;

void leg_init(void);
void leg_cal(void);
void leg_control_cal(void);
void leg_feedforward(leg_control_t *legcontrol ,float left,float right);
void leg_speed_cal(void);
extern leg_control_t leg_control;
extern float leg_feedforward_left; 
extern float leg_feedforward_right;
extern float leg_feedforward_add_r;
extern float leg_feedforward_add_l;
extern int jump_flag;
extern int jump_mode;
extern int start_jump_flag;//开始起跳标准
#endif

