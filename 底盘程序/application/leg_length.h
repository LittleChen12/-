#ifndef LEG_LENGTH_H
#define LEG_LENGTH_H

#include "main.h"
#include "pid.h"
#include "leg_control.h"
#include "remote_control.h"


typedef struct
{
    pid_type_def leg_length_pid;
    float leg_push_force;
    float desire;
    float state;
    float roll_comp;
}leg_length_t;

void leg_length_pid_init(void);
void leg_length_state_assign(void);
void leg_length_pid_cal(void);
void roll_pid_cal(void);
void leg_pushforce_cal(void);
void clear_leg_pid(void);
void clear_leg_model(void);
float degree_to_rad(float degree);


extern int mode_gogo_flag;
extern leg_length_t leg_length[2];
extern pid_type_def roll_comp;            // roll÷·≤π≥•
extern float left_desire_slow;
extern float right_desire_slow;
#endif
