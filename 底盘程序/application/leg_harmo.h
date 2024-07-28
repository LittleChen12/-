#ifndef LEG_HARMO_H
#define LEG_HARMO_H

#include "main.h"
#include "pid.h"
#include "VMC.h"

typedef struct
{
    pid_type_def leg_harmo_pid;
    float leg_harmo_out;
}leg_harmo_t;

extern leg_harmo_t leg_harmo;
extern float leg_harmo_out_right;


void leg_harmo_pid_init(void);
void leg_harmo_pid_cal(float leg_left_angle,float leg_right_angle);
void leg_harmo_pid_clear(void);
#endif
