#ifndef LQR_R_H
#define LQR_R_H

#include "lqr.h"
#include "main.h"
#include "arm_math.h"

#include "chassis_task.h"
#include "VMC.h"
#include "leg_length.h"
#include "INS_task.h"
#include "kalman_filter.h"


#define DRIVING_R 0 // Çý¶¯ÂÖ
#define JOINT_R 1   // ÷Å¹Ø½Ú

typedef struct
{
    float32_t state[6];
    float32_t desire[6];
    float32_t error[6];
    float32_t pos_out[2];
		float theta_dot;
		float theta_dot_last;		
		float theta_ddot;
} lqr_t_R;
extern lqr_t_R lqr_R;

typedef struct
{
    float pose;
    float Dpose;
} pose_t_R;
extern pose_t_R pose_R;

int matrix_calculate_R(void);
void matrix_init_R(void);

extern uint8_t ground_detectionR_TEST;

extern int jiliflag_R;

#endif
