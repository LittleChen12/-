#ifndef LQR_H
#define LQR_H
#include "main.h"
#include "arm_math.h"

#include "chassis_task.h"
#include "VMC.h"
#include "leg_length.h"
#include "INS_task.h"
#include "kalman_filter.h"

#define DRIVING 0 // 驱动轮
#define JOINT 1   // 髋关节

typedef struct
{
    float32_t state[6];
    float32_t desire[6];
    float32_t error[6];
    float32_t pos_out[2];
		
		float theta_dot;
		float theta_dot_last;		
		float theta_ddot;
} lqr_t;

typedef struct
{
    float pose;
    float Dpose;
} pose_t;



extern lqr_t lqr;

extern int jiliflag_l;
extern float32_t k[12];
extern float32_t torque[2]; //[驱动轮输出力矩/髋关节输出力矩]
extern float k_para[12];
extern float jacobi_left[4];
extern float inv_jacobi_left[4];
extern float jacobi_right[4];
extern float inv_jacobi_right[4];

extern arm_matrix_instance_f32 ptr_k;
extern arm_matrix_instance_f32 ptr_state;
extern arm_matrix_instance_f32 ptr_torque;
extern arm_matrix_instance_f32 ptr_jacobi_left;
extern arm_matrix_instance_f32 ptr_inv_jacobi_left;
extern arm_matrix_instance_f32 ptr_jacobi_right;
extern arm_matrix_instance_f32 ptr_inv_jacobi_right;

void matrix_init(void);
void matrix_state_assign(void);
void matrix_desire_assign(void);
void matrix_error_calculate(void);
int matrix_calculate(void);
void match_k(void);
float data_fusion(void);
extern float lqr_speed;
extern float rc_control_speed;
extern uint8_t ground_detectionL_TEST;

void QIANKUI_slow(float *rec , float target , float slow_Inc);
#endif
