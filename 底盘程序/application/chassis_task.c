#include "chassis_task.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.002f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {100.0f, 0.0f, 
                           0.0f, 100.0f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {0.01f, 0.0f, 
                            0.0f,  0.01f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
float vel_acc[2]; 
uint32_t OBSERVE_TIME=2;//任务周期是3ms		
						
/*
 * 平衡解算计算及关节电机通信
 */

balance_chassis_t balance_chassis;

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


//底盘运动数据
static chassis_move_t   chassis_move = {0};

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

int lqr_cal_flag;
int code_start;
void chassis_task(void const *pvParameters)
{
    vTaskDelay(2000);
//电机使能
		DM8009_enable_motor(1);
		DM8009_enable_motor(2);
		DM8009_enable_motor(3);
		DM8009_enable_motor(4);
		
		leg_init();
		//xvEstimateKF_Init(&vaEstimateKF);

		float mgggg;
		TickType_t lasttick = xTaskGetTickCount();
    while (1)
    {
			if(rc_ctrl.rc.s[0]==1)
			{
				code_start = 1;
			}
			if(code_start == 1)
			{
				filter_cal();
				VMC_calculate();                                                          // 更新当前所有状态量
				leg_pushforce_cal();                                                      // 腿部竖直方向推力计算
				leg_harmo_pid_cal(motion_resolve[LEFT].phi0, motion_resolve[RIGHT].phi0); // 腿部水平方向力矩计算
				matrix_calculate();                                        // lqr计算
				matrix_calculate_R();  
				leg_speed_cal();
				leg_control_cal();                                                        // 腿部解算综合计算
				joint_torque_calculate();                                                 // 综合雅可比计算最终关节电机输出力矩
				if(mgggg <= 1000)
				{
					mgggg++;
				}
				else
				{
					DM8009_ctrl_motor(&CHASSIS_CAN,1,0,0,0,0,motion_resolve[RIGHT].torque[BACK]);	
					DM8009_ctrl_motor(&CHASSIS_CAN,2,0,0,0,0,motion_resolve[RIGHT].torque[FRONT]);
					DM8009_ctrl_motor(&CHASSIS_CAN,3,0,0,0,0,motion_resolve[LEFT].torque[FRONT]);	
					DM8009_ctrl_motor(&CHASSIS_CAN,4,0,0,0,0,motion_resolve[LEFT].torque[BACK]);	
					torque_close_control(&CHASSIS_CAN,1,leg_control.driving_iq[0]);
					torque_close_control(&CHASSIS_CAN,2,leg_control.driving_iq[1]);//左右方向是反的	
//					DM8009_ctrl_motor(&CHASSIS_CAN,1,0,0,0,0,0);	
//					DM8009_ctrl_motor(&CHASSIS_CAN,2,0,0,0,0,0);
//					DM8009_ctrl_motor(&CHASSIS_CAN,3,0,0,0,0,0);	
//					DM8009_ctrl_motor(&CHASSIS_CAN,4,0,0,0,0,0);	
//					torque_close_control(&CHASSIS_CAN,1,0);
//					torque_close_control(&CHASSIS_CAN,2,0);//左右方向是反的						
				}
			}
		
      vTaskDelayUntil(&lasttick,CHASSIS_CONTROL_TIME_MS);
    }
		
		
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}

