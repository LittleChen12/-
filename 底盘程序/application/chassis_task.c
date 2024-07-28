#include "chassis_task.h"

KalmanFilter_t vaEstimateKF;	   // �������˲����ṹ��

float vaEstimateKF_F[4] = {1.0f, 0.002f, 
                           0.0f, 1.0f};	   // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {100.0f, 0.0f, 
                           0.0f, 100.0f};    // Q�����ʼֵ

float vaEstimateKF_R[4] = {0.01f, 0.0f, 
                            0.0f,  0.01f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// ���þ���HΪ����
														 															 
float vel_acc[2]; 
uint32_t OBSERVE_TIME=2;//����������3ms		
						
/*
 * ƽ�������㼰�ؽڵ��ͨ��
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


//�����˶�����
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
//���ʹ��
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
				VMC_calculate();                                                          // ���µ�ǰ����״̬��
				leg_pushforce_cal();                                                      // �Ȳ���ֱ������������
				leg_harmo_pid_cal(motion_resolve[LEFT].phi0, motion_resolve[RIGHT].phi0); // �Ȳ�ˮƽ�������ؼ���
				matrix_calculate();                                        // lqr����
				matrix_calculate_R();  
				leg_speed_cal();
				leg_control_cal();                                                        // �Ȳ������ۺϼ���
				joint_torque_calculate();                                                 // �ۺ��ſɱȼ������չؽڵ���������
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
					torque_close_control(&CHASSIS_CAN,2,leg_control.driving_iq[1]);//���ҷ����Ƿ���	
//					DM8009_ctrl_motor(&CHASSIS_CAN,1,0,0,0,0,0);	
//					DM8009_ctrl_motor(&CHASSIS_CAN,2,0,0,0,0,0);
//					DM8009_ctrl_motor(&CHASSIS_CAN,3,0,0,0,0,0);	
//					DM8009_ctrl_motor(&CHASSIS_CAN,4,0,0,0,0,0);	
//					torque_close_control(&CHASSIS_CAN,1,0);
//					torque_close_control(&CHASSIS_CAN,2,0);//���ҷ����Ƿ���						
				}
			}
		
      vTaskDelayUntil(&lasttick,CHASSIS_CONTROL_TIME_MS);
    }
		
		
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// ״̬����2ά û�п����� ��������2ά
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //�������˲�������ֵ����
    EstimateKF->MeasuredVector[0] =	vel;//�����ٶ�
    EstimateKF->MeasuredVector[1] = acc;//�������ٶ�
    		
    //�������˲������º���
    Kalman_Filter_Update(EstimateKF);

    // ��ȡ����ֵ
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}

