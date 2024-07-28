#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "INS_task.h"
#include "user_lib.h"
#include "com_task.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "VMC.h"
#include "lqr.h"
#include "lqr_R.h"
#include "leg_control.h"
#include "kalman_filter.h"



// �Źؽڵ������
#define FRONT 0 // ǰ
#define BACK 1  // ��
// PID�±�
#define RPM 0
#define ANGLE 1
// ����Ǳ�
#define LEFT 0
#define RIGHT 1
// ״̬���Ǳ�
#define POSE_ANGLE 0
#define POSE_SPEED 1
#define DISPLAYCEMENT 2
#define SPEED 3
#define PITCH 4
#define PITCH_SPEED 5

#define X 0
#define X_SPEED 1
#define THETA 2
#define THETA_SPEED 3
#define DELTA 4
#define DELTA_SPEED 5

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ��
#define CHASSIS_CONTROL_TIME_MS   2
#define CHASSIS_CONTROL_TIME_S    0.002f
//ң������������
#define CHASSIS_RC_DEADLINE      10

//�����˶��������ǰ���ٶ�(m/s)
#define NORMAL_MAX_CHASSIS_SPEED_X    1.0f
//�����˶��������ƽ���ٶ�(m/s)
#define NORMAL_MAX_CHASSIS_SPEED_Y    1.0f
//�����˶����������ת�ٶ�(rad/s)
#define NORMAL_MAX_CHASSIS_SPEED_Z    (PI / 2)
//���̵������ٶ�(m/s)
#define MAX_WHEEL_SPEED               4.0f

//����ģʽͨ������
#define CHASSIS_MODE_SWITCH           1
//ң��ģʽͨ������
#define REMOTE_MODEL_SWITCH           0
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL             2
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL             3
//��ת��ң����ͨ������
#define CHASSIS_Z_CHANNEL             0

#define WHEEL_FB 0.40f //ǰ���־�
#define WHEEL_RL 0.43f //�����־�
//M3508ת�٣�r/min��ת���ɵ����ٶ�(m/s)�ı���
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    0.00041580974890349451720903384703738f
//M3508����������ת�������Ӿ���(m)�ı���
#define CHASSIS_MOTOR_ECD_TO_MOVE_SEN    0.0000030454815593517664834646033718558f
//�̶�����
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX   0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY   0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ   0.25f
//һ���˲������趨
#define CHASSIS_ACCEL_X_NUM        0.038f
#define CHASSIS_ACCEL_Y_NUM        0.038f
#define CHASSIS_ACCEL_Z_NUM        0.038f

//���̵���ٶ�PID
#define CHASSIS_MOTOR_SPEED_PID_KP          15000.0f
#define CHASSIS_MOTOR_SPEED_PID_KI          10.0f
#define CHASSIS_MOTOR_SPEED_PID_KD          0.0f
#define CHASSIS_MOTOR_SPEED_PID_MAX_OUT     16000.0f
#define CHASSIS_MOTOR_SPEED_PID_MAX_IOUT    10000.0f

//������ת����PID
#define CHASSIS_FOLLOW_YAW_PID_KP 5.0f
#define CHASSIS_FOLLOW_YAW_PID_KI 0.0f
#define CHASSIS_FOLLOW_YAW_PID_KD 0.0f
#define CHASSIS_FOLLOW_YAW_PID_MAX_OUT  (PI/2)
#define CHASSIS_FOLLOW_YAW_PID_MAX_IOUT 0.0f

//����״̬���趨
typedef enum
{
    CHASSIS_RELAX,
    CHASSIS_STOP,
    CHASSIS_HEAD,
    CHASSIS_NOHEAD,
    CHASSIS_AUTO,
} chassis_mode_t;


//�������Ϣ���ݰ�
typedef struct
{
    const motor_measure_t   *chassis_motor_measure;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;

} Chassis_Motor_t;


typedef  struct
{
    const RC_ctrl_t         *chassis_RC;             //ң����ָ��
    const fp32              *chassis_INS_angle;
    const chassis_data_t    *chassis_auto_set;

    chassis_mode_t         chassis_mode;          //��ǰ״̬��
    chassis_mode_t         last_chassis_mode;

    Chassis_Motor_t        motor_chassis[4];      //���̵������
    pid_type_def           motor_speed_pid[4];    //���̵���ٶ�pid
    pid_type_def		   chassis_angle_pid;     //���̸���Ƕ�pid

    first_order_filter_type_t     chassis_cmd_slow_set_vx;    //����ǰ��һ���˲�
    first_order_filter_type_t     chassis_cmd_slow_set_vy;    //��������һ���˲�
    first_order_filter_type_t     chassis_cmd_slow_set_wz;    //������תһ���˲�

    fp32 vx;                         //�����ٶ� ǰ������ ǰΪ����      ��λ m/s
    fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��        ��λ m/s
    fp32 wz;                         //���̽��ٶȣ���ʱ��Ϊ��          ��λ rad/s

    fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ����  ��λ  m/s
    fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ����  ��λ  m/s
    fp32 wz_set;                     //���̽��ٶȣ� ��ʱ��Ϊ��         ��λ  rad/s


    fp32 chassis_yaw_set;

    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;

    fp32 nohead_yaw_offest;
} chassis_move_t;

typedef enum
{
    CHASSIS_INIT_MODE = 0,
    CHASSIS_RC_MODE,
    CHASSIS_KEYBOARD_MODE,
    CHASSIS_DISABLE_MODE,
} chassis_control_mode_t;

typedef struct
{
    float yaw;
    float speed;
    float leg_length[2];
} chassis_desire_t;

typedef struct
{
    float yaw;
    float speed;
    float leg_length[2];
} chassis_state_t;

typedef struct
{
    chassis_desire_t chassis_desire;
    chassis_state_t chassis_state;
} chassis_status_t;

// ƽ���־λ
typedef enum
{
    BALANCE,
    SMALL_ANGLE_BALANCE,
    UNBALANCE,
} balance_flag_t;

typedef enum
{
    LEG_CONVERGE,
    LEG_DIVERGE,
} leg_status_flag_t;

typedef enum
{
    LAND_BALANCE,
    FLY_STATUS,
    LAND_UNBALANCE,
} fly_status_t;

typedef struct
{
    balance_flag_t balance_flag;
    leg_status_flag_t leg_status_flag;
    fly_status_t fly_status_flag;
} chassis_flag_t;

typedef struct
{
    chassis_status_t chassis_status;
    chassis_flag_t chassis_flag;
    float leg_angle_init[4];
    float leg_angle[4];
    float leg_force[4];
    float wheel_force[2];

    float yaw_offset;

    int16_t Vxy_power_set;
    int16_t Vw_power_set;
    int16_t Vshift_power_set;
    int16_t Vctrl_power_set;

    float Vx_set;
    float Vy_set;
    float Vw_set;

    float chassis_yaw;
    float chassis_pitch;
    float chassis_yaw_speed;

    float chassis_relative_angle_offset;
    float chassis_yaw_relative_angle; // ������̨ƫ���
    float chassis_pit_relative_angle;
    float chassis_speed_rate[2];

    float L0_avg; // ƽ���ȳ�

    uint16_t slow_Inc;
    uint16_t key_slow_Inc;

//    pid_t chassis_follow_pid[2];
//    pid_t chassis_roll_pid;
//    fuzzy_pid_t chassis_follow_fuzzy_pid[2]; // ���̸���ģ��pid
//    
//    pid_t chassis_rpm_pid[2];
		
		const fp32 *INS_ANGLE;
		
    chassis_control_mode_t chassis_control_mode;
    chassis_control_mode_t chassis_control_last_mode;

} balance_chassis_t;


void chassis_task(void const *pvParameters);
const chassis_move_t *get_chassis_move_point(void);

extern balance_chassis_t balance_chassis;
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern float vel_acc[2]; 
#endif
