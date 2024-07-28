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



// 髋关节电机定义
#define FRONT 0 // 前
#define BACK 1  // 后
// PID下标
#define RPM 0
#define ANGLE 1
// 电机角标
#define LEFT 0
#define RIGHT 1
// 状态量角标
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
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔
#define CHASSIS_CONTROL_TIME_MS   2
#define CHASSIS_CONTROL_TIME_S    0.002f
//遥控器死区限制
#define CHASSIS_RC_DEADLINE      10

//底盘运动过程最大前进速度(m/s)
#define NORMAL_MAX_CHASSIS_SPEED_X    1.0f
//底盘运动过程最大平移速度(m/s)
#define NORMAL_MAX_CHASSIS_SPEED_Y    1.0f
//底盘运动过程最大旋转速度(rad/s)
#define NORMAL_MAX_CHASSIS_SPEED_Z    (PI / 2)
//底盘电机最大速度(m/s)
#define MAX_WHEEL_SPEED               4.0f

//底盘模式通道开关
#define CHASSIS_MODE_SWITCH           1
//遥控模式通道开关
#define REMOTE_MODEL_SWITCH           0
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL             2
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL             3
//旋转的遥控器通道号码
#define CHASSIS_Z_CHANNEL             0

#define WHEEL_FB 0.40f //前后轮距
#define WHEEL_RL 0.43f //左右轮距
//M3508转速（r/min）转化成底盘速度(m/s)的比例
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    0.00041580974890349451720903384703738f
//M3508编码器计数转化成轮子距离(m)的比例
#define CHASSIS_MOTOR_ECD_TO_MOVE_SEN    0.0000030454815593517664834646033718558f
//固定参数
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX   0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY   0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ   0.25f
//一阶滤波参数设定
#define CHASSIS_ACCEL_X_NUM        0.038f
#define CHASSIS_ACCEL_Y_NUM        0.038f
#define CHASSIS_ACCEL_Z_NUM        0.038f

//底盘电机速度PID
#define CHASSIS_MOTOR_SPEED_PID_KP          15000.0f
#define CHASSIS_MOTOR_SPEED_PID_KI          10.0f
#define CHASSIS_MOTOR_SPEED_PID_KD          0.0f
#define CHASSIS_MOTOR_SPEED_PID_MAX_OUT     16000.0f
#define CHASSIS_MOTOR_SPEED_PID_MAX_IOUT    10000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_YAW_PID_KP 5.0f
#define CHASSIS_FOLLOW_YAW_PID_KI 0.0f
#define CHASSIS_FOLLOW_YAW_PID_KD 0.0f
#define CHASSIS_FOLLOW_YAW_PID_MAX_OUT  (PI/2)
#define CHASSIS_FOLLOW_YAW_PID_MAX_IOUT 0.0f

//底盘状态机设定
typedef enum
{
    CHASSIS_RELAX,
    CHASSIS_STOP,
    CHASSIS_HEAD,
    CHASSIS_NOHEAD,
    CHASSIS_AUTO,
} chassis_mode_t;


//单电机信息数据包
typedef struct
{
    const motor_measure_t   *chassis_motor_measure;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;

} Chassis_Motor_t;


typedef  struct
{
    const RC_ctrl_t         *chassis_RC;             //遥控器指针
    const fp32              *chassis_INS_angle;
    const chassis_data_t    *chassis_auto_set;

    chassis_mode_t         chassis_mode;          //当前状态机
    chassis_mode_t         last_chassis_mode;

    Chassis_Motor_t        motor_chassis[4];      //底盘电机数据
    pid_type_def           motor_speed_pid[4];    //底盘电机速度pid
    pid_type_def		   chassis_angle_pid;     //底盘跟随角度pid

    first_order_filter_type_t     chassis_cmd_slow_set_vx;    //底盘前后一阶滤波
    first_order_filter_type_t     chassis_cmd_slow_set_vy;    //底盘左右一阶滤波
    first_order_filter_type_t     chassis_cmd_slow_set_wz;    //底盘旋转一阶滤波

    fp32 vx;                         //底盘速度 前进方向 前为正，      单位 m/s
    fp32 vy;                         //底盘速度 左右方向 左为正        单位 m/s
    fp32 wz;                         //底盘角速度，逆时针为正          单位 rad/s

    fp32 vx_set;                     //底盘设定速度 前进方向 前为正，  单位  m/s
    fp32 vy_set;                     //底盘设定速度 左右方向 左为正，  单位  m/s
    fp32 wz_set;                     //底盘角速度， 逆时针为正         单位  rad/s


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

// 平衡标志位
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
    float chassis_yaw_relative_angle; // 底盘云台偏差角
    float chassis_pit_relative_angle;
    float chassis_speed_rate[2];

    float L0_avg; // 平均腿长

    uint16_t slow_Inc;
    uint16_t key_slow_Inc;

//    pid_t chassis_follow_pid[2];
//    pid_t chassis_roll_pid;
//    fuzzy_pid_t chassis_follow_fuzzy_pid[2]; // 底盘跟随模糊pid
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
