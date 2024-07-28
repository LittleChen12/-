/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can.h"


#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

#define TORQUE_CONS 0.32f
#define CAN_MAP 62.5f


/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
	
		CAN_DM_8009_ID_1 = 0x01,
		CAN_DM_8009_ID_2 = 0x02,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);



/**
  * @brief          DM8009的MIT控制命令发送函数
  * @param[in]      
  * @retval     

  * 注释：

  */
void DM8009_ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);

/**
  * @brief          DM8009电机使能函数
  * @param[in]      
  * @retval     

  * 注释：控制前先使能

  */

void DM8009_enable_motor(uint16_t id);

/**
  * @brief          DM8009电机失能函数
  * @param[in]      
  * @retval     

  * 注释：

  */

void DM8009_stop_motor(uint16_t id);

/**
  * @brief          DM8009电机保存位置零点函数
  * @param[in]      
  * @retval     

  * 注释：将当前位置作为零点，断电不丢失

  */

void DM8009_Save_position(uint16_t id);

/**
  * @brief          DM8009电机清除错误函数
  * @param[in]      
  * @retval     

  * 注释：电机红灯闪烁时，调用此函数

  */

void DM8009_RESET(uint16_t id);
extern float DM8009_READ_data_1[8];
extern float DM8009_READ_data_2[8];
extern float DM8009_READ_data_3[8];
extern float DM8009_READ_data_4[8];
extern uint8_t DM8009_ERROR;


/**                                            9025电机结构体等参数                            */

typedef enum
{
    LEFT_MOTOR_RECV = 0x141,
    RIGHT_MOTOR_RECV = 0x142,
    
    Infantry_Data_StdId_1 = 0xB1,
    Infantry_Data_StdId_2 = 0xB2,
	Infantry_Data_StdId_3 = 0xB3,
    
    STATE_RECV = 0xAA,
    CONTROL_RECV = 0xAB
}stdid_recv;

typedef enum
{
	READ_PID_PARAMETER = 0x30,
	WRITE_PID_PARAMETER_TORAM = 0x31,
	WRITE_PID_PARAMETER_TOROM = 0x32,
	READ_ACCEL_DATA = 0x33,
	WRITE_ACCEL_TORAM = 0x34,
	READ_ENCODER_COM = 0x90,
	WRITE_ENCODER_VALUE_AS_ZEROPOINT_TOROM = 0x91,
	WRITE_NOW_POSITION_AS_ZEROPOINT_TOROM = 0x19,    //直接将电机当前位置作为初始位置写入到ROM  不要频繁使用
	READ_MULTIPLE_CIRCLE = 0x92,
	READ_SINGLE_CIRCLE = 0x94,
	SET_MOTOR_INIT_POSTION = 0x95,                   //该功能无法正常使用
	READ_ERROR_FLAG_AND_MOTOR_STATUS_1 = 0x9A,
	CLEAR_ERROR_FLAG = 0x9B,    //电机状态没有恢复正常时，错误标志无法清除 
	READ_MOTOR_STATUS_2 = 0x9C,
	READ_MOTOR_STATUS_3 = 0x9D,
	MOTOR_STOP = 0x80,     //同时清除电机运行状态和之前接受的控制指令
	MOTOR_SUSPEND = 0x81,  //停止电机，但不清除电机运行状态和之前接受的控制指令
	MOTOR_RUN = 0x88,     //恢复停止前的控制方式
	T_OPEN_CIR_CONTROL = 0xA0,  //仅在MS上实现
	T_CLOSE_CIR_CONTROL = 0xA1,  //在MF和MG上实行
	SPEED_CLOSE_CIR_CONTROL = 0xA2,
	POSITION_CLOSE_CIR_CONTROL_1 = 0xA3, //多圈位置
    
	POSITION_CLOSE_CIR_CONTROL_2 = 0xA4, //多圈位置，但有max_speed限制了电机转动的最大速度
	POSITION_CLOSE_CIR_CONTROL_3 = 0xA5, //单圈角度，可设置电机转动方向 
	POSITION_CLOSE_CIR_CONTROL_4 = 0xA6, //单圈角度，可设置电机转动方向，有max_speed限制了电机转动的最大速度
	POSITION_CLOSE_CIR_CONTROL_5 = 0xA7, //控制电机的位置增量
	POSITION_CLOSE_CIR_CONTROL_6 = 0xA8  //控制电机的位置增量,但有max_speed限制了电机转动的最大速度
	
}Stdid_new;

typedef struct{
	uint8_t anglePidKp; //角度环
	uint8_t anglePidKi;
	uint8_t anglePidKd;
	uint8_t speedPidKp; //速度环
	uint8_t speedPidKi;
	uint8_t speedPidKd;
	uint8_t iqPidKp;    //转矩环
	uint8_t iqPidKi;
	uint8_t iqPidKd;
}motor_pid_inside_t;

typedef struct{
	motor_pid_inside_t motor_pid_inside;
	int32_t Accel;
	uint16_t encoder; //为编码器原始位置减去编码器零偏后的值
	uint16_t encoderRaw; //编码器原始位置
	uint16_t encoderOffset; //零偏，该点作为电机角度的0点  可以写入
	int64_t motorAngle;   //电机多圈累计角度值 +表示顺时针, -表示逆时针
	uint32_t circleAngle; //电机单圈角度，以编码器零点为起始点，顺时针增加，再次到达零点时数值回0
	int8_t temperature;  //电机温度 1摄氏度/LSB
	uint16_t voltage; //电压
	uint8_t errorState;// 0位: 0电压正常 1低压保护; 1位，2位:无效; 3位：0温度正常 1过温保护   4、5、6、7无效 
	int16_t iq;   //转矩电流
    int16_t torque;
	float speed;  //电机转速
	int16_t three_phase[3];
	uint8_t spinDirection; //0x00顺时 0x01逆时针
	int16_t power;
}moto_measure_t;

void torque_close_control(CAN_HandleTypeDef *hcan, uint8_t motorID, int16_t iq);


extern moto_measure_t driving_motor[2];//9025轮毂电机[LEFT][RIGHT]
#endif
