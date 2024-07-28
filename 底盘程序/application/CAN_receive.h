/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
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
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);



/**
  * @brief          DM8009��MIT��������ͺ���
  * @param[in]      
  * @retval     

  * ע�ͣ�

  */
void DM8009_ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);

/**
  * @brief          DM8009���ʹ�ܺ���
  * @param[in]      
  * @retval     

  * ע�ͣ�����ǰ��ʹ��

  */

void DM8009_enable_motor(uint16_t id);

/**
  * @brief          DM8009���ʧ�ܺ���
  * @param[in]      
  * @retval     

  * ע�ͣ�

  */

void DM8009_stop_motor(uint16_t id);

/**
  * @brief          DM8009�������λ����㺯��
  * @param[in]      
  * @retval     

  * ע�ͣ�����ǰλ����Ϊ��㣬�ϵ粻��ʧ

  */

void DM8009_Save_position(uint16_t id);

/**
  * @brief          DM8009������������
  * @param[in]      
  * @retval     

  * ע�ͣ���������˸ʱ�����ô˺���

  */

void DM8009_RESET(uint16_t id);
extern float DM8009_READ_data_1[8];
extern float DM8009_READ_data_2[8];
extern float DM8009_READ_data_3[8];
extern float DM8009_READ_data_4[8];
extern uint8_t DM8009_ERROR;


/**                                            9025����ṹ��Ȳ���                            */

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
	WRITE_NOW_POSITION_AS_ZEROPOINT_TOROM = 0x19,    //ֱ�ӽ������ǰλ����Ϊ��ʼλ��д�뵽ROM  ��ҪƵ��ʹ��
	READ_MULTIPLE_CIRCLE = 0x92,
	READ_SINGLE_CIRCLE = 0x94,
	SET_MOTOR_INIT_POSTION = 0x95,                   //�ù����޷�����ʹ��
	READ_ERROR_FLAG_AND_MOTOR_STATUS_1 = 0x9A,
	CLEAR_ERROR_FLAG = 0x9B,    //���״̬û�лָ�����ʱ�������־�޷���� 
	READ_MOTOR_STATUS_2 = 0x9C,
	READ_MOTOR_STATUS_3 = 0x9D,
	MOTOR_STOP = 0x80,     //ͬʱ����������״̬��֮ǰ���ܵĿ���ָ��
	MOTOR_SUSPEND = 0x81,  //ֹͣ�������������������״̬��֮ǰ���ܵĿ���ָ��
	MOTOR_RUN = 0x88,     //�ָ�ֹͣǰ�Ŀ��Ʒ�ʽ
	T_OPEN_CIR_CONTROL = 0xA0,  //����MS��ʵ��
	T_CLOSE_CIR_CONTROL = 0xA1,  //��MF��MG��ʵ��
	SPEED_CLOSE_CIR_CONTROL = 0xA2,
	POSITION_CLOSE_CIR_CONTROL_1 = 0xA3, //��Ȧλ��
    
	POSITION_CLOSE_CIR_CONTROL_2 = 0xA4, //��Ȧλ�ã�����max_speed�����˵��ת��������ٶ�
	POSITION_CLOSE_CIR_CONTROL_3 = 0xA5, //��Ȧ�Ƕȣ������õ��ת������ 
	POSITION_CLOSE_CIR_CONTROL_4 = 0xA6, //��Ȧ�Ƕȣ������õ��ת��������max_speed�����˵��ת��������ٶ�
	POSITION_CLOSE_CIR_CONTROL_5 = 0xA7, //���Ƶ����λ������
	POSITION_CLOSE_CIR_CONTROL_6 = 0xA8  //���Ƶ����λ������,����max_speed�����˵��ת��������ٶ�
	
}Stdid_new;

typedef struct{
	uint8_t anglePidKp; //�ǶȻ�
	uint8_t anglePidKi;
	uint8_t anglePidKd;
	uint8_t speedPidKp; //�ٶȻ�
	uint8_t speedPidKi;
	uint8_t speedPidKd;
	uint8_t iqPidKp;    //ת�ػ�
	uint8_t iqPidKi;
	uint8_t iqPidKd;
}motor_pid_inside_t;

typedef struct{
	motor_pid_inside_t motor_pid_inside;
	int32_t Accel;
	uint16_t encoder; //Ϊ������ԭʼλ�ü�ȥ��������ƫ���ֵ
	uint16_t encoderRaw; //������ԭʼλ��
	uint16_t encoderOffset; //��ƫ���õ���Ϊ����Ƕȵ�0��  ����д��
	int64_t motorAngle;   //�����Ȧ�ۼƽǶ�ֵ +��ʾ˳ʱ��, -��ʾ��ʱ��
	uint32_t circleAngle; //�����Ȧ�Ƕȣ��Ա��������Ϊ��ʼ�㣬˳ʱ�����ӣ��ٴε������ʱ��ֵ��0
	int8_t temperature;  //����¶� 1���϶�/LSB
	uint16_t voltage; //��ѹ
	uint8_t errorState;// 0λ: 0��ѹ���� 1��ѹ����; 1λ��2λ:��Ч; 3λ��0�¶����� 1���±���   4��5��6��7��Ч 
	int16_t iq;   //ת�ص���
    int16_t torque;
	float speed;  //���ת��
	int16_t three_phase[3];
	uint8_t spinDirection; //0x00˳ʱ 0x01��ʱ��
	int16_t power;
}moto_measure_t;

void torque_close_control(CAN_HandleTypeDef *hcan, uint8_t motorID, int16_t iq);


extern moto_measure_t driving_motor[2];//9025��챵��[LEFT][RIGHT]
#endif
