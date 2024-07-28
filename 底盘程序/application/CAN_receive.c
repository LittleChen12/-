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

#include "CAN_receive.h"
#include "stm32f4xx_hal_can.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

moto_measure_t driving_motor[2];//9025轮毂电机[LEFT][RIGHT]
#define LEFT 0
#define RIGHT 1

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  DM8009_tx_message;		
static uint8_t DM8009_can_send_data[8];

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
uint8_t rx_flag = 0;
//该数组的前3个数据没有作用。后5个数据分别是位置，速度，扭矩
float DM8009_READ_data_1[8];
float DM8009_READ_data_2[8];
float DM8009_READ_data_3[8];
float DM8009_READ_data_4[8];
uint8_t DM8009_ERROR;

void get_motor_inside_pid(moto_measure_t *motor, uint8_t *Data)
{
	motor->motor_pid_inside.anglePidKp = Data[2];
	motor->motor_pid_inside.anglePidKi = Data[3];
	motor->motor_pid_inside.speedPidKp = Data[4];
	motor->motor_pid_inside.speedPidKi = Data[5];
	motor->motor_pid_inside.iqPidKp =    Data[6];
	motor->motor_pid_inside.iqPidKi =    Data[7];
	
}
/**
  * @brief          获取电机加速度
  * @param[out]     
  * @retval         none
  */
void get_motor_accel(moto_measure_t *motor, uint8_t *Data)
{
	motor->Accel = (int32_t)(Data[4] | Data[5] << 8 | Data[6] << 16 | Data[7] << 24);
}

/**
  * @brief          获取电机编码值
  * @param[out]     
  * @retval         none
  */
void get_motor_encoder(moto_measure_t *motor, uint8_t *Data)
{
	motor->encoder = (uint16_t)(Data[2] | Data[3] << 8);
	motor->encoderRaw = (uint16_t)(Data[4] | Data[5] << 8);
	motor->encoderOffset = (uint16_t)(Data[6] | Data[7] << 8);
}

/**
  * @brief          获取电机零点
  * @param[out]     
  * @retval         none
  */
void get_motor_zeropoint(moto_measure_t *motor, uint8_t *Data)
{
	motor->encoderOffset = (uint16_t)(Data[6] | Data[7] << 8);
}

/**
  * @brief          读取多圈角度命令
  * @param[out]     
  * @retval         none
  */
void get_multiple_angle(moto_measure_t *motor, uint8_t *Data)
{
	motor->motorAngle = (int64_t)(Data[1] | Data[2] << 8 | Data[3] << 16 | Data[4] << 24 | (Data[5] << 16) << 16 | (Data[6] << 20) << 20 | (Data[7] << 24) << 24);
}

/**
  * @brief          读取单圈角度命令
  * @param[out]     
  * @retval         none
  */
void get_single_angle(moto_measure_t *motor, uint8_t *Data)
{
	motor->circleAngle = (uint32_t)(Data[4] | Data[5] << 8 | Data[6] << 16 | Data[7] << 24);
}

/**
* @brief          读取电机状态1和错误标志命令
  * @param[out]     
  * @retval         none
  */
void get_errorStatus_and_motorStatus1(moto_measure_t *motor, uint8_t *Data)
{
	motor->temperature = (int8_t)(Data[1]);
	motor->voltage = (uint16_t)(Data[3] | Data[4] << 8);
	motor->errorState = Data[7];
}

#define Radius 0.1f
/**
* @brief          读取电机状态2
  * @param[out]     
  * @retval         none
  */
void get_motor_status_2(moto_measure_t *motor, uint8_t *Data)
{
    int16_t speed;
		motor->temperature = (int8_t)(Data[1]);
		motor->iq = (int16_t)(Data[2] | Data[3] << 8);
		speed = (int16_t)(Data[4] | Data[5] << 8);
        motor->speed = speed*Radius/57.29f;
		motor->encoder = (uint16_t)(Data[6] | Data[7] << 8);
        motor->torque = motor->iq / 0.32f;
}

/**
  * @brief          读取电机状态3
  * @param[out]     
  * @retval         none
  */
void get_motor_status_3(moto_measure_t *motor, uint8_t *Data)
{
	motor->three_phase[0] = (int16_t)(Data[2] | Data[3] << 8);
	motor->three_phase[1] = (int16_t)(Data[4] | Data[5] << 8);
	motor->three_phase[2] = (int16_t)(Data[6] | Data[7] << 8);
	motor->temperature = (int8_t)(Data[1]);
}

/**
  * @brief          转矩开环控制
  * @param[out]     
  * @retval         none
  */
void get_motor_T_open_message(moto_measure_t *motor, uint8_t *Data)
{
	motor->temperature = (int8_t)(Data[1]);
	motor->power = (int16_t)(Data[2] | Data[3] << 8);
	motor->speed = (int16_t)(Data[4] | Data[5] << 8);
	motor->encoder = (uint16_t)(Data[6] | Data[7] << 8);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
    switch (rx_data[0])
    {
			case 1:
			{
        //get motor id
        DM8009_READ_data_1[0] = rx_data[1]<<8|rx_data[2];
				DM8009_READ_data_1[1] = rx_data[3]<<4|(rx_data[4]>>4);
				DM8009_READ_data_1[2] = (rx_data[4]&0xF)<<8|rx_data[5];
				
				DM8009_READ_data_1[3] = uint_to_float(DM8009_READ_data_1[0], P_MIN, P_MAX, 16);
				DM8009_READ_data_1[4] = uint_to_float(DM8009_READ_data_1[1], V_MIN, V_MAX, 12);
				DM8009_READ_data_1[5] = uint_to_float(DM8009_READ_data_1[2], T_MIN, T_MAX, 12);
				
				DM8009_READ_data_1[6] = rx_data[6];
				DM8009_READ_data_1[7] = rx_data[7];
				rx_flag = 1;
        break;
			}				
			case 2:
			{
        //get motor id
        DM8009_READ_data_2[0] = rx_data[1]<<8|rx_data[2];
				DM8009_READ_data_2[1] = rx_data[3]<<4|(rx_data[4]>>4);
				DM8009_READ_data_2[2] = (rx_data[4]&0xF)<<8|rx_data[5];
				
				DM8009_READ_data_2[3] = uint_to_float(DM8009_READ_data_2[0], P_MIN, P_MAX, 16);
				DM8009_READ_data_2[4] = uint_to_float(DM8009_READ_data_2[1], V_MIN, V_MAX, 12);
				DM8009_READ_data_2[5] = uint_to_float(DM8009_READ_data_2[2], T_MIN, T_MAX, 12);
				
				DM8009_READ_data_2[6] = rx_data[6];
				DM8009_READ_data_2[7] = rx_data[7];
				rx_flag = 1;
        break;
			}				
			case 3:
			{
        //get motor id
        DM8009_READ_data_3[0] = rx_data[1]<<8|rx_data[2];
				DM8009_READ_data_3[1] = rx_data[3]<<4|(rx_data[4]>>4);
				DM8009_READ_data_3[2] = (rx_data[4]&0xF)<<8|rx_data[5];
				
				DM8009_READ_data_3[3] = uint_to_float(DM8009_READ_data_3[0], P_MIN, P_MAX, 16);
				DM8009_READ_data_3[4] = uint_to_float(DM8009_READ_data_3[1], V_MIN, V_MAX, 12);
				DM8009_READ_data_3[5] = uint_to_float(DM8009_READ_data_3[2], T_MIN, T_MAX, 12);
				
				DM8009_READ_data_3[6] = rx_data[6];
				DM8009_READ_data_3[7] = rx_data[7];
				rx_flag = 1;
        break;
			}				
			case 4:
			{
        //get motor id
        DM8009_READ_data_4[0] = rx_data[1]<<8|rx_data[2];
				DM8009_READ_data_4[1] = rx_data[3]<<4|(rx_data[4]>>4);
				DM8009_READ_data_4[2] = (rx_data[4]&0xF)<<8|rx_data[5];
				
				DM8009_READ_data_4[3] = uint_to_float(DM8009_READ_data_4[0], P_MIN, P_MAX, 16);
				DM8009_READ_data_4[4] = uint_to_float(DM8009_READ_data_4[1], V_MIN, V_MAX, 12);
				DM8009_READ_data_4[5] = uint_to_float(DM8009_READ_data_4[2], T_MIN, T_MAX, 12);
				
				DM8009_READ_data_4[6] = rx_data[6];
				DM8009_READ_data_4[7] = rx_data[7];
				rx_flag = 1;
        break;
			}
			case READ_PID_PARAMETER:
			case WRITE_PID_PARAMETER_TORAM:
			case WRITE_PID_PARAMETER_TOROM:			
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_motor_inside_pid(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV:  get_motor_inside_pid(&driving_motor[RIGHT],rx_data); break;
				}
				break;
				
			case READ_ACCEL_DATA:
			case WRITE_ACCEL_TORAM:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:  get_motor_accel(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV: get_motor_accel(&driving_motor[RIGHT],rx_data); break;
				}
			break;
		
			case READ_ENCODER_COM:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_motor_encoder(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV:  get_motor_encoder(&driving_motor[RIGHT],rx_data); break;
				}
				
									
			break;
				 
			case WRITE_ENCODER_VALUE_AS_ZEROPOINT_TOROM:
			case WRITE_NOW_POSITION_AS_ZEROPOINT_TOROM:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_motor_zeropoint(&driving_motor[LEFT],rx_data);  break;
						case RIGHT_MOTOR_RECV:   get_motor_zeropoint(&driving_motor[RIGHT],rx_data); break;
				}
				
								 
			break;
				
			case READ_MULTIPLE_CIRCLE:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_multiple_angle(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV:  get_multiple_angle(&driving_motor[RIGHT],rx_data); break;
				}
				
								 
			break;
				 
			case READ_SINGLE_CIRCLE:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_single_angle(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV:  get_single_angle(&driving_motor[RIGHT],rx_data); break;
				}

								 
			break;
				 
//					 case SET_MOTOR_INIT_POSTION: //未实现
//					      break;
				 
			case READ_ERROR_FLAG_AND_MOTOR_STATUS_1:
			case CLEAR_ERROR_FLAG:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_errorStatus_and_motorStatus1(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV:  get_errorStatus_and_motorStatus1(&driving_motor[RIGHT],rx_data); break;
				}
			break;
				 
			case READ_MOTOR_STATUS_2:
			case T_CLOSE_CIR_CONTROL:
			case SPEED_CLOSE_CIR_CONTROL:
			case POSITION_CLOSE_CIR_CONTROL_1:
			case POSITION_CLOSE_CIR_CONTROL_2:
			case POSITION_CLOSE_CIR_CONTROL_3:
			case POSITION_CLOSE_CIR_CONTROL_4:
			case POSITION_CLOSE_CIR_CONTROL_5:
			case POSITION_CLOSE_CIR_CONTROL_6:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_motor_status_2(&driving_motor[LEFT],rx_data); rx_flag = 1;break;
						case RIGHT_MOTOR_RECV:  get_motor_status_2(&driving_motor[RIGHT],rx_data); rx_flag = 1;break;
				}
			break;
				 
			case READ_MOTOR_STATUS_3:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_motor_status_3(&driving_motor[LEFT],rx_data); rx_flag = 1;break;
						case RIGHT_MOTOR_RECV:  get_motor_status_3(&driving_motor[RIGHT],rx_data); rx_flag = 1;break;
				}
			break;
				 
			case MOTOR_STOP:
			case MOTOR_RUN:
			case MOTOR_SUSPEND:
			break;
				 
			case T_OPEN_CIR_CONTROL:
				switch(rx_header.StdId)
				{
						case LEFT_MOTOR_RECV:   get_motor_T_open_message(&driving_motor[LEFT],rx_data); break;
						case RIGHT_MOTOR_RECV:  get_motor_T_open_message(&driving_motor[RIGHT],rx_data); break;
				}
			break;

			default:
							{break;}
    }
}


/*
  * @brief          
  * @param[in]      
  * @param[in]      
  * @param[in]      
  * @param[in]      
  * @retval         
  */


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
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

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
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


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
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

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
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

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
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


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
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


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
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}


/**
  * @brief          DM8009的MIT命令采用浮点数据等比例转换成整数
  * @param[in]      
  * @retval     

  * 注释：这两个转换函数需要首先确定两个等比例转换的最大最小值，这两个值可以
	在参数设定页面查询，其中 KP、KD 的最大最小值默认分别为 0.0~500.0、0.0~5.0。
	Pos、Vel、Torque分别预设为±12.5、±45、±18，这三个参数可以根据电机的实际参数进行调整。
	但发送控制命令时，一定要与设定值保持一致， 否则会控制命令会发生等比例缩放。
  */

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;	
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;	
	float offset = x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  发送标准ID的数据帧
 * @param  hcan     CAN的句柄
 * @param  ID       数据帧ID
 * @param  pData    数组指针
 * @param  Len      字节数0~8
 */
uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
	
	CAN1->TSR  &=~ 0x00000001;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==0)
	{
	}
	HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1);
	while(rx_flag != 1)
	{
	}
	rx_flag = 0;
	
	return 1;
}

/**
  * @brief          DM8009电机使能函数
  * @param[in]      
  * @retval     

  * 注释：控制前先使能

  */

void DM8009_enable_motor(uint16_t id)
{
	static uint8_t date_enable[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
  CANx_SendStdData(&CHASSIS_CAN,id,date_enable,8);	
}

/**
  * @brief          DM8009电机失能函数
  * @param[in]      
  * @retval     

  * 注释：

  */

void DM8009_stop_motor(uint16_t id)
{
	static uint8_t date_enable[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd};
  CANx_SendStdData(&CHASSIS_CAN,id,date_enable,8);	
}

/**
  * @brief          DM8009电机保存位置零点函数
  * @param[in]      
  * @retval     

  * 注释：将当前位置作为零点，断电不丢失

  */

void DM8009_Save_position(uint16_t id)
{
	static uint8_t date_enable[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe};
  CANx_SendStdData(&CHASSIS_CAN,id,date_enable,8);	
}

/**
  * @brief          DM8009电机清除错误函数
  * @param[in]      
  * @retval     

  * 注释：电机红灯闪烁时，调用此函数

  */

void DM8009_RESET(uint16_t id)
{
	static uint8_t date_enable[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfb};
  CANx_SendStdData(&CHASSIS_CAN,id,date_enable,8);	
}



/**
  * @brief          DM8009的MIT控制命令发送函数
  * @param[in]      
  * @retval     

  * 注释：

  */

void DM8009_ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  uint32_t send_mail_box;
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	DM8009_tx_message.StdId = id;
	DM8009_tx_message.IDE = CAN_ID_STD;
	DM8009_tx_message.RTR = CAN_RTR_DATA;
	DM8009_tx_message.DLC = 0x08;
	DM8009_can_send_data[0] = (pos_tmp >> 8);
	DM8009_can_send_data[1] = pos_tmp;
	DM8009_can_send_data[2] = (vel_tmp >> 4);
	DM8009_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	DM8009_can_send_data[4] = kp_tmp;
	DM8009_can_send_data[5] = (kd_tmp >> 4);
	DM8009_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	DM8009_can_send_data[7] = tor_tmp;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==0)
	{
	}
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &DM8009_tx_message, DM8009_can_send_data, &send_mail_box);
	while(rx_flag != 1)
	{
	}
	rx_flag = 0;
}

/**
  * @brief          9025
  * @param[in]      
  * @retval     

  * 注释：

  */
#define DEVICE_STD_ID (0x140)
/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void send_message(CAN_HandleTypeDef *hcan, uint8_t mark, uint8_t motorID, uint8_t Tx1, uint8_t Tx2, uint8_t Tx3, uint8_t Tx4, uint8_t Tx5, uint8_t Tx6, uint8_t Tx7)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	
	TxHeader.StdId = DEVICE_STD_ID + motorID;
	TxHeader.ExtId = 0x00;
	TxHeader.DLC = 0x08;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	
	TxData[0] = mark;
	TxData[1] = Tx1;
	TxData[2] = Tx2;
	TxData[3] = Tx3;
	TxData[4] = Tx4;
	TxData[5] = Tx5;
	TxData[6] = Tx6;
	TxData[7] = Tx7;
	
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
	{
		Error_Handler();
	}
	while(rx_flag != 1)
	{
	}
	rx_flag = 0;
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void send_message_float(CAN_HandleTypeDef *hcan, uint32_t canID, float Tx1, float Tx2)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	
	TxHeader.StdId = canID;
	TxHeader.ExtId = 0x00;
	TxHeader.DLC = 0x08;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	
	
	TxData[0] = *(uint32_t *)&Tx1;
	TxData[1] = *(uint32_t *)&Tx1 >> 8;
	TxData[2] = *(uint32_t *)&Tx1 >> 16;
	TxData[3] = *(uint32_t *)&Tx1 >> 24;
	TxData[4] = *(uint32_t *)&Tx2;
	TxData[5] = *(uint32_t *)&Tx2 >> 8;
	TxData[6] = *(uint32_t *)&Tx2 >> 16;
	TxData[7] = *(uint32_t *)&Tx2 >> 24;
	
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0)
	{
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
		{
			Error_Handler();
		}
	}
	while(rx_flag != 1)
	{
	}
	rx_flag = 0;
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void write_pid_parameter_to_motorRAM_com(CAN_HandleTypeDef *hcan, uint8_t motorID, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t iqKp, uint8_t iqKi)
{
	send_message(hcan, WRITE_PID_PARAMETER_TORAM, motorID, 0,  angleKp, angleKi, speedKp, speedKi, iqKp, iqKi);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void read_current_motor_pid_inside_com(CAN_HandleTypeDef *hcan, uint8_t motorID)
{
	send_message(hcan, READ_PID_PARAMETER, motorID, 0,0, 0, 0, 0, 0, 0);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void write_pid_parameter_to_motorROM_com(CAN_HandleTypeDef *hcan,uint8_t motorID, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t iqKp, uint8_t iqKi)
{
	send_message(hcan, WRITE_PID_PARAMETER_TOROM, motorID,  0,angleKp, angleKi, speedKp, speedKi, iqKp, iqKi);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void read_accel_com(CAN_HandleTypeDef *hcan, uint8_t motorID)
{
	send_message(hcan, READ_ACCEL_DATA, motorID,  0,0, 0, 0, 0, 0, 0);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void write_accel_toRAM_com(CAN_HandleTypeDef *hcan ,uint8_t motorID, int32_t Accel)
{
	send_message(hcan, WRITE_ACCEL_TORAM, motorID,  0,0, 0, ((uint8_t *)(&Accel))[0], ((uint8_t *)(&Accel))[1], ((uint8_t *)(&Accel))[2], ((uint8_t *)(&Accel))[3]);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void read_multiple_angle(CAN_HandleTypeDef *hcan,uint8_t motorID)
{
	send_message(hcan, READ_MULTIPLE_CIRCLE, motorID,  0, 0, 0, 0, 0, 0, 0);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void read_motor_status_2(CAN_HandleTypeDef *hcan,uint8_t motorID)
{
	send_message(hcan, READ_MOTOR_STATUS_2, motorID, 0, 0, 0, 0, 0, 0, 0);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void write_run_com(CAN_HandleTypeDef *hcan, uint8_t motorID)
{
	send_message(hcan, MOTOR_RUN, motorID, 0, 0, 0, 0, 0, 0, 0);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void write_moto_stop(CAN_HandleTypeDef *hcan, uint8_t motorID)
{
    send_message(hcan, MOTOR_STOP, motorID,0,0,0,0,0,0,0);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void position_clcir_com(CAN_HandleTypeDef *hcan,uint8_t motorID, int32_t angleControl)
{
	send_message(hcan, POSITION_CLOSE_CIR_CONTROL_5, motorID, 0,0,0, ((uint8_t *)(&angleControl))[0], ((uint8_t *)(&angleControl))[1], ((uint8_t *)(&angleControl))[2], ((uint8_t *)(&angleControl))[3]);
}

/**
    *@bref. 
    *@param[in] set： 
    *@param[in] real	
    */
void torque_close_control(CAN_HandleTypeDef *hcan, uint8_t motorID, int16_t iq)
{
    send_message(hcan, T_CLOSE_CIR_CONTROL, motorID,0,0,0,((uint8_t*)(&iq))[0], ((uint8_t*)(&iq))[1], 0, 0);
}
