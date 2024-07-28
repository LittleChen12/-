#include "com_task.h"
#include "usart.h"

#include "stdio.h"
#include "cmsis_os.h"

#include "user_lib.h"
#include "arm_math.h"
#include "detect_task.h"
#include "chassis_task.h"

static data_pool_t data_pool = {0};
const chassis_move_t *chassis_feedback;

static void data_pool_init(data_pool_t *data_init);
static void data_pool_update(data_pool_t *data_update);

static uint8_t uart1TxBuf[200];//传感器数据上传
static uint8_t uart1Tx_len;
//与主机通信
static uint8_t uart6Rx_flag = 0;
static uint8_t uart6RxBuf[80];
static uint8_t uart6TxBuf[80];

//static __packed struct {
//    fp32 vx_set;
//    fp32 vy_set;
//    fp32 wz_set;
//    uint8_t flag;
//    fp32 real_x;
//    fp32 real_y;
//    fp32 real_z;
//} pc_data;
static __packed struct {
    uint16_t QR_x;
    uint16_t QR_y;
    fp32 delta_x;
    fp32 delta_y;
    fp32 real_z;
	uint8_t null;
} pc_data;
void com_task(void const *pvParameters)
{
    uint8_t uart6Tx_cnt = 0;

    vTaskDelay(2000);

    uart6TxBuf[0] = 0xab;
    uart6TxBuf[1] = 0xef;
    uart6TxBuf[2 + sizeof(robot_position_t)] = 0x55;

    data_pool_init(&data_pool);
    TickType_t lasttick = xTaskGetTickCount();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6,uart6RxBuf,sizeof(uart6RxBuf));
    while (1)
    {
        data_pool_update(&data_pool);
		
		data_pool.chassis_data.chassis_delta_x *= 1.0169f;
		data_pool.chassis_data.chassis_delta_y *= 0.9605f;
        data_pool.robot_position.x_pos += data_pool.chassis_data.chassis_delta_x * arm_cos_f32(data_pool.robot_position.yaw) 
										  - data_pool.chassis_data.chassis_delta_y * arm_sin_f32(data_pool.robot_position.yaw);
        data_pool.robot_position.y_pos += data_pool.chassis_data.chassis_delta_x * arm_sin_f32(data_pool.robot_position.yaw)
										  + data_pool.chassis_data.chassis_delta_y * arm_cos_f32(data_pool.robot_position.yaw);
		data_pool.chassis_data.chassis_z += data_pool.chassis_data.chassis_delta_z;
		data_pool.chassis_data.chassis_z = 1.0031f * rad_format(data_pool.chassis_data.chassis_z);
		
		static float x_pos = 0.0f,y_pos = 0.0f,z_pos = 0.0f * PI/180;
		x_pos += data_pool.chassis_data.chassis_delta_x * arm_cos_f32(data_pool.robot_position.yaw + z_pos) 
					- data_pool.chassis_data.chassis_delta_y * arm_sin_f32(data_pool.robot_position.yaw + z_pos);
		y_pos += data_pool.chassis_data.chassis_delta_x * arm_sin_f32(data_pool.robot_position.yaw + z_pos)
					+ data_pool.chassis_data.chassis_delta_y * arm_cos_f32(data_pool.robot_position.yaw + z_pos);

        if(uart6Rx_flag == 1 && pc_data.null != 0)
        {
			uart6Rx_flag = 0;
			static float qr_x = 0,qr_y = 0,qr_z = 0;
			qr_z = pc_data.real_z*180/PI;
			qr_x = pc_data.QR_x - pc_data.delta_x*arm_sin_f32(qr_z)+pc_data.delta_y*arm_cos_f32(qr_z);
			qr_y = pc_data.QR_y + pc_data.delta_x*arm_cos_f32(qr_z)+pc_data.delta_y*arm_sin_f32(qr_z);
			qr_z = pc_data.real_z;
//			if(data_pool.robot_position.x_pos > 0.5f)
//			{
//				uart1Tx_len = sprintf((void*)uart1TxBuf,"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",x_pos,y_pos,data_pool.robot_position.yaw*180/PI,qr_x,qr_y,qr_z);
//				HAL_UART_Transmit_DMA(&huart1,uart1TxBuf,uart1Tx_len);
//			}
//            data_pool.chassis_data.vx_set = pc_data.vx_set;
//            data_pool.chassis_data.vy_set = pc_data.vy_set;
//            data_pool.chassis_data.wz_set = pc_data.wz_set;
//            data_pool.robot_position.x_pos = qr_x;
//            data_pool.robot_position.y_pos = qr_y;
//            data_pool.yaw_offest = data_pool.ins_data.ins_data_angle[0] - pc_data.real_z;
        }
		
        uart6Tx_cnt ++;
        if(uart6Tx_cnt == 50)
        {
            uart6Tx_cnt = 0;
            memcpy(uart6TxBuf + 2,&data_pool.robot_position, sizeof(robot_position_t));
            //HAL_UART_Transmit_DMA(&huart6,uart6TxBuf,3 + sizeof(robot_position_t));
		
//			uart1Tx_len = sprintf((void*)uart1TxBuf,"%.6f\r\n",data_pool.ins_data.ins_data_angle[0]);
//			HAL_UART_Transmit_DMA(&huart1,uart1TxBuf,uart1Tx_len);
        }
		
        if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart6,uart6RxBuf,sizeof(uart6RxBuf));
        }
        vTaskDelayUntil(&lasttick,1);
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart -> Instance == USART6)
    {
        if(*(uint16_t*)uart6RxBuf == 0xefab && uart6RxBuf[Size - 1] == 0x55 && Size == sizeof(pc_data) + 3)
        {
            memcpy(&pc_data,uart6RxBuf + 2, sizeof(pc_data));
            uart6Rx_flag = 1;
            detect_hook(MINIPC_TOE);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6,uart6RxBuf,sizeof(uart6RxBuf));
    }
}
static void data_pool_init(data_pool_t *data_init)
{
	for( uint8_t i = 0; i < 4; i++)
    {
        data_init->chassis_data.motor_chassis[i] = get_chassis_motor_measure_point(i);
    }
	
    chassis_feedback = get_chassis_move_point();

    data_init->ins_data.ins_gyro_p = get_gyro_data_point();
    data_init->ins_data.ins_accel_p = get_accel_data_point();
	data_init->ins_data.ins_quat_p = get_INS_quat_point();
    data_init->ins_data.ins_angle_p = get_INS_angle_point();

    data_pool_update(data_init);

    data_init->yaw_offest = data_init->ins_data.ins_data_angle[0];
}
static void data_pool_update(data_pool_t *data_update)
{
	static int16_t motor_ecd_delat[4];
	
	for(uint8_t i = 0; i < 4; i++)
    {
        motor_ecd_delat[i] = data_update->chassis_data.motor_chassis[i]->ecd - data_update->chassis_data.motor_chassis[i]->last_ecd;
		motor_ecd_delat[i] = motor_ecd_delat[i] > 4096 ? motor_ecd_delat[i] - 8192 : motor_ecd_delat[i];
		motor_ecd_delat[i] = motor_ecd_delat[i] < -4096 ? motor_ecd_delat[i] + 8192 : motor_ecd_delat[i];
    }
	
	data_update->chassis_data.chassis_delta_x = (-motor_ecd_delat[0] + motor_ecd_delat[1] + motor_ecd_delat[2] - motor_ecd_delat[3])
												* CHASSIS_MOTOR_ECD_TO_MOVE_SEN * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    data_update->chassis_data.chassis_delta_y = (-motor_ecd_delat[0] - motor_ecd_delat[1] + motor_ecd_delat[2] + motor_ecd_delat[3])
												* CHASSIS_MOTOR_ECD_TO_MOVE_SEN * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    data_update->chassis_data.chassis_delta_z = (-motor_ecd_delat[0] - motor_ecd_delat[1] - motor_ecd_delat[2] - motor_ecd_delat[3])
												* CHASSIS_MOTOR_ECD_TO_MOVE_SEN * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / ((WHEEL_FB + WHEEL_RL)/2);
	
    data_update->chassis_data.vx = chassis_feedback->vx;
    data_update->chassis_data.vy = chassis_feedback->vy;
    data_update->chassis_data.wz = chassis_feedback->wz;

    data_update->robot_position.ins_data_gyro[0] = data_update->ins_data.ins_data_gyro[0] = *(data_update->ins_data.ins_gyro_p + INS_GYRO_X_ADDRESS_OFFSET);
    data_update->robot_position.ins_data_gyro[1] = data_update->ins_data.ins_data_gyro[1] = *(data_update->ins_data.ins_gyro_p + INS_GYRO_Y_ADDRESS_OFFSET);
    data_update->robot_position.ins_data_gyro[2] = data_update->ins_data.ins_data_gyro[2] = *(data_update->ins_data.ins_gyro_p + INS_GYRO_Z_ADDRESS_OFFSET);

    data_update->robot_position.ins_data_accel[0] = data_update->ins_data.ins_data_accel[0] = *(data_update->ins_data.ins_accel_p + INS_ACCEL_X_ADDRESS_OFFSET);
    data_update->robot_position.ins_data_accel[1] = data_update->ins_data.ins_data_accel[1] = *(data_update->ins_data.ins_accel_p + INS_ACCEL_Y_ADDRESS_OFFSET);
    data_update->robot_position.ins_data_accel[2] = data_update->ins_data.ins_data_accel[2] = *(data_update->ins_data.ins_accel_p + INS_ACCEL_Z_ADDRESS_OFFSET);

	data_update->ins_data.ins_data_quat[0] = *data_update->ins_data.ins_quat_p;
	data_update->ins_data.ins_data_quat[1] = *(data_update->ins_data.ins_quat_p + 1);
	data_update->ins_data.ins_data_quat[2] = *(data_update->ins_data.ins_quat_p + 2);
	data_update->ins_data.ins_data_quat[3] = *(data_update->ins_data.ins_quat_p + 3);
	
    data_update->ins_data.ins_data_angle[0] = *(data_update->ins_data.ins_angle_p + INS_YAW_ADDRESS_OFFSET);
    data_update->ins_data.ins_data_angle[1] = *(data_update->ins_data.ins_angle_p + INS_PITCH_ADDRESS_OFFSET);
    data_update->ins_data.ins_data_angle[2] = *(data_update->ins_data.ins_angle_p + INS_ROLL_ADDRESS_OFFSET);

    data_update->robot_position.yaw = rad_format(data_update->ins_data.ins_data_angle[0] - data_update->yaw_offest);
}
const chassis_data_t *get_chassis_control_point(void)
{
    return &data_pool.chassis_data;
}
