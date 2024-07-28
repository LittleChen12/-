#ifndef COM_TASK_H
#define COM_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "user_lib.h"

typedef struct {
	const motor_measure_t  *motor_chassis[4];     //底盘电机数据
	
	fp32 chassis_delta_x;
	fp32 chassis_delta_y;
	fp32 chassis_delta_z;
	
	fp32 chassis_z;
	
    fp32 vx;                         //底盘速度 前进方向 前为正，      单位 m/s
    fp32 vy;                         //底盘速度 左右方向 左为正        单位 m/s
    fp32 wz;                         //底盘角速度，逆时针为正          单位 rad/s

    fp32 vx_set;                     //底盘设定速度 前进方向 前为正，  单位  m/s
    fp32 vy_set;                     //底盘设定速度 左右方向 左为正，  单位  m/s
    fp32 wz_set;                     //底盘角速度， 逆时针为正         单位  rad/s
} chassis_data_t;
typedef struct {
    const fp32 *ins_gyro_p;
    const fp32 *ins_accel_p;
    const fp32 *ins_quat_p;
    const fp32 *ins_angle_p;
    fp32 ins_data_gyro[3];
    fp32 ins_data_accel[3];
    fp32 ins_data_quat[4];
    fp32 ins_data_angle[3];
} ins_data_t;
typedef __packed struct {
    fp32 x_pos;
    fp32 y_pos;
    fp32 yaw;
    fp32 ins_data_gyro[3];
    fp32 ins_data_accel[3];
} robot_position_t;

typedef struct {
    chassis_data_t		chassis_data;
    ins_data_t			ins_data;
    robot_position_t	robot_position;
    fp32				yaw_offest;
} data_pool_t;

void com_task(void const *pvParameters);
const chassis_data_t *get_chassis_control_point(void);

#endif
