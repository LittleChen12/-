#include "chassis_filter.h"


/*
 *  -----------------备注-------------------------    
 *  滤波器均未使用，在theta_dot中采用了滤波后的数据
*/


//状态量以及输出量滤波
first_order_filter_type_t state_filter[6];
float state_filter_para[6];
first_order_filter_type_t output_filter[2];
float output_filter_para[2];
//解算后关节电机力矩滤波
first_order_filter_type_t joint_filter[4];
float joint_filter_para = 0.005;
//关节电机w滤波
first_order_filter_type_t joint_w_filter[4];
float joint_w_filter_para = 0.0007f;
//三阶低通
LPF_t moto_lpf[4];

/**
 * @description: 低通滤波器初始化
 * @param {}
 * @return {*}
 */
void filter_init(void)
{
    //关节电机w滤波
//    first_order_filter_init(&joint_w_filter[0],0.003,&joint_w_filter_para);
//    first_order_filter_init(&joint_w_filter[1],0.003,&joint_w_filter_para);
//    first_order_filter_init(&joint_w_filter[2],0.003,&joint_w_filter_para);
//    first_order_filter_init(&joint_w_filter[3],0.003,&joint_w_filter_para);
//		SetLpfCutoffFreq(&moto_lpf[0],500,100);
//		SetLpfCutoffFreq(&moto_lpf[1],500,100);
//		SetLpfCutoffFreq(&moto_lpf[2],500,100);
//		SetLpfCutoffFreq(&moto_lpf[3],500,100);
	
}

/**
 * @description: 低通滤波器计算
 * @param {}
 * @return {*}
 */
void filter_cal(void)
{
    //关节电机w滤波
//	joint_w_filter[0].out = ApplyLpf(&moto_lpf[0],DM8009_READ_data_3[4]);
//	joint_w_filter[1].out = ApplyLpf(&moto_lpf[1],DM8009_READ_data_4[4]);
//	
//	joint_w_filter[2].out = ApplyLpf(&moto_lpf[2],DM8009_READ_data_1[4]);
//	joint_w_filter[3].out = ApplyLpf(&moto_lpf[3],DM8009_READ_data_2[4]);
	
    //关节电机w滤波
	joint_w_filter[0].out = DM8009_READ_data_3[4];
	joint_w_filter[1].out = DM8009_READ_data_4[4];
	
	joint_w_filter[2].out = DM8009_READ_data_1[4];
	joint_w_filter[3].out = DM8009_READ_data_2[4];
}
