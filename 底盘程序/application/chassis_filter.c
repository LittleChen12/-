#include "chassis_filter.h"


/*
 *  -----------------��ע-------------------------    
 *  �˲�����δʹ�ã���theta_dot�в������˲��������
*/


//״̬���Լ�������˲�
first_order_filter_type_t state_filter[6];
float state_filter_para[6];
first_order_filter_type_t output_filter[2];
float output_filter_para[2];
//�����ؽڵ�������˲�
first_order_filter_type_t joint_filter[4];
float joint_filter_para = 0.005;
//�ؽڵ��w�˲�
first_order_filter_type_t joint_w_filter[4];
float joint_w_filter_para = 0.0007f;
//���׵�ͨ
LPF_t moto_lpf[4];

/**
 * @description: ��ͨ�˲�����ʼ��
 * @param {}
 * @return {*}
 */
void filter_init(void)
{
    //�ؽڵ��w�˲�
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
 * @description: ��ͨ�˲�������
 * @param {}
 * @return {*}
 */
void filter_cal(void)
{
    //�ؽڵ��w�˲�
//	joint_w_filter[0].out = ApplyLpf(&moto_lpf[0],DM8009_READ_data_3[4]);
//	joint_w_filter[1].out = ApplyLpf(&moto_lpf[1],DM8009_READ_data_4[4]);
//	
//	joint_w_filter[2].out = ApplyLpf(&moto_lpf[2],DM8009_READ_data_1[4]);
//	joint_w_filter[3].out = ApplyLpf(&moto_lpf[3],DM8009_READ_data_2[4]);
	
    //�ؽڵ��w�˲�
	joint_w_filter[0].out = DM8009_READ_data_3[4];
	joint_w_filter[1].out = DM8009_READ_data_4[4];
	
	joint_w_filter[2].out = DM8009_READ_data_1[4];
	joint_w_filter[3].out = DM8009_READ_data_2[4];
}
