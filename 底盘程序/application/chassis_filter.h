#ifndef CHASSIS_FILTER_H
#define CHASSIS_FILTER_H

#include "main.h"
#include "CAN_receive.h"
#include "user_lib.h"

void filter_init(void);
void filter_cal(void);
extern first_order_filter_type_t state_filter[6];
extern first_order_filter_type_t output_filter[2];
extern first_order_filter_type_t joint_filter[4];
extern first_order_filter_type_t joint_w_filter[4];

#endif
