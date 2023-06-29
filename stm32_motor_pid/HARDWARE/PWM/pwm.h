#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

extern u16 pwm_arr, cap_arr;
extern u16 *ccr;
extern u16 ccr1, ccr2;
extern u16 target_speed;
extern u16 motor_speed;
extern int motor_active;
extern u16 screen_cnt;

void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM2_Cap_Init(u16 arr,u16 psc);

#endif
