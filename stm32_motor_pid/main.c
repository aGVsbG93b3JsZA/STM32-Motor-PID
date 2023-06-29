#include "delay.h"
#include "sys.h"
#include "pwm.h"
#include "key.h"
#include "usart.h"
#include "lcd.h"
#include "remote.h"

u16 *ccr;
u16 pwm_arr=1999;
u16 pwm_psc=1799;
u16 cap_arr=29999;
u16 cap_psc=719;
u16 target_speed=0;
u16 motor_speed=0;
int motor_active=0;
int forward=1;
u16 screen_cnt=0;
u16 ccr1;
u16 ccr2;
void update_screen();

int main(void)
{	
	u8 key;
	u8 rmt_key;
	u8 rnt_key;
	u8 WKUP_CNT=0;
	delay_init();
	uart_init(9600);
	delay_init();
	KEY_Init();
	LCD_Init();
	Remote_Init();
	POINT_COLOR=RED; 
	LCD_Clear(WHITE);
	TIM3_PWM_Init(pwm_arr, pwm_psc);
	TIM2_Cap_Init(cap_arr, cap_psc);
  while(1)
	{	
		// Ò£¿ØÆ÷¿ØÖÆ
		rmt_key = Remote_Scan();
		if (rmt_key != rnt_key)
		{	
			rnt_key = rmt_key;
			switch(rnt_key)
			{	
				// Ò£¿ØÆ÷ÉÏ¼ü
				case 98:
					if (target_speed < 100) 
						target_speed += 20,
						update_screen();
					break;
				// Ò£¿ØÆ÷ÏÂ¼ü	
				case 168:
					if (target_speed > 60) 
						target_speed -= 20,
						update_screen();
					break;
				// Ò£¿ØÆ÷×ó¼ü
				case 34:
					if (forward==0) 
					{
						ccr1 = ccr2;
						ccr = &ccr1;
						ccr2 = 0;
						TIM_SetCompare1(TIM3, ccr1);
						TIM_SetCompare2(TIM3, ccr2);
						forward = 1;
						update_screen();
					}
					break;
				// Ò£¿ØÆ÷ÓÒ¼ü
				case 194:
					if (forward==1)
					{
						ccr2 = ccr1;
						ccr = &ccr2;
						ccr1 = 0;
						TIM_SetCompare1(TIM3, ccr1);
						TIM_SetCompare2(TIM3, ccr2);
						forward = 0;
						update_screen();
					}
					break;
				// Ò£¿ØÆ÷ÔÝÍ££¨¿ªÊ¼£©¼ü
				case 2:
					if (motor_active==0)
					{
						ccr = &ccr1;
						target_speed = 80;
						motor_active = 1;
						forward = 1;
					}
					else
					{
						target_speed = 0;
						motor_active = 0;
						*ccr = 0;
						TIM_SetCompare1(TIM3, ccr1);
						TIM_SetCompare2(TIM3, ccr2);
					}
					update_screen();
					break;
				}
		}
		
		// °´¼ü¿ØÖÆ
		key=KEY_Scan(1);		
		switch(key)
		{				 
			case KEY0_PRES:
				if (target_speed < 100) 
					target_speed += 20,
					update_screen();
				break;
				
			case KEY1_PRES:
				if (target_speed > 60) 
					target_speed -= 20,
					update_screen();
				break;
			
			case WKUP_PRES:
				WKUP_CNT = (WKUP_CNT+1) % 3;
				switch(WKUP_CNT)
				{
					case 1:
						ccr = &ccr1;
						target_speed = 80;
						motor_active = 1;
						forward = 1;
						break;
					case 2:
						ccr2 = ccr1;
						ccr = &ccr2;
						ccr1 = 0;
						TIM_SetCompare1(TIM3, ccr1);
						TIM_SetCompare2(TIM3, ccr2);
						forward = 0;
						break;
					case 0:
						target_speed = 0;
						motor_active = 0;
						*ccr = 0;
						TIM_SetCompare1(TIM3, ccr1);
						TIM_SetCompare2(TIM3, ccr2);
						break;
				}
				update_screen();
				break;
		}
		
		if (screen_cnt==0) 
		{
			update_screen();
			screen_cnt ++;
		}
		delay_ms(20);
	}
}
 

void update_screen()
{	
	u8 state[12], target[12], current[12];
	if (motor_active==0) sprintf((char*)state,"OFF      ");
	else if (forward) sprintf((char*)state,"Forward  ");
	else sprintf((char*)state,"Backward");
	
	sprintf((char*)target,"%d r/min  ", target_speed);
	sprintf((char*)current,"%d r/min  ", motor_speed);

	LCD_ShowString(30,40,200,16,16, "State:");
	LCD_ShowString(30,80,200,24,24, state);	
	LCD_ShowString(30,120,200,16,16,"Target Speed:");	
	LCD_ShowString(30,160,200,24,24, target);	
	LCD_ShowString(30,200,200,16,16, "Current Speed:");
	LCD_ShowString(30,240,200,24,24, current);	
}

