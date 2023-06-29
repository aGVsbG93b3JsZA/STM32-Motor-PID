#include "pwm.h"
#include "sys.h"
#include "usart.h"


void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	                                                                     	

   //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //TIM3_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM3_CH2
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE �����ʹ��	

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
 
   
}






TIM_ICInitTypeDef  TIM2_ICInitStructure;

void TIM2_Cap_Init(u16 arr,u16 psc)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ��TIM2ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;  //PA3 ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA3 ����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);						 //PA3 ����
	
	//��ʼ����ʱ��2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM2���벶�����
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI4��
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI4��
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ
	TIM2_ICInitStructure.TIM_ICFilter = 8;//IC1F=0000 ���������˲���
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC4IE�����ж�	
  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2
 
}



u16 capture_cnt = 0; //���벶����
int increment; //PID����
int err[3]={0}; //PID�����

float Kp=10;
float Ki=5;
float Kd=10;

u16 rct_cnt[5] = {0};
u16 total_cnt = 0;
u8 seq=0;

//������ƽ���ٶ�
u16 update_speed(u16 new_cnt)
{	
	total_cnt = total_cnt + new_cnt - rct_cnt[seq];
	rct_cnt[seq] = new_cnt;
	seq = (seq + 1) % 5;
	return total_cnt * 2;
}

//����PID����
int PID_increment(u16 motor_speed)
{
	err[2] = err[1];
	err[1] = err[0];
	err[0] = target_speed - motor_speed;
	return Kp*(err[0]-err[1]) + Ki*err[0] + Kd*(err[0]-2*err[1]+err[2]);
}

//����PWMռ�ձ�
void upgrade_pwm(int increment)
{	
	int cmp = *ccr + increment;
	if (cmp > pwm_arr) *ccr = pwm_arr;
	else if (cmp < 0) *ccr = 0;
	else *ccr = cmp;
	TIM_SetCompare1(TIM3, ccr1);
	TIM_SetCompare2(TIM3, ccr2);
	printf("ccr: %d\r\n", *ccr);
}

//��ʱ��2�жϷ������	 
void TIM2_IRQHandler(void)
{ 
	// ���벶���ж�
	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
	{
		capture_cnt ++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	}		
	// �����������ն�
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{	
		motor_speed = update_speed(capture_cnt);
		capture_cnt = 0;
		printf("motor_speed: %d\r\n", motor_speed);
		increment = PID_increment(motor_speed);
		if (motor_active) 
			upgrade_pwm(increment);
		screen_cnt = (screen_cnt + 1) % 4;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

