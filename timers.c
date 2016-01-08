/*
 * timers.c
 *
 *  Created on: 7.6.2015
 *      Author: Alvar
 */

#include "timers.h"
#include "stm32f2xx_tim.c"

unsigned char tim7Cnt = 0;
/* TIM1 init function */
void timersInit(void){
	//TIM_TimeBaseInitTypeDef Tim_init;

	//Tim_init->TIM_Prescaler = 40;//kaikki timerit samalla 1MHz kellolla(paitsi tim7 joka ei tee PWM:ää)
	//Tim_init->TIM_CounterMode = TIM_CounterMode_Up;
	//Tim_init->TIM_Period = 60000;
	//Tim_init->TIM_ClockDivision = TIM_CKD_DIV1;
	//Tim_init->TIM_RepetitionCounter = 0;

	TIM1->PSC = 39;
	TIM1->ARR = 60000;
	TIM2->PSC = 39;
	TIM2->ARR = 60000;
	TIM3->PSC = 39;
	TIM3->ARR = 60000;
	TIM4->PSC = 39;
	TIM4->ARR = 60000;
	TIM5->PSC = 39;
	TIM5->ARR = 60000;
	TIM8->PSC = 39;
	TIM8->ARR = 60000;
	TIM9->PSC = 39;
	TIM9->ARR = 60000;
	TIM10->PSC = 39;
	TIM10->ARR = 60000;
	TIM11->PSC = 39;
	TIM11->ARR = 60000;
	TIM12->PSC = 39;
	TIM12->ARR = 60000;
	TIM13->PSC = 39;
	TIM13->ARR = 60000;
	TIM14->PSC = 39;
	TIM14->ARR = 60000;

	//TIM1_ch1 pwm input capture (CCR2)
	TIM1->CCMR1 |= TIM_CCMR1_CC2S_1;//kanava 2 saa signaalin TI1
	//TIM1->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC1NP);//rising edge
	TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;//aktiivinen kanava 1 saa signaalin TI1
	//TIM1->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM1->CCER |= TIM_CCER_CC2P;//falling edge johtaa cc1-captureen
	TIM1->SMCR |= TIM_SMCR_TS_0|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM1->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM1->CR1 |= TIM_CR1_CEN;

	//TIM2_ch2 pwm input capture (CCR1)
	TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;//kanava 2 saa signaalin TI2
	//TIM2->CCER &= ~(TIM_CCER_CC2P|TIM_CCER_CC2NP);//rising edge
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_1;//kanava 1 saa signaalin TI2
	//TIM2->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM2->CCER |= TIM_CCER_CC1P;//falling edge johtaa cc1-captureen
	TIM2->SMCR |= TIM_SMCR_TS_1|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM2->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM2->CR1 |= TIM_CR1_CEN;

	//TIM3 ch1234 pwm out
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2M_1; //110, PWM node 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE;//preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1|TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1; //110, PWM node 1
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE|TIM_CCMR2_OC4PE;//preload enable
	TIM3->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;//auto-reload preload, UPDATE DISABLE(käytä ug) ja counter enable
	TIM3->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E; //active high(cc1p=0) ja enable
	TIM3->CCR1 = 1500;//1,5ms
	TIM3->CCR2 = 1500;//1,5ms
	TIM3->CCR3 = 1500;//1,5ms
	TIM3->CCR4 = 1500;//1,5ms
	TIM3->ARR = 40000;//40 ms

	//TIM4_ch1 pwm input capture (CCR2)
	TIM4->CCMR1 |= TIM_CCMR1_CC2S_1;//kanava 2 saa signaalin TI1
	//TIM4->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC1NP);//rising edge
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;//aktiivinen kanava 1 saa signaalin TI1
	//TIM4->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM4->CCER |= TIM_CCER_CC2P;//falling edge johtaa cc1-captureen
	TIM4->SMCR |= TIM_SMCR_TS_0|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM4->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM4->CR1 |= TIM_CR1_CEN;

	//TIM5_ch1 pwm input capture (CCR2)
	TIM5->CCMR1 |= TIM_CCMR1_CC2S_1;//kanava 2 saa signaalin TI1
	//TIM5->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC1NP);//rising edge
	TIM5->CCMR1 |= TIM_CCMR1_CC1S_0;//aktiivinen kanava 1 saa signaalin TI1
	//TIM5->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM5->CCER |= TIM_CCER_CC2P;//falling edge johtaa cc1-captureen
	TIM5->SMCR |= TIM_SMCR_TS_0|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM5->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM5->CR1 |= TIM_CR1_CEN;

	//TIM7 100Hz roll-over
	TIM7->CR1 |= TIM_CR1_ARPE;
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->PSC = 383;//100kHz, todellinen taajuus on 0,96-kertainen
	TIM7->ARR = 999; //100Hz ovf
	TIM7->CR1 |= TIM_CR1_CEN;

	//TIM8_ch3 pwm input capture (CCR2)
	TIM8->CR2 |= TIM_CR2_TI1S;//xor-juttu
	TIM8->CCMR1 |= TIM_CCMR1_CC2S_1;//kanava 2 saa signaalin TI1
	//TIM8->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC1NP);//rising edge
	TIM8->CCMR1 |= TIM_CCMR1_CC1S_0;//aktiivinen kanava 1 saa signaalin TI1
	//TIM8->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM8->CCER |= TIM_CCER_CC2P;//falling edge johtaa cc1-captureen
	TIM8->SMCR |= TIM_SMCR_TS_0|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM8->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM8->CR1 |= TIM_CR1_CEN;

	//TIM9_ch2 pwm input capture (CCR1)
	TIM9->CCMR1 |= TIM_CCMR1_CC2S_0;//kanava 2 saa signaalin TI2
	//TIM9->CCER &= ~(TIM_CCER_CC2P|TIM_CCER_CC2NP);//rising edge
	TIM9->CCMR1 |= TIM_CCMR1_CC1S_1;//kanava 1 saa signaalin TI2
	//TIM9->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM9->CCER |= TIM_CCER_CC1P;//falling edge johtaa cc1-captureen
	TIM9->SMCR |= TIM_SMCR_TS_1|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM9->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM9->CR1 |= TIM_CR1_CEN;

	//TIM10 ch1 pwm out
	TIM10->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1; //110, PWM node 1
	TIM10->CCMR1 |= TIM_CCMR1_OC1PE;//preload enable
	TIM10->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;//auto-reload preload, UPDATE DISABLE(käytä ug) ja counter enable
	TIM10->CCER |= TIM_CCER_CC1E; //active high(cc1p=0) ja enable
	TIM10->CCR1 = 1500;//1,5ms
	TIM10->ARR = 40000;//40 ms

	//TIM11 ch1 pwm out
	TIM11->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1; //110, PWM node 1
	TIM11->CCMR1 |= TIM_CCMR1_OC1PE;//preload enable
	TIM11->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;//auto-reload preload, UPDATE DISABLE(käytä ug) ja counter enable
	TIM11->CCER |= TIM_CCER_CC1E; //active high(cc1p=0) ja enable
	TIM11->CCR1 = 1500;//1,5ms
	TIM11->ARR = 40000;//40 ms

	//TIM12_ch1 pwm input capture (CCR2)
	TIM12->CCMR1 |= TIM_CCMR1_CC2S_1;//kanava 2 saa signaalin TI1
	//TIM12->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC1NP);//rising edge
	TIM12->CCMR1 |= TIM_CCMR1_CC1S_0;//aktiivinen kanava 1 saa signaalin TI1
	//TIM12->CCMR1 |= TIM_CCMR1_IC1F_0|TIM_CCMR1_IC1F_1;//input filter(4)
	TIM12->CCER |= TIM_CCER_CC2P;//falling edge johtaa cc1-captureen
	TIM12->SMCR |= TIM_SMCR_TS_0|TIM_SMCR_TS_2|TIM_SMCR_SMS_2;//reset kun TI2FP2 nousee
	TIM12->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;//enabloidaan
	TIM12->CR1 |= TIM_CR1_CEN;

	//TIM13 ch1 pwm out
	TIM13->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1; //110, PWM node 1
	TIM13->CCMR1 |= TIM_CCMR1_OC1PE;//preload enable
	TIM13->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;//auto-reload preload, UPDATE DISABLE(käytä ug) ja counter enable
	TIM13->CCER |= TIM_CCER_CC1E; //active high(cc1p=0) ja enable
	TIM13->CCR1 = 1500;//1,5ms
	TIM13->ARR = 40000;//40 ms

	//TIM14 ch1 pwm out
	TIM14->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1; //110, PWM node 1
	TIM14->CCMR1 |= TIM_CCMR1_OC1PE;//preload enable
	TIM14->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;//auto-reload preload, UPDATE DISABLE(käytä ug) ja counter enable
	TIM14->CCER |= TIM_CCER_CC1E; //active high(cc1p=0) ja enable
	TIM14->CCR1 = 1500;//1,5ms
	TIM14->ARR = 40000;//40 ms
}

void timersBaseInit(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM7|RCC_APB1Periph_TIM12|RCC_APB1Periph_TIM13|RCC_APB1Periph_TIM14, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_TIM8|RCC_APB2Periph_TIM9|RCC_APB2Periph_TIM10|RCC_APB2Periph_TIM11, ENABLE);

	GPIO_InitTypeDef tim_GPIO_InitStruct;

	/**TIM1 GPIO Configuration
	PA8     ------> TIM1_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, 8, GPIO_AF_TIM1);

	/**TIM2 GPIO Configuration
	PA1     ------> TIM2_CH2
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, 1, GPIO_AF_TIM2);


	/**TIM3 GPIO Configuration
	PB0     ------> TIM3_CH3
	PB1     ------> TIM3_CH4
	PC6     ------> TIM3_CH1
	PC7     ------> TIM3_CH2
	*/

	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &tim_GPIO_InitStruct);

	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &tim_GPIO_InitStruct);

	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &tim_GPIO_InitStruct);

	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &tim_GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, 0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, 1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, 6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, 7, GPIO_AF_TIM3);

	/**TIM4 GPIO Configuration
	PB6     ------> TIM4_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOB, 6, GPIO_AF_TIM4);

	/**TIM5 GPIO Configuration
	PA0-WKUP     ------> TIM5_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, 0, GPIO_AF_TIM5);

	/**TIM8 GPIO Configuration
	PC8     ------> TIM8_CH3
	*/

	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC, 8, GPIO_AF_TIM8);

	/**TIM9 GPIO Configuration
	PA3     ------> TIM9_CH2
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, 3, GPIO_AF_TIM9);

	/**TIM10 GPIO Configuration
	PB8     ------> TIM10_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOB, 8, GPIO_AF_TIM10);

	/**TIM11 GPIO Configuration
	PB9     ------> TIM11_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOB, 9, GPIO_AF_TIM11);

	/**TIM12 GPIO Configuration
	PB14     ------> TIM12_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOB, 14, GPIO_AF_TIM12);

	/**TIM13 GPIO Configuration
	PA6     ------> TIM13_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, 6, GPIO_AF_TIM13);

	/**TIM14 GPIO Configuration
	PA7     ------> TIM14_CH1
	*/
	tim_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	tim_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	tim_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	tim_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tim_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, 7, GPIO_AF_TIM14);
}

void TIM7NvicInit(void){
	/*TIM7 Peripheral interrupt init*/
	/* Sets the priority grouping field */

	NVIC_InitTypeDef tim7_NVIC_InitStruct;

	tim7_NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	tim7_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	tim7_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	tim7_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&tim7_NVIC_InitStruct);
}


void timerUG(void){
	TIM11->EGR |= 1;
	TIM10->EGR |= 1;
	TIM3->EGR |= 1;
	TIM3->EGR |= 1;
	TIM3->EGR |= 1;
	TIM3->EGR |= 1;
	TIM14->EGR |= 1;
	TIM13->EGR |= 1;
}

void delay_ms(unsigned int ms){
	for(long unsigned int i = 0; i<19230*ms; i++);
}


