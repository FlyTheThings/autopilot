/*
 * rcc.c
 *
 *  Created on: 7.6.2015
 *      Author: Alvar
 */

#include "rcc.h"


void systemClockConfig(void)
{
	FLASH->ACR = FLASH_ACR_DCEN|FLASH_ACR_ICEN|FLASH_ACR_PRFTEN|FLASH_Latency_3;

	RCC->CR |= RCC_CR_HSEBYP;
	while(!(RCC->CR & RCC_CR_HSERDY));

	RCC_PLLConfig(RCC_PLLSource_HSE, 16, 320, 4, 4);
	RCC->CR |= RCC_CR_PLLON;

	while(!(RCC->CR & RCC_CR_PLLRDY));

	//RCC_ClockSecuritySystemCmd(ENABLE);//voisi tehdä sitten interruptin, joka asettaa PLL:n ja muut uudelleen.

	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div4);
	RCC_PCLK2Config(RCC_HCLK_Div4);

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	RCC->CR &= !RCC_CR_HSION;

};
