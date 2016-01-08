/*
 * gpio.c
 *
 *  Created on: 7.6.2015
 *      Author: Alvar
 */
#include "gpio.h"

void GPIOABCHInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOH, ENABLE);

    GPIOC->MODER |= GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0;//LEDs


}
