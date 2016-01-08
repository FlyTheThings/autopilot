/*
 * timers.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "autopilot.h"


extern unsigned char tim7Cnt;

void timersInit(void);
void timersBaseInit(void);
void TIM7NvicInit(void);
void timerUG(void);
void delay_ms(unsigned int ms);

#endif /* TIMERS_H_ */
