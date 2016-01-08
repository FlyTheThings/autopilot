/*
 * autopilot.h
 *
 *  Created on: 5.6.2015
 *      Author: Alvar
 */


#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#define pi 3.14159265

#include "AHRS.h"
#include "airspeed.h"
#include "altimeter.h"
#include "connection.h"
#include "control.h"
#include "dma.h"
#include "EEPROM.h"
#include "gpio.h"
#include "gps.h"
#include "i2c.h"
#include "parameters.h"
#include "rcc.h"
#include "spi.h"
#include "timers.h"
#include "ui.h"
#include "usart.h"

#define svih GPIOC->BSRRL = GPIO_Pin_4
#define spun GPIOC->BSRRL = GPIO_Pin_5
#define rvih GPIOC->BSRRH = GPIO_Pin_4
#define rpun GPIOC->BSRRH = GPIO_Pin_5
#define rivi print(USART1, "\n\r")

#define ailIn TIM4->CCR2
#define eleIn TIM1->CCR2
#define thrIn TIM8->CCR2
#define rudIn TIM12->CCR2
#define in5 TIM5->CCR2
#define in6 TIM2->CCR1
#define in7 TIM9->CCR1
#define manual (in7 < 1300)
#define stabilized (in7 > 1300 && in7 <1700)
#define mission (in7 > 1700)

#define dps *0.02f //dt ohjauksessa on 0.02s

#define ailOut TIM11->CCR1
#define eleOut TIM10->CCR1
#define thrOut TIM3->CCR2
#define ruddOut TIM3->CCR1
#define out5 TIM3->CCR4
#define out6 TIM3->CCR3
#define out7 TIM14->CCR1
#define out8 TIM13->CCR1
#define gpio1 PC0
#define gpio2 PC1

struct aircraftData{
	float airspeed;
	float alt;
	float roll;
	float pitch;
	float crs;
	float fp;
	float vertSpd;
	float usonicAlt;

	float rollErr;
	float pitchErr;
	float crsErr;
	float altErr;
	float fpErr;
	float airspeedErr;
};
struct aircraftData aircraft;

typedef struct{
	float lat;
	float lon;
	int alt;
	unsigned char number;
}Waypoint;
Waypoint currentWpt;

extern unsigned int maxTime;








#endif /* AUTOPILOT_H_ */
