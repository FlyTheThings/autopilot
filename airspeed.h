/*
 * airspeed.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_

#include "autopilot.h"
#include <math.h>

#define AIRSPEED_UPDATE_FREQ 5

extern float cumulTempAs;
extern char asCnt;

void ADC1Init(void);
void calibrateADC1(void);
void airspeedUpdate(void);

#endif /* AIRSPEED_H_ */
