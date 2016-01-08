/*
 * altimeter.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef ALTIMETER_H_
#define ALTIMETER_H_

#include "autopilot.h"
#include <math.h>

#define ALTITUDE_UPDATE_FREQ 5
#define QNH 101300 //using the standard atmospheric pressure at msl for now

//Other variables
extern int temperature;
extern float pressure;
extern unsigned char bmp183Cnt;
extern unsigned char bmp183Cal;

extern unsigned char altCnt;
extern float tempAlt[2];
extern float prevAlt;

//Boolean values
extern volatile char altitudeValid;

void BMP183Init(void);
void altitudeUpdate(void);
void calibrateAltimeter(void);
char bmp183SpiRead(char reg);
void bmp183SpiWrite(char reg, char val);

#endif /* ALTIMETER_H_ */
