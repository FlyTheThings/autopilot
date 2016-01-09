/*
 * parameters.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "autopilot.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//Flags for separate eeprom storing
extern volatile char saveNumberOfWpts_Flag;
extern volatile char saveCurrentWptNumber_Flag;
extern unsigned int numberOfParameters;

extern unsigned int eeprom_paramsStart; //the first character of the first parameter name
extern unsigned int eeprom_wptsStart; //the first character of the first waypoint's number

float kpRoll, kpPitch, kpYaw, kpCrs, kpAs, pitch2thr_mix, kpFp, kpVertSpd, kiRoll, kiPitch, kiYaw, kiCrs, kiFp, kiAs,
	kdRoll, kdPitch, kdYaw, kdCrs, kdFp, kdAs,

	lpfGainFp, lpfGainErrFp, lpfGainGpsCrs,

	maxVertSpd, minVertSpd, maxAs, minAs, minAsClb, minAsRoll, crzAs, maxBankAbs, maxBankNav, maxPitch, minPitch, roll2ele_mix,
	vel2ail_mix, vel2ele_mix,

	//Cruize altitude:
	crzAlt,

	//Servo output values:
	iMaxPitch, iMinPitch, iMaxRoll, iMaxThr, iMaxCrs, iMaxFp,

	maxThr, minThr, ail0, ele0, rud0, thr0,

	minAilOut, maxAilOut, minEleOut, maxEleOut, minRudOut, maxRudOut,

	//Acc & gyro offsets:
	axOffset, ayOffset, azOffset, gxOffset, gyOffset, gzOffset,

	//ADC offset:
	ADC1_U0,

	//Altimeter offset:
	h0,

	//Waypoints
	numberOfWpts, currWptNumber;


typedef struct struct_parameter{
	const char name[15];
	unsigned int eepromLocation;
	float* value;
} parameter;

extern parameter parameters[];

void loadParameters(void);
void saveParameter(char* name, float value, char singleExecute);
void initializeEeprom(void);
void editWpt(char* data);
void addWpt(char* data);
void resetRoute(void);
void updateCurrentWpt(void);
void formatFloatForEeprom(char* dst, float val);

#endif /* PARAMETERS_H_ */
