/*
 * parameters.c
 *
 *  Created on: 5.6.2015
 *      Author: Alvar
 */

#include "parameters.h"

volatile char saveNumberOfWpts_Flag = 0;
volatile char saveCurrentWptNumber_Flag = 0;
unsigned int numberOfParameters = 0;

unsigned int eeprom_paramsStart = 0; //the first character of the first parameter name
unsigned int eeprom_wptsStart = 0; //the first character of the first waypoint's number

parameter parameters[] = {
		{"kpRoll", 0, &kpRoll},
		{"kpPitch", 0, &kpPitch},
		{"kpYaw", 0, &kpYaw},
		{"kpCrs", 0, &kpCrs},
		{"kpAs", 0, &kpAs},
		{"pitch2thr_mix", 0, &pitch2thr_mix},
		{"kpFp", 0, &kpFp},
		{"kpVertSpd", 0, &kpVertSpd},

		{"kiRoll", 0, &kiRoll},
		{"kiPitch", 0, &kiPitch},
		{"kiYaw", 0, &kiYaw},
		{"kiCrs", 0, &kiCrs},
		{"kiFp", 0, &kiFp},
		{"kiAs", 0, &kiAs},

		{"kdRoll", 0, &kdRoll},
		{"kdPitch", 0, &kdPitch},
		{"kdYaw", 0, &kdYaw},
		{"kdCrs", 0, &kdCrs},
		{"kdFp", 0, &kdFp},
		{"kdAs", 0, &kdAs},

		//Low-pass filter gain stored as percentiles
		{"lpfGainFp%", 0, &lpfGainFp},
		{"lpfGainErrFp%", 0, &lpfGainErrFp},
		{"lpfGainGps%", 0, &lpfGainGpsCrs},

		{"maxVertSpd", 0, &maxVertSpd},
		{"minVertSpd", 0, &minVertSpd},
		{"maxAs", 0, &maxAs},
		{"minAs", 0, &minAs},
		{"minAsClb", 0, &minAsClb},
		{"minAsRoll", 0, &minAsRoll},
		{"crzAs", 0, &crzAs},
		{"maxBankAbs", 0, &maxBankAbs},
		{"maxBankNav", 0, &maxBankNav},
		{"maxPitch", 0, &maxPitch},
		{"minPitch", 0, &minPitch},
		{"roll2ele_mix", 0, &roll2ele_mix},
		{"vel2ail_mix", 0, &vel2ail_mix},
		{"vel2ele_mix", 0, &vel2ele_mix},

		//Cruize altitude:
		{"crzAlt", 0, &crzAlt},

		//Servo output values:
		{"iMaxPitch", 0, &iMaxPitch},
		{"iMinPitch", 0, &iMinPitch},
		{"iMaxRoll", 0, &iMaxRoll},
		{"iMaxThr", 0, &iMaxThr},
		{"iMaxCrs", 0, &iMaxCrs},
		{"iMaxFp", 0, &iMaxFp},

		{"maxThr", 0, &maxThr},
		{"minThr", 0, &minThr},
		{"ail0", 0, &ail0},
		{"ele0", 0, &ele0},
		{"rud0", 0, &rud0},
		{"thr0", 0, &thr0},

		{"minAilOut", 0, &minAilOut},
		{"maxAilOut", 0, &maxAilOut},
		{"minEleOut", 0, &minEleOut},
		{"maxEleOut", 0, &maxEleOut},
		{"minRudOut", 0, &minRudOut},
		{"maxRudOut", 0, &maxRudOut},

		//Acc & gyro offsets:
		{"axOffset", 0, &axOffset},
		{"ayOffset", 0, &ayOffset},
		{"azOffset", 0, &azOffset},
		{"gxOffset", 0, &gxOffset},
		{"gyOffset", 0, &gyOffset},
		{"gzOffset", 0, &gzOffset},

		//ADC offset:
		{"ADC1_U0", 0, &ADC1_U0},

		//Altimeter offset:
		{"h0", 0, &h0},

		//Waypoints
		{"numberOfWpts", 0, &numberOfWpts},
		{"currWptNumber", 0, &currWptNumber},
};



//loads the parameters from the EEPROM memory.
/*Style:
 *
Parameters:
kpRoll +0010.8
kpPitch +0007.5
kpYaw +0001.2

Waypoints:
0 +60.123456 +024.123456 090
1 +60.123345 +024.133355 120
 *
 *
 */
void loadParameters(void){
	/*Reading the parameters from EEPROM.
	 * It must be taken into account that all parameters must fit in the eepromReadBuffer.
	 * Currently the maximum number is 128 parameters for the size of eepromReadBuffer being 256 bytes.
	 */
	char line[40] = {0};//what the max line length could possibly ever be...
	char name[15] = {0};
	numberOfParameters  = sizeof(parameters) / sizeof(parameters[0]);
	unsigned int pos = 0;
	unsigned int linePos = 0;
	while(pos < 30000){//For a 32768 byte eeprom 30kb could be ok top limit.
		eepromReadLine(line, &pos);
		if(strstr(line, "Waypoints:")){//to check where the waypoints start
			eeprom_wptsStart = pos+1;//the first waypoint should start after the newline character.
			break;
		}

		if(strstr(line, "Parameters:")){
			eeprom_paramsStart = pos+1;

			while(1){
				pos++;
				eepromReadLine(line, &pos);
				if(strlen(line) == 0){
					pos++;//avoiding stop at the line terminating character
					break;
				}
				linePos = 0;
				while(line[linePos]){
					if(line[linePos] != ' '){
						name[linePos] = line[linePos];
						linePos++;
					}
					else{
						name[linePos] = 0;
						linePos++;
						for(unsigned int i = 0; i<numberOfParameters; i++){
							if(!strcmp(parameters[i].name, name)){
								parameters[i].eepromLocation = pos-strlen(line)+linePos;
								*(parameters[i].value) = (float)atof(line+linePos);
							}
						}
					}
				}
			}
		}
	}

	eepromBusy = 1;
}

//Saves a parameter to the variable and the EEPROM memory.
//if singleExecute is set the parameter is saved within one function call
void saveParameter(char* name, float value, char singleExecute){
	for(unsigned int i = 0; i < numberOfParameters; i++){
		if(!strcmp(parameters[i].name, name)){
			*(parameters[i].value) = value;
			char s[10];
			formatFloatForEeprom(s, value);
			eepromWriteString(s, parameters[i].eepromLocation, singleExecute);
		}
	}
}

void initializeEeprom(void){
	unsigned int pos = 0;
	numberOfParameters  = sizeof(parameters) / sizeof(parameters[0]);
	eepromWriteString("Parameters:\n", pos, 1);
	pos += 12;
	delay_ms(10);

	char line[30];
	for(unsigned int i = 0; i < numberOfParameters; i++){
		sprintf(line, "%s +0000.0\n", parameters[i].name);
		eepromWriteString(line, pos, 1);
		pos += strlen(line);
		delay_ms(10);
	}
	eepromWriteString("\nWaypoints:\n0\n\n", pos, 1);
	delay_ms(10);

}

//First waypoint number is 0. The data should be in format eg. 0 +60.123456 +024.123456 120\n1
void editWpt(char* data){
	int number = atoi(data);//the first number is the number.
	if(number >= numberOfWpts){
		return;
	}

	char line[40];
	unsigned int pos = eeprom_wptsStart;
	while(pos < 30000){
		eepromReadLine(line, &pos);
		if(atoi(line) == number){
			char wpt [40];
			sprintf(wpt, "%s\n%d ", data, number+1); //Making sure that ther is a newline and a number after every waypoint and that the next number can be rad with atoi.
			eepromWriteString(wpt, pos-strlen(line), 0);
			return;
		}else if(strlen(line) == 0){
			return;
		}
		pos++;
	}
}

//Adds a waypoint.
void addWpt(char* data){
	char wpt[40];
	char line[40];
	unsigned int pos = eeprom_wptsStart;
	while(pos < 30000){ //making sure that no infinite loop or end of memory
		eepromReadLine(line, &pos);
		if(atoi(line) == (int)numberOfWpts){
			sprintf(wpt,"%d %s\n%d\n", (int)numberOfWpts, data, (int)(numberOfWpts+1));//making sure that there is the terminating newline character so that the last line is empty,
			eepromWriteString(wpt, pos-strlen(line), 0);
			numberOfWpts++;
			saveNumberOfWpts_Flag = 1;
			return;
		}
		pos++;
	}
}


//(pseudo-)removes all waypoints but the first. numberOfWpts can't be zero as it may lead to division by zero.
void resetRoute(void){
	saveParameter("currWptNumber", 0, 0);
	numberOfWpts = 1;
	saveNumberOfWpts_Flag = 1;
}

void updateCurrentWpt(void){

	char line[40];
	char* linePos = line;
	unsigned int pos = eeprom_wptsStart;
	while(1){
		eepromReadLine(line, &pos);
		if(atoi(line) == currWptNumber){
			linePos = strchr(linePos, ' ') + 1;
			currentWpt.lat = atof(linePos);
			linePos = strchr(linePos, ' ') + 1;
			currentWpt.lon = atof(linePos);
			linePos = strchr(linePos, ' ') + 1;
			currentWpt.alt = atoi(linePos);
			saveCurrentWptNumber_Flag = 1;
			return;
		}else if(strlen(line) == 0){
			return;
		}
		pos++;
	}
	eepromBusy = 1;
}

//Format +1234.5. Note that limits are -9999.9 - 9999.9
void formatFloatForEeprom(char* dst, float val){
	if(val < -9999.9)
		val = -9999.9;
	else if(val > 9999.9)
		val = 9999.9;
	int i1 = val;							// Get the integer part
	float f2;
	if(val > 0)
		f2 = val - i1; 						// Get fractional part
	else
		f2 = i1 - val;

	int i2 = roundf(f2*10);
	sprintf(dst, "%+05d.%01d", i1, i2);
}
