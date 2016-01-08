/*
 * ui.c
 *
 *  Created on: 6.6.2015
 *      Author: Alvar
 */
#include "ui.h"

char USART1RxBuffer[INPUTBUFFER_LENGTH] = { [0 ... INPUTBUFFER_LENGTH-2] = -1, [INPUTBUFFER_LENGTH-1] = '\n'};
char inputLine[INPUTBUFFER_LENGTH] = {0};
volatile char USART1NewDataAvail = 0;
char flightDataBuffer[FLIGHTDATABUFFER_LENGTH] = {0};
volatile unsigned int wptToSend = 0;
volatile unsigned int parameterToSend = 0;

//The data from the GSM module is read to a buffer by DMA first.
void USART1_NewDataHandler(){

	//If the last received character is LF or CR, then the leading characters until the
	//previous LF or CR are copied to inputLine.

	if(USART1->DR == '\n' || USART1->DR == '\r'){//if the last character is LF or CR
		int end = INPUTBUFFER_LENGTH-DMA2_Stream2->NDTR-1;
		//Scroll back to before the line feed chars
		while(USART1RxBuffer[end] == '\n' || USART1RxBuffer[end] == '\r'){
			end--;
			if(end < 0)
				end += INPUTBUFFER_LENGTH;
		}
		end++;//end is at the first line feed char

		//Scrolling back to the previous line feed chars
		int start = end-1;
		while(USART1RxBuffer[start] != '\n' && USART1RxBuffer[start] != '\r'){
			start--;
			if(start < 0)
				start += INPUTBUFFER_LENGTH;
		}
		start++;//start is the first non-line feed character
		end %= INPUTBUFFER_LENGTH;

		//Characters are copied. Termination by '\0'
		for(int i = 0; i<INPUTBUFFER_LENGTH; i++){
			int j = (i + start)%INPUTBUFFER_LENGTH;
			if(j != end){
				inputLine[i] = USART1RxBuffer[j];
			}
			else{
				inputLine[i] = '\0';
				break;
			}
		}
		USART1NewDataAvail = 1;//Setting this flag to indicate that a possible response has arrived
		//print(UART4, "input: ");//for debug
		//print(UART4, inputLine);
		//print(UART4, "\n\r");
		if(inputLine[0] == '#'){//# character indicates that this message was sent from the server.
			dataLinkInputHandler();
			connectionTimer = 0;
		}

		else if(TCP_Connected && strstr(inputLine, "CLOSED")){
			TCP_Connected = 0;
		}
		else if(TCP_Connected && strstr(inputLine, "ERROR")){
			connectionError = 10;
			TCP_Connected = 0;
		}
	}
}


//this function can be interrupted safely.
void sendData(void){
	spun;
	print(USART1, "AT+CIPSEND\r");
	connectionTimer = 0;
	//The Sim 900 will respond to AT+CIPSEND with an "> " when it is ready. There should be no other
	//incoming data at this state except for ERROR.

	while(USART1->DR != ' '){//The space character can be used for check as there shouldn't be any other responses ending with it.
		//On a delay of over 1s the process is terminated. A new try will occur in a few seconds as connectionTimer
		//reaches 500(standard delay for re-send). No problems will arise as the server won't send new data with new
		//checksums. Although, the next sent packet might have some extra characters in the start, like a part of
		//"AT+CIPSEND\r".
		if(!TCP_Connected){
			return;
		}

		if(connectionTimer > 100){
			connectionError++;
			return;
		}
	}
	delay_ms(10);
	rpun;

	print(USART1, flightDataBuffer);
	putChar(USART1, 26);//SUB character to send
	connectionTimer = 0;
	connectionError = 0;
}

void parseCommand(char* type){
	if(!strcmp(type, "reset") && aircraft.alt < 2 && aircraft.airspeed < 4 && thrOut < 1100 && manual){//reset
		NVIC_SystemReset();
	}
	else if(!strcmp(type, "reset route")){//route reset: go to first waypoint
		resetRoute();
	}
	else if(!strcmp(type, "cal imu") && manual){//calibrate imu/attitude. Can only be done on the ground because disables TIM7 interrupts
		TIM_Cmd(TIM7, DISABLE);
		calibrateAttitude();
		TIM_Cmd(TIM7, ENABLE);
	}
	else if(!strcmp(type, "cal alt") && manual){//Calibrate altimeter
		calibrateAltimeter();
	}
	else if(!strcmp(type, "cal pitot") && manual){//Calibrate airspeed sensor
		calibrateADC1();
	}
	else if(!strcmp(type, "update nav")){ //used if the current wpt is changed
		currWptNumber--;//it will be incremented in the navDataUpdate handler by 1 anyway
		navDataUpdate_Flag = 1;
	}
	else if(!strcmp(type, "trim") && manual){//trim controls. Can only be done on the ground because disables TIM7 interrupts
		TIM_Cmd(TIM7, DISABLE);
		trimControls();
		TIM_Cmd(TIM7, ENABLE);
	}
}

//This time no flight data is sent, only the response.
int parseQuery(char* type){

	//send waypoint: Q,send1w 21*c -> #R,wpt,21 +60.123456 +024.123456 120*c
	if(strstr(type, "send1w")){
		char* pos;
		if(!(pos = strchr(inputLine, ' ')+1))
			return 0;

		int number = atoi(pos);
		return sendWpt(number);
	}
	//send  parameter: Q,send1p kpRoll*c
	else if(strstr(type, "send1p")){
		char* pos;
		if(!(pos = strchr(inputLine, ' ')+1))
			return 0;

		char name[15];
		unsigned int i = 0;
		while(*pos != '*' && *pos){
			name[i++] = *(pos++);
		}
		name[i] = 0;

		return sendParameter(name, 0);
	}
	//sending all waypints
	else if(strstr(type, "sendaw")){
		if(sendWpt(0)){ //if success of sending 1st wpt send all
			wptToSend = 1;
			return 1;
		}
	}
	//sending all parameters
	else if(strstr(type, "sendap")){
		if(sendParameter((char*)(parameters[0].name), 0)){ //if success of sending 1st param send all
			parameterToSend = 1;
			return 1;
		}
	}
	return 0;
}

int sendWpt(int number){
	if(number >= numberOfWpts){
		sprintf(flightDataBuffer, "#R,wpt,err*%c", (char)TCPChecksum);
		return 0;
	}
	char wpt[40];
	unsigned int reg = eeprom_wptsStart;
	while(reg < 30000){
		eepromReadLine(wpt, &reg);
		if(atoi(wpt) == number){
			sprintf(flightDataBuffer, "#R,wpt,%s*%c", wpt, (char)TCPChecksum);
			return 1;
		}
		if(strlen(wpt) == 0){
			sprintf(flightDataBuffer, "#R,wpt,err*%c", (char)TCPChecksum);
			return 0;
		}
		reg++;
	}
	return 0;
}

//sends a parameter based on its name, OR its number if the number is not zero(used by mass send).
int sendParameter(char* name, unsigned int number){
	char value[20];
	for(unsigned int i = 0; i<numberOfParameters; i++){
		if(!strcmp(name, parameters[i].name) || (number > 0)){
			unsigned int reg;
			if(number >0){
				i = number; //the number for mass send
			}
			reg = parameters[i].eepromLocation; //by name of single send.
			sprintf(flightDataBuffer, "#R,param,%s ",parameters[i].name);
			eepromReadLine(value, &reg);
			sprintf(flightDataBuffer+strlen(flightDataBuffer), "%s*%c", value, (char)TCPChecksum);
			return 1;
		}
	}
	//if nothing is found
	sprintf(flightDataBuffer, "#R,param,err*%c", (char)TCPChecksum);
	return 0;
}

//Prints tthe current flight data to flightDataBuffer buffer.
void printFlightData(void){
	//Theoretically the maximum length for single message is 44 characters: #D,ppp,rrr,ccc,aaaaaaaaa,bbbbbbbbb,sss,hhh*n
	sprintf(flightDataBuffer,"#D,%d,%d,%d,%ld,%ld,%d,%d*%c",(int)aircraft.pitch,(int)aircraft.roll,(int)aircraft.crs,
		(long)(gps.lat*1000000), (long)(gps.lon*1000000), (int)aircraft.airspeed, (int)aircraft.alt, (char)TCPChecksum);

}


//At this moment the success of these processes must be confirmed separately.
void dataLinkInputHandler(void){
	//OK: A*c
	if(inputLine[1] == 'A'){
		TCPChecksum = inputLine[3];
		//sending possibly all parameters or waypoints
		if(wptToSend > 0){
			if(wptToSend == numberOfWpts){
				sprintf(flightDataBuffer, "#R,end of wpts*%c", (char)TCPChecksum);
				wptToSend = 0;
				sendData_Flag = 1;
				return;
			}else{
				if(sendWpt(wptToSend)){
					wptToSend++;
					sendData_Flag = 1;//not printing the flightdatabuffer again
					return;
				}
			}
		}else if(parameterToSend > 0){
			if(parameterToSend == numberOfParameters){
				sprintf(flightDataBuffer, "#R,end of params*%c", (char)TCPChecksum);
				parameterToSend = 0;
				sendData_Flag = 1;
				return;
			}else{
				if(sendParameter("", parameterToSend)){
					parameterToSend++;
					sendData_Flag = 1;
					return;
				}
			}
		}
	}
	//Parameters:P,kpRoll 12.76*c
	else if(inputLine[1] == 'P'){
		char* pos;
		if(!(pos = strchr(inputLine, ',')+1))
			return;

		char name[15];
		unsigned int i = 0;
		while(*pos != ' ' && *pos){
			name[i++] = *(pos++);
		}
		name[i] = 0;
		float val = atof(pos+1);
		saveParameter(name, val, 0);
		if(!(pos = strchr(inputLine, '*')))
			return;
		TCPChecksum = *(pos+1);
	}
	//Queries:Q,typetext*c
	else if(inputLine[1] == 'Q'){
		char* pos;
		if(!(pos = strchr(inputLine, ',')+1))
			return;

		char type[30];
		unsigned int i = 0;
		while(*pos != '*' && *pos){
			type[i++] = *(pos++);
		}
		type[i] = 0;
		TCPChecksum = *(pos+1);
		if(parseQuery(type)){ // on success
			sendData_Flag = 1;//Not sendFlightData_Flag cos flightDataBuffer is already printed.
			return;//Skipping the printFlightData function as the response is in the same buffer
		}
	}
	//Commands:C,typetext*c
	else if(inputLine[1] == 'C'){
		char* pos;
		if(!(pos = strchr(inputLine, ',')+1))
			return;

		char type[30];
		unsigned int i = 0;
		while(*pos != '*' && *pos){
			type[i++] = *(pos++);
		}
		type[i] = 0;
		parseCommand(type);
		TCPChecksum = *(pos+1);
	}
	//Waypoints: W,a/e,123 +60.123456 +024.123456 120*c
	else if(inputLine[1] == 'W'){
		char* pos;
		if(!(pos = strchr(inputLine, ',')+1))
			return;

		char type = *pos;
		if(!(pos = strchr(pos, ',')+1))
			return;

		char data[40];
		unsigned int i = 0;
		while(*pos != '*' && *pos){
			data[i++] = *(pos++);
		}
		data[i] = 0;

		//add wpt
		if(type == 'a'){
			addWpt(data);
		}
		//edit wpt
		else if(type == 'e'){
			editWpt(data);
		}

		TCPChecksum = *(pos+1);
	}
	printFlightData();
	sendData_Flag = 1;
}
