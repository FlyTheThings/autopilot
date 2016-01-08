/*
 * ui.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef UI_H_
#define UI_H_

#include "autopilot.h"
#include "stm32f2xx_dma.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#define INPUTBUFFER_LENGTH 256
//important note: the send of entire flightdatabuffer must take no longer than ~5ms for a reasonable 150% safety margin
#define FLIGHTDATABUFFER_LENGTH 512 //Must fit at least all the waypoints(448 at 32 waypoints in base 64) and one flight data message(~44 bytes)


extern char USART1RxBuffer[INPUTBUFFER_LENGTH];
extern char inputLine[INPUTBUFFER_LENGTH];
extern volatile char USART1NewDataAvail;
extern char flightDataBuffer[FLIGHTDATABUFFER_LENGTH];

extern volatile unsigned int wptToSend;
extern volatile unsigned int parameterToSend;


void sendData(void);
void parseCommand(char* type);
int parseQuery(char* type);
int sendWpt(int number);
int sendParameter(char* name, unsigned int number);
void printFlightData(void);
void dataLinkInputHandler(void);
void USART1_NewDataHandler(void);



#endif /* UI_H_ */
