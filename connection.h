/*
 * connection.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef CONNECTION_H_
#define CONNECTION_H_

#include "string.h"
#include "autopilot.h"

extern volatile char connectionError;
extern volatile char TCP_Connected;
extern volatile unsigned int connectionTimer;//Used for measuring time between send packet and received packet under normal conditions.
extern volatile char sendConnInit_Flag;//Used to send init
extern volatile char sendData_Flag;//Used to send pre-set data
extern char TCPChecksum;

int initConnection(void);
int initTCP(void);
int resetConnection(void);



#endif /* CONNECTION_H_ */
