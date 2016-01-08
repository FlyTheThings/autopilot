/*
 * connection.c
 *
 *  Created on: 14.7.2015
 *      Author: Alvar
 */

#include "connection.h"

volatile char connectionError = 0;
volatile char TCP_Connected = 0;
volatile unsigned int connectionTimer = 0;//Used for measuring time between send packet and received packet under normal conditions.
volatile char sendConnInit_Flag = 0;//Used to send init
volatile char sendData_Flag = 0;//Used to send pre-set data
char TCPChecksum = 100;

//TODO: yhteysajastin(jos palvelin ei lähetä ACK, pakettinumeron seuranta,...

int resetConnection(void){
	while(connectionError){
		//print(UART4, "rescon, err: ");//for debug
		//putInt(UART4, connectionError);
		//print(UART4, "\n\r");

		GPIOC->MODER |= GPIO_MODER_MODER0_0; //GPIO1 outputiksi ja nollaan
		delay_ms(100);
		GPIOC->MODER &= ~GPIO_MODER_MODER0_0; //GPIO1 takas inputiksi flouttiin
		delay_ms(5000);
		while(!initConnection());
		while(!initTCP()){
			//print(UART4, "inittcp loop: ");//for debug
			//putInt(UART4, connectionError);
			//print(UART4, "\n\r");
			if(connectionError >= 5){
				//print(UART4, "rescon broke\n\r");//for debug
				break;
			}
		}
	}
	return 1;
}

//34s approximate reconnection time after reset
int initConnection(void){
	//print(UART4, "initcon\n\r");//for debug
	while(1){
		inputLine[0] = 0;
		print(USART1, "AT\r");
		delay_ms(500);
		if(strstr(inputLine, "OK"))
			break;

	}

	inputLine[0] = 0;//Emptying inputLine
	print(USART1, "ATE0\r");
	delay_ms(500);

	//Unlock SIM-card. Returns OK
	inputLine[0] = 0;//Emptying inputLine
	for(int i = 0; i<5; i++){
		print(USART1, "AT+CPIN=\"9055\"\r");
		delay_ms(500);
		if(strstr(inputLine, "OK"))
			break;
	}

	svih;
	delay_ms(100);
	rvih;

	//Set APN. Returns only OK. NOTICE: If there is a previous TCP session active, returns ERROR.
	inputLine[0] = 0;//Emptying inputLine
	for(int i = 0; i<10; i++){
		delay_ms(1000);
		print(USART1, "AT+CSTT=\"internet.saunalahti\"\r");
		delay_ms(500);
		if(strstr(inputLine, "OK"))
			break;
	}

	spun;
	delay_ms(100);
	rpun;

	//Bring up wireless. Returns only OK
	inputLine[0] = 0;//Emptying inputLine
	for(int i = 0; i<5; i++){
		print(USART1, "AT+CIICR\r");
		delay_ms(500);
		if(strstr(inputLine, "OK"))
			break;
	}
	delay_ms(20);

	return 1;//return success

}

int initTCP(void){
	//print(UART4, "initTCP\n\r");//for debug
	spun;
	svih;
	delay_ms(100);
	rpun;
	rvih;

	//Close possible previous session
	inputLine[0] = 0;//Emptying inputLine
	print(USART1, "AT+CIPSHUT\r");
	connectionTimer = 0;
	while(1){
		if(connectionTimer > 500){
			connectionError++;
			//print(UART4, "initTCP2\n\r");//for debug
			return 0;
		}
		if(strstr(inputLine, "OK")){
			//print(UART4, "initTCP3\n\r");//for debug
			break;
		}
	}
	spun;
	delay_ms(100);
	rpun;

	//Establish TCP connection to address
	inputLine[0] = 0;//Emptying inputLine
	print(USART1, " AT+CIPSTART=\"TCP\",\"fdserver.dy.fi\",\"39999\"\r");
	connectionTimer = 0;
	while(1){
		//print(UART4, "tcpinitloop, t: ");//for debug
		//putInt(UART4, connectionTimer);
		//print(UART4, "\n\r");
		if(connectionTimer > 1000){
			connectionError++;
			//print(UART4, "initTCP4\n\r");//for debug
			return 0;
		}
		if(strstr(inputLine, "CONNECT OK"))
			break;
	}
	//print(UART4, "initTCP5\n\r");//for debug
	TCP_Connected = 1;
	connectionError = 0;
	return 1;
}


