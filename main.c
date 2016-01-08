//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "autopilot.h"


// ----------------------------------------------------------------------------
//
// STM32F2 empty sample (trace via ITM).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


int main(void)
{
	systemClockConfig();
	GPIOABCHInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//4 bitti‰ pre-emption prioritylle
	delay_ms(2000);//This avoids the USART problems caused by incoming data after short blackouts.

	ADC1Init();
	SPI1Init();
	delay_ms(500);
	BMP183Init();
	MPU9250Init();

	delay_ms(100);

	timersBaseInit();
	timersInit();
	delay_ms(100);//odotetaan ett‰ timerit stabiloituu
	SPI1->CR1 &= ~SPI_CR1_BR_2; //div2->10MHz, kun ei tarvitse en‰‰ kirjoittaa mit‰‰n

	USART1Init();
	UART4Init();
	DMA1Str2Init();
	DMA2Str2Init();

	delay_ms(3000);
	//initializeEeprom(); //Used when the eeprom data sturcture is altered (eg. parameters are added)
	delay_ms(10);
	loadParameters();
	delay_ms(100);

	TIM7NvicInit();
	delay_ms(200);


	initConnection();
	while(!initTCP());

	//Main loop that handles the not-so time critical TCP communications.
	while(1)
	{
		uint32_t a = 0;
		while(a<100000)
		{
			if(connectionError >= 5){
				while(!resetConnection()){
					//print(UART4, "main rescon\n\r");//for debug
				}
			}
			else if(!TCP_Connected){
				while(!initTCP() && connectionError < 5){
					//print(UART4, "main inittcp\n\r"); //for debug
				}
				continue;
			}
			if(sendConnInit_Flag){
				TCPChecksum = 100;
				sprintf(flightDataBuffer, "#D*%c", (char)TCPChecksum);
				sendData();
				sendConnInit_Flag = 0;
				sendData_Flag = 0;
			}
			else if(sendData_Flag){
				sendData();
				sendData_Flag = 0;
			}

			if(aircraft.roll > 45 || aircraft.roll <-45 || aircraft.pitch > 45 || aircraft.pitch < -45)
				spun;
			else
				rpun;

			//Blinker
			if(a<50000){
				svih;
			}

			else{
				rvih;
			}
			a++;
		}

	}
}

#pragma GCC diagnostic pop


// ----------------------------------------------------------------------------
