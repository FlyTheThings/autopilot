/*
 * TIM7_IRQ.c
 *
 *  Created on: 31.7.2015
 *      Author: Alvar
 */

#include "autopilot.h"

unsigned int DMA2_Stream2_PreviousNDTR = INPUTBUFFER_LENGTH;
unsigned int DMA1_Stream2_PreviousNDTR = 256;
//unsigned int debugCnt = 0; //For debug

//The whole call lasts generally <1ms. If the time starts to approach 5ms, it is possible that writing the eeprom gets corrupted.
//It is possible that navDataUpdate() is called after EEPROM is written with some parameters. Thus it must be ensured that
//loadNextWpt() is not executed on the same cycle as the EEPROM can still be in write cycle.
//This irq is the only way to access the EEPROM.
//It must be also noted that new data can't be received if the eeprom write is busy.
void TIM7_IRQHandler(void){
	TIM7->SR = 0; //IF pois

	//Eeprom cant't always be written all at once because of page boundaries. Thus we need a timed writer.
	if(eepromWriteProgress){ //if there is ongoing eeprom write that is not stuck
		eepromWriteString(eepromWriteBuffer, eepromWriteProgress, 0);
		if(eepromBusy > 5){//The write might be stuck;
			eepromWriteProgress = 0;
			eepromBusy = 0;
		}
	}else{
		eepromBusy = 0;
	}

	attitudeUpdate();//0,35ms, 100Hz

	if(tim7Cnt%2 == 0)//0,2,4,6,8,10,12,14,16,18, 50Hz
	{
		updateControls();
	}

	if((tim7Cnt-1)%4 == 0)//1,5,9,13, 17, 25Hz
	{
		altitudeUpdate();//0,19ms, 25/5=5Hz altitude update
	}

	if((tim7Cnt-3)%4 == 0)//3,7,11,15,19, 25Hz
	{
		airspeedUpdate();//0,17ms, 25/5=5Hz airspeed update
	}

	//Incrementing routine task timer
	tim7Cnt++;
	if(tim7Cnt >= 19)
		tim7Cnt = 0;

	//Checking whether the DMA has transferred new data from USART1
	if(((DMA2_Stream2_PreviousNDTR != DMA2_Stream2->NDTR) && !eepromBusy)){ //waiting for the eeprom write to finish
		DMA2_Stream2_PreviousNDTR = DMA2_Stream2->NDTR;
		USART1_NewDataHandler();//Possibly sets EEPROM_Busy_Flag
	}
	else if((DMA2->HISR & DMA_HISR_TCIF4) && !eepromBusy){
		DMA2->HIFCR = DMA_HISR_TCIF4;
		USART1_NewDataHandler();//Possibly sets EEPROM_Busy_Flag
	}

	//Checking whether there is a new GPS message
	if((DMA1_Stream2_PreviousNDTR != DMA1_Stream2->NDTR)){
		DMA1_Stream2_PreviousNDTR = DMA1_Stream2->NDTR;
		gpsHandler();
	}
	//NOTE: the following check is not neceessary as the nmea-message with only gprmc is always shortes than 256 characters.
	else if(DMA1->HISR & DMA_HISR_TCIF4){
		DMA1->HIFCR = DMA_HISR_TCIF4;
		gpsHandler();
	}

	/* This is executed only if the EEPROM was busy on the last cycle and we came close enought to the current waypoint.
	 * There should not be possible EEPROM_Busy_Flag = navDataUpdate_Flag = 1 as is is impossible that params are updated
	 * (meaning new gprs data) at 10ms intervals. It is there for safety anyway. The EEPROM_Busy_Flag(inside navDataUpdate()
	 * is set as EEPROM is accessed. It also prevents the next of this newly loaded waypoint from being loaded.
	 */
	if(navDataUpdate_Flag && !eepromBusy){
		gps.wptPassTimer = 0;
		currWptNumber = (int)(currWptNumber+1)%(int)numberOfWpts;
		updateCurrentWpt();
		navDataUpdate_Flag = 0;
	}
	else if(saveNumberOfWpts_Flag && !eepromBusy){
		saveParameter("numberOfWpts", numberOfWpts, 0);//It has already been saved to the variable but not to eeprom.
		saveNumberOfWpts_Flag = 0;
	}
	else if(saveCurrentWptNumber_Flag && !eepromBusy){
		saveParameter("currWptNumber", currWptNumber, 0);
		saveCurrentWptNumber_Flag = 0;
	}

	//Incrementing the TCP connection timeout timer (10ms per count)
	connectionTimer++;

	//If there is no response from the server.
	if(connectionTimer > 500 && TCP_Connected){
		sendConnInit_Flag = 1;
		connectionTimer = 0;
	}
/*
	//For debug
	char data[100];
	if(TIM7->CNT > maxTime)
		maxTime = TIM7->CNT;
	sprintf(data, "P: %d, R: %d, Lat: %d, time: %lu, maxTime: %u     \r", (int)aircraft.pitch, (int)aircraft.roll, (int)(gps.lat*1000000), TIM7->CNT, maxTime);
	print(USART1, data);

	if(debugCnt++ > 1000){
		print(UART4, "irq\n\r");
		debugCnt = 0;
	}
	*/
}
