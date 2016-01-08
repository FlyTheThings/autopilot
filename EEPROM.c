/*
 * EEPROM.c
 *
 *  Created on: 30.7.2015
 *      Author: Alvar
 */


#include "EEPROM.h"


volatile char eepromBusy = 0;
volatile unsigned int eepromWriteProgress = 0;

//the reg value will be incremented to the found newline.
void eepromReadLine(char* buffer, unsigned int* reg){
	volatile char tmp = 0;
	unsigned int i = 0;

	tmp++;
	tmp++;
	tmp++;

	GPIOA->BSRRH = GPIO_Pin_12;//PA12 alas

	tmp++;
	tmp++;
	tmp++;

	SPI1->DR = 0b00000011; //read
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = *reg >> 8;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = *reg & 0xFF;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(1){
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = 0;
		while(!(SPI1->SR & SPI_SR_RXNE));
		tmp = (char)SPI1->DR;
		if(tmp == '\n' || !tmp){//newline or null terminates
			buffer[i] = 0;
			break;
		}
		buffer[i] = tmp;
		i++;
		(*reg)++;
	}

	GPIOA->BSRRL = GPIO_Pin_12;//PA12 ylös
	eepromBusy = 1;
}

//Writes a string to eeprom. Note that the max length to be written is 256 bytes.
//if singleExecute is set the string is saved within one function call
void eepromWriteString(char* str, unsigned int reg, char singleExeute){
	eepromBusy++;//counter keeps track of the write attempts. 5 should be max if buffer is 256.
	eepromWriteStart(reg);
	while(*str){
		eepromWrite(*str);
		str++;
		reg++;
		if(!(reg%64) && !singleExeute){//if page boundary
			eepromWriteEnd();
			unsigned int i = 0;
			while(*str){//copying the rest of the message to a buffer
				eepromWriteBuffer[i] = *str;
				str++;
				i++;
			}
			eepromWriteProgress = reg;
			eepromWriteBuffer[i] = 0;
			return;
		}else if(!(reg%64) && singleExeute){
			eepromWriteEnd();
			delay_ms(10);
			eepromWriteStart(reg);
		}
	}
	eepromWriteEnd();
	eepromWriteProgress = 0;
}

//Aloittaa EEPROMiin kirjoittamisen muistipaikkaan reg
void eepromWriteStart(unsigned int reg)
{
	volatile char tmp = 0;

	tmp++;
	tmp++;
	tmp++;
	GPIOA->BSRRH = GPIO_Pin_12;//PA12 alas
	SPI1->DR = 0b00000110;//wren
	while(!(SPI1->SR & SPI_SR_TXE));
	while((SPI1->SR & SPI_SR_BSY));
	GPIOA->BSRRL = GPIO_Pin_12;//PA12 ylös

	tmp++;
	tmp++;
	tmp++;

	GPIOA->BSRRH = GPIO_Pin_12;//PA12 alas
	SPI1->DR = 0b00000010;//write
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = reg>>8;
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = reg & 0xFF;
	while(!(SPI1->SR & SPI_SR_TXE));
}

//Kirjoittaa arvon val EEPROMin seuraavaan muistipaikkaan
void eepromWrite(char val)
{
	SPI1->DR = val;
	while(!(SPI1->SR & SPI_SR_TXE));
}

//Lopettaa EEPROMiin kirjoittamisen
void eepromWriteEnd(void)
{
	while((SPI1->SR & SPI_SR_BSY));

	GPIOA->BSRRL = GPIO_Pin_12;//PA12 ylös

	volatile char tmp = SPI1->DR;//rxne-flag pois
	tmp++;
}


//Maximum of 256 characters to read per execution. It must be noted that this function mustn't be interrupted.
void eepromRead(unsigned int reg, unsigned char number)
{
	volatile char tmp = 0;

	tmp++;
	tmp++;
	tmp++;

	GPIOA->BSRRH = GPIO_Pin_12;//PA12 alas

	tmp++;
	tmp++;
	tmp++;

	SPI1->DR = 0b00000011; //read
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = reg >> 8;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = reg & 0xFF;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	for(int i = 0; i<number; i++){
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = 0;
		while(!(SPI1->SR & SPI_SR_RXNE));
		eepromReadBuffer[i] = (char)SPI1->DR;
	}

	GPIOA->BSRRL = GPIO_Pin_12;//PA12 ylös
}

unsigned char eepromReadStatus(void)
{
	volatile char tmp = 0;

	tmp++;
	tmp++;
	tmp++;

	GPIOA->BSRRH = GPIO_Pin_12;//PA11 alas
	SPI1->DR = 0b00000101; //read status register
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	GPIOA->BSRRL = GPIO_Pin_12;//PA11 ylös
	eepromBusy = 1;
	return tmp;
}

