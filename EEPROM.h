/*
 * EEPROM.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "autopilot.h"

#define EEPROM_SIZE 32768 //bytes
#define EEPROM_READBUFFER_LENGTH 512
#define EEPROM_WRITEBUFFER_LENGTH 256 //Some amount
#define EEPROM_READBUFFER_LENGTH 512 //Should be enough for all reads

char eepromReadBuffer[512];
char eepromWriteBuffer[64];

extern volatile char eepromBusy;
extern volatile unsigned int eepromWriteProgress;

void eepromRead(unsigned int reg, unsigned char number);
void eepromReadLine(char* buffer, unsigned int* reg);
void eepromWriteString(char* str, unsigned int reg, char singleExeute);
void eepromWriteStart(unsigned int reg);
void eepromWrite(char val);
void eepromWriteEnd(void);

#endif /* EEPROM_H_ */
