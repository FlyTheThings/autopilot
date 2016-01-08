/*
 * usart.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef USART_H_
#define USART_H_

#include "autopilot.h"
#include "stm32f2xx_usart.h"

extern unsigned char inputBufferCnt;

void USART1Init(void);
void UART4Init(void);
void putChar(USART_TypeDef* USARTx, char c);
void putInt(USART_TypeDef* USARTx, long int val);
void putFloat(USART_TypeDef* USARTx, float val, int decimals);
void print(USART_TypeDef* USARTx, char* src);

#endif /* USART_H_ */
