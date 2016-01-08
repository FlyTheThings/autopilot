/*
 * usart.c
 *
 *  Created on: 6.6.2015
 *      Author: Alvar
 */

#include "usart.h"

#include "stm32f2xx_usart.c"

unsigned char inputBufferCnt = 0;


void USART1Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitTypeDef usart_GPIO_InitStruct;

	/**usart GPIO Configuration
	PA9     ------> usart_TX
	PA10     ------> usart_RX
	*/
	usart_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	usart_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	usart_GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	usart_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	usart_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &usart_GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, 9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, 10, GPIO_AF_USART1);

	/*Peripheral init*/

	USART_InitTypeDef USART_InitStruct;

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);
}

void UART4Init(void){
	GPIO_InitTypeDef usart_GPIO_InitStruct;

	/**UART4 GPIO Configuration
	PC10     ------> usart_TX
	PC11     ------> usart_RX
	*/
	usart_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	usart_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	usart_GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	usart_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	usart_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &usart_GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOC, 10, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC, 11, GPIO_AF_UART4);

	/*Peripheral init*/

	USART_InitTypeDef USART_InitStruct;

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(UART4, &USART_InitStruct);
	USART_Cmd(UART4, ENABLE);
}

//Puts a char to USART1
void putChar(USART_TypeDef* USARTx, char c)
{
	while(!(USARTx->SR & USART_SR_TXE));
	USARTx->DR = c;
}

void putInt(USART_TypeDef* USARTx, long int val) //laittaa luvut -999999 - 999999
{
	if (val < 0){
		putChar(USARTx, '-');
		val = -val;
	}

	putChar(USARTx, val/100000 + 48);
	putChar(USARTx, val/10000 - val/100000*10 +  48);
	putChar(USARTx, val/1000 - val/10000*10 + 48);
	putChar(USARTx, val/100 - val/1000*10 + 48);
	putChar(USARTx, val/10 - val/100*10 + 48);
	putChar(USARTx, val - val/10*10 + 48);

}

void putFloat(USART_TypeDef* USARTx, float val, int decimals){
	char a[20] = {0};
	int i1 = val;  				// Get the integer part
	sprintf(a+strlen(a), "%d.", i1);
	float f2 = val - i1; 							// Get fractional part
	for(int i = 0; i < decimals; i++){
		f2 = f2*10;
		if(f2<1)
			strcat(a, "0");
	}
	int i2 = f2;
	sprintf(a+strlen(a), "%d", i2);
	print(USARTx, a);
}

//Prints a string to USART1
void print(USART_TypeDef* USARTx, char* src)
{
	while (*src){
		putChar(USARTx, *src);
		src++;
	}
}
