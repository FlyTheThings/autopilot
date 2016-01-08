/*
 * dma.c
 *
 *  Created on: 13.7.2015
 *      Author: Alvar
 */
#include "dma.h"
#include "stm32f2xx_dma.c"

void DMA1Str2Init(void){
	//UART4 Rx (gps)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA1_Stream2->PAR = (uint32_t)&UART4->DR;
	DMA1_Stream2->M0AR = (uint32_t)RxBuffer;//gps-buffer
	DMA1_Stream2->NDTR = 256;//number of items to be transferred
	DMA1_Stream2->CR |= DMA_Channel_4;//stream2, ch4
	DMA1_Stream2->CR |= DMA_Priority_High;//priority high
	DMA1_Stream2->CR |= DMA_DIR_PeripheralToMemory;
	DMA1_Stream2->CR |= DMA_MemoryInc_Enable;
	DMA1_Stream2->CR |= DMA_Mode_Circular;
	DMA1_Stream2->CR |= DMA_SxCR_EN;

	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
}
void DMA2Str2Init(void){
	//USART1 Rx (data)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
	DMA2_Stream2->M0AR = (uint32_t)USART1RxBuffer;//USART-buffer
	DMA2_Stream2->NDTR = 256;//number of items to be transferred
	DMA2_Stream2->CR |= DMA_Channel_4;//stream2, ch4
	DMA2_Stream2->CR |= DMA_Priority_High;//priority high
	DMA2_Stream2->CR |= DMA_DIR_PeripheralToMemory;
	DMA2_Stream2->CR |= DMA_MemoryInc_Enable;
	DMA2_Stream2->CR |= DMA_Mode_Circular;
	DMA2_Stream2->CR |= DMA_SxCR_EN;

	/* Enable the USART Tx DMA request */
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

