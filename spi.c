/*
 * spi.c
 *
 *  Created on: 6.6.2015
 *      Author: Alvar
 */
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include "stm32f2xx_spi.c"
/* SPI1 init function */
void SPI1Init(void)
{

    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
	GPIO_InitTypeDef spi1_GPIO_InitStruct;

	spi1_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	spi1_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	spi1_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	spi1_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	spi1_GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOB, &spi1_GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOB, 3, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, 4, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, 5, GPIO_AF_SPI1);
    GPIOA->MODER |= GPIO_MODER_MODER12_0; //PA12 = cs_eeprom
    GPIOB->MODER |= GPIO_MODER_MODER15_0; //PB15 = cs_bmp
    GPIOC->MODER |= GPIO_MODER_MODER12_0;//PC12 = cs_mpu9250
    GPIOA->BSRRL = GPIO_Pin_12;
	GPIOB->BSRRL = GPIO_Pin_15;
	GPIOC->BSRRL = GPIO_Pin_12;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//625 kBit/s jotta voi kirjoittaa mpu9250:n rekistereihin.
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}









