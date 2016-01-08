/*
 * altimeter.c
 *
 *  Created on: 6.6.2015
 *      Author: Alvar
 */

#include "altimeter.h"

#define ALTITUDE_UPDATE_FREQ 5
#define QNH 101300 //using the standard atmospheric pressure at msl for now

//BMP183 pressure calcultation variables
int ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned int ac4, ac5, ac6;
float x1, x2, x3, b3, b4, b5, b6, b7;

long up[4];
unsigned int ut;


//Other variables
int temperature;
float pressure;
unsigned char bmp183Cnt = 5;
unsigned char bmp183Cal = 0;

unsigned char altCnt = 0;
float tempAlt[2] = {0};
float prevAlt = 0;

//Boolean values
volatile char altitudeValid = 0;

void BMP183Init(void)
{

	ac1 = bmp183SpiRead(0xAA)<<8 | (unsigned char)bmp183SpiRead(0xAB);
	ac2 = bmp183SpiRead(0xAC)<<8 | (unsigned char)bmp183SpiRead(0xAD);
	ac3 = bmp183SpiRead(0xAE)<<8 | (unsigned char)bmp183SpiRead(0xAF);
	ac4 = (unsigned char)bmp183SpiRead(0xB0)<<8 | (unsigned char)bmp183SpiRead(0xB1);
	ac5 = (unsigned char)bmp183SpiRead(0xB2)<<8 | (unsigned char)bmp183SpiRead(0xB3);
	ac6 = (unsigned char)bmp183SpiRead(0xB4)<<8 | (unsigned char)bmp183SpiRead(0xB5);
	b1 = bmp183SpiRead(0xB6)<<8 | (unsigned char)bmp183SpiRead(0xB7);
	b2 = bmp183SpiRead(0xB8)<<8 | (unsigned char)bmp183SpiRead(0xB9);
	mb = bmp183SpiRead(0xBA)<<8 | (unsigned char)bmp183SpiRead(0xBB);
	mc = bmp183SpiRead(0xBC)<<8 | (unsigned char)bmp183SpiRead(0xBD);
	md = bmp183SpiRead(0xBE)<<8 | (unsigned char)bmp183SpiRead(0xBF);

	bmp183SpiWrite(0xF4, 0x2E);//k‰ynnistet‰‰n t-mittaus
}


/*The altitude is updated on every 5th execution.*/
void altitudeUpdate(void)
{
	if(bmp183Cnt >= 5)
	{
		ut = (unsigned char)bmp183SpiRead(0xF6)<<8 | (unsigned char)bmp183SpiRead(0xF7);
		bmp183SpiWrite(0xF4, 0xF4);//k‰ynnistet‰‰n, oss 3
		bmp183Cnt = 0;
		return;
	}

	up[(unsigned)bmp183Cnt] = (unsigned char)bmp183SpiRead(0xF6)*65536 + (unsigned char)bmp183SpiRead(0xF7)*256 + (unsigned char)bmp183SpiRead(0xF8);
	up[(unsigned)bmp183Cnt] /= 32;


	bmp183Cnt++;

	bmp183SpiWrite(0xF4, 0xF4);//k‰ynnistet‰‰n, oss 3

	if (bmp183Cnt >= 4)//bst:n algoritmi
	{
		bmp183SpiWrite(0xF4, 0x2E);//k‰ynnistet‰‰n t-mittaus

		up[3] += up[0] + up[1] + up[2];
		up[3] /= 4;

		x1 = ut - ac6;//bst:n algoritmi alkaa
		x2 = ac5;
		x2 /= 32768;
		x1 *= x2;
		x2 = mc;
		x3 = md;
		x2 = (x2*2048)/(x1 + x3);

		b5 = x1 + x2;

		temperature  = (b5 + 8)/160;

		b6 = b5-4000;

		x2 = b6/4096;
		x1 = b2;
		x1 = (x1*b6*x2)/2048;

		x2 = ac2;
		x2 = (x2*b6)/2048;
		x3 = x1 + x2;

		b3 = ac1;
		b3 = (((b3*4+x3)*8)+2)/4;

		x1 = ac3;
		x1 = (x1 * b6)/8192;

		x2 = b1;
		x2 = (x2*(b6*b6/4096))/65536;
		x3 = ((x1+x2)+2)/4;

		b4 = ac4;
		b4 = b4*(unsigned long)(x3+32768)/32768;

		b7 = ((unsigned long)up[3]-b3)*(50000/8);

		if (b7 < 0x80000000)
		{
			pressure = b7*2/b4;
		}
		else
		{
			pressure = b7/b4*2;
		}

		x1 = pressure/256;
		x1 *= x1;
		x1 = (x1 * 3038)/65536;
		x2 = (-7357*pressure)/65536;
		pressure += (x1 + x2 + 3791)/16;

		//bst:n algoritmi loppuu

		tempAlt[altCnt] = 44330*(1-pow((pressure/QNH), 0.1902632))-h0;//l‰mpˆtila vaikuttaa oikeastaan vain lento-ominaisuuksiin

		aircraft.alt = (tempAlt[0] + tempAlt[1])*0.5;

		altCnt = (altCnt+1)%2;

		aircraft.vertSpd = ALTITUDE_UPDATE_FREQ*(aircraft.alt - prevAlt);//5Hz
		prevAlt = aircraft.alt;

		altitudeValid = 1;
		bmp183Cnt++;
	}
}

//Sets the altimeter to zero
void calibrateAltimeter(void){
	saveParameter("h0", aircraft.alt+h0, 0);//compensates for if the previous h0 is not 0
}

//Lukee BMP183:sta muistipaikan reg arvon.
char bmp183SpiRead(char reg)
{
	volatile char tmp = 0;

	tmp++;//ootetaan, jotta cs todellakin ehtisi vaihtaa tilaa
	tmp++;
	tmp++;

	GPIOB->BSRRH = GPIO_Pin_15;//PB15 alas
	SPI1->DR = reg;//alkaa jo valmiix ygˆsel
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = reg;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;

	GPIOB->BSRRL = GPIO_Pin_15;//PB15 ylˆs

	return tmp;
}

//Kirjoittaa BMP183:n rekisteriin reg arvon val
void bmp183SpiWrite(char reg, char val)
{
	volatile char tmp = 0;

	tmp++;
	tmp++;
	tmp++;

	GPIOB->BSRRH = GPIO_Pin_15;//PB15 alas
	SPI1->DR = reg-128; //write-> address-=128
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = val;
	while(!(SPI1->SR & SPI_SR_TXE));
	while((SPI1->SR & SPI_SR_BSY));

	GPIOB->BSRRL = GPIO_Pin_15;//PB15 ylˆs
	tmp = SPI1->DR;
}


