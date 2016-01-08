/*
 * airspeed.c
 *
 *  Created on: 7.6.2015
 *      Author: Alvar
 */


#include "airspeed.h"
#include "stm32f2xx_adc.c"

float cumulTempAs = 0;
char asCnt = 0;

void ADC1Init(void)
{
	svih;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitTypeDef adc1_GPIO_InitStruct;
	adc1_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	adc1_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	adc1_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &adc1_GPIO_InitStruct);

	//The following code compiles but doesn't work(it's the default settings anyway).
	/*
	ADC_CommonInitTypeDef adc1_init_common;

	adc1_init_common.ADC_Mode = ADC_Mode_Independent;
	adc1_init_common.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInit(&adc1_init_common);

	ADC_InitTypeDef adc1_initStruct;

	//Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)

	adc1_initStruct.ADC_Resolution = ADC_Resolution_12b;
	adc1_initStruct.ADC_ScanConvMode = DISABLE;
	adc1_initStruct.ADC_ContinuousConvMode = DISABLE;
	adc1_initStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc1_initStruct.ADC_DataAlign = ADC_DataAlign_Right;
	adc1_initStruct.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &adc1_initStruct);
*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
	ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
}

void calibrateADC1(void)
{
	int tempAdcU0 = 0;//Int is capable of storing 16 12-bit measurments.
	for(char a = 0; a < 16; a++)
	{
		ADC1->CR2 |= ADC_CR2_SWSTART;
		while(!(ADC1->SR & ADC_SR_EOC));
		tempAdcU0 += ADC1->DR;

	}
	tempAdcU0 >>= 4;
	saveParameter("ADC1_U0", tempAdcU0, 0);
}

//laskee uuden ilmanopeuden. Huom. p‰ivitt‰‰ aircraft.airspeed:in vain
//joka 5. suorituskerta (liukumattoman)keskiarvoistuksen takia.
void airspeedUpdate(void)
{

	int adcMeas = 0;//Int is capable of storing 16 12-bit measurments.
	float tempAs = 0;

	//Average of 4 samples
	for(char a = 0; a < 16; a++)
	{
		ADC1->CR2 |= ADC_CR2_SWSTART;
		while(!(ADC1->SR & ADC_SR_EOC));
		adcMeas += ADC1->DR;
	}
	adcMeas >>= 4;

	//The ADC measured value is compared with the calibration value. However, the adjusted value must be >0.
	if(adcMeas > ADC1_U0)
		adcMeas -= ADC1_U0;
	else
		adcMeas = 0;

	//vel *= 3.3;
	//vel /= 4096;//3.3V k‰yttˆj‰nnitteell‰ 1bit =3.3V/4096
	//vel *=1515;//tuloksena Pa, sill‰ p= 1.515*Vout-1, [p]=kPa
	tempAs = (float)adcMeas*1.22f;

	cumulTempAs  += sqrt(2*tempAs/(1.293));//palauttaa indicated airspeedin, eli miten ilma todellisuudessa vaikuttaa siipiin. Ilman tiheys l‰mpˆtilakompensoitu.

	asCnt++;

	if(asCnt >= 5){
		aircraft.airspeed = cumulTempAs/AIRSPEED_UPDATE_FREQ;
		cumulTempAs = 0;
		asCnt = 0;
	}

}

