/*
 * AHRS.c
 *
 *  Created on: 5.6.2015
 *      Author: Alvar
 *
 * Contains functions, definitions and variables for the AHRS(/IMU) system.
 */

#include "AHRS.h"

int gx, gy, gz, ax, ay, az;

float gxf, gyf, gzf, prevCrs, tempCrs, deltaCrs;

float q0 = 1;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float exInt = 0;	// scaled integral error
float eyInt = 0;
float ezInt = 0;

//Lukee MPU9250:sta muistipaikan reg arvon.
char mpu9250SpiRead(char reg)
{
	volatile char tmp = 0;

	tmp++;
	tmp++;
	tmp++;

	GPIOC->BSRRH = GPIO_Pin_12;//PC12 alas
	SPI1->DR = reg+128; //read-> address+=128
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = reg+128;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;
	while(!(SPI1->SR & SPI_SR_RXNE));
	tmp = SPI1->DR;

	GPIOC->BSRRL = GPIO_Pin_12;//PC12 ylˆs

	return tmp;
}


//Kirjoittaa MPU9250:n rekisteriin reg arvon val
void mpu9250SpiWrite(char reg, char val)
{
	volatile char tmp = 0;

	tmp++;
	tmp++;
	tmp++;

	GPIOC->BSRRH = GPIO_Pin_12;//PC12 alas
	SPI1->DR = reg; //write-> address+=0
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = val;
	while(!(SPI1->SR & SPI_SR_TXE));
	while((SPI1->SR & SPI_SR_BSY));

	GPIOC->BSRRL = GPIO_Pin_12;//PC12 ylˆs
	tmp = SPI1->DR;
}

void MPU9250Init(void){
	mpu9250SpiWrite(107, 1);//her‰t‰ ja gyro-kello
	mpu9250SpiWrite(106, 0b00010000);//i2c disable
	mpu9250SpiWrite(25, 1);//sample rate divider(200Hz)
	mpu9250SpiWrite(26, 0b01000010); //fifo mode, dlpf_cfg=2 eli gyro bw~100Hz
	mpu9250SpiWrite(27, 0b00001000);//+-500dps, 65,5 lsb/dps
	mpu9250SpiWrite(28, 0b00001000);//+-4g, 8192lsb/g
	mpu9250SpiWrite(29, 0b00000010);//acc bw~100Hz
	mpu9250SpiWrite(35, 0b01111000);//gyro+acc fifo enable

	mpu9250SpiWrite(55, 0b00010000);//flag l‰htee mill‰ tahansa luvulla
	mpu9250SpiWrite(56, 0b00000000);//data ready interrupt disable
	//mpu9250SpiWrite(106, 0b01110000);//fifo enable

	//AK8963 init
	mpu9250SpiWrite(106, 0b00100000);//i2c_mst enable
	mpu9250SpiWrite(36, 0b00011101);//(~i2c_multimst_en), v‰liss‰ stop, 400kHz
	mpu9250SpiWrite(37, 0x0C);// rw ja i2c_slv0_addr(0X0C AK8963:lle), write
	mpu9250SpiWrite(99, 0b00010110);//slv0_do, 16-bit, ConMeasMode2(100Hz)
	mpu9250SpiWrite(38, 0x0A);//i2c_slv0_reg,

	mpu9250SpiWrite(103, 0b00000001);//i2c_slv0_dly_en
	mpu9250SpiWrite(52, 0b00000001);//100Hz slavetaajuus, div = 1+1
	mpu9250SpiWrite(37, 140);// read
	mpu9250SpiWrite(38, 3);//xl,xh,yl,yh,zl,zh,st2
	mpu9250SpiWrite(39, 0b10000111);//slv0_read_en, luetaan slv0 sample ratella, 7 rekisteri‰(viiminen st2, jolla ei merkityst‰ mutta pit‰‰ lukea

}

//This function sets the pitch and roll values to 0. There is a delay between each EEPROM write.
void calibrateAttitude(void)
{
	long axOffsetTemp = 0, ayOffsetTemp = 0, azOffsetTemp = 0, gxOffsetTemp = 0, gyOffsetTemp = 0, gzOffsetTemp = 0;
	for(char i = 0; i<10; i++)
	{
		ax = mpu9250SpiRead(59)<<8 | (unsigned char)mpu9250SpiRead(60);
		ay = mpu9250SpiRead(61)<<8 | (unsigned char)mpu9250SpiRead(62);
		az = mpu9250SpiRead(63)<<8 | (unsigned char)mpu9250SpiRead(64);
		gx = mpu9250SpiRead(67)<<8 | (unsigned char)mpu9250SpiRead(68);
		gy = mpu9250SpiRead(69)<<8 | (unsigned char)mpu9250SpiRead(70);
		gz = mpu9250SpiRead(71)<<8 | (unsigned char)mpu9250SpiRead(72);

		axOffsetTemp += ax;
		ayOffsetTemp += ay;
		azOffsetTemp += az - 8192;//-1g

		gxOffsetTemp += gx;
		gyOffsetTemp += gy;
		gzOffsetTemp += gz;

		if (i == 9)
		{
			saveParameter("axOffset", (int)(axOffsetTemp*0.1), 1);
			delay_ms(10);
			saveParameter("ayOffset", (int)(ayOffsetTemp*0.1), 1);
			delay_ms(10);
			saveParameter("azOffset", (int)(azOffsetTemp*0.1), 1);
			delay_ms(10);
			saveParameter("gxOffset", (int)(gxOffsetTemp*0.1), 1);
			delay_ms(10);
			saveParameter("gyOffset", (int)(gyOffsetTemp*0.1), 1);
			delay_ms(10);
			saveParameter("gzOffset", (int)(gzOffsetTemp*0.1), 1);
		}

		delay_ms(15);//delay
	}
}

//This function calculates the current pitch and roll as well as the inertially measured change in horizontal course.
void attitudeUpdate(void)
{
	//tulee integerej‰, jotta OR toimii
	ax = mpu9250SpiRead(59)<<8 | (unsigned char)mpu9250SpiRead(60);
	ay = mpu9250SpiRead(61)<<8 | (unsigned char)mpu9250SpiRead(62);
	az = mpu9250SpiRead(63)<<8 | (unsigned char)mpu9250SpiRead(64);
	gx = mpu9250SpiRead(67)<<8 | (unsigned char)mpu9250SpiRead(68);
	gy = mpu9250SpiRead(69)<<8 | (unsigned char)mpu9250SpiRead(70);
	gz = mpu9250SpiRead(71)<<8 | (unsigned char)mpu9250SpiRead(72);

	gx -= gxOffset;// -=  gx_offset;
	gy -= gyOffset;// -= gy_offset;
	gz -= gzOffset;// -= gz_offset;

	gxf = gx;
	gyf = gy;
	gzf = gz;

	gxf *= 0.000266462473;//rad/s
	gyf *= 0.000266462473;
	gzf *= 0.000266462473;

	//offsettien, kiihtyvyyden ja normaalikiihtyvyyksien poisto (1m/s^2 = 835lsb)
	ax = ax - axOffset - gps.speed*gzf*835;//sill‰ orientaatiolla, ett‰ y on taakse ja z ylˆs (=pwm_out takana)
	ay = ay - ayOffset + gps.acc*835;
	az = az - azOffset + gps.speed*gxf*835;

	IMUupdate(gxf, gyf, gzf, ax, ay, az);

	//lasketaan kvaternioista euklidisiksi ja muunnetaan asteiksi radiaaneista.
	tempCrs = (atan2(-2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1))*57.295779;
	aircraft.roll = (asin(2*q1*q3 - 2*q0*q2))*57.295779;
	aircraft.pitch  = (atan2(-2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 -1))*57.295779;

	deltaCrs = tempCrs - prevCrs;//muutos crs

	prevCrs = tempCrs;


	if(deltaCrs > 10)//kun esim. 0->359 -> todellisuudessa -1 vaikka n‰ytt‰‰ +359 (yli 10 tarkoittaisi ~3 kierrosta sekunnissa)
		deltaCrs -= 360;

	else if(deltaCrs < -10)//kun esim 359->0 -> todellisuudessa +1 vaikka n‰ytt‰‰ -359
		deltaCrs += 360;


	aircraft.crs += deltaCrs;

	//mod 360 floatille
	if(aircraft.crs >= 360)
		aircraft.crs -= 360;

	else if(aircraft.crs < 0)
		aircraft.crs += 360;
}


//Originally by Seb Madgwick
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)//ei-oma
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float q0_temp, q1_temp, q2_temp, q3_temp;

	// auxiliary variables to reduce number of repeated operations
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax /= norm;
	ay /= norm;
	az /= norm;
	norm = sqrt(mx*mx + my*my + mz*mz);
	//mx /= norm;
	//my /= norm;
	//mz /= norm;

	// compute reference direction of flux
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = 2*q0q0 + q3q3 - 1;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//temporary values
	q0_temp = q0;
	q1_temp = q1;
	q2_temp = q2;
	q3_temp = q3;

	// integrate quaternion rate and normalise
	q0 = q0_temp + (-q1_temp*gx - q2_temp*gy - q3_temp*gz)*halfT;
	q1 = q1_temp + (q0_temp*gx + q2_temp*gz - q3_temp*gy)*halfT;
	q2 = q2_temp + (q0_temp*gy - q1_temp*gz + q3_temp*gx)*halfT;
	q3 = q3_temp + (q0_temp*gz + q1_temp*gy - q2_temp*gx)*halfT;

	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) //ei-oma
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float q0_temp, q1_temp, q2_temp, q3_temp;

	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	//vz = 2*q0*q0 + q3*q3 - 1;

	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//temporary vaues
	q0_temp = q0;
	q1_temp = q1;
	q2_temp = q2;
	q3_temp = q3;

	// integrate quaternion rate and normalise
	q0 = q0_temp + (-q1_temp*gx - q2_temp*gy - q3_temp*gz)*halfT;
	q1 = q1_temp + (q0_temp*gx + q2_temp*gz - q3_temp*gy)*halfT;
	q2 = q2_temp + (q0_temp*gy - q1_temp*gz + q3_temp*gx)*halfT;
	q3 = q3_temp + (q0_temp*gz + q1_temp*gy - q2_temp*gx)*halfT;

	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
}





