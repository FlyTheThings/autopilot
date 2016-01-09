/*
 * gps.c
 *
 *  Created on: 5.6.2015
 *      Author: Alvar
 */


#include "gps.h"

unsigned char RxRd = 0;
unsigned char RxData = 0;
unsigned char RxWr = 0;
unsigned char RxBuffer[256] = {0};

char gps_time_index = 0; //että saadaan tallennettua vain 1/s
unsigned int eeprom_index = 32;
float vel_gps_temp[5] = {0};
unsigned char vel_gps_i = 0;
float vel_gps_prev = 0;

char navDataUpdate_Flag = 0;//Indicates whether the next waypoint wasnt't loaded due to busy EEPROM

struct gpsData gps;

void navDataUpdate(void)
{
	if (gps.wptDist < 20 && mission)
	{
		if (gps.wptDist < 10 || aircraft.crsErr > 90 || aircraft.crsErr < -90)
		{
			navDataUpdate_Flag = 1;
		}
	}

	gps.wptDist = 111319*sqrt(pow((currentWpt.lat-gps.lat),2) + pow(cos(gps.lat/(360/(2*pi)))*(currentWpt.lon - gps.lon),2));//toiminee pienillä etäisyyksillä

	gps.wptCrs = 180 - 180/pi*atan2(111319*cos(gps.lat/(360/(2*pi)))*(currentWpt.lon - gps.lon), 111319*(gps.lat - currentWpt.lat));
	//wpt_course 0-359 astetta
}

unsigned char data_field_seek(volatile unsigned char *buf, unsigned char start, unsigned char tgt)
{
	unsigned char i;
	unsigned char cnt = 0;
	unsigned char fld = 0;

	i = start;

	while(fld < tgt)
	{
		cnt++;

		if(buf[i++] == ',')
			fld++;
	}

	return start + cnt;   // returns the array index to the data field start position
}


unsigned int read_int(volatile unsigned char *buf, unsigned char i, unsigned char digits)
{
	unsigned char d = 0;
	unsigned int value = 0;

	while(d++ < digits)
	{
		if(buf[i] >= 0x30 && buf[i] <= 0x39)   // 0..9
		{
			value *= 10;
			value += (buf[i++] - 0x30);
		}
		else
			break;
	}

	return value;
}


unsigned int read_hex(volatile unsigned char *buf, unsigned char i, unsigned char digits)
{
	unsigned char d = 0;
	unsigned int value = 0;

	while(d++ < digits)
	{
		if(buf[i] >= 0x30 && buf[i] <= 0x39)   // 0..9
		{
			value *= 16;
			value += (buf[i++] - 0x30);
		}
		else if(buf[i] >= 0x41 && buf[i] <= 0x46)   // A..F
		{
			value *= 16;
			value += (buf[i++] - 0x41 + 10);
		}
		else if(buf[i] >= 0x61 && buf[i] <= 0x66)   // a..f
		{
			value *= 16;
			value += (buf[i++] - 0x61 + 10);
		}
		else
			break;
	}

	return value;
}


unsigned char read_header_type(volatile unsigned char *buf, unsigned char i)
{
	unsigned char xor = 0;

	while(buf[i] != ',')
	{
		xor = xor ^ buf[i++];
	}

	return xor;
}

// 	GPRMC:
// 	$GPRMC,hhmmss,[A|V],AAmm.mmmm,[N|S],OOmm.mmmm,[E|W],123.4,090.7,ddmmyy,,,[A|D|N]*76
//
// 	1: RMC header            ($GPRMC)
// 	2: UTC of position         (hhmmss)
// 	3: Status            (A: Data valid, V: Data invalid)
// 	4: Latitude            (AA: latitude in degrees, mm.mmmm: minutes of latitude)
// 	5: North/South            (N or S: North Latitude or South Latitude)
// 	6: Longitude            (OO: longitude in degrees, mm.mmmm: minutes of longitude)
// 	7: East/West            (E or W: East Longitude or West Longitude)
// 	8: Speed over ground         (123.4: receiver's speed in knots)
// 	9: Course over ground         (090.7: Receiver's direction of travel. Moving clockwise starting at due north)
// 	10: Date            (ddmmyy)
// 	11: Magnetic variation         (not output)
// 	12: East/West            (E or W, not output)
// 	13: Mode indicator         (A: Autonomous, D: D-GPS, N: Data not valid)
// 	14: Checksum            (*76)

void data_parser(unsigned char h, volatile unsigned char *buf, unsigned char start)
{
	unsigned char pos;

	float latitude_min;
	float latitude_min_frac;
	unsigned char latitude_dir = 0;

	float longitude_min;
	float longitude_min_frac;
	unsigned char longitude_dir = 0;

	unsigned char speed_knots;
	unsigned char speed_knots_frac;

	if(h == 0x6F)   // GPRMC
	{
		// Latitude
		pos = data_field_seek(buf, start, 3);

		if(buf[pos] != ',')
		{
			gps.lat		  = read_int(buf, pos + 0, 2);
			latitude_min       = read_int(buf, pos + 2, 2);
			latitude_min		 /= 60;
			latitude_min_frac  = read_int(buf, pos + 5, 4);   // 2 + 2 + decimal dot precedes
			latitude_min_frac /= 600000;

			gps.lat += latitude_min + latitude_min_frac;

			if(buf[pos + 10] == 'N' || buf[pos + 10] == 'S')
				latitude_dir = buf[pos + 10];

			if (latitude_dir == 'S')
			{
				gps.lat = -gps.lat;
			}
		}

		// Longitude
		pos = data_field_seek(buf, start, 5);

		if(buf[pos] != ',')
		{
			gps.lon		   = read_int(buf, pos + 0, 3);
			longitude_min       = read_int(buf, pos + 3, 2);
			longitude_min	  /= 60;
			longitude_min_frac  = read_int(buf, pos + 6, 4);   // 2 + 2 + decimal dot precedes
			longitude_min_frac /= 600000;

			if(buf[pos + 11] == 'E' || buf[pos + 11] == 'W')
				longitude_dir = buf[pos + 11];

			gps.lon += longitude_min + longitude_min_frac;

			if (longitude_dir == 'W')
			{
				gps.lon = -gps.lon; //west -> pituusasteet negatiivisia
			}

		}

		// Speed in knots
		pos = data_field_seek(buf, start, 7);

		if(buf[pos] != ',')
		{
			speed_knots      = read_int(buf, pos + 0, 3);
			speed_knots_frac   = read_int(buf, pos + 4, 1);   // 3 digits + decimal dot precedes

			vel_gps_temp[vel_gps_i] = 0.5144*(speed_knots + 0.1*speed_knots_frac);

			vel_gps_i = (vel_gps_i+1)%2;

			gps.speed = (vel_gps_temp[0] + vel_gps_temp[1])/2;

			gps.acc = (gps.speed - vel_gps_prev)*5;
			vel_gps_prev = gps.speed;
		}

		// Course
		pos = data_field_seek(buf, start, 8);

		if(buf[pos] != ',')
		{

			gps.crs = read_int(buf, pos + 0, 3);
			if(aircraft.crs-gps.crs > 180)
				gps.crs += 360;
			else if(gps.crs-aircraft.crs > 180)
				gps.crs -= 360;
			aircraft.crs = aircraft.crs*(1-lpfGainGpsCrs*0.01) + gps.crs*lpfGainGpsCrs*0.01;//alipäästösuodatin pehmentää kurssin muutoksia

			if(aircraft.crs >= 360)
				aircraft.crs -= 360;

			else if(aircraft.crs < 0)
				aircraft.crs += 360;

		}
	}
}

//This function works specifically with an input buffer of length 256.
void gpsHandler(void){//ei-oma
	static unsigned char start = 0;
	static unsigned char end = 0;

	if(RxBuffer[(256-DMA1_Stream2->NDTR - 1)%256] == '\n')
		end = (256-DMA1_Stream2->NDTR - 1)%256;
	else
		return;

	for(int i = 0; i<256; i++){
		if(RxBuffer[(end-i)%256] == '$'){
			start = (end-i)%256;
			unsigned char j;
			unsigned char c;
			unsigned char xor = 0;

			j = start + 1;

			while(j != (unsigned char)(end - 4))
			{
				xor = xor ^ RxBuffer[j++];
			}

			c = read_hex(RxBuffer, (unsigned char)(end - 3), 2);
			if(c == xor)   // checksum OK
			{
				unsigned char header;
				header = read_header_type(RxBuffer, start);
				data_parser(header, RxBuffer, start);
			}
			else   // checksum error
			{
			}

			navDataUpdate();
			return;
		}
	}
}





