/*
 * gps.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef GPS_H_
#define GPS_H_

#include <math.h>
#include <string.h>
#include "autopilot.h"

extern unsigned char RxRd;
extern unsigned char RxData;
extern unsigned char RxWr;
extern unsigned char RxBuffer[256];

extern char gps_time_index; //että saadaan tallennettua vain 1/s
extern unsigned int eeprom_index ;
extern float vel_gps_temp[5];
extern unsigned char vel_gps_i;
extern float vel_gps_prev;

extern char navDataUpdate_Flag;//Indicates whether the next waypoint wasnt't loaded due to busy EEPROM

struct gpsData{
	float lat;
	float lon;
	float speed;
	float crs;
	float acc;
	float wptDist;
	float wptCrs;
	unsigned char actWpt;
	unsigned char numberOfWaypoints;
	unsigned int wptPassTimer;
};
extern struct gpsData gps;

void navDataUpdate(void);
unsigned char data_field_seek(volatile unsigned char *buf, unsigned char start, unsigned char tgt);
unsigned int read_int(volatile unsigned char *buf, unsigned char i, unsigned char digits);
unsigned int read_hex(volatile unsigned char *buf, unsigned char i, unsigned char digits);
unsigned char read_header_type(volatile unsigned char *buf, unsigned char i);
void data_parser(unsigned char h, volatile unsigned char *buf, unsigned char start);
void gpsHandler(void);

#endif /* GPS_H_ */
