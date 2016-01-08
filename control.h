/*
 * control.h
 *
 *  Created on: 4.1.2016
 *      Author: Alvar
 */

#ifndef CONTROL_H_
#define CONTROL_H_

float err_roll_prev;
float err_pitch_prev;
float err_crs_prev;
float err_alt_prev;
float err_fp_prev;
float err_vel_prev;

float ierr_roll;
float ierr_pitch;
float ierr_crs;
float ierr_alt;
float ierr_fp;
float ierr_vel;

float derr_roll;
float derr_pitch;
float derr_crs;
float derr_fp;
float derr_vel;

float des_roll;
float des_pitch;
float des_crs;
float des_dalt;
float des_fp;
float des_vel;

extern char climbing;
extern char descending;
extern char bankCompensation;
extern char stalling;

void updateControls(void);
void trimControls(void);

#endif /* CONTROL_H_ */
