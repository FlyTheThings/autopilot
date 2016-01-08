/*
 * control.c
 *
 *  Created on: 5.6.2015
 *      Author: Alvar
 */
#include "autopilot.h"
#include "control.h"


char climbing = 0;
char descending = 0;
char bankCompensation = 0;
char stalling = 0;

void updateControls(void)
{

/*
 * MAIN TARGET: To keep the plane airborne.
 *
* Ohjauksen perusperiaatteet: 1.Pitää kone ilmassa 2. Saada kone kulkemaan määrättyä reittiä
*
* Siis: Kaiken tavoite on pitää kone semmoisessa asennossa, että se voi kulkea reittiä. Jos tulee poikkeustilanne, siitä
* poistuminen on etusijalla.
*
* Navigointi perustuu kurssivirheeseen ja korkeusvirheeseen. Kurssivirheestä saadaan haluttu roll ja
* yaw-nopeus(ja niiden kautta korkeusperäsintrimmi kaarroksiin). Korkeusvirheestä saadaan ohjeita kaasun käyttöön.
*
* Näiden lisäksi ilmanopeus on myös pidettävä hyvänä(esim. 10m/s).
*
* Sitten on kaikkea muuta pientä kivaa: laippojen säätö, eri lentotilat
* -Lentotilat täytyy kattoa: ettei ierr kasva manual-modessa.
*
* -nopeus on huomioitava ohjauksessa
* -poikkeustilanteet
*
* ardupilotissa(josta on saanut joitain vinkkejä, mitä on hyvä ottaa huomioon) mukana vielä:
* -max roll- ja pitch-ratet
*
* -yaw damper, turn coordinator, ail-rudd-mix
*
* tehtävä lisäxi:
* -nopeudensäätö, korkeudensäätö,
* -mahdollisesti: kodin suuntaan palaaminen(kodin suunta lasketaan koko ajan)
* -nopeuden ja nousun rajoittimet niin, että nopeus pysyy hyvänä, ja myöskin kohtauskulman kasvattaminen nopeasti.
*/
	int temp = 0;
	if(gps.wptPassTimer < 300)
	{
		gps.wptPassTimer++;//Timer, 50 = 1s
	}

//crs-säätö
//kun mission niin mennään waypointin(=ohjaustikku) mukaan, muutoin kurssi on aina oikea.
//Jos nopeus on pieni, ei ohjata.
	if(mission)
	{

		aircraft.crsErr = gps.wptCrs - aircraft.crs;

		//välillä -185 - 185 (=10 asteen hystereesi)
		if(aircraft.crsErr > 185)
			aircraft.crsErr -= 360;

		else if(aircraft.crsErr < -185)
			aircraft.crsErr += 360;

		derr_crs = (aircraft.crsErr - err_crs_prev)*50;
		err_crs_prev = aircraft.crsErr;

		if(gps.wptPassTimer >= 300)//jos edellinen wpt on selvästi ohitettu (6s), niin voidaan ottaa ierr_crs taas käyttöön.
		{
			//ierr_crs compensates for calibration errors
			ierr_crs += aircraft.crsErr/50;

			if(ierr_crs*kiCrs > iMaxCrs)
				ierr_crs = iMaxCrs/kiCrs;
			else if(ierr_crs*kiCrs < -iMaxCrs)
				ierr_crs = -iMaxCrs/kiCrs;
		}

		if(aircraft.airspeed >= minAsRoll)
			des_roll = kpCrs*aircraft.crsErr + kdCrs*derr_crs + kiCrs*ierr_crs;
		else
			des_roll *= 0.97f;//2 sekunnissa ~nollaan

		if(gps.wptPassTimer < 300)//6s vara, ettei mene niin paljoa yli kaarrokset.
			des_roll *= 0.8;

		if(des_roll > maxBankNav)
			des_roll = maxBankNav;
		else if(des_roll < -maxBankNav)
			des_roll = -maxBankNav;
	}

//Korkeudensäätö: Korkeusvirheen mukaan ja kallistuskulman kasvaessa kompensoidaan pitchillä. Missionilla pitää nykyisen korkeuden.
	//Halutut nousunopeudet otetaan flight pathina ~pitchiin. Jos nousuteho ei riitä tai kaarretaan, pitch otetaan nollaan.
	//Kaarroissa lisätään vähän pitchiä.
	//Tutkitaan myös uä-korkeusmittari ja pehmennetään mahdollisesti laskua.

	if(mission)//tai jos ollaan selvästi laskeutumassa, eikä maassa
	{
		aircraft.altErr = currentWpt.alt - aircraft.alt;
		//err_alt = haluttu_korkeus - alt;

		des_dalt = kpVertSpd*aircraft.altErr;

		if(des_dalt > 0)
			des_dalt *= 1-aircraft.roll/57;//Compensates for smaller lift in turns. The approximation works best at small bank angles.
		else if(des_dalt < minVertSpd)
			des_dalt = minVertSpd;

		if(aircraft.airspeed >= minAsClb)
		{
			aircraft.fp = 57*aircraft.vertSpd/aircraft.airspeed;//approksimoidaan siniä, perusta että 1 rad ~57 astetta
			des_fp = 57*des_dalt/aircraft.airspeed;
			aircraft.fpErr = lpfGainErrFp*(des_fp - aircraft.fp) + (1-lpfGainErrFp)*aircraft.fpErr;

			derr_fp = aircraft.fpErr - err_fp_prev;
			err_fp_prev = aircraft.fpErr;

			ierr_fp += aircraft.fpErr/50;

			if(ierr_fp*kiFp > iMaxFp)
				ierr_fp = iMaxFp/kiFp;
			else if(ierr_fp*kiFp < -iMaxFp)
				ierr_fp = -iMaxFp/kiFp;
		}

		if((thrOut >= ((unsigned)maxThr - 50) || aircraft.airspeed <= minAsClb) && des_fp > 0)
			des_fp = 0;

		des_pitch = lpfGainFp*(kpFp*des_fp + kiFp*ierr_fp + kdFp*derr_fp) + (1-lpfGainFp)*des_pitch; //Lpf is needed to protect the servos
	}


//Nopeudensäätö: Nopeus säädetään tikusta, ja säädetään pidillä ja kompensoidaan pitchillä.
	//Minimi- ja maximinopeudet on suojattuja missionissa.
	//Annetaan thrOutja lasketaan des_pitchiä tarpeen mukaan.
	//Pitäisi palauttaa sakkauksesta ja nopeuden kasvettua palata alkuperäiselle korkeudelle samalla kurssilla missionissa.
	//Jos korkeus on alle 7, niin sakkaussuojaa ei ole, vaan toivottu pitch on 0 ja kaasu täysille.
	//nopeuden muutoksia saattaa olla vaikea ennustaa

	if(!manual)
	{
		if(mission)
			des_vel = crzAs;

		else
		{
			temp = thrIn - thr0;
			des_vel = temp*0.025f;//0-19m/s
		}

		aircraft.airspeedErr = des_vel - aircraft.airspeed;
		derr_vel = (aircraft.airspeedErr - err_vel_prev)*50;
		err_vel_prev = aircraft.airspeedErr;

		ierr_vel += aircraft.airspeedErr/50;

		if(ierr_vel*kiAs > iMaxThr)
			ierr_vel = iMaxThr/kiAs;
		else if(ierr_vel*kiAs < minThr)
			ierr_vel = minThr/kiAs;
	}

	if(mission && aircraft.alt > 3)//Not useful to put full throttle at low altitude while stalling(risk of ground impact)
	{
		if(aircraft.airspeed < minAs || stalling)
		{
			if(aircraft.pitch >= 0)
				stalling = 1;
		}

		else if(aircraft.airspeed >= maxAs)
		{
			des_pitch += 4 dps;
		}
	}

//roll-säätö(ail): Stabilizedissa haluttu roll tulee tikusta, missionissa halutusta coursesta.
	if(des_roll > maxBankAbs)
		des_roll = maxBankAbs;
	else if(des_roll < -maxBankAbs)
		des_roll = -maxBankAbs;

	if(stabilized)
	{
		temp = -(ailIn - ail0);//etumerkki konekohtainen

		aircraft.rollErr = temp - aircraft.roll*10;
		aircraft.rollErr *= 0.1f;
	}

	else
		aircraft.rollErr = des_roll - aircraft.roll;

	derr_roll = (aircraft.rollErr - err_roll_prev)*50;//50Hz päivitystaajuus
	err_roll_prev = aircraft.rollErr;

	if(!manual && (gps.wptPassTimer >= 300))//6s ajastin, toimii myös ennen ekaa wpt
	{
		ierr_roll += aircraft.rollErr/50;

		if(ierr_roll*kiRoll > iMaxRoll)
			ierr_roll = iMaxRoll/kiRoll;
		else if(ierr_roll*kiRoll < -iMaxRoll)
			ierr_roll = -iMaxRoll/kiRoll;
	}


//pitch-säätö(ele): Stabilizedissa haluttu pitch tulee tikusta, missionissa halutuista korkeuden- ja  nopeudenmuutoksista.

	if(des_pitch > maxPitch)
		des_pitch = maxPitch;
	else if(des_pitch < minPitch)
		des_pitch = minPitch;

	if(stabilized){//stabilized{
		temp = -(eleIn - ele0);//etumerkki konekohtainen

		aircraft.pitchErr = temp - aircraft.pitch*10;

		aircraft.pitchErr *= 0.1f;

	}
	else //mission
		aircraft.pitchErr = des_pitch - aircraft.pitch;

	derr_pitch = (aircraft.pitchErr - err_pitch_prev)*50;//50Hz päivitystaajuus
	err_pitch_prev = aircraft.pitchErr;

	if(!manual)
	{
		ierr_pitch += aircraft.pitchErr/50;

		if(kiPitch*ierr_pitch > iMaxPitch)
			ierr_pitch = iMaxPitch/kiPitch;
		else if(kiPitch*ierr_pitch < -iMinPitch)
			ierr_pitch = -iMinPitch/kiPitch;
	}

// pwm out
	if(!manual)
	{
		if(gps.wptPassTimer < 300)//wpt_pass
			ailOut= ail0 + 1.2*kpRoll*aircraft.rollErr + kiRoll*ierr_roll + kdRoll*derr_roll;
		else
			ailOut= ail0 + kpRoll*aircraft.rollErr + kiRoll*ierr_roll + kdRoll*derr_roll;

		eleOut= ele0 + kpPitch*aircraft.pitchErr + kiPitch*ierr_pitch + kdPitch*derr_pitch + roll2ele_mix*aircraft.roll;

		ruddOut= rudIn; /*+ kp_yaw*aircraft.rollErr*/; //siivekkeiden käytön aiheuttaman indusoidun vastuksen aiheuttaman yawin kopensointi

		thrOut= kpAs*aircraft.airspeedErr + kiAs*ierr_vel + kdAs*derr_vel + pitch2thr_mix*aircraft.pitch;

		if(aircraft.alt < 10)//Near ground the throttle is commanded manually.
			thrOut= thrIn;

		if(stalling){
			thrOut+= 15;//+75%/s
			if(des_pitch > -15)
			des_pitch -= 20 dps;

			if(aircraft.pitch < 0 && aircraft.airspeed > minAs*1.2)
			{
				thrOut= 1500;//nopeus kasvaa jo aika kovaa
				des_pitch += 10 dps;
				stalling = 0;
			}
		}
	}

	else if(manual)
	{
		ailOut= ailIn;
		eleOut= eleIn;
		thrOut= thrIn;
		ruddOut= rudIn;

	}

//ollaanko ohitettu wpt (gps.wptPassTimer on varana)
	if(aircraft.crsErr < 3 && aircraft.crsErr > -3)
		gps.wptPassTimer = 300;

//poikkeustilanteiden hallinta tbd

//end-pointit

	if(ailOut > (unsigned)maxAilOut)
		ailOut = maxAilOut;
	else if(ailOut < (unsigned)minAilOut)
		ailOut = minAilOut;

	if(eleOut > (unsigned)maxEleOut)
		eleOut = maxEleOut;
	else if(eleOut < (unsigned)minEleOut)
		eleOut = minEleOut;

	if(ruddOut> (unsigned)maxRudOut)
		ruddOut= maxRudOut;
	else if(ruddOut< (unsigned)minRudOut)
		ruddOut= minRudOut;

	if(!manual)
	{
		if(thrOut > (unsigned)maxThr)
			thrOut= maxThr;
		else if(thrOut < (unsigned)minThr)
			thrOut= minThr;
	}

	timerUG();
}

void trimControls(void){
	saveParameter("ail0", ailIn, 1);
	delay_ms(10);
	saveParameter("ele0", eleIn, 1);
	delay_ms(10);
	saveParameter("rud0", rudIn, 1);
	delay_ms(10);
	saveParameter("thr0", thrIn, 1);
	delay_ms(10);
}


