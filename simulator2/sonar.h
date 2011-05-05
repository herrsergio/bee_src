
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.3.8 (released Aug 5, 1998)
 *****                  this is not an official BeeSoft version
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include "bUtils.h"

#ifdef __cplusplus
extern "C" {
#endif


#define NO_CHAINS		  4
#define NO_OF_SONARS		 bRobot.sonar_cols[0]
#define SONAR_ANGLE		 15		/*** degrees ***/
#ifdef B21
#define SONAR_OFFSET             7.5            
#endif
#ifdef B14
#define SONAR_OFFSET             11.5
#endif
#define RAYS_PER_SONAR		 3		
#define SONAR_RANGE		 650		/*** cm ***/
#define SONAR_INFINITY		 3610.777832		/*** cm ***/
#define SONAR_MIN_RANGE		  0.02		/*** meter ***/

#define CM_PER_SECOND 		33000.0
#define SECONDS_PER_CYCLE 	3.2552e-6
#define	SONAR_ANSWER_TIME	50            /* (int) (800/6)  in ms ***/ 



/**********************************************************************************************************/
/* PROCEDURE :			InitSonar()      			***********************************/
/*									***********************************/
/* Initialize sonar_readings						***********************************/
/* 									***********************************/
/**********************************************************************************************************/

void 	InitSonar();


/**********************************************************************************************************/
/* PROCEDURE :                  sonarReport()                           ***********************************/
/* Parameter :                  none                                    ***********************************/
/*                                                                      ***********************************/
/* Sends sonar report via TCX                                           ***********************************/
/*                                                                      ***********************************/
/**********************************************************************************************************/

void  	sonarReport();

extern int no_of_sonars;
extern float sonar_angle;
extern float sonar_range;
extern float sonar_infinity;
extern float sonar_offset;
extern float sonar_malfunc_rate;
extern int rays_per_sonar;


#ifdef __cplusplus
}
#endif
