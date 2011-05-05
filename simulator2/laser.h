
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




#define NUMBER_OF_LASERS 2
#define NUMBER_OF_LASER_READINGS 180
#define LASER_ANGLE_RESOLUTION 1                /* unfortunately in Deg */
#define LASER_RANGE		   500		/*** centimeter ***/
#define LASER_INFINITY_RANGE       5000         /*** centimeter ***/
#define LASER_OFFSET 11.5

typedef float laserReading;

typedef struct
{
  float x;
  float y;
} position;


typedef struct {
  int numberOfReadings;
  laserReading reading[NUMBER_OF_LASER_READINGS];
  float angleResolution;
  float distanceFromCenter;
  float startAngle;
  position robotPos;
  float robotRot;
} laserRangeFinder;

#ifdef __cplusplus
extern "C" {
#endif
    
void
initLasers();


/***************************************************************************/
/* PROCEDURE :			init( LaserlaserRangeFinder *laser)       **/
/*									  **/
/* Initialize laser         						  **/
/* 									  **/
/***************************************************************************/

void 	initLaser(laserRangeFinder *laser, float startAngle);



/***************************************************************************/
/* PROCEDURE :			laserReport()      			  **/
/*									  **/
/* sends laser report via TCX 						  **/
/* 									  **/
/***************************************************************************/

void  	laserReport();

#ifdef __cplusplus
}
#endif







