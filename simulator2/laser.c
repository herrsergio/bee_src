
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





#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <values.h>
#include <math.h>
#include <stdio.h>

#include "laser.h"
#include "trigofkt.h"
#include "robot.h"
#include "simxw.h"
#include "playground.hh"
#include "surface.hh"


#define LASER_DEBUG 1
#define TCX_define_variables  /*** this makes sure variables are installed ***/
#include "LASER-messages.h"
#include "SIMULATOR-messages.h"

/* TCX_MODULE_PTR MODULE_LASER = NULL; */

static SIMULATOR_message_to_laser_type laserMsg;

int no_of_lasers = NUMBER_OF_LASERS;
int no_of_readings = NUMBER_OF_LASER_READINGS;
float laser_resolution = LASER_ANGLE_RESOLUTION;
float laser_range = LASER_RANGE;
float laser_offset = LASER_OFFSET;
float laser_zpos = 40.0;

static laserRangeFinder frontLaser;
static laserRangeFinder rearLaser;
static firstTime = 1;

float 
normedAngle(float angle);

extern Boolean use_baseServer;

int Float2Int(float x)
{
  if (x > MAXINT)
    x = MAXINT;
  else if (x < 0.0)
    x = -1;
  return (int) x;
}  

void
initLasers()
{
    int i;
    switch(2) {                             /* we initialize both because
					       we always send both */
    case 2: initLaser(&rearLaser, 90.0);
    case 1: initLaser(&frontLaser, 270.0);
    default:
    }
    laserMsg.r_reading = (int *) malloc(NUMBER_OF_LASER_READINGS
					 * sizeof(int));
    laserMsg.f_reading = (int *) malloc(NUMBER_OF_LASER_READINGS
					* sizeof(int));    
    laserMsg.r_numberOfReadings = NUMBER_OF_LASER_READINGS;
    laserMsg.f_numberOfReadings = NUMBER_OF_LASER_READINGS;
    laserMsg.r_angleResolution = LASER_ANGLE_RESOLUTION;
    laserMsg.f_angleResolution = LASER_ANGLE_RESOLUTION;
    laserMsg.r_startAngle = 0;
    laserMsg.f_startAngle = 0;
    for(i = 0; i < NUMBER_OF_LASER_READINGS; i++) {
	laserMsg.r_reading[i] = LASER_INFINITY_RANGE;
	laserMsg.f_reading[i] = LASER_INFINITY_RANGE;
    }
}

float
Deg2Rad(float angle)
{
  return angle * M_PI / 180.0;
}


void
initLaser(laserRangeFinder *laser, float startAngle)
{

  int i;
  
  laser->numberOfReadings = no_of_readings;

  for (i = 0; i < laser->numberOfReadings; i++)
	laser->reading[i] = 0.0;

  laser->angleResolution = laser_resolution;

  laser->startAngle = startAngle;

  laser->distanceFromCenter = laser_offset;
  
  return;

} 

float
laserAngle(int beam, float robotRot, laserRangeFinder *laser)
{
  return robotRot + laser->startAngle + beam * laser->angleResolution;
}

position
laserPosition(float robotX, float robotY, float robotRot,
		   laserRangeFinder *laser)
{
  position pos;
  
  pos.x = robotX +
    laser->distanceFromCenter*
      cos(Deg2Rad(normedAngle( robotRot + laser->startAngle + 90)));
  pos.y = robotY + laser->distanceFromCenter*sin( Deg2Rad(normedAngle(robotRot
						   + laser->startAngle + 90)));
  return pos;
}





/*************************************************************************/
/* PROCEDURE :	GetDistance(LaserRangFinder *laser)      		**/
/* Parameter :			No. of cell				**/
/* 									**/
/* Pass through list of obstacles and find the closest one inside range **/
/* Check angle and surface and check whether if obstacle is hit		**/
/* 									**/
/*************************************************************************/

void
getLaserReadings(laserRangeFinder *laser)
{
  float angle, dist;
  float rx,ry,rori;
  int beam;
  position pos, endPos;

  getRobotPosition(&rx,&ry,&rori);
  pos = laserPosition(rx, ry, rori, laser);
  laser->robotPos.x = rx;
  laser->robotPos.y = ry;
  
  laser->robotRot = rori;

  for (beam = 0; beam < laser->numberOfReadings; beam++)
  {
    angle = normedAngle( laserAngle(beam, rori, laser));

    endPos.x = pos.x + laser_range * cos( Deg2Rad( angle));
    endPos.y = pos.y + laser_range * sin( Deg2Rad(angle));    
    if( get_distance(LASER_SENSOR, pos.x, pos.y, laser_zpos,
		     0.001,
		     endPos.x, endPos.y, &dist))
      laser->reading[beam] = dist;
    else
      laser->reading[beam] = laser_range;
  }
}

void DrawLaserReadings(int offset, laserRangeFinder *laser)
{
    float angle1, dist;
    float rx,ry,rori;
    int beam;
    position startPos, endPos, pos;

    if(!laserOn) return;
    getRobotPosition(&rx,&ry,&rori);
    pos = laserPosition(rx, ry, rori, laser);
    pos.x = playground_x(pos.x);
    pos.y = playground_y(pos.y);  
    
    for (beam = 0; beam < laser->numberOfReadings; beam+=3) {
	if(laser->reading[beam] >= laser_range) {
	  expose_laser_beam(offset+beam, pos.x, pos.y, pos.x, pos.y);
	  continue;
	}
	angle1 = Deg2Rad(normedAngle(laserAngle(beam, rori, laser)));
	startPos.x = playground_x(rx) + robot.RobotRadius * cos(angle1);
	startPos.y = playground_y(ry) + robot.RobotRadius * sin(angle1);
	endPos.x = pos.x+laser->reading[beam]*cos(angle1);
	endPos.y = pos.y+laser->reading[beam]*sin(angle1);
	expose_laser_beam(offset + beam,
			  startPos.x, startPos.y, endPos.x, endPos.y);
    }
}

void DrawLasers()
{
    switch(no_of_lasers) {
    case 2: DrawLaserReadings(frontLaser.numberOfReadings, &rearLaser);    
    case 1: DrawLaserReadings(0, &frontLaser);
    default:
    }
}

void
getLasers()
{
  if (firstTime){
    fprintf(stderr,"# Initializing Lasers ...");
    initLasers();
    fprintf(stderr," done\n");
    firstTime = 0;
  }
  switch(no_of_lasers) {
  case 2:
    getLaserReadings(&rearLaser);
  case 1:
    getLaserReadings(&frontLaser);
  default:
  }
}


/*************************************************************************/
/* PROCEDURE :			laserReport()      			**/
/* Parameter :			none					**/
/* 									**/
/* Sends laser report via TCX						**/
/* 									**/
/*************************************************************************/

void
laserReport()
{
  int i=0;
  int time_lost;

  
  getLasers();

#ifdef LASERDEBUG
  fprintf(stderr, "SIMULATOR :   SENDING  LASER Report\n");
  fprintf(stderr, "# Copying laser values in message format ...");
#endif
  switch(no_of_lasers) {
  case 2:
      laserMsg.r_numberOfReadings = rearLaser.numberOfReadings;
      for (i=0; i<rearLaser.numberOfReadings; i++){
	  laserMsg.r_reading[i] =
	      Float2Int( rearLaser.reading[i]);
      }
      laserMsg.r_startAngle = Deg2Rad(normedAngle(rearLaser.startAngle
						  + 180.0));
      laserMsg.r_angleResolution = Deg2Rad(rearLaser.angleResolution);
  case 1:
      laserMsg.f_numberOfReadings = frontLaser.numberOfReadings;
      for (i=0; i<frontLaser.numberOfReadings; i++){
	  laserMsg.f_reading[i] =
	      Float2Int( frontLaser.reading[i]);
      }
      laserMsg.f_startAngle = Deg2Rad( normedAngle(frontLaser.startAngle));
      laserMsg.f_angleResolution = Deg2Rad( frontLaser.angleResolution);
  default:
  }
#ifdef LASERDEBUG
  fprintf(stderr, " done\n");
  fprintf(stderr, "# Sending message ...");
#endif
  if (use_baseServer == FALSE) {
    tcxSendMsg( MODULE_BASE, "SIMULATOR_message_to_laser", &laserMsg);
  }
#ifdef LASERDEBUG
  fprintf(stderr, " done\n");
#endif
/*
  free(laserMsg.r_reading);
  free(laserMsg.f_reading);
*/
  return;

} /* laserReport() */

