
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








/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: ROBOT.h
 *
 * ABSTRACT:
 * 
 * Header file for basic robot data structures.
 *
 *****************************************************************************/

#ifndef ROBOT_LOADED
#define ROBOT_LOADED

#include "Common.h"
#include "rwibase_interface.h"


typedef struct {
  float d;			/* goal; cm to move */
  float deg;			/* goal; degree to turn */
  int   type;			/* error type */
  float ac;			/* real movement */
  float sonarb;		/* if BASE_OBJECT error, this is the
			 * distance where object was detected*/
  float sonarh;  int angle;	/* if HEAD_OBJECT error, these are the
				 * distance and angle where object was
				 * detected */
  float lmpos, rmpos;		/* if WHEEL_SLIP, WHEEL_STUCK, TURN_BLOCKED,
				 * or TORSO_SLIP error, these
				 * are the distances travalled by left and
				 * right base motors.
				 */
  float acd;			/* real turn done */
  float tmpos;			/* if TORSO_SLIP or TORSO_BLOCKED error, this
				 * is the current torso position */
}  GUARD_ERROR_TYPE, *GUARD_ERROR_PTR;

#define GErrorForm "{float,float,int,float,float,float,int,float,float,float,float}"


typedef struct {
  int num;				/* # of readings */
  float *angle;
  float *reading;
}  SWEEP_TYPE, *SWEEP_PTR;

#define SweepForm "{int,<float:1>,<float:1>}"


BOOLEAN start_robot();

void stop_robot(void);

#endif









