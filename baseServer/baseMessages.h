
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



#ifndef _BASE_MESSAGES_H
#define _BASE_MESSAGES_H

#include <mcpStatus.h>


/* Message format for the confirmation of a baseServer command 
 * put in by Sebastian, July 1997 */

typedef struct  {
  unsigned long operation;
  unsigned long param;
} confirmCommandDataType;




typedef void (*baseEventHandler)(unsigned long, unsigned long); 
typedef void (*watchdogHandler)();
typedef void (*statusHandler)(statusReportType*);
typedef void (*commandConfirmationHandler)(confirmCommandDataType*);


typedef enum {

  /* USED BY BASE SERVER ONLY */
  BASE_subscribe,
  
  /* USED BY BASE MODULE & SERVER TO COMMUNICATE WITH USER */
  
  /* error conditions */
  BASE_translateError,
  BASE_rotateError,
  BASE_batteryHigh,
  BASE_batteryLow,
  BASE_watchdogTimeout,     
  
  /* commands */
  BASE_baseKill,
  BASE_loadHeading,
  BASE_loadPosition,
  BASE_statusReportData,
  BASE_statusReportPeriod,
  BASE_statusReport,
  BASE_assumeWatchdog,
  BASE_watchdogTimer,     
  
  
  BASE_findRotIndex,
  BASE_rotateLimp,
  BASE_rotateHalt,
  BASE_rotateVelocityPos,
  BASE_rotateVelocityNeg,
  BASE_rotateRelativePos,
  BASE_rotateRelativeNeg,
  BASE_rotateTorquePos,
  BASE_rotateTorqueNeg,
  BASE_rotatePowerPos,
  BASE_rotatePowerNeg,
  BASE_rotateToPosition,
  
  BASE_setRotateSlope,
  BASE_setRotateTorque,
  BASE_setRotateZero,
  BASE_setRotateAcceleration,
  BASE_setRotateFriction,
  BASE_setRotateVelocity,


  BASE_translateLimp,
  BASE_translateHalt,
  BASE_translateVelocityPos,
  BASE_translateVelocityNeg,
  BASE_translateRelativePos,
  BASE_translateRelativeNeg,
  BASE_translateTorquePos,
  BASE_translateTorqueNeg,
  BASE_translatePowerPos,
  BASE_translatePowerNeg,
  BASE_translateToPosition,
  
  BASE_setTranslateSlope,
  BASE_setTranslateTorque,
  BASE_setTranslateZero,
  BASE_setTranslateAcceleration,
  BASE_setTranslateVelocity,
  
  /* commands that return values, message is used when returning value too */
  BASE_batteryCurrent,
  BASE_batteryVoltage,
  BASE_rotateCurrent,
  BASE_rotateWhere,
  BASE_translateCurrent,
  BASE_translateWhere,
  
  BASE_sonarStart,
  BASE_sonarStop,
  
  BASE_indexReport,
  
  /* New odometry commands */
  BASE_odometryChangeX,
  BASE_odometryChangeY,
  BASE_odometryChangeH,
  
  BASE_requestOdometryLock,
  BASE_releaseOdometryLock,
  BASE_odometryLockNotify,

  /* New message to report back a base command */
  BASE_confirmCommand
  
}  BaseMessages;

#endif
