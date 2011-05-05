
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




#include <baseMessages.h>
#include <sonarClient.h>
#include <irClient.h>
#include <tactileClient.h>
#include <sensorMessages.h>


#define TCX_SERVER_MODULE_NAME "baseTCXServer"

/***** TCX messages and handlers for the base server ****/

#define SERVER_messages \
  {"baseFixed", "{int, long}"}, \
  {"baseVar", "{int,int,<char: 2>}"}



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SERVER;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SERVER;	/* otherwise: reference */

#endif




void baseSendServerFixed(int operation,unsigned long arg);
void init_server_connection();



/* Each member of the API on the client side consists of a function which */
/* just sends the corresponding message (fcn name with BASE_ prepended    */
/* and perhaps argument, to the base server.  So I wrote macros to form   */ 
/* the functions */

/* I have long suspected trained monkeys could write C :-) */  

#define HserverCmd(name) void name(void);
#define HserverArgCmd(name) void name(unsigned long arg);



/*** GENERAL ****/
HserverCmd(baseKill)
HserverArgCmd(loadHeading)
HserverArgCmd(loadPosition)
HserverArgCmd(statusReportData)
HserverArgCmd(statusReportPeriod)

HserverCmd(batteryVoltage)
HserverCmd(batteryCurrent)

/*** ROTATION ****/
HserverCmd(rotateLimp)
HserverCmd(rotateHalt)
HserverCmd(rotateVelocityPos)
HserverCmd(rotateVelocityNeg)
HserverArgCmd(rotateRelativePos)
HserverArgCmd(rotateRelativeNeg)
HserverArgCmd(rotateTorquePos)
HserverArgCmd(rotateTorqueNeg)
HserverArgCmd(rotatePowerPos)
HserverArgCmd(rotatePowerNeg)
HserverArgCmd(rotateToPosition) 
HserverCmd(findRotIndex)

HserverArgCmd(setRotateFriction)
HserverArgCmd(setRotateVelocity)
HserverArgCmd(setRotateAcceleration)
HserverArgCmd(setRotateSlope)
HserverArgCmd(setRotateTorque)
HserverArgCmd(setRotateZero)

HserverCmd(rotateCurrent)
HserverCmd(rotateWhere)


/*** TRANSLATION ****/

HserverCmd(translateLimp)
HserverCmd(translateHalt)
HserverCmd(translateVelocityPos)
HserverCmd(translateVelocityNeg)
HserverArgCmd(translateRelativePos)
HserverArgCmd(translateRelativeNeg)
HserverArgCmd(translateTorquePos)
HserverArgCmd(translateTorqueNeg)
HserverArgCmd(translatePowerPos)
HserverArgCmd(translatePowerNeg)
HserverArgCmd(translateToPosition) 

HserverArgCmd(setTranslateVelocity)
HserverArgCmd(setTranslateAcceleration)
HserverArgCmd(setTranslateSlope)
HserverArgCmd(setTranslateTorque)
HserverArgCmd(setTranslateZero)

HserverCmd(translateCurrent)
HserverCmd(translateWhere)



#ifdef define_base_message_names


char *baseMessageNamesXXX[] ={
   "BASE_subscribe",
   "BASE_translateError",
   "BASE_rotateError",
   "BASE_batteryHigh",
   "BASE_batteryLow",
   "BASE_watchdogTimeout", 
   "BASE_baseKill",
   "BASE_loadHeading",
   "BASE_loadPosition",
   "BASE_statusReportData",
   "BASE_statusReportPeriod",
   "BASE_statusReport",
   "BASE_assumeWatchdog",
   "BASE_watchdogTimer",
   "BASE_findRotIndex",
   "BASE_rotateLimp",
   "BASE_rotateHalt",
   "BASE_rotateVelocityPos",
   "BASE_rotateVelocityNeg",
   "BASE_rotateRelativePos",
   "BASE_rotateRelativeNeg",
   "BASE_rotateTorquePos",
   "BASE_rotateTorqueNeg",
   "BASE_rotatePowerPos",
   "BASE_rotatePowerNeg",
   "BASE_rotateToPosition",
   "BASE_setRotateSlope",
   "BASE_setRotateTorque",
   "BASE_setRotateZero",
   "BASE_setRotateAcceleration",
   "BASE_setRotateFriction",
   "BASE_setRotateVelocity",
   "BASE_translateLimp",
   "BASE_translateHalt",
   "BASE_translateVelocityPos",
   "BASE_translateVelocityNeg",
   "BASE_translateRelativePos",
   "BASE_translateRelativeNeg",
   "BASE_translateTorquePos",
   "BASE_translateTorqueNeg",
   "BASE_translatePowerPos",
   "BASE_translatePowerNeg",
   "BASE_translateToPosition",
   "BASE_setTranslateSlope",
   "BASE_setTranslateTorque",
   "BASE_setTranslateZero",
   "BASE_setTranslateAcceleration",
   "BASE_setTranslateVelocity",
   "BASE_batteryCurrent",
   "BASE_batteryVoltage",
   "BASE_rotateCurrent",
   "BASE_rotateWhere",
   "BASE_translateCurrent",
   "BASE_translateWhere"};


#else
  extern char *baseMessageNamesXXX[];
#endif
