
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



#ifndef _BASE_CLIENT_H
#define _BASE_CLIENT_H

#include <math.h>
#include <raiClient.h>
/* for definition of sonar struct and export of sonars[] */
#include <sonarClient.h>
#include <irClient.h>
#include <tactileClient.h>
#include <baseMessages.h>
#include <odoClient.h>


/* we supply a callback to fill this in when new reports come, but */
/* client can read it directly if they wish */ 
extern statusReportType activeStatusReport; 

/**********************************************************************
 *
 * Vars for functions with consistent units and coordinate systems
 *
 *********************************************************************/

extern float bOriginX;
extern float bOriginY;
extern float bOriginHeading;

extern TCX_MODULE_PTR baseServer;

#ifdef __cplusplus
extern "C" {
#endif

void registerBaseClient(void);
int  baseConnect(int blocking);
void registerBaseCallback(baseEventHandler);
void registerStatusCallback(statusHandler);
void registerWatchdogCallback(watchdogHandler);
void registerCommandConfirmationCallback(commandConfirmationHandler fcn);

/* General */

void baseKill();
void loadHeading(unsigned long heading);
void loadPosition(unsigned long);
void statusReportData(unsigned long);
void statusReportPeriod(unsigned long);
void batteryVoltage(void);
void batteryVoltage(void);
void assumeWatchdog(void); 
void watchdogTimer(unsigned long);
void findRotIndex(void);

/* Rotation Commands*/

void rotateLimp();
void rotateHalt(void);
void rotateVelocityPos(void);
void rotateVelocityNeg(void);
void rotateToPosition(unsigned long position);
void rotateRelativePos(unsigned long relative);
void rotateRelativeNeg(unsigned long relative);
void rotateTorquePos(unsigned long relative);
void rotateTorqueNeg(unsigned long relative);
void rotatePowerPos(unsigned long relative);
void rotatePowerNeg(unsigned long relative);

void setRotateVelocity(unsigned long velocity);
void setRotateAcceleration(unsigned long accel);
void setRotateTorque(unsigned long tork);

/* Rotation Queries*/
void rotateWhere(void);
void rotateCurrent(void);

/* Translation Commands*/

void translateVelocityPos(void);
void translateVelocityNeg(void);
void translateToPosition(unsigned long);
void translateLimp(void);
void translateHalt(void);
void translateRelativePos(unsigned long);
void translateRelativeNeg(unsigned long);
void translateTorquePos(unsigned long);
void translateTorqueNeg(unsigned long);
void translatePowerPos(unsigned long);
void translatePowerNeg(unsigned long);

void setTranslateVelocity(unsigned long);
void setTranslateAcceleration(unsigned long);
void setTranslateTorque(unsigned long);

void translateCurrent(void);
void translateWhere(void);

/**********************************************************************
 *
 * Functions with consistent units and coordinate systems
 *
 *********************************************************************/


float bNormalizeAngle(float radians);
float bWorldAngle(float robotAngle, float dt);
float bRobotAngle(float worldAngle, float dt);

float bSonarAngle(int sonarRow, int sonarCol);
float bIrAngle(int irRow, int irCol);
float bTactileAngle(int tactileRow, int tactileCol);

void  bSetVel(float rotVel, float transVel);
float bGetRotVel(void);
float bGetTransVel(void);

float bOdoToRobotX(float X, float Y);
float bOdoToRobotY(float X, float Y);
float bRobotToOdoX(float X, float Y);
float bRobotToOdoY(float X, float Y);

int   bSetPosition(float X, float Y, float heading);
void  bUpdatePosition(float dX, float dY, float dHeading);

void  bRegisterOdometryLockCallback(odoCallbackType fcn);
void  bRequestOdometryLock(unsigned short priority);
void  bReleaseOdometryLock();

float bRobotHeading(float dt);
float bRobotX(float dt);
float bRobotY(float dt);

/*
 * Obsolete functions
 */

int  findBaseServer(void); /* replaced by baseConnect */

#ifdef __cplusplus
}
#endif

#endif
