
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



/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

/****************************************************************/
/*    FILE: base.h                                              */
/****************************************************************/

#ifndef _BASE_H
#define _BASE_H

#include <termios.h>

#include <baseMessages.h>
#include <bUtils.h>

#define BASE_CONFIG_VARIABLE "BASEHOST"
#define BASE_DEVICE "/dev/RWI_base"


/* How long in seconds the base controller can go without hearing from */
/* the base module. Any longer and it will limp all motors */
#define BASE_WATCHDOG_INTERVAL_IN_SEC 2 

#ifdef  __cplusplus
extern "C" {
#endif

  void BaseInit(const char *deviceName, speed_t baudRate);
  void BaseShutdown(void);

  void leaveDirectMode(void);
  void batteryCurrent(void);
  void batteryVoltage(void);
  void baseClock(void);
  void errorAcknowledge(unsigned long err_ack);
  void errorDelay(unsigned long delay);
  void baseDelay(unsigned long delay);
  void findRotIndex(void);
  void baseKill(void);
  void loadHeading(unsigned long heading);
  void loadPosition(unsigned long heading);

  /* rotation */
  void rotateHalt(void);
  void rotateLimp(void);
  void rotateVelocityPos(void);
  void rotateVelocityNeg(void);
  void rotateToPosition(unsigned long position);
  void rotateRelativePos(unsigned long relative);
  void rotateRelativeNeg(unsigned long relative);
  void rotateTorquePos(unsigned long relative);
  void rotateTorqueNeg(unsigned long relative);
  void rotatePowerPos(unsigned long relative);
  void rotatePowerNeg(unsigned long relative);

  void setRotateVelocity(int velocity);
  void setRotateFriction(unsigned long friction);
  void setRotateSlope(unsigned long slope);
  void setRotateTorque(unsigned long tork);
  void setRotateZero(unsigned long zero);
  void setRotateAcceleration(unsigned long accel);

  void rotateCurrent(void);
  void rotateWhere(void);

  void statusReportData(unsigned long data);
  void statusReportPeriod(unsigned long period);
  void statusReportRequest(unsigned long report);
  void registerStatusCallback(statusHandler fcn);
  void registerWatchdogCallback(watchdogHandler fcn);
  void assumeWatchdog();

  void getStatusReport(void);
  void joystickDisable(unsigned long disable);
  void radioDisable(unsigned long disable);
  void bumpSStopEnable(unsigned long);  
  void watchdogTimer(unsigned long);
  void registerBaseCallback(baseEventHandler);


  void setTranslateAcceleration(unsigned long);
  void setTranslateTorque(unsigned long);
  void setTranslateZero(unsigned long);
  void setTranslateSlope(unsigned long);
  void setTranslateVelocity(unsigned long);

  void translateCurrent(void);
  void translateWhere(void);

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


#ifdef  __cplusplus
}
#endif


/****************************************************************************/
/*
     Other admisitrivia
*/
/****************************************************************************/
#define BASE_TIME_UNITS_PER_SECOND 256


/* The types of errors the base can generate */
#define TERR 0x01
#define TC   0x02
#define RERR 0x04
#define RC   0x08
#define BHI  0x10
#define BLO  0x20



/* Obviously, programs will not be running if the base is at 0 volts. */
/* So, 0 volts, or a very high voltage reading, is probably a problem with*/
/* the base (like the base computer has been turned off) so we should not */ 
/* believe it. */

#define MIN_BELIEVABLE_VOLTAGE 1
#define MAX_BELIEVABLE_VOLTAGE 100

#endif
