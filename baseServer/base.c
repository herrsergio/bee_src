
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
 
#ifndef lint
static char rcsid[] = "$Id: base.c,v 1.17 1997/10/21 17:48:41 tyson Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>
#include <termios.h>

#include <tcx.h>
#include <rai.h>
#include <lockNames.h>
#include <baseMessages.h>
#include <bUtils.h>

#include "base.h"
#include "baseOpcodes.h"
#include "statusReport.h"

#undef DEBUG
#define WATCHDOG


/* We want to reset the watchdog timeout with a margin of safety.*/
/* If we miss, the base will go limp. So, we reset at some fraction */
/* of the timeout time */
#define WATCH_DOG_SAFETY_MARGIN 0.4

#define BASE_WATCHDOG_TIC_INTERVAL \
(BASE_WATCHDOG_INTERVAL_IN_SEC * BASE_TIME_UNITS_PER_SECOND) 

/* The baud rate at which connection to the base MCP should be made */ 
#define BASE_BAUD_RATE B9600    /* from termios.h, included in mcpIO.h */

/* battery calls are in terms of battery units, which are .1 volts */
#define BATTERY_UNITS_PER_VOLT 10


/* _HACK_ to support B14 base bumps */
int tactileReportHandler (long mspAddr, const unsigned long switches);

/* callbacks for the base's RAI module */
void baseShutDown(RaiModule*);
void baseSelect(RaiModule*);
void batteryPoll(RaiModule*);
void WatchdogPoll(RaiModule*);

/* user level call back for errors */
mcpIOState* baseMcp;
baseEventHandler userCallback= NULL;
void (*wdCallback)(void) = NULL;


int baseModuleInitialized = FALSE;
int baseModuleRunning = FALSE;

static int indexState = -1;
static int indexing = 0;

/* but who will watch the watch dog? */
int watchdogByUser = FALSE;

unsigned long status_report_format = 0;
unsigned long statusPeriod = 0;

/* we want to bag if the battery voltage gets too low  */
/* we call getBatteryVoltage and we snag the value when*/
/* the info comes back from the base */ 

unsigned long lastBatteryReading = 0;

extern int useSimulator;
extern TCX_MODULE_PTR simulatorHandle;

/***************************************************************************/


void baseSendCmd(unsigned char opCode, unsigned long parameter)
{
  char message[6];
#if 0  
  fprintf(stderr,"%s:%5d:%s() - 0x%02X:0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, opCode, parameter);
#endif
  if (useSimulator==FALSE) {
    mcpSendCmd(baseMcp,opCode,parameter);
    return;
  }
  else if (simulatorHandle) {
    message[0] = opCode;
    message[1] = (parameter>>24) & 0xFF;
    message[2] = (parameter>>16) & 0xFF;
    message[3] = (parameter>> 8) & 0xFF;
    message[4] = (parameter    ) & 0xFF;
    message[5] = ( message[0] ^ message[1] ^ message[2] ^
		   message[3] ^ message[4] );
    
    tcxSendMsg(simulatorHandle, "SIMULATOR_message_from_baseServer", message);
    
    return;
  }
  
  return;
}


/***************************************************************************
*
*  Base Module 
*
*  Handles output from base
*
***************************************************************************/

/* If the user tries to start status reports before we are ready to */
/* handle them, we defer and start them here.  This is called via   */
/* a RAI timeout when we are ready to rock */

void baseStartup()
{
  baseModuleRunning = TRUE;

  baseSendCmd(OP_INDEX_REPORT,0xFF);

  if (statusPeriod != 0)
    {
      baseSendCmd(OP_SET_STATUS_DATA,status_report_format);
      baseSendCmd(OP_SET_STATUS_PERIOD,statusPeriod);
    }
  /* set the first watch dog timeout */
#ifdef WATCHDOG
  fprintf(stderr,"BaseModule: Watchdog timer is set to %d seconds\n",
	  BASE_WATCHDOG_INTERVAL_IN_SEC);
  watchdogTimer(BASE_WATCHDOG_TIC_INTERVAL);
#else
  fprintf(stderr,"BaseModule: Not using watchdog timer! \n");
#endif
}

void BaseInit(const char * baseDevice, speed_t baudRate)
{
  RaiModule * base_module;
  RaiModule * battery_module;
  
  if (localTo(BASE_CONFIG_VARIABLE) != TRUE) {
    fprintf(stderr,"Could not verify that base is connected to this host\n");
    exit(-1);
  }
  
  if (baseModuleInitialized == FALSE) {
    baseModuleInitialized = TRUE;
    if (useSimulator==FALSE) {
      baseMcp = openMcp(baseDevice, BASE_LOCK, baudRate);
      if (baseMcp == NULL) {
	fprintf(stderr, "BaseInit: openMcp(%s) failed.\nExiting.\n",
		(const char*)baseDevice);
	exit(1);
      }
      resetMcpComm(baseMcp); /* send mcp a bunch of 0s to get it in synch */
      baseSendCmd(OP_INDEX_REPORT,0xFF);
    }
    
    initStatusStructure();
    
    RaiInit();
    base_module = makeModule("Rai base module", baseShutDown);

    if (useSimulator==FALSE) {
      addSelect(base_module, baseSelect, baseMcp->fd);
    }

    /* we want to reset the watchdog interval with a margin */
    /* of safety.  If we miss, the base will go limp         */
    
#ifdef WATCHDOG
    addPolling(base_module, WatchdogPoll,
		 BASE_WATCHDOG_TIC_INTERVAL*WATCH_DOG_SAFETY_MARGIN);
#endif

      addTimeout(base_module, baseStartup, 01);

      battery_module = makeModule("Rai battery module", NULL);
      /* check battery every 30 seconds */
      addPolling(battery_module, batteryPoll, 30000);

#ifdef DEBUG
      trace(base_module);
#endif
    }
}



void handleBaseError(unsigned char opCode,char * data) 
{
  unsigned char errorByte = data[4];

  errorAcknowledge(0); /* ack all errors for now */

  fprintf(stderr,"\a Received error. Type=");

  if (errorByte & TERR) {
    fprintf(stderr,"TERR\n");
    if (userCallback != NULL)
      userCallback(BASE_translateError,0);
  }
  else if (errorByte & TC) {
    fprintf(stderr,"TC\n");
    if (userCallback != NULL)
      userCallback(BASE_translateError,0);
  }
  else if (errorByte & RERR) {
    fprintf(stderr,"RERR\n");
    if (userCallback != NULL)
      userCallback(BASE_rotateError,0);
  }
  else if (errorByte & RC) {
    fprintf(stderr,"RC ");
    if (userCallback != NULL)
      userCallback(BASE_rotateError,0);
  }
  else if (errorByte & BHI) {
    fprintf(stderr,"BHI\n");
    if (userCallback != NULL) {
      userCallback(BASE_batteryHigh,0);
    }
  }
  else if (errorByte & BLO) {
    fprintf(stderr,"BLO\n");
    if (userCallback != NULL)
      userCallback(BASE_batteryLow,0);
  }
  else {
    fprintf(stderr, "0x%02X\n", errorByte);
  }
}



void baseDispatchValue( unsigned char opcode, unsigned long value)
{

  unsigned long message;

  if (userCallback == NULL)
    return;

  /* convert code to the same enum that */
  /* the servers use to communicate.  Right now */
  /* opcode is the RWI base opcode, which is    */
  /* subject to change */

  switch(opcode) 
    {
    case OP_GET_ROTATE_CURRENT:	message =    BASE_rotateCurrent; break;
    case OP_GET_ROTATE_WHERE:	message =    BASE_rotateWhere; break;
      
    case OP_GET_BATTERY_VOLTAGE:
      {
	message = BASE_batteryVoltage;
	value = value * 10 / BATTERY_UNITS_PER_VOLT;
	/* the base module is interested in the voltage to bag if too low*/
	lastBatteryReading = value;
	break;
      }
    case OP_GET_BATTERY_CURRENT: message =    BASE_batteryCurrent; break;
      
    case OP_GET_TRANS_CURRENT: message =    BASE_translateCurrent; break;
    case OP_GET_TRANS_WHERE:   message =    BASE_translateWhere; break;

    case OP_INDEX_REPORT: /* XXX index reports on the B14 are broken :-( */
      fprintf(stderr, "%s:%6d:%s() - INDEX REPORT = 0x%02X\n", 
	      __FILE__, __LINE__, __FUNCTION__, value);

      message = BASE_indexReport;
      value = 0xFFFFFF;

      if (indexState == -1) {
	if (value & 0x02) {
	  indexState = 1;
	}
	else {
	  indexState = 0;
	}
	return;
      }

      if (((value & 0x02) != 0) ^ (indexState != 0)) {

	/* index set event */
        if (value & 0x02) {
          indexState = 1;
	  if (!(activeStatusReport.TranslateStatus & 0x800)) {
	    value = activeStatusReport.Heading;
	    activeStatusReport.BaseRelativeHeading = value;
	    if (indexing) {
	      baseSendCmd(OP_ROTATE_REL_POS, 0);
	      indexing = 0;
	    }
	  }
        }
	/* index clear event */
        else {
          value = indexState = 0;
	  if ((activeStatusReport.TranslateStatus & 0x800)) {
	    value = activeStatusReport.Heading;
	    activeStatusReport.BaseRelativeHeading = value;
	    if (indexing) {
	      baseSendCmd(OP_ROTATE_REL_POS, 0);
	      indexing = 0;
	    }
	  }
        }
      }

      if (value == 0xFFFFFF) {
	return;
      }

      break;

   default:	
      fprintf(stderr,"%s:%5d:%s() - Unhandled opcode 0x%02X\n",
	      __FILE__, __LINE__, __FUNCTION__, opcode);
      return;
    }

  userCallback(message, value);
}

void handleReport(char* rawReport)
{
  unsigned char rotatePhase;
  static unsigned char lastRotatePhase = PHASE_STOP;  
  unsigned char translatePhase;
  static unsigned char lastTranslatePhase = PHASE_STOP;  
  statusReportType* newReport;
  static unsigned long lastRotateVelocity = 0;
  static unsigned long lastTranslateVelocity = 0;

  newReport=parseReport(rawReport);
  rotatePhase = statusPhase(newReport->RotateStatus);
  translatePhase = statusPhase(newReport->TranslateStatus);

  if (phaseMotionless(rotatePhase) && !phaseMotionless(lastRotatePhase)) {
    
#ifdef DEBUG
    fprintf(stderr,"%s:%5d:%s() - rotateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
#endif

    if(userCallback) {
      userCallback(BASE_rotateHalt, 0);
      if (indexing) {
	indexing = 0;
	userCallback(BASE_indexReport, 0);
      }
    }
  }
  else if ((!phaseMotionless(lastRotatePhase)) && 
	   (rotatePhase == PHASE_ACCEL) &&
	   (!newReport->RotateVelocity) &&
	   (!lastRotateVelocity)) {

#ifdef DEBUG
      fprintf(stderr, "%s:%6d:%s() - ROTATE HALT? \n",
	      __FILE__, __LINE__, __FUNCTION__);
#endif
      rotateHalt();
  }

  if (phaseMotionless(translatePhase) &&
      !phaseMotionless(lastTranslatePhase)) {

#ifdef DEBUG
    fprintf(stderr,"%s:%5d:%s() - translateHalt\n",
	    __FILE__, __LINE__, __FUNCTION__);
#endif
    if(userCallback) {
      userCallback(BASE_translateHalt,0);
    }
  }
  else if ((!phaseMotionless(lastTranslatePhase)) && 
	   (translatePhase == PHASE_ACCEL) &&
	   (!newReport->TranslateVelocity) &&
	   (!lastTranslateVelocity)) {
#ifdef DEBUG
      fprintf(stderr, "%s:%6d:%s() - TRANSLATE HALT? \n",
	      __FILE__, __LINE__, __FUNCTION__);
#endif
      translateHalt();
  }

  lastRotateVelocity = newReport->RotateVelocity;
  lastTranslateVelocity = newReport->TranslateVelocity;

  lastRotatePhase = rotatePhase;
  lastTranslatePhase = translatePhase;
}

void handleBaseOutput(unsigned char* base_output)
{
  unsigned char* cur_byte = base_output;
  unsigned char* return_string = 0;
  unsigned char* cur_string;
  unsigned long* hack_long_ptr;
  unsigned char op_code;
  unsigned char  output_length = 0;
  unsigned long value = 0;
  int i,j;

#ifdef DEBUG
  fprintf(stderr,"%s:%5d:%s() - \n",
	  __FILE__, __LINE__, __FUNCTION__);
#endif

  if ((*cur_byte == 2) && (*(cur_byte+1) == 2))
    {
      cur_byte += 2;
      output_length = *cur_byte;

      if ((*cur_byte == 3) || (*cur_byte == 2))
	cur_byte++;      

      cur_byte++;
      op_code = *cur_byte;

      /* why are  these checks here? - jmk */
      if ((*cur_byte == 3) || (*cur_byte == 2))
	cur_byte++;      

      output_length--;
      cur_byte++;
      
      return_string = (unsigned char *)
	calloc(output_length+2, sizeof(unsigned char));

      cur_string = return_string;
      if (output_length < 4)
	*(cur_string++) = 4;
      else
	*(cur_string++) = output_length;

      #ifdef DEBUG
      fprintf(stderr,"Handling output of length %d\n", output_length);
      #endif

      /* Note:
        We do not always want to put the length as the first byte,
	for example when we call dispatchValue below.  This should
        be rewritten to fix this, but for now I just fixed the code
        to pass only the bytes which occur after the length when needed.
      */

      hack_long_ptr = (unsigned long*) cur_string;

      for (j = 0; j < (4 - output_length); j++)
	{
	  *(cur_string) = 0;
	  cur_string++;
	}

      for (i = 0; i < output_length; i++)
	{
	  *(cur_string + i) = *cur_byte;
	  *(cur_string + i + 1) = '\0';

	  if ((*cur_byte == 3) || (*cur_byte == 2))
	    cur_byte++;
	  cur_byte++;
	}

#ifdef DEBUG
      fprintf(stderr,"%s:%5d:%s() - opcode 0x%02X\n",
	      __FILE__, __LINE__, __FUNCTION__, op_code);
#endif
      switch(op_code)
	{
	case 0x00 :
	  if (useSimulator==FALSE) {
	    /* send mcp a bunch of 0s to get it in synch */
	    resetMcpComm(baseMcp);
	  }
	  break;
	  
	case 0x01 :
	  handleBaseError(op_code,return_string);
	  break;
	  
	case 0x02 :
	  break;  /* error clear */ 
	  
	case OP_SET_STATUS_DATA:
	case OP_GET_ONE_REPORT:
	  /*	  baseSendCmd(OP_INDEX_REPORT, 0xFF); XXX */
	  handleReport(return_string);
	  break;
	  
	case OP_WATCH_DOG_TIMER:
	  fprintf(stderr,"BaseModule: Received watchdog timeout\n");
	  if (wdCallback != NULL)
	    wdCallback();
	  break;
	  
	case OP_GET_USER_MESSAGE:
	  fprintf(stderr,"User messages not supported\n");
	  break;
	  
#ifdef B14
	  
	  /*
	   * _HACK_ This is to emulate MSP base bumps on a B14
	   */
	  
	case 0x62:
	  value = htonl(*hack_long_ptr);
	  tactileReportHandler(0x80, ~value);
	  break;
#endif
	  
	default:
	  value = htonl(*hack_long_ptr);
	  baseDispatchValue(op_code,value);
	  break;
	}
      free (return_string);
    }
  else
    fprintf(stderr,"Packet with missing start sent to handleBaseOutput\n");
}





void baseShutDown(RaiModule* base_module)
{
  watchdogTimer(0);
  rotateLimp();
  translateLimp();
  BaseShutdown();
}


void baseSelect(RaiModule* base_module)
{

  char* message;
  #ifdef DEBUG
  fprintf(stderr,"Starting base select\n");
  #endif

  message = mcpSelect(baseMcp);
  if (message != NULL) {
    handleBaseOutput(message);
    free(message);
  }

  #ifdef DEBUG
  fprintf(stderr,"Done base select\n");
  #endif
}

/***************************************************************************
*
*  Status Report Calls
*
****************************************************************************/

#define REQUIRED_STATUS_DATA REPORT_EVERYTHING

void statusReportData(unsigned long data)
{
  status_report_format = data | REQUIRED_STATUS_DATA;
#if 0  /* XXX index reports on the B14 are broken :-( */
    & ~REPORT_BASE_RELATIVE_HEADING;
#endif

  if (baseModuleRunning)
    baseSendCmd(OP_SET_STATUS_DATA,status_report_format);
}

void statusReportPeriod(unsigned long period)
{
  statusPeriod= period;

  /* if the user has not specified which data yet, at least give*/
  /* them the minimum needed for rotateHalt messages and such   */

  status_report_format |= REQUIRED_STATUS_DATA;
#if 0 /* XXX index reports on the B14 are broken :-( */
    & ~REPORT_BASE_RELATIVE_HEADING;
#endif

  /* do not actually start reports coming unless this is called */
  /* after base rai module is running */
  if (baseModuleRunning)
    baseSendCmd(OP_SET_STATUS_DATA, status_report_format);
  if (baseModuleRunning)
    baseSendCmd(OP_SET_STATUS_PERIOD,period);
}

void WatchdogPoll(RaiModule* mod)
{
  /* this will cancel the impending timeout and set a new one */
  /* unless the user is insisting on doing it him/herself */
  if (!watchdogByUser)
    watchdogTimer(BASE_WATCHDOG_TIC_INTERVAL);
}

void batteryPoll(RaiModule* mod)
{
  static int warnings = 0;
  float currentVoltage=0;         /* what a merry punster I am */

  /* the first time, current voltage will be zero, but we will not
     believe this */

  currentVoltage = (float)lastBatteryReading / 10.0;
  if ((currentVoltage > 1.0) &&
      (currentVoltage < 100.0))
    {
      if (currentVoltage < bRobot.volt_warn) {
	fprintf(stderr,"\a\n");
	fprintf(stderr,"\aBase Voltage Getting Dangerously Low\n");
	fprintf(stderr,"\a\n");
	system("echo Base Voltage Getting Dangerously Low | wall");
      }
      else if (currentVoltage < bRobot.volt_panic) {
	fprintf(stderr,"\a\n");
	fprintf(stderr,"\aBase Voltage CRITICALLY Low \n");
	fprintf(stderr,"Recharge the base\n");
	warnings++;
	system("echo Base Voltage CRITICALLY Low! Recharge Now|wall");
	
	if (warnings > 3) {
	  fprintf(stderr,"Killing program due to low batteries\n");
	  fprintf(stderr,"\a\n");
	  RaiShutdown();
	  exit(-1);
	}
      }
      else {
	warnings = 0;
      }
    }
  /* ask the base for the next battery voltage */
  batteryVoltage();
}



void statusReportRequest(unsigned long report)
{
  if (report)
    {
      /* always make sure the report send what was requested */
      /* as first item */
      baseSendCmd(OP_GET_ONE_REPORT,REPORT_STATUS_DATA | report);
    }
}


void registerWatchdogCallback(watchdogHandler function_ptr)
{
  if (baseModuleInitialized == FALSE) {
    fprintf(stderr, "baseModule not initialzed\n");
    exit(0);
  }
  wdCallback = function_ptr;
}

void registerStatusCallback(statusHandler function_ptr)
{
  if (baseModuleInitialized == FALSE) {
    fprintf(stderr, "baseModule not initialzed\n");
    exit(0);
  }
  regStatusCallback((void*) function_ptr);
}


void assumeWatchdog()
{
  /* put off watchdog one last time to give user program a second to */
  /* get its shit together */
  watchdogTimer(BASE_WATCHDOG_TIC_INTERVAL);
  watchdogByUser = TRUE;
}


void BaseShutdown(void)
{
  baseSendCmd(OP_SET_STATUS_PERIOD,0);
  if (useSimulator==FALSE) {
    closeMcp(baseMcp);
  }
}


void registerBaseCallback(baseEventHandler cb)
{
  userCallback= cb;
}







/***************************************************************************
***************************************************************************/




#define makeSet(name,cmd)  void name(unsigned long arg) {baseSendCmd(cmd,arg);}
/* creates a function like this 
   void name(unsigned long arg) { baseSendCmd(cmd,arg);}

   Could replace all of the below with this.   
*/


/* Request base to send up value, and get it later during callback*/
void batteryVoltage(void) 	{ baseSendCmd(OP_GET_BATTERY_VOLTAGE,0); }
void batteryCurrent(void) 	{ baseSendCmd(OP_GET_BATTERY_CURRENT,0); }
void baseClock(void) 		{ baseSendCmd(OP_GET_MCP_CLOCK,0); 	}
void rotateCurrent(void)	{ baseSendCmd(OP_GET_ROTATE_CURRENT,0);}
void rotateWhere(void)		{ baseSendCmd(OP_GET_ROTATE_WHERE,0);}
void translateCurrent(void)	{ baseSendCmd(OP_GET_TRANS_CURRENT,0);	}
void translateWhere(void)	{ baseSendCmd(OP_GET_TRANS_WHERE,0);	}


/*------------------------------------------------------------*/

void
errorAcknowledge(unsigned long error_ack)
{
  baseSendCmd(OP_ERROR_ACKNOWLEDGE,error_ack);
}

/*------------------------------------------------------------*/

void
watchdogTimer(unsigned long interval)
{
  baseSendCmd(OP_WATCH_DOG_TIMER,interval);
}

/*------------------------------------------------------------*/

void 
findRotIndex(void)
{
#if 0 /* XXX index reports on the B14 are broken :-( */
  baseSendCmd(OP_INDEX_REPORT,0xFF);
  baseSendCmd(OP_ROTATE_VEL_POS, 0);
  indexing = 1;
#else
  indexing = 1;
  baseSendCmd(OP_FIND_ROT_INDEX, 0);
#endif
}

/*------------------------------------------------------------*/

void
baseKill()
{
  baseSendCmd(OP_MCP_KILL,0);
}

/*------------------------------------------------------------*/

void
loadHeading(unsigned long arg)
{
  baseSendCmd(OP_LOAD_HEADING,arg);
}

/*------------------------------------------------------------*/

void
loadPosition(unsigned long arg)
{
  baseSendCmd(OP_LOAD_POSITION,arg);
}


/************************************************************************
* 
*  ROTATION COMMANDS
*
************************************************************************/

void setRotateFriction(unsigned long arg){ baseSendCmd(OP_SET_ROTATE_FRICTION,arg);}
void setRotateAcceleration(unsigned long arg) {baseSendCmd(OP_SET_ROTATE_ACCEL,arg);}
void setRotateSlope(unsigned long arg)	{ baseSendCmd(OP_SET_ROTATE_SLOPE,arg);	}	
void setRotateTorque(unsigned long arg)	{ baseSendCmd(OP_SET_ROTATE_TORQUE,arg);}
void setRotateZero(unsigned long arg)	{ baseSendCmd(OP_SET_ROTATE_ZERO,arg);	}


void rotateToPosition(unsigned long arg) {baseSendCmd(OP_ROTATE_TO_POS,arg);	}
void rotateHalt(void)		{ baseSendCmd(OP_ROTATE_HALT,0);	}
void rotateLimp(void)		{ baseSendCmd(OP_ROTATE_LIMP,0);	}
void setRotateVelocity(int arg)	{ baseSendCmd(OP_SET_ROTATE_VEL,arg);}

/*************************************************************************

 ROTATE STUFF WHICH IS EFFECTED  BY BACKWARDS ROTATION OF SOME ROBOTS

 Some RWI robots rotate counterclockwise on a rotate positive command,
 some rotate clockwise.

 By convention, we will adopt CCW to be positive.  If your robot
 rotates CW when given a postivie rotate command, BASE_ROTATES_BACKWARDS
 should be defined in its .h file.

 We use this to append NEG or POS to the end of the rotate commands
 we are sending to the base.  Thus, if BASE_ROTATES_BACKWARDS is defined,
 makePos will turn OP_ROTATE_VEL_ into OP_ROTATE_VEL_NEG.  If it is
 not defined,  makePos will turn OP_ROTATE_VEL_ into OP_ROTATE_VEL_POS.

*************************************************************************/

void
rotateVelocityPos(void)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_VEL_NEG, 0);
  }
  else {
    baseSendCmd(OP_ROTATE_VEL_POS, 0);
  }
}

void
rotateVelocityNeg(void)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_VEL_POS, 0);
  }
  else {
    baseSendCmd(OP_ROTATE_VEL_NEG, 0);
  }
}

void
rotateRelativePos(unsigned long arg)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_REL_NEG, arg);
  }
  else {
    baseSendCmd(OP_ROTATE_REL_POS, arg);
  }
}

void
rotateRelativeNeg(unsigned long arg)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_REL_POS, arg);
  }
  else {
    baseSendCmd(OP_ROTATE_REL_NEG, arg);
  }
}

void
rotateTorquePos(unsigned long arg)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_TRQ_NEG, arg);
  }
  else {
    baseSendCmd(OP_ROTATE_TRQ_POS, arg);
  }
}

void
rotateTorqueNeg(unsigned long arg)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_TRQ_POS, arg);
  }
  else {
    baseSendCmd(OP_ROTATE_TRQ_NEG, arg);
  }
}

void
rotatePowerPos(unsigned long arg)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_PWR_NEG, arg);
  }
  else {
    baseSendCmd(OP_ROTATE_PWR_POS, arg);
  }
}

void
rotatePowerNeg(unsigned long arg)
{
  if (bRobot.base_rotBackwards) {
    baseSendCmd(OP_ROTATE_PWR_POS, arg);
  }
  else {
    baseSendCmd(OP_ROTATE_PWR_NEG, arg);
  }
}

/*************************************************************************
 ROTATE STUFF WHICH IS EFFECTED  BY BACKWARDS ROTATION
*************************************************************************/





/************************************************************************
* 
*  TRANSLATION COMMANDS
*
************************************************************************/

void translateHalt(void)	        {baseSendCmd(OP_TRANS_HALT,0);}
void translateLimp(void)	        {baseSendCmd(OP_TRANS_LIMP,0);	}
void translateToPosition(unsigned long arg)     {baseSendCmd(OP_TRANS_TO_POS,arg);}
void setTranslateTorque(unsigned long arg)	{baseSendCmd(OP_SET_TRANS_TORQUE,arg);}
void setTranslateZero(unsigned long arg)	{baseSendCmd(OP_SET_TRANS_ZERO,arg);}
void setTranslateSlope(unsigned long arg)	{baseSendCmd(OP_SET_TRANS_SLOPE,arg);}
void translateVelocityPos(void)	        {baseSendCmd(OP_TRANS_VEL_POS,0);}
void translateVelocityNeg(void)	        {baseSendCmd(OP_TRANS_VEL_NEG,0);}
void translateTorquePos(unsigned long arg)	{baseSendCmd(OP_TRANS_TRQ_POS,arg);}
void translateTorqueNeg(unsigned long arg)	{baseSendCmd(OP_TRANS_TRQ_NEG,arg);}
void translatePowerPos(unsigned long arg)	{baseSendCmd(OP_TRANS_PWR_POS,arg);}
void translatePowerNeg(unsigned long arg)	{baseSendCmd(OP_TRANS_PWR_NEG,arg);}


/************************************************************************

 STUFF WHICH IS EFFECTED BY BASE TRANSLATE GEARING

 Different RWI bases have different gear ratios.  So, telling one robot
 to move forward N base encoder counts will have different results on
 different robots.  To keep things sane, we define all translate commands
 in terms of MM and do the conversion to encoder counts used in the
 commands based upon bRobot.base_encPerCm, defined in bUtils.h for 
 each robot.

************************************************************************/
#define convertMMToEncoders(arg)\
((arg * bRobot.base_encPerCm) / 10)

void translateRelativePos(unsigned long arg)
{baseSendCmd(OP_TRANS_REL_POS,convertMMToEncoders(arg));}

void translateRelativeNeg(unsigned long arg) 	
{baseSendCmd(OP_TRANS_REL_NEG,convertMMToEncoders(arg));}

void setTranslateVelocity(unsigned long arg)
{
#if DEBUG
  fprintf(stderr,"%s:%5d:%s() - %d:0x%08X\n",
	  __FILE__, __LINE__, __FUNCTION__, arg,
	  (unsigned long)convertMMToEncoders(arg));
#endif
  baseSendCmd(OP_SET_TRANS_VEL,(unsigned long)convertMMToEncoders((float)arg));
}

void setTranslateAcceleration(unsigned long arg)
{baseSendCmd(OP_SET_TRANS_ACCEL,convertMMToEncoders(arg));}


/************************************************************************
 END OF TRANSLATE STUFF WHICH IS EFFECTED BY BASE TRANSLATE GEARING
************************************************************************/



/************************************************************************
************************************************************************/

void joystickDisable(unsigned long arg)
{ baseSendCmd(OP_JOYSTICK_DISABLE,arg);	}

void radioDisable(unsigned long arg)
{ baseSendCmd(OP_RADIO_DISABLE,arg);	}

void bumpSStopEnable(unsigned long arg)
{ baseSendCmd(OP_BUMPS_ENABLE,arg);	}

void errorDelay(unsigned long arg)
{ baseSendCmd(OP_ERROR_DELAY,arg);	}

void baseDelay(unsigned long arg)
{ baseSendCmd(OP_MCP_DELAY,arg);	}
